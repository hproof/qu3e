//--------------------------------------------------------------------------------------------------
/**
@file	q3Scene.h

@author	Randy Gaul
@date	10/10/2014

	Copyright (c) 2014 Randy Gaul http://www.randygaul.net

	This software is provided 'as-is', without any express or implied
	warranty. In no event will the authors be held liable for any damages
	arising from the use of this software.

	Permission is granted to anyone to use this software for any purpose,
	including commercial applications, and to alter it and redistribute it
	freely, subject to the following restrictions:
	  1. The origin of this software must not be misrepresented; you must not
	     claim that you wrote the original software. If you use this software
	     in a product, an acknowledgment in the product documentation would be
	     appreciated but is not required.
	  2. Altered source versions must be plainly marked as such, and must not
	     be misrepresented as being the original software.
	  3. This notice may not be removed or altered from any source distribution.
*/
//--------------------------------------------------------------------------------------------------

#include <stdlib.h>

#include "q3Scene.h"
#include "../dynamics/q3Body.h"
#include "../dynamics/q3Island.h"
#include "../dynamics/q3Contact.h"
#include "../dynamics/q3Island.h"
#include "../dynamics/q3ContactSolver.h"
#include "../collision/q3Box.h"

//--------------------------------------------------------------------------------------------------
// q3Scene
//--------------------------------------------------------------------------------------------------
// 构造函数：初始化各成员。
// dt/gravity/iterations 直接保存；sleep、friction 默认开启。
q3Scene::q3Scene( r32 dt, const q3Vec3& gravity, i32 iterations )
	: m_contactManager( &m_stack )            // 接触管理器使用栈分配器（临时数据）
	, m_boxAllocator( sizeof( q3Box ), 256 )  // q3Box 页式分配器，每页 256 个
	, m_bodyCount( 0 )
	, m_bodyList( NULL )
	, m_gravity( gravity )
	, m_dt( dt )
	, m_iterations( iterations )
	, m_newBox( false )
	, m_allowSleep( true )
	, m_enableFriction( true )
{
}

//--------------------------------------------------------------------------------------------------
q3Scene::~q3Scene( )
{
	Shutdown( );
}

//--------------------------------------------------------------------------------------------------
//==============================================================================
// Step()：把物理世界向前推进一个固定时间步长 dt。完整流程：
//   1. 宽相位更新 + 碰撞检测（TestCollisions）
//   2. 构建「岛屿」（island，由接触连接起来的刚体连通分量）
//   3. 对每个岛屿独立求解（约束求解 + 积分）
//   4. 更新宽相位 AABB、查找新接触、清空力
//==============================================================================
void q3Scene::Step( )
{
	// 若有新加入的盒体，先更新宽相位里的碰撞对
	if ( m_newBox )
	{
		m_contactManager.m_broadphase.UpdatePairs( );
		m_newBox = false;
	}

	// 碰撞检测：用宽相位找到的潜在对，做精确 SAT 检测并生成接触约束
	m_contactManager.TestCollisions( );

	// 清除所有刚体的「已归属岛屿」标志，准备重新建岛
	for ( q3Body* body = m_bodyList; body; body = body->m_next )
		body->m_flags &= ~q3Body::eIsland;

	// 在栈上为岛屿预留「最坏情况」的内存（所有 body 都是同一岛屿时的大小）。
	// 栈分配极快，适合这种每帧临时、用完即弃的分配。
	// Size the stack island, pick worst case size
	m_stack.Reserve(
		sizeof( q3Body* ) * m_bodyCount
		+ sizeof( q3VelocityState ) * m_bodyCount
		+ sizeof( q3ContactConstraint* ) * m_contactManager.m_contactCount
		+ sizeof( q3ContactConstraintState ) * m_contactManager.m_contactCount
		+ sizeof( q3Body* ) * m_bodyCount
	);

	// 初始化岛屿求解器的工作缓冲区
	q3Island island;
	island.m_bodyCapacity = m_bodyCount;
	island.m_contactCapacity = m_contactManager.m_contactCount;
	island.m_bodies = (q3Body**)m_stack.Allocate( sizeof( q3Body* ) * m_bodyCount );
	island.m_velocities = (q3VelocityState *)m_stack.Allocate( sizeof( q3VelocityState ) * m_bodyCount );
	island.m_contacts = (q3ContactConstraint **)m_stack.Allocate( sizeof( q3ContactConstraint* ) * island.m_contactCapacity );
	island.m_contactStates = (q3ContactConstraintState *)m_stack.Allocate( sizeof( q3ContactConstraintState ) * island.m_contactCapacity );
	island.m_allowSleep = m_allowSleep;
	island.m_enableFriction = m_enableFriction;
	island.m_bodyCount = 0;
	island.m_contactCount = 0;
	island.m_dt = m_dt;
	island.m_gravity = m_gravity;
	island.m_iterations = m_iterations;

	// ---- 构建并求解每个岛屿 ----
	// 岛屿 = 由接触（约束）连接起来的刚体连通分量。彼此不接触的刚体分属不同岛屿，
	// 可以独立求解（互相不影响），这也是并行化的基础。
	// Build each active island and then solve each built island
	i32 stackSize = m_bodyCount;
	q3Body** stack = (q3Body**)m_stack.Allocate( sizeof( q3Body* ) * stackSize );
	for ( q3Body* seed = m_bodyList; seed; seed = seed->m_next )
	{
		// 已归属某岛屿的种子跳过
		// Seed cannot be apart of an island already
		if ( seed->m_flags & q3Body::eIsland )
			continue;

		// 休眠中的刚体跳过（不参与求解）
		// Seed must be awake
		if ( !(seed->m_flags & q3Body::eAwake) )
			continue;

		// 静态刚体不作为种子（避免岛屿过大）；它会在被动态刚体接触时纳入岛屿
		// Seed cannot be a static body in order to keep islands
		// as small as possible
		if ( seed->m_flags & q3Body::eStatic )
			continue;

		i32 stackCount = 0;
		stack[ stackCount++ ] = seed;
		island.m_bodyCount = 0;
		island.m_contactCount = 0;

		// 标记种子已归属岛屿
		// Mark seed as apart of island
		seed->m_flags |= q3Body::eIsland;

		// 用 DFS（深度优先搜索）遍历约束图，收集所有连通的身体和接触
		// Perform DFS on constraint graph
		while( stackCount > 0 )
		{
			// 弹栈，实现迭代式回溯
			// Decrement stack to implement iterative backtracking
			q3Body *body = stack[ --stackCount ];
			island.Add( body );

			// 唤醒与岛屿相连的所有刚体（接触唤醒传播）
			// Awaken all bodies connected to the island
			body->SetToAwake( );

			// 不跨过静态刚体继续搜索（保持岛屿小），但静态刚体本身要留在岛屿里，
			// 以便正确表示完整接触（例如地面和箱子之间）
			// Do not search across static bodies to keep island
			// formations as small as possible, however the static
			// body itself should be apart of the island in order
			// to properly represent a full contact
			if ( body->m_flags & q3Body::eStatic )
				continue;

			// 遍历该刚体的所有接触，把相连的刚体加入岛屿
			// Search all contacts connected to this body
			q3ContactEdge* contacts = body->m_contactList;
			for ( q3ContactEdge* edge = contacts; edge; edge = edge->next )
			{
				q3ContactConstraint *contact = edge->constraint;

				// 已加入岛屿的接触跳过
				// Skip contacts that have been added to an island already
				if ( contact->m_flags & q3ContactConstraint::eIsland )
					continue;

				// 没有真正碰撞的接触跳过
				// Can safely skip this contact if it didn't actually collide with anything
				if ( !(contact->m_flags & q3ContactConstraint::eColliding) )
					continue;

				// 传感器跳过（只检测不产生物理响应，不参与求解）
				// Skip sensors
				if ( contact->A->sensor || contact->B->sensor )
					continue;

				// 标记接触并加入岛屿
				// Mark island flag and add to island
				contact->m_flags |= q3ContactConstraint::eIsland;
				island.Add( contact );

				// 把接触另一端的刚体也压栈，实现唤醒传播
				// Attempt to add the other body in the contact to the island
				// to simulate contact awakening propogation
				q3Body* other = edge->other;
				if ( other->m_flags & q3Body::eIsland )
					continue;

				assert( stackCount < stackSize );

				stack[ stackCount++ ] = other;
				other->m_flags |= q3Body::eIsland;
			}
		}

		assert( island.m_bodyCount != 0 );

		// 初始化岛屿（预热速度等），然后求解（迭代约束求解 + 积分）
		island.Initialize( );
		island.Solve( );

		// 清除岛屿中静态刚体的 island 标志，让它们能参与其他岛屿
		// Reset all static island flags
		// This allows static bodies to participate in other island formations
		for ( i32 i = 0; i < island.m_bodyCount; i++ )
		{
			q3Body *body = island.m_bodies[ i ];

			if ( body->m_flags & q3Body::eStatic )
				body->m_flags &= ~q3Body::eIsland;
		}
	}

	// 释放栈上的岛屿缓冲区
	m_stack.Free( stack );
	m_stack.Free( island.m_contactStates );
	m_stack.Free( island.m_contacts );
	m_stack.Free( island.m_velocities );
	m_stack.Free( island.m_bodies );

	// 更新宽相位里动态刚体的 AABB（它们经过积分后位置变了）
	// Update the broadphase AABBs
	for ( q3Body* body = m_bodyList; body; body = body->m_next )
	{
		if ( body->m_flags & q3Body::eStatic )
			continue;

		body->SynchronizeProxies( );
	}

	// 基于更新后的 AABB 查找新的潜在接触对
	// Look for new contacts
	m_contactManager.FindNewContacts( );

	// 清空所有刚体上累积的力和力矩（它们已在积分时用掉）
	// Clear all forces
	for ( q3Body* body = m_bodyList; body; body = body->m_next )
	{
		q3Identity( body->m_force );
		q3Identity( body->m_torque );
	}
}

//--------------------------------------------------------------------------------------------------
// 创建新刚体并加入场景。
// 内存从堆分配器（m_heap）分配，用 placement new 构造，然后头插进 m_bodyList 链表。
q3Body* q3Scene::CreateBody( const q3BodyDef& def )
{
	q3Body* body = (q3Body*)m_heap.Allocate( sizeof( q3Body ) );
	new (body) q3Body( def, this );   // placement new：在已分配内存上构造 q3Body

	// 头插法：新刚体插入链表头部
	// Add body to scene bodyList
	body->m_prev = NULL;
	body->m_next = m_bodyList;

	if ( m_bodyList )
		m_bodyList->m_prev = body;

	m_bodyList = body;
	++m_bodyCount;

	return body;
}

//--------------------------------------------------------------------------------------------------
// 从场景移除一个刚体：先移除它的所有接触和盒体，再从链表摘除，最后释放内存。
void q3Scene::RemoveBody( q3Body* body )
{
	assert( m_bodyCount > 0 );

	// 移除与该刚体相关的所有接触
	m_contactManager.RemoveContactsFromBody( body );

	// 移除刚体上附着的所有盒体
	body->RemoveAllBoxes( );

	// 从双向链表中摘除该刚体
	// Remove body from scene bodyList
	if ( body->m_next )
		body->m_next->m_prev = body->m_prev;

	if ( body->m_prev )
		body->m_prev->m_next = body->m_next;

	if ( body == m_bodyList )
		m_bodyList = body->m_next;

	--m_bodyCount;

	m_heap.Free( body );
}

//--------------------------------------------------------------------------------------------------
void q3Scene::RemoveAllBodies( )
{
	q3Body* body = m_bodyList;

	while ( body )
	{
		q3Body* next = body->m_next;

		body->RemoveAllBoxes( );

		m_heap.Free( body );

		body = next;
	}

	m_bodyList = NULL;
}

//--------------------------------------------------------------------------------------------------
void q3Scene::SetAllowSleep( bool allowSleep )
{
	m_allowSleep = allowSleep;

	if ( !allowSleep )
	{
		for ( q3Body* body = m_bodyList; body; body = body->m_next )
			body->SetToAwake( );
	}
}

//--------------------------------------------------------------------------------------------------
void q3Scene::SetIterations( i32 iterations )
{
	m_iterations = q3Max( 1, iterations );
}

//--------------------------------------------------------------------------------------------------
void q3Scene::SetEnableFriction( bool enabled )
{
	m_enableFriction = enabled;
}

//--------------------------------------------------------------------------------------------------
void q3Scene::Render( q3Render* render ) const
{
	q3Body* body = m_bodyList;

	while ( body )
	{
		body->Render( render );
		body = body->m_next;
	}

	m_contactManager.RenderContacts( render );
	//m_contactManager.m_broadphase.m_tree.Render( render );
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Scene::GetGravity( ) const
{
	return m_gravity;
}

//--------------------------------------------------------------------------------------------------
void q3Scene::SetGravity( const q3Vec3& gravity )
{
	m_gravity = gravity;
}

//--------------------------------------------------------------------------------------------------
void q3Scene::Shutdown( )
{
	RemoveAllBodies( );

	m_boxAllocator.Clear( );
}

//--------------------------------------------------------------------------------------------------
void q3Scene::SetContactListener( q3ContactListener* listener )
{
	m_contactManager.m_contactListener = listener;
}

//--------------------------------------------------------------------------------------------------
// AABB 查询：找出所有可能与该 AABB 相交的形状。
// 流程：先用宽相位树快速筛出 AABB 重叠的候选盒体，再对每个候选做精确的 AABB-AABB 检测，
// 真正相交才回调 ReportShape。返回 true 继续查询，false 提前终止。
void q3Scene::QueryAABB( q3QueryCallback *cb, const q3AABB& aabb ) const
{
	// 局部回调包装器：宽相位树查询时回调 TreeCallBack
	struct SceneQueryWrapper
	{
		bool TreeCallBack( i32 id )
		{
			q3AABB aabb;
			q3Box *box = (q3Box *)broadPhase->m_tree.GetUserData( id );

			// 精确检测：算该盒体的世界 AABB，与查询 AABB 做相交测试
			box->ComputeAABB( box->body->GetTransform( ), &aabb );

			if ( q3AABBtoAABB( m_aabb, aabb ) )
			{
				return cb->ReportShape( box );
			}

			return true;
		}

		q3QueryCallback *cb;
		const q3BroadPhase *broadPhase;
		q3AABB m_aabb;
	};

	SceneQueryWrapper wrapper;
	wrapper.m_aabb = aabb;
	wrapper.broadPhase = &m_contactManager.m_broadphase;
	wrapper.cb = cb;
	m_contactManager.m_broadphase.m_tree.Query( &wrapper, aabb );
}

//--------------------------------------------------------------------------------------------------
// 点查询：找出所有包含世界空间点 point 的形状。
// 因为宽相位树只支持 AABB 查询，所以把点扩成一个很小的 AABB（半宽 0.5）去查询，
// 再对候选盒体做精确的点包含测试（TestPoint）。
void q3Scene::QueryPoint( q3QueryCallback *cb, const q3Vec3& point ) const
{
	struct SceneQueryWrapper
	{
		bool TreeCallBack( i32 id )
		{
			q3Box *box = (q3Box *)broadPhase->m_tree.GetUserData( id );

			// 精确检测：点是否在盒体内
			if ( box->TestPoint( box->body->GetTransform( ), m_point ) )
			{
				cb->ReportShape( box );
			}

			return true;
		}

		q3QueryCallback *cb;
		const q3BroadPhase *broadPhase;
		q3Vec3 m_point;
	};

	SceneQueryWrapper wrapper;
	wrapper.m_point = point;
	wrapper.broadPhase = &m_contactManager.m_broadphase;
	wrapper.cb = cb;
	// 把点「加粗」成一个小 AABB，以便交给宽相位树查询
	const r32 k_fattener = r32( 0.5 );
	q3Vec3 v( k_fattener, k_fattener, k_fattener );
	q3AABB aabb;
	aabb.min = point - v;
	aabb.max = point + v;
	m_contactManager.m_broadphase.m_tree.Query( &wrapper, aabb );
}

//--------------------------------------------------------------------------------------------------
// 射线查询：找出所有被射线击中的形状。
// 先用宽相位树筛出射线穿过的 AABB 的候选，再对每个候选做精确的 Raycast。
void q3Scene::RayCast( q3QueryCallback *cb, q3RaycastData& rayCast ) const
{
	struct SceneQueryWrapper
	{
		bool TreeCallBack( i32 id )
		{
			q3Box *box = (q3Box *)broadPhase->m_tree.GetUserData( id );

			// 精确检测：射线与盒体的 slab 求交
			if ( box->Raycast( box->body->GetTransform( ), m_rayCast ) )
			{
				return cb->ReportShape( box );
			}

			return true;
		}

		q3QueryCallback *cb;
		const q3BroadPhase *broadPhase;
		q3RaycastData *m_rayCast;
	};

	SceneQueryWrapper wrapper;
	wrapper.m_rayCast = &rayCast;
	wrapper.broadPhase = &m_contactManager.m_broadphase;
	wrapper.cb = cb;
	m_contactManager.m_broadphase.m_tree.Query( &wrapper, rayCast );
}

//--------------------------------------------------------------------------------------------------
// 场景转储：把所有刚体和形状输出成 C++ 代码到日志文件，可直接复制进 demo 重建场景。
// 注意：接触不会被记录（缓存的历史求解不会被保存），所以最好在场景初始化时 dump，
// 而不是模拟中途 dump。
void q3Scene::Dump( FILE* file ) const
{
	fprintf( file, "// Ensure 64/32-bit memory compatability with the dump contents\n" );
	fprintf( file, "assert( sizeof( int* ) == %lu );\n", sizeof( int* ) );
	fprintf( file, "scene.SetGravity( q3Vec3( %.15lf, %.15lf, %.15lf ) );\n", m_gravity.x, m_gravity.y, m_gravity.z );
	fprintf( file, "scene.SetAllowSleep( %s );\n", m_allowSleep ? "true" : "false" );
	fprintf( file, "scene.SetEnableFriction( %s );\n", m_enableFriction ? "true" : "false" );

	fprintf( file, "q3Body** bodies = (q3Body**)q3Alloc( sizeof( q3Body* ) * %d );\n", m_bodyCount );

	i32 i = 0;
	for ( q3Body* body = m_bodyList; body; body = body->m_next, ++i )
	{
		body->Dump( file, i );
	}

	fprintf( file, "q3Free( bodies );\n" );
}
