//--------------------------------------------------------------------------------------------------
/**
@file	q3ContactManager.cpp

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

#include "q3ContactManager.h"
#include "../collision/q3Box.h"
#include "q3Body.h"
#include "q3Contact.h"
#include "../scene/q3Scene.h"
#include "../debug/q3Render.h"

//--------------------------------------------------------------------------------------------------
// q3ContactManager
//--------------------------------------------------------------------------------------------------
// 构造函数：初始化栈分配器、接触约束分配器（每块 256 个）、宽相位，
// 以及接触链表、计数和事件回调为空。
q3ContactManager::q3ContactManager( q3Stack* stack )
	: m_stack( stack )
	, m_allocator( sizeof( q3ContactConstraint ), 256 )
	, m_broadphase( this )
{
	m_contactList = NULL;
	m_contactCount = 0;
	m_contactListener = NULL;
}

//--------------------------------------------------------------------------------------------------
void q3ContactManager::AddContact( q3Box *A, q3Box *B )
{
	q3Body *bodyA = A->body;
	q3Body *bodyB = B->body;
	// 先做廉价判断：两刚体是否可能碰撞（类型/碰撞层过滤）
	if ( !bodyA->CanCollide( bodyB ) )
		return;

	// Search for existing matching contact
	// Return if found duplicate to avoid duplicate constraints
	// Mark pre-existing duplicates as active
	// 在 A 的接触边链表里找是否已存在与 bodyB 的接触；
	// 若形状也相同，说明这对形状的约束已存在，直接返回，避免重复约束。
	q3ContactEdge* edge = A->body->m_contactList;
	while ( edge )
	{
		if ( edge->other == bodyB )
		{
			q3Box *shapeA = edge->constraint->A;
			q3Box *shapeB = edge->constraint->B;

			// @TODO: Verify this against Box2D; not sure if this is all we need here
			if( (A == shapeA) && (B == shapeB) )
				return;
		}

		edge = edge->next;
	}

	// Create new contact
	// 分配并初始化新的接触约束
	q3ContactConstraint *contact = (q3ContactConstraint*)m_allocator.Allocate( );
	contact->A = A;
	contact->B = B;
	contact->bodyA = A->body;
	contact->bodyB = B->body;
	contact->manifold.SetPair( A, B );
	contact->m_flags = 0;
	contact->friction = q3MixFriction( A, B );        // 混合摩擦系数
	contact->restitution = q3MixRestitution( A, B );  // 混合恢复系数
	contact->manifold.contactCount = 0;

	for ( i32 i = 0; i < 8; ++i )
		contact->manifold.contacts[ i ].warmStarted = 0;

	// 头插法挂到全局接触链表
	contact->prev = NULL;
	contact->next = m_contactList;
	if ( m_contactList )
		m_contactList->prev = contact;
	m_contactList = contact;

	// Connect A
	// 把接触约束连接到 A 刚体的接触边链表
	contact->edgeA.constraint = contact;
	contact->edgeA.other = bodyB;

	contact->edgeA.prev = NULL;
	contact->edgeA.next = bodyA->m_contactList;
	if ( bodyA->m_contactList )
		bodyA->m_contactList->prev = &contact->edgeA;
	bodyA->m_contactList = &contact->edgeA;

	// Connect B
	// 同样连接到 B 刚体的接触边链表
	contact->edgeB.constraint = contact;
	contact->edgeB.other = bodyA;

	contact->edgeB.prev = NULL;
	contact->edgeB.next = bodyB->m_contactList;
	if ( bodyB->m_contactList )
		bodyB->m_contactList->prev = &contact->edgeB;
	bodyB->m_contactList = &contact->edgeB;

	// 新接触产生，唤醒两端刚体（让它们参与下一帧求解）
	bodyA->SetToAwake( );
	bodyB->SetToAwake( );

	++m_contactCount;
}

//--------------------------------------------------------------------------------------------------
// 让宽相位重新配对可能相交的形状（会回调 AddContact 建立新接触）。
void q3ContactManager::FindNewContacts( )
{
	m_broadphase.UpdatePairs( );
}

//--------------------------------------------------------------------------------------------------
// 移除一个接触约束：先从 A、B 两刚体的接触边链表里摘除两条边，
// 再从全局链表摘除该约束，最后归还内存。
void q3ContactManager::RemoveContact( q3ContactConstraint *contact )
{
	q3Body *A = contact->bodyA;
	q3Body *B = contact->bodyB;

	// Remove from A
	// 从 A 的接触边链表摘除 edgeA
	if ( contact->edgeA.prev )
		contact->edgeA.prev->next = contact->edgeA.next;

	if ( contact->edgeA.next )
		contact->edgeA.next->prev = contact->edgeA.prev;

	if ( &contact->edgeA == A->m_contactList )
		A->m_contactList = contact->edgeA.next;

	// Remove from B
	// 从 B 的接触边链表摘除 edgeB
	if ( contact->edgeB.prev )
		contact->edgeB.prev->next = contact->edgeB.next;

	if ( contact->edgeB.next )
		contact->edgeB.next->prev = contact->edgeB.prev;

	if ( &contact->edgeB == B->m_contactList )
		B->m_contactList = contact->edgeB.next;

	// 接触被移除，唤醒两端刚体（它们的状态发生了变化）
	A->SetToAwake( );
	B->SetToAwake( );

	// Remove contact from the manager
	// 从全局链表摘除该约束
	if ( contact->prev )
		contact->prev->next = contact->next;

	if ( contact->next )
		contact->next->prev = contact->prev;

	if ( contact == m_contactList )
		m_contactList = contact->next;

	--m_contactCount;

	m_allocator.Free( contact );
}

//--------------------------------------------------------------------------------------------------
// 移除某刚体上的所有接触：遍历它的接触边链表，逐个调用 RemoveContact。
void q3ContactManager::RemoveContactsFromBody( q3Body *body )
{
	q3ContactEdge* edge = body->m_contactList;

	while( edge )
	{
		q3ContactEdge* next = edge->next;   // 先记住下一个（RemoveContact 会改链表）
		RemoveContact( edge->constraint );
		edge = next;
	}
}

//--------------------------------------------------------------------------------------------------
// 把某刚体的所有形状从宽相位中移除（用于删除刚体时清理）。
void q3ContactManager::RemoveFromBroadphase( q3Body *body )
{
	q3Box* box = body->m_boxes;

	while ( box )
	{
		m_broadphase.RemoveBox( box );
		box = box->next;
	}
}

//--------------------------------------------------------------------------------------------------
// 接触维护的核心：遍历所有接触约束，做窄相位碰撞检测并更新状态。
// 对每个约束依次处理：
//   1. 清 eIsland 标记；
//   2. 两端都休眠 → 跳过（不需要更新）；
//   3. 不再可能碰撞（CanCollide 或宽相位无重叠）→ 移除该约束；
//   4. 否则重新做碰撞检测，warm start 复用上一帧的冲量，
//      并触发 Begin/EndContact 事件。
void q3ContactManager::TestCollisions( void )
{
	q3ContactConstraint* constraint = m_contactList;

	while( constraint )
	{
		q3Box *A = constraint->A;
		q3Box *B = constraint->B;
		q3Body *bodyA = A->body;
		q3Body *bodyB = B->body;

		constraint->m_flags &= ~q3ContactConstraint::eIsland;

		// 两端都休眠的接触不更新
		if( !bodyA->IsAwake( ) && !bodyB->IsAwake( ) )
		{
			constraint = constraint->next;
			continue;
		}

		// 不再可能碰撞（如碰撞层被改、刚体类型变了）→ 移除
		if ( !bodyA->CanCollide( bodyB ) )
		{
			q3ContactConstraint* next = constraint->next;
			RemoveContact( constraint );
			constraint = next;
			continue;
		}

		// Check if contact should persist
		// 宽相位已无重叠（包围盒都不相交了）→ 移除
		if ( !m_broadphase.TestOverlap( A->broadPhaseIndex, B->broadPhaseIndex ) )
		{
			q3ContactConstraint* next = constraint->next;
			RemoveContact( constraint );
			constraint = next;
			continue;
		}

		// 保存旧流形用于 warm start（把上一帧的冲量复用过来）
		q3Manifold* manifold = &constraint->manifold;
		q3Manifold oldManifold = constraint->manifold;
		q3Vec3 ot0 = oldManifold.tangentVectors[ 0 ];
		q3Vec3 ot1 = oldManifold.tangentVectors[ 1 ];

		// 重新做碰撞检测，生成新的接触点
		constraint->SolveCollision( );

		// 由法线构造两个正交切向量（构成接触点的局部坐标系）
		q3ComputeBasis( manifold->normal, manifold->tangentVectors, manifold->tangentVectors + 1 );

		for ( i32 i = 0; i < manifold->contactCount; ++i )
		{
			q3Contact *c = manifold->contacts + i;
			c->tangentImpulse[ 0 ] = c->tangentImpulse[ 1 ] = c->normalImpulse = r32( 0.0 );
			u8 oldWarmStart = c->warmStarted;
			c->warmStarted = u8( 0 );

			// 在新流形中寻找特征标识相同的旧接触点，把旧冲量复用过来
			for ( i32 j = 0; j < oldManifold.contactCount; ++j )
			{
				q3Contact *oc = oldManifold.contacts + j;
				if ( c->fp.key == oc->fp.key )
				{
					c->normalImpulse = oc->normalImpulse;

					// Attempt to re-project old friction solutions
					// 旧的摩擦冲量在新切向基下重新投影（因为切向量可能随法线变化）
					q3Vec3 friction = ot0 * oc->tangentImpulse[ 0 ] + ot1 * oc->tangentImpulse[ 1 ];
					c->tangentImpulse[ 0 ] = q3Dot( friction, manifold->tangentVectors[ 0 ] );
					c->tangentImpulse[ 1 ] = q3Dot( friction, manifold->tangentVectors[ 1 ] );
					c->warmStarted = q3Max( oldWarmStart, u8( oldWarmStart + 1 ) );
					break;
				}
			}
		}

		// 触发碰撞事件（BeginContact / EndContact）
		if ( m_contactListener )
		{
			i32 now_colliding = constraint->m_flags & q3ContactConstraint::eColliding;
			i32 was_colliding = constraint->m_flags & q3ContactConstraint::eWasColliding;

			if ( now_colliding && !was_colliding )
				m_contactListener->BeginContact( constraint );

			else if ( !now_colliding && was_colliding )
				m_contactListener->EndContact( constraint );
		}

		constraint = constraint->next;
	}
}

//--------------------------------------------------------------------------------------------------
// 调试渲染所有接触点：颜色表示 warm start 程度（越红 = warm start 越久），
// 并从每个接触点沿法线方向画一条短线。
void q3ContactManager::RenderContacts( q3Render* render ) const
{
	const q3ContactConstraint *contact = m_contactList;

	while ( contact )
	{
		const q3Manifold *m = &contact->manifold;

		// 只渲染正在碰撞的接触
		if ( !(contact->m_flags & q3ContactConstraint::eColliding) )
		{
			contact = contact->next;
			continue;
		}

		for ( i32 j = 0; j < m->contactCount; ++j)
		{
			const q3Contact *c = m->contacts + j;
			f32 blue = (f32)(255 - c->warmStarted) / 255.0f;
			f32 red = 1.0f - blue;
			render->SetScale( 10.0f, 10.0f, 10.0f );
			render->SetPenColor( red, blue, blue );
			render->SetPenPosition( c->position.x, c->position.y, c->position.z );
			render->Point( );

			if ( m->A->body->IsAwake( ) )
				render->SetPenColor( 1.0f, 1.0f, 1.0f );
			else
				render->SetPenColor( 0.2f, 0.2f, 0.2f );

			render->SetPenPosition( c->position.x, c->position.y, c->position.z );
			render->Line(
				c->position.x + m->normal.x * 0.5f,
				c->position.y + m->normal.y * 0.5f,
				c->position.z + m->normal.z * 0.5f
				);
		}

		contact = contact->next;
	}

	render->SetScale( 1.0f, 1.0f, 1.0f );
}
