//--------------------------------------------------------------------------------------------------
/**
@file	q3BroadPhase.h

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

#ifndef Q3BROADPHASE_H
#define Q3BROADPHASE_H

#include "../common/q3Types.h"
#include "q3DynamicAABBTree.h"
#include "../common/q3Memory.h"

//--------------------------------------------------------------------------------------------------
// q3BroadPhase
//--------------------------------------------------------------------------------------------------
class q3ContactManager;
struct q3Box;
struct q3Transform;
struct q3AABB;

// 一对可能相交的形状，用其在动态 AABB 树中的索引表示（A <= B）。
struct q3ContactPair
{
	i32 A;
	i32 B;
};

//==============================================================================
// q3BroadPhase：宽相位（broadphase）。
// 物理引擎的碰撞检测分两阶段：
//   1. 宽相位：用廉价的包围盒（AABB）快速找出「可能」相交的形状对；
//   2. 窄相位：对这些候选对做精确的形状碰撞检测（见 q3Collide）。
// 宽相位内部用一个「动态 AABB 树」（q3DynamicAABBTree）维护所有形状的包围盒，
// 只对「本帧移动过的」形状重新查询，避免每帧对所有形状两两比较（O(n²) → 近似 O(n log n)）。
// 找到的候选对交给 q3ContactManager 建立接触约束。
//==============================================================================
class q3BroadPhase
{
public:
	q3BroadPhase( q3ContactManager *manager );
	~q3BroadPhase( );

	// 向宽相位插入一个形状的包围盒，返回/记录其树索引。
	void InsertBox( q3Box *shape, const q3AABB& aabb );

	// 从宽相位移除一个形状。
	void RemoveBox( const q3Box *shape );

	// Generates the contact list. All previous contacts are returned to the allocator
	// before generation occurs.
	// 生成接触候选对列表（本帧移动过的形状重新查询树），
	// 并把找到的形状对交给 ContactManager 建立接触。
	void UpdatePairs( void );

	// 更新某形状的包围盒（物体移动后调用）。
	void Update( i32 id, const q3AABB& aabb );

	// 判断两个形状的（膨胀后的）包围盒是否重叠。
	bool TestOverlap( i32 A, i32 B ) const;

private:
	q3ContactManager *m_manager;    // 关联的接触管理器（用于回调 AddContact）

	q3ContactPair* m_pairBuffer;    // 候选接触对缓冲
	i32 m_pairCount;                // 当前候选对数量
	i32 m_pairCapacity;             // 候选对缓冲容量

	i32* m_moveBuffer;              // 本帧移动过的形状索引缓冲
	i32 m_moveCount;                // 当前移动形状数量
	i32 m_moveCapacity;             // 移动缓冲容量

	q3DynamicAABBTree m_tree;       // 动态 AABB 树（BVH）
	i32 m_currentIndex;             // 当前正在查询的形状索引（查询回调用）

	void BufferMove( i32 id );      // 把移动形状的索引加入缓冲
	bool TreeCallBack( i32 index ); // 树查询回调：把命中的形状对加入候选列表

	friend class q3DynamicAABBTree;
	friend class q3Scene;
};

// 树查询回调：当查询的 AABB 与某个叶节点重叠时被调用。
// 把「当前形状 m_currentIndex」与「命中形状 index」组成一对（无序化，保证 A<=B）
// 存入候选对缓冲，供后续排序去重。
inline bool q3BroadPhase::TreeCallBack( i32 index )
{
	// Cannot collide with self
	// 不能和自己碰撞
	if ( index == m_currentIndex )
		return true;

	// 候选对缓冲满了就翻倍扩容
	if ( m_pairCount == m_pairCapacity )
	{
		q3ContactPair* oldBuffer = m_pairBuffer;
		m_pairCapacity *= 2;
		m_pairBuffer = (q3ContactPair*)q3Alloc( m_pairCapacity * sizeof( q3ContactPair ) );
		memcpy( m_pairBuffer, oldBuffer, m_pairCount * sizeof( q3ContactPair ) );
		q3Free( oldBuffer );
	}

	// 无序对规范化为 A <= B，便于后续排序去重
	i32 iA = q3Min( index, m_currentIndex );
	i32 iB = q3Max( index, m_currentIndex );

	m_pairBuffer[ m_pairCount ].A = iA;
	m_pairBuffer[ m_pairCount ].B = iB;
	++m_pairCount;

	return true;
}

#endif // Q3BROADPHASE_H
