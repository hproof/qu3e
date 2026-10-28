//--------------------------------------------------------------------------------------------------
/**
@file	q3ContactManager.h

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

#ifndef Q3CONTACTMANAGER_H
#define Q3CONTACTMANAGER_H

#include "../common/q3Types.h"
#include "../broadphase/q3BroadPhase.h"
#include "../common/q3Memory.h"

//--------------------------------------------------------------------------------------------------
// q3ContactManager
//--------------------------------------------------------------------------------------------------
struct q3ContactConstraint;
class q3ContactListener;
struct q3Box;
class q3Body;
class q3Render;
class q3Stack;

//==============================================================================
// q3ContactManager：接触管理器。负责「碰撞检测的宽相位 + 接触约束的增删维护」。
// 它内部持有：
//   - 一个宽相位（q3BroadPhase m_broadphase）：快速找出可能相交的形状对；
//   - 一条全局接触约束链表（m_contactList）：保存所有当前活跃的接触；
//   - 一个接触约束的内存分配器（m_allocator）。
// 每个接触约束 q3ContactConstraint 都会同时挂在「全局链表」和「两端刚体的
// 接触边链表」上，因此删除刚体/形状时能高效地把相关接触一并移除。
//==============================================================================
class q3ContactManager
{
public:
	q3ContactManager( q3Stack* stack );

	// Add a new contact constraint for a pair of objects
	// unless the contact constraint already exists
	// 为一对形状添加接触约束（若该对已存在则跳过，避免重复约束）。
	void AddContact( q3Box *A, q3Box *B );

	// Has broadphase find all contacts and call AddContact on the
	// ContactManager for each pair found
	// 让宽相位找出所有可能接触的形状对，并对每一对调用 AddContact。
	void FindNewContacts( void );

	// Remove a specific contact
	// 移除某个具体的接触约束。
	void RemoveContact( q3ContactConstraint *contact );

	// Remove all contacts from a body
	// 移除某刚体上的所有接触约束。
	void RemoveContactsFromBody( q3Body *body );

	// 把某刚体的所有形状从宽相位中移除。
	void RemoveFromBroadphase( q3Body *body );

	// Remove contacts without broadphase overlap
	// Solves contact manifolds
	// 对每个接触约束做窄相位碰撞检测（生成/更新接触点），
	// 并移除那些宽相位已无重叠的接触。即「接触维护」的核心步骤。
	void TestCollisions( void );

	// 碰撞求解入口（供并行调用），参数为接触约束指针。
	static void SolveCollision( void* param );

	// 调试渲染所有接触点与法线。
	void RenderContacts( q3Render* debugDrawer ) const;

private:
	q3ContactConstraint* m_contactList; // 全局接触约束链表头
	i32 m_contactCount;                 // 当前接触约束数量
	q3Stack* m_stack;                   // 栈分配器（宽相位临时缓冲用）
	q3PagedAllocator m_allocator;       // 接触约束的分页内存分配器
	q3BroadPhase m_broadphase;          // 宽相位（快速配对可能相交的形状）
	q3ContactListener *m_contactListener;   // 碰撞事件回调（Begin/EndContact）

	friend class q3BroadPhase;
	friend class q3Scene;
	friend struct q3Box;
	friend class q3Body;
};

#endif // Q3CONTACTMANAGER_H
