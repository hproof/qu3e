//--------------------------------------------------------------------------------------------------
/**
@file	q3Island.h

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

#ifndef Q3ISLAND_H
#define Q3ISLAND_H

#include "../math/q3Math.h"
#include "../common/q3Geometry.h"
#include "../common/q3Settings.h"

//==============================================================================
// q3Island：岛屿求解器。
// 岛屿 = 由接触连接起来的刚体连通分量（见 q3Scene::Step 的 DFS 建岛）。
// 本结构负责对岛屿内的一组刚体 + 接触约束做一次完整的动力学求解：
//   施加重力 → 积分速度 → 迭代求解接触约束 → 积分位置 → 休眠判定。
// 注意：它自身不分配内存，所有数组（m_bodies/m_velocities/m_contacts/m_contactStates）
// 都指向 q3Scene::Step 在栈上预留的缓冲区（单缓冲区复用，岛屿串行求解）。
//==============================================================================

//--------------------------------------------------------------------------------------------------
// q3Island
//--------------------------------------------------------------------------------------------------
class q3BroadPhase;
class q3Body;
struct q3ContactConstraint;
struct q3ContactConstraintState;

// 刚体的速度状态缓冲：接触求解器在工作副本上迭代，求解完再拷回刚体。
struct q3VelocityState
{
	q3Vec3 w;   // 角速度（angular velocity）
	q3Vec3 v;   // 线速度（linear velocity）
};

struct q3Island
{
	void Solve( );                        // 求解整个岛屿（重力 + 积分 + 约束求解 + 休眠）
	void Add( q3Body *body );             // 加入一个刚体
	void Add( q3ContactConstraint *contact );  // 加入一个接触约束
	void Initialize( );                   // 初始化接触状态（从 manifold 拷贝到求解缓冲）

	// 刚体数组 + 速度状态缓冲（一一对应）
	q3Body **m_bodies;
	q3VelocityState *m_velocities;
	i32 m_bodyCapacity;                   // 容量（栈上预分配）
	i32 m_bodyCount;                      // 当前刚体数量

	// 接触约束数组 + 约束状态缓冲（一一对应）
	q3ContactConstraint **m_contacts;
	q3ContactConstraintState *m_contactStates;
	i32 m_contactCount;
	i32 m_contactCapacity;

	r32 m_dt;                             // 时间步长
	q3Vec3 m_gravity;                     // 重力向量
	i32 m_iterations;                     // 接触求解的迭代次数

	bool m_allowSleep;                    // 是否允许休眠
	bool m_enableFriction;                // 是否启用摩擦
};

#endif // Q3ISLAND_H
