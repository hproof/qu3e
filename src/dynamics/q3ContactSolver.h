//--------------------------------------------------------------------------------------------------
/**
@file	q3ContactSolver.h

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

#ifndef Q3CONTACTSOLVER_H
#define Q3CONTACTSOLVER_H

#include "../math/q3Math.h"
#include "../common/q3Settings.h"

//--------------------------------------------------------------------------------------------------
// q3ContactSolver
//--------------------------------------------------------------------------------------------------
struct q3Island;
struct q3VelocityState;

// 单个接触点的求解状态（求解器的工作副本，求解完再拷回 q3Contact）。
struct q3ContactState
{
	q3Vec3 ra;					// 从质心 A 到接触点的力臂向量（Vector from C.O.M to contact position）
	q3Vec3 rb;					// 从质心 B 到接触点的力臂向量
	r32 penetration;			// 穿透深度（Depth of penetration from collision）
	r32 normalImpulse;			// 累计法向冲量（Accumulated normal impulse）
	r32 tangentImpulse[ 2 ];	// 累计切向（摩擦）冲量（Accumulated friction impulse）
	r32 bias;					// 位置修正偏置（恢复系数 + Baumgarte）
	r32 normalMass;				// 法向约束质量（Normal constraint mass）
	r32 tangentMass[ 2 ];		// 切向约束质量（Tangent constraint mass）
};

// 一个接触约束（两物体间可能有多个接触点）的求解状态。
struct q3ContactConstraintState
{
	q3ContactState contacts[ 8 ];   // 最多 8 个接触点
	i32 contactCount;               // 实际接触点数量
	q3Vec3 tangentVectors[ 2 ];	// 两个正交切向量（摩擦方向）
	q3Vec3 normal;				// 接触法线（从 A 指向 B）
	q3Vec3 centerA;             // 刚体 A 的质心
	q3Vec3 centerB;             // 刚体 B 的质心
	q3Mat3 iA;                  // 刚体 A 的世界逆惯性张量
	q3Mat3 iB;                  // 刚体 B 的世界逆惯性张量
	r32 mA;                     // 刚体 A 的逆质量
	r32 mB;                     // 刚体 B 的逆质量
	r32 restitution;            // 恢复（弹性）系数
	r32 friction;               // 摩擦系数
	i32 indexA;                 // 刚体 A 在岛屿刚体数组中的索引
	i32 indexB;                 // 刚体 B 在岛屿刚体数组中的索引
};

//==============================================================================
// q3ContactSolver：接触约束求解器。
// 采用「顺序冲量（Sequential Impulse）」算法，本质是 Gauss-Seidel 迭代：
// 逐个接触点施加冲量修正速度，直到所有接触点的速度约束都尽量满足。
// 每个接触点有两个约束：
//   1. 法向约束（非穿透）：冲量只能推、不能拉（>= 0）；
//   2. 切向约束（摩擦）：冲量被限制在摩擦锥内（|Ft| <= μ·Fn，库仑摩擦）。
// 求解器不直接操作刚体，而是在岛屿提供的工作副本（m_velocities / m_contacts）
// 上迭代，最后拷回刚体。
//==============================================================================
struct q3ContactSolver
{
	void Initialize( q3Island *island );   // 从岛屿取得接触/速度缓冲区
	void ShutDown( void );                  // 求解完把冲量拷回流形（供 warm start）
	void PreSolve( r32 dt );                // 预热：计算有效质量、bias，施加历史冲量
	void Solve( void );                     // 迭代求解法向 + 摩擦约束

	q3Island *m_island;                             // 所属岛屿
	q3ContactConstraintState *m_contacts;           // 接触求解状态数组
	i32 m_contactCount;                             // 接触约束数量
	q3VelocityState *m_velocities;                  // 刚体速度状态数组（工作副本）

	bool m_enableFriction;                          // 是否启用摩擦
};

#endif // Q3CONTACTSOLVER_H
