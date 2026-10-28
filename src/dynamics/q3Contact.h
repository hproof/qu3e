//--------------------------------------------------------------------------------------------------
/**
@file	q3Contact.h

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

#ifndef Q3CONTACT_H
#define Q3CONTACT_H

#include "../math/q3Math.h"
#include "../common/q3Settings.h"
#include "../collision/q3Box.h"
#include "../collision/q3Collide.h"

//--------------------------------------------------------------------------------------------------
// q3Contact
//--------------------------------------------------------------------------------------------------
class q3Body;
struct q3Box;
struct q3ContactConstraint;

// Restitution mixing. The idea is to use the maximum bounciness, so bouncy
// objects will never not bounce during collisions.
// 弹性（恢复系数）混合：取两者中较大值，保证「弹的物体撞到硬的物体仍然会弹」。
inline r32 q3MixRestitution( const q3Box* A, const q3Box* B )
{
	return q3Max( A->restitution, B->restitution );
}

// Friction mixing. The idea is to allow a very low friction value to
// drive down the mixing result. Example: anything slides on ice.
// 摩擦系数混合：取两者乘积的平方根（几何平均），这样任一方摩擦极低（如冰面）
// 都会把最终摩擦拉低，符合「任何东西在冰上都打滑」的直觉。
inline r32 q3MixFriction( const q3Box* A, const q3Box* B )
{
	return std::sqrt( A->friction * B->friction );
}

// in stands for "incoming"
// out stands for "outgoing"
// I stands for "incident"
// R stands for "reference"
// See D. Gregorius GDC 2015 on creating contacts for more details
// Each feature pair is used to cache solutions from one physics tick to another. This is
// called warmstarting, and lets boxes stack and stay stable. Feature pairs identify points
// of contact over multiple physics ticks. Each feature pair is the junction of an incoming
// feature and an outgoing feature, usually a result of clipping routines. The exact info
// stored in the feature pair can be arbitrary as long as the result is a unique ID for a
// given intersecting configuration.
//==============================================================================
// q3FeaturePair：接触特征的唯一标识（feature pair）。
// 名词：in/out = 入射/出射，I/R = incident/reference（参考面），
// 详见 Dirk Gregorius GDC 2015 关于接触生成的演讲。
//
// 用途：把一个接触点在多个物理帧之间对应起来，从而「复用上一帧的解」，
// 即 warm starting（预热）——这让盒子堆叠时更稳定、不容易抖动。
// 特征对由「入射特征 + 出射特征」组合而成，通常是裁剪（clipping）算法的产物。
// 只要该组合能唯一标识一个相交构型，存的具体内容可以任意。
//==============================================================================
union q3FeaturePair
{
	struct
	{
		u8 inR;
		u8 outR;
		u8 inI;
		u8 outI;
	};

	i32 key;
};

// 单个接触点：记录两物体相碰的位置、穿透深度、以及求解用到的冲量/质量等状态。
struct q3Contact
{
	q3Vec3 position;			// 接触点的世界坐标
	r32 penetration;			// 穿透深度（两物体互相嵌入的深度）
	r32 normalImpulse;			// 累计法向冲量（沿法线方向，用于推开物体）
	r32 tangentImpulse[ 2 ];	// 累计切向（摩擦）冲量（两个切向分量）
	r32 bias;					// 位置修正偏置（恢复系数 + Baumgarte 稳定项）
	r32 normalMass;				// 法向约束质量（有效质量的倒数形式，见 PreSolve）
	r32 tangentMass[ 2 ];		// 切向约束质量（同上，用于摩擦约束）
	q3FeaturePair fp;			// 该接触点的特征标识（跨帧匹配/warm start 用）
	u8 warmStarted;				// 已 warm start 的帧数（调试渲染用）
};

// 碰撞流形（manifold）：一次碰撞检测在两物体间生成的所有接触点的集合。
// 最多 8 个接触点，加上法线、切向量等共同描述两物体如何接触。
struct q3Manifold
{
	void SetPair( q3Box *a, q3Box *b );

	q3Box *A;
	q3Box *B;

	q3Vec3 normal;				// 接触法线（方向从 A 指向 B）
	q3Vec3 tangentVectors[ 2 ];	// 两个正交的切向量（与法线垂直，用于摩擦）
	q3Contact contacts[ 8 ];    // 接触点数组
	i32 contactCount;           // 实际接触点数量

	q3Manifold* next;           // 流形链表：下一个
	q3Manifold* prev;           // 流形链表：上一个

	bool sensor;                // 是否为传感器（只检测、不产生物理响应）
};

// 接触边：把一个刚体与「某个接触约束」关联起来。
// 每个刚体维护一条接触边链表（q3Body::m_contactList），
// 每条边指向另一端刚体和所属的接触约束。
struct q3ContactEdge
{
	q3Body *other;              // 接触的另一端刚体
	q3ContactConstraint *constraint;    // 所属的接触约束
	q3ContactEdge* next;        // 链表：下一个
	q3ContactEdge* prev;        // 链表：上一个
};

//==============================================================================
// q3ContactConstraint：一个接触约束，表示「两个形状之间的接触关系」。
// 它同时挂在两条链表上：
//   1. 全局接触链表（q3ContactManager 维护）；
//   2. 两个刚体各自的接触边链表（通过 edgeA / edgeB 连接）。
// 内部包含一个流形（manifold），并记录摩擦/恢复系数、碰撞状态标志等。
//==============================================================================
struct q3ContactConstraint
{
	void SolveCollision( void );

	q3Box *A, *B;               // 两个碰撞形状
	q3Body *bodyA, *bodyB;      // 两个碰撞刚体

	q3ContactEdge edgeA;        // A 侧的接触边
	q3ContactEdge edgeB;        // B 侧的接触边
	q3ContactConstraint* next;  // 全局接触链表：下一个
	q3ContactConstraint* prev;  // 全局接触链表：上一个

	r32 friction;               // 混合后的摩擦系数
	r32 restitution;            // 混合后的恢复（弹性）系数

	q3Manifold manifold;        // 接触流形

	enum
	{
		eColliding    = 0x00000001, // 本帧正在碰撞（接触求解时置位）
		eWasColliding = 0x00000002, // 上一帧在碰撞（用于检测「开始/结束碰撞」事件）
		eIsland       = 0x00000004, // 岛屿划分时的内部标记
	};

	i32 m_flags;

	friend class q3ContactManager;
	friend class q3Scene;
	friend struct q3Island;
	friend struct q3ContactSolver;
};

#endif // Q3CONTACT_H
