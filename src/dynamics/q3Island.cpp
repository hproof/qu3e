//--------------------------------------------------------------------------------------------------
/**
@file	q3Island.cpp

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

#include "q3Island.h"
#include "q3Body.h"
#include "../common/q3Memory.h"
#include "q3ContactSolver.h"
#include "../common/q3Settings.h"
#include "../broadphase/q3BroadPhase.h"
#include "q3Contact.h"

//--------------------------------------------------------------------------------------------------
// q3Island
//--------------------------------------------------------------------------------------------------
//==============================================================================
// Solve()：求解整个岛屿。完整流程：
//   1. 施加重力，计算世界惯性张量，积分速度，施加阻尼
//   2. 创建接触求解器：初始化（拷贝 manifold 状态）+ 预热（warm start）+ 迭代求解
//   3. 把解出的速度拷回刚体，积分位置（线平移 + 四元数旋转）
//   4. 休眠判定：整个岛屿静止足够久则整体休眠
//==============================================================================
void q3Island::Solve( )
{
	// ---- 阶段 1：施加重力 + 积分速度 ----
	// Apply gravity
	// Integrate velocities and create state buffers, calculate world inertia
	for ( i32 i = 0 ; i < m_bodyCount; ++i )
	{
		q3Body *body = m_bodies[ i ];
		q3VelocityState *v = m_velocities + i;

		if ( body->m_flags & q3Body::eDynamic )   // 只处理动态刚体
		{
			// 施加重力（乘以刚体的重力缩放因子 gravityScale）
			body->ApplyLinearForce( m_gravity * body->m_gravityScale );

			// 计算世界空间惯性张量的逆：I_world^-1 = R * I_model^-1 * Rᵀ
			// 惯性张量是二阶张量，在旋转 R 下按 R·I·Rᵀ 变换
			// Calculate world space intertia tensor
			q3Mat3 r = body->m_tx.rotation;
			body->m_invInertiaWorld = r * body->m_invInertiaModel * q3Transpose( r );

			// 积分速度：v += (F/m)·dt，w += (I⁻¹·τ)·dt（牛顿第二定律 + 转动方程）
			// Integrate velocity
			body->m_linearVelocity += (body->m_force * body->m_invMass) * m_dt;
			body->m_angularVelocity += (body->m_invInertiaWorld * body->m_torque) * m_dt;

			// 阻尼：模拟空气阻力等，让速度按指数衰减。
			// 精确解 v2 = v1·exp(-c·dt)，这里用 Pade 近似 v2 = v1/(1 + c·dt)，
			// 计算更快且无条件稳定。
			// From Box2D!
			// Apply damping.
			// ODE: dv/dt + c * v = 0
			// Solution: v(t) = v0 * exp(-c * t)
			// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
			// v2 = exp(-c * dt) * v1
			// Pade approximation:
			// v2 = v1 * 1 / (1 + c * dt)
			body->m_linearVelocity *= r32( 1.0 ) / (r32( 1.0 ) + m_dt * body->m_linearDamping);
			body->m_angularVelocity *= r32( 1.0 ) / (r32( 1.0 ) + m_dt * body->m_angularDamping);
		}

		// 把速度拷到工作缓冲（接触求解器在这份副本上迭代，不直接改刚体）
		v->v = body->m_linearVelocity;
		v->w = body->m_angularVelocity;
	}

	// ---- 阶段 2：接触约束求解 ----
	// 创建接触求解器，传入速度/约束状态缓冲。
	// Initialize：把 manifold 里的接触数据拷贝到连续内存缓冲；
	// PreSolve：预热（warm start，复用上一帧的历史冲量，加速收敛）。
	// Create contact solver, pass in state buffers, create buffers for contacts
	// Initialize velocity constraint for normal + friction and warm start
	q3ContactSolver contactSolver;
	contactSolver.Initialize( this );
	contactSolver.PreSolve( m_dt );

	// 迭代求解接触约束（Gauss-Seidel 式，迭代次数越多越收敛，但越慢）
	// Solve contacts
	for ( i32 i = 0; i < m_iterations; ++i )
		contactSolver.Solve( );

	contactSolver.ShutDown( );

	// ---- 阶段 3：拷回速度 + 积分位置 ----
	// Copy back state buffers
	// Integrate positions
	for ( i32 i = 0 ; i < m_bodyCount; ++i )
	{
		q3Body *body = m_bodies[ i ];
		q3VelocityState *v = m_velocities + i;

		if ( body->m_flags & q3Body::eStatic )
			continue;

		// 把接触求解器算出的最终速度拷回刚体
		body->m_linearVelocity = v->v;
		body->m_angularVelocity = v->w;

		// 积分位置：质心沿线速度平移 + 四元数沿角速度旋转
		// Integrate position
		body->m_worldCenter += body->m_linearVelocity * m_dt;
		body->m_q.Integrate( body->m_angularVelocity, m_dt );
		body->m_q = q3Normalize( body->m_q );       // 归一化四元数，防止数值漂移
		body->m_tx.rotation = body->m_q.ToMat3( );  // 四元数转回旋转矩阵
	}

	// ---- 阶段 4：休眠判定 ----
	// 休眠是一种 CPU 优化：静止的刚体不再参与求解，直到被外力/碰撞唤醒。
	// 只有当「整个岛屿」都静止足够久，才整体休眠（保证一起静止的刚体一起睡）。
	if ( m_allowSleep )
	{
		// 求整个岛屿里最小的「静止时间」
		// Find minimum sleep time of the entire island
		f32 minSleepTime = Q3_R32_MAX;
		for ( i32 i = 0; i < m_bodyCount; ++i )
		{
			q3Body* body = m_bodies[ i ];

			if ( body->m_flags & q3Body::eStatic )
				continue;

			// 线速度/角速度的平方与容差比较（平方避免开根号）
			const r32 sqrLinVel = q3Dot( body->m_linearVelocity, body->m_linearVelocity );
			const r32 cbAngVel = q3Dot( body->m_angularVelocity, body->m_angularVelocity );
			const r32 linTol = Q3_SLEEP_LINEAR;
			const r32 angTol = Q3_SLEEP_ANGULAR;

			if ( sqrLinVel > linTol || cbAngVel > angTol )
			{
				// 还在动 → 静止时间归零
				minSleepTime = r32( 0.0 );
				body->m_sleepTime = r32( 0.0 );
			}

			else
			{
				// 近似静止 → 累计静止时间
				body->m_sleepTime += m_dt;
				minSleepTime = q3Min( minSleepTime, body->m_sleepTime );
			}
		}

		// 若整个岛屿的静止时间都超过阈值，则整体休眠；
		// 若某个刚体还在动（minSleepTime 被归零），则下帧重新检测。
		// Put entire island to sleep so long as the minimum found sleep time
		// is below the threshold. If the minimum sleep time reaches below the
		// sleeping threshold, the entire island will be reformed next step
		// and sleep test will be tried again.
		if ( minSleepTime > Q3_SLEEP_TIME )
		{
			for ( i32 i = 0; i < m_bodyCount; ++i )
				m_bodies[ i ]->SetToSleep( );
		}
	}
}

//--------------------------------------------------------------------------------------------------
// 把刚体加入岛屿。记录它在岛屿数组中的索引（m_islandIndex），
// 供接触状态里的 indexA/indexB 引用。
void q3Island::Add( q3Body *body )
{
	assert( m_bodyCount < m_bodyCapacity );

	body->m_islandIndex = m_bodyCount;

	m_bodies[ m_bodyCount++ ] = body;
}

//--------------------------------------------------------------------------------------------------
// 把接触约束加入岛屿。
void q3Island::Add( q3ContactConstraint *contact )
{
	assert( m_contactCount < m_contactCapacity );

	m_contacts[ m_contactCount++ ] = contact;
}

//--------------------------------------------------------------------------------------------------
// 初始化接触求解状态：把每个接触约束（q3ContactConstraint）的 manifold 数据
// 拷贝到连续内存的求解状态（q3ContactConstraintState）里，供求解器高效线性遍历。
// 拷贝的关键量：
//   centerA/B：两刚体质心（用于算接触点到质心的力臂）
//   iA/iB    ：世界惯性张量的逆
//   mA/mB    ：质量的逆
//   ra/rb    ：接触点到两质心的向量（用于计算冲量产生的力矩）
//   normalImpulse/tangentImpulse：上一帧的历史冲量（warm start 用）
void q3Island::Initialize( )
{
	for ( i32 i = 0; i < m_contactCount; ++i )
	{
		q3ContactConstraint *cc = m_contacts[ i ];

		q3ContactConstraintState *c = m_contactStates + i;

		c->centerA = cc->bodyA->m_worldCenter;
		c->centerB = cc->bodyB->m_worldCenter;
		c->iA = cc->bodyA->m_invInertiaWorld;
		c->iB = cc->bodyB->m_invInertiaWorld;
		c->mA = cc->bodyA->m_invMass;
		c->mB = cc->bodyB->m_invMass;
		c->restitution = cc->restitution;
		c->friction = cc->friction;
		c->indexA = cc->bodyA->m_islandIndex;
		c->indexB = cc->bodyB->m_islandIndex;
		c->normal = cc->manifold.normal;
		c->tangentVectors[ 0 ] = cc->manifold.tangentVectors[ 0 ];
		c->tangentVectors[ 1 ] = cc->manifold.tangentVectors[ 1 ];
		c->contactCount = cc->manifold.contactCount;

		for ( i32 j = 0; j < c->contactCount; ++j )
		{
			q3ContactState *s = c->contacts + j;
			q3Contact *cp = cc->manifold.contacts + j;
			s->ra = cp->position - c->centerA;   // 接触点到 A 质心的力臂
			s->rb = cp->position - c->centerB;   // 接触点到 B 质心的力臂
			s->penetration = cp->penetration;
			s->normalImpulse = cp->normalImpulse;             // 历史法向冲量
			s->tangentImpulse[ 0 ] = cp->tangentImpulse[ 0 ]; // 历史切向冲量
			s->tangentImpulse[ 1 ] = cp->tangentImpulse[ 1 ];
		}
	}
}
