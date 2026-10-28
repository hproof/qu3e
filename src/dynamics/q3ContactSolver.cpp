//--------------------------------------------------------------------------------------------------
/**
@file	q3ContactSolver.cpp

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

#include "q3ContactSolver.h"
#include "q3Contact.h"
#include "q3Island.h"
#include "../common/q3Memory.h"
#include "q3Body.h"
#include "../common/q3Geometry.h"
#include "../common/q3Settings.h"

//--------------------------------------------------------------------------------------------------
// q3ContactSolver
//--------------------------------------------------------------------------------------------------
// 初始化：从岛屿取得接触约束状态数组、速度工作副本，以及摩擦开关。
void q3ContactSolver::Initialize( q3Island *island )
{
	m_island = island;
	m_contactCount = island->m_contactCount;
	m_contacts = island->m_contactStates;
	m_velocities = m_island->m_velocities;
	m_enableFriction = island->m_enableFriction;
}

//--------------------------------------------------------------------------------------------------
// 求解结束：把工作副本里的累计冲量拷回真正的接触点（q3Contact），
// 供下一帧 warm start 复用。
void q3ContactSolver::ShutDown( void )
{
	for ( i32 i = 0; i < m_contactCount; ++i )
	{
		q3ContactConstraintState *c = m_contacts + i;
		q3ContactConstraint *cc = m_island->m_contacts[ i ];

		for ( i32 j = 0; j < c->contactCount; ++j )
		{
			q3Contact *oc = cc->manifold.contacts + j;
			q3ContactState *cs = c->contacts + j;
			oc->normalImpulse = cs->normalImpulse;
			oc->tangentImpulse[ 0 ] = cs->tangentImpulse[ 0 ];
			oc->tangentImpulse[ 1 ] = cs->tangentImpulse[ 1 ];
		}
	}
}

//--------------------------------------------------------------------------------------------------
// PreSolve：求解前的准备阶段，为每个接触点做三件事：
//   1. 预计算「有效质量」normalMass / tangentMass（约束质量，用于把速度误差换算成冲量）；
//   2. 预计算偏置 bias（Baumgarte 位置修正 + 恢复系数）；
//   3. warm start：把上一帧的累计冲量提前施加到速度上，加速收敛。
void q3ContactSolver::PreSolve( r32 dt )
{
	for ( i32 i = 0; i < m_contactCount; ++i )
	{
		q3ContactConstraintState *cs = m_contacts + i;

		// 取出两端刚体当前的速度（工作副本）
		q3Vec3 vA = m_velocities[ cs->indexA ].v;
		q3Vec3 wA = m_velocities[ cs->indexA ].w;
		q3Vec3 vB = m_velocities[ cs->indexB ].v;
		q3Vec3 wB = m_velocities[ cs->indexB ].w;

		for ( i32 j = 0; j < cs->contactCount; ++j )
		{
			q3ContactState *c = cs->contacts + j;

			// Precalculate JM^-1JT for contact and friction constraints
			// ---- 计算有效质量（约束质量） ----
			// 对速度约束 J·v = 0，冲量修正为 λ = -(J·v) / (J·M⁻¹·Jᵀ)。
			// 分母 J·M⁻¹·Jᵀ 就是「有效质量」，它衡量：施加单位冲量能产生多大速度变化。
			// 这里分别对法向和两个切向计算。
			// 法向的雅可比：J_n = [ -n, -(ra×n), n, (rb×n) ]（从 A 指向 B 的法线）
			q3Vec3 raCn = q3Cross( c->ra, cs->normal );   // ra × n
			q3Vec3 rbCn = q3Cross( c->rb, cs->normal );   // rb × n
			r32 nm = cs->mA + cs->mB;                     // 平动部分
			r32 tm[ 2 ];
			tm[ 0 ] = nm;
			tm[ 1 ] = nm;

			// 加上转动部分：(ra×n)ᵀ·I_A⁻¹·(ra×n) + (rb×n)ᵀ·I_B⁻¹·(rb×n)
			nm += q3Dot( raCn, cs->iA * raCn ) + q3Dot( rbCn, cs->iB * rbCn );
			c->normalMass = q3Invert( nm );               // 有效质量 = 1 / 分母

			// 切向的雅可比：J_t = [ -t, -(ra×t), t, (rb×t) ]
			for ( i32 i = 0; i < 2; ++i )
			{
				q3Vec3 raCt = q3Cross( cs->tangentVectors[ i ], c->ra );
				q3Vec3 rbCt = q3Cross( cs->tangentVectors[ i ], c->rb );
				tm[ i ] += q3Dot( raCt, cs->iA * raCt ) + q3Dot( rbCt, cs->iB * rbCt );
				c->tangentMass[ i ] = q3Invert( tm[ i ] );
			}

			// Precalculate bias factor
			// ---- 预计算位置修正偏置（bias）----
			// Baumgarte 稳定化：当物体穿透超过允许容差（Q3_PENETRATION_SLOP）时，
			// 额外给一个向外的速度，把物体逐渐推开，避免穿透累积。
			// 只有真的穿透（penetration + slop < 0，即穿得更深）才修正。
			c->bias = -Q3_BAUMGARTE * (r32( 1.0 ) / dt) * q3Min( r32( 0.0 ), c->penetration + Q3_PENETRATION_SLOP );

			// Warm start contact
			// ---- warm start：把上一帧累计的冲量先施加到速度上 ----
			// 上一帧的冲量是很好的初始猜测，提前施加能大大加快本帧的收敛。
			q3Vec3 P = cs->normal * c->normalImpulse;

			if ( m_enableFriction )
			{
				P += cs->tangentVectors[ 0 ] * c->tangentImpulse[ 0 ];
				P += cs->tangentVectors[ 1 ] * c->tangentImpulse[ 1 ];
			}

			// 冲量 P 施加到 A（负向）、B（正向）
			vA -= P * cs->mA;
			wA -= cs->iA * q3Cross( c->ra, P );   // 冲量在 ra 处产生的角速度变化：I⁻¹·(ra×P)

			vB += P * cs->mB;
			wB += cs->iB * q3Cross( c->rb, P );

			// Add in restitution bias
			// ---- 叠加恢复系数（弹性）偏置 ----
			// 计算接触点沿法线的相对速度 dv = (vB + wB×rb) - (vA + wA×ra) 在法线上的投影。
			// 若 dv 明显为负（两物体正快速接近），按恢复系数给它一个反弹速度。
			r32 dv = q3Dot( vB + q3Cross( wB, c->rb ) - vA - q3Cross( wA, c->ra ), cs->normal );

			if ( dv < -r32( 1.0 ) )
				c->bias += -(cs->restitution) * dv;
		}

		// 把施加了 warm start 冲量后的速度写回工作副本
		m_velocities[ cs->indexA ].v = vA;
		m_velocities[ cs->indexA ].w = wA;
		m_velocities[ cs->indexB ].v = vB;
		m_velocities[ cs->indexB ].w = wB;
	}
}

//--------------------------------------------------------------------------------------------------
// Solve：顺序冲量迭代求解（一次 Gauss-Seidel 扫描）。
// 外层会被 q3Island::Solve 调用多次（m_iterations 次），次数越多越收敛。
// 对每个接触点：先解摩擦（切向），再解法向；每个冲量都立即施加到速度上，
// 因此后面的接触点能看到前面的修正结果（这就是 Gauss-Seidel 的顺序性）。
void q3ContactSolver::Solve( )
{
	for ( i32 i = 0; i < m_contactCount; ++i )
	{
		q3ContactConstraintState *cs = m_contacts + i;

		q3Vec3 vA = m_velocities[ cs->indexA ].v;
		q3Vec3 wA = m_velocities[ cs->indexA ].w;
		q3Vec3 vB = m_velocities[ cs->indexB ].v;
		q3Vec3 wB = m_velocities[ cs->indexB ].w;

		for ( i32 j = 0; j < cs->contactCount; ++j )
		{
			q3ContactState *c = cs->contacts + j;

			// relative velocity at contact
			// 接触点处的相对速度（B 相对 A）
			q3Vec3 dv = vB + q3Cross( wB, c->rb ) - vA - q3Cross( wA, c->ra );

			// Friction
			// ---- 摩擦约束（切向） ----
			if ( m_enableFriction )
			{
				for ( i32 i = 0; i < 2; ++i )
				{
					// 消除该切向的相对速度所需的冲量
					r32 lambda = -q3Dot( dv, cs->tangentVectors[ i ] ) * c->tangentMass[ i ];

					// Calculate frictional impulse
					// 摩擦冲量的上限：库仑摩擦 μ·Fn（摩擦系数 × 法向冲量）
					r32 maxLambda = cs->friction * c->normalImpulse;

					// Clamp frictional impulse
					// 把累计切向冲量夹在 [-μFn, +μFn] 内（摩擦锥）
					r32 oldPT = c->tangentImpulse[ i ];
					c->tangentImpulse[ i ] = q3Clamp( -maxLambda, maxLambda, oldPT + lambda );
					lambda = c->tangentImpulse[ i ] - oldPT;   // 实际本帧新增的冲量

					// Apply friction impulse
					// 施加摩擦冲量
					q3Vec3 impulse = cs->tangentVectors[ i ] * lambda;
					vA -= impulse * cs->mA;
					wA -= cs->iA * q3Cross( c->ra, impulse );

					vB += impulse * cs->mB;
					wB += cs->iB * q3Cross( c->rb, impulse );
				}
			}

			// Normal
			// ---- 法向约束（非穿透） ----
			{
				// 摩擦修正后重新计算相对速度
				dv = vB + q3Cross( wB, c->rb ) - vA - q3Cross( wA, c->ra );

				// Normal impulse
				// 沿法线的接近速度（负值 = 正在互相靠近/穿透）
				r32 vn = q3Dot( dv, cs->normal );

				// Factor in positional bias to calculate impulse scalar j
				// 计算法向冲量：λ = normalMass · (-vn + bias)
				// bias 用于修正穿透（Baumgarte）和反弹（恢复系数）。
				r32 lambda = c->normalMass * (-vn + c->bias);

				// Clamp impulse
				// 法向冲量只能 >= 0（只能推、不能拉，避免接触点「粘住」）
				r32 tempPN = c->normalImpulse;
				c->normalImpulse = q3Max( tempPN + lambda, r32( 0.0 ) );
				lambda = c->normalImpulse - tempPN;   // 实际本帧新增的冲量

				// Apply impulse
				// 施加法向冲量
				q3Vec3 impulse = cs->normal * lambda;
				vA -= impulse * cs->mA;
				wA -= cs->iA * q3Cross( c->ra, impulse );

				vB += impulse * cs->mB;
				wB += cs->iB * q3Cross( c->rb, impulse );
			}
		}

		// 写回本接触约束修正后的速度
		m_velocities[ cs->indexA ].v = vA;
		m_velocities[ cs->indexA ].w = wA;
		m_velocities[ cs->indexB ].v = vB;
		m_velocities[ cs->indexB ].w = wB;
	}
}
