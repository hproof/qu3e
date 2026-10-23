//--------------------------------------------------------------------------------------------------
/**
@file	q3Box.cpp

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

#include "q3Box.h"
#include "../math/q3Vec3.h"

//--------------------------------------------------------------------------------------------------
// q3Box
//--------------------------------------------------------------------------------------------------

// 检测 box 和 point 相交
bool q3Box::TestPoint( const q3Transform& tx, const q3Vec3& p ) const
{
	q3Transform world = q3Mul( tx, local );		// 计算 box 的世界变化
	q3Vec3 p0 = q3MulT( world, p );						// 把 p 变化到 box 的 局部坐标

	// 依次比较 x/y/z 三维
	for ( int i = 0; i < 3; ++i )
	{
		r32 d = p0[ i ];
		r32 ei = e[ i ];

		if ( d > ei || d < -ei )
		{
			return false;
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------

// 检测 ray 和 box 相交
bool q3Box::Raycast( const q3Transform& tx, q3RaycastData* raycast ) const
{
	q3Transform world = q3Mul( tx, local );			// 计算 box 的 世界变化
	q3Vec3 d = q3MulT( world.rotation, raycast->dir );		// 局部 方向
	q3Vec3 p = q3MulT( world, raycast->start );						// 局部 位置
	const r32 epsilon = r32( 1.0e-8 );		// 误差值
	r32 tmin = 0;
	r32 tmax = raycast->t;						// 射线 time 的 [min, max]

	// t = (e[ i ] - p.[ i ]) / d[ i ]
	r32 t0;
	r32 t1;
	q3Vec3 n0;

	// 在 3个轴线上， 分别检查 slab(2个平面构成的区域) 和 ray 的 (t0,t1)
	for ( int i = 0; i < 3; ++i )
	{
		// 如果 ray 和 该轴平行， 则检查 半长
		// Check for ray parallel to and outside of AABB
		if ( q3Abs( d[ i ] ) < epsilon )
		{
			// Detect separating axes
			if ( p[ i ] < -e[ i ] || p[ i ] > e[ i ] )
			{
				return false;
			}
		}

		else
		{
			r32 d0 = r32( 1.0 ) / d[ i ];
			r32 s = q3Sign( d[ i ] );
			r32 ei = e[ i ] * s;
			q3Vec3 n( 0, 0, 0 );
			n[ i ] = -s;

			t0 = -(ei + p[ i ]) * d0;			// 进入 slab 的 time 
			t1 = (ei - p[ i ]) * d0;			// 离开 slab 的 time

			if ( t0 > tmin )			// 进入 取最大值
			{
				n0 = n;
				tmin = t0;
			}

			tmax = q3Min( tmax, t1 );		// 离开 取 最小值

			if ( tmin > tmax )
			{
				return false;
			}
		}
	}

	raycast->normal = q3Mul( world.rotation, n0 ); 
	raycast->toi = tmin;

	return true;
}

//--------------------------------------------------------------------------------------------------
void q3Box::ComputeAABB( const q3Transform& tx, q3AABB* aabb ) const
{
	q3Transform world = q3Mul( tx, local );		// 计算 local to world

	// 计算 local 
	q3Vec3 v[ 8 ] = {
		q3Vec3( -e.x, -e.y, -e.z ),
		q3Vec3( -e.x, -e.y,  e.z ),
		q3Vec3( -e.x,  e.y, -e.z ),
		q3Vec3( -e.x,  e.y,  e.z ),
		q3Vec3(  e.x, -e.y, -e.z ),
		q3Vec3(  e.x, -e.y,  e.z ),
		q3Vec3(  e.x,  e.y, -e.z ),
		q3Vec3(  e.x,  e.y,  e.z )
	};

	// 转换为 world
	for ( i32 i = 0; i < 8; ++i )
		v[ i ] = q3Mul( world, v[ i ] );

	// 计算 min-max
	q3Vec3 min( Q3_R32_MAX, Q3_R32_MAX, Q3_R32_MAX );
	q3Vec3 max( -Q3_R32_MAX, -Q3_R32_MAX, -Q3_R32_MAX );

	for ( i32 i = 0; i < 8; ++i )
	{
		min = q3Min( min, v[ i ] );
		max = q3Max( max, v[ i ] );
	}

	aabb->min = min;
	aabb->max = max;
}

//--------------------------------------------------------------------------------------------------

// 计算 质量属性
void q3Box::ComputeMass( q3MassData* md ) const
{
	// 质量 = 体积 * 密度
	r32 mass = r32( 8.0 ) * e.x * e.y * e.z * density;		

	// 绕质心的惯性张量, 长方体标准公式
	// Calculate inertia tensor
	r32 ex2 = r32( 4.0 ) * e.x * e.x;
	r32 ey2 = r32( 4.0 ) * e.y * e.y;
	r32 ez2 = r32( 4.0 ) * e.z * e.z;
	r32 x = r32( 1.0 / 12.0 ) * mass * (ey2 + ez2);		// 绕 X 轴的 惯性由 YZ 决定
	r32 y = r32( 1.0 / 12.0 ) * mass * (ex2 + ez2);
	r32 z = r32( 1.0 / 12.0 ) * mass * (ex2 + ey2);
	q3Mat3 I = q3Diagonal( x, y, z );

	// 旋转到 刚体模型 空间,  I' = R * I * T(R)
	// Transform tensor to local space
	I = local.rotation * I * q3Transpose( local.rotation );

	// 平移到 刚体模型 空间原点
	q3Mat3 identity;
	q3Identity( identity );
	I += (identity * q3Dot( local.position, local.position ) - q3OuterProduct( local.position, local.position )) * mass;

	md->center = local.position;
	md->inertia = I;
	md->mass = mass;
}

//--------------------------------------------------------------------------------------------------
const i32 kBoxIndices[ 36 ] = {
	1 - 1, 7 - 1, 5 - 1,
	1 - 1, 3 - 1, 7 - 1,
	1 - 1, 4 - 1, 3 - 1,
	1 - 1, 2 - 1, 4 - 1,
	3 - 1, 8 - 1, 7 - 1,
	3 - 1, 4 - 1, 8 - 1,
	5 - 1, 7 - 1, 8 - 1,
	5 - 1, 8 - 1, 6 - 1,
	1 - 1, 5 - 1, 6 - 1,
	1 - 1, 6 - 1, 2 - 1,
	2 - 1, 6 - 1, 8 - 1,
	2 - 1, 8 - 1, 4 - 1
};

//--------------------------------------------------------------------------------------------------

// 绘制
void q3Box::Render( const q3Transform& tx, bool awake, q3Render* render ) const
{
	q3Transform world = q3Mul( tx, local );		// local to world

	// local 坐标下的 8个顶点
	q3Vec3 vertices[ 8 ] = {
		q3Vec3( -e.x, -e.y, -e.z ),
		q3Vec3( -e.x, -e.y,  e.z ),
		q3Vec3( -e.x,  e.y, -e.z ),
		q3Vec3( -e.x,  e.y,  e.z ),
		q3Vec3(  e.x, -e.y, -e.z ),
		q3Vec3(  e.x, -e.y,  e.z ),
		q3Vec3(  e.x,  e.y, -e.z ),
		q3Vec3(  e.x,  e.y,  e.z )
	};

	// 转换为 世界坐标
	for ( i32 i = 0; i < 36; i += 3 )
	{
		// 世界坐标顶点
		q3Vec3 a = q3Mul( world, vertices[ kBoxIndices[ i ] ] );
		q3Vec3 b = q3Mul( world, vertices[ kBoxIndices[ i + 1 ] ] );
		q3Vec3 c = q3Mul( world, vertices[ kBoxIndices[ i + 2 ] ] );

		// 计算法线
		q3Vec3 n = q3Normalize( q3Cross( b - a, c - a ) );

		//render->SetPenColor( 0.2f, 0.4f, 0.7f, 0.5f );
		//render->SetPenPosition( a.x, a.y, a.z );
		//render->Line( b.x, b.y, b.z );
		//render->Line( c.x, c.y, c.z );
		//render->Line( a.x, a.y, a.z );

		// 绘制三角面
		render->SetTriNormal( n.x, n.y, n.z );
		render->Triangle( a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z );
	}
}
