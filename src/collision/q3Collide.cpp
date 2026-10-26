//--------------------------------------------------------------------------------------------------
/**
@file	q3Collide.cpp

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

#include "q3Collide.h"
#include "../dynamics/q3Body.h"
#include "../dynamics/q3Contact.h"

//==============================================================================
// 本文件实现 OBB（有向包围盒）之间的碰撞检测，并生成碰撞流形（manifold）。
// 
// 参考： https://zhuanlan.zhihu.com/p/508874496
//		. 2个 cube 检测， 设为 A B , 分离轴 仅可能存在与以下选项中: 
//			. A 的 6个面的法线， 去重后共 3 个
//			. B 的 面法线， 3个
//			. A的 3个轴  和 B 的 3个轴  两两叉乘， 得到的 9 个 向量
//			. 总共 3+3+9 = 15 个 候选轴
//
// 核心算法：SAT（分离轴定理，Separating Axis Theorem）
//   两个凸形状不相交 ⟺ 存在一条轴，使两者在该轴上的投影不重叠（分离）。
//   对两个 OBB 只需检查 15 条候选轴：
//     - 6 条面轴：A 的 3 条主轴（x/y/z）+ B 的 3 条主轴
//     - 9 条边轴：A 的三条边 × B 的三条边（叉积方向），仅在两盒非平行时检查
//
// 整体流程（详见 q3BoxtoBox）：
//   1. SAT 检查 15 条轴，找「最小穿透轴」→ 得到碰撞法线 + 穿透深度
//   2. 面接触（最小穿透轴是某条面轴）：用 Sutherland-Hodgman 裁剪，生成多个接触点
//   3. 边接触（最小穿透轴是两条边的叉积）：求两支撑边的最短距离，生成 1 个接触点
//==============================================================================

//--------------------------------------------------------------------------------------------------
// qBoxtoBox
//--------------------------------------------------------------------------------------------------

// 检查 当前轴(n/s/normal) 是否分离；  如果不分离(重叠) 在 (axis/sMax/axisNormal) 中保存 穿透最浅 的记录
// 总共 6 个 面轴
// 检查一条「面轴」（A 或 B 的一条主轴）是否是分离轴。
// 参数：
//   axis/sMax/axisNormal：输出，记录当前找到的最小穿透轴（碰撞法线候选）
//   n     ：轴编号（0~5 对应 A 的 xyz、B 的 xyz）
//   s     ：两盒在该轴上的投影重叠量。s > 0 表示有间隙（分离），s < 0 表示重叠
//   normal：该轴在世界空间的方向（单位向量）
// 返回 true 表示该轴是分离轴（两盒不相交），调用方应立即 return。
inline bool q3TrackFaceAxis( i32* axis, i32 n, r32 s, r32* sMax, const q3Vec3& normal, q3Vec3* axisNormal )
{
	// s > 0：投影不重叠 → 找到分离轴，两盒不相交
	if ( s > r32( 0.0 ) )
		return true;

	// s ≤ 0：重叠。保留「穿透最浅」的轴（s 最大，最接近 0）作为碰撞法线候选
	if ( s > *sMax )
	{
		*sMax = s;
		*axis = n;
		*axisNormal = normal;
	}

	return false;
}

//--------------------------------------------------------------------------------------------------
// 检查 当前轴(n/s/normal) 是否为分离轴；  如果不是（重叠） 则在 (axis/sMax/axisNormal) 中保存 穿透最浅 的记录
// 总共 9个 边轴
// 检查一条「边轴」（A 的一条边 × B 的一条边的叉积方向）是否是分离轴。
// 与 q3TrackFaceAxis 的区别：边轴法线（叉积结果）通常不是单位向量，
// 需要先归一化，把「沿未归一化法线的重叠量」折算成「沿单位法线的真实距离」再比较。
inline bool q3TrackEdgeAxis( i32* axis, i32 n, r32 s, r32* sMax, const q3Vec3& normal, q3Vec3* axisNormal )
{
	// 是分离轴
	if ( s > r32( 0.0 ) )
		return true;

	// 归一化：l = 1/|normal|，把 s 从「沿 normal」折算到「沿单位法线」的尺度
	r32 l = r32( 1.0 ) / q3Length( normal );
	s *= l;

	if ( s > *sMax )
	{
		*sMax = s;
		*axis = n;
		*axisNormal = normal * l;
	}

	return false;
}

//--------------------------------------------------------------------------------------------------
// 裁剪用的顶点：几何位置 + 特征对（feature pair）。
// 特征对记录该顶点来自哪个面/边，供接触求解器做 warm starting（热启动）复用历史冲量。
struct q3ClipVertex
{
	q3ClipVertex( )
	{
		f.key = ~0;   // ~0 = 无效特征标记
	}

	q3Vec3 v;						// 顶点
	q3FeaturePair f;		// 来自哪里
};

//--------------------------------------------------------------------------------------------------
// 根据 碰撞法线(n) 计算 碰撞面信息， out=4条边编号， basis=面的(u,v,normal), e=半长
//
// 根据「参考面」（reference face，即最小穿透轴所在的那个面）构造裁剪所需信息。
// 参考面由 axis（0~5：A/B 的 xyz 面轴）和法线方向决定，本函数输出：
//   out[4]：4 条裁剪边（clip edge）的特征索引，裁剪求交点时给新顶点标注来源
//   basis ：参考面的正交基，三行分别是 (u, v, n)：两个切向 + 一个法向
//   e     ：参考面在 basis 下的半长（裁剪时作为 u/v 方向的边界）
// 目的：把「任意朝向的参考面」摆到一个标准局部坐标系里，让后续裁剪变成
//       在 u/v 两个轴上的简单区间比较。下面的 out[] 数字是盒体面/边的特征编号。
void q3ComputeReferenceEdgesAndBasis( 
		const q3Vec3& eR, 						// box 的 半长
		const q3Transform& rtx, 			// box 的 worldTransform
		q3Vec3 n, 										// 碰撞法线
		i32 axis, 										// 最小穿透轴 0~5, (0~2 = A 的 xyz, 3~5 = B 的 xyz)
		u8* out, 											// 输出： 4条剪裁边的特征编号
		q3Mat3* basis, 								// 输出： 参考面的 正交基, world 坐标中该平面的 (u,v,n)
		q3Vec3* e )										// 输出： 参考面在 basis 中的半长
{
	// 法线变换到参考盒的局部空间，用其分量符号决定取参考面的哪个朝向
	n = q3MulT( rtx.rotation, n );

	// axis 0~2 是 A 的面轴，3~5 是 B 的面轴，这里统一归到 0~2（对应 x/y/z）
	if ( axis >= 3 )
		axis -= 3;

	switch ( axis )
	{
	case 0: // X 轴
		if ( n.x > r32( 0.0 ) )
		{
			out[ 0 ] = 1;			// 面的4条边 编号
			out[ 1 ] = 8;
			out[ 2 ] = 7;
			out[ 3 ] = 9;

			e->Set( eR.y, eR.z, eR.x );			// u=y, v=z, normal=x
			basis->SetRows( rtx.rotation.ey, rtx.rotation.ez, rtx.rotation.ex );
		}

		else // -X 轴
		{
			out[ 0 ] = 11;
			out[ 1 ] = 3;
			out[ 2 ] = 10;
			out[ 3 ] = 5;

			e->Set( eR.z, eR.y, eR.x );			// 交换 uv, 要保证 右手性
			basis->SetRows( rtx.rotation.ez, rtx.rotation.ey, -rtx.rotation.ex );
		}
		break;

	case 1:
		if ( n.y > r32( 0.0 ) )
		{
			out[ 0 ] = 0;
			out[ 1 ] = 1;
			out[ 2 ] = 2;
			out[ 3 ] = 3;

			e->Set( eR.z, eR.x, eR.y );
			basis->SetRows( rtx.rotation.ez, rtx.rotation.ex, rtx.rotation.ey );
		}

		else
		{
			out[ 0 ] = 4;
			out[ 1 ] = 5;
			out[ 2 ] = 6;
			out[ 3 ] = 7;

			e->Set( eR.z, eR.x, eR.y );
			basis->SetRows( rtx.rotation.ez, -rtx.rotation.ex, -rtx.rotation.ey );
		}
		break;

	case 2:
		if ( n.z > r32( 0.0 ) )
		{
			out[ 0 ] = 11;
			out[ 1 ] = 4;
			out[ 2 ] = 8;
			out[ 3 ] = 0;

			e->Set( eR.y, eR.x, eR.z );
			basis->SetRows( -rtx.rotation.ey, rtx.rotation.ex, rtx.rotation.ez );
		}

		else
		{
			out[ 0 ] = 6;
			out[ 1 ] = 10;
			out[ 2 ] = 2;
			out[ 3 ] = 9;

			e->Set( eR.y, eR.x, eR.z );
			basis->SetRows( -rtx.rotation.ey, -rtx.rotation.ex, -rtx.rotation.ez );
		}
		break;
	}
}

//--------------------------------------------------------------------------------------------------
// 计算「入射面」（incident face）：另一个盒体上最朝向参考面的那个面，输出其 4 个顶点。
// 原理：把法线变换到入射盒局部空间，分量绝对值最大的轴就是入射面的法向轴；
//       再按法线符号取该轴正/负一侧的 4 个角点。
// out 里的 inI/outI 是特征索引（面/边编号），与 q3ComputeReferenceEdgesAndBasis 的
// 编号约定一一对应，供接触求解器做 warm starting（热启动）复用历史冲量。
void q3ComputeIncidentFace( const q3Transform& itx, const q3Vec3& e, q3Vec3 n, q3ClipVertex* out )
{
	// 法线取反后变换到入射盒局部空间：我们要找的是「朝向参考面」的那一面
	n = -q3MulT( itx.rotation, n );
	q3Vec3 absN = q3Abs( n );

	if ( absN.x > absN.y && absN.x > absN.z )
	{
		if ( n.x > r32( 0.0 ) )
		{
			out[ 0 ].v.Set(  e.x,  e.y, -e.z );
			out[ 1 ].v.Set(  e.x,  e.y,  e.z );
			out[ 2 ].v.Set(  e.x, -e.y,  e.z );
			out[ 3 ].v.Set(  e.x, -e.y, -e.z );

			out[ 0 ].f.inI = 9;
			out[ 0 ].f.outI = 1;
			out[ 1 ].f.inI = 1;
			out[ 1 ].f.outI = 8;
			out[ 2 ].f.inI = 8;
			out[ 2 ].f.outI = 7;
			out[ 3 ].f.inI = 7;
			out[ 3 ].f.outI = 9;
		}

		else
		{
			out[ 0 ].v.Set( -e.x, -e.y,  e.z );
			out[ 1 ].v.Set( -e.x,  e.y,  e.z );
			out[ 2 ].v.Set( -e.x,  e.y, -e.z );
			out[ 3 ].v.Set( -e.x, -e.y, -e.z );

			out[ 0 ].f.inI = 5;
			out[ 0 ].f.outI = 11;
			out[ 1 ].f.inI = 11;
			out[ 1 ].f.outI = 3;
			out[ 2 ].f.inI = 3;
			out[ 2 ].f.outI = 10;
			out[ 3 ].f.inI = 10;
			out[ 3 ].f.outI = 5;
		}
	}

	else if ( absN.y > absN.x && absN.y > absN.z )
	{
		if ( n.y > r32( 0.0 ) )
		{
			out[ 0 ].v.Set( -e.x,  e.y,  e.z );
			out[ 1 ].v.Set(  e.x,  e.y,  e.z );
			out[ 2 ].v.Set(  e.x,  e.y, -e.z );
			out[ 3 ].v.Set( -e.x,  e.y, -e.z );

			out[ 0 ].f.inI = 3;
			out[ 0 ].f.outI = 0;
			out[ 1 ].f.inI = 0;
			out[ 1 ].f.outI = 1;
			out[ 2 ].f.inI = 1;
			out[ 2 ].f.outI = 2;
			out[ 3 ].f.inI = 2;
			out[ 3 ].f.outI = 3;
		}

		else
		{
			out[ 0 ].v.Set(  e.x, -e.y,  e.z );
			out[ 1 ].v.Set( -e.x, -e.y,  e.z );
			out[ 2 ].v.Set( -e.x, -e.y, -e.z );
			out[ 3 ].v.Set(  e.x, -e.y, -e.z );

			out[ 0 ].f.inI = 7;
			out[ 0 ].f.outI = 4;
			out[ 1 ].f.inI = 4;
			out[ 1 ].f.outI = 5;
			out[ 2 ].f.inI = 5;
			out[ 2 ].f.outI = 6;
			out[ 3 ].f.inI = 5;
			out[ 3 ].f.outI = 6;
		}
	}

	else
	{
		if ( n.z > r32( 0.0 ) )
		{
			out[ 0 ].v.Set( -e.x,  e.y,  e.z );
			out[ 1 ].v.Set( -e.x, -e.y,  e.z );
			out[ 2 ].v.Set(  e.x, -e.y,  e.z );
			out[ 3 ].v.Set(  e.x,  e.y,  e.z );

			out[ 0 ].f.inI = 0;
			out[ 0 ].f.outI = 11;
			out[ 1 ].f.inI = 11;
			out[ 1 ].f.outI = 4;
			out[ 2 ].f.inI = 4;
			out[ 2 ].f.outI = 8;
			out[ 3 ].f.inI = 8;
			out[ 3 ].f.outI = 0;
		}

		else
		{
			out[ 0 ].v.Set(  e.x, -e.y, -e.z );
			out[ 1 ].v.Set( -e.x, -e.y, -e.z );
			out[ 2 ].v.Set( -e.x,  e.y, -e.z );
			out[ 3 ].v.Set(  e.x,  e.y, -e.z );

			out[ 0 ].f.inI = 9;
			out[ 0 ].f.outI = 6;
			out[ 1 ].f.inI = 6;
			out[ 1 ].f.outI = 10;
			out[ 2 ].f.inI = 10;
			out[ 2 ].f.outI = 2;
			out[ 3 ].f.inI = 2;
			out[ 3 ].f.outI = 9;
		}
	}

	// 把 4 个顶点从入射盒局部空间变换到世界空间
	for ( i32 i = 0; i < 4; ++i )
		out[ i ].v = q3Mul( itx, out[ i ].v );
}

//--------------------------------------------------------------------------------------------------
// Sutherland-Hodgman 裁剪的辅助宏，判断顶点相对裁剪边的位置：
//   InFront(a)：在裁剪边「前」（外侧，要被裁掉）
//   Behind(a) ：在裁剪边「后」（内侧，保留）
//   On(a)     ：近似落在裁剪边上（容差 0.005）
// 「前/后」由 sign*coord - e 的符号决定：< 0 视为前（外侧），>= 0 视为后（内侧）。
#define InFront( a ) \
	((a) < r32( 0.0 ))

#define Behind( a ) \
	((a) >= r32( 0.0 ))

#define On( a ) \
	((a) < r32( 0.005 ) && (a) > -r32( 0.005 ))

// Sutherland-Hodgman 裁剪的单边步骤：用一条直线裁剪多边形，保留「后」（内侧）部分。
// 裁剪边由 sign/e/axis 定义：在 axis 分量上满足 sign*coord = e 的那条线。
// 对多边形的每条边 a→b，计算 da、db（顶点到裁剪边的有符号距离），分四种情况：
//   - a、b 都在后（内侧）        ：保留 b
//   - a 在前、b 在后            ：保留交点（进入）
//   - a 在后、b 在前            ：保留交点 + b（离开）
//   - 都在前（外侧）            ：丢弃
// 交点用线性插值求：t = da/(da - db)，交点在 a→b 的 t 处。
i32 q3Orthographic( r32 sign, r32 e, i32 axis, i32 clipEdge, q3ClipVertex* in, i32 inCount, q3ClipVertex* out )
{
	i32 outCount = 0;
	q3ClipVertex a = in[ inCount - 1 ];   // 前一个顶点（首尾相接的环）

	for ( i32 i = 0; i < inCount; ++i )
	{
		q3ClipVertex b = in[ i ];         // 当前顶点

		// 有符号距离：sign*coord - e，< 0 在前（外），>= 0 在后（内）
		r32 da = sign * a.v[ axis ] - e;
		r32 db = sign * b.v[ axis ] - e;

		q3ClipVertex cv;

		// B：a、b 都在内侧（或落在边上）→ 保留 b
		if ( ((InFront( da ) && InFront( db )) || On( da ) || On( db )) )
		{
			assert( outCount < 8 );
			out[ outCount++ ] = b;
		}

		// I：a 在前、b 在后 → 边从外穿入，保留交点
		else if ( InFront( da ) && Behind( db ) )
		{
			cv.f = b.f;
			cv.v = a.v + (b.v - a.v) * (da / (da - db));   // 线性插值求交点
			cv.f.outR = clipEdge;
			cv.f.outI = 0;
			assert( outCount < 8 );
			out[ outCount++ ] = cv;
		}

		// I, B：a 在后、b 在前 → 边从内穿出，保留交点 + b
		else if ( Behind( da ) && InFront( db ) )
		{
			cv.f = a.f;
			cv.v = a.v + (b.v - a.v) * (da / (da - db));
			cv.f.inR = clipEdge;
			cv.f.inI = 0;
			assert( outCount < 8 );
			out[ outCount++ ] = cv;

			assert( outCount < 8 );
			out[ outCount++ ] = b;
		}

		a = b;
	}

	return outCount;
}

//--------------------------------------------------------------------------------------------------
// Resources (also see q3BoxtoBox's resources):
// http://www.randygaul.net/2013/10/27/sutherland-hodgman-clipping/
// 完整裁剪：把入射面（4 顶点）依次用参考面的 4 条侧边裁剪，
// 得到落在参考面范围内的接触点，并输出每个点的穿透深度。
// 步骤：
//   1. 把入射面顶点变换到参考面的局部基（basis）下，并相对参考面中心 rPos 平移
//   2. 依次用 4 条边做 Sutherland-Hodgman 裁剪（+x、+y、-x、-y 四个半平面）
//   3. 保留「参考面背面」（z ≤ e.z）的点作为接触点，其 z 坐标差即穿透深度
// 参考：http://www.randygaul.net/2013/10/27/sutherland-hodgman-clipping/
i32 q3Clip( const q3Vec3& rPos, const q3Vec3& e, u8* clipEdges, const q3Mat3& basis, q3ClipVertex* incident, q3ClipVertex* outVerts, r32* outDepths )
{
	i32 inCount = 4;
	i32 outCount;
	q3ClipVertex in[ 8 ];
	q3ClipVertex out[ 8 ];

	// 入射面顶点变换到参考面的局部基下（q3MulT 是逆变换，相当于换坐标系）
	for ( i32 i = 0; i < 4; ++i )
		in[ i ].v = q3MulT( basis, incident[ i ].v - rPos );

	// 依次用 +x、+y、-x、-y 四条边裁剪（对应参考面的 4 条侧边）
	outCount = q3Orthographic( r32( 1.0 ), e.x, 0, clipEdges[ 0 ], in, inCount, out );

	if ( !outCount )
		return 0;

	inCount = q3Orthographic( r32( 1.0 ), e.y, 1, clipEdges[ 1 ], out, outCount, in );

	if ( !inCount )
		return 0;

	outCount = q3Orthographic( r32( -1.0 ), e.x, 0, clipEdges[ 2 ], in, inCount, out );

	if ( !outCount )
		return 0;

	inCount = q3Orthographic( r32( -1.0 ), e.y, 1, clipEdges[ 3 ], out, outCount, in );

	// 只保留在参考面「背面」的点（z ≤ e.z），z 坐标差就是穿透深度
	outCount = 0;
	for ( i32 i = 0; i < inCount; ++i )
	{
		r32 d = in[ i ].v.z - e.z;

		if ( d <= r32( 0.0 ) )
		{
			// 变回世界空间，并记录该接触点的穿透深度
			outVerts[ outCount ].v = q3Mul( basis, in[ i ].v ) + rPos;
			outVerts[ outCount ].f = in[ i ].f;
			outDepths[ outCount++ ] = d;
		}
	}

	assert( outCount <= 8 );

	return outCount;
}

//--------------------------------------------------------------------------------------------------
// 求两条空间线段（PA→QA 与 PB→QB）之间的最近点对（closest points），
// 用于边-边接触时生成接触点。
// 数学推导：把两线段参数化 P(s)=PA+DA·s、Q(t)=PB+DB·t，
//   最小化距离平方 |P(s)-Q(t)|²，对 s、t 分别求偏导并令其为 0，得到 2×2 线性方程组：
//     [ a  -b ] [s]   [-c]
//     [ b  -e ] [t] = [-f]
//   其中 a=|DA|², b=DA·DB, e=|DB|², c=DA·r, f=DB·r, r=PA-PB。
//   解出 s(=TA)、t(=TB) 后代回，即得最近点 CA、CB。
inline void q3EdgesContact( q3Vec3 *CA, q3Vec3 *CB, const q3Vec3& PA, const q3Vec3& QA, const q3Vec3& PB, const q3Vec3& QB )
{
	q3Vec3 DA = QA - PA;   // A 线段方向
	q3Vec3 DB = QB - PB;   // B 线段方向
	q3Vec3 r = PA - PB;    // 两起点之差
	r32 a = q3Dot( DA, DA );   // |DA|²
	r32 e = q3Dot( DB, DB );   // |DB|²
	r32 f = q3Dot( DB, r );    // DB·r
	r32 c = q3Dot( DA, r );    // DA·r

	r32 b = q3Dot( DA, DB );   // DA·DB
	r32 denom = a * e - b * b; // 系数矩阵行列式

	// 解 2×2 线性方程组得到两线段参数 TA、TB
	r32 TA = (b * f - c * e) / denom;
	r32 TB = (b * TA + f) / e;

	// 代回参数式得到最近点
	*CA = PA + DA * TA;
	*CB = PB + DB * TB;
}

//--------------------------------------------------------------------------------------------------
// 求盒体在方向 n 下的「支撑边」（support edge）：沿 n 方向投影最远的那条边。
// 用于边-边接触：两个盒体各取一条支撑边，再求两边的最近点作为接触点。
// 原理：n 分量绝对值最大的两个轴决定支撑点位置（取 ±e 的最远侧），
//       分量最小的轴形成边的方向（边的两个端点在该轴取 ±e）。
void q3SupportEdge( const q3Transform& tx, const q3Vec3& e, q3Vec3 n, q3Vec3* aOut, q3Vec3* bOut )
{
	// 法线变换到盒体局部空间，方便在 AABB 上做支撑计算
	n = q3MulT( tx.rotation, n );
	q3Vec3 absN = q3Abs( n );
	q3Vec3 a, b;

	// x > y
	if ( absN.x > absN.y )
	{
		// x > y > z
		if ( absN.y > absN.z )
		{
			a.Set( e.x, e.y, e.z );
			b.Set( e.x, e.y, -e.z );
		}

		// x > z > y || z > x > y
		else
		{
			a.Set( e.x, e.y, e.z );
			b.Set( e.x, -e.y, e.z );
		}
	}

	// y > x
	else
	{
		// y > x > z
		if ( absN.x > absN.z )
		{
			a.Set( e.x, e.y, e.z );
			b.Set( e.x, e.y, -e.z );
		}

		// z > y > x || y > z > x
		else
		{
			a.Set( e.x, e.y, e.z );
			b.Set( -e.x, e.y, e.z );
		}
	}

	// 按 n 的符号翻折：支撑边应落在 n 指向的那一侧
	r32 signx = q3Sign( n.x );
	r32 signy = q3Sign( n.y );
	r32 signz = q3Sign( n.z );

	a.x *= signx;
	a.y *= signy;
	a.z *= signz;
	b.x *= signx;
	b.y *= signy;
	b.z *= signz;

	// 变换回世界空间
	*aOut = q3Mul( tx, a );
	*bOut = q3Mul( tx, b );
}

//--------------------------------------------------------------------------------------------------
// Resources:
// http://www.randygaul.net/2014/05/22/deriving-obb-to-obb-intersection-sat/
// https://box2d.googlecode.com/files/GDC2007_ErinCatto.zip
// https://box2d.googlecode.com/files/Box2D_Lite.zip
//==============================================================================
// OBB vs OBB 碰撞检测主函数：检测两盒是否相交，相交则生成碰撞流形（manifold）。
//
// 三步流程：
//   1. SAT：检查 15 条候选轴（6 面轴 + 9 边轴），找「最小穿透轴」→ 法线 + 穿透深度
//   2. 若最小穿透轴是面轴（axis < 6）→ 面接触，用 Sutherland-Hodgman 裁剪生成多个接触点
//   3. 若最小穿透轴是边轴（axis >= 6）→ 边接触，求两支撑边最近点，生成 1 个接触点
//
// 参考资料：
//   http://www.randygaul.net/2014/05/22/deriving-obb-to-obb-intersection-sat/
//   https://box2d.googlecode.com/files/GDC2007_ErinCatto.zip
//==============================================================================
void q3BoxtoBox( q3Manifold* m, q3Box* a, q3Box* b )
{
	// 两个盒体的世界变换 = 所属刚体的世界变换 × 盒体的局部变换
	q3Transform atx = a->body->GetTransform( );
	q3Transform btx = b->body->GetTransform( );
	q3Transform aL = a->local;
	q3Transform bL = b->local;
	atx = q3Mul( atx, aL );			// a 的 localToWorld 矩阵
	btx = q3Mul( btx, bL );
	q3Vec3 eA = a->e;   // A 的半长
	q3Vec3 eB = b->e;   // B 的半长

	// C * x 的含义： 把方向 x 从 B 的局部坐标 变化到 A 的局部坐标内
	// 	. 先应用 b 的旋转， 再应用 a 的旋转的逆
	// C：B 的坐标系在 A 的坐标系下的旋转矩阵（相对旋转）
	// B's frame in A's space
	q3Mat3 C = q3Transpose( atx.rotation ) * btx.rotation;

	// absC：C 的逐元素绝对值，用于 SAT 里计算投影半径（半径与朝向符号无关）
	q3Mat3 absC;
	bool parallel = false;
	const r32 kCosTol = r32( 1.0e-6 );
	for ( i32 i = 0; i < 3; ++i )
	{
		for ( i32 j = 0; j < 3; ++j )
		{
			r32 val = q3Abs( C[ i ][ j ] );
			absC[ i ][ j ] = val;

			// 若某元素接近 ±1，说明两盒有轴平行（夹角余弦 ≈ 1）
			if ( val + kCosTol >= r32( 1.0 ) )
				parallel = true;
		}
	}

	// t：从 A(中心) 指向 B(中心) 的向量，表示在 A 的局部空间
	// Vector from center A to center B in A's space
	q3Vec3 t = q3MulT( atx.rotation, btx.position - atx.position );

	// Query states
	r32 s;
	r32 aMax = -Q3_R32_MAX;
	r32 bMax = -Q3_R32_MAX;
	r32 eMax = -Q3_R32_MAX;
	i32 aAxis = ~0;
	i32 bAxis = ~0;
	i32 eAxis = ~0;
	q3Vec3 nA;
	q3Vec3 nB;
	q3Vec3 nE;

	// ---- SAT 面轴检测（6 条）----
	// 对 A 的 3 条主轴和 B 的 3 条主轴，检查两盒投影是否分离。
	// 重叠量 s = |中心距在该轴的分量| - (两盒在该轴的投影半径之和)
	//          = |t·axis| - (eA + |axis·eB|)
	// s > 0 表示投影有间隙 → 分离轴成立，两盒不相交，直接返回。
	// Face axis checks

	// a's x axis
	s = q3Abs( t.x ) - (eA.x + q3Dot( absC.Column0( ), eB ));		// 重叠量 = 中心点距离 - (a.半长 + b.半长)
	if ( q3TrackFaceAxis( &aAxis, 0, s, &aMax, atx.rotation.ex, &nA ) )
		return;

	// a's y axis
	s = q3Abs( t.y ) - (eA.y + q3Dot( absC.Column1( ), eB ));
	if ( q3TrackFaceAxis( &aAxis, 1, s, &aMax, atx.rotation.ey, &nA ) )
		return;

	// a's z axis
	s = q3Abs( t.z ) - (eA.z + q3Dot( absC.Column2( ), eB ));
	if ( q3TrackFaceAxis( &aAxis, 2, s, &aMax, atx.rotation.ez, &nA ) )
		return;

	// b's x axis
	s = q3Abs( q3Dot( t, C.ex ) ) - (eB.x + q3Dot( absC.ex, eA ));
	if ( q3TrackFaceAxis( &bAxis, 3, s, &bMax, btx.rotation.ex, &nB ) )
		return;

	// b's y axis
	s = q3Abs( q3Dot( t, C.ey ) ) - (eB.y + q3Dot( absC.ey, eA ));
	if ( q3TrackFaceAxis( &bAxis, 4, s, &bMax, btx.rotation.ey, &nB ) )
		return;

	// b's z axis
	s = q3Abs( q3Dot( t, C.ez ) ) - (eB.z + q3Dot( absC.ez, eA ));
	if ( q3TrackFaceAxis( &bAxis, 5, s, &bMax, btx.rotation.ez, &nB ) )
		return;

	// ---- SAT 边轴检测（9 条）----
	// 边轴 = A 的一条边 × B 的一条边的叉积方向。
	// 仅在两盒任意一对轴都不平行时才需要检查（若平行，边轴会退化成面轴，已检查过）。
	if ( !parallel )
	{
		// Edge axis checks
		r32 rA;
		r32 rB;

		// Cross( a.x, b.x )
		rA = eA.y * absC[ 0 ][ 2 ] + eA.z * absC[ 0 ][ 1 ];
		rB = eB.y * absC[ 2 ][ 0 ] + eB.z * absC[ 1 ][ 0 ];
		s = q3Abs( t.z * C[ 0 ][ 1 ] - t.y * C[ 0 ][ 2 ] ) - (rA + rB);
		if ( q3TrackEdgeAxis( &eAxis, 6, s, &eMax, q3Vec3( r32( 0.0 ), -C[ 0 ][ 2 ], C[ 0 ][ 1 ] ), &nE ) )
			return;

		// Cross( a.x, b.y )
		rA = eA.y * absC[ 1 ][ 2 ] + eA.z * absC[ 1 ][ 1 ];
		rB = eB.x * absC[ 2 ][ 0 ] + eB.z * absC[ 0 ][ 0 ];
		s = q3Abs( t.z * C[ 1 ][ 1 ] - t.y * C[ 1 ][ 2 ] ) - (rA + rB);
		if ( q3TrackEdgeAxis( &eAxis, 7, s, &eMax, q3Vec3( r32( 0.0 ), -C[ 1 ][ 2 ], C[ 1 ][ 1 ] ), &nE ) )
			return;

		// Cross( a.x, b.z )
		rA = eA.y * absC[ 2 ][ 2 ] + eA.z * absC[ 2 ][ 1 ];
		rB = eB.x * absC[ 1 ][ 0 ] + eB.y * absC[ 0 ][ 0 ];
		s = q3Abs( t.z * C[ 2 ][ 1 ] - t.y * C[ 2 ][ 2 ] ) - (rA + rB);
		if ( q3TrackEdgeAxis( &eAxis, 8, s, &eMax, q3Vec3( r32( 0.0 ), -C[ 2 ][ 2 ], C[ 2 ][ 1 ] ), &nE ) )
			return;

		// Cross( a.y, b.x )
		rA = eA.x * absC[ 0 ][ 2 ] + eA.z * absC[ 0 ][ 0 ];
		rB = eB.y * absC[ 2 ][ 1 ] + eB.z * absC[ 1 ][ 1 ];
		s = q3Abs( t.x * C[ 0 ][ 2 ] - t.z * C[ 0 ][ 0 ] ) - (rA + rB);
		if ( q3TrackEdgeAxis( &eAxis, 9, s, &eMax, q3Vec3( C[ 0 ][ 2 ], r32( 0.0 ), -C[ 0 ][ 0 ] ), &nE ) )
			return;

		// Cross( a.y, b.y )
		rA = eA.x * absC[ 1 ][ 2 ] + eA.z * absC[ 1 ][ 0 ];
		rB = eB.x * absC[ 2 ][ 1 ] + eB.z * absC[ 0 ][ 1 ];
		s = q3Abs( t.x * C[ 1 ][ 2 ] - t.z * C[ 1 ][ 0 ] ) - (rA + rB);
		if ( q3TrackEdgeAxis( &eAxis, 10, s, &eMax, q3Vec3( C[ 1 ][ 2 ], r32( 0.0 ), -C[ 1 ][ 0 ] ), &nE ) )
			return;

		// Cross( a.y, b.z )
		rA = eA.x * absC[ 2 ][ 2 ] + eA.z * absC[ 2 ][ 0 ];
		rB = eB.x * absC[ 1 ][ 1 ] + eB.y * absC[ 0 ][ 1 ];
		s = q3Abs( t.x * C[ 2 ][ 2 ] - t.z * C[ 2 ][ 0 ] ) - (rA + rB);
		if ( q3TrackEdgeAxis( &eAxis, 11, s, &eMax, q3Vec3( C[ 2 ][ 2 ], r32( 0.0 ), -C[ 2 ][ 0 ] ), &nE ) )
			return;

		// Cross( a.z, b.x )
		rA = eA.x * absC[ 0 ][ 1 ] + eA.y * absC[ 0 ][ 0 ];
		rB = eB.y * absC[ 2 ][ 2 ] + eB.z * absC[ 1 ][ 2 ];
		s = q3Abs( t.y * C[ 0 ][ 0 ] - t.x * C[ 0 ][ 1 ] ) - (rA + rB);
		if ( q3TrackEdgeAxis( &eAxis, 12, s, &eMax, q3Vec3( -C[ 0 ][ 1 ], C[ 0 ][ 0 ], r32( 0.0 ) ), &nE ) )
			return;

		// Cross( a.z, b.y )
		rA = eA.x * absC[ 1 ][ 1 ] + eA.y * absC[ 1 ][ 0 ];
		rB = eB.x * absC[ 2 ][ 2 ] + eB.z * absC[ 0 ][ 2 ];
		s = q3Abs( t.y * C[ 1 ][ 0 ] - t.x * C[ 1 ][ 1 ] ) - (rA + rB);
		if ( q3TrackEdgeAxis( &eAxis, 13, s, &eMax, q3Vec3( -C[ 1 ][ 1 ], C[ 1 ][ 0 ], r32( 0.0 ) ), &nE ) )
			return;

		// Cross( a.z, b.z )
		rA = eA.x * absC[ 2 ][ 1 ] + eA.y * absC[ 2 ][ 0 ];
		rB = eB.x * absC[ 1 ][ 2 ] + eB.y * absC[ 0 ][ 2 ];
		s = q3Abs( t.y * C[ 2 ][ 0 ] - t.x * C[ 2 ][ 1 ] ) - (rA + rB);
		if ( q3TrackEdgeAxis( &eAxis, 14, s, &eMax, q3Vec3( -C[ 2 ][ 1 ], C[ 2 ][ 0 ], r32( 0.0 ) ), &nE ) )
			return;
	}

	// ---- 在所有重叠轴中选「最小穿透轴」作为碰撞法线 ----
	// 给边轴加一个偏置（kRelTol=0.95）：只有边轴穿透明显比面轴浅时才选边轴。
	// 这是为了「帧间一致性」（frame coherence），避免面接触/边接触来回抖动。
	// Artificial axis bias to improve frame coherence
	const r32 kRelTol = r32( 0.95 );
	const r32 kAbsTol = r32( 0.01 );
	i32 axis;
	r32 sMax;
	q3Vec3 n;
	r32 faceMax = q3Max( aMax, bMax );
	if ( kRelTol * eMax > faceMax + kAbsTol )
	{
		axis = eAxis;
		sMax = eMax;
		n = nE;
	}

	else
	{
		if ( kRelTol * bMax > aMax + kAbsTol )
		{
			axis = bAxis;
			sMax = bMax;
			n = nB;
		}

		else
		{
			axis = aAxis;
			sMax = aMax;
			n = nA;
		}
	}

	// 确保法线从 A 指向 B
	if ( q3Dot( n, btx.position - atx.position ) < r32( 0.0 ) )
		n = -n;

	if ( axis == ~0 )
		return;

	// ---- 情形 1：面接触（最小穿透轴是某条面轴，axis 0~5）----
	// axis 0~2 是 A 的面，3~5 是 B 的面。确定「参考面」和「入射面」，
	// 用 Sutherland-Hodgman 裁剪生成落在参考面范围内的多个接触点。
	if ( axis < 6 )
	{
		q3Transform rtx;   // 参考盒的世界变换
		q3Transform itx;   // 入射盒的世界变换
		q3Vec3 eR;         // 参考盒半长
		q3Vec3 eI;         // 入射盒半长
		bool flip;         // 是否交换了 A/B（法线方向需要相应翻转）

		if ( axis < 3 )
		{
			rtx = atx;
			itx = btx;
			eR = eA;
			eI = eB;
			flip = false;
		}

		else
		{
			rtx = btx;
			itx = atx;
			eR = eB;
			eI = eA;
			flip = true;
			n = -n;
		}

		// 计算入射面（4 顶点）和参考面的裁剪边/基
		// Compute reference and incident edge information necessary for clipping
		q3ClipVertex incident[ 4 ];
		q3ComputeIncidentFace( itx, eI, n, incident );
		u8 clipEdges[ 4 ];
		q3Mat3 basis;
		q3Vec3 e;
		q3ComputeReferenceEdgesAndBasis( eR, rtx, n, axis, clipEdges, &basis, &e );

		// 把入射面裁剪到参考面的侧平面内，得到接触点 + 穿透深度
		// Clip the incident face against the reference face side planes
		q3ClipVertex out[ 8 ];
		r32 depths[ 8 ];
		i32 outNum;
		outNum = q3Clip( rtx.position, e, clipEdges, basis, incident, out, depths );

		if ( outNum )
		{
			m->contactCount = outNum;
			m->normal = flip ? -n : n;

			for ( i32 i = 0; i < outNum; ++i )
			{
				q3Contact* c = m->contacts + i;

				q3FeaturePair pair = out[ i ].f;

				if ( flip )
				{
					std::swap( pair.inI, pair.inR );
					std::swap( pair.outI, pair.outR );
				}

				c->fp = out[ i ].f;
				c->position = out[ i ].v;
				c->penetration = depths[ i ];
			}
		}
	}

	// ---- 情形 2：边接触（最小穿透轴是两条边的叉积，axis 6~14）----
	// 求两个盒体的支撑边，再求两边的最近点，接触点取两最近点的中点。
	else
	{
		// 边轴法线此前是在 A 局部空间表示的，这里旋回世界空间
		n = atx.rotation * n;

		if ( q3Dot( n, btx.position - atx.position ) < r32( 0.0 ) )
			n = -n;

		// 各取一条支撑边（沿 ±n 投影最远的边）
		q3Vec3 PA, QA;
		q3Vec3 PB, QB;
		q3SupportEdge( atx, eA, n, &PA, &QA );
		q3SupportEdge( btx, eB, -n, &PB, &QB );

		// 两支撑边的最近点
		q3Vec3 CA, CB;
		q3EdgesContact( &CA, &CB, PA, QA, PB, QB );

		m->normal = n;
		m->contactCount = 1;

		// 边-边接触只有 1 个接触点：两最近点的中点，穿透深度 = sMax
		q3Contact* c = m->contacts;
		q3FeaturePair pair;
		pair.key = axis;
		c->fp = pair;
		c->penetration = sMax;
		c->position = (CA + CB) * r32( 0.5 );
	}
}
