//--------------------------------------------------------------------------------------------------
/**
@file	q3Box.h

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

#ifndef Q3BOX_H
#define Q3BOX_H

#include "../math/q3Vec3.h"
#include "../math/q3Mat3.h"
#include "../math/q3Transform.h"
#include "../debug/q3Render.h"

//--------------------------------------------------------------------------------------------------
// q3MassData
//--------------------------------------------------------------------------------------------------
struct q3MassData
{
	q3Mat3 inertia;		// 惯性张量， 抵抗旋转, 相对于 模型空间原点
	q3Vec3 center;		// 质心位置
	r32 mass;					// 质量， 抵抗平动
};

//--------------------------------------------------------------------------------------------------
// q3Box
//--------------------------------------------------------------------------------------------------
struct q3Box
{
	q3Transform local;			// 相对父刚体的位置
	q3Vec3 e; 							// 每个轴方向的 半长； extent, as in the extent of each OBB axis

	q3Box* next;						// 同一个 刚体 上的多个 box 
	class q3Body* body;			// 所属 刚体
	r32 friction;						// 摩擦系数
	r32 restitution;				// 恢复系数（弹性）
	r32 density;						// 密度
	i32 broadPhaseIndex;		// 宽相位树 中的 索引
	mutable void* userData;
	mutable bool sensor;		// 是否为 传感器

	void SetUserdata( void* data ) const;
	void* GetUserdata( ) const;
	void SetSensor( bool isSensor );

	bool TestPoint( const q3Transform& tx, const q3Vec3& p ) const;
	bool Raycast( const q3Transform& tx, q3RaycastData* raycast ) const;
	void ComputeAABB( const q3Transform& tx, q3AABB* aabb ) const;
	void ComputeMass( q3MassData* md ) const;
	void Render( const q3Transform& tx, bool awake, q3Render* render ) const;
};

//--------------------------------------------------------------------------------------------------
// q3BoxDef
//--------------------------------------------------------------------------------------------------
class q3BoxDef
{
public:
	q3BoxDef( )
	{
		// Common default values
		m_friction = r32( 0.4 );
		m_restitution = r32( 0.2 );
		m_density = r32( 1.0 );
		m_sensor = false;
	}

	void Set( const q3Transform& tx, const q3Vec3& extents );

	void SetFriction( r32 friction );
	void SetRestitution( r32 restitution );
	void SetDensity( r32 density );
	void SetSensor( bool sensor );

private:
	q3Transform m_tx;
	q3Vec3 m_e;

	r32 m_friction;
	r32 m_restitution;
	r32 m_density;
	bool m_sensor;

	friend class q3Body;
};

#include "q3Box.inl"


#endif // Q3BOX_H
