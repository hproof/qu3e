//--------------------------------------------------------------------------------------------------
/**
@file	q3Body.cpp

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

#include "q3Body.h"
#include "../scene/q3Scene.h"
#include "q3Contact.h"
#include "../broadphase/q3BroadPhase.h"
#include "../collision/q3Box.h"

//--------------------------------------------------------------------------------------------------
// q3Body
//--------------------------------------------------------------------------------------------------
//==============================================================================
// 构造函数：根据 q3BodyDef 初始化刚体的运动状态、质量属性和各种标志位。
// 注意：刚体不在这里计算质量（CalculateMassData 由后续 AddBox 触发），
// 因为此时还没有任何碰撞形状。
//==============================================================================
q3Body::q3Body( const q3BodyDef& def, q3Scene* scene )
{
	m_linearVelocity = def.linearVelocity;
	m_angularVelocity = def.angularVelocity;
	q3Identity( m_force );      // 累计力清零
	q3Identity( m_torque );     // 累计力矩清零
	// 由「旋转轴 axis + 旋转角 angle」构造朝向四元数
	m_q.Set( q3Normalize( def.axis ), def.angle );
	m_tx.rotation = m_q.ToMat3( );  // 四元数转成 3x3 旋转矩阵
	m_tx.position = def.position;
	m_sleepTime = r32( 0.0 );
	m_gravityScale = def.gravityScale;
	m_layers = def.layers;
	m_userData = def.userData;
	m_scene = scene;
	m_flags = 0;
	m_linearDamping = def.linearDamping;
	m_angularDamping = def.angularDamping;

	// 根据刚体类型设置对应的标志位
	if ( def.bodyType == eDynamicBody )
		m_flags |= q3Body::eDynamic;

	else
	{
		if ( def.bodyType == eStaticBody )
		{
			m_flags |= q3Body::eStatic;
			// 静态刚体永不运动，速度和累计力/力矩全部清零
			q3Identity( m_linearVelocity );
			q3Identity( m_angularVelocity );
			q3Identity( m_force );
			q3Identity( m_torque );
		}

		else if ( def.bodyType == eKinematicBody )
			m_flags |= q3Body::eKinematic;
	}

	if ( def.allowSleep )
		m_flags |= eAllowSleep;

	if ( def.awake )
		m_flags |= eAwake;

	if ( def.active )
		m_flags |= eActive;

	if ( def.lockAxisX )
		m_flags |= eLockAxisX;

	if ( def.lockAxisY )
		m_flags |= eLockAxisY;

	if ( def.lockAxisZ )
		m_flags |= eLockAxisZ;

	m_boxes = NULL;
	m_contactList = NULL;
}

//--------------------------------------------------------------------------------------------------
const q3Box* q3Body::AddBox( const q3BoxDef& def )
{
	q3AABB aabb;
	// 从场景内存池分配一个盒子（自定义堆，避免频繁 new/delete）
	q3Box* box = (q3Box*)m_scene->m_heap.Allocate( sizeof( q3Box ) );
	box->local = def.m_tx;      // 盒子在刚体局部空间的变换
	box->e = def.m_e;           // 盒子半边长（half-extents）
	box->next = m_boxes;        // 头插法插入形状链表
	m_boxes = box;
	// 计算盒子的世界空间包围盒 AABB（用于宽相位碰撞检测）
	box->ComputeAABB( m_tx, &aabb );

	box->body = this;
	box->friction = def.m_friction;       // 摩擦系数
	box->restitution = def.m_restitution; // 弹性（恢复）系数
	box->density = def.m_density;         // 密度（×体积 = 质量）
	box->sensor = def.m_sensor;           // 是否为传感器（只检测碰撞、不产生物理响应）

	// 新增形状改变了质量分布，重新计算质量/质心/惯性张量
	CalculateMassData( );

	// 把盒子的包围盒插入宽相位，并标记场景有新形状
	m_scene->m_contactManager.m_broadphase.InsertBox( box, aabb );
	m_scene->m_newBox = true;

	return box;
}

//--------------------------------------------------------------------------------------------------
void q3Body::RemoveBox( const q3Box* box )
{
	assert( box );
	assert( box->body == this );

	q3Box* node = m_boxes;

	// 在形状链表中找到该盒子并摘除
	bool found = false;
	if ( node == box )
	{
		m_boxes = node->next;
		found = true;
	}

	else
	{
		while ( node )
		{
			if ( node->next == box )
			{
				node->next = box->next;
				found = true;
				break;
			}

			node = node->next;
		}
	}

	// This shape was not connected to this body.
	// 该形状不在此刚体上（说明调用方传错了指针）
	assert( found );

	// Remove all contacts associated with this shape
	// 移除与该形状相关的所有接触约束
	q3ContactEdge* edge = m_contactList;
	while ( edge )
	{
		q3ContactConstraint* contact = edge->constraint;
		edge = edge->next;

		q3Box* A = contact->A;
		q3Box* B = contact->B;

		if ( box == A || box == B )
			m_scene->m_contactManager.RemoveContact( contact );
	}

	// 从宽相位移除该盒子的包围盒
	m_scene->m_contactManager.m_broadphase.RemoveBox( box );

	// 形状减少，重新计算质量属性
	CalculateMassData( );

	// 归还内存
	m_scene->m_heap.Free( (void*)box );
}

//--------------------------------------------------------------------------------------------------
void q3Body::RemoveAllBoxes( )
{
	while ( m_boxes )
	{
		q3Box* next = m_boxes->next;

		m_scene->m_contactManager.m_broadphase.RemoveBox( m_boxes );
		m_scene->m_heap.Free( (void*)m_boxes );

		m_boxes = next;
	}

	// 移除该刚体相关的所有接触约束
	m_scene->m_contactManager.RemoveContactsFromBody( this );
}

//--------------------------------------------------------------------------------------------------
// 施加线力（作用点在质心，只改线速度）。
// 注意本引擎的约定：这里把传入的「力」先乘以质量存起来（m_force += force * m_mass），
// 积分时再乘 m_invMass 还原。因此传入的其实是「加速度」量纲——这也是为什么
// 重力（加速度）可以直接作为参数传给本函数。
void q3Body::ApplyLinearForce( const q3Vec3& force )
{
	m_force += force * m_mass;

	SetToAwake( );
}

//--------------------------------------------------------------------------------------------------
// 在世界空间某点施力。力不经过质心时会额外产生力矩（力矩 = 力臂 × 力），
// 力臂 = 施力点 - 质心，力矩 = 叉积(力臂, 力)。
void q3Body::ApplyForceAtWorldPoint( const q3Vec3& force, const q3Vec3& point )
{
	m_force += force * m_mass;
	m_torque += q3Cross( point - m_worldCenter, force );

	SetToAwake( );
}

//--------------------------------------------------------------------------------------------------
// 施加冲量。冲量直接改变速度：Δv = 冲量 / 质量 = 冲量 * m_invMass。
void q3Body::ApplyLinearImpulse( const q3Vec3& impulse )
{
    m_linearVelocity += impulse * m_invMass;

    SetToAwake( );
}

//--------------------------------------------------------------------------------------------------
// 在世界空间某点施加冲量。同时改变线速度和角速度：
//   线速度增量 = 冲量 / 质量
//   角速度增量 = I_world⁻¹ · (力臂 × 冲量)  （转动方程 τ·dt = I·Δw）
void q3Body::ApplyLinearImpulseAtWorldPoint( const q3Vec3& impulse, const q3Vec3& point )
{
    m_linearVelocity += impulse * m_invMass;
    m_angularVelocity += m_invInertiaWorld * q3Cross( point - m_worldCenter, impulse );

    SetToAwake( );
}

//--------------------------------------------------------------------------------------------------
// 施加力矩（只影响旋转，不影响平移）。力矩 = 力 × 力臂。
void q3Body::ApplyTorque( const q3Vec3& torque )
{
	m_torque += torque;
}

//--------------------------------------------------------------------------------------------------
// 唤醒刚体：置位 eAwake 标志，并把静止计时归零。
void q3Body::SetToAwake( )
{
	if( !(m_flags & eAwake) )
	{
		m_flags |= eAwake;
		m_sleepTime = r32( 0.0 );
	}
}

//--------------------------------------------------------------------------------------------------
// 休眠刚体：清除 eAwake，静止计时归零，并把速度/力/力矩全部清零
//（既然要睡了，运动状态一律抹平）。
void q3Body::SetToSleep( )
{
	m_flags &= ~eAwake;
	m_sleepTime = r32( 0.0 );
	q3Identity( m_linearVelocity );
	q3Identity( m_angularVelocity );
	q3Identity( m_force );
	q3Identity( m_torque );
}

//--------------------------------------------------------------------------------------------------
bool q3Body::IsAwake( ) const
{
	return m_flags & eAwake ? true : false;
}

//--------------------------------------------------------------------------------------------------
r32 q3Body::GetMass( ) const
{
    return m_mass;
}

//--------------------------------------------------------------------------------------------------
r32 q3Body::GetInvMass( ) const
{
    return m_invMass;
}

//--------------------------------------------------------------------------------------------------
r32 q3Body::GetGravityScale( ) const
{
	return m_gravityScale;
}

//--------------------------------------------------------------------------------------------------
void q3Body::SetGravityScale( r32 scale )
{
	m_gravityScale = scale;
}

//--------------------------------------------------------------------------------------------------
// 世界 → 局部（点）。q3MulT 做逆变换：先减平移、再用旋转矩阵的转置旋转。
// 由于旋转矩阵是正交矩阵，其转置就是它的逆，因此「逆旋转」用转置实现。
const q3Vec3 q3Body::GetLocalPoint( const q3Vec3& p ) const
{
	return q3MulT( m_tx, p );
}

//--------------------------------------------------------------------------------------------------
// 世界 → 局部（向量）。向量只有方向、没有位置，逆变换时只做逆旋转，不减平移。
const q3Vec3 q3Body::GetLocalVector( const q3Vec3& v ) const
{
	return q3MulT( m_tx.rotation, v );
}

//--------------------------------------------------------------------------------------------------
// 局部 → 世界（点）。q3Mul 做正变换：先旋转、再加平移。
const q3Vec3 q3Body::GetWorldPoint( const q3Vec3& p ) const
{
	return q3Mul( m_tx, p );
}

//--------------------------------------------------------------------------------------------------
// 局部 → 世界（向量）。向量只旋转、不平移。
const q3Vec3 q3Body::GetWorldVector( const q3Vec3& v ) const
{
	return q3Mul( m_tx.rotation, v );
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Body::GetLinearVelocity( ) const
{
	return m_linearVelocity;
}

//--------------------------------------------------------------------------------------------------
// 求世界空间某点的瞬时速度。刚体是旋转的，因此表面某点的速度 =
//   质心线速度 + 角速度在该点产生的切向速度。
// 切向速度 = 角速度 × 力臂 = q3Cross(w, p - 质心)。
// 这个公式在碰撞求解里非常关键（决定接触点的相对速度）。
const q3Vec3 q3Body::GetVelocityAtWorldPoint( const q3Vec3& p ) const
{
	q3Vec3 directionToPoint = p - m_worldCenter;                 // 质心到该点的力臂
	q3Vec3 relativeAngularVel = q3Cross( m_angularVelocity, directionToPoint ); // w × r

	return m_linearVelocity + relativeAngularVel;
}

//--------------------------------------------------------------------------------------------------
void q3Body::SetLinearVelocity( const q3Vec3& v )
{
	// Velocity of static bodies cannot be adjusted
	// 静态刚体的速度不可修改
	if ( m_flags & eStatic )
		assert( false );

	// 若速度非零则唤醒（保证它在下一帧参与积分）
	if ( q3Dot( v, v ) > r32( 0.0 ) )
	{
		SetToAwake( );
	}

	m_linearVelocity = v;
}

//--------------------------------------------------------------------------------------------------
const q3Vec3 q3Body::GetAngularVelocity( ) const
{
	return m_angularVelocity;
}

//--------------------------------------------------------------------------------------------------
void q3Body::SetAngularVelocity( const q3Vec3 v )
{
	// Velocity of static bodies cannot be adjusted
	// 静态刚体的速度不可修改
	if ( m_flags & eStatic )
		assert( false );

	if ( q3Dot( v, v ) > r32( 0.0 ) )
	{
		SetToAwake( );
	}

	m_angularVelocity = v;
}

//--------------------------------------------------------------------------------------------------
// 判断两个刚体是否可能碰撞。需要同时满足：
//   1. 不是同一个刚体；
//   2. 至少一方是动态刚体（两个静态/运动学刚体之间不需要碰撞）；
//   3. 碰撞层位掩码有交集（m_layers & other->m_layers 非零）。
bool q3Body::CanCollide( const q3Body *other ) const
{
	if ( this == other )
		return false;

	// Every collision must have at least one dynamic body involved
	if ( !(m_flags & eDynamic) && !(other->m_flags & eDynamic) )
		return false;

	if ( !(m_layers & other->m_layers) )
		return false;

	return true;
}

//--------------------------------------------------------------------------------------------------
const q3Transform q3Body::GetTransform( ) const
{
	return m_tx;
}

//--------------------------------------------------------------------------------------------------
// 手动设置刚体位置（等价于瞬移）。会同步更新宽相位中的包围盒。
void q3Body::SetTransform( const q3Vec3& position )
{
	m_worldCenter = position;

	SynchronizeProxies( );
}

//--------------------------------------------------------------------------------------------------
// 手动设置刚体的位置和朝向（绕 axis 旋转 angle 弧度）。
void q3Body::SetTransform( const q3Vec3& position, const q3Vec3& axis, r32 angle )
{
	m_worldCenter = position;
	m_q.Set( axis, angle );
	m_tx.rotation = m_q.ToMat3( );

	SynchronizeProxies( );
}

//--------------------------------------------------------------------------------------------------
i32 q3Body::GetFlags( ) const
{
	return m_flags;
}

//--------------------------------------------------------------------------------------------------
void q3Body::SetLayers( i32 layers )
{
	m_layers = layers;
}

//--------------------------------------------------------------------------------------------------
i32 q3Body::GetLayers( ) const
{
	return m_layers;
}

//--------------------------------------------------------------------------------------------------
const q3Quaternion q3Body::GetQuaternion( ) const
{
	return m_q;
}

//--------------------------------------------------------------------------------------------------
void* q3Body::GetUserData( ) const
{
  return m_userData;
}

//--------------------------------------------------------------------------------------------------
void q3Body::SetLinearDamping( r32 damping )
{
	m_linearDamping = damping;
}

//--------------------------------------------------------------------------------------------------
r32 q3Body::GetLinearDamping( r32 damping ) const
{
	return m_linearDamping;
}

//--------------------------------------------------------------------------------------------------
void q3Body::SetAngularDamping( r32 damping )
{
	m_angularDamping = damping;
}

//--------------------------------------------------------------------------------------------------
r32 q3Body::GetAngularDamping( r32 damping ) const
{
	return m_angularDamping;
}

//--------------------------------------------------------------------------------------------------
// 调试渲染：遍历所有形状，交给各自的 Render 画出线框。
void q3Body::Render( q3Render* render ) const
{
	bool awake = IsAwake( );
	q3Box* box = m_boxes;

	while ( box )
	{
		box->Render( m_tx, awake, render );
		box = box->next;
	}
}

//--------------------------------------------------------------------------------------------------
// 把刚体及其形状导出为 C++ 代码片段（写入日志文件），用于重现场景初始状态。
// 精确到 15 位小数以保证往返精度（%.15lf）。
void q3Body::Dump( FILE* file, i32 index ) const
{
	fprintf( file, "{\n" );
	fprintf( file, "\tq3BodyDef bd;\n" );

	// 导出刚体类型
	switch ( m_flags & (eStatic | eDynamic | eKinematic) )
	{
	case eStatic:
		fprintf( file, "\tbd.bodyType = q3BodyType( %d );\n", eStaticBody );
		break;

	case eDynamic:
		fprintf( file, "\tbd.bodyType = q3BodyType( %d );\n", eDynamicBody );
		break;

	case eKinematic:
		fprintf( file, "\tbd.bodyType = q3BodyType( %d );\n", eKinematicBody );
		break;
	}

	// 导出位置、朝向（四元数转回轴-角表示）、速度、重力缩放、碰撞层等
	fprintf( file, "\tbd.position.Set( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", m_tx.position.x, m_tx.position.y, m_tx.position.z );
	q3Vec3 axis;
	r32 angle;
	m_q.ToAxisAngle( &axis, &angle );
	fprintf( file, "\tbd.axis.Set( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", axis.x, axis.y, axis.z );
	fprintf( file, "\tbd.angle = r32( %.15lf );\n", angle );
	fprintf( file, "\tbd.linearVelocity.Set( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", m_linearVelocity.x, m_linearVelocity.y, m_linearVelocity.z );
	fprintf( file, "\tbd.angularVelocity.Set( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", m_angularVelocity.x, m_angularVelocity.y, m_angularVelocity.z );
	fprintf( file, "\tbd.gravityScale = r32( %.15lf );\n", m_gravityScale );
	fprintf( file, "\tbd.layers = %d;\n", m_layers );
	fprintf( file, "\tbd.allowSleep = bool( %d );\n", m_flags & eAllowSleep );
	fprintf( file, "\tbd.awake = bool( %d );\n", m_flags & eAwake );
	fprintf( file, "\tbd.awake = bool( %d );\n", m_flags & eAwake );
	fprintf( file, "\tbd.lockAxisX = bool( %d );\n", m_flags & eLockAxisX );
	fprintf( file, "\tbd.lockAxisY = bool( %d );\n", m_flags & eLockAxisY );
	fprintf( file, "\tbd.lockAxisZ = bool( %d );\n", m_flags & eLockAxisZ );
	fprintf( file, "\tbodies[ %d ] = scene.CreateBody( bd );\n\n", index );

	// 导出刚体上的每个形状
	q3Box* box = m_boxes;

	while ( box )
	{
		fprintf( file, "\t{\n" );
		fprintf( file, "\t\tq3BoxDef sd;\n" );
		fprintf( file, "\t\tsd.SetFriction( r32( %.15lf ) );\n", box->friction );
		fprintf( file, "\t\tsd.SetRestitution( r32( %.15lf ) );\n", box->restitution );
		fprintf( file, "\t\tsd.SetDensity( r32( %.15lf ) );\n", box->density );
		i32 sensor = (int)box->sensor;
		fprintf( file, "\t\tsd.SetSensor( bool( %d ) );\n", sensor );
		fprintf( file, "\t\tq3Transform boxTx;\n" );
		// 形状的局部变换用「三根轴 + 位置」的形式导出（旋转矩阵 → 三个基向量）
		q3Transform boxTx = box->local;
		q3Vec3 xAxis = boxTx.rotation.ex;
		q3Vec3 yAxis = boxTx.rotation.ey;
		q3Vec3 zAxis = boxTx.rotation.ez;
		fprintf( file, "\t\tq3Vec3 xAxis( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", xAxis.x, xAxis.y, xAxis.z );
		fprintf( file, "\t\tq3Vec3 yAxis( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", yAxis.x, yAxis.y, yAxis.z );
		fprintf( file, "\t\tq3Vec3 zAxis( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", zAxis.x, zAxis.y, zAxis.z );
		fprintf( file, "\t\tboxTx.rotation.SetRows( xAxis, yAxis, zAxis );\n" );
		fprintf( file, "\t\tboxTx.position.Set( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n", boxTx.position.x, boxTx.position.y, boxTx.position.z );
		// 注意：导出时 e 是半边长，Set 里传的是整边长，所以要 ×2
		fprintf( file, "\t\tsd.Set( boxTx, q3Vec3( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) ) );\n", box->e.x * 2.0f, box->e.y * 2.0f, box->e.z * 2.0f );
		fprintf( file, "\t\tbodies[ %d ]->AddBox( sd );\n", index );
		fprintf( file, "\t}\n" );
		box = box->next;
	}

	fprintf( file, "}\n\n" );
}

//--------------------------------------------------------------------------------------------------
// 重新计算刚体的质量属性：总质量、质心、惯性张量（及其逆）。
// 流程：遍历所有形状，累加各自的质量/惯性/质心加权，再用「平行轴定理」
// 把各形状的惯性张量平移到刚体质心，最后求逆得到逆惯性张量。
void q3Body::CalculateMassData( )
{
	q3Mat3 inertia = q3Diagonal( r32( 0.0 ) );   // 累加惯性张量（初始为零矩阵）
	m_invInertiaModel = q3Diagonal( r32( 0.0 ) );
	m_invInertiaWorld = q3Diagonal( r32( 0.0 ) );
	m_invMass = r32( 0.0 );
	m_mass = r32( 0.0 );
	r32 mass = r32( 0.0 );

	// 静态/运动学刚体视为无限质量（invMass=0），质心即原点，无需计算
	if ( m_flags & eStatic || m_flags &eKinematic )
	{
		q3Identity( m_localCenter );
		m_worldCenter = m_tx.position;
		return;
	}

	q3Vec3 lc;          // 质心的累加器（加权和）
	q3Identity( lc );

	// 遍历所有形状，累加质量、惯性张量、以及「质心 × 质量」的加权和
	for ( q3Box* box = m_boxes; box; box = box->next)
	{
		if ( box->density == r32( 0.0 ) )   // 密度为 0 的形状不贡献质量
			continue;

		q3MassData md;
		box->ComputeMass( &md );            // 计算该形状的质量、质心、惯性张量
		mass += md.mass;
		inertia += md.inertia;
		lc += md.center * md.mass;          // 加权质心累加（最后除以总质量得到质心）
	}

	if ( mass > r32( 0.0 ) )
	{
		m_mass = mass;
		m_invMass = r32( 1.0 ) / mass;
		lc *= m_invMass;                    // 得到局部空间质心

		// 平行轴定理（parallel axis theorem）：把各形状相对各自原点的惯性张量，
		// 平移到刚体质心。公式：I = Σ(I_i) - m·[ (lc·lc)·I - lc⊗lc ]
		// 其中 lc⊗lc 是外积（outer product，得到一个 3x3 矩阵）。
		q3Mat3 identity;
		q3Identity( identity );
		inertia -= (identity * q3Dot( lc, lc ) - q3OuterProduct( lc, lc )) * mass;

		// 惯性张量求逆，得到局部空间逆惯性张量（供角加速度计算用）
		m_invInertiaModel = q3Inverse( inertia );

		// 若锁定了某根轴的旋转，则把逆惯性张量对应行清零，
		// 使该轴上的角加速度恒为 0，即无法绕该轴旋转。
		if ( m_flags & eLockAxisX )
			q3Identity( m_invInertiaModel.ex );

		if ( m_flags & eLockAxisY )
			q3Identity( m_invInertiaModel.ey );

		if ( m_flags & eLockAxisZ )
			q3Identity( m_invInertiaModel.ez );
	}
	else
	{
		// Force all dynamic bodies to have some mass
		// 总质量为零的动态刚体，强制给一个单位质量，避免除以零
		m_invMass = r32( 1.0 );
		m_invInertiaModel = q3Diagonal( r32( 0.0 ) );
		m_invInertiaWorld = q3Diagonal( r32( 0.0 ) );
	}

	m_localCenter = lc;
	// 把局部质心变换到世界空间
	m_worldCenter = q3Mul( m_tx, lc );
}

//--------------------------------------------------------------------------------------------------
// 把刚体的世界变换同步到宽相位：更新所有形状的包围盒（AABB）。
// 关键：世界位置 m_tx.position 是由「世界质心 - 旋转后的局部质心」反推出来的，
// 因为质心才是真正参与积分的位置，而形状坐标相对的是 m_tx 原点。
void q3Body::SynchronizeProxies( )
{
	q3BroadPhase* broadphase = &m_scene->m_contactManager.m_broadphase;

	m_tx.position = m_worldCenter - q3Mul( m_tx.rotation, m_localCenter );

	q3AABB aabb;
	q3Transform tx = m_tx;

	// 逐个形状重新计算世界空间包围盒，并更新宽相位
	q3Box* box = m_boxes;
	while ( box )
	{
		box->ComputeAABB( tx, &aabb );
		broadphase->Update( box->broadPhaseIndex, aabb );
		box = box->next;
	}
}
