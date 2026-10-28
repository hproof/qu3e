//--------------------------------------------------------------------------------------------------
/**
@file	q3Body.h

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

#ifndef Q3BODY_H
#define Q3BODY_H

#include <stdio.h>

#include "../math/q3Math.h"
#include "../math/q3Transform.h"

//--------------------------------------------------------------------------------------------------
// q3Body
//--------------------------------------------------------------------------------------------------
class q3Scene;
struct q3BodyDef;
class q3BoxDef;
struct q3ContactEdge;
class q3Render;
struct q3Box;

//==============================================================================
// q3BodyType：刚体类型。决定一个刚体在物理世界中如何参与运动与碰撞。
//   - 静态刚体（eStaticBody）    ：永远不动、不积分，但会被当作无限质量处理。
//     典型用途：地面、墙壁等不会移动的关卡几何。
//   - 动态刚体（eDynamicBody）   ：正常受力和积分，是「会飞会弹」的物体。
//     典型用途：玩家、箱子、球等一切可动的物体。
//   - 运动学刚体（eKinematicBody）：质量无限（不受力、不响应碰撞反弹），
//     但可以主动移动/旋转（常用于移动平台、电梯、传送带等由代码控制的物体）。
//==============================================================================
enum q3BodyType
{
	eStaticBody,
	eDynamicBody,
	eKinematicBody
};

//==============================================================================
// q3Body：刚体（rigid body）。
// 刚体是物理引擎的核心对象，代表一个「不会变形」的物体：无论受多大的力，
// 它内部任意两点间的距离都不变。因此整个物体的运动只需 6 个自由度来描述：
//   位置(3) + 朝向(3)。
//
// 一个刚体可以挂多个碰撞形状（本引擎里是 q3Box），这些形状整体一起运动、
// 共享同一份质量属性（质量、质心、惯性张量）。
//
// 职责边界：q3Body 只保存「运动状态」（位置/朝向/线速度/角速度）和「质量属性」，
// 并向外提供施加力/冲量、坐标变换等接口。真正的数值积分和碰撞约束求解
// 由 q3Scene / q3Island / q3ContactSolver 完成。
//
// ---- 动力学概念速查（外部作用 vs 自身状态） ----
// 外部对刚体施加的作用：
//   力(Force)    ：持续作用，累加进 m_force，在积分阶段改变线速度（F = m·a）。
//   力矩(Torque) ：持续作用，累加进 m_torque，在积分阶段改变角速度（τ = I·α）。
//   冲量(Impulse)：瞬时作用，直接修改 m_linearVelocity / m_angularVelocity，
//                  用完即弃、不存储（Δv = J/m）。
// 刚体自身携带的状态：
//   质量 m_mass / 逆质量 m_invMass：平动惯性，衡量抵抗线速度变化的能力。
//   逆惯性张量 m_invInertia*：转动惯性（3×3 矩阵，相当于旋转版的「质量」）。
//   速度 m_linearVelocity / m_angularVelocity：当前的运动状态。
//
// 注意：ApplyLinearForce 内部执行 m_force += force * m_mass，积分时再乘
// m_invMass 抵消，因此该接口传入的「力」实际被当作「加速度」处理
//（Δv = force·dt）。这就是重力（加速度）可以直接传进去的原因。
//==============================================================================
class q3Body
{
public:

	// Adds a box to this body. Boxes are all defined in local space
	// of their owning body. Boxes cannot be defined relative to one
	// another. The body will recalculate its mass values. No contacts
	// will be created until the next q3Scene::Step( ) call.
	// 给刚体添加一个盒子碰撞形状。盒子的位置/朝向都定义在「刚体的局部空间」
	// （相对刚体质点原点），盒子之间不能相互相对定义。添加后刚体会重新计算质量。
	// 新的接触要等到下一次 q3Scene::Step() 才会生成。返回指向该盒子的指针。
	const q3Box* AddBox( const q3BoxDef& def );

	// Removes this box from the body and broadphase. Forces the body
	// to recompute its mass if the body is dynamic. Frees the memory
	// pointed to by the box pointer.
	// 从刚体和宽相位（broadphase）中移除该盒子，若刚体是动态的会重新计算质量，
	// 并释放该盒子指针指向的内存。
	void RemoveBox( const q3Box* box );

	// Removes all boxes from this body and the broadphase.
	// 移除刚体上的所有盒子，并从宽相位中移除。
	void RemoveAllBoxes( );

	// 施加「力」。力会在接下来的积分中被累加，持续影响速度（F = m·a）。
	// 作用点默认在质心，因此只改变线速度、不产生旋转。
	void ApplyLinearForce( const q3Vec3& force );

	// 在世界空间某点施加力。因为力不经过质心，会额外产生力矩，让物体旋转。
	// 参数均为世界空间坐标。
	void ApplyForceAtWorldPoint( const q3Vec3& force, const q3Vec3& point );

	// 施加「冲量」。冲量是力的时间积分，作用是瞬时改变动量，直接修改速度。
	// 常用于模拟瞬间的撞击、跳跃、枪械后坐力等。
	void ApplyLinearImpulse( const q3Vec3& impulse );

	// 在世界空间某点施加冲量。同样因为不经过质心，会同时改变线速度和角速度。
	void ApplyLinearImpulseAtWorldPoint( const q3Vec3& impulse, const q3Vec3& point );

	// 施加「力矩」。力矩是「力 × 力臂」，只改变角速度（旋转），不改变线速度。
	void ApplyTorque( const q3Vec3& torque );

	// 唤醒刚体（让它重新参与物理求解）。
	void SetToAwake( );

	// 让刚体休眠（静止不动，停止求解以节省 CPU）。
	void SetToSleep( );

	// 是否处于唤醒状态。
	bool IsAwake( ) const;

	// 重力缩放因子：乘以全局重力后才是这个刚体实际受到的重力。
	// 设为 0 表示不受重力（如悬空物体），设为负值可模拟「反重力」。
	r32 GetGravityScale( ) const;
	void SetGravityScale( r32 scale );

	// 坐标变换（局部 ↔ 世界）。
	// 点（point）：既受旋转又受平移影响，用于位置坐标。
	// 向量（vector）：只受旋转影响、不受平移影响，用于方向/速度等。
	const q3Vec3 GetLocalPoint( const q3Vec3& p ) const;   // 世界 → 局部
	const q3Vec3 GetLocalVector( const q3Vec3& v ) const;  // 世界 → 局部
	const q3Vec3 GetWorldPoint( const q3Vec3& p ) const;   // 局部 → 世界
	const q3Vec3 GetWorldVector( const q3Vec3& v ) const;  // 局部 → 世界

	// 线速度（质心平移的速度，单位：长度/秒）。
	const q3Vec3 GetLinearVelocity( ) const;

	// 世界空间某点处的速度 = 质心线速度 + 角速度产生的切向速度（w × r）。
	// 用于了解物体表面上某一点此刻的瞬时速度。
	const q3Vec3 GetVelocityAtWorldPoint( const q3Vec3& p ) const;

	void SetLinearVelocity( const q3Vec3& v );

	// 角速度（绕质心旋转的速度）。方向为旋转轴，大小为转速（弧度/秒）。
	const q3Vec3 GetAngularVelocity( ) const;
	void SetAngularVelocity( const q3Vec3 v );

	// 判断两个刚体是否可能发生碰撞（还需满足：至少一方是动态的、碰撞层有交集）。
	bool CanCollide( const q3Body *other ) const;

	// 世界变换（位置 + 旋转矩阵）。
	const q3Transform GetTransform( ) const;

	i32 GetFlags( ) const;

	// 碰撞层（layers）：用位掩码表示。两个刚体只有在「至少有一位同时置 1」时才能碰撞。
	// 例：层 1 = 玩家，层 2 = 敌人，层 4 = 地形。给玩家设 1，敌人设 2，二者不相交则不碰撞。
	void SetLayers( i32 layers );
	i32 GetLayers( ) const;

	// 朝向四元数。四元数是一种表示 3D 旋转的数学工具，可以避免欧拉角的「万向锁」问题。
	const q3Quaternion GetQuaternion( ) const;

	// 用户自定义数据（如指向游戏对象、实体 ID 的指针），物理引擎不解释其内容。
	void* GetUserData( ) const;

	// 线/角阻尼：模拟空气阻力等，让速度随时间衰减。值越大衰减越快。
	void SetLinearDamping( r32 damping );
	r32 GetLinearDamping( r32 damping ) const;
	void SetAngularDamping( r32 damping );
	r32 GetAngularDamping( r32 damping ) const;

	// Manipulating the transformation of a body manually will result in
	// non-physical behavior. Contacts are updated upon the next call to
	// q3Scene::Step( ). Parameters are in world space. All body types
	// can be updated.
	// 手动设置刚体的世界变换（瞬移）。手动移动物体会产生非物理行为（不会产生
	// 正确的速度/冲量），接触会在下一次 q3Scene::Step() 时更新。参数为世界空间。
	// 所有类型的刚体都可调用。
	void SetTransform( const q3Vec3& position );
	void SetTransform( const q3Vec3& position, const q3Vec3& axis, r32 angle );

	// Used for debug rendering lines, triangles and basic lighting
	// 调试渲染：画出刚体的线框、三角形与基础光照。
	void Render( q3Render* render ) const;

	// Dump this rigid body and its shapes into a log file. The log can be
	// used as C++ code to re-create an initial scene setup.
	// 把该刚体及其形状导出为日志文件。导出的日志是可直接编译的 C++ 代码，
	// 可用于重现一个初始场景。
	void Dump( FILE* file, i32 index ) const;

	// 质量（千克）与质量的倒数。用倒数是因为物理公式里常需要除以质量，
	// 预先存 1/m 可把除法变成乘法，且静态/运动学刚体用 0 表示「无限质量」。
	r32 GetMass( ) const;
	r32 GetInvMass( ) const;
private:
	// m_flags
	enum
	{
		eAwake		= 0x001,  // 唤醒中（参与求解）
		eActive		= 0x002,  // 活跃（存在于场景、参与碰撞检测）
		eAllowSleep	= 0x004,  // 允许休眠
		eIsland		= 0x010,  // 已加入岛屿（岛屿 = 由接触相连的刚体集合）
		eStatic		= 0x020,  // 静态刚体
		eDynamic	= 0x040,  // 动态刚体
		eKinematic	= 0x080,  // 运动学刚体
		eLockAxisX	= 0x100,  // 锁定绕 X 轴旋转
		eLockAxisY	= 0x200,  // 锁定绕 Y 轴旋转
		eLockAxisZ	= 0x400,  // 锁定绕 Z 轴旋转
	};

	// 惯性张量：描述物体「转动的惯性」，是 3x3 矩阵，相当于旋转版的「质量」。
	// 计算角加速度用 α = I⁻¹·τ，所以引擎存的是「逆惯性张量」。
	q3Mat3 m_invInertiaModel;   // 局部（模型）空间的逆惯性张量，不随旋转变化
	q3Mat3 m_invInertiaWorld;   // 世界空间的逆惯性张量，每帧由局部值旋转得到
	r32 m_mass;                 // 质量
	r32 m_invMass;              // 质量的倒数（静态/运动学刚体为 0，表示无限质量）
	q3Vec3 m_linearVelocity;    // 线速度（世界空间）
	q3Vec3 m_angularVelocity;   // 角速度（世界空间，方向=转轴，大小=转速）
	q3Vec3 m_force;             // 累计力（每帧积分后清零）
	q3Vec3 m_torque;            // 累计力矩（每帧积分后清零）
	q3Transform m_tx;           // 世界变换：position（世界位置）+ rotation（3x3 旋转矩阵）
	q3Quaternion m_q;           // 朝向四元数（旋转的另一种表示，积分用，避免万向锁）
	q3Vec3 m_localCenter;       // 质心在局部空间的位置
	q3Vec3 m_worldCenter;       // 质心在世界空间的位置（运动积分的是它，而非 m_tx.position）
	r32 m_sleepTime;            // 已保持静止的时间（用于休眠判定）
	r32 m_gravityScale;         // 重力缩放因子
	i32 m_layers;               // 碰撞层位掩码
	i32 m_flags;                // 状态标志位（见上面的枚举）

	q3Box* m_boxes;             // 碰撞形状链表头
	void *m_userData;           // 用户自定义数据
	q3Scene* m_scene;           // 所属场景
	q3Body* m_next;             // 场景刚体链表：下一个
	q3Body* m_prev;             // 场景刚体链表：上一个
	i32 m_islandIndex;          // 在岛屿刚体数组中的索引（求解时用）

	r32 m_linearDamping;        // 线阻尼
	r32 m_angularDamping;       // 角阻尼

	q3ContactEdge* m_contactList;   // 与其它刚体的接触边链表头

	friend class q3Scene;
	friend struct q3Manifold;
	friend class q3ContactManager;
	friend struct q3Island;
	friend struct q3ContactSolver;

	q3Body( const q3BodyDef& def, q3Scene* scene );

	void CalculateMassData( );    // 由形状重新计算质量、质心、惯性张量
	void SynchronizeProxies( );   // 把世界变换同步到宽相位中的包围盒（AABB）
};

//--------------------------------------------------------------------------------------------------
// q3BodyDef
//--------------------------------------------------------------------------------------------------
//==============================================================================
// q3BodyDef：创建刚体时使用的参数结构体（构造后传给 q3Scene::CreateBody）。
// 其中 position/axis/angle 一起定义了刚体的初始世界变换：
//   position = 初始位置，axis = 初始旋转轴，angle = 绕该轴旋转的角度（弧度）。
// 即初始朝向等价于「绕 axis 轴旋转 angle 弧度」。
//==============================================================================
struct q3BodyDef
{
	q3BodyDef( )
	{
		// Set all initial positions/velocties to zero
		// 所有初始位置/速度都置为零向量
		q3Identity( axis );
		angle = r32( 0.0 );
		q3Identity( position );
		q3Identity( linearVelocity );
		q3Identity( angularVelocity );

		// Usually a gravity scale of 1 is the best
		// 通常重力缩放取 1 最合适
		gravityScale = r32( 1.0 );

		// Common default values
		// 常见默认值
		bodyType = eStaticBody;
		layers = 0x000000001;
		userData = NULL;
		allowSleep = true;
		awake = true;
		active = true;
		lockAxisX = false;
		lockAxisY = false;
		lockAxisZ = false;

		linearDamping = r32( 0.0 );
		angularDamping = r32( 0.1 );
	}

	q3Vec3 axis;			// 初始世界变换：旋转轴（单位向量）。
	r32 angle;				// 初始世界变换：绕 axis 的旋转角度，单位弧度（radians）。
	q3Vec3 position;		// 初始世界变换：世界空间位置。
	q3Vec3 linearVelocity;	// 初始线速度（世界空间）。
	q3Vec3 angularVelocity;	// 初始角速度（世界空间）。
	r32 gravityScale;		// 重力缩放因子（分别作用于重力的 x/y/z 方向的便捷缩放）。
	i32 layers;				// 碰撞层位掩码，两个刚体至少有一位相同才能碰撞。
	void* userData;			// 存放应用程序自定义数据（物理引擎不解释其含义）。

	r32 linearDamping;      // 线阻尼
	r32 angularDamping;     // 角阻尼

	// Static, dynamic or kinematic. Dynamic bodies with zero mass are defaulted
	// to a mass of 1. Static bodies never move or integrate, and are very CPU
	// efficient. Static bodies have infinite mass. Kinematic bodies have
	// infinite mass, but *do* integrate and move around. Kinematic bodies do not
	// resolve any collisions.
	// 刚体类型：静态 / 动态 / 运动学。
	// - 动态刚体：若形状质量为零，会被默认成质量 1。
	// - 静态刚体：从不移动、不积分，CPU 效率极高，视为无限质量。
	// - 运动学刚体：无限质量，但会积分并移动，且不参与碰撞求解（不会被弹开）。
	q3BodyType bodyType;

	bool allowSleep;	// 允许休眠。休眠让刚体进入静止状态，可大幅降低 CPU 占用。
	bool awake;			// 初始休眠状态，true 表示唤醒。
	bool active;		// 刚体可以一开始就不活跃，只驻留内存不参与计算。
	bool lockAxisX;		// 锁定绕 X 轴旋转。
	bool lockAxisY;		// 锁定绕 Y 轴旋转。
	bool lockAxisZ;		// 锁定绕 Z 轴旋转。
};

#endif // Q3BODY_H
