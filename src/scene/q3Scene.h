//--------------------------------------------------------------------------------------------------
/**
@file	q3Scene.h

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

#ifndef Q3SCENE_H
#define Q3SCENE_H

#include <stdio.h>

#include "../common/q3Settings.h"
#include "../common/q3Memory.h"
#include "../dynamics/q3ContactManager.h"

//==============================================================================
// q3Scene：物理世界（场景）的容器，也是用户操作引擎的主要入口。
// 它持有所有刚体（q3Body）的链表、接触管理器（负责宽相位碰撞检测）、
// 内存分配器，并在 Step() 里驱动整个模拟流程：
//   碰撞检测 → 岛屿（island）构建 → 约束求解 → 速度/位置积分。
// 通常一个程序只需要一个场景，用固定时间步长 dt 驱动。
//==============================================================================

//--------------------------------------------------------------------------------------------------
// q3Scene
//--------------------------------------------------------------------------------------------------
class q3Body;
struct q3BodyDef;
struct q3ContactConstraint;
class q3Render;
struct q3Island;

// 碰撞监听器：捕获两个形状开始/结束碰撞的事件，可用于游戏逻辑和音效。
// 注意：在这些回调里新创建的物理对象要到下一帧才会被报告；
// 回调可能被频繁调用，实现应尽量高效。
// This listener is used to gather information about two shapes colliding. This
// can be used for game logic and sounds. Physics objects created in these
// callbacks will not be reported until the following frame. These callbacks
// can be called frequently, so make them efficient.
class q3ContactListener
{
public:
	virtual ~q3ContactListener( )
	{
	}

	virtual void BeginContact( const q3ContactConstraint *contact ) = 0;
	virtual void EndContact( const q3ContactConstraint *contact ) = 0;
};


// 用于 point/AABB/Raycast 检测的回调
// This class represents general queries for points, AABBs and Raycasting.
// ReportShape is called the moment a valid shape is found. The return
// value of ReportShape controls whether to continue or stop the query.
// By returning only true, all shapes that fulfill the query will be re-
// ported.
class q3QueryCallback
{
public:
	virtual ~q3QueryCallback( )
	{
	}

	virtual bool ReportShape( q3Box *box ) = 0;
};

class q3Scene
{
public:
	// 构造一个物理场景。
	// dt         ：固定时间步长（秒），如 1/60。不支持变步长。
	// gravity    ：全局重力向量，默认 (0, -9.8, 0)，即 y 轴向下。
	// iterations ：约束求解的迭代次数，越大越精确但越慢，常用 5~20。
	q3Scene( r32 dt, const q3Vec3& gravity = q3Vec3( r32( 0.0 ), r32( -9.8 ), r32( 0.0 ) ), i32 iterations = 20 );
	~q3Scene( );

	// 把模拟向前推进一个固定时间步长 dt。变时间步长不被支持。
	// Run the simulation forward in time by dt (fixed timestep). Variable
	// timestep is not supported.
	void Step( );

	// Construct a new rigid body. The BodyDef can be reused at the user's
	// discretion, as no reference to the BodyDef is kept.
	q3Body* CreateBody( const q3BodyDef& def );

	// Frees a body, removes all shapes associated with the body and frees
	// all shapes and contacts associated and attached to this body.
	void RemoveBody( q3Body* body );
	void RemoveAllBodies( );

	// Enables or disables rigid body sleeping. Sleeping is an effective CPU
	// optimization where bodies are put to sleep if they don't move much.
	// Sleeping bodies sit in memory without being updated, until the are
	// touched by something that wakes them up. The default is enabled.
	void SetAllowSleep( bool allowSleep );

	// Increasing the iteration count increases the CPU cost of simulating
	// Scene.Step(). Decreasing the iterations makes the simulation less
	// realistic (convergent). A good iteration number range is 5 to 20.
	// Only positive numbers are accepted. Non-positive and negative
	// inputs set the iteration count to 1.
	void SetIterations( i32 iterations );

	// Friction occurs when two rigid bodies have shapes that slide along one
	// another. The friction force resists this sliding motion.
	void SetEnableFriction( bool enabled );

	// Render the scene with an interpolated time between the last frame and
	// the current simulation step.
	void Render( q3Render* render ) const;

	// Gets and sets the global gravity vector used during integration
	const q3Vec3 GetGravity( ) const;
	void SetGravity( const q3Vec3& gravity );

	// Removes all bodies from the scene.
	void Shutdown( );

	// Sets the listener to report collision start/end. Provides the user
	// with a pointer to an q3ContactConstraint. The q3ContactConstraint
	// holds pointers to the two shapes involved in a collision, and the
	// two bodies connected to each shape. The q3ContactListener will be
	// called very often, so it is recommended for the funciton to be very
	// efficient. Provide a NULL pointer to remove the previously set
	// listener.
	void SetContactListener( q3ContactListener* listener );

	// Query the world to find any shapes that can potentially intersect
	// the provided AABB. This works by querying the broadphase with an
	// AAABB -- only *potential* intersections are reported. Perhaps the
	// user might use lmDistance as fine-grained collision detection.
	void QueryAABB( q3QueryCallback *cb, const q3AABB& aabb ) const;

	// Query the world to find any shapes intersecting a world space point.
	void QueryPoint( q3QueryCallback *cb, const q3Vec3& point ) const;

	// Query the world to find any shapes intersecting a ray.
	void RayCast( q3QueryCallback *cb, q3RaycastData& rayCast ) const;

	// Dump all rigid bodies and shapes into a log file. The log can be
	// used as C++ code to re-create an initial scene setup. Contacts
	// are *not* logged, meaning any cached resolution solutions will
	// not be saved to the log file. This means the log file will be most
	// accurate when dumped upon scene initialization, instead of mid-
	// simulation.
	void Dump( FILE* file ) const;

private:
	q3ContactManager m_contactManager;   // 接触管理器：宽相位 + 接触约束的管理
	q3PagedAllocator m_boxAllocator;     // q3Box 对象的页式内存分配器

	i32 m_bodyCount;                     // 刚体数量
	q3Body* m_bodyList;                  // 刚体链表头（所有刚体串成链表）

	q3Stack m_stack;                     // 栈分配器：Step 里为岛屿临时分配内存（快）
	q3Heap m_heap;                       // 堆分配器：刚体等持久对象的分配

	q3Vec3 m_gravity;                    // 全局重力向量
	r32 m_dt;                            // 固定时间步长
	i32 m_iterations;                    // 约束求解迭代次数

	bool m_newBox;                       // 是否有新盒体加入（需要重建宽相位对）
	bool m_allowSleep;                   // 是否允许刚体休眠（CPU 优化）
	bool m_enableFriction;               // 是否启用摩擦力

	friend class q3Body;
};

#endif // Q3SCENE_H
