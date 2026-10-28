//--------------------------------------------------------------------------------------------------
/**
@file	q3DynamicAABBTree.h

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

#ifndef Q3DYNAMICAABBTREE_H
#define Q3DYNAMICAABBTREE_H

#include "../math/q3Math.h"
#include "../common/q3Geometry.h"

//--------------------------------------------------------------------------------------------------
// q3DynamicAABBTree
//--------------------------------------------------------------------------------------------------
// Resources:
// http://box2d.org/2014/08/balancing-dynamic-trees/
// http://www.randygaul.net/2013/08/06/dynamic-aabb-tree/
class q3Render;

//==============================================================================
// q3DynamicAABBTree：动态 AABB 树（一种「包围盒层次」结构，BVH）。
// 它是宽相位的核心数据结构：每个「叶节点」存一个形状的包围盒（AABB），
// 每个「分支节点」的 AABB 恰好包住它的两个孩子。这样查询某个 AABB 与哪些形状
// 相交时，只需从根向下剪枝：不重叠的分支整棵跳过，无需逐个形状比较。
//
// 「动态」体现在：物体移动时叶节点的 AABB 会更新（update），树结构会随之调整；
// 叶节点存的是「膨胀过的」AABB（fat AABB），这样物体小幅移动时无需每帧重建树。
// 节点用数组（m_nodes）+ 空闲链表（free list）管理，避免频繁 new/delete。
//
// 参考：Box2D 的 b2DynamicTree。
//==============================================================================
class q3DynamicAABBTree
{
public:
	q3DynamicAABBTree( );
	~q3DynamicAABBTree( );

	// Provide tight-AABB
	// 插入一个形状（传入紧致的 AABB，内部会膨胀成 fat AABB），返回节点索引 id。
	i32 Insert( const q3AABB& aabb, void *userData );

	// 删除指定节点。
	void Remove( i32 id );

	// 更新节点的 AABB（物体移动后调用）。若新 AABB 仍被旧 fat AABB 包含则无需重建。
	bool Update( i32 id, const q3AABB& aabb );

	// 取节点存的用户数据（叶节点才有效）。
	void *GetUserData( i32 id ) const;

	// 取节点的 fat AABB。
	const q3AABB& GetFatAABB( i32 id ) const;

	// 调试渲染树的所有 AABB。
	void Render( q3Render *render ) const;

	// 查询与给定 AABB 重叠的所有叶节点，逐个回调 cb->TreeCallBack。
	template <typename T>
	void Query( T *cb, const q3AABB& aabb ) const;

	// 查询与给定射线相交的所有叶节点（用于 raycast）。
	template <typename T>
	void Query( T *cb, q3RaycastData& rayCast ) const;

	// For testing
	// 校验内部结构（空闲链表 + 树结构），供调试/测试用。
	void Validate( ) const;

private:
	struct Node
	{
		bool IsLeaf( void ) const
		{
			// The right leaf does not use the same memory as the userdata,
			// and will always be Null (no children)
			// 叶节点的 right 恒为 Null（没有孩子），据此判断是否叶节点。
			return right == Null;
		}

		// Fat AABB for leafs, bounding AABB for branches
		// 叶节点存「膨胀后的 AABB」，分支节点存「包住两个孩子的 AABB」。
		q3AABB aabb;

		// 叶节点用 parent（父节点索引），空闲节点用 next（空闲链表的下一个）。
		union
		{
			i32 parent;
			i32 next; // free list
		};

		// Child indices
		// 左右孩子索引
		struct
		{
			i32 left;
			i32 right;
		};

		// Since only leaf nodes hold userdata, we can use the
		// same memory used for left/right indices to store
		// the userdata void pointer
		// 只有叶节点存用户数据（形状指针）。分支节点的 userData 未用。
		void *userData;

		// leaf = 0, free nodes = -1
		// 高度：叶节点 = 0，空闲节点 = -1。
		i32 height;

		static const i32 Null = -1;   // 空索引哨兵
	};

	inline i32 AllocateNode( );        // 从空闲链表取一个节点（不够则扩容）
	inline void DeallocateNode( i32 index );   // 把节点归还空闲链表
	i32 Balance( i32 index );          // 旋转平衡（AVL 式），保持树较平衡
	void InsertLeaf( i32 index );      // 按表面积启发（SAH）插入叶节点
	void RemoveLeaf( i32 index );      // 移除叶节点并让兄弟顶上
	void ValidateStructure( i32 index ) const;   // 递归校验树结构
	void RenderNode( q3Render *render, i32 index ) const;   // 递归渲染

	// Correct AABB hierarchy heights and AABBs starting at supplied
	// index traversing up the heirarchy
	// 从给定节点向上，重新计算沿途分支的高度和 AABB。
	void SyncHeirarchy( i32 index );

	// Insert nodes at a given index until m_capacity into the free list
	// 把从 index 到 m_capacity 的节点加入空闲链表。
	void AddToFreeList( i32 index );

	i32 m_root;         // 根节点索引
	Node *m_nodes;      // 节点数组
	i32 m_count;	// 当前活跃（被占用）节点数量
	i32 m_capacity;	// 节点数组容量
	i32 m_freeList;     // 空闲链表头
};

#include "q3DynamicAABBTree.inl"

#endif // Q3DYNAMICAABBTREE_H
