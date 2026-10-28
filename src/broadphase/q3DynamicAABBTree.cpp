//--------------------------------------------------------------------------------------------------
/**
@file	q3DynamicAABBTree.cpp

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

#include "q3DynamicAABBTree.h"
#include "../debug/q3Render.h"
#include "../common/q3Memory.h"

//--------------------------------------------------------------------------------------------------
// q3DynamicAABBTree
//--------------------------------------------------------------------------------------------------
// 把 AABB 向外膨胀一个固定量（每边 0.5）。
// 膨胀后的 fat AABB 可以容忍物体小幅移动而无需每帧重建树。
inline void FattenAABB( q3AABB& aabb )
{
	const r32 k_fattener = r32( 0.5 );
	q3Vec3 v( k_fattener, k_fattener, k_fattener );

	aabb.min -= v;
	aabb.max += v;
}

//--------------------------------------------------------------------------------------------------
// 构造：分配初始 1024 个节点，并把它们全部放入空闲链表。
q3DynamicAABBTree::q3DynamicAABBTree( )
{
	m_root = Node::Null;

	m_capacity = 1024;
	m_count = 0;
	m_nodes = (Node *)q3Alloc( sizeof( Node ) * m_capacity );

	AddToFreeList( 0 );
}

//--------------------------------------------------------------------------------------------------
q3DynamicAABBTree::~q3DynamicAABBTree( )
{
	q3Free( m_nodes );
}

//--------------------------------------------------------------------------------------------------
// 插入：分配一个叶节点，膨胀 AABB，设置用户数据和高度，然后按 SAH 插入树中。
i32 q3DynamicAABBTree::Insert( const q3AABB& aabb, void *userData )
{
	i32 id = AllocateNode( );

	// Fatten AABB and set height/userdata
	m_nodes[ id ].aabb = aabb;
	FattenAABB( m_nodes[id].aabb );
	m_nodes[ id ].userData = userData;
	m_nodes[ id ].height = 0;   // 叶节点高度为 0

	InsertLeaf( id );

	return id;
}

//--------------------------------------------------------------------------------------------------
// 删除：先摘除叶节点，再归还到空闲链表。
void q3DynamicAABBTree::Remove( i32 id )
{
	assert( id >= 0 && id < m_capacity );
	assert( m_nodes[ id ].IsLeaf( ) );

	RemoveLeaf( id );
	DeallocateNode( id );
}

//--------------------------------------------------------------------------------------------------
// 更新节点 AABB。若新 AABB 仍被旧 fat AABB 包含，说明树结构无需变化，返回 false；
// 否则摘除再重新插入（因为叶节点在树中的位置可能变了），返回 true。
bool q3DynamicAABBTree::Update( i32 id, const q3AABB& aabb )
{
	assert( id >= 0 && id < m_capacity );
	assert( m_nodes[ id ].IsLeaf( ) );

	// 新 AABB 仍被旧的 fat AABB 包住 → 无需更新
	if ( m_nodes[ id ].aabb.Contains( aabb ) )
		return false;

	RemoveLeaf( id );

	m_nodes[ id ].aabb = aabb;
	FattenAABB( m_nodes[ id ].aabb );

	InsertLeaf( id );

	return true;
}

//--------------------------------------------------------------------------------------------------
void *q3DynamicAABBTree::GetUserData( i32 id ) const
{
	assert( id >= 0 && id < m_capacity );

	return m_nodes[ id ].userData;
}

//--------------------------------------------------------------------------------------------------
const q3AABB& q3DynamicAABBTree::GetFatAABB( i32 id ) const
{
	assert( id >= 0 && id < m_capacity );

	return m_nodes[ id ].aabb;
}

//--------------------------------------------------------------------------------------------------
// 调试渲染：从根开始递归画出所有节点的 AABB 线框。
void q3DynamicAABBTree::Render( q3Render *render ) const
{
	if ( m_root != Node::Null )
	{
		render->SetPenColor( 0.5f, 0.5f, 1.0f );
		RenderNode( render, m_root );
	}
}

//--------------------------------------------------------------------------------------------------
// 画一个 AABB 的 12 条边（长方体线框），然后递归渲染两个孩子。
void q3DynamicAABBTree::RenderNode( q3Render *render, i32 index ) const
{
	assert( index >= 0 && index < m_capacity );

	Node *n = m_nodes + index;
	const q3AABB& b = n->aabb;

	// 顶面四条边
	render->SetPenPosition( b.min.x, b.max.y, b.min.z );

	render->Line( b.min.x, b.max.y, b.max.z );
	render->Line( b.max.x, b.max.y, b.max.z );
	render->Line( b.max.x, b.max.y, b.min.z );
	render->Line( b.min.x, b.max.y, b.min.z );

	// 底面四条边
	render->SetPenPosition( b.min.x, b.min.y, b.min.z );

	render->Line( b.min.x, b.min.y, b.max.z );
	render->Line( b.max.x, b.min.y, b.max.z );
	render->Line( b.max.x, b.min.y, b.min.z );
	render->Line( b.min.x, b.min.y, b.min.z );

	// 四根竖边
	render->SetPenPosition( b.min.x, b.min.y, b.min.z );
	render->Line( b.min.x, b.max.y, b.min.z );
	render->SetPenPosition( b.max.x, b.min.y, b.min.z );
	render->Line( b.max.x, b.max.y, b.min.z );
	render->SetPenPosition( b.max.x, b.min.y, b.max.z );
	render->Line( b.max.x, b.max.y, b.max.z );
	render->SetPenPosition( b.min.x, b.min.y, b.max.z );
	render->Line( b.min.x, b.max.y, b.max.z );

	// 非叶节点：递归渲染左右子树
	if ( !n->IsLeaf( ) )
	{
		RenderNode( render, n->left );
		RenderNode( render, n->right );
	}
}

//--------------------------------------------------------------------------------------------------
// 校验内部一致性：
//   1. 空闲链表里的节点数 + 活跃节点数 == 容量；
//   2. 根节点的 parent 为 Null；
//   3. （Debug 下）递归校验树结构（孩子 parent 指回父节点、叶节点高度为 0）。
void q3DynamicAABBTree::Validate( ) const
{
	// Verify free list
	// 统计空闲链表长度
	i32 freeNodes = 0;
	i32 index = m_freeList;

	while ( index != Node::Null )
	{
		assert( index >= 0 && index < m_capacity );
		index = m_nodes[ index ].next;
		++freeNodes;
	}

	assert( m_count + freeNodes == m_capacity );

	// Validate tree structure
	if ( m_root != Node::Null )
	{
		assert( m_nodes[m_root].parent == Node::Null );

#ifdef _DEBUG
		ValidateStructure( m_root );
#endif
	}
}

//--------------------------------------------------------------------------------------------------
// 递归校验树结构：叶节点 right 为 Null 且高度为 0；
// 分支节点的左右孩子 parent 都指回自己，并递归向下校验。
void q3DynamicAABBTree::ValidateStructure( i32 index ) const
{
	Node *n = m_nodes + index;

	i32 il = n->left;
	i32 ir = n->right;

	if ( n->IsLeaf( ) )
	{
		assert( ir == Node::Null );
		assert( n->height == 0 );
		return;
	}

	assert( il >= 0 && il < m_capacity );
	assert( ir >= 0 && ir < m_capacity );
	Node *l = m_nodes + il;
	Node *r = m_nodes + ir;

	assert( l->parent == index );
	assert( r->parent == index );

	ValidateStructure( il );
	ValidateStructure( ir );
}

//--------------------------------------------------------------------------------------------------
// 从空闲链表取一个节点并初始化；若链表为空则先翻倍扩容。
i32 q3DynamicAABBTree::AllocateNode( )
{
	// 空闲链表空 → 容量翻倍，拷贝旧节点，把新增段加入空闲链表
	if ( m_freeList == Node::Null )
	{
		m_capacity *= 2;
		Node *newNodes = (Node *)q3Alloc( sizeof( Node ) * m_capacity );
		memcpy( newNodes, m_nodes, sizeof( Node ) * m_count );
		q3Free( m_nodes );
		m_nodes = newNodes;

		AddToFreeList( m_count );
	}

	// 弹出空闲链表头
	i32 freeNode = m_freeList;
	m_freeList = m_nodes[m_freeList].next;
	m_nodes[ freeNode ].height = 0;
	m_nodes[ freeNode ].left = Node::Null;
	m_nodes[ freeNode ].right = Node::Null;
	m_nodes[ freeNode ].parent = Node::Null;
	m_nodes[ freeNode ].userData = NULL;
	++m_count;
	return freeNode;
}

//--------------------------------------------------------------------------------------------------
// Balance：对节点 iA 做 AVL 式旋转平衡。
// 树允许左右子树高度差最多为 1；若失衡则旋转，返回新的子树根。
// 旋转后要重新合并 AABB 并更新高度。
//
// 结构示意：
//           A
//         /   \
//        B     C
//       / \   / \
//      D   E F   G
i32 q3DynamicAABBTree::Balance( i32 iA )
{
	Node *A = m_nodes + iA;

	// 叶节点或高度为 1 的节点无需平衡
	if ( A->IsLeaf( ) || A->height == 1 )
		return iA;

	i32 iB = A->left;
	i32 iC = A->right;
	Node *B = m_nodes + iB;
	Node *C = m_nodes + iC;

	// 平衡因子 = 右子树高度 - 左子树高度
	i32 balance = C->height - B->height;

	// C is higher, promote C
	// 右子树过高（balance > 1）：左旋，提升 C
	if ( balance > 1 )
	{
		i32 iF = C->left;
		i32 iG = C->right;
		Node *F = m_nodes + iF;
		Node *G = m_nodes + iG;

		// grandParent point to C
		// 让 A 的父节点指向 C（C 顶替 A 的位置）
		if ( A->parent != Node::Null )
		{
			if(m_nodes[ A->parent ].left == iA)
				m_nodes[ A->parent ].left = iC;

			else
				m_nodes[ A->parent ].right = iC;
		}
		else
			m_root = iC;

		// Swap A and C
		// C 成为新根，A 成为 C 的左孩子
		C->left = iA;
		C->parent = A->parent;
		A->parent = iC;

		// Finish rotation
		// 根据 F、G 的高度决定单旋还是双旋
		if ( F->height > G->height )
		{
			// 双旋：F 较高，F 保留在 C 下，G 挂到 A 下
			C->right = iF;
			A->right = iG;
			G->parent = iA;
			A->aabb = q3Combine( B->aabb, G->aabb );
			C->aabb = q3Combine( A->aabb, F->aabb );

			A->height = 1 + q3Max( B->height, G->height );
			C->height = 1 + q3Max( A->height, F->height );
		}

		else
		{
			// 单旋：G 保留在 C 下，F 挂到 A 下
			C->right = iG;
			A->right = iF;
			F->parent = iA;
			A->aabb = q3Combine( B->aabb, F->aabb );
			C->aabb = q3Combine( A->aabb, G->aabb );

			A->height = 1 + q3Max( B->height, F->height );
			C->height = 1 + q3Max( A->height, G->height );
		}

		return iC;
	}

	// B is higher, promote B
	// 左子树过高（balance < -1）：右旋，提升 B
	else if ( balance < -1 )
	{
		i32 iD = B->left;
		i32 iE = B->right;
		Node *D = m_nodes + iD;
		Node *E = m_nodes + iE;

		// grandParent point to B
		// 让 A 的父节点指向 B（B 顶替 A 的位置）
		if ( A->parent != Node::Null )
		{
			if(m_nodes[ A->parent ].left == iA)
				m_nodes[ A->parent ].left = iB;
			else
				m_nodes[ A->parent ].right = iB;
		}

		else
			m_root = iB;

		// Swap A and B
		// B 成为新根，A 成为 B 的右孩子
		B->right = iA;
		B->parent = A->parent;
		A->parent = iB;

		// Finish rotation
		// 根据 D、E 的高度决定单旋还是双旋
		if ( D->height > E->height )
		{
			// 双旋：D 较高，D 保留在 B 下，E 挂到 A 下
			B->left = iD;
			A->left = iE;
			E->parent = iA;
			A->aabb = q3Combine( C->aabb, E->aabb );
			B->aabb = q3Combine( A->aabb, D->aabb );

			A->height = 1 + q3Max( C->height, E->height );
			B->height = 1 + q3Max( A->height, D->height );
		}

		else
		{
			// 单旋：E 保留在 B 下，D 挂到 A 下
			B->left = iE;
			A->left = iD;
			D->parent = iA;
			A->aabb = q3Combine( C->aabb, D->aabb );
			B->aabb = q3Combine( A->aabb, E->aabb );

			A->height = 1 + q3Max( C->height, D->height );
			B->height = 1 + q3Max( A->height, E->height );
		}

		return iB;
	}

	return iA;
}

//--------------------------------------------------------------------------------------------------
// InsertLeaf：按「表面积启发（Surface Area Heuristic, SAH）」把叶节点插入树。
// 思路：自顶向下，比较「在当前分支下新建父节点」的成本 vs 「继续下降到左/右子树」的成本，
// 选择成本最低的位置作为新叶的兄弟（sibling），最终为它们新建一个父分支节点。
// SAH 的核心直觉：让 AABB 表面积增长最小的位置就是好位置（表面积越小，后续查询剪枝越有效）。
void q3DynamicAABBTree::InsertLeaf( i32 id )
{
	// 空树：新叶直接当根
	if ( m_root == Node::Null )
	{
		m_root = id;
		m_nodes[ m_root ].parent = Node::Null;
		return;
	}

	// Search for sibling
	// 从根开始，寻找新叶的最佳兄弟节点
	i32 searchIndex = m_root;
	q3AABB leafAABB = m_nodes[ id ].aabb;
	while ( !m_nodes[ searchIndex ].IsLeaf( ) )
	{
		// Cost for insertion at index (branch node), involves creation
		// of new branch to contain this index and the new leaf
		// 在当前分支处插入的成本：需要新建一个父节点包住「该分支 + 新叶」，
		// 成本用「合并后表面积的 2 倍」估计（2 是因为新增了一个父节点层级）。
		q3AABB combined = q3Combine( leafAABB, m_nodes[ searchIndex ].aabb );
		r32 combinedArea = combined.SurfaceArea( );
		r32 branchCost = r32( 2.0 ) * combinedArea;

		// Inherited cost (surface area growth from heirarchy update after descent)
		// 继承成本：下降后沿途分支 AABB 膨胀带来的表面积增长。
		r32 inheritedCost = r32( 2.0 ) * (combinedArea - m_nodes[ searchIndex ].aabb.SurfaceArea( ));

		i32 left = m_nodes[ searchIndex ].left;
		i32 right = m_nodes[ searchIndex ].right;

		// Calculate costs for left/right descents. If traversal is to a leaf,
		// then the cost of the combind AABB represents a new branch node. Otherwise
		// the cost is only the inflation of the pre-existing branch.
		// 计算下降到左/右子树的成本。若目标是叶节点，合并 AABB 的成本代表新建分支；
		// 否则只是既有分支的膨胀增量。
		r32 leftDescentCost;
		if ( m_nodes[left].IsLeaf( ) )
			leftDescentCost = q3Combine( leafAABB, m_nodes[left].aabb ).SurfaceArea( ) + inheritedCost;
		else
		{
			r32 inflated = q3Combine( leafAABB, m_nodes[left].aabb ).SurfaceArea( );
			r32 branchArea = m_nodes[ left ].aabb.SurfaceArea( );
			leftDescentCost = inflated - branchArea + inheritedCost;
		}

		// Cost for right descent
		r32 rightDescentCost;
		if ( m_nodes[right].IsLeaf( ) )
			rightDescentCost = q3Combine( leafAABB, m_nodes[right].aabb ).SurfaceArea( ) + inheritedCost;
		else
		{
			r32 inflated = q3Combine( leafAABB, m_nodes[right].aabb ).SurfaceArea( );
			r32 branchArea = m_nodes[ right ].aabb.SurfaceArea( );
			rightDescentCost = inflated - branchArea + inheritedCost;
		}

		// Determine traversal direction, or early out on a branch index
		// 若「在此处直接插入」比继续下降到左右子树都便宜，就停在这里
		if ( branchCost < leftDescentCost && branchCost < rightDescentCost )
			break;

		// 否则向成本更低的一侧继续下降
		if ( leftDescentCost < rightDescentCost )
			searchIndex = left;

		else
			searchIndex = right;
	}

	// 找到的兄弟节点
	i32 sibling = searchIndex;

	// Create new parent
	// 新建一个父节点，包住兄弟和新叶
	i32 oldParent = m_nodes[sibling].parent;
	i32 newParent = AllocateNode( );
	m_nodes[ newParent ].parent = oldParent;
	m_nodes[ newParent ].userData = NULL;
	m_nodes[ newParent ].aabb = q3Combine( leafAABB, m_nodes[sibling].aabb );
	m_nodes[ newParent ].height = m_nodes[sibling].height + 1;

	// Sibling was root
	// 兄弟原来是根：新父节点成为根
	if ( oldParent == Node::Null )
	{
		m_nodes[ newParent ].left = sibling;
		m_nodes[ newParent ].right = id;
		m_nodes[ sibling ].parent = newParent;
		m_nodes[ id ].parent = newParent;
		m_root = newParent;
	}

	// 否则把新父节点接到原来的 parent 下（替换掉兄弟的位置）
	else
	{
		if ( m_nodes[ oldParent ].left == sibling )
			m_nodes[ oldParent ].left = newParent;

		else
			m_nodes[ oldParent ].right = newParent;

		m_nodes[ newParent ].left = sibling;
		m_nodes[ newParent ].right = id;
		m_nodes[ sibling ].parent = newParent;
		m_nodes[ id ].parent = newParent;
	}

	// 从新父节点向上，重新平衡、合并 AABB、更新高度
	SyncHeirarchy( m_nodes[ id ].parent );
}

//--------------------------------------------------------------------------------------------------
// RemoveLeaf：摘除叶节点，并让它的兄弟顶替父节点的位置。
void q3DynamicAABBTree::RemoveLeaf( i32 id )
{
	// 删除的是根：树变空
	if ( id == m_root )
	{
		m_root = Node::Null;
		return;
	}

	// Setup parent, grandParent and sibling
	// 找到父节点、祖父节点和兄弟节点
	i32 parent = m_nodes[ id ].parent;
	i32 grandParent = m_nodes[parent].parent;
	i32 sibling;

	if ( m_nodes[ parent ].left == id )
		sibling = m_nodes[ parent ].right;

	else
		sibling = m_nodes[ parent ].left;

	// Remove parent and replace with sibling
	// 祖父非空：让祖父直接指向兄弟（兄弟顶替父节点）
	if ( grandParent != Node::Null )
	{
		// Connect grandParent to sibling
		if ( m_nodes[ grandParent ].left == parent )
			m_nodes[ grandParent ].left = sibling;

		else
			m_nodes[ grandParent ].right = sibling;

		// Connect sibling to grandParent
		m_nodes[ sibling ].parent = grandParent;
	}

	// Parent was root
	// 父节点是根：兄弟成为新根
	else
	{
		m_root = sibling;
		m_nodes[ sibling ].parent = Node::Null;
	}

	// 父节点归还空闲链表，并从祖父向上重新同步（平衡 + 合并 AABB + 高度）
	DeallocateNode( parent );
	SyncHeirarchy( grandParent );
}

//--------------------------------------------------------------------------------------------------
// 从给定节点开始向上遍历到根，逐层：平衡旋转 → 更新高度 → 重新合并 AABB。
void q3DynamicAABBTree::SyncHeirarchy( i32 index )
{
	while ( index != Node::Null )
	{
		index = Balance( index );   // 平衡后可能返回新的子树根

		i32 left = m_nodes[ index ].left;
		i32 right = m_nodes[ index ].right;

		m_nodes[ index ].height = 1 + q3Max( m_nodes[ left ].height, m_nodes[ right ].height );
		m_nodes[ index ].aabb = q3Combine( m_nodes[ left ].aabb, m_nodes[ right ].aabb );

		index = m_nodes[ index ].parent;
	}
}
