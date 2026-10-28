//--------------------------------------------------------------------------------------------------
// q3DynamicAABBTree.inl
//
//	Copyright (c) 2014 Randy Gaul http://www.randygaul.net
//
//	This software is provided 'as-is', without any express or implied
//	warranty. In no event will the authors be held liable for any damages
//	arising from the use of this software.
//
//	Permission is granted to anyone to use this software for any purpose,
//	including commercial applications, and to alter it and redistribute it
//	freely, subject to the following restrictions:
//	  1. The origin of this software must not be misrepresented; you must not
//	     claim that you wrote the original software. If you use this software
//	     in a product, an acknowledgment in the product documentation would be
//	     appreciated but is not required.
//	  2. Altered source versions must be plainly marked as such, and must not
//	     be misrepresented as being the original software.
//	  3. This notice may not be removed or altered from any source distribution.
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// q3DynamicAABBTree
//--------------------------------------------------------------------------------------------------
// 把 [index, m_capacity) 区间的节点串成一条空闲链表（用 next 字段连接），
// 空闲节点的 height 记为 Null（-1）。
inline void q3DynamicAABBTree::AddToFreeList( i32 index )
{
	for ( i32 i = index; i < m_capacity - 1; ++i)
	{
		m_nodes[ i ].next = i + 1;
		m_nodes[ i ].height = Node::Null;
	}

	m_nodes[ m_capacity - 1 ].next = Node::Null;
	m_nodes[ m_capacity - 1 ].height = Node::Null;
	m_freeList = index;
}

//--------------------------------------------------------------------------------------------------
// 归还节点：把它插到空闲链表头部，并减少活跃节点计数。
inline void q3DynamicAABBTree::DeallocateNode( i32 index )
{
	assert( index >= 0 && index < m_capacity );

	m_nodes[ index ].next = m_freeList;
	m_nodes[ index ].height = Node::Null;
	m_freeList = index;

	--m_count;
}

//--------------------------------------------------------------------------------------------------
// 通过 aabb 初筛， 回调 cb
// 查询与给定 AABB 重叠的所有叶节点：从根向下用栈迭代遍历，
// 不重叠的分支整棵剪掉；命中的叶节点回调 cb->TreeCallBack。
template <typename T>
inline void q3DynamicAABBTree::Query( T *cb, const q3AABB& aabb ) const
{
	const i32 k_stackCapacity = 256;
	i32 stack[ k_stackCapacity ];   // 显式栈（避免递归开销）
	i32 sp = 1;

	*stack = m_root;

	while ( sp )
	{
		// k_stackCapacity too small
		assert( sp < k_stackCapacity );

		i32 id = stack[ --sp ];

		const Node *n = m_nodes + id;
		// 只有和查询 AABB 重叠的节点才继续处理
		if ( q3AABBtoAABB( aabb, n->aabb ) )
		{
			if ( n->IsLeaf( ) )
			{
				// 命中叶节点 → 回调；返回 false 表示提前终止
				if ( !cb->TreeCallBack( id ) )
					return;
			}
			else
			{
				// 分支节点 → 两个孩子入栈继续遍历
				stack[ sp++ ] = n->left;
				stack[ sp++ ] = n->right;
			}
		}
	}
}

//--------------------------------------------------------------------------------------------------
// 射线查询：找出与射线相交的所有叶节点。
// 采用「slab 方法」（三轴区间法）测试射线与 AABB 是否相交，
// 相交的分支继续下探，命中的叶节点回调 cb->TreeCallBack。
template <typename T>
void q3DynamicAABBTree::Query( T *cb, q3RaycastData& rayCast ) const
{
	const r32 k_epsilon = r32( 1.0e-6 );
	const i32 k_stackCapacity = 256;
	i32 stack[ k_stackCapacity ];
	i32 sp = 1;

	*stack = m_root;

	q3Vec3 p0 = rayCast.start;              // 射线起点
	q3Vec3 p1 = p0 + rayCast.dir * rayCast.t;   // 射线终点（沿方向走 t 距离）

	while( sp )
	{
		// k_stackCapacity too small
		assert( sp < k_stackCapacity );

		i32 id = stack[--sp];

		if ( id == Node::Null )
			continue;

		const Node *n = m_nodes + id;

		// ---- slab 方法：射线与 AABB 相交测试 ----
		// e = AABB 的半边长（half-extents）
		q3Vec3 e = n->aabb.max - n->aabb.min;
		q3Vec3 d = p1 - p0;                          // 射线方向向量
		// m = 射线中点相对 AABB 中心的偏移（详见 slab 公式）
		q3Vec3 m = p0 + p1 - n->aabb.min - n->aabb.max;

		// 对 x/y/z 三个轴分别做区间重叠测试，任一轴不重叠则射线不相交
		r32 adx = q3Abs( d.x );

		if ( q3Abs( m.x ) > e.x + adx )
			continue;

		r32 ady = q3Abs( d.y );

		if ( q3Abs( m.y ) > e.y + ady )
			continue;

		r32 adz = q3Abs( d.z );

		if ( q3Abs( m.z ) > e.z + adz )
			continue;

		// 加一个微小容差，避免浮点误差导致漏检
		adx += k_epsilon;
		ady += k_epsilon;
		adz += k_epsilon;

		// 用叉积做「射线与 AABB 的 12 条边」的分离轴测试（三维 slab 的完整形式）
		if( q3Abs( m.y * d.z - m.z * d.y) > e.y * adz + e.z * ady )
			continue;

		if( q3Abs( m.z * d.x - m.x * d.z) > e.x * adz + e.z * adx )
			continue;

		if ( q3Abs( m.x * d.y - m.y * d.x) > e.x * ady + e.y * adx )
			continue;

		// 相交：叶节点回调，分支节点继续下探两个孩子
		if ( n->IsLeaf( ) )
		{
			if ( !cb->TreeCallBack( id ) )
				return;
		}

		else
		{
			stack[ sp++ ] = n->left;
			stack[ sp++ ] = n->right;
		}
	}
}
