qu3e v1.01
==========

qu3e 是一个使用 C++ 编写的紧凑、轻量且快速的 3D 物理引擎。它是专为在游戏中使用而创建的。它具有良好的可移植性，除了若干标准 C 头文件（例如 **cassert** 和 **cmath**）之外，没有任何外部依赖。qu3e 的设计目标是提供一个极其简单的接口，用于创建和操作刚体（rigid body）。

qu3e 特别适合那些需要一个快速且简单的 3D 物理引擎，同时又不愿花太多时间去学习整个引擎工作原理的用户。为了让新用户能够保持简单和友好，目前仅支持盒体（box）碰撞，不支持其他形状（如果用户有需求，未来可能会添加胶囊体和球体）。

由于 qu3e 是用 C++ 编写的，它面向熟悉 C++ 的用户。qu3e 的内部代码有相当多的注释，是用户学习 3D 物理引擎工作方式的绝佳场所。

qu3e 代表 "cube"（立方体），因为数字 3 看起来有点像字母 *b*，而盒体（或者说立方体！）是主要的碰撞对象类型。

截图（Screenshots）
-----------

![screenshot 1](/screenshots/screen0.PNG?raw=true)
![screenshot 2](/screenshots/screen1.PNG?raw=true)
![screenshot 2](/screenshots/anim0.gif?raw=true)
![screenshot 3](/screenshots/rays.gif?raw=true)
![screenshot 4](/screenshots/aggregate.gif?raw=true)

功能列表（Feature List）
------------

由于 qu3e 的首要目标是使用简单，因此功能列表被有意保持在最小范围。如果你需要一个功能更全面的开源物理引擎，我推荐 Bullet 物理库：

* 极其简单、友好易用的 API
* 3D 有向包围盒（Oriented Bounding Box, OBB）碰撞检测与求解
* 离散碰撞检测（Discrete collision detection）
* 向世界中发射 3D 射线检测（Raycasting）（示例用法参见 demo 中的 RayPush.h）
* 能够用 AABB 和点来查询世界（query the world）
* 碰撞事件回调
* 传感器（Sensors，即碰撞体）
* 能够创建由任意数量盒体组成的聚合刚体（aggregate rigid body）
* 盒体堆叠（Box stacking）
* 岛屿化与休眠（Islanding and sleeping），用于 CPU 优化
* 与渲染器无关的调试绘制接口
* 动态 AABB 树宽相位（broad phase）
* 通过分离轴定理（Separating Axis Theorem, SAT）生成高精度碰撞流形（manifold）
* 碰撞层（Collision layers）
* 旋转轴锁定（x、y 或 z 轴）
* 可修改的 q3Alloc 和 q3Free 函数，用于自定义内存分配
* 内部堆（heap）和动态数组用于内存管理，使用 q3Alloc/q3Free
* 场景转储（Scene dump）—— 可以输出一个 C++ 代码日志文件，用于重建物理场景

使用 qu3e（Using qu3e）
----------

使用 qu3e 的核心在于创建刚体并为其附加盒体。刚体在最初创建时没有任何碰撞信息。可以为刚体附加任意数量的形状（shape）。所有形状都相对于其所属的刚体来定义，换句话说：所有形状都定义在模型空间（model space）中，而模型空间就是父刚体的参考系。

唯一需要包含的头文件是 **q3.h**。首先创建一个物理场景（physics scene）。通常只需要创建一个物理场景。用户在创建场景时必须指定一个固定的时间步长（timestep）：

```c++
#include "q3.h"

q3Scene scene( 1.0 / 60.0 );
```

刚体由物理场景创建，且刚体（或形状）不能在不同场景之间共享。创建刚体首先要创建一个 **q3BodyDef** 对象。Def 对象可以直接在栈上创建：

```c++
q3BodyDef bodyDef;
q3Body* body = scene.CreateBody( bodyDef );
```

**q3BodyDef** 可以传递给场景，以换取一个新的刚体。body 定义允许用户指定许多设置，详情参见 q3Body.h。一旦刚体构建完成，可以通过向刚体提供一个 **q3BoxDef** 来为其添加任意数量的盒体：

```c++
q3BoxDef boxDef; // 设置详情参见 q3Box.h
q3Transform localSpace; // 包含位置和朝向，详情参见 q3Transform.h
q3Identity( localSpace ); // 指定原点，以及单位（identity）朝向

// 在原点处创建一个宽、高、深 = (1.0, 1.0, 1.0) 的盒体
// 并将其添加到刚体。该变换是相对于所属刚体定义的
boxDef.Set( localSpace, q3Vec3( 1.0, 1.0, 1.0 ) );
body->AddBox( boxDef );
```

要模拟场景，只需调用 **scene.Step( )**。这会让世界按照场景构建时指定的时间步长（通常是 1/60 或 1/30）向前推进。

报告 Bug（Reporting Bugs）
--------------
<b>我发现了 bug，该如何报告？或者我可以自己修复吗？</b>

如果你愿意修复 bug，请直接提交一个 pull request，我会尽力审阅并合并该请求。否则，GitHub 的 "issues" 功能非常适合报告和讨论 bug。

<b>如果可能，请使用 q3Scene::Dump( FILE* ) 功能。</b>

qu3e 有一个 Box2D 也采用的酷炫功能：场景可以将 C++ 代码转储到一个文本文件中。这些代码可以直接复制/粘贴到我自己的某个 demo 文件中。如果任何人有特定的 bug，并且能够在 bug 发生之前转储场景，就可以共享场景文件 —— 这为我自己（或其他人）提供了一种非常简便的方式来定位并消除 bug。场景转储工具总体来说也是一种共享场景初始化的好方法。注意：由于转储输出的是 C++ 代码，转储本身并不是一种合适的序列化形式，因为需要重新编译才能运行转储的内容。


常见问题（FAQ）
---

<b>你们能加入这个功能吗？</b>

主要目标之一是保持较低的功能列表规模，以控制代码膨胀、复杂度以及 API 本身。如果你想要一个功能更全面的开源物理库，请参考 Bullet 物理引擎。话虽如此，我始终乐于接受新想法。请使用 GitHub 的 "issues" 功能来建议或讨论改进 —— 只要时间允许，我会很乐意回复。

<b>与 Bullet、Havok 或 ODE 之类的库相比，使用你的库有哪些优缺点？</b>

据我所知，目前*还没有*任何简单、快速的开源现代 3D 物理引擎。希望 qu3e 能够填补这一角色，其优点包括：体积小、易于移植到其他语言、免费、实现风格一致、依赖极少甚至无依赖。与其他引擎相比，qu3e 唯一的缺点是功能列表较少，但这是否是一个显著的缺点，取决于用户的需求。

<b>你为什么不在代码中使用这个（插入某个 C++ 特性/库/工具）？</b>

使用各种 C++ 特性或外部 C++ 库会增加整个项目的依赖。我们的理念是，如果简单的类 C 代码普遍存在，那么移植到其他语言就会变得简单。仅仅因为某个特性存在，并不意味着就应该使用它。

<b>支持哪些碰撞形状？</b>

目前只有盒体（宽、高、深）。球体和胶囊体将来可能会添加，取决于用户是否提出需求。目前可以使用任意数量的盒体来构建聚合刚体 —— 这满足了大多数用户的大部分碰撞需求。也许在遥远的将来会添加凸包（convex hull）和网格（mesh）。

未来（Future）
------

随着时间的推移，qu3e 的功能列表有意不会大幅增长（至少在不久的将来是这样）。由于该库的首要目标是在实现上非常简单，且易于使用，因此应当避免膨胀。然而，如果有人愿意为以下任何一个领域做出贡献，我会非常乐意与贡献者合作，将他们的贡献纳入 qu3e。我会根据具体情况处理贡献，贡献者将在源代码注释和本 readme 中获得完整的署名。

<b>Baumgarte 稳定法与 Post Stabilization（完全非线性 Gauss Seidel）</b>

目前 qu3e 使用 Baumgarte 稳定法来确保不发生 "约束漂移"（constraint drift）。Baumgarte 方法非常高效且易于实现，因此 qu3e 使用了它。然而，Baumgarte 方法在许多情况下的效果并不尽如人意，并且会给系统带来额外的能量。这就是为什么 qu3e 在模拟零恢复系数（zero restitution）时会有困难的原因。

一个更好的稳定方法是 [Post Stabilization](https://www.cs.rutgers.edu/~dpai/papers/ClinePai03.pdf)。其思路是利用速度约束中的速度雅可比矩阵（velocity jacobian），构建一个迭代式的位置层求解器（position level solver）。关于各种稳定技术（包括本段所讨论的首选方法）的良好讨论，可以在 Box2D 的 b2Island.cpp 文件靠近顶部的注释中找到。要实现恰当的 Post Stabilization，需要对 qu3e 的碰撞检测进行相当多的重构。目前 qu3e 是在世界空间（world space）中构建碰撞流形的。如果将其改为存储相对于与流形相关联的形状的信息，那么实现 Post Stabilization 就会容易得多。还需要做一些其他更改，以确保避免大量缓存未命中（cache miss）。

我希望在未来的某个时候升级 qu3e 的稳定方法，但目前不知道何时才能实现，因为这需要相当多的时间。

<b>流形缩减（Manifold Reduction）</b>

碰撞流形由接触点（contact point）组成。（对于离散碰撞检测）只需要 4 个精心选择的接触点就可以创建一个稳定的流形。两个形状之间的许多流形可能会有远多于四个的接触点。目前，如果两个盒体相互叠放，最多可能会产生 8 个接触点。

将流形缩减到最多 4 个接触点的恒定上限，将提高迭代式接触求解器的稳定性。这种缩减还会降低在内存中保留流形的内存开销，从而作为副作用提高缓存行（cache line）效率。

流形缩减目前在 qu3e 中尚未实现。以下是算法概述，供有意尝试实现的人参考：

* 沿固定方向查询 2D 接触平面以获取一个支撑点（support point），先添加这个第一个点
* 找到距离第一个点最远的点，这形成一条线段
* 找到距离前一条线段最远的第三个点，形成一个三角形
* 找到第四个点，使 2D 接触平面中新四边形（quad）的面积最大化

对于连续碰撞检测，流形中最深的点需要保留在流形中。这会使流形缩减算法变得复杂，此处不讨论这种复杂性。

<b>连续碰撞检测（Continuous Collision Detection）</b>

连续碰撞检测目前在 qu3e 中尚未实现，但拥有它会很酷，并且对一般玩法（gameplay）也很有帮助。Erin Catto 提供了许多免费资源，用于实现他的双边推进碰撞时间（Bilateral Advancement Time of Impact, TOI）求解器。Erin 的 TOI 求解器可用于在两个凸形状之间找到 TOI。从这里开始，可以结合上面描述的 Post Stabilization 方法与计算出的 TOI，来防止 Static/Dynamic 以及 Static/Kinematic 刚体对之间的所有穿透（tunneling）。

Dynamic 对 Dynamic 或 Kinematic 对 Dynamic 的连续碰撞检测将涉及运行一个 O( N^2 ) 算法，其 N 的因子是未知的（且可能非常巨大）。出于这个原因，这里不讨论这些碰撞对。

在未来的某个时候，我很想利用 Erin 的资源，为连续碰撞检测实现一些很酷的开源工具。也许在我从学校毕业之后，这会是一个好主意。

<b>多线程支持（Multi-Threading Support）</b>

多线程是一个有趣的话题，qu3e 在编写时就考虑到了线程化。一个作业系统（job system）或任务系统（task system）对于批量处理诸如碰撞检测之类的任务是非常理想的。也许未来会由我或某位贡献者为 qu3e 添加一个线程池（threadpool）。q3Body 的岛屿索引（island index）需要做一个小调整。还需要做一些小的内存对齐（memory alignment）更改。q3Stack 可以用来为作业分配内存，因为栈分配非常快。

<b>单指令多数据（SIMD）支持</b>

SIMD —— 实际上我没有使用 SIMD 的经验，qu3e 中的所有数学运算都是标量（scalar）的。这对任何人来说都不应该真正构成性能问题，但显然它还有改进空间。不过，Collide.cpp 中有一段巧妙的代码：两个 3D 多边形的裁剪（clipping）是通过单维 lerp（linear interpolation，线性插值）来完成的！这是一种很好的方式，利用标量数学在这个特定情况下避免了任何 SIMD 的需求。

我确实认为，为任何性能瓶颈（bottleneck）换入某个开源数学库，或者手工编写特定代码段，都是相当容易的。

<b>额外的碰撞形状（球体、胶囊体、凸包、网格）</b>

编写碰撞检测需要时间，而在上学期间，我没有无限的时间来专门编写健壮的碰撞检测代码。球体和胶囊体很可能在不久的将来由我自己添加。凸包和网格是其他物理引擎中的功能，所以让 qu3e 去竞争可能没有意义 —— 尤其是其他开源引擎可能会直接将为 qu3e 编写的代码吸收到它们自己的代码库中。

<b>高级关节与弹簧（Advanced Joints and Springs）</b>

高级关节（弹簧、连杆、旋转关节/移动关节）是其他物理库的一个不错的功能，qu3e 可能会纳入其中一些。它们不在任何待办列表中，除非用户提出需求，否则不会添加。某些关节类型不会使库变得杂乱，并且对于精通 C++ 的人来说相当易于使用。更高级关节的最大问题在于设置它们。通常编辑器或可视化工具是设置关节的最佳方式，尽管 qu3e 本身需要使用原始的 C++。

致谢（Credits）
-------
由 [Randy Gaul](http://randygaul.net) 开发。

Erin Catto 的在线资源（例如 [Box2D](http://box2d.org)）为 qu3e 中的大部分代码提供了重要的灵感来源。特别感谢 Dirk Gregorius 和 Nathan Carlson 频繁且宝贵的帮助与建议。

许可证（License）
-------

qu3e 依据 [zlib license](http://en.wikipedia.org/wiki/Zlib_License) 许可。更多信息请参见 LICENSE.txt。

更新日志（Change Log）
----------

* v1.00 : 初始版本
* v1.01 :
    - 射线检测（Raycasting）
    - AABB 查询
    - 点查询（Point query）
    - q3Scene::Dump( FILE* ); 将场景内容转储为 C++ 文件
    - 次要的内部 bug 修复，主要涉及 body -> local 空间的变换
    - 更新了 readme，对潜在的未来功能进行了大量讨论
