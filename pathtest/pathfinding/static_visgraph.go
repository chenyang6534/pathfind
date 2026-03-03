package pathfinding

import (
	"container/heap"
	"fmt"
	"math"
	"time"
)

// =========================================================================
// StaticVisGraph — 静态障碍物可见性图预计算
// 原理：静态障碍物的角点和可见性关系在初始化时一次性预计算，
// 寻路时只需计算起点/终点/动态障碍物与静态节点的可见性。
// =========================================================================

// StaticVisGraph 预计算的静态可见性图
type StaticVisGraph struct {
	MoverHalfW float64   // 移动者半宽
	MoverHalfH float64   // 移动者半高
	Nodes      []Vec2    // 静态角点（Minkowski 扩展后外偏移）
	Expanded   []Rect    // Minkowski 扩展后的静态障碍物
	
	// 静态-静态可见性邻接表（稀疏存储，节省内存）
	// adj[i] = [(j, dist), ...] 表示节点i可见的所有静态节点
	Adj [][]staticEdge
	
	// 预构建的空间哈希（加速线段碰撞检测）
	SpatialHash *spatialHash
	CellSize    float64
}

type staticEdge struct {
	To   int
	Dist float64
}

// StaticGraphCache 多尺寸静态图缓存
// 不同尺寸的移动者使用不同的预计算数据
type StaticGraphCache struct {
	Graphs map[float64]*StaticVisGraph // moverHalf → 静态图
	Bounds MapBounds
}

// NewStaticGraphCache 创建静态图缓存
func NewStaticGraphCache(bounds MapBounds) *StaticGraphCache {
	return &StaticGraphCache{
		Graphs: make(map[float64]*StaticVisGraph),
		Bounds: bounds,
	}
}

// BuildStaticVisGraph 为指定移动者尺寸构建静态可见性图
// staticObs - 静态障碍物列表
// moverW, moverH - 移动者完整宽高
// bounds - 地图边界
func BuildStaticVisGraph(staticObs []*Obstacle, moverW, moverH float64, bounds MapBounds) *StaticVisGraph {
	t0 := time.Now()
	halfW := moverW / 2
	halfH := moverH / 2
	effectiveBounds := bounds.Shrink(halfW, halfH)

	// 1. Minkowski 扩展所有静态障碍物
	expanded := make([]Rect, len(staticObs))
	for i, obs := range staticObs {
		expanded[i] = obs.Rect.Expand(halfW, halfH)
	}

	// 2. 构建空间哈希
	cellSize := estimateCellSize(expanded)
	sh := newSpatialHash(cellSize, expanded)

	// 3. 提取角点
	nodes := make([]Vec2, 0, len(expanded)*4)
	for _, r := range expanded {
		corners := expandedCorners(r, cornerOffset)
		for _, c := range corners {
			if !effectiveBounds.Contains(c) {
				continue
			}
			if pointInsideAnyRectSH(c, expanded, sh, cellSize) {
				continue
			}
			nodes = append(nodes, c)
		}
	}

	// 4. 构建碰撞检测器
	sc := &segChecker{
		expanded: expanded,
		sh:       sh,
		visited:  make([]int, len(expanded)),
	}

	// 5. 预计算静态节点间的可见性（邻接表）
	n := len(nodes)
	adj := make([][]staticEdge, n)
	edgeCount := 0

	// 距离阈值：避免计算过远的节点对
	maxEdgeDist := 15000.0

	for i := 0; i < n; i++ {
		adj[i] = make([]staticEdge, 0, 20) // 预分配
		for j := i + 1; j < n; j++ {
			d := nodes[i].DistTo(nodes[j])
			if d > maxEdgeDist {
				continue
			}
			if !sc.blocked(nodes[i], nodes[j]) {
				// 双向添加
				adj[i] = append(adj[i], staticEdge{To: j, Dist: d})
				adj[j] = append(adj[j], staticEdge{To: i, Dist: d})
				edgeCount++
			}
		}
	}

	elapsed := time.Since(t0)
	fmt.Printf("[静态可见性图] 移动者尺寸 %.0fx%.0f: %d 节点, %d 可见边, 耗时 %dms\n",
		moverW, moverH, n, edgeCount, elapsed.Milliseconds())

	return &StaticVisGraph{
		MoverHalfW:  halfW,
		MoverHalfH:  halfH,
		Nodes:       nodes,
		Expanded:    expanded,
		Adj:         adj,
		SpatialHash: sh,
		CellSize:    cellSize,
	}
}

// GetOrBuild 获取或构建指定尺寸的静态图
func (c *StaticGraphCache) GetOrBuild(staticObs []*Obstacle, moverW, moverH float64) *StaticVisGraph {
	// 使用半宽作为key（正方形移动者宽高相等）
	key := moverW / 2
	if g, ok := c.Graphs[key]; ok {
		return g
	}
	g := BuildStaticVisGraph(staticObs, moverW, moverH, c.Bounds)
	c.Graphs[key] = g
	return g
}

// PreBuildAll 预构建所有常用尺寸的静态图
func (c *StaticGraphCache) PreBuildAll(staticObs []*Obstacle, sizes []float64) {
	fmt.Printf("[静态可见性图] 开始预构建 %d 种尺寸...\n", len(sizes))
	t0 := time.Now()
	for _, size := range sizes {
		c.GetOrBuild(staticObs, size, size)
	}
	fmt.Printf("[静态可见性图] 全部预构建完成, 总耗时 %dms\n", time.Since(t0).Milliseconds())
}

// pointInsideAnyRectSH 使用空间哈希加速的点包含检测
func pointInsideAnyRectSH(p Vec2, rects []Rect, sh *spatialHash, cellSize float64) bool {
	cx := int(math.Floor(p.X / cellSize))
	cy := int(math.Floor(p.Y / cellSize))
	for _, idx := range sh.cells[[2]int{cx, cy}] {
		if rects[idx].ContainsPoint(p) {
			return true
		}
	}
	return false
}

// =========================================================================
// 使用静态图的寻路方法
// =========================================================================

// FindPathWithStaticGraph 使用预计算静态图的寻路
// 核心优化：静态节点间的可见性直接查表，只需计算动态部分
func (pf *PathFinder) FindPathWithStaticGraph(
	start, end Vec2,
	dynamicObs []*Obstacle,
	staticGraph *StaticVisGraph,
) []Vec2 {
	halfW := pf.MoverW / 2
	halfH := pf.MoverH / 2
	effectiveBounds := pf.Bounds.Shrink(halfW, halfH)

	// 1. 展开动态障碍物（静态的已在 staticGraph 中预计算）
	dynamicExpanded := make([]Rect, 0, len(dynamicObs))
	for _, obs := range dynamicObs {
		dynamicExpanded = append(dynamicExpanded, obs.Rect.Expand(halfW, halfH))
	}

	// 2. 合并所有扩展障碍物用于碰撞检测
	allExpanded := make([]Rect, 0, len(staticGraph.Expanded)+len(dynamicExpanded))
	allExpanded = append(allExpanded, staticGraph.Expanded...)
	allExpanded = append(allExpanded, dynamicExpanded...)

	// 3. 构建碰撞检测器
	sc := newSegChecker(allExpanded)

	// 4. 检查终点合法性
	if sc.pointInside(end) {
		return nil
	}

	// 5. 调整起点
	adjustedStart := pushStartOutside(start, allExpanded)
	startAdjusted := !start.Equals(adjustedStart)

	if start.Equals(end) {
		return []Vec2{start}
	}

	// 6. 直线可达
	if !sc.blocked(adjustedStart, end) {
		if startAdjusted {
			return []Vec2{start, adjustedStart, end}
		}
		return []Vec2{start, end}
	}

	// 7. 收集动态障碍物的角点
	dynamicNodes := make([]Vec2, 0, len(dynamicExpanded)*4)
	for _, r := range dynamicExpanded {
		corners := expandedCorners(r, cornerOffset)
		for _, c := range corners {
			if !effectiveBounds.Contains(c) {
				continue
			}
			if sc.pointInside(c) {
				continue
			}
			dynamicNodes = append(dynamicNodes, c)
		}
	}

	// 8. 构建完整节点列表：[起点, 终点, 静态节点..., 动态节点...]
	staticN := len(staticGraph.Nodes)
	totalN := 2 + staticN + len(dynamicNodes)
	nodes := make([]Vec2, totalN)
	nodes[0] = adjustedStart
	nodes[1] = end
	copy(nodes[2:2+staticN], staticGraph.Nodes)
	copy(nodes[2+staticN:], dynamicNodes)

	// 9. 使用混合A*搜索
	path := pf.hybridAstar(nodes, staticGraph, sc, staticN, totalN)
	if path == nil {
		return nil
	}

	// 10. 路径平滑
	path = smoothPath(path, sc)
	if startAdjusted {
		path = append([]Vec2{start}, path...)
	}
	return path
}

// hybridAstar 混合A*搜索
// 静态-静态：查预计算邻接表
// 涉及动态节点/起点/终点：实时计算可见性
func (pf *PathFinder) hybridAstar(
	nodes []Vec2,
	staticGraph *StaticVisGraph,
	sc *segChecker,
	staticN, totalN int,
) []Vec2 {
	// 索引说明：
	// 0 = 起点
	// 1 = 终点
	// [2, 2+staticN) = 静态节点
	// [2+staticN, totalN) = 动态节点

	endPos := nodes[1]
	startEndDist := nodes[0].DistTo(endPos)
	maxEdgeDist := startEndDist * 1.5
	if maxEdgeDist < 5000 {
		maxEdgeDist = 5000
	}

	gCost := make([]float64, totalN)
	parent := make([]int, totalN)
	closed := make([]bool, totalN)
	openRef := make([]*astarNode, totalN)

	for i := range gCost {
		gCost[i] = math.MaxFloat64
		parent[i] = -1
	}
	gCost[0] = 0

	hp := &astarHeap{}
	heap.Init(hp)
	startNode := &astarNode{idx: 0, gCost: 0, fCost: nodes[0].DistTo(endPos)}
	heap.Push(hp, startNode)
	openRef[0] = startNode

	// 构建节点空间哈希（加速邻居查询）
	nodeGridSize := maxEdgeDist / 3.0
	if nodeGridSize < 500 {
		nodeGridSize = 500
	}
	nodeGrid := make(map[[2]int][]int, totalN)
	for i := 2; i < totalN; i++ {
		gx := int(math.Floor(nodes[i].X / nodeGridSize))
		gy := int(math.Floor(nodes[i].Y / nodeGridSize))
		key := [2]int{gx, gy}
		nodeGrid[key] = append(nodeGrid[key], i)
	}
	nodeSearchCells := int(math.Ceil(maxEdgeDist/nodeGridSize)) + 1

	// tryRelax 尝试松弛边
	tryRelax := func(ci, j int, curG float64, curPos Vec2) {
		if j == ci || closed[j] {
			return
		}
		d := curPos.DistTo(nodes[j])
		if ci > 1 && j > 1 && d > maxEdgeDist {
			return
		}
		newG := curG + d
		if newG >= gCost[j] {
			return
		}

		// 判断可见性：
		// - 静态-静态：已在预计算邻接表中，此处跳过（通过邻接表遍历）
		// - 其他情况：实时计算
		visible := !sc.blocked(curPos, nodes[j])
		if !visible {
			return
		}

		gCost[j] = newG
		parent[j] = ci
		f := newG + nodes[j].DistTo(endPos)
		if openRef[j] != nil {
			openRef[j].gCost = newG
			openRef[j].fCost = f
			heap.Fix(hp, openRef[j].heapIdx)
		} else {
			nd := &astarNode{idx: j, gCost: newG, fCost: f}
			heap.Push(hp, nd)
			openRef[j] = nd
		}
	}

	// 通过预计算邻接表松弛静态邻居（O(1)可见性查表）
	relaxStaticNeighbors := func(ci int, curG float64) {
		if ci < 2 || ci >= 2+staticN {
			return // 不是静态节点
		}
		staticIdx := ci - 2
		for _, e := range staticGraph.Adj[staticIdx] {
			j := e.To + 2 // 转换为全局索引
			if closed[j] {
				continue
			}
			// 还需检查动态障碍物的阻挡
			if len(sc.expanded) > len(staticGraph.Expanded) {
				if sc.blocked(nodes[ci], nodes[j]) {
					continue
				}
			}
			newG := curG + e.Dist
			if newG >= gCost[j] {
				continue
			}
			gCost[j] = newG
			parent[j] = ci
			f := newG + nodes[j].DistTo(endPos)
			if openRef[j] != nil {
				openRef[j].gCost = newG
				openRef[j].fCost = f
				heap.Fix(hp, openRef[j].heapIdx)
			} else {
				nd := &astarNode{idx: j, gCost: newG, fCost: f}
				heap.Push(hp, nd)
				openRef[j] = nd
			}
		}
	}

	for hp.Len() > 0 {
		cur := heap.Pop(hp).(*astarNode)
		ci := cur.idx
		if closed[ci] {
			continue
		}
		closed[ci] = true
		openRef[ci] = nil

		// 到达终点
		if ci == 1 {
			var pathIdx []int
			for idx := 1; idx != -1; idx = parent[idx] {
				pathIdx = append(pathIdx, idx)
			}
			for i, j := 0, len(pathIdx)-1; i < j; i, j = i+1, j-1 {
				pathIdx[i], pathIdx[j] = pathIdx[j], pathIdx[i]
			}
			path := make([]Vec2, len(pathIdx))
			for i, idx := range pathIdx {
				path[i] = nodes[idx]
			}
			return path
		}

		curPos := nodes[ci]
		curG := gCost[ci]

		// 始终尝试终点
		tryRelax(ci, 1, curG, curPos)

		// 如果当前是静态节点，使用预计算邻接表
		relaxStaticNeighbors(ci, curG)

		// 通过空间哈希查找邻近节点（处理起点/动态节点）
		gx := int(math.Floor(curPos.X / nodeGridSize))
		gy := int(math.Floor(curPos.Y / nodeGridSize))
		for dx := -nodeSearchCells; dx <= nodeSearchCells; dx++ {
			for dy := -nodeSearchCells; dy <= nodeSearchCells; dy++ {
				for _, j := range nodeGrid[[2]int{gx + dx, gy + dy}] {
					// 静态-静态已通过邻接表处理，跳过
					if ci >= 2 && ci < 2+staticN && j >= 2 && j < 2+staticN {
						continue
					}
					tryRelax(ci, j, curG, curPos)
				}
			}
		}
	}

	return nil
}

// heap.Interface 实现已在 pathfinder.go 中定义，这里复用

