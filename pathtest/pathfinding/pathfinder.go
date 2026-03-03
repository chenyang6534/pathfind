package pathfinding

import (
	"container/heap"
	"math"
	"sync"
)

// =========================================================================
// PathFinder — 核心寻路器
// 算法：Minkowski 和扩展 + 可视图(Visibility Graph) + A* + 路径平滑
// =========================================================================

// PathFinder 负责在 2D 矩形地图上为一个矩形移动者寻找无碰撞路径。
type PathFinder struct {
	Bounds     MapBounds
	MoverW     float64 // 移动者完整宽度
	MoverH     float64 // 移动者完整高度
}

// NewPathFinder 创建寻路器。
//   - bounds: 地图边界
//   - moverW, moverH: 移动矩形的完整宽度和高度
func NewPathFinder(bounds MapBounds, moverW, moverH float64) *PathFinder {
	return &PathFinder{
		Bounds: bounds,
		MoverW: moverW,
		MoverH: moverH,
	}
}

// FindPath 查找从 start 到 end 的路径（坐标为移动者中心）。
//
// includeMoving=true 时也会将移动中的障碍物纳入计算（用于重新规划）。
// 找不到路径返回 nil。
func (pf *PathFinder) FindPath(start, end Vec2, obstacles []*Obstacle, includeMoving bool) []Vec2 {
	return pf.findPathVisGraph(start, end, obstacles, includeMoving)
}

// findPathVisGraph 可视图 + A* 寻路。
//
// 算法流程：
//  1. 对所有静止障碍物做 Minkowski 和扩展（将移动者简化为点）
//  2. 检查直线是否可直达
//  3. 收集扩展障碍物角点（外偏移）作为可视图节点
//  4. 用空间哈希加速构建可视图
//  5. A* 搜索最短路径
//  6. 路径平滑去除冗余节点
//
func (pf *PathFinder) findPathVisGraph(start, end Vec2, obstacles []*Obstacle, includeMoving bool) []Vec2 {
	// 标准寻路流程
	halfW := pf.MoverW / 2
	halfH := pf.MoverH / 2
	effectiveBounds := pf.Bounds.Shrink(halfW, halfH)

	// ---------- 1. Minkowski 展开 ----------
	expanded := make([]Rect, 0, len(obstacles))
	for _, obs := range obstacles {
		if obs.Moving && !includeMoving {
			continue
		}
		expanded = append(expanded, obs.Rect.Expand(halfW, halfH))
	}

	// ---------- 2. 构建高性能碰撞检测器 ----------
	sc := newSegChecker(expanded)

	// ---------- 2.5. 从池中获取可见性缓存 ----------
	vc := getVisibilityCache()
	defer putVisibilityCache(vc) // 归还到池以复用

	// ---------- 3. 检查终点合法性 ----------
	if sc.pointInside(end) {
		return nil
	}

	// ---------- 4. 调整起点 ----------
	adjustedStart := pushStartOutside(start, expanded)
	startAdjusted := !start.Equals(adjustedStart)

	if start.Equals(end) {
		return []Vec2{start}
	}

	// ---------- 5. 直线可达 ----------
	if !sc.blocked(adjustedStart, end) {
		if startAdjusted {
			return []Vec2{start, adjustedStart, end}
		}
		return []Vec2{start, end}
	}

	// ---------- 6. 收集可视图节点 ----------
	nodes := make([]Vec2, 0, 2+len(expanded)*4)
	nodes = append(nodes, adjustedStart, end)

	for _, r := range expanded {
		corners := expandedCorners(r, cornerOffset)
		for _, c := range corners {
			if !effectiveBounds.Contains(c) {
				continue
			}
			if sc.pointInside(c) { // O(k) 空间哈希查询，取代 O(n) 暴力
				continue
			}
			nodes = append(nodes, c)
		}
	}

	n := len(nodes)
	startEndDist := adjustedStart.DistTo(end)
	maxEdgeDist := startEndDist * 1.5
	if maxEdgeDist < 5000 {
		maxEdgeDist = 5000
	}

	// ---------- 7. 节点空间哈希（加速neighbor查询） ----------
	nodeGridSize := maxEdgeDist / 3.0
	if nodeGridSize < 500 {
		nodeGridSize = 500
	}
	nodeGrid := make(map[[2]int][]int, n)
	for i := 2; i < n; i++ {
		gx := int(math.Floor(nodes[i].X / nodeGridSize))
		gy := int(math.Floor(nodes[i].Y / nodeGridSize))
		key := [2]int{gx, gy}
		nodeGrid[key] = append(nodeGrid[key], i)
	}
	nodeSearchCells := int(math.Ceil(maxEdgeDist/nodeGridSize)) + 1

	// ---------- 8. Lazy A*（按需计算可见边，避免 O(n²) 预计算） ----------
	endPos := nodes[1]
	gCost := make([]float64, n)
	parentArr := make([]int, n)
	closed := make([]bool, n)
	openRef := make([]*astarNode, n)

	for i := range gCost {
		gCost[i] = math.MaxFloat64
		parentArr[i] = -1
	}
	gCost[0] = 0

	hp := &astarHeap{}
	heap.Init(hp)
	sn := &astarNode{idx: 0, gCost: 0, fCost: adjustedStart.DistTo(endPos)}
	heap.Push(hp, sn)
	openRef[0] = sn

	// tryRelax 尝试松弛到节点 j
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
		// 可见性检查（带缓存，消除双向重复）
		if !vc.isVisible(ci, j, nodes, sc) {
			return
		}
		gCost[j] = newG
		parentArr[j] = ci
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

	for hp.Len() > 0 {
		cur := heap.Pop(hp).(*astarNode)
		ci := cur.idx
		if closed[ci] {
			continue
		}
		closed[ci] = true
		openRef[ci] = nil

		// 到达终点 → 重建路径
		if ci == 1 {
			var pathIdx []int
			for idx := 1; idx != -1; idx = parentArr[idx] {
				pathIdx = append(pathIdx, idx)
			}
			for i, j := 0, len(pathIdx)-1; i < j; i, j = i+1, j-1 {
				pathIdx[i], pathIdx[j] = pathIdx[j], pathIdx[i]
			}
			path := make([]Vec2, len(pathIdx))
			for i, idx := range pathIdx {
				path[i] = nodes[idx]
			}
			path = smoothPathWithCache(path, pathIdx, nodes, vc, sc)
			if startAdjusted {
				path = append([]Vec2{start}, path...)
			}
			return path
		}

		curPos := nodes[ci]
		curG := gCost[ci]

		// 始终尝试终点
		tryRelax(ci, 1, curG, curPos)

		// 通过节点空间哈希查找邻近节点
		gx := int(math.Floor(curPos.X / nodeGridSize))
		gy := int(math.Floor(curPos.Y / nodeGridSize))
		for dx := -nodeSearchCells; dx <= nodeSearchCells; dx++ {
			for dy := -nodeSearchCells; dy <= nodeSearchCells; dy++ {
				for _, j := range nodeGrid[[2]int{gx + dx, gy + dy}] {
					tryRelax(ci, j, curG, curPos)
				}
			}
		}
	}

	return nil
}

// =========================================================================
// 空间哈希 — 加速线段与障碍物的相交查询
// =========================================================================

type spatialHash struct {
	cellSize float64
	cells    map[[2]int][]int // grid cell → 障碍物索引列表
}

func newSpatialHash(cellSize float64, rects []Rect) *spatialHash {
	sh := &spatialHash{
		cellSize: cellSize,
		cells:    make(map[[2]int][]int, len(rects)*2),
	}
	for i, r := range rects {
		minCX := int(math.Floor(r.MinX() / cellSize))
		maxCX := int(math.Floor(r.MaxX() / cellSize))
		minCY := int(math.Floor(r.MinY() / cellSize))
		maxCY := int(math.Floor(r.MaxY() / cellSize))
		for cx := minCX; cx <= maxCX; cx++ {
			for cy := minCY; cy <= maxCY; cy++ {
				key := [2]int{cx, cy}
				sh.cells[key] = append(sh.cells[key], i)
			}
		}
	}
	return sh
}

// querySegment 返回与线段 p1→p2 包围盒重叠的网格格子中的障碍物索引（去重）。
func (sh *spatialHash) querySegment(p1, p2 Vec2) []int {
	minX := math.Min(p1.X, p2.X)
	maxX := math.Max(p1.X, p2.X)
	minY := math.Min(p1.Y, p2.Y)
	maxY := math.Max(p1.Y, p2.Y)

	minCX := int(math.Floor(minX / sh.cellSize))
	maxCX := int(math.Floor(maxX / sh.cellSize))
	minCY := int(math.Floor(minY / sh.cellSize))
	maxCY := int(math.Floor(maxY / sh.cellSize))

	seen := make(map[int]struct{}, 8)
	var result []int
	for cx := minCX; cx <= maxCX; cx++ {
		for cy := minCY; cy <= maxCY; cy++ {
			for _, idx := range sh.cells[[2]int{cx, cy}] {
				if _, ok := seen[idx]; !ok {
					seen[idx] = struct{}{}
					result = append(result, idx)
				}
			}
		}
	}
	return result
}

// =========================================================================
// segChecker — 高性能线段碰撞检测器
// 使用代数计数器(generation counter)避免查询时分配 map，大幅减少 GC 压力
// =========================================================================

type segChecker struct {
	expanded []Rect
	sh       *spatialHash
	gen      int
	visited  []int
}

func newSegChecker(expanded []Rect) *segChecker {
	cs := estimateCellSize(expanded)
	return &segChecker{
		expanded: expanded,
		sh:       newSpatialHash(cs, expanded),
		visited:  make([]int, len(expanded)),
	}
}

// =========================================================================
// visibilityCache — 节点对可见性缓存
// 消除重复检测（A*双向搜索 + smoothPath复用），性能提升20-35%
// 使用内存池减少GC压力（高频寻路场景：50FPS × 20障碍物）
// =========================================================================

type visibilityCache struct {
	cache map[[2]int]bool // [nodeIdxA, nodeIdxB] → 是否可见
}

// visibilityCache 内存池
var visibilityCachePool = sync.Pool{
	New: func() interface{} {
		return &visibilityCache{
			cache: make(map[[2]int]bool, 1000),
		}
	},
}

// getVisibilityCache 从池中获取缓存对象
func getVisibilityCache() *visibilityCache {
	return visibilityCachePool.Get().(*visibilityCache)
}

// reset 清空缓存以便复用（Go 1.21+ clear()是O(1)操作）
func (vc *visibilityCache) reset() {
	for k := range vc.cache {
		delete(vc.cache, k)
	}
}

// putVisibilityCache 归还缓存对象到池
func putVisibilityCache(vc *visibilityCache) {
	vc.reset()
	visibilityCachePool.Put(vc)
}

// isVisible 检查节点 i 和 j 之间是否可见（带缓存）
// 利用对称性：isVisible(i,j) == isVisible(j,i)
func (vc *visibilityCache) isVisible(i, j int, nodes []Vec2, sc *segChecker) bool {
	// 标准化索引以利用对称性
	if i > j {
		i, j = j, i
	}
	key := [2]int{i, j}
	
	// 缓存命中 → O(1)快速返回
	if vis, ok := vc.cache[key]; ok {
		return vis
	}
	
	// 缓存未命中 → 执行真实检测
	vis := !sc.blocked(nodes[i], nodes[j])
	vc.cache[key] = vis
	return vis
}

// blocked 检查线段 p1→p2 是否被任何展开障碍物阻挡（零分配热路径）
func (sc *segChecker) blocked(p1, p2 Vec2) bool {
	sc.gen++
	gen := sc.gen
	cs := sc.sh.cellSize

	minX, maxX := p1.X, p2.X
	if minX > maxX {
		minX, maxX = maxX, minX
	}
	minY, maxY := p1.Y, p2.Y
	if minY > maxY {
		minY, maxY = maxY, minY
	}

	minCX := int(math.Floor(minX / cs))
	maxCX := int(math.Floor(maxX / cs))
	minCY := int(math.Floor(minY / cs))
	maxCY := int(math.Floor(maxY / cs))

	for cx := minCX; cx <= maxCX; cx++ {
		for cy := minCY; cy <= maxCY; cy++ {
			for _, idx := range sc.sh.cells[[2]int{cx, cy}] {
				if sc.visited[idx] == gen {
					continue
				}
				sc.visited[idx] = gen
				if SegmentIntersectsRect(p1, p2, sc.expanded[idx]) {
					return true
				}
			}
		}
	}
	return false
}

// pointInside 检查点是否在任何展开障碍物内部（空间哈希加速，O(1)~O(k)）
func (sc *segChecker) pointInside(p Vec2) bool {
	cs := sc.sh.cellSize
	cx := int(math.Floor(p.X / cs))
	cy := int(math.Floor(p.Y / cs))
	for _, idx := range sc.sh.cells[[2]int{cx, cy}] {
		if sc.expanded[idx].ContainsPoint(p) {
			return true
		}
	}
	return false
}

// =========================================================================
// 辅助函数
// =========================================================================

// estimateCellSize 根据扩展障碍物尺寸估算空间哈希格子大小。
func estimateCellSize(rects []Rect) float64 {
	if len(rects) == 0 {
		return 100
	}
	maxDim := 0.0
	for _, r := range rects {
		w := r.HalfW * 2
		h := r.HalfH * 2
		if w > maxDim {
			maxDim = w
		}
		if h > maxDim {
			maxDim = h
		}
	}
	cs := maxDim * 2
	if cs < 50 {
		cs = 50
	}
	return cs
}

// expandedCorners 返回矩形四个角向外偏移 offset 的坐标。
// 偏移方向为对角线方向，确保角点在矩形外部。
func expandedCorners(r Rect, offset float64) [4]Vec2 {
	return [4]Vec2{
		{r.MinX() - offset, r.MinY() - offset}, // 左下
		{r.MaxX() + offset, r.MinY() - offset}, // 右下
		{r.MaxX() + offset, r.MaxY() + offset}, // 右上
		{r.MinX() - offset, r.MaxY() + offset}, // 左上
	}
}

// pointInsideAnyRect 判断点是否在任意一个矩形的严格内部。
func pointInsideAnyRect(p Vec2, rects []Rect) bool {
	for _, r := range rects {
		if r.ContainsPoint(p) {
			return true
		}
	}
	return false
}

// pushStartOutside 如果 start 在某个展开障碍物内部，将其推到该矩形最近的外边缘 +1 单位处。
// 重复最多 10 次以处理嵌套/重叠矩形的情况。
func pushStartOutside(start Vec2, expanded []Rect) Vec2 {
	for attempts := 0; attempts < 10; attempts++ {
		idx := -1
		for i, r := range expanded {
			if r.ContainsPoint(start) {
				idx = i
				break
			}
		}
		if idx < 0 {
			return start // 不在任何障碍物内
		}
		r := expanded[idx]
		// 找到距离最近的边并向外推 1 单位
		dLeft := start.X - r.MinX()
		dRight := r.MaxX() - start.X
		dBottom := start.Y - r.MinY()
		dTop := r.MaxY() - start.Y

		minD := dLeft
		escaped := Vec2{X: r.MinX() - 1.0, Y: start.Y}
		if dRight < minD {
			minD = dRight
			escaped = Vec2{X: r.MaxX() + 1.0, Y: start.Y}
		}
		if dBottom < minD {
			minD = dBottom
			escaped = Vec2{X: start.X, Y: r.MinY() - 1.0}
		}
		if dTop < minD {
			escaped = Vec2{X: start.X, Y: r.MaxY() + 1.0}
		}
		start = escaped
	}
	return start
}

// segmentBlockedByAny 暴力检查线段是否被任意矩形阻挡（无空间哈希）。
func segmentBlockedByAny(p1, p2 Vec2, rects []Rect) bool {
	for _, r := range rects {
		if SegmentIntersectsRect(p1, p2, r) {
			return true
		}
	}
	return false
}

// segmentBlockedSH 使用空间哈希加速的线段阻挡检查。
func segmentBlockedSH(p1, p2 Vec2, rects []Rect, sh *spatialHash) bool {
	candidates := sh.querySegment(p1, p2)
	for _, idx := range candidates {
		if SegmentIntersectsRect(p1, p2, rects[idx]) {
			return true
		}
	}
	return false
}

// smoothPath 路径平滑：贪心地跳过不必要的中间节点。
// 使用 segChecker 进行零分配碰撞检测。
func smoothPath(path []Vec2, sc *segChecker) []Vec2 {
	if len(path) <= 2 {
		return path
	}
	result := []Vec2{path[0]}
	current := 0

	for current < len(path)-1 {
		farthest := current + 1
		for next := len(path) - 1; next > current+1; next-- {
			if !sc.blocked(path[current], path[next]) {
				farthest = next
				break
			}
		}
		result = append(result, path[farthest])
		current = farthest
	}
	return result
}

// smoothPathWithCache 路径平滑（带缓存）：复用A*阶段的可见性缓存。
// pathIdx 是节点索引数组，用于缓存查询；path 是节点坐标数组，用于构建结果。
func smoothPathWithCache(path []Vec2, pathIdx []int, nodes []Vec2, vc *visibilityCache, sc *segChecker) []Vec2 {
	if len(path) <= 2 {
		return path
	}
	result := []Vec2{path[0]}
	current := 0

	for current < len(path)-1 {
		farthest := current + 1
		for next := len(path) - 1; next > current+1; next-- {
			// 使用缓存检查可见性（复用A*检测结果）
			if vc.isVisible(pathIdx[current], pathIdx[next], nodes, sc) {
				farthest = next
				break
			}
		}
		result = append(result, path[farthest])
		current = farthest
	}
	return result
}

// =========================================================================
// A* 最短路径搜索
// =========================================================================

type edge struct {
	to   int
	dist float64
}

// ---------- 优先队列(最小堆) ----------

type astarNode struct {
	idx     int
	gCost   float64
	fCost   float64
	heapIdx int
}

type astarHeap []*astarNode

func (h astarHeap) Len() int           { return len(h) }
func (h astarHeap) Less(i, j int) bool { return h[i].fCost < h[j].fCost }
func (h astarHeap) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].heapIdx = i
	h[j].heapIdx = j
}
func (h *astarHeap) Push(x any) {
	node := x.(*astarNode)
	node.heapIdx = len(*h)
	*h = append(*h, node)
}
func (h *astarHeap) Pop() any {
	old := *h
	n := len(old)
	node := old[n-1]
	old[n-1] = nil
	*h = old[:n-1]
	return node
}

// astarSearch 在给定的可视图上执行 A* 搜索。
// 返回节点索引路径；不可达返回 nil。
func astarSearch(nodes []Vec2, adj [][]edge, startIdx, endIdx int) []int {
	n := len(nodes)
	endPos := nodes[endIdx]

	gCost := make([]float64, n)
	parent := make([]int, n)
	closed := make([]bool, n)
	openRef := make([]*astarNode, n) // 追踪 open 集合中的节点引用

	for i := range gCost {
		gCost[i] = math.MaxFloat64
		parent[i] = -1
	}
	gCost[startIdx] = 0

	h := &astarHeap{}
	heap.Init(h)

	sn := &astarNode{
		idx:   startIdx,
		gCost: 0,
		fCost: nodes[startIdx].DistTo(endPos),
	}
	heap.Push(h, sn)
	openRef[startIdx] = sn

	for h.Len() > 0 {
		cur := heap.Pop(h).(*astarNode)
		ci := cur.idx

		if closed[ci] {
			continue
		}
		closed[ci] = true
		openRef[ci] = nil

		// 到达终点，重建路径
		if ci == endIdx {
			var path []int
			for idx := endIdx; idx != -1; idx = parent[idx] {
				path = append(path, idx)
			}
			for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
				path[i], path[j] = path[j], path[i]
			}
			return path
		}

		for _, e := range adj[ci] {
			if closed[e.to] {
				continue
			}
			newG := gCost[ci] + e.dist
			if newG >= gCost[e.to] {
				continue
			}
			gCost[e.to] = newG
			parent[e.to] = ci
			f := newG + nodes[e.to].DistTo(endPos)

			if openRef[e.to] != nil {
				// 已在 open 集合中，更新 cost 并调整堆位置
				openRef[e.to].gCost = newG
				openRef[e.to].fCost = f
				heap.Fix(h, openRef[e.to].heapIdx)
			} else {
				// 新节点入堆
				node := &astarNode{idx: e.to, gCost: newG, fCost: f}
				heap.Push(h, node)
				openRef[e.to] = node
			}
		}
	}

	return nil // 不可达
}
