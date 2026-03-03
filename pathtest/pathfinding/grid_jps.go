package pathfinding

import (
	"container/heap"
	"math"
)

// =========================================================================
// Grid-based pathfinding with JPS (Jump Point Search) for square movers.
// 所有移动者都是正方形且尺寸为 2 的幂次（半径 2,4,8,16...），
// 因此网格精度能完美匹配。
// =========================================================================

// localGrid 局部可通行网格（仅覆盖寻路走廊区域）
type localGrid struct {
	blocked  []bool  // true = 阻塞
	w, h     int     // 网格尺寸（格数）
	originX  float64 // 格子 (0,0) 左下角世界 X
	originY  float64 // 格子 (0,0) 左下角世界 Y
	cellSize float64 // 每格世界单位
}

func (g *localGrid) inBounds(x, y int) bool {
	return x >= 0 && x < g.w && y >= 0 && y < g.h
}

func (g *localGrid) isBlocked(x, y int) bool {
	if !g.inBounds(x, y) {
		return true
	}
	return g.blocked[y*g.w+x]
}

func (g *localGrid) isWalkable(x, y int) bool {
	return g.inBounds(x, y) && !g.blocked[y*g.w+x]
}

func (g *localGrid) worldToGrid(wx, wy float64) (int, int) {
	return int(math.Floor((wx - g.originX) / g.cellSize)),
		int(math.Floor((wy - g.originY) / g.cellSize))
}

func (g *localGrid) gridToWorld(gx, gy int) Vec2 {
	return Vec2{
		X: g.originX + (float64(gx)+0.5)*g.cellSize,
		Y: g.originY + (float64(gy)+0.5)*g.cellSize,
	}
}

// buildLocalGrid 从 Minkowski 展开后的障碍物构建局部可通行网格
// cellSize 使用移动者半径（moverHalf），保证网格精度匹配移动者尺寸
func buildLocalGrid(start, end Vec2, expanded []Rect, bounds MapBounds, moverHalf float64) *localGrid {
	// 格子大小 = 移动者半径，最小 2，最大 64
	cellSize := moverHalf
	if cellSize < 2 {
		cellSize = 2
	}
	if cellSize > 64 {
		cellSize = 64
	}

	// 走廊包围盒
	corridorMargin := 5500.0
	minX := math.Min(start.X, end.X) - corridorMargin
	maxX := math.Max(start.X, end.X) + corridorMargin
	minY := math.Min(start.Y, end.Y) - corridorMargin
	maxY := math.Max(start.Y, end.Y) + corridorMargin

	// 额外留出边距
	pad := cellSize * 5
	minX -= pad
	maxX += pad
	minY -= pad
	maxY += pad

	// 限制在地图范围内
	if minX < bounds.MinX {
		minX = bounds.MinX
	}
	if minY < bounds.MinY {
		minY = bounds.MinY
	}
	if maxX > bounds.MaxX {
		maxX = bounds.MaxX
	}
	if maxY > bounds.MaxY {
		maxY = bounds.MaxY
	}

	gw := int(math.Ceil((maxX-minX)/cellSize)) + 1
	gh := int(math.Ceil((maxY-minY)/cellSize)) + 1

	// 如果网格过大，自动加粗格子（翻倍直到合适）
	const maxCells = 4_000_000
	for gw*gh > maxCells {
		cellSize *= 2
		gw = int(math.Ceil((maxX-minX)/cellSize)) + 1
		gh = int(math.Ceil((maxY-minY)/cellSize)) + 1
	}

	grid := &localGrid{
		blocked:  make([]bool, gw*gh),
		w:        gw,
		h:        gh,
		originX:  minX,
		originY:  minY,
		cellSize: cellSize,
	}

	// 标记阻塞格子：格心在展开障碍物内部 → 阻塞
	for _, er := range expanded {
		gxMin := int(math.Floor((er.MinX() - minX) / cellSize))
		gxMax := int(math.Ceil((er.MaxX()-minX)/cellSize)) - 1
		gyMin := int(math.Floor((er.MinY() - minY) / cellSize))
		gyMax := int(math.Ceil((er.MaxY()-minY)/cellSize)) - 1

		if gxMin < 0 {
			gxMin = 0
		}
		if gyMin < 0 {
			gyMin = 0
		}
		if gxMax >= gw {
			gxMax = gw - 1
		}
		if gyMax >= gh {
			gyMax = gh - 1
		}

		for gy := gyMin; gy <= gyMax; gy++ {
			// 验证格心 Y 是否在障碍物内
			centerY := minY + (float64(gy)+0.5)*cellSize
			if centerY <= er.MinY() || centerY >= er.MaxY() {
				continue
			}
			off := gy * gw
			for gx := gxMin; gx <= gxMax; gx++ {
				centerX := minX + (float64(gx)+0.5)*cellSize
				if centerX > er.MinX() && centerX < er.MaxX() {
					grid.blocked[off+gx] = true
				}
			}
		}
	}

	return grid
}

// findNearestWalkable 螺旋搜索最近可通行格子
func findNearestWalkable(g *localGrid, cx, cy int) (bool, int, int) {
	for r := 1; r <= 50; r++ {
		for dx := -r; dx <= r; dx++ {
			for _, dy := range [2]int{-r, r} {
				if g.isWalkable(cx+dx, cy+dy) {
					return true, cx + dx, cy + dy
				}
			}
		}
		for dy := -r + 1; dy <= r-1; dy++ {
			for _, dx := range [2]int{-r, r} {
				if g.isWalkable(cx+dx, cy+dy) {
					return true, cx + dx, cy + dy
				}
			}
		}
	}
	return false, 0, 0
}

// =========================================================================
// JPS (Jump Point Search) — 跳跃式搜索
// =========================================================================

var sqrt2 = math.Sqrt(2)

func iSign(v int) int {
	if v > 0 {
		return 1
	}
	if v < 0 {
		return -1
	}
	return 0
}

// octileDist 八方向启发距离
func octileDist(x1, y1, x2, y2 int) float64 {
	dx := math.Abs(float64(x2 - x1))
	dy := math.Abs(float64(y2 - y1))
	if dx > dy {
		return dy*sqrt2 + (dx - dy)
	}
	return dx*sqrt2 + (dy - dx)
}

// jumpStraight 沿直线方向跳跃（水平或垂直）
func (g *localGrid) jumpStraight(x, y, dx, dy, gx, gy int) (int, int, bool) {
	for {
		nx, ny := x+dx, y+dy
		if !g.isWalkable(nx, ny) {
			return 0, 0, false
		}
		if nx == gx && ny == gy {
			return nx, ny, true
		}
		if dx != 0 { // 水平
			if (!g.isWalkable(nx, ny+1) && g.isWalkable(nx+dx, ny+1)) ||
				(!g.isWalkable(nx, ny-1) && g.isWalkable(nx+dx, ny-1)) {
				return nx, ny, true
			}
		} else { // 垂直
			if (!g.isWalkable(nx+1, ny) && g.isWalkable(nx+1, ny+dy)) ||
				(!g.isWalkable(nx-1, ny) && g.isWalkable(nx-1, ny+dy)) {
				return nx, ny, true
			}
		}
		x, y = nx, ny
	}
}

// jumpDiag 沿对角线方向跳跃
func (g *localGrid) jumpDiag(x, y, dx, dy, gx, gy int) (int, int, bool) {
	for {
		nx, ny := x+dx, y+dy
		if !g.isWalkable(nx, ny) {
			return 0, 0, false
		}
		// 防止穿角：至少一个直线方向可通行
		if !g.isWalkable(x+dx, y) && !g.isWalkable(x, y+dy) {
			return 0, 0, false
		}
		if nx == gx && ny == gy {
			return nx, ny, true
		}
		// 强制邻居检查
		if (!g.isWalkable(nx-dx, ny) && g.isWalkable(nx-dx, ny+dy)) ||
			(!g.isWalkable(nx, ny-dy) && g.isWalkable(nx+dx, ny-dy)) {
			return nx, ny, true
		}
		// 沿两个分量方向递归跳跃
		if _, _, ok := g.jumpStraight(nx, ny, dx, 0, gx, gy); ok {
			return nx, ny, true
		}
		if _, _, ok := g.jumpStraight(nx, ny, 0, dy, gx, gy); ok {
			return nx, ny, true
		}
		x, y = nx, ny
	}
}

// prunedDirs 根据 JPS 裁剪规则返回需要搜索的方向
func (g *localGrid) prunedDirs(x, y, dx, dy int) [][2]int {
	dirs := make([][2]int, 0, 5)

	if dx != 0 && dy != 0 {
		// 对角线：自然邻居 + 强制邻居
		dirs = append(dirs, [2]int{dx, 0}, [2]int{0, dy}, [2]int{dx, dy})
		if !g.isWalkable(x-dx, y) {
			dirs = append(dirs, [2]int{-dx, dy})
		}
		if !g.isWalkable(x, y-dy) {
			dirs = append(dirs, [2]int{dx, -dy})
		}
	} else if dx != 0 {
		// 水平
		dirs = append(dirs, [2]int{dx, 0})
		if !g.isWalkable(x, y+1) {
			dirs = append(dirs, [2]int{dx, 1})
		}
		if !g.isWalkable(x, y-1) {
			dirs = append(dirs, [2]int{dx, -1})
		}
	} else {
		// 垂直
		dirs = append(dirs, [2]int{0, dy})
		if !g.isWalkable(x+1, y) {
			dirs = append(dirs, [2]int{1, dy})
		}
		if !g.isWalkable(x-1, y) {
			dirs = append(dirs, [2]int{-1, dy})
		}
	}
	return dirs
}

// ----- JPS 优先队列 -----

type jpsNode struct {
	x, y    int
	px, py  int // 父节点坐标 (-1,-1 = 起点)
	gCost   float64
	fCost   float64
	heapIdx int
}

type jpsHeap []*jpsNode

func (h jpsHeap) Len() int            { return len(h) }
func (h jpsHeap) Less(i, j int) bool  { return h[i].fCost < h[j].fCost }
func (h jpsHeap) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].heapIdx = i
	h[j].heapIdx = j
}
func (h *jpsHeap) Push(x any) {
	n := x.(*jpsNode)
	n.heapIdx = len(*h)
	*h = append(*h, n)
}
func (h *jpsHeap) Pop() any {
	old := *h
	n := len(old)
	item := old[n-1]
	old[n-1] = nil
	*h = old[:n-1]
	return item
}

// jpsSearch 在局部网格上执行 JPS A* 搜索
func (g *localGrid) jpsSearch(sx, sy, ex, ey int) [][2]int {
	if !g.isWalkable(sx, sy) || !g.isWalkable(ex, ey) {
		return nil
	}
	if sx == ex && sy == ey {
		return [][2]int{{sx, sy}}
	}

	type nk = [2]int
	gCosts := make(map[nk]float64)
	parents := make(map[nk]nk)
	closed := make(map[nk]bool)
	openRefs := make(map[nk]*jpsNode)

	h := &jpsHeap{}
	heap.Init(h)

	sk := nk{sx, sy}
	gCosts[sk] = 0
	sn := &jpsNode{x: sx, y: sy, px: -1, py: -1, gCost: 0, fCost: octileDist(sx, sy, ex, ey)}
	heap.Push(h, sn)
	openRefs[sk] = sn

	for h.Len() > 0 {
		cur := heap.Pop(h).(*jpsNode)
		ck := nk{cur.x, cur.y}

		if closed[ck] {
			continue
		}
		closed[ck] = true
		delete(openRefs, ck)

		// 到达目标
		if cur.x == ex && cur.y == ey {
			var path [][2]int
			k := nk{ex, ey}
			for {
				path = append(path, k)
				pk, ok := parents[k]
				if !ok {
					break
				}
				k = pk
			}
			for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
				path[i], path[j] = path[j], path[i]
			}
			return path
		}

		// 获取搜索方向
		var dirs [][2]int
		if cur.px == -1 && cur.py == -1 {
			dirs = [][2]int{
				{-1, -1}, {-1, 0}, {-1, 1},
				{0, -1}, {0, 1},
				{1, -1}, {1, 0}, {1, 1},
			}
		} else {
			dx := iSign(cur.x - cur.px)
			dy := iSign(cur.y - cur.py)
			dirs = g.prunedDirs(cur.x, cur.y, dx, dy)
		}

		// 沿各方向跳跃
		for _, d := range dirs {
			var jx, jy int
			var ok bool
			if d[0] != 0 && d[1] != 0 {
				jx, jy, ok = g.jumpDiag(cur.x, cur.y, d[0], d[1], ex, ey)
			} else {
				jx, jy, ok = g.jumpStraight(cur.x, cur.y, d[0], d[1], ex, ey)
			}
			if !ok {
				continue
			}

			jk := nk{jx, jy}
			if closed[jk] {
				continue
			}

			newG := cur.gCost + octileDist(cur.x, cur.y, jx, jy)
			if oldG, found := gCosts[jk]; found && newG >= oldG {
				continue
			}

			gCosts[jk] = newG
			parents[jk] = ck
			f := newG + octileDist(jx, jy, ex, ey)

			if ref, found := openRefs[jk]; found {
				ref.gCost = newG
				ref.fCost = f
				ref.px = cur.x
				ref.py = cur.y
				heap.Fix(h, ref.heapIdx)
			} else {
				node := &jpsNode{x: jx, y: jy, px: cur.x, py: cur.y, gCost: newG, fCost: f}
				heap.Push(h, node)
				openRefs[jk] = node
			}
		}
	}

	return nil
}

// =========================================================================
// interpolateGridPath 将 JPS 的跳跃路径插值为逐格路径
// JPS 返回的相邻点可能相隔很多格，需要插值才能正确平滑
// =========================================================================

func interpolateGridPath(gridPath [][2]int) [][2]int {
	if len(gridPath) <= 1 {
		return gridPath
	}
	result := make([][2]int, 0, len(gridPath)*2)
	result = append(result, gridPath[0])

	for i := 1; i < len(gridPath); i++ {
		prev := gridPath[i-1]
		cur := gridPath[i]
		dx := iSign(cur[0] - prev[0])
		dy := iSign(cur[1] - prev[1])
		x, y := prev[0], prev[1]
		for x != cur[0] || y != cur[1] {
			x += dx
			y += dy
			result = append(result, [2]int{x, y})
		}
	}
	return result
}

// =========================================================================
// findPathGrid — 正方形移动者的网格寻路入口
// =========================================================================

func (pf *PathFinder) findPathGrid(start, end Vec2, obstacles []*Obstacle, includeMoving bool) []Vec2 {
	halfW := pf.MoverW / 2
	halfH := pf.MoverH / 2

	// 1. 预计算 Minkowski 展开障碍物（完整列表，不做过滤）
	expanded := make([]Rect, 0, len(obstacles))
	for _, obs := range obstacles {
		if obs.Moving && !includeMoving {
			continue
		}
		expanded = append(expanded, obs.Rect.Expand(halfW, halfH))
	}

	// 2. 终点合法性
	for _, r := range expanded {
		if r.ContainsPoint(end) {
			return nil
		}
	}

	// 3. 直线可达
	if start.Equals(end) {
		return []Vec2{start}
	}
	if !segmentBlockedByAny(start, end, expanded) {
		return []Vec2{start, end}
	}

	// 4. 构建局部网格（格子大小 = 移动者半径）
	grid := buildLocalGrid(start, end, expanded, pf.Bounds, halfW)

	// 5. 转换起终点到网格坐标
	sx, sy := grid.worldToGrid(start.X, start.Y)
	ex, ey := grid.worldToGrid(end.X, end.Y)

	clamp := func(v, mx int) int {
		if v < 0 {
			return 0
		}
		if v >= mx {
			return mx - 1
		}
		return v
	}
	sx = clamp(sx, grid.w)
	sy = clamp(sy, grid.h)
	ex = clamp(ex, grid.w)
	ey = clamp(ey, grid.h)

	// 起终点被阻塞则找最近可通行格子
	if grid.isBlocked(sx, sy) {
		ok, nx, ny := findNearestWalkable(grid, sx, sy)
		if !ok {
			return nil
		}
		sx, sy = nx, ny
	}
	if grid.isBlocked(ex, ey) {
		ok, nx, ny := findNearestWalkable(grid, ex, ey)
		if !ok {
			return nil
		}
		ex, ey = nx, ny
	}

	// 同格 → 回退到可视图
	if sx == ex && sy == ey {
		return nil
	}

	// 6. JPS 搜索
	gridPath := grid.jpsSearch(sx, sy, ex, ey)
	if gridPath == nil {
		return nil
	}

	// 7. 插值跳跃路径为逐格路径，再转世界坐标
	interpPath := interpolateGridPath(gridPath)
	worldPath := make([]Vec2, len(interpPath))
	for i, gp := range interpPath {
		worldPath[i] = grid.gridToWorld(gp[0], gp[1])
	}
	// 强制首尾为精确的起终点
	worldPath[0] = start
	worldPath[len(worldPath)-1] = end

	// 8. 路径平滑（使用完整障碍物列表）
	if len(expanded) > 0 {
		cs := estimateCellSize(expanded)
		sh := newSpatialHash(cs, expanded)
		sc := &segChecker{
			expanded: expanded,
			sh:       sh,
			visited:  make([]int, len(expanded)),
		}
		worldPath = smoothPath(worldPath, sc)
	}

	return worldPath
}
