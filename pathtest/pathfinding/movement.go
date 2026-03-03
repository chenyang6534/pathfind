package pathfinding

// =========================================================================
// Mover — 移动控制器
// 负责沿寻路结果移动矩形实体，并在运行时动态处理碰撞：
//   - 碰到正在移动的障碍物 → 等待一小段时间后重试
//   - 碰到静止障碍物 → 立即重新寻路
// =========================================================================

// MoverState 移动者当前状态
type MoverState int

const (
	StateIdle    MoverState = iota // 空闲/已到达
	StateMoving                    // 正在沿路径移动
	StateWaiting                   // 等待移动障碍物让路
)

// MoveResult 单次 Update 的返回结果
type MoveResult int

const (
	ResultInProgress MoveResult = iota // 仍在移动/等待中
	ResultArrived                      // 已到达目的地
	ResultNoPath                       // 找不到可用路径
	ResultIdle                         // 当前没有移动任务
)

// MoverConfig 移动者行为配置
type MoverConfig struct {
	WaitDuration float64 // 每次等待检查间隔（秒）
	MaxRetries   int     // 等待最大重试次数，超过后尝试重新寻路
}

// DefaultMoverConfig 默认配置
var DefaultMoverConfig = MoverConfig{
	WaitDuration: 0.3,
	MaxRetries:   8,
}

// Mover 可移动的矩形实体。
// 使用方式：
//  1. 调用 SetTarget 设置目标并计算初始路径
//  2. 每帧调用 Update(dt, obstacles) 推进移动
type Mover struct {
	Rect       Rect       // 当前位置和大小
	Speed      float64    // 移动速度（单位/秒）
	State      MoverState // 当前状态（只读）
	PathFinder *PathFinder

	path       []Vec2  // 当前路径（中心坐标序列）
	pathIdx    int     // 当前路径段索引（正在从 path[pathIdx] 移向 path[pathIdx+1]）
	target     Vec2    // 最终目标点
	waitTimer  float64 // 等待累积时间
	retryCount int     // 当前连续等待次数
	config     MoverConfig
}

// NewMover 创建移动者。可选传入自定义配置，否则使用默认配置。
func NewMover(rect Rect, speed float64, pf *PathFinder, config ...MoverConfig) *Mover {
	cfg := DefaultMoverConfig
	if len(config) > 0 {
		cfg = config[0]
	}
	return &Mover{
		Rect:       rect,
		Speed:      speed,
		State:      StateIdle,
		PathFinder: pf,
		config:     cfg,
	}
}

// SetTarget 设置目标并计算路径。成功返回 true。
// 路径计算时默认忽略正在移动的障碍物。
func (m *Mover) SetTarget(target Vec2, obstacles []*Obstacle) bool {
	path := m.PathFinder.FindPath(m.Rect.Center(), target, obstacles, false)
	if path == nil {
		return false
	}
	m.path = path
	m.pathIdx = 0
	m.target = target
	m.State = StateMoving
	m.retryCount = 0
	m.waitTimer = 0
	return true
}

// GetPath 获取当前路径副本（调试/可视化用）。
func (m *Mover) GetPath() []Vec2 {
	cp := make([]Vec2, len(m.path))
	copy(cp, m.path)
	return cp
}

// IsMoving 是否正在执行移动任务。
func (m *Mover) IsMoving() bool {
	return m.State == StateMoving || m.State == StateWaiting
}

// Update 每帧调用以推进移动。
//   - dt: 本帧时间增量（秒）
//   - obstacles: 当前所有障碍物（含移动中的），用于碰撞检测
func (m *Mover) Update(dt float64, obstacles []*Obstacle) MoveResult {
	switch m.State {
	case StateIdle:
		return ResultIdle
	case StateWaiting:
		return m.updateWaiting(dt, obstacles)
	case StateMoving:
		return m.updateMoving(dt, obstacles)
	}
	return ResultIdle
}

// ---------- 等待状态更新 ----------
func (m *Mover) updateWaiting(dt float64, obstacles []*Obstacle) MoveResult {
	m.waitTimer += dt
	if m.waitTimer < m.config.WaitDuration {
		return ResultInProgress
	}
	m.waitTimer = 0

	// 检查当前路径段是否仍被阻塞
	blocker := m.findBlockerOnSegment(obstacles)
	if blocker == nil {
		// 障碍物已移开，恢复移动
		m.State = StateMoving
		m.retryCount = 0
		return ResultInProgress
	}

	if blocker.Moving {
		m.retryCount++
		if m.retryCount > m.config.MaxRetries {
			// 等待超时：将移动障碍物也纳入寻路计算，寻找替代路线
			return m.repath(obstacles, true)
		}
		return ResultInProgress // 继续等待
	}

	// 被静止障碍物阻塞 → 重新寻路
	return m.repath(obstacles, false)
}

// ---------- 移动状态更新 ----------
func (m *Mover) updateMoving(dt float64, obstacles []*Obstacle) MoveResult {
	if m.pathIdx >= len(m.path)-1 {
		m.State = StateIdle
		return ResultArrived
	}

	moveAmount := m.Speed * dt

	for moveAmount > epsilon && m.pathIdx < len(m.path)-1 {
		pos := m.Rect.Center()
		nextWP := m.path[m.pathIdx+1]
		dir := nextWP.Sub(pos)
		dist := dir.Len()

		if dist < epsilon {
			m.pathIdx++
			continue
		}

		// 计算本子步的目标位置
		var stepTarget Vec2
		var reachedWP bool
		if moveAmount >= dist {
			stepTarget = nextWP
			reachedWP = true
		} else {
			normalized := dir.Normalize()
			stepTarget = pos.Add(normalized.Scale(moveAmount))
			reachedWP = false
		}

		// ★ 碰撞检测 1: 线段扫掠检查（Minkowski 扩展后的障碍物与移动路径的相交）
		blocker := m.checkSegmentCollision(pos, stepTarget, obstacles)

		// ★ 碰撞检测 2: 目标位置的 AABB 重叠检查
		if blocker == nil {
			stepRect := Rect{stepTarget.X, stepTarget.Y, m.Rect.HalfW, m.Rect.HalfH}
			blocker = FindOverlappingObstacle(stepRect, obstacles)
		}

		if blocker != nil {
			if blocker.Moving {
				// 碰到移动障碍物 → 进入等待状态
				m.State = StateWaiting
				m.waitTimer = 0
				m.retryCount = 0
				return ResultInProgress
			}
			// 碰到静止障碍物 → 立即重新寻路
			return m.repath(obstacles, false)
		}

		// 安全移动到 stepTarget
		m.Rect.CenterX = stepTarget.X
		m.Rect.CenterY = stepTarget.Y

		if reachedWP {
			moveAmount -= dist
			m.pathIdx++
		} else {
			moveAmount = 0
		}
	}

	if m.pathIdx >= len(m.path)-1 {
		m.State = StateIdle
		return ResultArrived
	}
	return ResultInProgress
}

// checkSegmentCollision 检查移动者从 from 移动到 to 时是否与任何障碍物碰撞。
// 将障碍物做 Minkowski 扩展（加上移动者半尺寸），再用线段相交检测。
func (m *Mover) checkSegmentCollision(from, to Vec2, obstacles []*Obstacle) *Obstacle {
	for _, obs := range obstacles {
		expanded := obs.Rect.Expand(m.Rect.HalfW, m.Rect.HalfH)
		if SegmentIntersectsRect(from, to, expanded) {
			return obs
		}
	}
	return nil
}

// ---------- 检查当前路径段的阻塞情况 ----------
func (m *Mover) findBlockerOnSegment(obstacles []*Obstacle) *Obstacle {
	if m.pathIdx >= len(m.path)-1 {
		return nil
	}
	pos := m.Rect.Center()
	next := m.path[m.pathIdx+1]

	for _, obs := range obstacles {
		expanded := obs.Rect.Expand(m.Rect.HalfW, m.Rect.HalfH)
		if SegmentIntersectsRect(pos, next, expanded) {
			return obs
		}
	}
	return nil
}

// ---------- 重新寻路 ----------
func (m *Mover) repath(obstacles []*Obstacle, includeMoving bool) MoveResult {
	path := m.PathFinder.FindPath(m.Rect.Center(), m.target, obstacles, includeMoving)
	if path == nil {
		m.State = StateIdle
		return ResultNoPath
	}
	m.path = path
	m.pathIdx = 0
	m.State = StateMoving
	m.retryCount = 0
	m.waitTimer = 0
	return ResultInProgress
}
