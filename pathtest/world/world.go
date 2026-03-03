package world

import (
	"math/rand"
	"pathtest/pathfinding"
)

// ---------------------------------------------------------------------------
// World 世界状态，管理所有障碍物、移动逻辑和后台寻路
// 所有状态访问通过 cmdCh 串行化，无需 sync.Mutex
// ---------------------------------------------------------------------------

type World struct {
	cmdCh      chan func() // 命令通道，保证串行访问
	obstacles  []*pathfinding.Obstacle
	movingInfo map[int]*MovingObsInfo
	mapW, mapH float64
	active     bool
	nowMs      int64 // 系统运行时间（毫秒）
	pathLogs   []PathLogEntry
	repathCh   chan int
	epoch      int

	// 性能统计收集器（低耦合独立模块）
	stats *PathStatsCollector

	// 静态可见性图缓存（预计算优化）
	staticGraphCache *pathfinding.StaticGraphCache
	staticObstacles  []*pathfinding.Obstacle // 静态障碍物列表（用于寻路）
}

// New 创建世界实例并启动主循环和后台寻路协程
func New(workers int) *World {
	w := &World{
		cmdCh:    make(chan func(), 64),
		repathCh: make(chan int, 30),
		stats:    NewPathStatsCollector(20), // 最多保存20条慢寻路记录
	}
	go w.run()
	for i := 0; i < workers; i++ {
		go w.repathWorker()
	}
	return w
}

// run 世界主循环，所有状态访问通过此 goroutine 串行执行
func (w *World) run() {
	for cmd := range w.cmdCh {
		cmd()
	}
}

// do 在世界主循环中同步执行函数
func (w *World) do(fn func()) {
	done := make(chan struct{})
	w.cmdCh <- func() {
		fn()
		close(done)
	}
	<-done
}

// GetSlowPathRecords 获取慢寻路记录
func (w *World) GetSlowPathRecords() []SlowPathRecord {
	var records []SlowPathRecord
	w.do(func() {
		records = w.stats.GetSlowRecords()
	})
	return records
}

// ---------------------------------------------------------------------------
// Init 初始化/重置世界
// ---------------------------------------------------------------------------

type InitObstacle struct {
	CX, CY float64
	W, H   float64
	ID     int
	Moving bool
}

func (w *World) Init(mapW, mapH float64, obs []InitObstacle) {
	w.do(func() {
		// 排空旧的寻路队列
	drainLoop:
		for {
			select {
			case <-w.repathCh:
			default:
				break drainLoop
			}
		}

		obstacles := make([]*pathfinding.Obstacle, len(obs))
		movingInfo := make(map[int]*MovingObsInfo)
		for i, o := range obs {
			obstacles[i] = &pathfinding.Obstacle{
				Rect:   pathfinding.NewRect(o.CX, o.CY, o.W, o.H),
				ID:     o.ID,
				Moving: o.Moving,
			}
			if o.Moving {
				movingInfo[o.ID] = &MovingObsInfo{
					Speed:   300 + rand.Float64()*2700,
					Arrived: true, // 触发首次寻路
				}
			}
		}

		w.obstacles = obstacles
		w.movingInfo = movingInfo
		w.mapW = mapW
		w.mapH = mapH
		w.active = true

		// 检测并重定位与其他障碍物重叠的移动障碍物
		for _, obs := range obstacles {
			if obs.Moving {
				w.relocateIfOverlapping(obs)
			}
		}
		w.nowMs = 0
		w.pathLogs = nil
		w.epoch++

		// 重置统计收集器
		w.stats = NewPathStatsCollector(20)

		// 提取静态障碍物并预构建静态可见性图
		staticObs := make([]*pathfinding.Obstacle, 0, len(obstacles))
		for _, obs := range obstacles {
			if !obs.Moving {
				staticObs = append(staticObs, obs)
			}
		}
		w.staticObstacles = staticObs

		// 预构建常用移动者尺寸的静态图（正方形：32, 64, 128, 256）
		bounds := pathfinding.MapBounds{MinX: 0, MinY: 0, MaxX: mapW, MaxY: mapH}
		w.staticGraphCache = pathfinding.NewStaticGraphCache(bounds)
		if len(staticObs) > 0 {
			commonSizes := []float64{32, 64, 128, 256}
			w.staticGraphCache.PreBuildAll(staticObs, commonSizes)
		}
	})
}

// ---------------------------------------------------------------------------
// Tick 推进世界一帧 (需外部加锁或内部加锁)
// ---------------------------------------------------------------------------

func (w *World) Tick() {
	w.do(func() {
		if !w.active {
			return
		}
		w.tick(20) // 固定50帧，每帧20ms
	})
}

// TickAndCollect 推进一帧并收集移动物体状态及统计信息
func (w *World) TickAndCollect() (active bool, movingObs []MovingObsFullInfo, logs []PathLogEntry, stats PathStats) {
	w.do(func() {
		if !w.active {
			return
		}
		w.tick(20) // 固定50帧，每帧20ms
		active = true
		movingObs = w.getMovingObsFull()
		
		// 返回并清空日志
		logs = make([]PathLogEntry, len(w.pathLogs))
		copy(logs, w.pathLogs)
		w.pathLogs = nil

		// 返回统计信息
		stats = w.stats.GetStats()
	})
	return
}

// ---------------------------------------------------------------------------
// 查询接口
// ---------------------------------------------------------------------------

// Active 返回世界是否处于活动状态
func (w *World) Active() bool {
	var result bool
	w.do(func() {
		result = w.active
	})
	return result
}

// Obstacles 返回当前障碍物列表的快照引用（调用方不应修改）
func (w *World) Obstacles() []*pathfinding.Obstacle {
	var result []*pathfinding.Obstacle
	w.do(func() {
		result = w.obstacles
	})
	return result
}

// FilterNearbyObstacles 返回起点→终点走廊附近的障碍物
func (w *World) FilterNearbyObstacles(start, end pathfinding.Vec2, selfID int) []*pathfinding.Obstacle {
	var result []*pathfinding.Obstacle
	w.do(func() {
		result = w.filterNearbyObstacles(start, end, selfID)
	})
	return result
}

// MapSize 返回地图尺寸
func (w *World) MapSize() (float64, float64) {
	var mw, mh float64
	w.do(func() {
		mw = w.mapW
		mh = w.mapH
	})
	return mw, mh
}

// AddObstacle 动态添加单个障碍物，不会打断现有物体的移动逻辑
func (w *World) AddObstacle(cx, cy, width, height float64, id int, moving bool) {
	w.do(func() {
		if !w.active {
			return
		}

		newObs := &pathfinding.Obstacle{
			Rect:   pathfinding.NewRect(cx, cy, width, height),
			ID:     id,
			Moving: moving,
		}

		w.obstacles = append(w.obstacles, newObs)

		if moving {
			w.movingInfo[id] = &MovingObsInfo{
				Speed:   300 + rand.Float64()*2700,
				Arrived: true, // 触发首次寻路
			}
			// 检测并重定位如果与其他障碍物重叠
			w.relocateIfOverlapping(newObs)
		}
	})
}
