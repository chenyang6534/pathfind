package world

import (
	"math"
	"pathtest/pathfinding"
	"time"
)

// repathWorker 后台寻路协程
// 正数ID = 到达目标后随机新目标; 负数ID = 保持当前目标重新寻路
// 通过 cmdCh 与主循环串行交互，分为读取、寻路、写回三个阶段
func (w *World) repathWorker() {
	for rawID := range w.repathCh {
		keepTarget := rawID < 0
		obsID := rawID
		if keepTarget {
			obsID = -rawID
		}

		// ---- Phase 1: 在主循环中读取状态、准备寻路参数 ----
		type prepData struct {
			skip             bool
			obs              *pathfinding.Obstacle
			info             *MovingObsInfo
			nearbyObs        []*pathfinding.Obstacle
			start, end       pathfinding.Vec2
			dimW, dimH       float64
			mapW, mapH       float64
			staticGraphCache *pathfinding.StaticGraphCache
			staticObs        []*pathfinding.Obstacle
		}

		prepCh := make(chan prepData, 1)
		w.cmdCh <- func() {
			if !w.active {
				prepCh <- prepData{skip: true}
				return
			}

			var obs *pathfinding.Obstacle
			for _, o := range w.obstacles {
				if o.ID == obsID {
					obs = o
					break
				}
			}
			info := w.movingInfo[obsID]
			if obs == nil || info == nil {
				prepCh <- prepData{skip: true}
				return
			}

			w.relocateIfOverlapping(obs)

			if !keepTarget || (info.TargetX == 0 && info.TargetY == 0) {
				w.pickValidTarget(obs, info)
			}
			w.adjustTargetIfBlocked(obs, info)

			start := pathfinding.Vec2{X: obs.Rect.CenterX, Y: obs.Rect.CenterY}
			end := pathfinding.Vec2{X: info.TargetX, Y: info.TargetY}
			nearbyObs := w.filterNearbyObstacles(start, end, obs.ID)

			// 对附近障碍物做快照，避免寻路期间 tick 修改坐标造成并发读写
			snapshotObs := make([]*pathfinding.Obstacle, len(nearbyObs))
			for i, o := range nearbyObs {
				clone := *o
				snapshotObs[i] = &clone
			}

			prepCh <- prepData{
				obs:              obs,
				info:             info,
				nearbyObs:        snapshotObs,
				start:            start,
				end:              end,
				dimW:             obs.Rect.HalfW * 2,
				dimH:             obs.Rect.HalfH * 2,
				mapW:             w.mapW,
				mapH:             w.mapH,
				staticGraphCache: w.staticGraphCache,
				staticObs:        w.staticObstacles,
			}
		}
		r := <-prepCh
		if r.skip {
			continue
		}

		// ---- Phase 2: 寻路（不在主循环中，不阻塞 tick） ----
		t0 := time.Now()
		bounds := pathfinding.MapBounds{MinX: 0, MinY: 0, MaxX: r.mapW, MaxY: r.mapH}
		pf := pathfinding.NewPathFinder(bounds, r.dimW, r.dimH)

		var path []pathfinding.Vec2

		// 尝试使用静态图加速寻路
		if r.staticGraphCache != nil && len(r.staticObs) > 0 {
			// 获取或构建对应尺寸的静态图
			staticGraph := r.staticGraphCache.GetOrBuild(r.staticObs, r.dimW, r.dimH)

			// 过滤出动态障碍物：
			// - nearbyObs 中 Moving=true 的是真正的动态障碍物
			// - nearbyObs 中 Moving=false 但不在 staticObs 中的是等待中的移动障碍物（被临时标记为静止）
			dynamicObs := make([]*pathfinding.Obstacle, 0, len(r.nearbyObs))
			staticIDSet := make(map[int]struct{}, len(r.staticObs))
			for _, obs := range r.staticObs {
				staticIDSet[obs.ID] = struct{}{}
			}
			for _, obs := range r.nearbyObs {
				if obs.Moving {
					// 正在移动的障碍物
					dynamicObs = append(dynamicObs, obs)
				} else if _, isStatic := staticIDSet[obs.ID]; !isStatic {
					// Moving=false 但不在静态列表中 → 等待中的移动障碍物
					dynamicObs = append(dynamicObs, obs)
				}
			}

			path = pf.FindPathWithStaticGraph(r.start, r.end, dynamicObs, staticGraph)
		} else {
			// 回退到原始寻路
			path = pf.FindPath(r.start, r.end, r.nearbyObs, false)
		}

		elapsed := time.Since(t0).Seconds() * 1000

		// 计算可视图节点数（估算：起点+终点+障碍物角点）
		nodeCount := 2 + len(r.nearbyObs)*4

		// ---- Phase 3: 在主循环中写回结果 ----
		done := make(chan struct{})
		w.cmdCh <- func() {
			defer close(done)

			logEntry := PathLogEntry{
				ID:     obsID,
				TimeMs: math.Round(elapsed*100) / 100,
			}
			if path != nil {
				r.info.Path = path
				r.info.PathIdx = 0
				logEntry.Nodes = len(path)
				logEntry.Success = true
				r.info.Arrived = false
			} else {
				// 寻路失败：立即选择新的随机目标（无需等待）
				logEntry.Success = false
				r.info.Path = nil
				r.info.PathIdx = 0
				r.info.Arrived = true // 立即触发新目标选择
				r.info.Waiting = false
			}
			r.info.LastPathTimeMs = logEntry.TimeMs
			r.info.Repathing = false

			// 记录寻路性能统计（使用独立的统计收集器）
			w.stats.RecordPath(
				obsID,
				elapsed,
				path != nil,
				r.start.X, r.start.Y,
				r.end.X, r.end.Y,
				len(r.nearbyObs),
				nodeCount,
				len(path),
			)

			// 只记录耗时超过0.3ms的寻路
			if logEntry.TimeMs > 0.3 {
				w.pathLogs = append(w.pathLogs, logEntry)
			}

			// 每5秒打印一次统计
			if w.stats.ShouldPrint(w.nowMs) {
				w.stats.PrintStats(w.nowMs)
			}
		}
		<-done
	}
}
