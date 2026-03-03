package world

import (
	"math"
	"math/rand"
	"pathtest/pathfinding"
)

// tick 推进移动障碍物一帧（内部方法，调用方需持锁）
// dtMs 为帧时间（毫秒），固定50帧=20ms
func (w *World) tick(dtMs int64) {
	w.nowMs += dtMs
	dt := float64(dtMs) / 1000.0 // 转换为秒，用于速度计算

	for _, obs := range w.obstacles {
		if !obs.Moving {
			continue
		}
		info, ok := w.movingInfo[obs.ID]
		if !ok {
			continue
		}

		// 等待状态检查
		if info.Waiting || info.WaitingAtDest {
			if w.nowMs < info.WaitUntilMs {
				continue // 继续等待
			}
			// 等待结束
			if info.WaitingAtDest {
				info.WaitingAtDest = false
				info.Arrived = true
			} else {
				info.Waiting = false
				info.NeedRepath = true
			}
		}

		// 需要重新寻路（保持目标）
		if info.NeedRepath {
			if !info.Repathing {
				info.Repathing = true
				info.NeedRepath = false
				select {
				case w.repathCh <- -obs.ID: // 负数 = 保持目标
				default:
					// 队列满了，先等待一会儿再重试
					info.Repathing = false
					info.Waiting = true
					info.WaitUntilMs = w.nowMs + 100 // 等待100ms后重试
					info.NeedRepath = false // 等待结束后会自动重试
				}
			}
			continue
		}

		// 如果已到达或无路径，加入后台寻路队列（新随机目标）
		if info.Arrived || len(info.Path) == 0 {
			if !info.Repathing {
				info.Repathing = true
				select {
				case w.repathCh <- obs.ID: // 正数 = 新目标
				default:
					// 队列满了，先等待一会儿再重试
					info.Repathing = false
					info.Waiting = true
					info.WaitUntilMs = w.nowMs + 200 // 等待200ms后重试
				}
			}
			continue
		}

		// 沿路径移动
		step := info.Speed * dt
		for step > 1e-6 && info.PathIdx < len(info.Path)-1 {
			nextPt := info.Path[info.PathIdx+1]
			dx := nextPt.X - obs.Rect.CenterX
			dy := nextPt.Y - obs.Rect.CenterY
			dist := math.Sqrt(dx*dx + dy*dy)

			if dist < 1e-6 {
				info.PathIdx++
				continue
			}

			var stepTarget pathfinding.Vec2
			var reachedWP bool
			if step >= dist {
				stepTarget = nextPt
				reachedWP = true
			} else {
				stepTarget = pathfinding.Vec2{
					X: obs.Rect.CenterX + (dx/dist)*step,
					Y: obs.Rect.CenterY + (dy/dist)*step,
				}
				reachedWP = false
			}

			// 碰撞检测
			testRect := pathfinding.Rect{
				CenterX: stepTarget.X, CenterY: stepTarget.Y,
				HalfW: obs.Rect.HalfW, HalfH: obs.Rect.HalfH,
			}
			hitObs := w.findCollidingObstacle(testRect, obs.ID)
			if hitObs != nil {
				hitInfo := w.movingInfo[hitObs.ID]
				hitIsWaitingOrStatic := !hitObs.Moving || (hitInfo != nil && (hitInfo.Waiting || hitInfo.WaitingAtDest))

				if hitObs.Moving && !hitIsWaitingOrStatic {
					// A 碰到正在移动的 B：A 停下等待
					waitMs := int64(300 + rand.Intn(501))
					info.Waiting = true
					info.WaitUntilMs = w.nowMs + waitMs
					info.Path = nil
				} else {
					// 碰到静止物体或等待中的物体 → 立即重新寻路，保持目标
					info.NeedRepath = true
					info.Path = nil
				}
				break
			}

			obs.Rect.CenterX = stepTarget.X
			obs.Rect.CenterY = stepTarget.Y

			if reachedWP {
				step -= dist
				info.PathIdx++
			} else {
				step = 0
			}
		}

		// 检查是否到达终点
		if info.PathIdx >= len(info.Path)-1 && !info.Waiting && !info.WaitingAtDest && !info.NeedRepath {
			waitMs := int64(1000 + rand.Intn(1001))
			info.WaitingAtDest = true
			info.WaitUntilMs = w.nowMs + waitMs
			info.Path = nil
		}
	}
}

// getMovingObsFull 返回所有移动障碍物的完整信息（调用方需持锁）
func (w *World) getMovingObsFull() []MovingObsFullInfo {
	result := make([]MovingObsFullInfo, 0, len(w.movingInfo))
	for _, obs := range w.obstacles {
		if !obs.Moving {
			continue
		}
		info, ok := w.movingInfo[obs.ID]
		if !ok {
			continue
		}
		mj := MovingObsFullInfo{
			ID: obs.ID, CX: obs.Rect.CenterX, CY: obs.Rect.CenterY,
			TargetX: info.TargetX, TargetY: info.TargetY,
			WaitingAtDest: info.WaitingAtDest,
		}
		if info.WaitingAtDest {
			remaining := info.WaitUntilMs - w.nowMs
			if remaining < 0 {
				remaining = 0
			}
			mj.WaitRemainingMs = float64(remaining)
		}
		if len(info.Path) > 0 && info.PathIdx < len(info.Path) {
			remaining := info.Path[info.PathIdx:]
			pathJSON := make([]Vec2JSON, len(remaining))
			for i, p := range remaining {
				pathJSON[i] = Vec2JSON{X: p.X, Y: p.Y}
			}
			mj.Path = pathJSON
		}
		result = append(result, mj)
	}
	return result
}
