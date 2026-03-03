package world

import (
	"log"
	"math"
	"math/rand"
	"pathtest/pathfinding"
)

// ---------------------------------------------------------------------------
// filterNearbyObstacles 返回起点→终点走廊附近的障碍物（调用方需持锁）
// 策略：
//   - 所有静止障碍物始终包含
//   - 等待中的移动障碍物包含（视为静止）
//   - 正在移动的障碍物不包含（位置快速变化，通过碰撞检测处理）
// ---------------------------------------------------------------------------

func (w *World) filterNearbyObstacles(start, end pathfinding.Vec2, selfID int) []*pathfinding.Obstacle {
	margin := 5000.0
	minX := math.Min(start.X, end.X) - margin
	maxX := math.Max(start.X, end.X) + margin
	minY := math.Min(start.Y, end.Y) - margin
	maxY := math.Max(start.Y, end.Y) + margin

	result := make([]*pathfinding.Obstacle, 0, 100)
	for _, obs := range w.obstacles {
		if obs.ID == selfID {
			continue
		}

		// 静止障碍物始终包含，避免路径穿过
		if !obs.Moving {
			result = append(result, obs)
			continue
		}

		// 移动障碍物：仅包含等待中的（视为临时静止）
		// 正在移动的不包含，通过碰撞检测动态处理
		if obs.Rect.MaxX() >= minX && obs.Rect.MinX() <= maxX &&
			obs.Rect.MaxY() >= minY && obs.Rect.MinY() <= maxY {
			info := w.movingInfo[obs.ID]
			if info != nil && (info.Waiting || info.WaitingAtDest) {
				// 等待中的移动物体视为静止障碍物，让寻路器绕行
				clone := *obs
				clone.Moving = false
				result = append(result, &clone)
			}
			// 正在移动的障碍物不加入寻路，避免使用过期位置
		}
	}
	return result
}

// ---------------------------------------------------------------------------
// relocateIfOverlapping 检查移动障碍物是否与其他障碍物重叠，如果是则重定位
// ---------------------------------------------------------------------------

func (w *World) relocateIfOverlapping(obs *pathfinding.Obstacle) bool {
	testRect := pathfinding.Rect{
		CenterX: obs.Rect.CenterX, CenterY: obs.Rect.CenterY,
		HalfW: obs.Rect.HalfW, HalfH: obs.Rect.HalfH,
	}
	if !w.overlapsAnyObstacle(testRect, obs.ID) {
		return false
	}

	const numDirs = 16
	stepSize := math.Max(obs.Rect.HalfW, obs.Rect.HalfH) * 0.5
	if stepSize < 50 {
		stepSize = 50
	}

	dirs := [numDirs][2]float64{}
	for i := 0; i < numDirs; i++ {
		angle := float64(i) * 2 * math.Pi / numDirs
		dirs[i] = [2]float64{math.Cos(angle), math.Sin(angle)}
	}

	bestDist := math.MaxFloat64
	bestX, bestY := obs.Rect.CenterX, obs.Rect.CenterY
	found := false

	for radius := stepSize; radius <= 15000; radius += stepSize {
		for _, d := range dirs {
			cx := obs.Rect.CenterX + d[0]*radius
			cy := obs.Rect.CenterY + d[1]*radius

			cx = math.Max(obs.Rect.HalfW+10, math.Min(w.mapW-obs.Rect.HalfW-10, cx))
			cy = math.Max(obs.Rect.HalfH+10, math.Min(w.mapH-obs.Rect.HalfH-10, cy))

			candidateRect := pathfinding.Rect{
				CenterX: cx, CenterY: cy,
				HalfW: obs.Rect.HalfW, HalfH: obs.Rect.HalfH,
			}
			if !w.overlapsAnyObstacle(candidateRect, obs.ID) {
				dx := cx - obs.Rect.CenterX
				dy := cy - obs.Rect.CenterY
				dist := dx*dx + dy*dy
				if dist < bestDist {
					bestDist = dist
					bestX = cx
					bestY = cy
					found = true
				}
			}
		}
		if found {
			break
		}
	}

	if found {
		log.Printf("移动障碍物 M%d 与其他障碍物重叠，从 (%.0f,%.0f) 重定位到 (%.0f,%.0f)",
			obs.ID, obs.Rect.CenterX, obs.Rect.CenterY, bestX, bestY)
		obs.Rect.CenterX = bestX
		obs.Rect.CenterY = bestY
	} else {
		for attempt := 0; attempt < 100; attempt++ {
			cx := obs.Rect.HalfW + 200 + rand.Float64()*(w.mapW-obs.Rect.HalfW*2-400)
			cy := obs.Rect.HalfH + 200 + rand.Float64()*(w.mapH-obs.Rect.HalfH*2-400)
			candidateRect := pathfinding.Rect{
				CenterX: cx, CenterY: cy,
				HalfW: obs.Rect.HalfW, HalfH: obs.Rect.HalfH,
			}
			if !w.overlapsAnyObstacle(candidateRect, obs.ID) {
				log.Printf("移动障碍物 M%d 重叠严重，随机重定位到 (%.0f,%.0f)", obs.ID, cx, cy)
				obs.Rect.CenterX = cx
				obs.Rect.CenterY = cy
				found = true
				break
			}
		}
	}
	return found
}

// ---------------------------------------------------------------------------
// adjustTargetIfBlocked 检查目标点是否被障碍物覆盖，如果是则调整
// ---------------------------------------------------------------------------

func (w *World) adjustTargetIfBlocked(obs *pathfinding.Obstacle, info *MovingObsInfo) {
	targetRect := pathfinding.Rect{
		CenterX: info.TargetX, CenterY: info.TargetY,
		HalfW: obs.Rect.HalfW, HalfH: obs.Rect.HalfH,
	}

	blocked := false
	for _, o := range w.obstacles {
		if o.ID == obs.ID {
			continue
		}
		isStatic := !o.Moving
		isWaiting := false
		if o.Moving {
			if oi := w.movingInfo[o.ID]; oi != nil && (oi.Waiting || oi.WaitingAtDest) {
				isWaiting = true
			}
		}
		if (isStatic || isWaiting) && targetRect.Overlaps(o.Rect) {
			blocked = true
			break
		}
	}

	if !blocked {
		return
	}

	const numDirs = 16
	stepSize := math.Max(obs.Rect.HalfW, obs.Rect.HalfH) * 0.5
	if stepSize < 50 {
		stepSize = 50
	}

	dirs := [numDirs][2]float64{}
	for i := 0; i < numDirs; i++ {
		angle := float64(i) * 2 * math.Pi / numDirs
		dirs[i] = [2]float64{math.Cos(angle), math.Sin(angle)}
	}

	bestDist := math.MaxFloat64
	bestX, bestY := info.TargetX, info.TargetY
	found := false

	for radius := stepSize; radius <= 10000; radius += stepSize {
		for _, d := range dirs {
			cx := info.TargetX + d[0]*radius
			cy := info.TargetY + d[1]*radius

			cx = math.Max(obs.Rect.HalfW+10, math.Min(w.mapW-obs.Rect.HalfW-10, cx))
			cy = math.Max(obs.Rect.HalfH+10, math.Min(w.mapH-obs.Rect.HalfH-10, cy))

			candidateRect := pathfinding.Rect{
				CenterX: cx, CenterY: cy,
				HalfW: obs.Rect.HalfW, HalfH: obs.Rect.HalfH,
			}

			overlaps := false
			for _, o := range w.obstacles {
				if o.ID == obs.ID {
					continue
				}
				isStatic := !o.Moving
				isWaiting := false
				if o.Moving {
					if oi := w.movingInfo[o.ID]; oi != nil && (oi.Waiting || oi.WaitingAtDest) {
						isWaiting = true
					}
				}
				if (isStatic || isWaiting) && candidateRect.Overlaps(o.Rect) {
					overlaps = true
					break
				}
			}

			if !overlaps {
				dx := cx - info.TargetX
				dy := cy - info.TargetY
				dist := dx*dx + dy*dy
				if dist < bestDist {
					bestDist = dist
					bestX = cx
					bestY = cy
					found = true
				}
			}
		}
		if found {
			break
		}
	}

	if found {
		log.Printf("移动障碍物 M%d 目标点 (%.0f,%.0f) 被覆盖，调整到 (%.0f,%.0f)",
			obs.ID, info.TargetX, info.TargetY, bestX, bestY)
		info.TargetX = bestX
		info.TargetY = bestY
	} else {
		log.Printf("移动障碍物 M%d 目标点被覆盖但无法找到附近空位，保持原目标", obs.ID)
	}
}

// ---------------------------------------------------------------------------
// pickValidTarget 为移动障碍物随机选择一个不与障碍物重叠的目标
// ---------------------------------------------------------------------------

func (w *World) pickValidTarget(obs *pathfinding.Obstacle, info *MovingObsInfo) {
	maxRange := 28000.0
	halfW := obs.Rect.HalfW
	halfH := obs.Rect.HalfH
	for attempt := 0; attempt < 30; attempt++ {
		tx := obs.Rect.CenterX + (rand.Float64()*2-1)*maxRange
		ty := obs.Rect.CenterY + (rand.Float64()*2-1)*maxRange
		tx = math.Max(halfW+100, math.Min(w.mapW-halfW-100, tx))
		ty = math.Max(halfH+100, math.Min(w.mapH-halfH-100, ty))
		testRect := pathfinding.Rect{
			CenterX: tx, CenterY: ty,
			HalfW: halfW, HalfH: halfH,
		}
		if !w.overlapsAnyObstacle(testRect, obs.ID) {
			info.TargetX = tx
			info.TargetY = ty
			return
		}
	}
	tx := obs.Rect.CenterX + (rand.Float64()*2-1)*maxRange
	ty := obs.Rect.CenterY + (rand.Float64()*2-1)*maxRange
	info.TargetX = math.Max(halfW+100, math.Min(w.mapW-halfW-100, tx))
	info.TargetY = math.Max(halfH+100, math.Min(w.mapH-halfH-100, ty))
}
