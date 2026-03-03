package world

import (
	"pathtest/pathfinding"
)

// findCollidingObstacle 查找与矩形 r 碰撞的障碍物（排除自身）
func (w *World) findCollidingObstacle(r pathfinding.Rect, selfID int) *pathfinding.Obstacle {
	for _, obs := range w.obstacles {
		if obs.ID == selfID {
			continue
		}
		if r.Overlaps(obs.Rect) {
			return obs
		}
	}
	return nil
}

// overlapsAnyObstacle 检查矩形 r 是否与除自身以外的任何障碍物重叠
func (w *World) overlapsAnyObstacle(r pathfinding.Rect, selfID int) bool {
	for _, obs := range w.obstacles {
		if obs.ID == selfID {
			continue
		}
		if r.Overlaps(obs.Rect) {
			return true
		}
	}
	return false
}
