package pathfinding

import "math"

// SegmentIntersectsRect 检查线段 p1→p2 是否穿过矩形 r 的严格内部。
// 使用 Slab(平板) 方法，仅边缘擦过或角点接触不算相交。
// 内部将矩形微缩一个 margin 以避免浮点精度问题。
func SegmentIntersectsRect(p1, p2 Vec2, r Rect) bool {
	const margin = 1e-6

	rMinX := r.MinX() + margin
	rMaxX := r.MaxX() - margin
	rMinY := r.MinY() + margin
	rMaxY := r.MaxY() - margin

	// 矩形过小则忽略
	if rMinX >= rMaxX || rMinY >= rMaxY {
		return false
	}

	dx := p2.X - p1.X
	dy := p2.Y - p1.Y

	tMin := 0.0
	tMax := 1.0

	// ---------- X 轴 slab ----------
	if math.Abs(dx) > epsilon {
		invD := 1.0 / dx
		t1 := (rMinX - p1.X) * invD
		t2 := (rMaxX - p1.X) * invD
		if t1 > t2 {
			t1, t2 = t2, t1
		}
		if t1 > tMin {
			tMin = t1
		}
		if t2 < tMax {
			tMax = t2
		}
		if tMin >= tMax {
			return false
		}
	} else {
		// 线段几乎平行于 Y 轴
		if p1.X <= rMinX || p1.X >= rMaxX {
			return false
		}
	}

	// ---------- Y 轴 slab ----------
	if math.Abs(dy) > epsilon {
		invD := 1.0 / dy
		t1 := (rMinY - p1.Y) * invD
		t2 := (rMaxY - p1.Y) * invD
		if t1 > t2 {
			t1, t2 = t2, t1
		}
		if t1 > tMin {
			tMin = t1
		}
		if t2 < tMax {
			tMax = t2
		}
		if tMin >= tMax {
			return false
		}
	} else {
		if p1.Y <= rMinY || p1.Y >= rMaxY {
			return false
		}
	}

	return tMin < tMax
}

// FindOverlappingObstacle 查找与矩形 r 严格重叠的第一个障碍物。
// 未找到返回 nil。
func FindOverlappingObstacle(r Rect, obstacles []*Obstacle) *Obstacle {
	for _, obs := range obstacles {
		if r.Overlaps(obs.Rect) {
			return obs
		}
	}
	return nil
}
