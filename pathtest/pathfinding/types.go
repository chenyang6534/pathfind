// Package pathfinding 提供基于矩形碰撞的2D寻路算法。
// 核心思路：使用 Minkowski 和将矩形移动者简化为点，
// 构建可视图(Visibility Graph) + A* 算法寻路，支持动态避障。
package pathfinding

import "math"

const (
	epsilon      = 1e-9 // 浮点数比较精度
	cornerOffset = 0.5  // 可视图角点向外偏移量，避免路径擦边
)

// ---------------------------------------------------------------------------
// Vec2 二维向量/坐标点
// ---------------------------------------------------------------------------

type Vec2 struct {
	X, Y float64
}

func (v Vec2) Add(o Vec2) Vec2     { return Vec2{v.X + o.X, v.Y + o.Y} }
func (v Vec2) Sub(o Vec2) Vec2     { return Vec2{v.X - o.X, v.Y - o.Y} }
func (v Vec2) Scale(s float64) Vec2 { return Vec2{v.X * s, v.Y * s} }
func (v Vec2) Len() float64        { return math.Sqrt(v.X*v.X + v.Y*v.Y) }
func (v Vec2) LenSq() float64      { return v.X*v.X + v.Y*v.Y }
func (v Vec2) DistTo(o Vec2) float64 { return v.Sub(o).Len() }

// Normalize 返回单位向量；零向量返回零值。
func (v Vec2) Normalize() Vec2 {
	l := v.Len()
	if l < epsilon {
		return Vec2{}
	}
	return Vec2{v.X / l, v.Y / l}
}

// Equals 判断两个向量是否在精度范围内相等。
func (v Vec2) Equals(o Vec2) bool {
	return math.Abs(v.X-o.X) < epsilon && math.Abs(v.Y-o.Y) < epsilon
}

// ---------------------------------------------------------------------------
// Rect 轴对齐矩形 (AABB)，用中心 + 半宽半高表示
// ---------------------------------------------------------------------------

type Rect struct {
	CenterX, CenterY float64 // 中心坐标
	HalfW, HalfH     float64 // 半宽、半高（均 >= 0）
}

// NewRect 通过中心坐标和完整宽高创建矩形。
func NewRect(centerX, centerY, width, height float64) Rect {
	return Rect{centerX, centerY, width / 2, height / 2}
}

// NewRectFromBounds 通过左下角和右上角坐标创建矩形。
func NewRectFromBounds(minX, minY, maxX, maxY float64) Rect {
	return Rect{
		CenterX: (minX + maxX) / 2,
		CenterY: (minY + maxY) / 2,
		HalfW:   (maxX - minX) / 2,
		HalfH:   (maxY - minY) / 2,
	}
}

func (r Rect) MinX() float64  { return r.CenterX - r.HalfW }
func (r Rect) MaxX() float64  { return r.CenterX + r.HalfW }
func (r Rect) MinY() float64  { return r.CenterY - r.HalfH }
func (r Rect) MaxY() float64  { return r.CenterY + r.HalfH }
func (r Rect) Center() Vec2   { return Vec2{r.CenterX, r.CenterY} }
func (r Rect) Width() float64 { return r.HalfW * 2 }
func (r Rect) Height() float64 { return r.HalfH * 2 }

// Expand 返回通过 Minkowski 和扩展后的矩形。
// 等效于将移动者缩为点后，障碍物在各方向上外扩移动者的半尺寸。
func (r Rect) Expand(hw, hh float64) Rect {
	return Rect{r.CenterX, r.CenterY, r.HalfW + hw, r.HalfH + hh}
}

// ContainsPoint 严格内部包含检测（不含边界）。
func (r Rect) ContainsPoint(p Vec2) bool {
	return p.X > r.MinX()+epsilon && p.X < r.MaxX()-epsilon &&
		p.Y > r.MinY()+epsilon && p.Y < r.MaxY()-epsilon
}

// Overlaps 严格重叠检测（仅边缘接触不算）。
func (r Rect) Overlaps(o Rect) bool {
	return r.MinX() < o.MaxX()-epsilon && r.MaxX() > o.MinX()+epsilon &&
		r.MinY() < o.MaxY()-epsilon && r.MaxY() > o.MinY()+epsilon
}

// Corners 返回矩形四个角点。顺序：左下、右下、右上、左上。
func (r Rect) Corners() [4]Vec2 {
	return [4]Vec2{
		{r.MinX(), r.MinY()},
		{r.MaxX(), r.MinY()},
		{r.MaxX(), r.MaxY()},
		{r.MinX(), r.MaxY()},
	}
}

// ---------------------------------------------------------------------------
// Obstacle 地图障碍物
// ---------------------------------------------------------------------------

type Obstacle struct {
	Rect   Rect // 障碍物矩形
	ID     int  // 唯一标识
	Moving bool // 是否正在移动中
}

// ---------------------------------------------------------------------------
// MapBounds 地图边界
// ---------------------------------------------------------------------------

type MapBounds struct {
	MinX, MinY, MaxX, MaxY float64
}

// Contains 检查点是否在地图范围内。
func (m MapBounds) Contains(p Vec2) bool {
	return p.X >= m.MinX && p.X <= m.MaxX && p.Y >= m.MinY && p.Y <= m.MaxY
}

// Shrink 将边界向内收缩指定量，用于确保移动者不会超出地图。
func (m MapBounds) Shrink(hw, hh float64) MapBounds {
	return MapBounds{
		MinX: m.MinX + hw,
		MinY: m.MinY + hh,
		MaxX: m.MaxX - hw,
		MaxY: m.MaxY - hh,
	}
}
