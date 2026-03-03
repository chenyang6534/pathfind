package world

import (
	"pathtest/pathfinding"
)

// ---------------------------------------------------------------------------
// MovingObsInfo 移动障碍物的运行时状态
// ---------------------------------------------------------------------------

type MovingObsInfo struct {
	TargetX        float64            // 最终目标点
	TargetY        float64
	Speed          float64
	Path           []pathfinding.Vec2 // 寻路计算的路径
	PathIdx        int                // 当前路径段索引
	LastPathTimeMs float64            // 上一次寻路耗时（毫秒）
	Arrived        bool               // 是否已到达目标（需要新随机目标）
	Repathing      bool               // 正在后台寻路中
	Waiting        bool               // 碰撞等待（被视为静止障碍物）
	WaitingAtDest  bool               // 到达目的地后等待
	WaitUntilMs    int64              // 等待结束的系统时间（毫秒）
	NeedRepath     bool               // 需要重新寻路但保持当前目标
}

// ---------------------------------------------------------------------------
// MovingObsFullInfo 返回给调用方的移动障碍物完整信息
// ---------------------------------------------------------------------------

type MovingObsFullInfo struct {
	ID               int              `json:"id"`
	CX               float64          `json:"cx"`
	CY               float64          `json:"cy"`
	Path             []Vec2JSON       `json:"path,omitempty"`
	TargetX          float64          `json:"targetX"`
	TargetY          float64          `json:"targetY"`
	WaitingAtDest    bool             `json:"waitingAtDest"`
	WaitRemainingMs  float64          `json:"waitRemainingMs"`
}

// Vec2JSON 二维坐标 JSON 格式
type Vec2JSON struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

// ---------------------------------------------------------------------------
// PathLogEntry 寻路日志条目
// ---------------------------------------------------------------------------

type PathLogEntry struct {
	ID      int     `json:"id"`
	TimeMs  float64 `json:"timeMs"`
	Nodes   int     `json:"nodes"`
	Success bool    `json:"success"`
}

// ---------------------------------------------------------------------------
// PathStats 寻路性能统计信息
// ---------------------------------------------------------------------------

type PathStats struct {
	AvgTimeMs      float64 `json:"avgTimeMs"`      // 平均寻路耗时
	TotalCount     int     `json:"totalCount"`     // 总寻路次数
	MaxTimeMs      float64 `json:"maxTimeMs"`      // 5秒内最大耗时
	MaxID          int     `json:"maxID"`          // 5秒内最大耗时的障碍物ID
	MaxSuccess     bool    `json:"maxSuccess"`     // 5秒内最大耗时是否成功
	HasData        bool    `json:"hasData"`        // 是否有统计数据
}

// ---------------------------------------------------------------------------
// SlowPathRecord 慢寻路记录（用于性能分析）
// ---------------------------------------------------------------------------

type SlowPathRecord struct {
	Timestamp      int64   `json:"timestamp"`      // 记录时间（系统nowMs）
	ID             int     `json:"id"`            // 障碍物ID
	TimeMs         float64 `json:"timeMs"`        // 寻路耗时
	Success        bool    `json:"success"`       // 是否成功
	StartX, StartY float64 `json:"startX,startY"` // 起点
	EndX, EndY     float64 `json:"endX,endY"`     // 终点
	Distance       float64 `json:"distance"`      // 直线距离
	ObstacleCount  int     `json:"obstacleCount"` // 附近障碍物数量
	NodeCount      int     `json:"nodeCount"`     // 可视图节点数
	PathLength     int     `json:"pathLength"`    // 路径长度
	
	// 分析结果
	Analysis string `json:"analysis"` // 慢的原因分析
}
