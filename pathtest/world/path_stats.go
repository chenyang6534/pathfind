package world

import (
	"log"
	"math"
	"strings"
)

// PathStatsCollector 寻路性能统计收集器（独立模块，低耦合设计）
type PathStatsCollector struct {
	// 累计统计（自启动以来的总计）
	totalMs float64 // 累计寻路总耗时
	count   int     // 累计寻路次数
	
	// 5秒窗口统计
	window5sMs    float64 // 当前5秒窗口内的总耗时
	window5sCount int     // 当前5秒窗口内的寻路次数
	
	// 5秒内峰值统计
	maxMs      float64 // 当前5秒内最大寻路耗时
	maxID      int     // 当前5秒内最大耗时的障碍物ID
	maxSuccess bool    // 当前5秒内最大耗时是否成功
	lastPrint  int64   // 上次打印统计的时间（毫秒）
	
	// 慢寻路记录
	slowRecords    []SlowPathRecord // 最近的慢寻路记录
	maxSlowRecords int              // 最多保存记录数（默认20）
	slowContext    *SlowPathRecord  // 当前5秒内最慢寻路的详细信息
}

// NewPathStatsCollector 创建性能统计收集器
func NewPathStatsCollector(maxRecords int) *PathStatsCollector {
	return &PathStatsCollector{
		maxSlowRecords: maxRecords,
		slowRecords:    make([]SlowPathRecord, 0, maxRecords),
	}
}

// RecordPath 记录一次寻路操作
func (s *PathStatsCollector) RecordPath(
	obsID int,
	elapsed float64,
	success bool,
	startX, startY, endX, endY float64,
	nearbyObsCount int,
	nodeCount int,
	pathLength int,
) {
	// 累计统计（不重置）
	s.totalMs += elapsed
	s.count++
	
	// 5秒窗口统计
	s.window5sMs += elapsed
	s.window5sCount++
	
	// 更新5秒内峰值
	if elapsed > s.maxMs {
		s.maxMs = elapsed
		s.maxID = obsID
		s.maxSuccess = success
		
		// 计算直线距离
		distance := math.Sqrt(math.Pow(endX-startX, 2) + math.Pow(endY-startY, 2))
		
		// 保存详细上下文
		s.slowContext = &SlowPathRecord{
			ID:            obsID,
			TimeMs:        elapsed,
			Success:       success,
			StartX:        startX,
			StartY:        startY,
			EndX:          endX,
			EndY:          endY,
			Distance:      distance,
			ObstacleCount: nearbyObsCount,
			NodeCount:     nodeCount,
			PathLength:    pathLength,
		}
	}
}

// GetStats 获取当前统计数据（用于API返回）
func (s *PathStatsCollector) GetStats() PathStats {
	avgMs := 0.0
	if s.count > 0 {
		avgMs = s.totalMs / float64(s.count)
	}
	return PathStats{
		AvgTimeMs:  math.Round(avgMs*100) / 100,
		TotalCount: s.count,
		MaxTimeMs:  math.Round(s.maxMs*100) / 100,
		MaxID:      s.maxID,
		MaxSuccess: s.maxSuccess,
		HasData:    s.count > 0,
	}
}

// ShouldPrint 检查是否应该打印统计（每5秒一次）
func (s *PathStatsCollector) ShouldPrint(nowMs int64) bool {
	return nowMs-s.lastPrint >= 5000
}

// PrintStats 打印统计信息并重置5秒窗口计数器
func (s *PathStatsCollector) PrintStats(nowMs int64) {
	// 总是打印统计，即使没有寻路也显示状态
	if s.window5sCount == 0 {
		log.Printf("[寻路统计] 最近5秒无寻路操作 | 慢寻路记录数: %d", len(s.slowRecords))
	} else {
		statusStr := "成功"
		if !s.maxSuccess {
			statusStr = "失败"
		}
		avgMs := s.window5sMs / float64(s.window5sCount)
		log.Printf("[寻路统计] 5秒最慢: M%d %.2fms (%s) | 平均: %.2fms (%d次) | 慢寻路记录数: %d",
			s.maxID,
			math.Round(s.maxMs*100)/100,
			statusStr,
			math.Round(avgMs*100)/100,
			s.window5sCount,
			len(s.slowRecords),
		)
		
		// 保存并分析慢寻路记录
		if s.slowContext != nil {
			s.slowContext.Timestamp = nowMs
			s.slowContext.Analysis = s.analyzeSlowPath(s.slowContext)
			s.slowRecords = append(s.slowRecords, *s.slowContext)
			
			// 限制记录数量
			if len(s.slowRecords) > s.maxSlowRecords {
				s.slowRecords = s.slowRecords[1:]
			}
			
			log.Printf("[慢寻路分析] ID=%d 距离=%.0f 障碍=%d 节点=%d 路径长度=%d | %s",
				s.slowContext.ID,
				s.slowContext.Distance,
				s.slowContext.ObstacleCount,
				s.slowContext.NodeCount,
				s.slowContext.PathLength,
				s.slowContext.Analysis,
			)
		}
	}
	
	// 重置5秒窗口统计（累计统计不重置）
	s.window5sMs = 0
	s.window5sCount = 0
	s.maxMs = 0
	s.lastPrint = nowMs
	s.slowContext = nil
}

// GetSlowRecords 获取慢寻路记录（线程安全的复制）
func (s *PathStatsCollector) GetSlowRecords() []SlowPathRecord {
	records := make([]SlowPathRecord, len(s.slowRecords))
	copy(records, s.slowRecords)
	return records
}

// analyzeSlowPath 分析慢寻路原因（私有方法）
func (s *PathStatsCollector) analyzeSlowPath(rec *SlowPathRecord) string {
	var reasons []string
	
	// 1. 距离因素
	if rec.Distance > 30000 {
		reasons = append(reasons, "超长距离(>30000)")
	} else if rec.Distance > 20000 {
		reasons = append(reasons, "长距离(>20000)")
	}
	
	// 2. 障碍物密度
	if rec.Distance > 0 {
		density := float64(rec.ObstacleCount) / (rec.Distance / 1000)
		if density > 15 {
			reasons = append(reasons, "障碍物极度密集")
		} else if density > 8 {
			reasons = append(reasons, "障碍物密集")
		}
	}
	if rec.ObstacleCount > 100 {
		reasons = append(reasons, "障碍物过多")
	}
	
	// 3. 可视图复杂度
	if rec.NodeCount > 2000 {
		reasons = append(reasons, "可视图节点过多")
	} else if rec.NodeCount > 1000 {
		reasons = append(reasons, "可视图节点较多")
	}
	
	// 4. 路径复杂度
	if !rec.Success {
		reasons = append(reasons, "寻路失败-无可行路径")
	} else if rec.PathLength > 50 {
		reasons = append(reasons, "路径极其曲折")
	} else if rec.PathLength > 30 {
		reasons = append(reasons, "路径曲折")
	}
	
	// 5. 性能阈值
	if rec.TimeMs > 20 {
		reasons = append(reasons, "严重性能问题")
	} else if rec.TimeMs > 10 {
		reasons = append(reasons, "明显性能瓶颈")
	}
	
	if len(reasons) == 0 {
		return "未明确识别原因"
	}
	
	return "原因: " + strings.Join(reasons, ", ")
}
