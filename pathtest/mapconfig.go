package main

import (
	"encoding/json"
	"math"
	"math/rand"
	"os"
)

// ---------------------------------------------------------------------------
// 地图配置文件格式
// ---------------------------------------------------------------------------

type MapConfig struct {
	MapW      float64          `json:"mapW"`
	MapH      float64          `json:"mapH"`
	Obstacles []ObstacleConfig `json:"obstacles"`
}

type ObstacleConfig struct {
	CX     float64 `json:"cx"`
	CY     float64 `json:"cy"`
	W      float64 `json:"w"`
	H      float64 `json:"h"`
	ID     int     `json:"id"`
	Moving bool    `json:"moving"`
}

const defaultConfigPath = "map_config.json"

// loadMapConfig 从 JSON 文件加载地图配置
func loadMapConfig(path string) (*MapConfig, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	var cfg MapConfig
	if err := json.Unmarshal(data, &cfg); err != nil {
		return nil, err
	}
	return &cfg, nil
}

// saveMapConfig 将地图配置保存为 JSON 文件
func saveMapConfig(path string, cfg *MapConfig) error {
	data, err := json.MarshalIndent(cfg, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(path, data, 0644)
}

// generateDefaultConfig 生成默认地图配置（150静止 + 20移动）
func generateDefaultConfig() *MapConfig {
	const mapW, mapH = 50000.0, 50000.0
	cfg := &MapConfig{MapW: mapW, MapH: mapH}
	nextID := 1

	// 150 个静止障碍物（优化：减少数量，增大尺寸，保持覆盖率）
	for i := 0; i < 150; i++ {
		w := 250 + rand.Float64()*1800
		h := 250 + rand.Float64()*1800
		cx := w/2 + 200 + rand.Float64()*(mapW-w-400)
		cy := h/2 + 200 + rand.Float64()*(mapH-h-400)
		// 四舍五入到整数，便于阅读
		cfg.Obstacles = append(cfg.Obstacles, ObstacleConfig{
			CX: math.Round(cx), CY: math.Round(cy),
			W: math.Round(w), H: math.Round(h),
			ID: nextID, Moving: false,
		})
		nextID++
	}

	// 20 个移动障碍物（正方形, 半径为2的幂次）
	movingSizes := []float64{16, 32, 64, 128}
	for i := 0; i < 20; i++ {
		half := movingSizes[rand.Intn(len(movingSizes))]
		side := half * 2
		cx := side/2 + 200 + rand.Float64()*(mapW-side-400)
		cy := side/2 + 200 + rand.Float64()*(mapH-side-400)
		cfg.Obstacles = append(cfg.Obstacles, ObstacleConfig{
			CX: math.Round(cx), CY: math.Round(cy),
			W: side, H: side,
			ID: nextID, Moving: true,
		})
		nextID++
	}

	return cfg
}
