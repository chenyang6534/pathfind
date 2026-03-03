package main

import (
	"embed"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"pathtest/world"
)

//go:embed static
var staticFiles embed.FS

// ---------------------------------------------------------------------------
// 全局世界实例
// ---------------------------------------------------------------------------

var gWorld = world.New(3)

// ---------------------------------------------------------------------------
// JSON 请求/响应类型
// ---------------------------------------------------------------------------

type obstacleJSON struct {
	CX     float64 `json:"cx"`
	CY     float64 `json:"cy"`
	W      float64 `json:"w"`
	H      float64 `json:"h"`
	ID     int     `json:"id"`
	Moving bool    `json:"moving"`
}

type vec2JSON = world.Vec2JSON

// =========================================================================
// API: /api/initworld
// =========================================================================

type initWorldReq struct {
	MapW      float64        `json:"mapW"`
	MapH      float64        `json:"mapH"`
	Obstacles []obstacleJSON `json:"obstacles"`
}

func handleInitWorld(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "POST only", http.StatusMethodNotAllowed)
		return
	}
	var req initWorldReq
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	obs := make([]world.InitObstacle, len(req.Obstacles))
	for i, o := range req.Obstacles {
		obs[i] = world.InitObstacle{
			CX: o.CX, CY: o.CY,
			W: o.W, H: o.H,
			ID: o.ID, Moving: o.Moving,
		}
	}

	gWorld.Init(req.MapW, req.MapH, obs)

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]any{"success": true})
}

// =========================================================================
// API: /api/addobstacle — 动态添加单个障碍物
// =========================================================================

type addObstacleReq struct {
	CX     float64 `json:"cx"`
	CY     float64 `json:"cy"`
	W      float64 `json:"w"`
	H      float64 `json:"h"`
	ID     int     `json:"id"`
	Moving bool    `json:"moving"`
}

func handleAddObstacle(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "POST only", http.StatusMethodNotAllowed)
		return
	}
	var req addObstacleReq
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	gWorld.AddObstacle(req.CX, req.CY, req.W, req.H, req.ID, req.Moving)

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]any{"success": true})
}

// =========================================================================
// API: /api/tickworld — 推进世界一帧
// =========================================================================

func handleTickWorld(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")

	active, movingObs, logs, stats := gWorld.TickAndCollect()

	if !active {
		json.NewEncoder(w).Encode(map[string]any{"active": false})
		return
	}

	resp := map[string]any{
		"active":    true,
		"movingObs": movingObs,
		"pathLogs":  logs,
	}
	if stats.HasData {
		resp["pathStats"] = stats
	}
	json.NewEncoder(w).Encode(resp)
}

// =========================================================================
// API: /api/slowpaths — 获取慢寻路记录
// =========================================================================

func handleSlowPaths(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	records := gWorld.GetSlowPathRecords()
	json.NewEncoder(w).Encode(map[string]any{
		"records": records,
		"count":   len(records),
	})
}

// =========================================================================
// API: /api/mapconfig — 获取地图配置
// =========================================================================

func handleMapConfig(w http.ResponseWriter, r *http.Request) {
	cfg, err := loadMapConfig(defaultConfigPath)
	if err != nil {
		http.Error(w, "配置文件读取失败: "+err.Error(), http.StatusInternalServerError)
		return
	}
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(cfg)
}

// =========================================================================
// API: /api/savemap — 保存当前地图配置
// =========================================================================

type saveMapReq struct {
	MapW      float64        `json:"mapW"`
	MapH      float64        `json:"mapH"`
	Obstacles []obstacleJSON `json:"obstacles"`
}

func handleSaveMap(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "POST only", http.StatusMethodNotAllowed)
		return
	}
	var req saveMapReq
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	// 转换为 MapConfig 格式
	cfg := &MapConfig{
		MapW:      req.MapW,
		MapH:      req.MapH,
		Obstacles: make([]ObstacleConfig, len(req.Obstacles)),
	}
	for i, o := range req.Obstacles {
		cfg.Obstacles[i] = ObstacleConfig{
			CX:     o.CX,
			CY:     o.CY,
			W:      o.W,
			H:      o.H,
			ID:     o.ID,
			Moving: o.Moving,
		}
	}

	// 保存到文件
	if err := saveMapConfig(defaultConfigPath, cfg); err != nil {
		http.Error(w, "保存失败: "+err.Error(), http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]any{"success": true, "message": "地图已保存到 map_config.json"})
}

// ---------------------------------------------------------------------------
// 加载地图配置并初始化世界
// ---------------------------------------------------------------------------

func initWorldFromConfig() {
	cfg, err := loadMapConfig(defaultConfigPath)
	if err != nil {
		if os.IsNotExist(err) {
			log.Printf("配置文件 %s 不存在，生成默认配置", defaultConfigPath)
			cfg = generateDefaultConfig()
			if saveErr := saveMapConfig(defaultConfigPath, cfg); saveErr != nil {
				log.Printf("保存配置文件失败: %v", saveErr)
			}
		} else {
			log.Fatalf("读取配置文件失败: %v", err)
		}
	}

	obs := make([]world.InitObstacle, len(cfg.Obstacles))
	for i, o := range cfg.Obstacles {
		obs[i] = world.InitObstacle{
			CX: o.CX, CY: o.CY,
			W: o.W, H: o.H,
			ID: o.ID, Moving: o.Moving,
		}
	}
	gWorld.Init(cfg.MapW, cfg.MapH, obs)
	log.Printf("世界已从配置文件初始化: %.0f×%.0f, %d 个障碍物", cfg.MapW, cfg.MapH, len(cfg.Obstacles))
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

func main() {
	// 启动时从配置文件加载地图
	initWorldFromConfig()

	http.Handle("/static/", http.FileServer(http.FS(staticFiles)))
	http.HandleFunc("/api/mapconfig", handleMapConfig)
	http.HandleFunc("/api/savemap", handleSaveMap)
	http.HandleFunc("/api/initworld", handleInitWorld)
	http.HandleFunc("/api/addobstacle", handleAddObstacle)
	http.HandleFunc("/api/tickworld", handleTickWorld)
	http.HandleFunc("/api/slowpaths", handleSlowPaths)

	addr := ":8080"
	fmt.Printf("服务启动: http://localhost%s/static/\n", addr)
	log.Fatal(http.ListenAndServe(addr, nil))
}
