# ROS2 Simple Follower 機器人跟隨系統

本專案提供完整的機器人跟隨解決方案，整合多種感測器和控制演算法，實現智慧物體追蹤和跟隨功能。所有程式均包含詳細的中文技術註解，適合學習和研發使用。

## 📁 程式模組概覽

### 1. **adjust_hsv.py** - HSV 顏色調整工具
> **功能**：互動式顏色範圍調整工具，用於標定目標物體的 HSV 參數

- **技術背景**：HSV 色彩空間、OpenCV 滑動條、形態學運算
- **詳細註解**：顏色轉換、閾值調整、即時預覽功能  
- **應用指導**：顏色範圍標定、環境適應性調整

### 2. **line_follow.py** - 線條跟隨機器人
> **功能**：基於顏色識別的線條跟隨控制，適用於 AGV 導航

- **技術背景**：線條跟隨原理、PID 控制理論、影像處理技術
- **詳細註解**：顏色篩選、重心計算、PD 控制演算法
- **實用資訊**：多顏色支援、ROI 設定、控制參數調整

### 3. **laserTracker.py** - LiDAR 物體追蹤器  
> **功能**：使用雷射掃描進行物體偵測和位置追蹤

- **技術背景**：LiDAR 原理、空間差異分析、聚類演算法
- **詳細註解**：掃描比較、目標偵測、位置計算
- **技術深度**：感測器融合、雜訊處理、追蹤穩定性

### 4. **visualTracker.py** - RGB-D 視覺追蹤器
> **功能**：結合彩色和深度資訊進行 3D 物體追蹤

- **技術背景**：RGB-D 相機、3D 座標轉換、訊息同步
- **詳細註解**：影像同步、輪廓分析、深度處理、3D 定位
- **高級概念**：相機標定、時間同步、異常處理

### 5. **laserfollower.py** - 雷射跟隨控制器
> **功能**：整合 LiDAR 感測器和手把控制的智慧跟隨系統

- **技術背景**：LiDAR 技術、物體跟隨策略、手把控制整合
- **詳細註解**：連線監控、PID 參數、安全機制
- **系統整合**：多執行緒控制、失效安全設計

### 6. **visualFollower.py** - 視覺跟隨控制器
> **功能**：基於視覺回饋的機器人跟隨控制核心

- **技術背景**：視覺伺服控制、多感測器融合、機器人運動學
- **詳細註解**：PID 控制實現、速度限制、模式切換
- **完整 PID 類別**：離散化實現、多軸控制、自適應增益

---

## 🎯 技術亮點

### 🔧 **深度技術解釋**
- **相機技術**：RGB-D 原理、深度感測、相機標定
- **控制理論**：PID 控制、死區處理、積分飽和  
- **電腦視覺**：HSV 色彩空間、形態學運算、輪廓偵測
- **感測器融合**：訊息同步、時間戳管理、資料驗證

### 🚀 **實用指導**
- **參數調整**：PID 增益、顏色範圍、安全距離
- **故障排除**：常見問題診斷、效能優化建議
- **系統整合**：硬體需求、軟體相依性、網路設定
- **使用流程**：從安裝到執行的完整指南

### 🛡️ **安全機制**
- **連線監控**：手把斷線自動停止
- **速度限制**：防止過激運動
- **異常處理**：感測器失效保護
- **模式管理**：安全的自動/手動切換

---

## 🛠️ 系統需求

### 硬體需求
- **機器人平台**：差動驅動底盤 (支援 `cmd_vel` 控制)
- **感測器**：
  - RGB-D 相機 (Intel RealSense D435 或相容設備)
  - 2D LiDAR (RPLIDAR A1/A2 或相容設備)
- **控制器**：無線手把 (PS3/PS4/Xbox 或相容設備)
- **運算平台**：支援 ROS2 的 Linux 系統 (推薦 Ubuntu 22.04)

### 軟體需求
```bash
# ROS2 環境
ROS2 Humble Hawksbill

# Python 相依套件
opencv-python >= 4.5.0
numpy >= 1.19.0
matplotlib >= 3.3.0

# ROS2 套件
sensor_msgs
geometry_msgs 
std_msgs
cv_bridge
```

---

## 🚀 快速開始

### 1. 環境設定
```bash
# 克隆專案
git clone <repository-url>
cd simple_follower_ros2

# 建置 ROS2 套件
colcon build
source install/setup.bash
```

### 2. 基本使用流程

#### 📹 **視覺跟隨模式**
```bash
# 終端機 1：啟動相機節點
ros2 launch realsense2_camera rs_launch.py

# 終端機 2：啟動視覺追蹤器
ros2 run simple_follower_ros2 visualTracker

# 終端機 3：啟動跟隨控制器
ros2 run simple_follower_ros2 visualFollower
```

#### 🔴 **線條跟隨模式**
```bash
# 終端機 1：顏色參數調整 (一次性設定)
ros2 run simple_follower_ros2 adjust_hsv

# 終端機 2：線條跟隨
ros2 run simple_follower_ros2 line_follow
```

#### 📡 **LiDAR 跟隨模式**
```bash
# 終端機 1：LiDAR 追蹤器
ros2 run simple_follower_ros2 laserTracker

# 終端機 2：跟隨控制器
ros2 run simple_follower_ros2 laserfollower
```

---

## ⚙️ 參數配置

### PID 控制參數調整
編輯 `param/PID_visual_param.yaml` 檔案：
```yaml
# 角度控制 PID 參數
angular_control:
  P: 1.2    # 比例增益 (響應強度)
  I: 0.0    # 積分增益 (穩態誤差消除)
  D: 0.005  # 微分增益 (震盪抑制)

# 距離控制 PID 參數  
distance_control:
  P: 0.2
  I: 0.0
  D: 0.0

# 安全參數
max_speed: 0.3      # 最大速度 (m/s)
target_distance: 600 # 目標跟隨距離 (mm)
```

### HSV 顏色範圍設定
使用 `adjust_hsv.py` 工具進行即時調整，或直接修改程式中的顏色範圍：
```python
# 紅色範圍示例
col_red = (0, 100, 80, 10, 255, 255)  # (H_min, S_min, V_min, H_max, S_max, V_max)
```

---

## 🔧 故障排除

### 常見問題

#### ❌ **相機無法啟動**
```bash
# 檢查相機連接
lsusb | grep Intel

# 檢查權限
sudo chmod 666 /dev/video*

# 重新安裝驅動
sudo apt install ros-humble-realsense2-camera
```

#### ❌ **無法偵測到目標**
1. 使用 `adjust_hsv.py` 重新標定顏色範圍
2. 檢查光照條件是否適合
3. 確認目標物體大小是否在合理範圍

#### ❌ **機器人運動異常**
1. 檢查 `cmd_vel` 主題是否正確發布
2. 調整 PID 參數降低增益
3. 確認機器人物理限制設定

#### ❌ **手把控制器無響應**
```bash
# 檢查手把連線
ls /dev/input/js*

# 測試手把輸入
ros2 topic echo /joy
```

### 效能優化建議

1. **降低影像解析度**：減少處理負載
2. **調整處理頻率**：平衡即時性與穩定性
3. **優化演算法參數**：減少不必要的計算
4. **使用硬體加速**：GPU 加速影像處理

---

## 📊 系統架構圖

```
┌─────────────────┐    ┌──────────────────┐    ┌───────────────────┐
│   感測器層      │    │     處理層       │    │     控制層        │
├─────────────────┤    ├──────────────────┤    ├───────────────────┤
│ RGB-D Camera    │───▶│ visualTracker.py │───▶│ visualFollower.py │
│ 2D LiDAR        │───▶│ laserTracker.py  │───▶│ laserfollower.py  │
│ Joystick        │───▶│ Joy Messages     │───▶│ Mode Control      │
└─────────────────┘    └──────────────────┘    └───────────────────┘
                                │                        │
                                ▼                        ▼
                       ┌──────────────────┐    ┌───────────────────┐
                       │ Position Messages│    │  cmd_vel Messages │
                       │ (角度/距離資訊)   │    │  (速度控制指令)   │
                       └──────────────────┘    └───────────────────┘
                                                         │
                                                         ▼
                                               ┌───────────────────┐
                                               │   機器人底盤      │
                                               │ (差動驅動控制)    │
                                               └───────────────────┘
```

---

## 📚 技術文件

### 重要概念說明

- **[HSV 色彩空間](docs/hsv-color-space.md)**：顏色識別的理論基礎
- **[PID 控制理論](docs/pid-control.md)**：機器人運動控制演算法
- **[感測器融合](docs/sensor-fusion.md)**：多感測器資料整合技術
- **[ROS2 訊息系統](docs/ros2-messages.md)**：分散式通訊架構

### API 參考

詳細的函數和類別說明請參考各程式檔案中的 docstring 註解。

---

## 🤝 貢獻指南

歡迎提交 Issue 和 Pull Request！請確保：

1. 程式碼符合 PEP 8 規範
2. 添加適當的中文註解
3. 包含測試案例
4. 更新相關文件

---

## 📄 授權條款

本專案採用 Apache 2.0 授權條款，詳見 [LICENSE](LICENSE) 檔案。

---

## 📞 技術支援

如有技術問題，請提交 Issue 或聯繫開發團隊。

**開發環境**：ROS2 Humble + Ubuntu 22.04  
**測試平台**：TurtleBot3, Wheeltec Robot  
**更新日期**：2025年7月