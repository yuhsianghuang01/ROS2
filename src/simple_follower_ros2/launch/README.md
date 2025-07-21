# Simple Follower ROS2 套件說明文件

## 概述

Simple Follower ROS2 是一個完整的機器人跟隨系統，支援多種感測器輸入和跟隨模式。此套件專為 Wheeltec 機器人平台設計，提供了雷射、視覺和線跟隨三種主要功能模式。

## 系統架構

### 整體系統架構圖
```
[操作者/目標物體] 
        ↓
[感測器層 - LiDAR/Camera]
        ↓ 
[追蹤器層 - Tracker Nodes]
        ↓
[控制器層 - Follower Nodes] 
        ↓
[執行器層 - Robot Base]
        ↓
[機器人動作]
```

### 核心組件關係
```
手把控制器 -----> 跟隨控制節點 -----> 機器人底盤
     |                |                   ↑
     |                ↓                   |
     |         PID 控制器              cmd_vel
     |                ↑                   |
     |                |                   |
感測器驅動 -----> 追蹤器節點 --------> 位置資訊
```

## 技術背景知識

### 1. ROS2 分散式系統概念

**節點 (Node)**: ROS2 中的基本執行單元，每個功能模組都是一個節點
- `laserTracker`: 雷射掃描資料處理節點
- `laserFollower`: 雷射跟隨控制節點  
- `visualTracker`: 視覺追蹤處理節點
- `visualFollower`: 視覺跟隨控制節點

**話題 (Topic)**: 節點間非同步通訊機制
- `/scan`: LiDAR 掃描資料
- `/image_raw`: 相機影像資料
- `/cmd_vel`: 機器人速度控制指令
- `/object_tracker/current_position`: 目標物體位置

**服務 (Service)**: 節點間同步通訊機制，用於參數設定和狀態查詢

**啟動系統 (Launch System)**: 管理多個節點的啟動、參數傳遞和依賴關係

### 2. 感測器融合技術

**LiDAR (光偵測和測距)**
- 工作原理：發射雷射脈衝，測量反射時間計算距離
- 優勢：高精度、不受光照影響、360度掃描
- 劣勢：無法識別顏色和紋理，成本較高
- 資料格式：LaserScan 訊息，包含距離陣列和角度資訊

**RGB-D 相機**
- 工作原理：結合彩色影像和深度資訊
- 優勢：豐富視覺資訊、可做物體識別
- 劣勢：受光照影響、視野有限
- 資料格式：Image + PointCloud2 訊息

**感測器標定**
- 內部標定：相機內參數校正 (焦距、畸變係數)
- 外部標定：感測器相對機器人的位置關係
- 時間同步：確保不同感測器資料時戳一致

### 3. 物體偵測與追蹤演算法

**基於 LiDAR 的追蹤**
```
掃描資料 → 前處理 → 聚類分析 → 物體偵測 → 卡爾曼濾波 → 位置輸出
```
- **前處理**: 雜訊過濾、距離限制、角度裁切
- **聚類分析**: DBSCAN 或 k-means 將相近點群組
- **物體偵測**: 基於形狀、大小過濾候選目標
- **卡爾曼濾波**: 預測物體運動軌跡，平滑追蹤結果

**基於視覺的追蹤**
```
RGB影像 → HSV轉換 → 顏色濾波 → 輪廓偵測 → 質心計算 → 深度匹配 → 3D位置
```
- **顏色空間**: HSV 比 RGB 更適合顏色偵測
- **形態學運算**: 開閉運算去除雜訊，連接破碎區域
- **輪廓分析**: 面積、周長、形狀特徵篩選
- **深度匹配**: 2D 像素座標轉換為 3D 世界座標

### 4. PID 控制理論

**PID 控制器組成**
- **比例項 (P)**: 當前誤差的直接響應，決定響應速度
- **積分項 (I)**: 累積誤差的修正，消除穩態誤差
- **微分項 (D)**: 誤差變化率的預測，減少超調和振盪

**離散 PID 實現**
```python
# PID 控制方程式
output = Kp * error + Ki * sum(errors) * dt + Kd * (error - last_error) / dt
```

**參數調整策略**
1. **Ziegler-Nichols 方法**: 經典的 PID 調整方法
2. **試驗法**: 逐步調整觀察響應
3. **自適應調整**: 根據系統響應自動調整參數

### 5. 機器人運動學

**差動驅動機器人**
```
左輪速度: v_left = v_linear - (w_angular * wheelbase) / 2
右輪速度: v_right = v_linear + (w_angular * wheelbase) / 2
```

**座標系轉換**
- **機器人座標系**: 以機器人為中心，x 軸向前，y 軸向左
- **世界座標系**: 固定參考座標系
- **感測器座標系**: 各感測器的本地座標系

## 功能模組詳解

### 1. 雷射跟隨系統 (Laser Following)

**核心檔案**
- `laserfollower.py`: 主控制邏輯
- `laserTracker.py`: 物體偵測算法
- `laser_follower.launch.py`: 系統啟動檔

**工作流程**
1. **初始化階段**
   - 啟動機器人底盤驅動
   - 初始化 LiDAR 感測器
   - 建立 ROS2 通訊節點
   - 載入 PID 控制參數

2. **感知階段**
   - 接收 LiDAR 掃描資料 (`/scan`)
   - 執行物體偵測算法
   - 計算目標物體位置
   - 發布位置資訊 (`/object_tracker/current_position`)

3. **控制階段**
   - 接收目標位置資訊
   - 計算位置和角度誤差
   - PID 控制器計算速度指令
   - 發布運動指令 (`/cmd_vel`)

4. **安全機制**
   - 手把連線監控
   - 速度限制檢查
   - 障礙物距離安全
   - 緊急停止功能

**參數配置**
```yaml
# PID 控制參數
P: [1.5, 0.5]     # [角度P, 距離P]
I: [0.0, 0.0]     # [角度I, 距離I]  
D: [0.02, 0.002]  # [角度D, 距離D]

# 安全參數
maxSpeed: 0.4     # 最大速度 (m/s)
targetDist: 0.8   # 目標距離 (m)
minDist: 0.15     # 最小安全距離 (m)
```

### 2. 視覺跟隨系統 (Visual Following)

**核心檔案**
- `visualFollower.py`: 視覺跟隨控制器
- `visualTracker.py`: 視覺物體追蹤器
- `visual_follower.launch.py`: 系統啟動檔

**顏色偵測流程**
1. **影像預處理**
   - RGB 轉 HSV 顏色空間
   - 高斯模糊降低雜訊
   - 直方圖均衡化增強對比

2. **顏色濾波**
   - HSV 範圍濾波器
   - 形態學開運算去除小雜點
   - 形態學閉運算填補空洞

3. **輪廓分析**
   - 找尋所有輪廓
   - 面積過濾排除小物體
   - 選擇最大面積輪廓作為目標

4. **位置計算**
   - 計算輪廓質心 (moments)
   - 像素座標轉世界座標
   - 結合深度資訊獲得 3D 位置

**HSV 參數調整工具**
```bash
ros2 launch simple_follower_ros2 adjust_hsv.launch.py
```

### 3. 線跟隨系統 (Line Following)

**核心檔案**
- `line_follow.py`: 線跟隨控制器
- `line_follower.launch.py`: 系統啟動檔

**線偵測算法**
1. **邊緣偵測**
   - Canny 邊緣偵測
   - 霍夫線變換 (Hough Transform)
   - 線段合併和過濾

2. **線中心計算**
   - 加權平均計算線中心
   - 線方向向量估計
   - 預測線的延伸方向

3. **控制策略**
   - PD 控制器 (比例-微分)
   - 前視點跟隨算法
   - 速度自適應調整

## Launch 檔案使用說明

### 1. laser_follower.launch.py

**功能**: 啟動完整雷射跟隨系統

**使用方式**:
```bash
ros2 launch simple_follower_ros2 laser_follower.launch.py
```

**系統組件**:
- 機器人底盤驅動
- LiDAR 感測器驅動
- 雷射追蹤器節點
- 雷射跟隨控制器
- 手把控制器支援

**參數**:
- `robot_type`: 機器人型號
- `lidar_type`: LiDAR 感測器型號
- `use_joy`: 是否使用手把控制

### 2. visual_follower.launch.py

**功能**: 啟動視覺跟隨系統

**使用方式**:
```bash
ros2 launch simple_follower_ros2 visual_follower.launch.py
```

**系統組件**:
- 機器人底盤驅動
- RGB-D 相機驅動
- 視覺追蹤器節點
- 視覺跟隨控制器
- 手把控制器支援

### 3. line_follower.launch.py

**功能**: 啟動線跟隨系統

**使用方式**:
```bash
ros2 launch simple_follower_ros2 line_follower.launch.py
```

### 4. adjust_hsv.launch.py

**功能**: HSV 顏色參數調整工具

**使用方式**:
```bash
ros2 launch simple_follower_ros2 adjust_hsv.launch.py
```

## 程式碼修改指南

### 1. 修改 PID 參數

**位置**: `src/simple_follower_ros2/param/` 目錄下的 `.yaml` 檔案

**步驟**:
1. 編輯對應的參數檔案
2. 調整 P, I, D 數值
3. 重新啟動系統測試效果

**調整建議**:
- P 值太大：反應過度，震盪
- P 值太小：反應緩慢
- I 值：消除穩態誤差，但可能導致積分飽和
- D 值：預測性控制，減少超調

### 2. 修改安全參數

**位置**: `laserfollower.py` 或 `visualFollower.py`

**重要參數**:
```python
self.max_speed = 0.4        # 最大速度
self.min_distance = 0.15    # 最小安全距離
self.target_distance = 0.8  # 目標跟隨距離
```

### 3. 修改感測器參數

**LiDAR 參數**:
- 檔案位置: `turn_on_wheeltec_robot/launch/wheeltec_lidar.launch.py`
- 修改項目: 掃描頻率、角度範圍、距離範圍

**相機參數**:
- 檔案位置: `turn_on_wheeltec_robot/launch/wheeltec_camera.launch.py`
- 修改項目: 解析度、幀率、曝光設定

### 4. 新增跟隨模式

**步驟**:
1. 在 `simple_follower_ros2/` 目錄新增 Python 檔案
2. 繼承 `Node` 類別實現控制邏輯
3. 在 `setup.py` 新增 entry point
4. 建立對應的 launch 檔案
5. 更新 `package.xml` 相依性

## 故障排除

### 1. 系統啟動問題

**症狀**: Launch 檔案無法正常啟動
**可能原因**:
- ROS2 環境未正確設定
- 套件路徑問題
- 相依性套件未安裝

**解決方法**:
```bash
# 檢查 ROS2 環境
printenv | grep ROS

# 重新編譯套件
cd ~/ros2_ws
colcon build --packages-select simple_follower_ros2

# 重新載入環境
source install/setup.bash
```

### 2. 感測器連接問題

**LiDAR 無資料**:
```bash
# 檢查 LiDAR 連接
ros2 topic list | grep scan
ros2 topic echo /scan --once

# 檢查設備權限
sudo chmod 777 /dev/ttyUSB*
```

**相機無影像**:
```bash
# 檢查相機連接
ros2 topic list | grep image
ros2 run rqt_image_view rqt_image_view

# 檢查 USB 相機
lsusb | grep -i camera
```

### 3. 控制性能問題

**機器人運動不穩**:
- 檢查 PID 參數設定
- 確認控制頻率足夠高
- 檢查機械傳動是否順暢

**跟隨效果不佳**:
- 調整目標偵測參數
- 檢查感測器資料品質
- 優化環境照明條件

### 4. 通訊問題

**節點間無通訊**:
```bash
# 檢查節點狀態
ros2 node list

# 檢查話題連接
ros2 topic info /cmd_vel

# 視覺化節點圖
rqt_graph
```

## 效能優化建議

### 1. 計算效能優化

- 使用 NumPy 向量化運算
- 適當的執行緒分離 (感知/控制)
- 記憶體池重複使用
- 適當的 QoS 設定

### 2. 即時性優化

- 設定適當的優先級
- 使用即時 Linux 核心
- 優化訊息佇列長度
- 減少不必要的資料複製

### 3. 穩定性優化

- 增加異常處理機制
- 實現看門狗定時器
- 加入系統健康監控
- 設計優雅的關機流程

## 擴展開發建議

### 1. 多感測器融合

整合多種感測器提高系統魯棒性：
- LiDAR + 視覺融合
- IMU 慣性導航輔助
- 輪編碼器里程計

### 2. 機器學習整合

- 物體分類識別
- 軌跡預測模型
- 自適應參數調整
- 強化學習控制策略

### 3. 高階功能

- SLAM 同步定位與建圖
- 多機器人協同跟隨
- 動態障礙物迴避
- 語音指令控制

## 參考資料

1. [ROS2 官方文件](https://docs.ros.org/en/humble/)
2. [Wheeltec 機器人文件](http://www.wheeltec.net/)
3. [OpenCV 電腦視覺函式庫](https://opencv.org/)
4. [PID 控制理論](https://en.wikipedia.org/wiki/PID_controller)
5. [機器人學：建模、規劃與控制](https://www.springer.com/gp/book/9781846286414)

---

**維護資訊**
- 最後更新：2025年7月
- 套件版本：1.0.0
- 支援 ROS2 版本：Humble Hawksbill
- 支援平台：Ubuntu 22.04 LTS

如有問題或建議，請參考故障排除章節或聯繫套件維護者。
