# WheelTec 機器人鍵盤控制程式技術說明

## 概述
`wheeltec_keyboard.py` 是一個基於 ROS2 的機器人鍵盤遙控程式，提供直觀的鍵盤控制介面來操控 WheelTec 系列機器人。此程式具備平滑速度控制、雙模式運動支援和跨平台相容性。

## 核心技術架構

### 1. ROS2 通訊架構

#### Topic 通訊模式
```
[鍵盤輸入] → [wheeltec_keyboard節點] → [cmd_vel topic] → [機器人控制器]
```

- **Topic**: `cmd_vel`
- **訊息類型**: `geometry_msgs/Twist`
- **通訊模式**: 發布者-訂閱者 (Publisher-Subscriber)
- **QoS 設定**: 佇列深度 10，確保訊息不遺失

#### Twist 訊息結構
```python
geometry_msgs/Twist:
  linear:          # 線性速度 (m/s)
    x: 前後運動     # 正值=前進，負值=後退
    y: 左右運動     # 正值=右移，負值=左移
    z: 上下運動     # 通常為 0 (地面機器人)
  angular:         # 角速度 (rad/s)
    x: 滾轉 (roll)  # 通常為 0
    y: 俯仰 (pitch) # 通常為 0  
    z: 偏航 (yaw)   # 正值=左轉，負值=右轉
```

### 2. 平滑速度控制演算法

#### 增量式速度控制
程式採用增量式控制演算法，避免速度突變：

```python
# 前後運動平滑控制
if target_speed > control_speed:
    control_speed = min(target_speed, control_speed + 0.1)  # 限制加速度
elif target_speed < control_speed:
    control_speed = max(target_speed, control_speed - 0.1)  # 限制減速度
```

#### 控制參數說明
- **線性加速度限制**: 0.1 m/s² (前後、橫向)
- **角加速度限制**: 0.5 rad/s² (轉向)
- **控制週期**: 約 100ms (取決於按鍵輸入頻率)

### 3. 運動模式設計

#### 一般模式 (CommonMode)
適用於差動驅動、阿克曼轉向等傳統機器人：
- **前後運動**: linear.x
- **轉向運動**: angular.z
- **橫向運動**: 不支援 (linear.y = 0)

#### 全向模式 (OmniMode)
適用於全向輪、麥輪等全向移動機器人：
- **前後運動**: linear.x
- **橫向運動**: linear.y  
- **轉向運動**: 停用 (angular.z = 0)

### 4. 跨平台鍵盤輸入處理

#### Windows 平台
```python
import msvcrt
key = msvcrt.getch().decode('utf-8')  # 直接獲取按鍵
```

#### Linux/Unix 平台
```python
import termios, tty, select
tty.setraw(sys.stdin.fileno())        # 設定原始模式
key = sys.stdin.read(1)               # 讀取單一字元
```

## 按鍵映射系統

### 運動控制按鍵
```
操作布局：
   u    i    o     ←→ 對應鍵盤 QWERTY 配置
   j    k    l     ←→ 類似 Vi 編輯器方向鍵
   m    ,    .
   
運動方向：
   ↖    ↑    ↗     (u: 左前, i: 前進, o: 右前)
   ←   停止   →     (j: 左轉, k: 停止, l: 右轉)  
   ↙    ↓    ↘     (m: 左後, ,: 後退, .: 右後)
```

### 速度調整按鍵
| 按鍵 | 功能 | 調整倍率 | 說明 |
|------|------|----------|------|
| q | 整體加速 | ×1.1 | 線性和角速度同時增加 10% |
| z | 整體減速 | ×0.9 | 線性和角速度同時減少 10% |
| w | 線性加速 | ×1.1 | 僅增加線性速度 10% |
| x | 線性減速 | ×0.9 | 僅減少線性速度 10% |
| e | 角速度加速 | ×1.1 | 僅增加角速度 10% |
| c | 角速度減速 | ×0.9 | 僅減少角速度 10% |

### 特殊功能按鍵
- **空白鍵/k**: 緊急停止 (立即歸零)
- **b**: 切換運動模式
- **Ctrl+C**: 安全退出程式

## 安全機制設計

### 1. 無效輸入處理
```python
count = count + 1
if count > 4:
    x = 0    # 連續 4 次無效輸入後自動停止
    th = 0.0
```

### 2. 程式退出安全
```python
finally:
    twist = Twist()  # 建立零速度訊息
    # 所有速度歸零...
    pub.publish(twist)  # 確保機器人停止
```

### 3. 終端機狀態恢復
```python
if os.name != 'nt':
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
```

## 效能與限制分析

### 效能特性
- **回應延遲**: < 100ms
- **CPU 使用率**: 極低 (事件驅動)
- **記憶體消耗**: < 10MB
- **網路頻寬**: < 1KB/s

### 系統限制
- **輸入頻率**: 受限於人工按鍵速度
- **精確度**: 離散化控制，無連續調節
- **同時按鍵**: 不支援多鍵組合

## 擴展應用範例

### 1. 機器人控制節點整合
```python
# launch 檔案整合範例
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheeltec_robot_keyboard',
            executable='wheeltec_keyboard',
            name='keyboard_controller',
            output='screen'
        ),
        Node(
            package='wheeltec_robot',
            executable='wheeltec_robot',
            name='robot_base',
            parameters=[{'use_sim_time': False}]
        )
    ])
```

### 2. 自定義速度限制
```python
# 在程式中添加速度限制檢查
def limit_velocity(linear, angular):
    max_linear = 0.5   # 最大線性速度
    max_angular = 1.0  # 最大角速度
    
    linear = max(-max_linear, min(max_linear, linear))
    angular = max(-max_angular, min(max_angular, angular))
    
    return linear, angular
```

### 3. 日誌記錄功能
```python
import logging

# 設定日誌
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 在控制迴圈中記錄
logger.info(f"Speed: {control_speed:.2f}, Turn: {control_turn:.2f}")
```

## 故障排除指南

### 常見問題與解決方案

#### 1. 程式無回應
**症狀**: 按鍵無反應，機器人不動
**原因**: ROS2 環境未正確設定
**解決方案**:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list  # 檢查 topic 是否可見
```

#### 2. 機器人運動異常
**症狀**: 機器人運動方向與預期不符
**原因**: 座標系統或馬達配置問題
**解決方案**:
```bash
ros2 topic echo /cmd_vel  # 監控發送的命令
# 檢查機器人座標系統定義
```

#### 3. 鍵盤輸入異常 (Linux)
**症狀**: 需要按 Enter 才有反應
**原因**: 終端機設定問題
**解決方案**:
```bash
# 確保在支援原始輸入的終端機中執行
# 避免在 IDE 的整合終端機中執行
```

#### 4. 權限錯誤
**症狀**: 程式啟動失敗，提示權限不足
**解決方案**:
```bash
sudo chmod +x wheeltec_keyboard.py
# 或檢查檔案權限設定
```

### 除錯工具

#### ROS2 除錯命令
```bash
# 檢查節點狀態
ros2 node list
ros2 node info /wheeltec_keyboard

# 檢查 topic 資訊
ros2 topic info /cmd_vel
ros2 topic hz /cmd_vel     # 檢查發布頻率
ros2 topic echo /cmd_vel   # 即時監控訊息

# 檢查服務品質
ros2 topic info /cmd_vel -v
```

#### 系統資源監控
```bash
# CPU 使用率
top -p $(pgrep -f wheeltec_keyboard)

# 記憶體使用情況  
ps aux | grep wheeltec_keyboard

# 網路流量 (ROS2 DDS)
iftop -i lo  # 本地迴環介面
```

## 最佳實務建議

### 1. 程式碼最佳化
- **模組化設計**: 將不同功能分離成獨立函數
- **參數外部化**: 將速度限制等參數移至設定檔
- **錯誤處理**: 增加更詳細的異常處理機制

### 2. 使用者體驗優化
- **視覺回饋**: 增加即時狀態顯示
- **聲音提示**: 在模式切換時提供音效回饋
- **說明文檔**: 提供詳細的操作手冊

### 3. 安全性增強
- **速度限制**: 實作硬體層面的安全速度限制
- **狀態監控**: 增加機器人狀態監控機制
- **緊急停止**: 實作無線緊急停止按鈕

### 4. 擴展性考量
- **多機器人支援**: 支援同時控制多台機器人
- **手把支援**: 整合遊戲手把控制介面
- **網路控制**: 支援遠程網路控制功能

## 相關技術延伸

### ROS2 進階概念
- **DDS 中介軟體**: 了解底層通訊機制
- **QoS 政策**: 深入學習服務品質設定
- **生命週期管理**: 學習節點生命週期控制

### 機器人運動學
- **差動驅動**: 理解雙輪差動機器人運動學
- **阿克曼轉向**: 了解汽車式轉向機制  
- **全向移動**: 學習全向輪和麥輪運動原理

### 控制理論
- **PID 控制**: 學習比例積分微分控制
- **濾波演算法**: 了解卡爾曼濾波等技術
- **路徑規劃**: 學習機器人路徑規劃演算法

這個技術說明文件涵蓋了 wheeltec_keyboard.py 程式的所有重要技術層面，有助於深入理解和擴展此機器人控制系統。
