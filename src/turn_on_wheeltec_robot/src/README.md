# 輪趣機器人 ROS2 控制系統

## 專案概述

本專案為輪趣(Wheeltec)機器人的 ROS2 控制系統，支援多種類型的移動機器人平台，包括差動輪式機器人、阿克曼轉向機器人和全向移動機器人。系統提供完整的底層控制、感測器數據處理、導航定位功能。

## 系統架構

```
ROS2 機器人控制系統
├── 硬體層
│   ├── STM32 下位機控制器
│   ├── MPU6050 慣性測量單元
│   ├── 編碼器里程計
│   └── 串口通訊(115200 baud)
├── 驅動層
│   ├── 串口通訊驅動
│   ├── IMU 數據處理
│   └── 里程計數據融合
├── ROS2 節點層
│   ├── turn_on_wheeltec_robot (主控制節點)
│   ├── 速度控制訂閱
│   ├── IMU 數據發布
│   └── 里程計數據發布
└── 應用層
    ├── 導航系統
    ├── SLAM 建圖
    └── 路徑規劃
```

## 主要功能模組

### 1. 主控制節點 (`turn_on_wheeltec_robot`)

**檔案位置**: `src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp`

**核心功能**:
- 串口通訊管理
- 速度指令處理
- 感測器數據發布
- 系統狀態監控

**主要類別**: `turn_on_robot`
- 繼承自 `rclcpp::Node`
- 管理所有硬體介面和 ROS2 通訊

### 2. 四元數姿態解算 (`Quaternion_Solution`)

**檔案位置**: `src/turn_on_wheeltec_robot/src/Quaternion_Solution.cpp`

**核心演算法**: Madgwick AHRS 濾波器
- **輸入**: 三軸陀螺儀數據 (gx, gy, gz) + 三軸加速度計數據 (ax, ay, az)
- **輸出**: 四元數姿態表示 (q0, q1, q2, q3)
- **優勢**: 避免萬向節鎖問題，計算效率高

**數學原理**:
```
1. 加速度計數據正規化 → 單位重力向量
2. 四元數轉換 → 估計重力方向
3. 誤差計算 → 向量外積
4. PID 控制 → 比例積分回饋
5. 四元數積分 → 姿態更新
6. 正規化 → 保持單位四元數
```

### 3. 速度控制介面

**支援的控制模式**:

#### A. 一般差動輪式控制 (`Cmd_Vel_Callback`)
- **訊息類型**: `geometry_msgs::msg::Twist`
- **控制參數**:
  - `linear.x`: 前進後退速度 (m/s)
  - `linear.y`: 左右平移速度 (m/s) - 全向輪使用
  - `angular.z`: 旋轉角速度 (rad/s)

#### B. 阿克曼轉向控制 (`Akm_Cmd_Vel_Callback`)
- **訊息類型**: `ackermann_msgs::msg::AckermannDriveStamped`
- **控制參數**:
  - `drive.speed`: 目標速度 (m/s)
  - `drive.steering_angle`: 轉向角度 (rad)

### 4. 感測器數據處理

#### IMU 數據發布 (`Publish_ImuSensor`)
- **話題**: `/mobile_base/sensors/imu_data`
- **訊息類型**: `sensor_msgs::msg::Imu`
- **包含數據**:
  - 四元數姿態 (orientation)
  - 三軸角速度 (angular_velocity)
  - 三軸線性加速度 (linear_acceleration)
  - 協方差矩陣 (covariance matrices)

#### 里程計數據發布 (`Publish_Odom`)
- **話題**: `/odom`
- **訊息類型**: `nav_msgs::msg::Odometry`
- **包含數據**:
  - 位置資訊 (position)
  - 姿態資訊 (orientation)
  - 線速度 (linear velocity)
  - 角速度 (angular velocity)

## 串口通訊協議

### 數據封包格式

#### 發送封包 (ROS2 → STM32)
```
位元組 | 內容 | 說明
-------|------|------
0      | 0x7B | 帧頭
1      | 0x00 | 預留
2      | 0x00 | 預留
3-4    | 速度高低位 | X軸線速度 (mm/s)
5-6    | 速度高低位 | Y軸線速度 (mm/s)
7-8    | 角速度高低位 | Z軸角速度 (mrad/s)
9      | 校驗碼 | BBC校驗
10     | 0x7D | 帧尾
```

#### 接收封包 (STM32 → ROS2)
```
位元組 | 內容 | 說明
-------|------|------
0      | 0x7B | 帧頭
1-2    | X軸速度 | 編碼器速度回饋
3-4    | Y軸速度 | 編碼器速度回饋
5-6    | Z軸角速度 | 編碼器角速度回饋
7-8    | 加速度X | MPU6050數據
9-10   | 加速度Y | MPU6050數據
11-12  | 加速度Z | MPU6050數據
13-14  | 陀螺儀X | MPU6050數據
15-16  | 陀螺儀Y | MPU6050數據
17-18  | 陀螺儀Z | MPU6050數據
19-20  | 電源電壓 | 系統電壓監控
21     | 狀態標誌 | 系統狀態
22     | 校驗碼 | BBC校驗
23     | 0x7D | 帧尾
```

### BBC校驗算法
```cpp
// 計算校驗碼：第1到第n-2個位元組的異或結果
unsigned char check_sum = 0;
for(int i = 0; i < n-2; i++) {
    check_sum = check_sum ^ data[i];
}
```

## 系統參數配置

### ROS2 參數列表
```yaml
serial_baud_rate: 115200        # 串口波特率
usart_port_name: "/dev/ttyCH343USB0"  # 串口設備路徑
cmd_vel: "cmd_vel"              # 速度指令話題名稱
akm_cmd_vel: "ackermann_cmd"    # 阿克曼指令話題名稱
odom_frame_id: "odom"           # 里程計座標框架
robot_frame_id: "base_link"     # 機器人座標框架
gyro_frame_id: "gyro_link"      # IMU座標框架
```

### IMU校準參數
```cpp
#define GYROSCOPE_RATIO   0.00026644f  // 陀螺儀轉換比例 (±500°/s)
#define ACCEl_RATIO       1671.84f     // 加速度計轉換比例 (±2g)
#define SAMPLING_FREQ     20.0f        // 採樣頻率 20Hz
```

## 編譯與運行

### 前置需求
- Ubuntu 20.04/22.04
- ROS2 Humble/Foxy
- 相關 ROS2 套件:
  ```bash
  sudo apt install ros-humble-ackermann-msgs
  sudo apt install ros-humble-sensor-msgs
  sudo apt install ros-humble-nav-msgs
  sudo apt install ros-humble-tf2
  sudo apt install ros-humble-serial
  ```

### 編譯步驟
```bash
# 進入工作空間
cd ~/ros2_ws

# 複製專案至 src 目錄
cp -r ROS2/* src/

# 安裝相依套件
rosdep install --from-paths src --ignore-src -r -y

# 編譯
colcon build

# 載入環境
source install/setup.bash
```

### 啟動系統
```bash
# 啟動主控制節點
ros2 run turn_on_wheeltec_robot wheeltec_robot

# 或使用 launch 檔案
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

### 測試指令
```bash
# 發送速度指令 (差動輪式)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"

# 查看 IMU 數據
ros2 topic echo /mobile_base/sensors/imu_data

# 查看里程計數據
ros2 topic echo /odom
```

## 故障排除

### 常見問題

1. **串口權限問題**
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   # 或加入用戶至 dialout 群組
   sudo usermod -a -G dialout $USER
   ```

2. **找不到串口設備**
   ```bash
   # 列出可用串口
   ls /dev/tty*
   # 修改參數檔案中的 usart_port_name
   ```

3. **IMU數據異常**
   - 檢查 MPU6050 接線
   - 確認 I2C 通訊正常
   - 檢查電源供應穩定性

4. **里程計漂移**
   - 校準編碼器參數
   - 檢查輪子直徑設定
   - 確認軸距參數正確

### 調試工具
```bash
# 查看節點狀態
ros2 node list
ros2 node info /wheeltec_robot

# 查看話題列表
ros2 topic list

# 監控話題頻率
ros2 topic hz /odom
ros2 topic hz /mobile_base/sensors/imu_data

# 查看 TF 變換
ros2 run tf2_tools view_frames.py
```

## 系統擴展

### 添加新感測器
1. 在 `wheeltec_robot.h` 中添加數據結構
2. 在 `wheeltec_robot.cpp` 中添加發布器
3. 修改串口協議以支援新數據
4. 更新校驗碼計算

### 支援新的機器人平台
1. 修改運動學模型
2. 調整速度指令解析
3. 更新里程計計算
4. 修改 TF 變換關係

## 效能優化

### 建議設定
- **串口波特率**: 115200 (可提升至 460800)
- **IMU發布頻率**: 100Hz
- **里程計發布頻率**: 50Hz
- **控制迴圈頻率**: 20Hz

### 系統調優
```cpp
// 在 wheeltec_robot.cpp 中調整發布頻率
imu_timer = create_wall_timer(10ms, [=]() { Publish_ImuSensor(); });      // 100Hz
odom_timer = create_wall_timer(20ms, [=]() { Publish_Odom(); });          // 50Hz
```

## 授權與貢獻

本專案基於輪趣機器人官方程式碼開發，僅供學習和研究用途。

如需貢獻代碼或回報問題，請遵循以下步驟：
1. Fork 本專案
2. 創建功能分支
3. 提交變更
4. 發起 Pull Request

## 相關資源

- [ROS2 官方文檔](https://docs.ros.org/en/humble/)
- [輪趣機器人官網](http://www.wheeltec.net/)
- [Madgwick AHRS 算法論文](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
- [四元數數學基礎](https://en.wikipedia.org/wiki/Quaternion)
