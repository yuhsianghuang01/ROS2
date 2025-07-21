#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
雷射跟隨系統啟動檔案
=====================================

技術背景知識：

1. 雷射跟隨系統架構：
   - 機器人底盤：提供基本移動能力和感測器介面
   - LiDAR 感測器：提供 360° 距離掃描資料
   - 雷射追蹤器：分析掃描資料，偵測移動物體
   - 雷射跟隨器：根據追蹤結果控制機器人運動

2. 系統資料流：
   LiDAR → /scan 主題 → laserTracker → /object_tracker/current_position 
   → laserFollower → /cmd_vel → 機器人底盤

3. 安全設計：
   - 多層級安全檢查：連線監控、速度限制、距離安全
   - 失效安全機制：任何組件故障時自動停止
   - 人機介面：手把控制器提供手動干預能力

4. 模組化設計：
   - turn_on_wheeltec_robot.launch.py：機器人基本系統
   - wheeltec_lidar.launch.py：LiDAR 驅動和配置
   - lasertracker：物體偵測和追蹤演算法
   - laserfollower：運動控制和安全監控

程式功能：
啟動完整的雷射跟隨系統，包括機器人底盤、
LiDAR 感測器、物體追蹤和跟隨控制功能。

適用場景：
- 人員跟隨機器人
- 自動搬運系統
- 移動目標追蹤
- 智慧導航應用
"""

import os                                                    # 作業系統介面模組
from ament_index_python.packages import get_package_share_directory  # ROS2 套件路徑工具
from launch import LaunchDescription                         # Launch 系統核心類別
from launch.actions import (DeclareLaunchArgument,          # 啟動參數宣告
                            GroupAction,                     # 動作群組化
                            IncludeLaunchDescription,        # 包含其他 launch 檔案
                            SetEnvironmentVariable)          # 環境變數設定
from launch.launch_description_sources import PythonLaunchDescriptionSource  # Python launch 檔案來源
from launch.actions import DeclareLaunchArgument            # 重複匯入
from launch.substitutions import LaunchConfiguration        # 啟動時參數替換
import launch_ros.actions                                    # ROS2 特定動作

#def launch(launch_descriptor, argv):  # ROS1 風格函數 (已過時)

def generate_launch_description():
    """
    產生雷射跟隨系統啟動描述
    
    回傳：
        LaunchDescription: 完整的系統啟動描述
        
    系統組件：
    1. 機器人底盤系統 (turn_on_wheeltec_robot)
    2. LiDAR 感測器系統 (wheeltec_lidar)
    3. 雷射追蹤器節點 (lasertracker)
    4. 雷射跟隨器節點 (laserfollower)
    
    啟動順序：
    先啟動硬體驅動程序，再啟動應用層演算法，
    確保資料流和通訊管道建立完成。
    """

    # 取得 turn_on_wheeltec_robot 套件的資源路徑
    # 此套件包含 Wheeltec 機器人的基本驅動和配置
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
    
    # 機器人基本系統啟動
    # 包含：底盤驅動、IMU、編碼器、電源管理等基礎功能
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')
            ),
    )
    
    # LiDAR 感測器系統啟動
    # 包含：雷射掃描器驅動、資料濾波、座標轉換
    # 輸出主題：/scan (sensor_msgs/LaserScan)
    wheeltec_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'wheeltec_lidar.launch.py')
            ),
    )
    
    # 回傳完整的啟動描述
    return LaunchDescription([
        # 硬體系統 (依序啟動以確保相依性)
        wheeltec_robot,      # 機器人底盤
        wheeltec_lidar,      # LiDAR 感測器
        
        # 應用層演算法節點
        launch_ros.actions.Node(
            package='simple_follower_ros2',   # 套件名稱
            executable='lasertracker',        # 可執行檔：雷射追蹤器
            name='lasertracker',              # 節點名稱
            # 可選配置：
            # output='screen',                # 輸出到螢幕
            # parameters=[                    # 節點參數
            #     {'winSize': 3},             # 比較視窗大小
            #     {'deltaDist': 0.15}         # 距離變化閾值
            # ],
            # remappings=[                    # 主題重新映射
            #     ('scan', '/lidar/scan')     # 如果需要重新映射掃描主題
            # ]
        ),
        
        launch_ros.actions.Node(
            package='simple_follower_ros2',   # 套件名稱
            executable='laserfollower',       # 可執行檔：雷射跟隨器
            # 注意：未指定 name，將使用預設節點名稱
            # 可選配置：
            # name='laserfollower',           # 明確指定節點名稱
            # output='screen',               # 顯示輸出訊息
            # parameters=[                   # PID 控制參數
            #     {'P': [1.5, 0.5]},         # 比例增益
            #     {'I': [0.0, 0.0]},         # 積分增益
            #     {'D': [0.02, 0.002]},      # 微分增益
            #     {'maxSpeed': 0.3}          # 最大速度限制
            # ]
        ),
    ])

