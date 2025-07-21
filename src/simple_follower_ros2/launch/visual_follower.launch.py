#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
視覺跟隨系統啟動檔案
=====================================

技術背景知識：

1. 視覺跟隨系統架構：
   - 機器人底盤：提供移動控制和基礎功能
   - RGB-D 相機：提供彩色影像和深度資訊
   - 視覺追蹤器：物體偵測和3D位置計算
   - 視覺跟隨器：PID控制和運動規劃

2. 3D 視覺追蹤原理：
   - 顏色分割：HSV色彩空間進行目標物體篩選
   - 輪廓分析：計算物體形狀、大小和中心位置
   - 深度融合：結合RGB和深度資訊獲得3D座標
   - 座標轉換：像素座標轉換為機器人座標系

3. 系統資料流：
   RGB-D相機 → 影像同步 → visualTracker → 3D位置計算
   → /object_tracker/current_position → visualFollower 
   → PID控制 → /cmd_vel → 機器人運動

4. 多感測器整合：
   - 相機：提供視覺資訊和深度資料
   - 手把控制：模式切換和安全監控
   - IMU/編碼器：運動回饋和狀態估計

5. 應用場景：
   - 人員跟隨助手機器人
   - 物體搬運和運輸
   - 互動式導覽機器人
   - 智慧購物車系統

程式功能：
啟動完整的視覺跟隨系統，機器人可自動
跟隨指定顏色的物體，保持適當距離。

技術特點：
- 3D感知：結合RGB和深度資訊
- 即時追蹤：低延遲的目標偵測和跟隨
- 安全控制：多重安全機制和緊急停止
- 人機互動：手把控制器模式管理
"""

import os                                                    # 作業系統介面
from ament_index_python.packages import get_package_share_directory  # 套件路徑工具
from launch import LaunchDescription                         # Launch 系統核心
from launch.actions import (DeclareLaunchArgument,          # 啟動參數
                            GroupAction,                     # 動作群組
                            IncludeLaunchDescription,        # 包含launch檔案
                            SetEnvironmentVariable)          # 環境變數
from launch.launch_description_sources import PythonLaunchDescriptionSource  # Python launch來源
from launch.actions import DeclareLaunchArgument            # 重複匯入
from launch.substitutions import LaunchConfiguration        # 參數替換
import launch_ros.actions                                    # ROS2節點動作
from launch.conditions import IfCondition                   # 條件執行

#def launch(launch_descriptor, argv):  # ROS1 風格 (已過時)

def generate_launch_description():
    """
    產生視覺跟隨系統啟動描述
    
    回傳：
        LaunchDescription: 完整的系統啟動配置
        
    系統組件：
    1. 機器人底盤驅動 (turn_on_wheeltec_robot)
    2. RGB-D 相機系統 (wheeltec_camera)
    3. 視覺物體追蹤器 (visualtracker)
    4. 視覺跟隨控制器 (visualfollow)
    
    啟動順序：
    硬體驅動 → 感測器系統 → 追蹤演算法 → 控制系統
    確保資料流暢通和系統穩定性。
    
    節點間通訊：
    - visualtracker 發布目標位置到 /object_tracker/current_position
    - visualfollow 訂閱位置資訊並發布控制指令到 /cmd_vel
    - 所有節點共享 /tf 座標變換資訊
    """
    
    # 取得機器人基本套件的安裝路徑
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # 機器人基本系統啟動
    # 功能：底盤驅動、馬達控制、感測器介面、安全監控
    # 提供：/cmd_vel速度控制、/odom里程計、/tf座標變換
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')
            ),
    )

    # RGB-D 相機系統啟動
    # 功能：相機驅動、影像處理、深度計算、相機標定
    # 輸出：/camera/color/image_raw (RGB影像)
    #      /camera/depth/image_raw (深度影像)
    #      /camera/color/camera_info (相機內參)
    wheeltec_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'wheeltec_camera.launch.py')
            ),
    )
    
    # 回傳完整的系統啟動描述
    return LaunchDescription([
        # 硬體系統 (按相依性順序啟動)
        wheeltec_robot,      # 機器人底盤 (基礎平台)
        wheeltec_camera,     # RGB-D 相機 (感測器)
        
        # 應用層演算法節點
        launch_ros.actions.Node(
            package='simple_follower_ros2',   # 套件名稱
            executable='visualtracker',       # 可執行檔：視覺追蹤器
            name='visualtracker',             # 節點名稱
            # 可選配置：
            # output='screen',                # 日誌輸出到螢幕
            # parameters=[                    # 節點參數
            #     {'target_color_lower': [0, 50, 50]},      # HSV下限
            #     {'target_color_upper': [10, 255, 255]},   # HSV上限
            #     {'min_contour_area': 500},                # 最小輪廓面積
            #     {'camera_fov_h': 69.4},                   # 相機水平視角
            #     {'camera_fov_v': 42.5}                    # 相機垂直視角
            # ],
            # remappings=[                    # 主題重新映射
            #     ('color_image', '/camera/color/image_raw'),
            #     ('depth_image', '/camera/depth/image_raw')
            # ]
        ),
        
        launch_ros.actions.Node(
            package='simple_follower_ros2',   # 套件名稱
            executable='visualfollow',        # 可執行檔：視覺跟隨器
            name='visualfollow',              # 節點名稱
            output='screen',                  # 顯示節點輸出到螢幕
            # 可選配置：
            # parameters=[                    # PID 控制參數
            #     {'angular_pid': [1.6, 0.0, 0.03]},      # 角度控制PID
            #     {'distance_pid': [0.5, 0.0, 0.005]},    # 距離控制PID
            #     {'target_distance': 0.8},                # 目標跟隨距離
            #     {'max_linear_speed': 0.3},               # 最大線速度
            #     {'max_angular_speed': 0.4},              # 最大角速度
            #     {'min_safe_distance': 0.15}              # 最小安全距離
            # ]
        ),
    ])

