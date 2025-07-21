#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
線條跟隨系統啟動檔案
=====================================

技術背景知識：

1. 線條跟隨系統架構：
   - 機器人底盤：提供基本移動能力
   - 相機系統：提供即時影像串流
   - 線條跟隨演算法：影像處理和路徑控制
   - 視覺化介面：顯示處理結果和調整參數

2. 視覺導航原理：
   - 影像擷取：RGB 相機提供彩色影像
   - 顏色篩選：HSV 色彩空間進行線條偵測
   - 中心計算：影像矩計算線條重心位置
   - 控制輸出：PD 控制器產生轉向指令

3. 系統資料流：
   相機 → /camera/color/image_raw → line_follow → 影像處理 
   → 誤差計算 → PD 控制 → /cmd_vel → 機器人移動

4. 應用場景：
   - 工廠自動導引車 (AGV)
   - 倉儲物流機器人
   - 清潔機器人路徑規劃
   - 教育機器人教學示範

程式功能：
啟動基於視覺的線條跟隨系統，
機器人可沿著預設顏色的線條自動移動。

設計特點：
- 即插即用：自動偵測和啟動相機
- 參數可調：支援多種線條顏色
- 即時視覺化：顯示處理過程和結果
- 安全可靠：失去目標時自動停止
"""

import os                                                    # 作業系統介面
import launch_ros.actions                                    # ROS2 節點啟動動作
from ament_index_python.packages import get_package_share_directory  # 套件路徑查詢
from launch import LaunchDescription                         # Launch 系統描述
from launch.actions import (DeclareLaunchArgument,          # 啟動參數宣告
                            IncludeLaunchDescription)        # 包含其他 launch 檔案
from launch.launch_description_sources import PythonLaunchDescriptionSource  # Python launch 來源
from launch.actions import DeclareLaunchArgument            # 重複匯入 (可移除)
from launch.substitutions import LaunchConfiguration        # 啟動時參數替換
from launch.conditions import IfCondition                   # 條件啟動
from launch.conditions import UnlessCondition               # 反向條件啟動
from launch.actions import ExecuteProcess                   # 程序執行

def generate_launch_description():
    """
    產生線條跟隨系統啟動描述
    
    回傳：
        LaunchDescription: 系統啟動配置
        
    系統組件：
    1. 機器人底盤驅動
    2. 相機驅動和影像處理
    3. 線條跟隨演算法節點
    
    可選參數：
    - is_uvc_cam: 是否使用 UVC 相機 (預設 false)
    
    啟動邏輯：
    依序啟動底盤、相機、跟隨演算法，
    確保每個組件都能正常獲得所需資源。
    """
    
    # 啟動參數：相機類型選擇
    # 'false': 使用 RealSense 或其他專用相機
    # 'true': 使用標準 UVC (USB Video Class) 相機
    is_uvc_cam = LaunchConfiguration('is_uvc_cam', default='false')

    # 取得機器人基本套件路徑
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # 相機系統啟動
    # 功能：啟動相機驅動，提供影像串流
    # 輸出主題：/camera/color/image_raw (彩色影像)
    #          /camera/depth/image_raw (深度影像，如果支援)
    wheeltec_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'wheeltec_camera.launch.py')
            ),
    )
    
    # 機器人基本系統啟動
    # 功能：底盤驅動、馬達控制、感測器介面
    # 提供：/cmd_vel 速度控制介面
    #       /odom 里程計資訊 (如果需要)
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')
            ),
    )
    
    # 回傳系統啟動描述
    return LaunchDescription([
        # 硬體驅動 (先啟動確保資源可用)
        wheeltec_robot,      # 機器人底盤系統
        wheeltec_camera,     # 相機影像系統
        
        # 應用層演算法
        launch_ros.actions.Node(
            package='simple_follower_ros2',   # 套件名稱
            executable='line_follow',         # 可執行檔：線條跟隨演算法
            name='line_follow',               # 節點名稱
            # 可選配置：
            # output='screen',                # 輸出日誌到螢幕
            # parameters=[                    # 節點參數
            #     {'linear_speed': 0.11},     # 前進速度
            #     {'angular_gain': 0.0011},   # 轉向增益
            #     {'roi_height': 30}          # 興趣區域高度
            # ],
            # remappings=[                    # 主題重新映射
            #     ('image_raw', '/camera/color/image_raw')  # 影像主題
            # ]
        )
    ])

