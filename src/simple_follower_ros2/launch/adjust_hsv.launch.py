#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HSV 顏色調整工具啟動檔案
=====================================

技術背景知識：

1. ROS2 Launch 系統：
   - 統一管理多個節點的啟動和配置
   - 支援參數傳遞、條件啟動、執行順序控制
   - 使用 Python 語法提供靈活的配置能力
   - 可包含其他 launch 檔案實現模組化設計

2. Launch 檔案結構：
   - generate_launch_description(): 主要函數，回傳 LaunchDescription
   - LaunchDescription: 包含所有啟動動作的容器
   - Node/IncludeLaunchDescription: 具體的啟動動作

3. 相機系統整合：
   - wheeltec_camera.launch.py: 啟動相機驅動和設定
   - 提供標準化的影像串流 (/camera/color/image_raw)
   - 支援多種相機類型 (USB相機、RealSense等)

4. HSV 調整應用：
   - 用於線條跟隨和物體追蹤的顏色參數調整
   - 即時預覽顏色篩選效果
   - 為後續視覺應用提供最佳參數

程式功能：
啟動相機節點和 HSV 顏色調整工具，
提供視覺化介面調整顏色參數，
用於線條跟隨和物體追蹤的前期準備工作。

使用場景：
- 線條跟隨參數調整
- 物體追蹤顏色設定
- 視覺演算法開發除錯
- 環境光照適應調整
"""

import os                                                    # 作業系統介面：檔案路徑操作
import launch_ros.actions                                    # ROS2 特定的啟動動作 (Node 等)
from ament_index_python.packages import get_package_share_directory  # 套件資源路徑查詢
from launch import LaunchDescription                         # Launch 檔案描述容器
from launch.actions import (DeclareLaunchArgument,          # 啟動參數宣告
                            IncludeLaunchDescription)        # 包含其他 launch 檔案
from launch.launch_description_sources import PythonLaunchDescriptionSource  # Python launch 檔案來源
from launch.actions import DeclareLaunchArgument            # 重複匯入 (可移除)
from launch.substitutions import LaunchConfiguration        # 啟動時參數替換
from launch.conditions import IfCondition                   # 條件啟動控制
from launch.conditions import UnlessCondition               # 條件啟動控制 (反向)
from launch.actions import ExecuteProcess                   # 執行系統程序

def generate_launch_description():
    """
    產生啟動描述函數 - ROS2 Launch 系統的核心函數
    
    回傳：
        LaunchDescription: 包含所有啟動動作的描述物件
        
    功能說明：
    1. 取得相關套件的路徑資訊
    2. 包含相機啟動檔案
    3. 建立 HSV 調整節點
    4. 組合成完整的啟動描述
    
    設計理念：
    - 模組化：分離相機驅動和應用程式
    - 可重用：相機啟動檔案可被其他 launch 檔案共用
    - 標準化：遵循 ROS2 Launch 最佳實踐
    """
    
    # 取得 turn_on_wheeltec_robot 套件的共享目錄路徑
    # 這個套件包含機器人基本功能，包括相機驅動
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    
    # 構建 launch 檔案目錄的完整路徑
    # 標準 ROS2 套件結構：<package>/share/<package>/launch/
    launch_dir = os.path.join(bringup_dir, 'launch')

    # 包含相機啟動檔案
    # 功能：啟動相機驅動程序並建立標準化的影像主題
    # 輸出主題：/camera/color/image_raw (RGB 影像)
    #          /camera/depth/image_raw (深度影像，如果支援)
    wheeltec_camera = IncludeLaunchDescription(
            # 指定要包含的 launch 檔案來源
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'wheeltec_camera.launch.py')
            ),
    )
    
    # 回傳完整的啟動描述
    return LaunchDescription([
        # 首先啟動相機節點
        wheeltec_camera,
        
        # 然後啟動 HSV 調整節點
        launch_ros.actions.Node(
            package='simple_follower_ros2',    # 節點所屬套件
            executable='adjust_hsv',           # 可執行檔名稱 (對應 setup.py 中的 entry_points)
            name='adjust_hsv',                 # 節點名稱 (在 ROS2 網路中的識別名稱)
            # 可選參數：
            # output='screen',                 # 輸出重導向到螢幕
            # parameters=[],                   # 節點參數
            # remappings=[],                   # 主題重新映射
            )]
    )

