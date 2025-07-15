#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
雷射物體追蹤器程式
=====================================

技術背景知識：

1. LiDAR (Light Detection and Ranging) 原理：
   - 發射雷射光束並測量反射時間
   - 計算距離：距離 = (光速 × 時間差) / 2
   - 360度掃描產生距離點雲資料
   - 常見型號：RPLIDAR, Hokuyo, Velodyne 等

2. 物體追蹤演算法：
   - 時間關聯：比較連續掃描間的距離變化
   - 空間濾波：使用滑動視窗減少雜訊影響
   - 最近鄰追蹤：優先追蹤距離最近的移動物體
   - 資料關聯：判斷不同時刻的測量是否為同一物體

3. 雷射掃描資料結構：
   - ranges[]: 距離陣列，按角度順序排列
   - angle_min/max: 掃描角度範圍
   - angle_increment: 角度解析度
   - range_min/max: 有效距離範圍

4. 座標系統與角度計算：
   - 機器人座標系：前方為 X 軸正向
   - 角度計算：起始角 + 索引 × 角度增量
   - 位置向量：(距離, 角度) 表示目標位置

5. ROS2 訊息類型：
   - LaserScan: 標準雷射掃描資料格式
   - Position: 自定義位置訊息 (角度 + 距離)

程式功能：
接收 LiDAR 掃描資料，透過比較連續掃描找出移動物體，
計算最近目標的角度和距離資訊，供後續跟隨控制器使用。

適用場景：
- 人員跟隨機器人
- 動態障礙物追蹤
- 移動目標偵測與定位
"""

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy, LaserScan
from turn_on_wheeltec_robot.msg import Position as PositionMsg
from std_msgs.msg import String as StringMsg	

class LaserTracker(Node):
    """
    雷射追蹤器節點類別
    
    主要功能：
    1. 接收並處理 LiDAR 掃描資料
    2. 與歷史掃描資料比較，偵測移動物體
    3. 計算最近目標的位置資訊 (距離 + 角度)
    4. 發布追蹤結果供控制器使用
    
    追蹤原理：
    - 時序比較：對比當前與上次掃描的距離差異
    - 空間關聯：使用鄰近視窗建立資料關聯
    - 最近優先：優先追蹤距離最近的有效目標
    - 變化檢測：僅追蹤距離變化在合理範圍內的點
    
    技術特點：
    - 即時處理：低延遲的目標偵測
    - 雜訊過濾：排除異常距離變化
    - 穩定追蹤：優先選擇連續性好的目標
    """

    def __init__(self):
        """
        雷射追蹤節點初始化
        
        設定項目：
        1. ROS2 節點基礎架構
        2. 追蹤演算法參數
        3. 訊息發布器和訂閱器
        4. 歷史資料儲存
        
        參數說明：
        - winSize: 比較視窗大小，用於空間關聯
        - deltaDist: 距離變化閾值，過濾異常變化
        """
        super().__init__('lasertracker')  # 建立名為 'lasertracker' 的 ROS2 節點
        
        # 歷史掃描資料：儲存上一次的距離測量結果
        # 用於與當前掃描比較，實現時序追蹤
        self.lastScan=None
        
        # QoS 設定：確保資料傳輸品質
        qos = QoSProfile(depth=10)  # 佇列深度 10，平衡延遲與可靠性
        
        # 宣告節點參數：可通過 launch 檔案或命令列調整
        # 比較視窗大小：決定空間關聯的範圍
        self.winSize = self.declare_parameter('~winSize', 2)
        
        # 距離變化閾值：超過此值的變化視為異常
        # 單位：公尺，預設 0.2m
        self.deltaDist =self.declare_parameter('~deltaDist', 0.2)
        
        # 位置資訊發布器：將追蹤結果傳送給控制器
        # 主題：'object_tracker/current_position'
        # 訊息：包含目標的角度和距離資訊
        self.positionPublisher = self.create_publisher(PositionMsg, 'object_tracker/current_position', qos)
        
        # 狀態資訊發布器：發送追蹤狀態和警告訊息
        # 主題：'object_tracker/info'
        # 用途：通知其他節點追蹤狀態（如目標丟失）
        self.infoPublisher = self.create_publisher(StringMsg, 'object_tracker/info', qos)
        
        # 雷射掃描訂閱器：接收 LiDAR 資料
        # 主題：'/scan' - 標準 LaserScan 訊息
        # 回調：registerScan - 處理每次新的掃描資料
        self.scanSubscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.registerScan,
            qos)

    def registerScan(self, scan_data):
        """
        雷射掃描處理回調函數 - 核心追蹤演算法
        
        參數：
            scan_data (sensor_msgs/LaserScan): 雷射掃描資料
            
        演算法流程：
        1. 距離資料預處理與排序
        2. 與歷史掃描資料比較
        3. 空間關聯與變化檢測
        4. 最近目標選擇
        5. 位置計算與結果發布
        
        技術細節：
        - 使用 numpy 提升計算效率
        - 按距離排序優先處理近距離目標
        - 滑動視窗實現空間資料關聯
        - 距離變化閾值過濾雜訊
        
        輸出：
        - 成功：目標位置資訊 (角度 + 距離)
        - 失敗：警告訊息通知目標丟失
        """
        # 距離資料預處理：轉換為 numpy 陣列便於計算
        ranges = np.array(scan_data.ranges)
        
        # sort by distance to check from closer to further away points if they might be something real
        # 按距離排序：優先處理近距離測量點
        # 原理：近距離目標通常更重要且測量更準確
        sortedIndices = np.argsort(ranges)
        
        # 初始化追蹤變數
        minDistanceID = None      # 最近目標的角度索引
        minDistance = float('inf') # 最近目標的距離值
        
        # 檢查是否有歷史掃描資料可供比較
        if(not(self.lastScan is None)):
            # if we already have a last scan to compare to:
            # 有歷史資料：進行時序比較追蹤
            
            # 遍歷所有測量點，從最近的開始檢查
            for i in sortedIndices:
                # check all distance measurements starting from the closest one
                tempMinDistance = ranges[i]  # 當前測量點距離
                
                # 計算空間關聯視窗範圍
                # 目的：檢查當前點附近是否有對應的歷史測量
                a = i-2  # 視窗左邊界（向左擴展 2 個點）
                b = i+3  # 視窗右邊界（向右擴展 3 個點）
                c = len(self.lastScan)  # 歷史掃描資料長度
                
                # 確保視窗索引在有效範圍內
                # np.clip: 將索引限制在 [0, c] 範圍內
                windowIndex = np.clip([a,b],0,c)
                
                # 提取歷史掃描中對應視窗的距離資料
                window = self.lastScan[windowIndex[0]:windowIndex[1]]
                
                # 資料關聯檢查：忽略數值異常
                with np.errstate(invalid='ignore'):
                    # 檢查視窗內是否有距離變化合理的點
                    # 條件：|歷史距離 - 當前距離| <= 0.2m
                    # 原理：真實物體的距離變化應該是連續的
                    if(np.any(abs(window-tempMinDistance)<=0.2)):
                        # 找到符合條件的目標
                        minDistanceID = i                    # 記錄角度索引
                        minDistance = ranges[minDistanceID]  # 記錄距離值
                        break # at least one point was equally close
                        # 跳出迴圈：已找到最近的有效目標
                        
        # 更新歷史掃描資料：為下次比較做準備
        self.lastScan=ranges

        # 結果判斷與發布
        if(minDistance > scan_data.range_max):
            # 未找到有效目標：距離超出感測器量程
            
            # 建立警告訊息
            msg = StringMsg()
            msg.data = 'laser:nothing found'
            
            # 記錄警告到 ROS2 日誌系統
            self.get_logger().warn('laser no object found')
            
            # 發布狀態訊息：通知其他節點目標丟失
            self.infoPublisher.publish(msg)
            
            # 輸出等待訊息
            self.get_logger().info(' waiting again...')
        else:
            # 成功找到目標：計算並發布位置資訊
            
            # 建立位置訊息物件
            msgdata = PositionMsg()
            
            # 計算目標角度：起始角度 + 索引 × 角度增量
            # 這是 LiDAR 標準的角度計算公式
            minDistanceAngle = scan_data.angle_min + minDistanceID * scan_data.angle_increment
            
            # 設定位置訊息內容
            msgdata.angle_x = minDistanceAngle    # X 軸角度（水平方向）
            msgdata.angle_y = 42.0               # Y 軸角度（固定值，可能用於標識）
            
            # 除錯輸出：顯示找到目標的角度索引
            print(minDistanceID)
            
            msgdata.distance = float(minDistance) # 目標距離（轉換為浮點數）
            
            # 發布目標位置資訊
            # 控制器節點會接收此訊息並據此調整機器人行為
            self.positionPublisher.publish(msgdata)

def main(args=None):
    """
    主程式執行函數
    
    參數：
        args: 命令列參數（可選）
        
    功能說明：
    1. 初始化 ROS2 通訊系統
    2. 建立雷射追蹤節點
    3. 進入事件驅動迴圈
    4. 處理程式退出與資源清理
    
    執行模式：
    - 事件驅動：使用 rclpy.spin() 等待並處理訊息
    - 阻塞式：程式會持續運行直到收到中斷信號
    - 自動清理：確保程式結束時正確釋放資源
    
    適用環境：
    - 需要連接支援的 LiDAR 感測器
    - 感測器驅動節點需正常發布 /scan 主題
    - 建議在開放空間測試以獲得較好效果
    """
    print('starting')  # 啟動提示訊息
    
    # 初始化 ROS2 分散式通訊系統
    rclpy.init(args=args)
    
    # 建立雷射追蹤節點實例
    lasertracker = LaserTracker()
    
    print('seem to do something')  # 初始化完成提示
    
    try:
        # 進入 ROS2 事件迴圈：等待並處理訊息
        # spin() 會阻塞程式直到收到中斷信號 (Ctrl+C)
        # 每當收到新的 LaserScan 訊息就會呼叫 registerScan()
        rclpy.spin(lasertracker)
    finally:
        # 確保程式結束時正確清理資源
        # 無論正常退出或異常中斷都會執行
        lasertracker.destroy_node()  # 銷毀節點實例
        rclpy.shutdown()            # 關閉 ROS2 系統

if __name__ == '__main__':
    """
    Python 模組直接執行進入點
    
    執行方式：
    1. 直接執行：python3 laserTracker.py
    2. ROS2 指令：ros2 run simple_follower_ros2 laserTracker
    
    系統需求：
    1. 硬體：支援的 LiDAR 感測器 (如 RPLIDAR A1/A2)
    2. 軟體：對應的 ROS2 驅動節點
    3. 網路：正確的 ROS2 通訊設定
    
    使用前檢查：
    1. 感測器連接與驅動狀態
       $ ros2 topic list | grep scan
    2. 掃描資料品質
       $ ros2 topic echo /scan
    3. 訊息發布頻率
       $ ros2 topic hz /scan
    
    除錯建議：
    1. 使用 rviz2 視覺化掃描資料
    2. 調整 deltaDist 參數適應環境
    3. 檢查感測器安裝高度與角度
    4. 注意移動物體的大小與速度限制
    
    常見問題：
    1. 目標丟失：可能是移動太快或距離變化過大
    2. 追蹤不穩定：考慮調整視窗大小和距離閾值
    3. 誤追蹤：環境中可能有其他移動物體干擾
    """
    main()
