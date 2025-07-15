#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HSV 顏色調整工具程式
======================================

技術背景知識：
1. HSV 色彩空間：
   - H (Hue 色調): 0-179°，表示顏色種類
   - S (Saturation 飽和度): 0-255，表示顏色純度
   - V (Value 明度): 0-255，表示顏色亮度
   
2. OpenCV 色彩轉換：
   - BGR -> HSV 轉換可以更好地分離顏色
   - cv2.inRange() 用於顏色範圍篩選
   
3. 形態學運算：
   - 侵蝕(erosion): 去除小噪點
   - 膨脹(dilation): 填補小孔洞
   
4. ROS2 概念：
   - Node: ROS2 中的基本執行單元
   - Publisher/Subscriber: 發布/訂閱通訊模式
   - QoS: 服務品質設定，控制訊息傳輸特性

程式功能：
此程式提供一個互動式介面，讓使用者透過滑動條即時調整HSV顏色範圍參數，
用於機器視覺應用中的顏色篩選和物體偵測預處理。
"""

import cv2
import cv_bridge

import numpy as np
import rclpy
import time
import numpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

def nothing(s):
    """
    空回調函數
    
    參數說明：
        s: 滑動條數值 (在此應用中不使用)
    
    用途：OpenCV 的 createTrackbar 需要一個回調函數，
         但在此應用中我們直接讀取滑動條數值，所以使用空函數
    """
    pass


class Adjust_hsv(Node):
    """
    HSV 調整節點類別
    
    繼承自 ROS2 的 Node 類別，實現：
    1. 訂閱相機影像串流
    2. 建立 HSV 參數調整介面
    3. 即時顯示顏色篩選結果
    
    技術重點：
    - 使用 cv_bridge 進行 ROS 影像訊息與 OpenCV 格式轉換
    - 實現即時影像處理和參數調整
    - 提供視覺化回饋給使用者
    """
    def __init__(self):
        """
        節點初始化函數
        
        設定項目：
        1. ROS2 節點基本配置
        2. OpenCV 橋接器設定
        3. 影像訂閱器建立
        4. 控制變數初始化
        """
        super().__init__('Adjust_hsv')  # 建立名為 'Adjust_hsv' 的 ROS2 節點
        
        # OpenCV 橋接器：用於 ROS 影像訊息與 OpenCV 影像格式間的轉換
        self.bridge = cv_bridge.CvBridge()
        
        # QoS (Quality of Service) 設定：控制訊息傳輸的可靠性和效能
        # depth=10 表示訊息佇列深度為 10，避免訊息堆積
        qos = QoSProfile(depth=10)
        
        # 影像資料儲存變數
        self.mat = None
        
        # 建立影像訂閱器
        # 訂閱主題：'/camera/color/image_raw' (RealSense 相機的標準彩色影像主題)
        # 回調函數：self.image_callback (每次收到新影像時被呼叫)
        self.image_sub = self.create_subscription(Image,'/camera/color/image_raw',self.image_callback,qos)
        
        # 控制變數：確保 GUI 介面只建立一次
        self.tmp=0


    def image_callback(self, msg):
        """
        影像回調函數 - 核心影像處理邏輯
        
        參數：
            msg (sensor_msgs/Image): ROS 影像訊息
            
        處理流程：
        1. 首次執行時建立 HSV 調整介面
        2. 讀取使用者設定的 HSV 範圍參數
        3. 影像格式轉換 (ROS -> OpenCV -> HSV)
        4. 形態學處理去除雜訊
        5. 顏色範圍篩選
        6. 顯示處理結果
        
        技術細節：
        - 使用 cv_bridge 進行影像格式轉換
        - HSV 色彩空間更適合顏色分析
        - 形態學運算提升篩選品質
        """
        # 首次執行時建立調整介面
        if self.tmp==0:
            # 建立可調整大小的視窗
            cv2.namedWindow('Adjust_hsv',cv2.WINDOW_NORMAL)
            
            # 建立 HSV 下限調整滑動條
            # 參數：(標籤, 視窗名稱, 初始值, 最大值, 回調函數)
            cv2.createTrackbar("lowerbH",'Adjust_hsv',0,255,nothing)  # H 色調下限 (0-179°)
            cv2.createTrackbar("lowerbS",'Adjust_hsv',0,255,nothing)  # S 飽和度下限 (0-255)
            cv2.createTrackbar("lowerbV",'Adjust_hsv',0,255,nothing)  # V 明度下限 (0-255)
            
            # 建立 HSV 上限調整滑動條
            cv2.createTrackbar("upperbH",'Adjust_hsv',0,255,nothing)  # H 色調上限
            cv2.createTrackbar("upperbS",'Adjust_hsv',0,255,nothing)  # S 飽和度上限
            cv2.createTrackbar("upperbV",'Adjust_hsv',0,255,nothing)  # V 明度上限
            
            self.tmp=1  # 標記介面已建立，避免重複建立
            
        # 讀取所有滑動條的當前數值
        lowerbH=cv2.getTrackbarPos("lowerbH",'Adjust_hsv')  # 取得 H 下限值
        lowerbS=cv2.getTrackbarPos("lowerbS",'Adjust_hsv')  # 取得 S 下限值
        lowerbV=cv2.getTrackbarPos("lowerbV",'Adjust_hsv')  # 取得 V 下限值
        upperbH=cv2.getTrackbarPos("upperbH",'Adjust_hsv')  # 取得 H 上限值
        upperbS=cv2.getTrackbarPos("upperbS",'Adjust_hsv')  # 取得 S 上限值
        upperbV=cv2.getTrackbarPos("upperbV",'Adjust_hsv')  # 取得 V 上限值

        # 影像格式轉換：ROS Image 訊息 -> OpenCV BGR 格式
        # desired_encoding='bgr8': 指定輸出為 8 位元 BGR 格式
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # hsv将RGB图像分解成色调H，饱和度S，明度V
        # 色彩空間轉換：BGR -> HSV
        # HSV 色彩空間的優點：
        # 1. 更接近人類對顏色的感知
        # 2. 對光照變化較不敏感
        # 3. 更容易進行顏色分離
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 颜色的范围        # 第二个参数：lower指的是图像中低于这个lower的值，图像值变为0
        # 第三个参数：upper指的是图像中高于这个upper的值，图像值变为0
        # 而在lower～upper之间的值变成255
        
        # 形態學運算前處理
        # 建立 5x5 結構元素 (kernel)，用於形態學運算
        kernel = numpy.ones((5,5),numpy.uint8)
        
        # 侵蝕運算：移除小的雜訊點和細線
        # iterations=1: 執行一次侵蝕運算
        hsv_erode = cv2.erode(hsv,kernel,iterations=1)
        
        # 膨脹運算：恢復物體原本大小，同時填補小孔洞
        # 先侵蝕後膨脹的組合稱為「開運算」，可有效去除雜訊
        hsv_dilate = cv2.dilate(hsv_erode,kernel,iterations=1)

        # 顏色範圍篩選：建立二值遮罩
        # cv2.inRange(): 將指定 HSV 範圍內的像素設為 255(白)，其餘設為 0(黑)
        # 參數：(輸入影像, 下限 tuple, 上限 tuple)
        mask=cv2.inRange(hsv_dilate,(lowerbH,lowerbS,lowerbV),(upperbH,upperbS,upperbV))
        
        # 位元運算：將原始影像與遮罩結合
        # cv2.bitwise_and(): 只保留遮罩為白色區域的原始影像內容
        masked = cv2.bitwise_and(image, image, mask=mask)
        
        # 在图像某处绘制一个指示，因为只考虑20行宽的图像，所以使用numpy切片将以外的空间区域清空
        # 顯示處理結果
        # 可選：cv2.imshow("img", image)  # 顯示原始影像
        cv2.imshow("Adjust_hsv", masked)  # 顯示篩選後的影像
        
        # 等待按鍵事件，3ms 超時
        # 此函數必須呼叫，否則 OpenCV 視窗無法正常更新
        cv2.waitKey(3)


def main(args=None):
    """
    主程式進入點
    
    參數：
        args: 命令列參數 (可選)
        
    功能：
    1. 初始化 ROS2 系統
    2. 建立並執行 HSV 調整節點
    3. 處理程式退出和資源清理
    
    執行模式：
    使用 spin_once() 的非阻塞模式，配合 time.sleep()
    控制更新頻率約 10Hz，平衡即時性與系統負載
    """
    # 初始化 ROS2 通訊系統
    rclpy.init(args=args)
    
    # 啟動訊息
    print('start patrolling')
    
    # 建立 HSV 調整節點實例
    adjust_hsv = Adjust_hsv()
    
    # 主執行迴圈
    while rclpy.ok():  # 檢查 ROS2 系統狀態
        # 非阻塞式處理一次節點回調
        # 相比 rclpy.spin()，這種方式可以控制處理頻率
        rclpy.spin_once(adjust_hsv)
        
        # 休眠 0.1 秒，控制迴圈頻率為 10Hz
        # 平衡即時性與 CPU 使用率
        time.sleep(0.1)

    # 程式結束時的清理工作
    adjust_hsv.destroy_node()  # 銷毀節點，釋放 ROS2 資源
    rclpy.shutdown()          # 關閉 ROS2 系統

if __name__ == '__main__':
    """
    Python 模組直接執行時的進入點
    
    使用方式：
    1. 直接執行：python3 adjust_hsv.py
    2. ROS2 啟動：ros2 run simple_follower_ros2 adjust_hsv
    
    注意事項：
    - 需要連接支援的相機 (如 RealSense)
    - 確保相機驅動節點已啟動
    - 影像主題名稱需與實際相機輸出對應
    """
    main()

