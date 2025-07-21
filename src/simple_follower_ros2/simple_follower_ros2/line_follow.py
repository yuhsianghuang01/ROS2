#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
線條跟隨機器人控制程式
=====================================

技術背景知識：

1. 線條跟隨原理：
   - 基於機器視覺的路徑追蹤技術
   - 透過顏色識別偵測指定線條
   - 計算線條中心與機器人中心的偏差
   - 使用 PID 控制器修正機器人方向

2. PID 控制理論：
   - P (比例): 與誤差成正比的控制量
   - I (積分): 消除穩態誤差
   - D (微分): 減少超調，提高穩定性
   
3. 影像處理技術：
   - 重心計算：cv2.moments() 計算影像的幾何中心
   - 興趣區域 (ROI)：只處理影像下方區域提高效率
   - 形態學運算：去除雜訊，平滑輪廓

4. 顏色空間選擇：
   - HSV 較 RGB 更適合顏色分離
   - 對光照變化具有較好的魯棒性
   
5. ROS2 機器人控制：
   - Twist 訊息：包含線性和角速度控制
   - cmd_vel 主題：標準的機器人速度控制介面

程式功能：
實現多顏色線條跟隨功能，機器人可自動沿著指定顏色的線條移動，
適用於倉儲自動導引車 (AGV) 或教育機器人等應用。
"""

import cv2                         # OpenCV 影像處理函式庫
import numpy as np                 # 數值計算函式庫，用於陣列運算
import rclpy                       # ROS2 Python 客戶端函式庫
import time                        # 時間相關函式，用於計算 dt
import numpy                       # 重複引入，可考慮移除
from rclpy.node import Node        # ROS2 節點基底類別
from sensor_msgs.msg import Image  # ROS2 影像訊息類型
from rclpy.qos import QoSProfile   # ROS2 服務品質設定
import cv_bridge                   # ROS2 和 OpenCV 之間的轉換橋樑
from geometry_msgs.msg import Twist # ROS2 速度控制訊息類型

# PID 控制相關全域變數
last_erro=0  # 記錄上一次的誤差值，用於計算微分項 (D 控制)
tmp_cv = 0   # 介面控制旗標，確保 OpenCV 視窗只建立一次，避免重複建立視窗

def nothing(s):
    """
    OpenCV 滑動條空回調函數
    
    參數：
        s: 滑動條數值 (在此處不使用)
        
    說明：OpenCV 的 createTrackbar 函數要求提供回調函數，
         但我們採用主動讀取滑動條數值的方式，因此使用空函數
    技術細節：這是 OpenCV GUI 程式設計的常見模式
    """
    pass

# 預定義顏色 HSV 範圍 - 針對常見線條顏色進行優化
# 格式：(H最小值, S最小值, V最小值, H最大值, S最大值, V最大值)
# 注意：OpenCV 中 H 值範圍為 0-179 (而非標準的 0-359)
# 技術要點：HSV 色彩空間對光照變化較為穩定，適合機器視覺應用
col_black = (0,0,0,180,255,46)      # 黑色範圍：低飽和度、低明度，適合黑色膠帶線條
col_red = (0,100,80,10,255,255)     # 紅色範圍：H值接近0或180，注意紅色橫跨HSV環形  
col_blue = (100,43,46,124,255,255)  # 藍色範圍：H值約100-124，穩定的藍色識別
col_green= (35,43,46,77,255,255)    # 綠色範圍：H值約35-77，涵蓋大部分綠色變化
col_yellow = (26,43,46,34,255,255)  # 黃色範圍：H值約26-34，明亮的黃色線條

# 顏色選擇介面說明文字 - 提供使用者選擇線條顏色的參考
Switch = '0:Red\n1:Green\n2:Blue\n3:Yellow\n4:Black'


class Follower(Node):
    """
    線條跟隨節點類別
    
    主要功能：
    1. 接收相機影像串流 - 透過 ROS2 image 主題
    2. 進行顏色篩選和線條偵測 - 使用 HSV 顏色空間濾波 
    3. 計算控制誤差 - 基於線條中心與影像中心的偏差
    4. 發送機器人運動指令 - 透過 cmd_vel 主題控制機器人
    
    控制策略：
    - 使用簡化 PD 控制器 (比例 + 微分控制)
    - 固定前進速度，動態調整轉向角速度
    - 基於線條中心偏移量計算轉向修正量
    - 當偵測不到線條時採用安全停止策略
    
    適用場景：
    - 室內導航線跟隨 (如醫院、工廠導引線)
    - 競賽機器人路徑追蹤 (如RoboCup救援線跟隨)
    - 自動倉儲系統導引 (AGV 自動導引車)
    - 教育機器人視覺控制教學
    """
    def __init__(self):
        """
        節點初始化設定
        
        功能說明：
        1. 建立 ROS2 節點和相關發布者/訂閱者
        2. 初始化 OpenCV 橋接器和控制參數
        3. 設定影像處理和控制相關變數
        4. 建立用戶介面視窗和參數調節介面
        """
        
        建立項目：
        1. ROS2 節點基礎架構
        2. 影像訂閱器 (接收相機資料)
        3. 速度指令發布器 (控制機器人移動)
        4. OpenCV 橋接器 (處理影像格式轉換)
        5. 控制參數初始化
        """
        super().__init__('follower')  # 建立名為 'follower' 的 ROS2 節點
        
        # OpenCV 橋接器：負責 ROS Image 與 OpenCV 格式間的轉換
        self.bridge = cv_bridge.CvBridge()
        
        # QoS 設定：優化影像傳輸效能
        qos = QoSProfile(depth=10)  # 佇列深度 10，平衡延遲與可靠性
        
        # 影像資料緩衝區
        self.mat = None
        
        # 影像訂閱器設定
        # 主題：'/camera/color/image_raw' - 標準 RGB 相機影像
        # 回調：self.image_callback - 每收到新影像時處理
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            qos)
        
        # 機器人速度控制發布器
        # 主題：'cmd_vel' - 標準機器人速度控制介面
        # 訊息類型：Twist - 包含線性速度 (linear) 和角速度 (angular)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        
        # 速度指令訊息物件
        self.twist = Twist()
        
        # GUI 控制旗標：避免重複建立使用者介面
        self.tmp = 0

    def image_callback(self, msg):
        """
        影像處理回調函數 - 線條跟隨的核心演算法
        
        參數：
            msg (sensor_msgs/Image): ROS 影像訊息
            
        處理流程：
        1. 建立顏色選擇介面 (首次執行)
        2. 影像前處理：格式轉換、形態學運算
        3. 顏色範圍篩選：根據使用者選擇的顏色
        4. 線條偵測：計算線條重心位置
        5. 誤差計算：線條中心與影像中心的偏差
        6. PD 控制：計算並發送速度指令
        
        技術細節：
        - 使用影像下方 30 像素作為檢測區域
        - 透過重心計算確定線條中心
        - 採用簡化 PD 控制器進行路徑修正
        """
        # 存取全域 PID 控制變數
        global last_erro  # 上次誤差 (用於微分控制)
        global tmp_cv      # 介面控制旗標
        
        # 首次執行：建立顏色選擇介面
        if self.tmp==0:
            # 建立可調整大小的 OpenCV 視窗
            cv2.namedWindow('Adjust_hsv',cv2.WINDOW_NORMAL)
            
            # 建立顏色選擇滑動條
            # 範圍 0-4 對應不同顏色：紅、綠、藍、黃、黑
            cv2.createTrackbar(Switch,'Adjust_hsv',0,4,nothing)
            
            self.tmp=1  # 標記介面已建立
            
        # 影像格式轉換：ROS Image -> OpenCV BGR
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # hsv将RGB图像分解成色调H，饱和度S，明度V
        # 色彩空間轉換：BGR -> HSV
        # HSV 優勢：
        # 1. H (色調) 對光照變化不敏感
        # 2. 更容易設定顏色範圍閾值
        # 3. 符合人類對顏色的直覺認知
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 颜色的范围        # 第二个参数：lower指的是图像中低于这个lower的值，图像值变为0
        # 第三个参数：upper指的是图像中高于这个upper的值，图像值变为0
        # 而在lower～upper之间的值变成255
        
        # 形態學前處理：改善影像品質
        kernel = numpy.ones((5,5),numpy.uint8)  # 5x5 結構元素
        
        # 侵蝕運算：去除小雜訊點
        # 原理：將結構元素在影像上滑動，只有完全被包含的區域才保留
        hsv_erode = cv2.erode(hsv,kernel,iterations=1)
        
        # 膨脹運算：恢復物體大小，填補小孔洞
        # 先侵蝕後膨脹 = 開運算，有效去除雜訊同時保持主要特徵
        hsv_dilate = cv2.dilate(hsv_erode,kernel,iterations=1)
        
        # 讀取使用者選擇的顏色類型
        m=cv2.getTrackbarPos(Switch,'Adjust_hsv')
        
        # 根據選擇設定對應的 HSV 範圍參數
        if m == 0:      # 紅色線條
            lowerbH=col_red[0]    # H 下限
            lowerbS=col_red[1]    # S 下限  
            lowerbV=col_red[2]    # V 下限
            upperbH=col_red[3]    # H 上限
            upperbS=col_red[4]    # S 上限
            upperbV=col_red[5]    # V 上限
        elif m == 1:    # 綠色線條
            lowerbH=col_green[0]
            lowerbS=col_green[1]
            lowerbV=col_green[2]
            upperbH=col_green[3]
            upperbS=col_green[4]
            upperbV=col_green[5]
        elif m == 2:    # 藍色線條
            lowerbH=col_blue[0]
            lowerbS=col_blue[1]
            lowerbV=col_blue[2]
            upperbH=col_blue[3]
            upperbS=col_blue[4]
            upperbV=col_blue[5]
        elif m == 3:    # 黃色線條
            lowerbH=col_yellow[0]
            lowerbS=col_yellow[1]
            lowerbV=col_yellow[2]
            upperbH=col_yellow[3]
            upperbS=col_yellow[4]
            upperbV=col_yellow[5]
        elif m == 4:    # 黑色線條
            lowerbH=col_black[0]
            lowerbS=col_black[1]
            lowerbV=col_black[2]
            upperbH=col_black[3]
            upperbS=col_black[4]
            upperbV=col_black[5]
        else:           # 預設：顯示所有顏色
            lowerbH=0
            lowerbS=0
            lowerbV=0
            upperbH=255
            upperbS=255
            upperbV=255

        # 顏色範圍篩選：建立二值化遮罩
        # 在指定 HSV 範圍內的像素 = 255 (白色)
        # 範圍外的像素 = 0 (黑色)
        mask=cv2.inRange(hsv_dilate,(lowerbH,lowerbS,lowerbV),(upperbH,upperbS,upperbV))
        
        # 將原始影像與遮罩結合，只保留目標顏色區域
        masked = cv2.bitwise_and(image, image, mask=mask)
        
        # 在图像某处绘制一个指示，因为只考虑20行宽的图像，所以使用numpy切片将以外的空间区域清空
        # 定義興趣區域 (ROI)：影像下方 30 像素
        # 原理：地面線條通常出現在相機視野下方
        h, w, d = image.shape      # 取得影像尺寸 (高度, 寬度, 通道數)
        search_top = h-30          # ROI 上邊界：距離底部 30 像素
        search_bot = h             # ROI 下邊界：影像底部
        
        # 遮蔽 ROI 以外的區域 (設為黑色)
        # 這樣可以避免背景干擾，只專注於地面線條
        mask[0:search_top, 0:w] = 0      # 清除上方區域
        mask[search_bot:h, 0:w] = 0      # 清除下方區域 (實際上 search_bot=h，此行無效果)
        
        # 计算mask图像的重心，即几何中心
        # 計算遮罩影像的影像矩 (Image Moments)
        # 用於求得線條的重心座標
        M = cv2.moments(mask)
        
        # 檢查是否偵測到線條 (m00 是零階矩，代表白色像素總數)
        if M['m00'] > 0:
            # 計算重心座標
            # m10/m00 = X 軸重心, m01/m00 = Y 軸重心
            cx = int(M['m10']/M['m00'])  # 線條中心 X 座標
            cy = int(M['m01']/M['m00'])  # 線條中心 Y 座標
            
            # 可選的視覺化標記 (已註解)
            #cv2.circle(image, (cx, cy), 10, (255, 0, 255), -1)         # 線條中心點
            #cv2.circle(image, (cx-60, cy), 10, (0, 0, 255), -1)        # 偏移參考點  
            #cv2.circle(image, (w/2, h), 10, (0, 255, 255), -1)         # 影像中心點
            
            if cv2.circle:  # 恆為真的條件檢查 (語法習慣)
            # 计算图像中心线和目标指示线中心的距离
                # PD 控制器計算
                # 誤差 = 線條中心 X 座標 - (影像中心 + 偏移補償)
                # 偏移 -60：可能是相機安裝位置的補償參數
                erro = cx - w/2-60
                
                # 計算誤差變化量 (微分項)
                # 用於預測趨勢，減少振盪
                d_erro=erro-last_erro
                
                # 設定機器人前進速度 (固定值)
                # 0.11 m/s：適中的速度，確保穩定跟隨
                self.twist.linear.x = 0.11
                
                # 根據誤差方向計算轉向控制
                if erro<0:      # 線條在左側，需要左轉 (負角速度)
                    # PD 控制公式：輸出 = Kp * 誤差 + Kd * 誤差變化率
                    # Kp = 0.0011 (比例增益), Kd = 0.0000 (微分增益，實際未使用)
                    self.twist.angular.z = -(float(erro)*0.0011-float(d_erro)*0.0000) #top_akm_bs
                elif erro>0:    # 線條在右側，需要右轉 (正角速度)
                    self.twist.angular.z = -(float(erro)*0.0011-float(d_erro)*0.0000) #top_akm_bs
                else :          # 線條在中央，直線前進
                    self.twist.angular.z = 0.0
                    
                # 更新誤差歷史，供下次計算微分項使用
                last_erro=erro
        else:
            # 未偵測到線條：停止機器人
            # 安全機制：避免機器人在失去目標時繼續移動
            self.twist.linear.x = 0.0   # 停止前進
            self.twist.angular.z = 0.0  # 停止轉向
            
        # 發布速度控制指令到 ROS 系統
        # 機器人底盤控制器會接收此指令並執行對應動作
        self.cmd_vel_pub.publish(self.twist)
        
        # 顯示處理結果影像
        cv2.imshow("Adjust_hsv", mask)  # 顯示二值化遮罩
        cv2.waitKey(3)                  # 處理視窗事件，3ms 超時
        
        #cv2.imshow("Adjust_hsv", mask)
        #print('start windows')
        #cv2.waitKey(3)

def main(args=None):
    """
    主程式執行函數
    
    參數：
        args: 命令列參數 (可選)
        
    功能說明：
    1. 初始化 ROS2 通訊系統
    2. 建立線條跟隨節點
    3. 進入主控制迴圈
    4. 處理程式退出與資源清理
    
    執行特點：
    - 使用 spin_once() 非阻塞模式
    - 配合 sleep() 控制更新頻率為 10Hz
    - 平衡即時性與系統資源使用
    
    適用環境：
    - 需要連接支援的相機設備
    - 機器人需支援 cmd_vel 速度控制介面
    - 建議在良好光照條件下使用
    """
    # 初始化 ROS2 分散式通訊系統
    rclpy.init(args=args)
    
    # 啟動提示訊息
    print('start patrolling')
    
    # 建立線條跟隨節點實例
    follower = Follower()
    
    # 主控制迴圈：持續處理 ROS 訊息
    while rclpy.ok():  # 檢查 ROS2 系統狀態
        # 處理一次節點回調函數
        # 包括影像接收、處理、控制指令發送
        rclpy.spin_once(follower)
        
        # 控制迴圈頻率：10Hz
        # 平衡控制即時性與計算負載
        time.sleep(0.1)

    # 程式結束時的清理作業
    follower.destroy_node()  # 銷毀節點，釋放 ROS2 資源
    rclpy.shutdown()        # 正常關閉 ROS2 系統

if __name__ == '__main__':
    """
    Python 模組直接執行進入點
    
    執行方式：
    1. 直接執行：python3 line_follow.py
    2. ROS2 指令：ros2 run simple_follower_ros2 line_follow
    
    使用前準備：
    1. 確保相機節點已啟動
    2. 檢查影像主題名稱是否正確
    3. 確認機器人支援 cmd_vel 控制
    4. 準備清晰可見的彩色線條作為導引路徑
    
    調試建議：
    1. 先使用 adjust_hsv.py 調整顏色參數
    2. 根據實際環境調整 HSV範圍
    3. 可調整 PID 參數以獲得更好的跟隨效果
    4. 注意光照條件對顏色識別的影響
    """
    main()
