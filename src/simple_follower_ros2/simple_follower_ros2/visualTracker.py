#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
視覺物體追蹤器程式
=====================================

技術背景知識：

1. RGB-D 相機原理：
   - RGB 感測器：提供彩色影像資訊
   - Depth 感測器：透過紅外線結構光或 ToF 技術測距
   - 資料融合：結合顏色和深度資訊進行 3D 物體偵測
   - 常見設備：Intel RealSense, Kinect, Orbbec Astra

2. 電腦視覺技術：
   - HSV 色彩空間：更適合顏色分離和追蹤
   - 輪廓偵測：cv2.findContours() 找出物體邊界
   - 最小外接矩形：cv2.minAreaRect() 計算物體方向和中心
   - 形態學運算：去除雜訊，改善輪廓品質

3. 3D 座標轉換：
   - 像素座標 -> 世界座標
   - 相機內參：焦距、主點座標
   - 深度資訊：Z 軸距離
   - 角度計算：arctan(位移/距離)

4. 訊息同步技術：
   - ApproximateTimeSynchronizer：同步不同頻率的感測器資料
   - 時間戳匹配：確保 RGB 和深度資料對應同一時刻
   - 佇列管理：處理感測器延遲和頻率差異

5. 物體追蹤策略：
   - 顏色篩選：HSV 範圍過濾目標物體
   - 輪廓分析：計算面積、中心、邊界框
   - 合理性檢查：排除不符合預期的偵測結果
   - 連續性追蹤：比較前後幀確保追蹤穩定性

程式功能：
使用 RGB-D 相機進行基於顏色的物體追蹤，
結合彩色影像和深度資訊計算目標的 3D 位置，
適用於室內機器人導航和人機互動應用。

適用場景：
- 特定顏色物體跟隨
- 室內導航標誌偵測  
- 人員追蹤 (穿著特定顏色衣物)
- 互動機器人應用
"""

from __future__ import division
import rclpy  # ROS2 Python 客戶端庫
import message_filters  # ROS2 訊息同步工具
from rclpy.node import Node
import numpy as np  # 數值計算庫
import cv2  # OpenCV 電腦視覺庫
from matplotlib import pyplot as plt  # 除錯視覺化工具
import cv_bridge  # ROS-OpenCV 橋接器

from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

from turn_on_wheeltec_robot.msg import Position as PositionMsg
from std_msgs.msg import String as StringMsg

# numpy 錯誤處理設定：遇到異常時拋出例外
# 有助於除錯數值計算問題
np.seterr(all='raise')  

# 除錯模式開關：設為 True 時顯示處理過程影像
displayImage=False

# 關閉所有 matplotlib 視窗：避免記憶體洩漏
plt.close('all')

class VisualTracker(Node):
    """
    視覺追蹤器節點類別
    
    主要功能：
    1. 同步接收 RGB 和深度影像
    2. 基於 HSV 顏色範圍進行物體偵測
    3. 結合深度資訊計算 3D 位置
    4. 發布目標位置給控制器
    
    技術特點：
    - 雙感測器融合：RGB + Depth
    - 即時處理：低延遲目標偵測
    - 3D 定位：提供完整的空間位置資訊
    - 魯棒性：多重篩選確保追蹤穩定性
    
    追蹤流程：
    1. 訊息同步：確保 RGB 和深度資料時間對應
    2. 顏色篩選：HSV 範圍過濾目標顏色
    3. 輪廓偵測：找出候選物體區域
    4. 深度分析：計算物體距離
    5. 位置計算：轉換為角度和距離資訊
    """
    def __init__(self):
        """
        視覺追蹤節點初始化
        
        設定項目：
        1. 相機參數設定
        2. 目標顏色範圍定義
        3. 訊息同步器建立
        4. 發布器設定
        5. 座標轉換參數計算
        """
        super().__init__('visualtracker')  # 建立名為 'visualtracker' 的 ROS2 節點
        
        # QoS 設定：優化影像傳輸效能
        qos = QoSProfile(depth=10)
        
        # OpenCV 橋接器：處理 ROS 影像與 OpenCV 格式轉換
        self.bridge = cv_bridge.CvBridge()
        
        # 目標顏色 HSV 範圍設定 (可通過參數檔案調整)
        #self.tmp_list = self.get_parameter('~targetred/upper').value
        # HSV 上限：針對紅色物體優化
        self.targetUpper = np.array([0, 50, 50])    # (H最小, S最小, V最小)
        # HSV 下限：
        self.targetLower = np.array([180, 255, 255]) # (H最大, S最大, V最大)
        
        # 相機參數設定 (可通過參數檔案調整)
        #self.pictureHeight= self.get_parameter('~pictureDimensions/pictureHeight')
        #self.pictureWidth = self.get_parameter('~pictureDimensions/pictureWidth')
        #vertAngle =self.get_parameter('~pictureDimensions/verticalAngle')
        #horizontalAngle =  self.get_parameter('~pictureDimensions/horizontalAngle')
        
        # RealSense D435 相機標準解析度
        self.pictureHeight= 480   # 影像高度 (像素)
        self.pictureWidth = 640   # 影像寬度 (像素)
        
        # 相機視野角度 (弧度)
        vertAngle = 0.43196898986859655      # 垂直視野角：約 24.7 度
        horizontalAngle = 0.5235987755982988  # 水平視野角：約 30 度
        
        # precompute tangens since thats all we need anyways:
        # 預計算正切值：用於座標轉換，提高計算效率
        # 角度計算公式：angle = arctan(像素位移 * tan(視野角/2))
        self.tanVertical = np.tan(vertAngle)      # 垂直方向正切值
        self.tanHorizontal = np.tan(horizontalAngle) # 水平方向正切值
        
        # 追蹤歷史：儲存上次偵測位置用於連續性檢查
        self.lastPoCsition = None
        
        # one callback that deals with depth and rgb at the same time
        # 建立訊息同步器：同時處理 RGB 和深度影像
        
        # RGB 影像訂閱器
        im_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        
        # 深度影像訂閱器：使用感測器專用 QoS 設定
        dep_sub = message_filters.Subscriber(self,Image,'/camera/depth/image_raw', qos_profile=qos_profile_sensor_data)
        
        # 同步器設定
        queue_size = 30  # 佇列大小：平衡延遲與同步成功率
 
        # 近似時間同步器：允許小幅時間差異
        # 參數：[訂閱器列表], 佇列大小, 最大時間差異(秒)
        self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, dep_sub],queue_size,0.05)
        
        # 註冊同步回調函數：當 RGB 和深度影像時間戳匹配時觸發
        self.timeSynchronizer.registerCallback(self.trackObject)
        
        # 位置資訊發布器：將追蹤結果傳送給控制器
        self.positionPublisher = self.create_publisher( PositionMsg,'/object_tracker/current_position', qos)
        
        # 位置訊息物件：重複使用以提高效率
        self.posMsg=PositionMsg()

    def trackObject(self, image_data, depth_data):
        """
        物體追蹤核心函數：同步處理 RGB 和深度資料
        
        參數：
            image_data (sensor_msgs/Image): RGB 彩色影像
            depth_data (sensor_msgs/Image): 深度影像
            
        處理流程：
        1. 影像格式驗證與轉換
        2. HSV 顏色空間轉換
        3. 顏色範圍篩選
        4. 形態學處理
        5. 輪廓偵測與分析
        6. 深度資訊提取
        7. 3D 位置計算
        8. 結果發布
        
        異常處理：
        - 影像格式檢查
        - 尺寸驗證
        - 數值異常處理
        """
        # 影像格式驗證：確保為標準 RGB8 格式
        if(image_data.encoding != 'rgb8'):
            raise ValueError('image is not rgb8 as expected')
            
        #convert both images to numpy arrays
        # 影像格式轉換：ROS Image -> OpenCV numpy 陣列
        frame = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='rgb8')
        # 深度影像轉換：保持原始數值 (16位元深度資料)
        depthFrame = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')#"32FC1")
        
        # 影像尺寸驗證：確保與設定參數一致
        if(np.shape(frame)[0:2] != (self.pictureHeight, self.pictureWidth)):
            raise ValueError('image does not have the right shape. shape(frame): {}, shape parameters:{}'.format(np.shape(frame)[0:2], (self.pictureHeight, self.pictureWidth)))
            
        # blure a little and convert to HSV color space
        # 可選的高斯模糊：減少雜訊 (目前已註解)
        #blurred = cv2.GaussianBlur(frame, (11,11), 0)
        
        # 色彩空間轉換：RGB -> HSV
        # HSV 對光照變化較不敏感，更適合顏色追蹤
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        # select all the pixels that are in the range specified by the target
        # 顏色範圍篩選：建立二值遮罩
        # 在目標 HSV 範圍內的像素設為 255，其餘設為 0
        mask = cv2.inRange(hsv, self.targetLower, self.targetUpper)
        
        # 形態學運算：改善遮罩品質
        # 定義結構元素：3x3 矩形核心
        kernel = np.ones((3,3),np.uint8)
        
        # 侵蝕運算：去除小雜訊點和細線
        # 原理：只有當結構元素完全在白色區域內時才保留該像素
        erosion = cv2.erode(mask,kernel,iterations = 1)
        
        # 膨脹運算：恢復被侵蝕的物體邊界
        # 先侵蝕後膨脹 = 開運算，有效去除雜訊同時保持主要物體
        dilation = cv2.dilate(erosion,kernel,iterations = 1)
        
        # 輪廓偵測：找出所有封閉區域的邊界
        # RETR_EXTERNAL：只偵測外部輪廓，忽略內部孔洞
        # CHAIN_APPROX_SIMPLE：壓縮輪廓，只保留端點
        contours, hierarchy = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 輪廓篩选與分析：找出最可能的目標物體
        largestArea = 0     # 記錄最大輪廓面積
        largestContour = [] # 儲存最大輪廓
        
        # 遍歷所有偵測到的輪廓
        for contour in contours:
            
            # 計算輪廓面積：面積太小的可能是雜訊
            area = cv2.contourArea(contour)
            
            # 面積閾值篩選：排除過小的區域
            # 最小面積 400 像素²，約為 20x20 像素的正方形
            if area > 400:
                
                # 更新最大輪廓記錄
                if area > largestArea:
                    largestArea = area
                    largestContour = contour
        
        # 檢查是否找到有效輪廓
        if largestArea > 0:
            
            # 輪廓分析：提取目標物體的位置和幾何資訊
            position = self.analyseContour(largestContour, depthFrame)
            
            # 位置合理性檢查：排除異常偵測結果
            if self.checkPosPlausible(position):
                
                # 發布有效的位置資訊
                self.publishPosition(position)
                
                # 更新追蹤歷史：用於下次連續性檢查
                self.lastPoCsition = position
        
        # 除錯視覺化：顯示處理結果 (可選)
        if displayImage:
            
            # 在原始影像上繪製偵測結果
            if largestArea > 0:
                # 繪製輪廓邊界：綠色線條，厚度 2
                cv2.drawContours(frame, [largestContour], -1, (0, 255, 0), 2)
                
                # 計算並標記物體中心
                M = cv2.moments(largestContour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])  # 重心 X 座標
                    cy = int(M['m01'] / M['m00'])  # 重心 Y 座標
                    
                    # 繪製中心點：紅色圓點，半徑 5
                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            
            # 顯示處理過程影像
            cv2.imshow('Original Frame', frame)      # 原始影像 + 偵測結果
            cv2.imshow('HSV Mask', mask)             # HSV 顏色遮罩
            cv2.imshow('After Morphology', dilation) # 形態學處理後結果
            cv2.waitKey(1)  # 更新視窗顯示

    def publishPosition(self, pos):
        """
        發布目標位置資訊
        
        參數：
            pos (tuple): 包含 (x, y, z, angleX, angleY, distance) 的位置資料
            
        功能說明：
        將計算出的 3D 位置和角度資訊封裝為 ROS 訊息並發布，
        供機器人控制器使用。
        
        訊息格式：
        - angle_x: 水平角度偏差 (弧度)
        - angle_y: 垂直角度偏差 (弧度) 
        - distance: 目標距離 (毫米)
        - x, y, z: 3D 座標位置 (毫米)
        """
        # 提取位置分量
        [x, y, z, angleX, angleY, distance] = pos
        
        # 封裝位置訊息
        self.posMsg.angle_x = angleX    # 水平角度：用於機器人轉向控制
        self.posMsg.angle_y = angleY    # 垂直角度：可用於雲台控制
        self.posMsg.distance = distance # 距離：用於前進/後退控制
        
        # 3D 座標：提供完整的空間位置資訊
        self.posMsg.x = x  # 左右位移 (相機座標系)
        self.posMsg.y = y  # 上下位移 (相機座標系)  
        self.posMsg.z = z  # 前後距離 (相機座標系)
        
        # 發布位置訊息到 ROS 主題
        self.positionPublisher.publish(self.posMsg)

    def checkPosPlausible(self, pos):
        """
        檢查位置資料的合理性
        
        參數：
            pos (tuple): 包含位置資料的元組
            
        回傳：
            bool: True 表示位置合理，False 表示可能是錯誤偵測
            
        檢查項目：
        1. 距離範圍：0.3m - 5m 之間
        2. 角度範圍：水平 ±30度，垂直 ±20度
        3. 連續性：與上次位置的變化是否過大
        4. 深度有效性：是否為有效的深度值
        
        合理性判斷原則：
        - 室內環境下的常見物體距離範圍
        - 相機視野角度限制
        - 物體移動的物理約束 (連續性)
        - 感測器測量範圍限制
        """
        [x, y, z, angleX, angleY, distance] = pos
        
        # 距離範圍檢查：300mm - 5000mm
        # 太近可能是雜訊，太遠可能是錯誤偵測
        if distance < 300 or distance > 5000:
            return False
            
        # 角度範圍檢查：確保目標在相機視野內
        max_horizontal_angle = 0.52  # 約 30 度
        max_vertical_angle = 0.35    # 約 20 度
        
        if abs(angleX) > max_horizontal_angle or abs(angleY) > max_vertical_angle:
            return False
            
        # 深度值有效性檢查：排除無效的深度測量
        if z <= 0 or np.isnan(z) or np.isinf(z):
            return False
            
        # 連續性檢查：如果有歷史位置記錄
        if self.lastPoCsition is not None:
            [last_x, last_y, last_z, last_angleX, last_angleY, last_distance] = self.lastPoCsition
            
            # 計算位置變化量
            distance_change = abs(distance - last_distance)
            angle_change_x = abs(angleX - last_angleX)
            angle_change_y = abs(angleY - last_angleY)
            
            # 設定合理的變化閾值 (根據系統更新頻率調整)
            max_distance_change = 1000  # 每幀最大距離變化 1m
            max_angle_change = 0.5      # 每幀最大角度變化 0.5 弧度
            
            # 如果變化過大，可能是錯誤偵測
            if (distance_change > max_distance_change or 
                angle_change_x > max_angle_change or 
                angle_change_y > max_angle_change):
                return False
        
        return True

    def calculateAngleX(self, pos):
        """
        計算水平角度偏差
        
        參數：
            pos (tuple): 像素座標 (x, y)
            
        回傳：
            float: 水平角度偏差 (弧度)
            
        計算原理：
        1. 將像素座標轉換為相對於影像中心的偏移
        2. 使用相機視野角和影像尺寸計算角度
        3. 公式：angle = arctan((pixel_offset / image_width) * tan(fov/2))
        
        座標系統：
        - 影像中心為原點 (0, 0)
        - 左側為負角度，右側為正角度
        - 角度範圍：約 ±30度 (取決於相機視野角)
        
        應用：
        用於機器人的轉向控制，正值表示目標在右側，
        負值表示目標在左側，需要相應調整機器人方向。
        """
        (x, y) = pos
        
        # 計算相對於影像中心的水平偏移
        # 影像中心 X 座標 = 寬度 / 2
        center_x = self.pictureWidth / 2
        pixel_offset_x = x - center_x
        
        # 歸一化偏移：轉換為 [-1, 1] 範圍
        normalized_offset_x = pixel_offset_x / (self.pictureWidth / 2)
        
        # 角度計算：使用預計算的正切值
        # arctan(normalized_offset * tan(fov/2))
        angle_x = np.arctan(normalized_offset_x * self.tanHorizontal)
        
        return angle_x

    def calculateAngleY(self, pos):
        """
        計算垂直角度偏差
        
        參數：
            pos (tuple): 像素座標 (x, y)
            
        回傳：
            float: 垂直角度偏差 (弧度)
            
        計算原理：
        與水平角度類似，但針對垂直方向的偏移。
        
        座標系統：
        - 影像中心為原點
        - 上方為負角度，下方為正角度
        - 角度範圍：約 ±20度
        
        應用：
        可用於雲台的俯仰控制，或者判斷目標的高度位置。
        在地面機器人應用中，通常專注於水平角度控制。
        """
        (x, y) = pos
        
        # 計算相對於影像中心的垂直偏移
        center_y = self.pictureHeight / 2
        pixel_offset_y = y - center_y
        
        # 歸一化偏移
        normalized_offset_y = pixel_offset_y / (self.pictureHeight / 2)
        
        # 角度計算
        angle_y = np.arctan(normalized_offset_y * self.tanVertical)
        
        return angle_y
    
    def analyseContour(self, contour, depthFrame):
        """
        分析輪廓並計算 3D 位置資訊
        
        參數：
            contour: OpenCV 輪廓資料
            depthFrame: 深度影像陣列
            
        回傳：
            tuple: (x, y, z, angleX, angleY, distance) 3D 位置和角度資訊
            
        處理流程：
        1. 計算輪廓的最小外接矩形
        2. 提取矩形中心作為目標位置
        3. 從深度影像中提取對應的距離資訊
        4. 計算 3D 座標和角度偏差
        5. 進行座標系轉換和單位換算
        
        技術細節：
        - 使用 minAreaRect 獲得更準確的物體中心
        - 深度值濾波：去除異常值，提高測量穩定性
        - 座標轉換：像素座標 -> 相機座標 -> 世界座標
        - 單位統一：像素 -> 毫米，像素 -> 弧度
        """
        # 計算最小外接矩形：獲得物體的準確中心和方向
        # minAreaRect 回傳：((中心x, 中心y), (寬度, 高度), 旋轉角度)
        rect = cv2.minAreaRect(contour)
        
        # 提取矩形中心座標
        (center_x, center_y), (width, height), rotation_angle = rect
        
        # 確保座標在影像範圍內
        center_x = int(np.clip(center_x, 0, self.pictureWidth - 1))
        center_y = int(np.clip(center_y, 0, self.pictureHeight - 1))
        
        # 深度資訊提取與處理
        # 從深度影像中取得目標位置的距離值
        raw_depth = depthFrame[center_y, center_x]
        
        # 深度值濾波：使用鄰域平均減少雜訊
        # 定義感興趣區域：目標中心周圍的小區域
        roi_size = 5  # 5x5 像素區域
        y_min = max(0, center_y - roi_size // 2)
        y_max = min(self.pictureHeight, center_y + roi_size // 2 + 1)
        x_min = max(0, center_x - roi_size // 2)
        x_max = min(self.pictureWidth, center_x + roi_size // 2 + 1)
        
        # 提取 ROI 深度值
        depth_roi = depthFrame[y_min:y_max, x_min:x_max]
        
        # 排除無效深度值 (0 或過大值)
        valid_depths = depth_roi[(depth_roi > 0) & (depth_roi < 10000)]
        
        # 如果有足夠的有效深度值，使用中位數濾波
        if len(valid_depths) > 0:
            filtered_depth = np.median(valid_depths)
        else:
            # 回退到原始深度值
            filtered_depth = raw_depth
        
        # 距離單位轉換：從相機單位轉換為毫米
        # RealSense D435 深度單位通常為毫米
        distance = float(filtered_depth)
        
        # 3D 座標計算：像素座標 -> 相機 3D 座標
        # 相機座標系：X軸向右，Y軸向下，Z軸向前
        
        # Z 座標：直接使用深度值
        z = distance
        
        # X 座標：水平位移 (毫米)
        # 公式：X = (pixel_x - center_x) * Z / focal_length
        # 簡化版：使用角度和距離計算
        angle_x = self.calculateAngleX((center_x, center_y))
        x = distance * np.tan(angle_x)
        
        # Y 座標：垂直位移 (毫米)
        angle_y = self.calculateAngleY((center_x, center_y))
        y = distance * np.tan(angle_y)
        
        # 回傳完整的 3D 位置和角度資訊
        # 格式：(x, y, z, angleX, angleY, distance)
        # x, y, z: 3D 座標 (毫米)
        # angleX, angleY: 角度偏差 (弧度)
        # distance: 距離 (毫米)
        return (x, y, z, angle_x, angle_y, distance)
    
def main(args=None):
    """
    主程式執行函數
    
    參數：
        args: 命令列參數 (可選)
        
    功能說明：
    1. 初始化 ROS2 通訊系統
    2. 建立視覺追蹤節點
    3. 進入事件驅動迴圈
    4. 處理程式退出與資源清理
    
    執行特點：
    - 訊息同步：等待 RGB 和深度影像同時到達
    - 事件驅動：使用 rclpy.spin() 處理訊息
    - 自動清理：確保資源正確釋放
    
    硬體需求：
    - RGB-D 相機 (如 Intel RealSense D435)
    - 支援的目標物體 (指定顏色)
    - 適當的光照條件
    """
    print('visual_tracker')  # 啟動提示
    
    # 初始化 ROS2 分散式通訊系統
    rclpy.init(args=args)
    
    # 建立視覺追蹤節點實例
    visualTracker = VisualTracker()
    
    print('visualTracker init done')  # 初始化完成提示
    
    try:
        # 進入 ROS2 主事件迴圈
        # spin() 為阻塞式呼叫，持續處理 ROS 訊息直到程式終止
        # 在此模式下，所有的回調函數都會在單一執行緒中執行
        rclpy.spin(visualTracker)
        
    finally:
        # 程式結束時的清理作業
        # 無論是正常結束或異常終止都會執行此區塊
        
        # 銷毀節點：釋放 ROS2 相關資源
        # 包括發布器、訂閱器、服務等
        visualTracker.destroy_node()
        
        # 正常關閉 ROS2 系統
        # 確保所有背景程序正確終止
        rclpy.shutdown()
        
        # 關閉 OpenCV 視窗：避免殘留視窗
        cv2.destroyAllWindows()

if __name__ == '__main__':
    """
    Python 模組直接執行進入點
    
    執行方式：
    1. 直接執行：python3 visualTracker.py  
    2. ROS2 指令：ros2 run simple_follower_ros2 visualTracker
    
    系統設定檢查：
    1. 相機驅動狀態：
       $ ros2 topic list | grep camera
       應看到：/camera/color/image_raw, /camera/depth/image_raw
    
    2. 影像品質檢查：
       $ ros2 run rqt_image_view rqt_image_view
       選擇對應主題檢視影像
    
    3. 訊息頻率檢查：
       $ ros2 topic hz /camera/color/image_raw
       $ ros2 topic hz /camera/depth/image_raw
    
    參數調整建議：
    1. 目標顏色範圍：根據實際物體調整 HSV 範圍
    2. 相機參數：確認解析度和視野角設定正確
    3. 形態學參數：根據環境雜訊調整侵蝕次數
    4. 距離範圍：根據應用場景調整有效距離範圍
    
    常見問題排除：
    1. 同步失敗：檢查兩個影像主題的發布頻率
    2. 偵測不穩定：調整 HSV 範圍和形態學參數
    3. 距離異常：檢查深度相機標定和環境反射
    4. 效能問題：降低影像解析度或處理頻率
    
    除錯模式：
    設定 displayImage = True 可檢視處理過程，
    但會顯著降低執行效能，僅建議開發階段使用。
    """
    main()



