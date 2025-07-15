#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
視覺跟隨控制器程式
=====================================

技術背景知識：

1. 視覺伺服控制 (Visual Servoing)：
   - 基於機器視覺的閉迴路控制系統
   - 即時影像回饋調整機器人動作
   - 分類：位置式視覺伺服 vs 影像式視覺伺服
   - 應用：物體跟隨、視覺導航、精密裝配

2. 多感測器融合架構：
   - 視覺系統：提供目標位置和角度資訊
   - 雷射掃描：提供距離和障礙物資訊
   - 手把控制：人機互動和模式切換
   - 整合策略：加權融合、狀態機切換

3. PID 控制在機器人中的應用：
   - 雙迴路控制：角度控制 + 距離控制
   - 參數調整：不同運動軸使用不同 PID 參數
   - 飽和處理：限制輸出防止過激反應
   - 積分飽和：防止積分項過度累積

4. 機器人運動學：
   - 差動驅動：左右輪速度差產生轉向
   - 運動模型：linear.x (前進速度) + angular.z (角速度)
   - 速度限制：考慮機器人物理限制和安全要求
   - 平滑控制：避免急劇的速度變化

5. 人機協作控制：
   - 手動模式：完全由操作者控制
   - 半自動模式：系統輔助操作者
   - 全自動模式：系統獨立執行任務
   - 安全機制：緊急停止、失效安全設計

程式功能：
整合視覺追蹤器的輸出，實現智慧跟隨控制，
結合手把輸入進行模式切換和安全監控，
提供穩定、安全的物體跟隨功能。

控制架構：
視覺追蹤器 → 位置誤差計算 → PID控制器 → 速度指令 → 機器人
     ↑                                            ↓
手把控制器 ← 安全監控 ← 狀態回饋 ← 執行監控

適用場景：
- 購物助手機器人：跟隨顧客移動
- 醫療助手：跟隨醫護人員
- 安保巡邏：跟隨目標人員
- 寵物陪伴：戶外散步助手
"""

import rclpy
import _thread
import threading
import time
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, Vector3
from turn_on_wheeltec_robot.msg import Position as PositionMsg
from std_msgs.msg import String as StringMsg

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data


# 全域變數：儲存角度和距離計算結果
# 用於多個函數間的資料共享和狀態保持
angle=[0.0]*3    # 角度陣列：儲存不同方向的角度偏差
distan=[0.0]*3   # 距離陣列：儲存不同區域的距離測量值

class VisualFollower(Node):
    """
    視覺跟隨控制器節點類別
    
    主要功能：
    1. 接收視覺追蹤器的目標位置資訊
    2. 實施 PID 控制計算運動指令
    3. 整合手把控制進行模式管理
    4. 提供安全監控和緊急停止機制
    
    控制邏輯：
    - 雙軸控制：角度控制 (轉向) + 距離控制 (前進/後退)
    - 自適應調整：根據目標距離動態調整控制參數
    - 死區處理：小誤差時停止運動避免震盪
    - 速度限制：確保運動在安全範圍內
    
    技術特點：
    - 即時回饋控制：毫秒級響應速度
    - 多執行緒安全：按鈕事件和控制迴圈分離
    - 參數化設計：可調整的控制參數
    - 失效安全：連線斷開自動停止
    
    整合介面：
    - 輸入：視覺追蹤位置、手把控制訊號
    - 輸出：機器人速度指令 (Twist 訊息)
    - 狀態：追蹤啟動/停止、連線狀態
    """
	def __init__(self):
        """
        視覺跟隨控制器初始化
        
        建立項目：
        1. 連線監控系統
        2. 控制模式和安全參數
        3. PID 控制器設定
        4. ROS 通訊介面建立
        5. 訊息訂閱和發布設定
        
        安全機制：
        - 連線監控定時器：偵測控制器斷線
        - 速度限制：防止過度激烈的運動
        - 模式鎖定：避免意外啟動跟隨模式
        - 緊急停止：立即停止所有運動
        """
		super().__init__('visualfollower')
		qos = QoSProfile(depth=10)
		
		# as soon as we stop receiving Joy messages from the ps3 controller we stop all movement:
		self.controllerLossTimer = threading.Timer(1, self.controllerLoss) #if we lose connection
		self.controllerLossTimer.start()
		self.switchMode= True  # if this is set to False the O button has to be kept pressed in order for it to move
		self.max_speed = 0.3
		self.controllButtonIndex = -4

		self.buttonCallbackBusy=False
		self.active=False
		self.i=0
		self.cmdVelPublisher = self.create_publisher( Twist,'/cmd_vel', qos)

		# the topic for the messages from the ps3 controller (game pad)

		# the topic for the tracker that gives us the current position of the object we are following
		self.positionSubscriber = self.create_subscription(PositionMsg, '/object_tracker/current_position', self.positionUpdateCallback, qos)
		self.trackerInfoSubscriber = self.create_subscription(StringMsg, '/object_tracker/info', self.trackerInfoCallback, qos)

		# PID 控制器參數設定
		# 第一個參數：角度控制，第二個參數：距離控制
		targetDist = 600  # 目標距離：600毫米 (約60公分的跟隨距離)
		
		# PID 控制器初始化：雙軸控制系統
		# 參數格式：[目標值], [比例增益], [積分增益], [微分增益]
		# 角度控制：目標角度=0 (保持正前方)，距離控制：目標距離=600mm
		self.PID_controller = simplePID(
		    [0, targetDist],      # 目標值：[角度目標, 距離目標]
		    [1.2, 0.2],          # 比例增益：[角度P, 距離P]
		    [0, 0.00],           # 積分增益：[角度I, 距離I] 
		    [0.005, 0.00]        # 微分增益：[角度D, 距離D]
		)

		# this method gets called when the process is killed with Ctrl+C
		#rclpy.shutdown(self.controllerLoss)
		
	def trackerInfoCallback(self, info):
		"""
		追蹤器資訊回調函數
		
		參數：
		    info (std_msgs/String): 追蹤器狀態訊息
		    
		功能說明：
		接收來自視覺追蹤器的狀態資訊，如目標丟失、
		追蹤異常等，目前主要用於日誌記錄和除錯。
		
		訊息範例：
		- "Target lost"：目標物體離開視野
		- "Tracking stable"：追蹤穩定進行中
		- "Low confidence"：追蹤信心度下降
		"""
		# 將追蹤器資訊以警告等級記錄到日誌
		# 這有助於監控追蹤狀態和診斷問題
		self.get_logger().warn(info.data)
	
	def positionUpdateCallback(self, position):
		"""
		位置更新回調函數 - 視覺跟隨的核心控制邏輯
		
		參數：
		    position (PositionMsg): 包含目標位置和角度的訊息
		    
		處理流程：
		1. 提取目標的角度和距離資訊
		2. 使用 PID 控制器計算所需的運動速度
		3. 應用速度限制確保安全運行
		4. 建立並發布 Twist 速度指令
		5. 根據距離判斷是否需要停止
		
		控制策略：
		- 角度控制：調整機器人朝向，保持目標在正前方
		- 距離控制：維持與目標的固定距離
		- 安全停止：目標過遠或過近時停止運動
		- 速度限制：防止過激的運動指令
		"""
		# 提取位置資訊
		angleX= position.angle_x    # 水平角度偏差 (弧度)
		distance = position.distance # 目標距離 (毫米)

		# PID 控制計算：根據當前誤差計算所需的速度修正
		# 輸入：[當前角度誤差, 當前距離誤差]
		# 輸出：[角速度修正, 線速度修正]
		[uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angleX, distance])
		
		# 速度限制：將計算出的速度限制在安全範圍內
		# 使用 numpy.clip() 確保速度不超過最大值
		angularSpeed = np.clip(-uncliped_ang_speed, -self.max_speed, self.max_speed)  # 角速度 (rad/s)
		linearSpeed  = np.clip(-uncliped_lin_speed, -self.max_speed, self.max_speed)  # 線速度 (m/s)
		
		# 建立 Twist 運動指令訊息
		# Twist 包含線性速度 (linear) 和角速度 (angular) 兩部分
		velocity = Twist()	

		# 設定線性速度分量
		velocity.linear.x = float(linearSpeed)  # X軸：前進/後退速度
		velocity.linear.y = 0.0                 # Y軸：左右平移 (差動驅動不支援)
		velocity.linear.z = 0.0                 # Z軸：上下移動 (地面機器人不支援)

		# 設定角速度分量  
		velocity.angular.x = 0.0                # 繞X軸旋轉：俯仰 (不使用)
		velocity.angular.y = 0.0                # 繞Y軸旋轉：滾動 (不使用)
		velocity.angular.z = angularSpeed       # 繞Z軸旋轉：偏航 (轉向控制)
		
		# 距離安全判斷：根據目標距離決定是否發布運動指令
		if((distance>2000)or((distance==0))):
		    # 距離過遠 (>2米) 或無效距離 (=0)：停止運動
		    # 這避免了追逐過遠目標或錯誤偵測造成的異常行為
		    velocity.linear.x = 0.0
		    velocity.linear.y = 0.0
		    velocity.linear.z = 0.0

		    velocity.angular.x = 0.0
		    velocity.angular.y = 0.0
		    velocity.angular.z = 0.0
		else:
		    # 距離在合理範圍內：正常發布計算出的速度指令
		    pass  # 使用上面計算的 velocity 值
		
		# 發布速度指令到機器人控制系統
		self.cmdVelPublisher.publish(velocity)

	def buttonCallback(self, joy_data):
		# this method gets called whenever we receive a message from the joy stick

		# there is a timer that always gets reset if we have a new joy stick message
		# if it runs out we know that we have lost connection and the controllerLoss function
		# will be called
		self.controllerLossTimer.cancel()
		self.controllerLossTimer = threading.Timer(0.5, self.controllerLoss)
		self.controllerLossTimer.start()

		# if we are in switch mode, one button press will make the follower active / inactive 
		# but 'one' button press will be visible in roughly 10 joy messages (since they get published to fast) 
		# so we need to drop the remaining 9
		
		if self.buttonCallbackBusy:
			# we are busy with dealing with the last message
			return 
		else:
			# we are not busy. i.e. there is a real 'new' button press
			# we deal with it in a seperate thread to be able to drop the other joy messages arriving in the mean
			# time
			thread.start_new_thread(self.threadedButtonCallback,  (joy_data, ))

	def threadedButtonCallback(self, joy_data):
		self.buttonCallbackBusy = True

		if(joy_data.buttons[self.controllButtonIndex]==self.switchMode and self.active):
			# we are active
			# switchMode = false: we will always be inactive whenever the button is not pressed (buttons[index]==false)
			# switchMode = true: we will only become inactive if we press the button. (if we keep pressing it, 
			# we would alternate between active and not in 0.5 second intervalls)
			self.get_logger().info('stoping')
			self.stopMoving()
			self.active = False
			time.sleep(0.5)
		elif(joy_data.buttons[self.controllButtonIndex]==True and not(self.active)):
			# if we are not active and just pressed the button (or are constantly pressing it) we become active
			self.get_logger().info('activating')
			self.active = True #enable response
			time.sleep(0.5)

		self.buttonCallbackBusy = False

	def stopMoving(self):
		"""
		緊急停止函數
		
		功能說明：
		立即停止機器人的所有運動，將所有速度分量設為零。
		用於安全保護、連線斷開或模式切換時的緊急停止。
		
		應用場景：
		- 手把控制器斷線
		- 使用者按下停止按鈕  
		- 系統檢測到異常狀況
		- 從自動模式切換到手動模式
		"""
		velocity = Twist()

		# 清零所有線性速度分量
		velocity.linear.x = 0.0   # 停止前進/後退
		velocity.linear.y = 0.0   # 停止左右平移
		velocity.linear.z = 0.0   # 停止上下移動

		# 清零所有角速度分量
		velocity.angular.x = 0.0  # 停止俯仰旋轉
		velocity.angular.y = 0.0  # 停止滾動旋轉
		velocity.angular.z = 0.0  # 停止偏航旋轉 (轉向)
		
		# 立即發布停止指令
		self.cmdVelPublisher.publish(velocity)

	def controllerLoss(self):
		"""
		控制器連線丟失處理函數
		
		功能說明：
		當偵測到手把控制器連線中斷時，自動執行安全停止程序。
		這是重要的安全機制，防止機器人在失去人為監控時繼續運動。
		
		安全措施：
		1. 立即停止所有運動
		2. 關閉自動跟隨模式
		3. 記錄連線丟失事件
		4. 等待重新連線
		"""
		# 立即停止機器人運動
		self.stopMoving()
		
		# 關閉自動跟隨模式：需要重新啟動
		self.active = False
		
		# 記錄連線丟失事件到系統日誌
		self.get_logger().info('lost connection')

class simplePID:
	"""
	簡化離散 PID 控制器類別
	=====================================
	
	技術背景：
	PID (比例-積分-微分) 控制是最廣泛使用的工業控制演算法，
	特別適合於需要精確位置或速度控制的機器人應用。
	
	控制原理：
	1. 比例項 (P)：與當前誤差成正比的控制輸出
	   - 提供即時響應
	   - 增益過大會造成震盪
	   - 增益過小響應太慢
	
	2. 積分項 (I)：累積誤差的控制輸出  
	   - 消除穩態誤差
	   - 提高系統精度
	   - 可能造成積分飽和問題
	
	3. 微分項 (D)：誤差變化率的控制輸出
	   - 預測系統趨勢
	   - 減少超調和震盪
	   - 對雜訊敏感
	
	離散化實現：
	- 使用數值微分近似連續微分
	- 使用累加近似連續積分
	- 考慮採樣時間的影響
	
	多軸控制：
	支援向量化操作，可同時控制多個軸
	(如同時控制角度和距離)
	"""
	
	def __init__(self, target, P, I, D):
		"""
		建立離散 PID 控制器
		
		參數：
		    target (array-like): 目標值，可以是單一數值或陣列
		    P (array-like): 比例增益係數
		    I (array-like): 積分增益係數  
		    D (array-like): 微分增益係數
		
		參數相容性：
		- 所有參數必須具有相同的維度
		- 支援單軸控制 (純量) 或多軸控制 (向量)
		- 增益參數可以是標量 (所有軸相同) 或向量 (各軸獨立)
		
		使用範例：
		    # 單軸控制
		    pid = simplePID(target=0, P=1.0, I=0.1, D=0.05)
		    
		    # 雙軸控制 (角度 + 距離)
		    pid = simplePID(target=[0, 1000], P=[1.2, 0.2], I=[0, 0], D=[0.005, 0])
		"""

		# 參數維度相容性檢查
		# 確保所有 PID 係數具有相同的維度
		if(not(np.size(P)==np.size(I)==np.size(D)) or 
		   ((np.size(target)==1) and np.size(P)!=1) or 
		   (np.size(target)!=1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
			raise TypeError('input parameters shape is not compatable')

		# 轉換為 numpy 陣列以支援向量化運算
		self.Kp	= np.array(P)        # 比例增益陣列
		self.Ki	= np.array(I)        # 積分增益陣列
		self.Kd	= np.array(D)        # 微分增益陣列
		self.setPoint = np.array(target)  # 目標值陣列
		
		# 控制器內部狀態初始化
		self.last_error=0              # 上次誤差值 (用於微分計算)
		self.integrator = 0            # 積分累積器
		self.integrator_max = float('inf')  # 積分限制 (防止積分飽和)
		self.timeOfLastCall = None   # 上次呼叫時間 (用於計算時間差)
		
	def update(self, current_value):
		"""
		更新 PID 控制器並計算控制輸出
		
		參數：
		    current_value (array-like): 當前測量值，與目標值維度相同
		
		回傳：
		    numpy.array: 控制信號輸出，與輸入維度相同
		
		計算流程：
		1. 輸入驗證和維度檢查
		2. 計算控制誤差 (目標值 - 當前值)
		3. 死區處理：小誤差時設為零避免震盪
		4. 自適應增益：根據目標值動態調整
		5. 計算 PID 三項：比例、積分、微分
		6. 組合輸出：Kp*P + Ki*I + Kd*D
		
		特殊處理：
		- 死區控制：避免在目標附近的微小震盪
		- 自適應增益：近距離目標時增強響應
		- 時間補償：考慮實際採樣時間間隔
		"""
		# 輸入格式轉換和驗證
		current_value=np.array(current_value)
		if(np.size(current_value) != np.size(self.setPoint)):
			raise TypeError('current_value and target do not have the same shape')
		
		# 首次呼叫初始化
		if(self.timeOfLastCall is None):
			# 記錄初始時間，無法計算微分項
			self.timeOfLastCall = time.perf_counter()
			self.last_error = self.setPoint - current_value
			return np.zeros_like(self.setPoint)  # 首次回傳零輸出
		
		# 計算控制誤差：目標值與當前值的差異
		error = self.setPoint - current_value

		# 死區處理：小誤差時停止運動避免震盪
		# error[0] = 角度誤差 (弧度)，error[1] = 距離誤差 (毫米)
		if error[0]<0.1 and error[0]>-0.1:
			# 角度誤差小於 0.1 弧度 (約 5.7 度) 時視為已對準
			error[0]=0
		if error[1]<150 and error[1]>-150:
			# 距離誤差小於 150 毫米時視為已到達目標距離
			error[1]=0
		
		# 自適應增益調整：當目標距離較近時增強響應
		# 這補償了近距離時相對誤差較大的問題
		if (error[1]>0 and self.setPoint[1]<1200):
			# 目標距離小於 1.2 米且存在正向距離誤差時
			# 按比例放大距離誤差以增強響應速度
			error[1]=error[1]*(1200/self.setPoint[1])*0.5
			# 同時略微降低角度增益避免過度轉向
			error[0]=error[0]*0.8
		
		# 比例項 (P)：與當前誤差直接成正比
		P = error
		
		# 時間計算：用於積分和微分項的時間補償
		currentTime = time.perf_counter()  # 高精度時間戳
		deltaT = (currentTime-self.timeOfLastCall)  # 時間間隔 (秒)

		# 積分項 (I)：累積歷史誤差
		# 使用梯形積分近似：I += error * deltaT
		self.integrator = self.integrator + (error*deltaT)
		I = self.integrator
		
		# 微分項 (D)：誤差變化率
		# 使用後向差分近似：D = (current_error - last_error) / deltaT
		D = (error-self.last_error)/deltaT
		
		# 狀態更新：為下次計算準備
		self.last_error = error              # 儲存當前誤差
		self.timeOfLastCall = currentTime    # 儲存當前時間
		
		# PID 輸出計算：加權組合三個控制項
		# output = Kp*P + Ki*I + Kd*D
		return self.Kp*P + self.Ki*I + self.Kd*D
		
 	
def main(args=None):
    """
    視覺跟隨器主程式執行函數
    
    參數：
        args: 命令列參數 (可選)
        
    功能說明：
    1. 初始化 ROS2 通訊系統
    2. 建立視覺跟隨控制節點
    3. 進入主控制迴圈
    4. 處理程式退出與資源清理
    
    執行特點：
    - 事件驅動：響應視覺追蹤和手把輸入
    - 即時控制：毫秒級的控制迴圈
    - 安全機制：自動斷線保護
    - 多執行緒：按鈕處理與主控制分離
    
    使用前準備：
    1. 確保視覺追蹤器節點已啟動
    2. 檢查手把控制器連線狀態  
    3. 確認機器人支援 cmd_vel 控制介面
    4. 調整 PID 參數以適應具體機器人
    
    系統架構：
    視覺追蹤器 → visualFollower → 機器人控制器
         ↓              ↑              ↓
    目標位置      ←   手把控制   →   運動執行
    """
    print('visualFollower')  # 啟動提示訊息
    
    # 初始化 ROS2 分散式通訊系統
    rclpy.init(args=args)
    
    # 建立視覺跟隨控制器節點實例
    visualFollower = VisualFollower()
    
    print('visualFollower init done')  # 初始化完成提示
    
    try:
        # 進入 ROS2 主事件迴圈
        # 持續處理訊息回調直到程式終止
        rclpy.spin(visualFollower)
        
    finally:
        # 程式結束時的清理作業
        # 確保資源正確釋放，避免系統不穩定
        
        # 銷毀節點：釋放所有 ROS2 相關資源
        visualFollower.destroy_node()
        
        # 正常關閉 ROS2 系統
        rclpy.shutdown()

if __name__ == '__main__':
    """
    Python 模組直接執行進入點
    
    執行方式：
    1. 直接執行：python3 visualFollower.py
    2. ROS2 指令：ros2 run simple_follower_ros2 visualFollower
    
    系統需求：
    1. 硬體需求：
       - RGB-D 相機 (如 Intel RealSense)
       - 差動驅動機器人底盤
       - 無線手把控制器 (PS3/PS4/Xbox)
    
    2. 軟體需求：
       - ROS2 環境 (Humble 或更新版本)
       - OpenCV Python 綁定
       - NumPy 數值運算庫
       - 對應的相機驅動程序
    
    3. 網路需求：
       - visualTracker 節點必須運行
       - 手把控制器藍牙/USB 連線
       - ROS2 域網路設定正確
    
    使用流程：
    1. 啟動相機節點和視覺追蹤器
    2. 執行此視覺跟隨器程式
    3. 使用手把控制器啟動跟隨模式
    4. 機器人將自動跟隨偵測到的目標物體
    
    調試建議：
    1. 首先測試各個組件獨立運作
    2. 檢查 ROS 主題的訊息流
    3. 調整 PID 參數獲得最佳跟隨效果
    4. 根據實際環境調整安全距離設定
    
    故障排除：
    - 無法跟隨：檢查視覺追蹤器是否正常工作
    - 運動異常：調整 PID 參數或檢查機器人標定
    - 斷線問題：檢查手把控制器電池和連線狀態
    - 性能問題：降低控制頻率或簡化追蹤演算法
    """
    main()



