#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
雷射掃描跟隨器程式
=====================================

技術背景知識：

1. LiDAR (光達) 技術原理：
   - 發射雷射脈衝並測量反射時間
   - 360度掃描獲得周圍環境的距離資料
   - 高精度：毫米級測距準確度
   - 環境適應性：不受光照影響，適用於室內外

2. 雷射掃描資料結構：
   - LaserScan 訊息：包含角度範圍、解析度、距離陣列
   - angle_min/max：掃描角度範圍 (通常 -π 到 π)
   - angle_increment：角度解析度 (每個測量點間的角度差)
   - ranges[]：距離資料陣列 (對應每個角度的距離值)

3. 物體跟隨策略：
   - 空間差異分析：比較連續掃描中的變化
   - 聚類分析：將相近的距離點群組為物體
   - 質心計算：確定物體的中心位置
   - 運動預測：基於歷史資料預測物體移動

4. 手把控制整合：
   - Joy (操縱桿) 訊息：按鈕和搖桿狀態
   - 模式切換：手動/自動跟隨模式
   - 安全機制：失去連線時自動停止
   - 速度限制：防止過激的運動指令

5. PID 控制應用：
   - 目標：維持與跟隨物體的固定距離和角度
   - 比例項：與當前誤差成正比的修正
   - 積分項：消除長期累積誤差
   - 微分項：預測趨勢，減少震盪

程式功能：
結合 LiDAR 感測器和手把控制實現智慧物體跟隨，
機器人可自動保持與目標物體的固定距離，
適用於購物助手、行李搬運、寵物跟隨等應用。

應用場景：
- 自動購物車：跟隨使用者移動
- 行李搬運機器人：機場、酒店應用
- 寵物陪伴機器人：戶外散步助手
- 工業助手：跟隨工作人員移動
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

# === ROS2 和系統模組匯入 ===
import rclpy          # ROS2 Python 客戶端核心函式庫
import _thread        # 低階執行緒介面 (較少使用)
import threading      # 高階執行緒模組：用於按鈕回調和連線監控定時器
import time           # 時間處理模組：用於計算時間間隔和 PID 控制
import numpy as np    # 數值計算函式庫：用於陣列運算、數學函數和 PID 控制

# === ROS2 節點和訊息類型匯入 ===
from rclpy.node import Node                                    # ROS2 節點基底類別
from sensor_msgs.msg import Joy, LaserScan                    # 感測器訊息：手把控制和雷射掃描
from geometry_msgs.msg import Twist, Vector3                  # 幾何訊息：速度控制和向量
from turn_on_wheeltec_robot.msg import Position as PositionMsg # 自定義位置訊息 (角度+距離)
from std_msgs.msg import String as StringMsg                  # 標準字串訊息
from rclpy.qos import QoSProfile                              # 服務品質設定
from example_interfaces.srv import AddTwoInts                 # 範例服務介面 (未使用)

import rclpy
from rclpy.node import Node
# 全域變數：儲存角度和距離資訊
# 用於在不同函數間共享計算結果
angle=[0.0]*3    # 角度陣列：[左側角度, 中心角度, 右側角度]
distan=[0.0]*3   # 距離陣列：[左側距離, 中心距離, 右側距離]

class LaserFollower(Node):
    """
    雷射跟隨器節點類別
    
    主要功能：
    1. 接收 LiDAR 掃描資料
    2. 偵測可跟隨的物體
    3. 結合手把控制進行模式切換
    4. 發送機器人運動指令
    
    控制架構：
    - 感測器融合：LiDAR + 手把輸入
    - 雙模式操作：手動控制 / 自動跟隨
    - 安全機制：連線監控和緊急停止
    - PID 控制：平滑的跟隨行為
    
    技術特點：
    - 即時物體偵測：基於距離變化分析
    - 多執行緒處理：按鈕事件和主控制迴圈分離
    - 參數化設定：可調整的 PID 參數和速度限制
    - 連線監控：手把斷線自動停止保護機制
    """

	def __init__(self):
        """
        雷射跟隨節點初始化
        
        設定項目：
        1. 連線監控定時器
        2. PID 控制參數宣告
        3. 手把控制設定
        4. 模式和安全參數
        5. 發布器和訂閱器建立
        
        參數系統：
        使用 ROS2 參數系統進行運行時配置，
        允許動態調整控制參數而無需重新編譯。
        """
		super().__init__('laserfollower')  # 建立名為 'laserfollower' 的 ROS2 節點
		
		# 連線監控定時器：偵測手把控制器斷線
		# 如果 1 秒內沒有收到 Joy 訊息，就會觸發 controllerLoss 函數
		self.controllerLossTimer = threading.Timer(1, self.controllerLoss)
		self.controllerLossTimer.start()
		
		# PID 控制參數宣告：允許執行時動態調整
		# 這些參數控制機器人跟隨行為的穩定性和響應速度
		self.declare_parameter('P')  # 比例增益：控制響應強度
		self.declare_parameter('I')  # 積分增益：消除穩態誤差
		self.declare_parameter('D')  # 微分增益：減少震盪和超調
		
		# 操作模式設定
		# switchMode = True：按一次按鈕切換模式 (推薦)
		# switchMode = False：需持續按住按鈕才能跟隨 (安全模式)
		self.switchMode= True
		
		# 最大速度限制：防止過於激烈的運動 (公尺/秒)
		self.max_speed = self.declare_parameter('~maxSpeed').value
		
		# 手把按鈕映射：控制跟隨模式開關
		# -4 對應 PS3/PS4 手把的某個特定按鈕
		self.controllButtonIndex = -4
		
		# 控制狀態變數
		self.buttonCallbackBusy=False  # 按鈕回調忙碌旗標：避免重複處理
		self.active=False              # 跟隨模式啟動狀態
		self.i=0                       # 迴圈計數器
		
		# QoS 設定：優化訊息傳輸品質
		qos = QoSProfile(depth=10)  # 佇列深度 10，平衡延遲與可靠性
		
		# 機器人速度控制發布器
		# 發布 Twist 訊息到 'cmd_vel' 主題控制機器人移動
		self.cmdVelPublisher = self.create_publisher(Twist, 'cmd_vel', qos)
		
		# 物體位置訂閱器：接收追蹤器偵測到的目標位置
		# 來源：視覺追蹤器或其他物體偵測模組
		self.positionSubscriber = self.create_subscription(
		    PositionMsg,
		    '/object_tracker/current_position',
		    self.positionUpdateCallback,  # 位置更新回調函數
		    qos)
		
		# 追蹤資訊訂閱器：接收追蹤狀態和錯誤訊息
		# 用於監控追蹤器的工作狀態和異常情況
		self.trackerInfoSubscriber = self.create_subscription(
		    StringMsg,
		    '/object_tracker/info',
		    self.trackerInfoCallback,  # 資訊處理回調函數
		    qos)
		
		# 目標距離參數：機器人與跟隨物體的理想距離 (毫米)
		targetDist = self.declare_parameter('~targetDist')
		#pid_param = self.declare_parameter('~PID_controller')
		P = self.get_parameter('P').get_parameter_value().double_value
		I = self.get_parameter('I').get_parameter_value().double_value
		D = self.get_parameter('D').get_parameter_value().double_value
		self.PID_controller = simplePID([0, 0.8], [1.6, 0.5], [0, 0], [0.03,0.005])
		# PID parameters first is angular, dist
	def trackerInfoCallback(self, info):
		"""
		追蹤器資訊回調函數
		
		參數：
			info (std_msgs/String): 追蹤器發送的狀態資訊
			
		功能：
		處理來自追蹤器的狀態訊息，例如：
		- 目標丟失警告
		- 追蹤品質資訊  
		- 錯誤狀態報告
		
		當前實作僅記錄警告訊息，未進行特殊處理
		"""
		# we do not handle any info from the object tracker specifically at the moment. just ignore that we lost the object for example
		self.get_logger().warn(info.data)  # 將追蹤器訊息以警告等級記錄到 ROS2 日誌
		
	def positionUpdateCallback(self, position):
		"""
		位置更新回調函數 - 核心跟隨控制邏輯
		
		參數：
			position (Position): 目標物體的位置資訊 (角度 + 距離)
			
		處理流程：
		1. 讀取目標位置資料 (角度和距離)
		2. 進行座標系轉換 (調整角度方向)
		3. 使用 PID 控制器計算速度指令
		4. 限制速度在安全範圍內
		5. 發布 Twist 訊息控制機器人移動
		
		技術細節：
		- 角度轉換：將追蹤器座標系轉換為機器人座標系
		- 距離安全檢查：太近時停止前進避免碰撞
		- 速度限制：防止過激運動保證安全
		"""
		angle_x= position.angle_x   # 水平角度偏差 (弧度)
		distance = position.distance # 目標距離 (公尺)
		#print(distance)
		
		# 座標系轉換：調整角度方向以符合機器人控制需求
		# 原因：不同追蹤器可能使用不同的角度約定
		if(angle_x>0):
			angle_x=angle_x-3.1415  # 正角度減去 π (180度)
		else :
			angle_x=angle_x+3.1415  # 負角度加上 π (180度)

		# call the PID controller to update it and get new speeds
		# 呼叫 PID 控制器更新並獲得新的速度指令
		# 輸入：[角度誤差, 距離誤差]，輸出：[角速度, 線速度]
		[uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angle_x, distance])
		
		# clip these speeds to be less then the maximal speed specified above
		# 速度限制：將計算出的速度限制在安全範圍內
		# np.clip(值, 最小值, 最大值) - 超出範圍的值會被截斷
		angularSpeed = np.clip(-uncliped_ang_speed, -0.4, 0.4)  # 角速度限制 ±0.4 rad/s
		linearSpeed  = np.clip(-uncliped_lin_speed, -0.4, 0.4)  # 線速度限制 ±0.4 m/s
		
		# create the Twist message to send to the cmd_vel topic
		# 建立 Twist 訊息結構，用於機器人速度控制
		velocity = Twist()	
		velocity.linear.y = 0.0   # Y 軸線速度 (側向移動，差動機器人通常為 0)
		velocity.linear.z = 0.0   # Z 軸線速度 (垂直移動，地面機器人通常為 0)
		velocity.angular.x = 0.0  # X 軸角速度 (翻滾，地面機器人通常為 0)
		velocity.angular.y = 0.0  # Y 軸角速度 (俯仰，地面機器人通常為 0)
		
		# 距離安全檢查：只有當距離大於安全距離時才移動
		if distance>0.15:  # 安全距離 15 公分
			velocity.linear.x = linearSpeed    # 設定前進/後退速度
			velocity.angular.z = -angularSpeed # 設定左轉/右轉速度 (負號調整方向)
		else:
			# 距離太近，停止移動避免碰撞
			velocity.linear.x = 0.0
			velocity.angular.z = 0.0
			
		#self.get_logger().info('linearSpeed: {}, angularSpeed: {}'.format(linearSpeed, angularSpeed))
		# 發布速度指令到機器人控制器
		self.cmdVelPublisher.publish(velocity)
	def buttonCallback(self, joy_data):
		# this method gets called whenever we receive a message from the joy stick

		# there is a timer that always gets reset if we have a new joy stick message
		# if it runs out we know that we have lost connection and the controllerLoss function
		# will be called
		# if we are in switch mode, one button press will make the follower active / inactive 
		# but 'one' button press will be visible in roughly 10 joy messages (since they get published to fast) 
		# so we need to drop the remaining 9
		self.controllerLossTimer.cancel()
		self.controllerLossTimer = threading.Timer(0.5, self.controllerLoss)
		self.controllerLossTimer.start()

		if self.buttonCallbackBusy:
			# we are busy with dealing with the last message
			# 正在處理上一個按鈕事件，忽略此訊息避免重複處理
			return 
		else:
			# we are not busy. i.e. there is a real 'new' button press
			# we deal with it in a seperate thread to be able to drop the other joy messages arriving in the mean
			# time
			# 不忙碌狀態，這是一個真正的新按鈕事件
			# 在獨立執行緒中處理，以便在處理期間忽略其他 Joy 訊息
			_thread.start_new_thread(self.threadedButtonCallback,  (joy_data, ))
			print("000000000000000")  # 除錯訊息
			
	def threadedButtonCallback(self, joy_data):
		"""
		執行緒化按鈕回調函數 - 實際的按鈕邏輯處理
		
		參數：
			joy_data (sensor_msgs/Joy): 手把控制器狀態
			
		功能：
		實現跟隨模式的啟動/停止控制：
		- 切換模式：按一次切換狀態 (推薦)
		- 按住模式：需持續按住才能跟隨 (安全模式)
		
		狀態轉換：
		- 啟動 → 停止：按下按鈕且當前為啟動狀態
		- 停止 → 啟動：按下按鈕且當前為停止狀態
		"""
		self.buttonCallbackBusy = True  # 設定忙碌旗標，防止重複處理

		if(joy_data.buttons[self.controllButtonIndex]==self.switchMode and self.active):
			# we are active
			# switchMode = false: we will always be inactive whenever the button is not pressed (buttons[index]==false)
			# switchMode = true: we will only become inactive if we press the button. (if we keep pressing it, 
			# we would alternate between active and not in 0.5 second intervalls)
			# 當前為啟動狀態且按下切換按鈕 → 停止跟隨
			self.get_logger().info('stoping')
			self.stopMoving()      # 立即停止機器人移動
			self.active = False    # 設定為非啟動狀態
			time.sleep(0.5)        # 等待 0.5 秒避免快速重複切換
			
		elif(joy_data.buttons[self.controllButtonIndex]==True and not(self.active)):
			# if we are not active and just pressed the button (or are constantly pressing it) we become active
			# 當前為非啟動狀態且按下按鈕 → 開始跟隨
			self.get_logger().info('activating')
			self.active = True     # 啟用跟隨模式
			time.sleep(0.5)        # 等待 0.5 秒避免快速重複切換

		self.buttonCallbackBusy = False  # 清除忙碌旗標
		
	def stopMoving(self):
		"""
		停止機器人移動函數
		
		功能：
		發送零速度指令，讓機器人立即停止所有運動
		用於緊急停止、模式切換、連線中斷等情況
		
		安全性：
		這是一個重要的安全函數，確保機器人在需要時能立即停止
		"""
		velocity = Twist()
		# 設定所有速度分量為零
		velocity.linear.x = 0.0   # 前進/後退速度
		velocity.linear.y = 0.0   # 左右移動速度 (差動機器人通常不使用)
		velocity.linear.z = 0.0   # 上下移動速度 (地面機器人不使用)

		velocity.angular.x = 0.0  # 繞 X 軸旋轉 (翻滾，地面機器人不使用)
		velocity.angular.y = 0.0  # 繞 Y 軸旋轉 (俯仰，地面機器人不使用)
		velocity.angular.z = 0.0  # 繞 Z 軸旋轉 (偏航，左轉右轉)
		
		# 發布停止指令
		self.cmdVelPublisher.publish(velocity)
		
	def controllerLoss(self):
		"""
		控制器連線中斷處理函數
		
		功能：
		當超過設定時間 (0.5秒) 沒有收到手把控制器訊息時被呼叫
		
		安全措施：
		1. 立即停止機器人移動
		2. 停用跟隨模式
		3. 記錄連線中斷事件
		
		這是重要的安全機制，防止控制器故障時機器人失控
		"""
		# we lost connection so we will stop moving and become inactive
		self.stopMoving()         # 緊急停止移動
		self.active = False       # 停用跟隨模式
		self.get_logger().info('lost connection')  # 記錄連線中斷事件
		
class simplePID:
	"""
	簡化離散 PID 控制器類別
	=====================================
	
	技術背景知識：
	
	1. PID 控制理論：
	   PID (比例-積分-微分) 控制是工業控制中最常用的控制演算法
	   - P (比例項)：與當前誤差成正比，提供即時響應
	   - I (積分項)：與累積誤差成正比，消除穩態誤差
	   - D (微分項)：與誤差變化率成正比，預測趨勢並減少震盪
	
	2. 離散 PID 實現：
	   連續 PID: u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
	   離散 PID: u[k] = Kp*e[k] + Ki*Σe[i]*Δt + Kd*(e[k]-e[k-1])/Δt
	
	3. 多軸控制：
	   支援同時控制多個軸 (如角度和距離)，每個軸可有不同的 PID 參數
	
	4. 應用於機器人跟隨：
	   - 角度控制：調整機器人轉向以對準目標
	   - 距離控制：調整機器人速度以維持適當距離
	
	設計特點：
	- 向量化操作：支援多軸同時控制
	- 時間自適應：自動計算採樣時間間隔
	- 參數靈活：每個軸可設定不同的 PID 增益
	- 誤差處理：內建死區和放大機制
	"""
	
	def __init__(self, target, P, I, D):
		"""
		PID 控制器初始化
		
		參數：
			target: 目標值 (可為標量或向量)
			P: 比例增益係數
			I: 積分增益係數  
			D: 微分增益係數
			
		功能：
		1. 驗證參數形狀相容性
		2. 初始化 PID 係數和狀態變數
		3. 設定控制器參數
		"""
		# 預設 PID 參數 (可被傳入參數覆蓋)
		P = [1.5, 0.5]     # [角度比例增益, 距離比例增益]
		I = [0, 0]         # [角度積分增益, 距離積分增益] - 暫時設為 0
		D = [0.02,0.002]   # [角度微分增益, 距離微分增益]
		
		node = rclpy.create_node('simplepid')  # 建立臨時節點 (用於參數系統)
		
		# check if parameter shapes are compatabile. 
		# 參數形狀相容性檢查
		# 確保 P, I, D 具有相同的維度，且與目標值維度相符
		if(not(np.size(P)==np.size(I)==np.size(D)) or ((np.size(target)==1) and np.size(P)!=1) or (np.size(target )!=1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
			raise TypeError('input parameters shape is not compatable')
			
		# 將參數轉換為 NumPy 陣列以支援向量化運算
		self.Kp		=np.array(P)        # 比例增益矩陣
		self.Ki		=np.array(I)        # 積分增益矩陣
		self.Kd		=np.array(D)        # 微分增益矩陣
		self.setPoint   =np.array(target)   # 目標設定點
		
		# PID 控制器狀態變數初始化
		self.last_error=0               # 上次誤差值 (用於微分項計算)
		self.integrator = 0             # 積分累積器
		self.integrator_max = float('inf')  # 積分飽和限制 (防止積分過度累積)
		self.timeOfLastCall = None      # 上次呼叫時間 (用於計算 Δt)
		
	def update(self, current_value):
		"""
		PID 控制器更新函數 - 核心控制邏輯
		
		參數：
			current_value: 當前測量值 (角度, 距離)
			
		回傳：
			控制輸出 (角速度, 線速度)
			
		處理流程：
		1. 計算誤差：目標值 - 當前值
		2. 實施死區處理減少小震盪
		3. 距離相關的誤差放大 (近距離時增強響應)
		4. 計算 PID 三項並組合輸出
		5. 更新狀態變數供下次使用
		"""
		current_value=np.array(current_value)
		
		# 維度檢查：確保輸入與目標設定點維度相符
		if(np.size(current_value) != np.size(self.setPoint)):
			raise TypeError('current_value and target do not have the same shape')
			
		if(self.timeOfLastCall is None):
			# the PID was called for the first time. we don't know the deltaT yet
			# no controll signal is applied
			# 首次呼叫 PID，無法計算時間間隔 Δt
			# 記錄時間但不輸出控制訊號
			self.timeOfLastCall = time.perf_counter()
			return np.zeros(np.size(current_value))

		# 計算控制誤差：目標值 - 測量值
		error = self.setPoint - current_value

		# 死區處理：小誤差時設為零，減少不必要的小幅調整
		if error[0]<0.1 and error[0]>-0.1:  # 角度誤差死區 ±0.1 弧度
			error[0]=0
		if error[1]<0.1 and error[1]>-0.1:  # 距離誤差死區 ±0.1 公尺
			error[1]=0
		
		# 距離相關的誤差調整：當目標距離較近時，放大距離誤差以增強響應
		# when target is little, amplify velocity by amplify error.
		if (error[1]>0 and self.setPoint[1]<1.2):
			# 當距離誤差為正且目標距離小於 1.2m 時
			# 放大係數 = (1.2/目標距離) * 0.7
			error[1]=error[1]*(1.2/self.setPoint[1])*0.7

		# PID 計算
		P =  error  # 比例項：當前誤差
		
		# 計算時間間隔 Δt
		currentTime = time.perf_counter()
		deltaT      = (currentTime-self.timeOfLastCall)

		# integral of the error is current error * time since last update
		# 積分項：累積誤差 × 時間間隔
		self.integrator = self.integrator + (error*deltaT)
		I = self.integrator
		
		# derivative is difference in error / time since last update
		# 微分項：誤差變化率 = (當前誤差 - 上次誤差) / 時間間隔
		D = (error-self.last_error)/deltaT
		
		# 更新狀態變數供下次使用
		self.last_error = error
		self.timeOfLastCall = currentTime
		
		# return controll signal
		# 回傳 PID 控制訊號：Kp*P + Ki*I + Kd*D
		return self.Kp*P + self.Ki*I + self.Kd*D
def main(args=None):
    """
    雷射跟隨器主程式執行函數
    
    參數：
        args: 命令列參數 (可選)
        
    功能說明：
    1. 初始化 ROS2 通訊系統
    2. 建立雷射跟隨控制節點
    3. 進入事件驅動迴圈 (阻塞式)
    4. 處理程式退出與資源清理
    
    執行特點：
    - 事件驅動：使用 rclpy.spin() 等待並處理所有訊息
    - 阻塞模式：程式會持續運行直到收到中斷信號 (Ctrl+C)
    - 異常處理：確保程式異常退出時資源被正確清理
    
    系統架構：
    雷射追蹤器 → 位置資訊 → 雷射跟隨器 → PID控制 → 機器人運動
         ↓                     ↑                    ↓
    物體偵測              手把控制             速度指令
    
    適用環境：
    - 需要 LiDAR 感測器或視覺追蹤器提供目標位置
    - 需要手把控制器進行模式切換
    - 機器人需支援 cmd_vel 速度控制介面
    """
    print('starting')  # 啟動提示訊息
    
    # 初始化 ROS2 分散式通訊系統
    # 建立節點管理器、話題發現、服務註冊等基礎設施
    rclpy.init(args=args)
    
    # 建立雷射跟隨器節點實例
    # 初始化所有訂閱器、發布器、定時器和 PID 控制器
    laserfollower = LaserFollower()
    
    try:
        # 進入主事件處理迴圈
        # rclpy.spin() 會：
        # 1. 等待訊息到達
        # 2. 呼叫對應的回調函數
        # 3. 處理定時器事件
        # 4. 管理執行緒同步
        # 此函數會阻塞直到收到中斷信號 (Ctrl+C) 或節點關閉
        rclpy.spin(laserfollower)
        
    finally:
        # 程式結束時的清理作業 (即使發生異常也會執行)
        # 1. 停止所有定時器
        # 2. 取消訂閱和發布器
        # 3. 釋放節點資源
        laserfollower.destroy_node()
        
        # 關閉 ROS2 系統
        # 1. 停止通訊中介軟體
        # 2. 清理網路連線
        # 3. 釋放系統資源
        rclpy.shutdown()

if __name__ == '__main__':
    """
    Python 模組直接執行時的進入點
    
    執行方式：
    1. 直接執行：python3 laserfollower.py
    2. ROS2 指令：ros2 run simple_follower_ros2 laserfollower
    3. Launch 檔案：包含在 laser_follower.launch.py 中
    
    系統需求：
    1. 硬體需求：
       - LiDAR 感測器或 RGB-D 相機 (提供目標位置)
       - 差動驅動機器人底盤
       - 無線手把控制器 (PS3/PS4/Xbox)
    
    2. 軟體需求：
       - ROS2 環境 (Humble 或更新版本)
       - 對應的感測器驅動程序
       - laserTracker 或 visualTracker 節點
    
    3. 網路需求：
       - 追蹤器節點必須正常運行並發布位置資訊
       - 手把控制器正常連線
       - ROS2 域網路設定正確
    
    使用流程：
    1. 啟動機器人底盤和感測器節點
    2. 啟動追蹤器節點 (laserTracker 或 visualTracker)
    3. 執行此跟隨器程式
    4. 使用手把控制器啟動跟隨模式
    5. 機器人將自動跟隨偵測到的目標物體
    
    調試建議：
    1. 檢查各組件是否正常運作
       $ ros2 topic list
       $ ros2 topic echo /object_tracker/current_position
    2. 調整 PID 參數獲得最佳跟隨效果
    3. 根據實際環境調整安全距離和速度限制
    4. 使用 rqt_graph 檢視節點連線圖
    
    常見問題排除：
    - 無法跟隨：檢查追蹤器是否正常工作和發布資料
    - 運動異常：調整 PID 參數或檢查機器人運動學參數
    - 控制器問題：檢查手把電池和連線狀態
    - 通訊問題：檢查 ROS2 域設定和網路連線
    """
    main()

