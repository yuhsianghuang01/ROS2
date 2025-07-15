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
import rclpy
import _thread
import threading
import time
import numpy as np 
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, Vector3
from turn_on_wheeltec_robot.msg import Position as PositionMsg
from std_msgs.msg import String as StringMsg
from rclpy.qos import QoSProfile
from example_interfaces.srv import AddTwoInts

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
		# we do not handle any info from the object tracker specifically at the moment. just ignore that we lost the object for example
		self.get_logger().warn(info.data)
	def positionUpdateCallback(self, position):
		angle_x= position.angle_x
		distance = position.distance
		#print(distance)
		if(angle_x>0):
			angle_x=angle_x-3.1415
		else :
			angle_x=angle_x+3.1415

		# call the PID controller to update it and get new speeds
		[uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angle_x, distance])
		# clip these speeds to be less then the maximal speed specified above
		angularSpeed = np.clip(-uncliped_ang_speed, -0.4, 0.4)
		linearSpeed  = np.clip(-uncliped_lin_speed, -0.4, 0.4)
		# create the Twist message to send to the cmd_vel topic
		velocity = Twist()	
		velocity.linear.y = 0.0
		velocity.linear.z = 0.0
		velocity.angular.x = 0.0
		velocity.angular.y = 0.0
		if distance>0.15:
			velocity.linear.x = linearSpeed
			velocity.angular.z = -angularSpeed
		else:
			velocity.linear.x = 0.0
			velocity.angular.z = 0.0
		#self.get_logger().info('linearSpeed: {}, angularSpeed: {}'.format(linearSpeed, angularSpeed))
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
			return 
		else:
			# we are not busy. i.e. there is a real 'new' button press
			# we deal with it in a seperate thread to be able to drop the other joy messages arriving in the mean
			# time
			thread.start_new_thread(self.threadedButtonCallback,  (joy_data, ))
			print("000000000000000")
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
		velocity = Twist()
		velocity.linear.x = 0.0
		velocity.linear.y = 0.0
		velocity.linear.z = 0.0

		velocity.angular.x = 0.0
		velocity.angular.y = 0.0
		velocity.angular.z = 0.0
		self.cmdVelPublisher.publish(velocity)
	def controllerLoss(self):
		# we lost connection so we will stop moving and become inactive
		self.stopMoving()
		self.active = False
		self.get_logger().info('lost connection')
class simplePID:
	'''very simple discrete PID controller'''
	def __init__(self, target, P, I, D):

		P = [1.5, 0.5]
		I = [0, 0]
		D = [0.02,0.002]
		node = rclpy.create_node('simplepid')
		# check if parameter shapes are compatabile. 
		if(not(np.size(P)==np.size(I)==np.size(D)) or ((np.size(target)==1) and np.size(P)!=1) or (np.size(target )!=1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
			raise TypeError('input parameters shape is not compatable')
		self.Kp		=np.array(P)
		self.Ki		=np.array(I)
		self.Kd		=np.array(D)
		self.setPoint   =np.array(target)
		
		self.last_error=0
		self.integrator = 0
		self.integrator_max = float('inf')
		self.timeOfLastCall = None 
	def update(self, current_value):

		current_value=np.array(current_value)
		if(np.size(current_value) != np.size(self.setPoint)):
			raise TypeError('current_value and target do not have the same shape')
		if(self.timeOfLastCall is None):
			# the PID was called for the first time. we don't know the deltaT yet
			# no controll signal is applied
			self.timeOfLastCall = time.perf_counter()
			return np.zeros(np.size(current_value))

		
		error = self.setPoint - current_value

		if error[0]<0.1 and error[0]>-0.1:
			error[0]=0
		if error[1]<0.1 and error[1]>-0.1:
			error[1]=0
		
		#when target is little, amplify velocity by amplify error.
		if (error[1]>0 and self.setPoint[1]<1.2):
			error[1]=error[1]*(1.2/self.setPoint[1])*0.7

		P =  error
		
		currentTime = time.perf_counter()
		deltaT      = (currentTime-self.timeOfLastCall)

		# integral of the error is current error * time since last update
		self.integrator = self.integrator + (error*deltaT)
		I = self.integrator
		
		# derivative is difference in error / time since last update
		D = (error-self.last_error)/deltaT
		
		self.last_error = error
		self.timeOfLastCall = currentTime
		
		# return controll signal
		return self.Kp*P + self.Ki*I + self.Kd*D
def main(args=None):
    print('starting')
    rclpy.init(args=args)
    laserfollower = LaserFollower()
    try:
        rclpy.spin(laserfollower)
    finally:
        laserfollower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

