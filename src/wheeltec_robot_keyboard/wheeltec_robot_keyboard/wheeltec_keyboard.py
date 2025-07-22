#!/usr/bin/env python
# coding=utf-8

"""
WheelTec 機器人鍵盤遙控程式
========================================
此程式提供透過鍵盤控制 WheelTec 機器人運動的功能，支援：
1. 基本移動控制 (前進、後退、轉向)
2. 全向移動模式 (適用於全向輪、麥輪機器人)
3. 速度調節和平滑控制
4. 即時鍵盤輸入處理

技術架構：
- ROS2 節點實作
- geometry_msgs/Twist 訊息發布
- 跨平台鍵盤輸入處理 (Windows/Linux)
- 平滑速度控制演算法

作者：基於 Willow Garage 的 TurtleBot 鍵盤控制程式修改
適用：WheelTec 系列機器人平台
"""

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Darby Lim

# 系統函式庫導入區段
import os      # 作業系統介面，用於判斷平台類型 (Windows/Linux)
import select  # I/O 多工處理，用於非阻塞式鍵盤輸入 (僅 Linux)
import sys     # 系統參數和函數，用於標準輸入/輸出處理
import rclpy   # ROS2 Python 客戶端函式庫，提供節點、發布者等核心功能

# ROS2 訊息和服務品質導入
from geometry_msgs.msg import Twist    # ROS2 標準幾何訊息，用於機器人速度控制
from rclpy.qos import QoSProfile       # 服務品質設定，控制訊息傳遞的可靠性和效能

# 平台相依的鍵盤輸入處理模組
if os.name == 'nt':    # Windows 平台 (nt = New Technology)
    import msvcrt      # Microsoft Visual C++ Runtime，提供 Windows 鍵盤輸入功能
else:                  # Unix-like 平台 (Linux, macOS)
    import termios     # 終端機 I/O 控制，用於設定終端機屬性
    import tty         # 終端機控制功能，提供原始模式輸入

# 機器人運動參數定義區段
# 這些參數定義了不同類型機器人的速度限制，確保安全操作

# BURGER 系列機器人速度限制 (TurtleBot3 Burger)
BURGER_MAX_LIN_VEL = 0.22  # 最大線性速度：0.22 m/s (公尺每秒)
BURGER_MAX_ANG_VEL = 2.84  # 最大角速度：2.84 rad/s (徑度每秒)

# WAFFLE 系列機器人速度限制 (TurtleBot3 Waffle/WheelTec)
WAFFLE_MAX_LIN_VEL = 0.26  # 最大線性速度：0.26 m/s
WAFFLE_MAX_ANG_VEL = 1.82  # 最大角速度：1.82 rad/s

# 速度調整步長參數
# 這些參數控制每次按鍵調整速度的增減量，實現平滑的速度變化
LIN_VEL_STEP_SIZE = 0.01   # 線性速度步長：每次調整 0.01 m/s
ANG_VEL_STEP_SIZE = 0.1    # 角速度步長：每次調整 0.1 rad/s

# 使用說明訊息
# 此字串包含完整的操作指南，在程式啟動時顯示給使用者
msg = """
Control Your carrrrrrrrrr!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
b : switch to OmniMode/CommonMode
CTRL-C to quit
"""

# 錯誤訊息定義
# 當通訊發生問題時顯示的錯誤訊息
e = """
Communications Failed
"""

# 鍵盤按鍵與機器人運動方向映射表
# 這個字典定義了鍵盤按鍵對應的運動方向 (線性運動, 角運動)
# 數值意義：(前後方向, 左右轉向/橫移方向)
#   正值表示前進/右轉，負值表示後退/左轉
moveBindings = {
        'i':( 1, 0),   # 前進：      ↑
        'o':( 1,-1),   # 右前：      ↗ (前進+右轉)
        'j':( 0, 1),   # 左轉/左移：  ←
        'l':( 0,-1),   # 右轉/右移：  →
        'u':( 1, 1),   # 左前：      ↖ (前進+左轉)
        ',':(-1, 0),   # 後退：      ↓
        '.':(-1, 1),   # 左後：      ↙ (後退+左轉)
        'm':(-1,-1),   # 右後：      ↘ (後退+右轉)
           }

# 鍵盤按鍵與速度調整映射表
# 這個字典定義了速度調整按鍵對應的倍率變化
# 數值意義：(線性速度倍率, 角速度倍率)
#键值对应速度增量
speedBindings={
        'q':(1.1,1.1), # q鍵：同時增加線性和角速度 10% (1.1倍)
        'z':(0.9,0.9), # z鍵：同時減少線性和角速度 10% (0.9倍)
        'w':(1.1,1),   # w鍵：僅增加線性速度 10%，角速度不變
        'x':(0.9,1),   # x鍵：僅減少線性速度 10%，角速度不變
        'e':(1,  1.1), # e鍵：僅增加角速度 10%，線性速度不變
        'c':(1,  0.9), # c鍵：僅減少角速度 10%，線性速度不變
          }

# 全域速度參數初始化
#获取键值函数
speed = 0.2 #默认移动速度 m/s    # 預設線性速度：0.2 公尺每秒
turn  = 1   #默认转向速度 rad/   # 預設角速度：1.0 徑度每秒
# 跨平台鍵盤輸入獲取函數
# 此函數處理不同作業系統的鍵盤輸入差異，實現統一的按鍵獲取介面
def get_key(settings):
    """
    跨平台鍵盤輸入處理函數
    
    參數:
        settings: Linux 平台的終端機設定 (Windows 平台忽略)
    
    回傳:
        str: 使用者按下的按鍵字元
    
    平台差異處理:
        - Windows: 使用 msvcrt.getch() 直接獲取按鍵
        - Linux: 使用 termios 設定終端機為原始模式，避免需要按 Enter
    """
    if os.name == 'nt':  # Windows 平台處理
        # msvcrt.getch() 直接讀取按鍵，無需按 Enter
        # decode('utf-8') 將位元組轉換為字串
        return msvcrt.getch().decode('utf-8')
    
    # Linux/Unix 平台處理
    tty.setraw(sys.stdin.fileno())  # 設定標準輸入為原始模式，字元立即可讀
    
    # 使用 select 實現非阻塞輸入，避免程式卡住
    # 超時時間 0.1 秒，如果沒有輸入就回傳空字串
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    
    if rlist:  # 有輸入可讀
        key = sys.stdin.read(1)  # 讀取一個字元
    else:      # 超時，沒有輸入
        key = ''

    # 恢復終端機正常設定，避免影響其他程式
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# 速度顯示函數
# 格式化輸出當前的移動速度和轉向速度，方便使用者了解機器人狀態
#以字符串格式返回当前速度
def print_vels(speed, turn):
    """
    格式化顯示當前速度參數
    
    參數:
        speed (float): 當前線性速度 (公尺每秒)
        turn (float): 當前角速度 (徑度每秒)
    
    功能:
        在終端機顯示格式化的速度資訊，幫助使用者監控機器人狀態
    """
    print('currently:\tspeed {0}\t turn {1} '.format(
        speed,    # 線性速度值
        turn))    # 角速度值

# 主程式函數
# 整合所有功能，實現完整的鍵盤控制機器人系統
def main():
    """
    主要控制迴圈函數
    
    功能架構：
    1. 初始化 ROS2 節點和發布者
    2. 設定終端機環境 (Linux)
    3. 進入主要控制迴圈
    4. 處理鍵盤輸入和運動控制
    5. 發布 Twist 訊息到 cmd_vel topic
    6. 處理異常和清理資源
    """
    
    # 終端機設定初始化 (僅 Linux 平台需要)
    settings = None
    if os.name != 'nt':  # 非 Windows 平台
        # 保存當前終端機設定，程式結束時恢復
        settings = termios.tcgetattr(sys.stdin)
    
    # ROS2 系統初始化
    rclpy.init()  # 初始化 ROS2 Python 客戶端

    # ROS2 發布者設定區段
    # 建立服務品質 (QoS) 設定檔
    qos = QoSProfile(depth=10)  # 訊息佇列深度：10，平衡記憶體使用與訊息完整性
    
    # 建立 ROS2 節點
    node = rclpy.create_node('wheeltec_keyboard')  # 節點名稱：wheeltec_keyboard
    
    # 建立 Twist 訊息發布者
    # Topic 名稱：cmd_vel (標準機器人速度命令主題)
    # 訊息類型：geometry_msgs/Twist (包含線性和角速度)
    pub = node.create_publisher(Twist, 'cmd_vel', qos)
    
    # 控制變數初始化區段
    # 這些變數控制機器人的運動狀態和平滑控制演算法
    speed = 0.2 #默认移动速度 m/s    # 使用者設定的基準線性速度
    turn  = 1.0   #默认转向速度 rad/  # 使用者設定的基準角速度
    x      = 0.0   #前进后退方向      # 當前前後運動方向 (-1:後退, 0:停止, 1:前進)
    th     = 0.0   #转向/横向移动方向  # 當前轉向/橫移方向 (-1:左, 0:直行, 1:右)
    count  = 0.0   #键值不再范围计数  # 無效按鍵計數器，用於自動停止
    
    # 目標速度變數 (使用者輸入產生的目標值)
    target_speed = 0.0 #前进后退目标速度        # 前後運動目標速度
    target_turn  = 0.0 #转向目标速度            # 轉向運動目標速度  
    target_HorizonMove = 0.0 #横向移动目标速度  # 橫向移動目標速度 (全向模式)
    
    # 實際控制速度變數 (平滑控制演算法輸出的實際控制值)
    control_speed = 0.0 #前进后退实际控制速度        # 前後運動實際控制速度
    control_turn  = 0.0 #转向实际控制速度            # 轉向運動實際控制速度
    control_HorizonMove = 0.0 #横向移动实际控制速度  # 橫向移動實際控制速度
    
    # 運動模式變數
    Omni = 0  # 全向移動模式旗標 (0:一般模式, 非0:全向模式)
    # 主要控制迴圈開始
    # 使用 try-except 結構確保程式異常時能正確清理資源
    try:
        # 顯示操作說明和初始速度狀態
        print(msg)                        # 顯示鍵盤操作說明
        print(print_vels(speed, turn))    # 顯示當前速度設定
        
        # 無限迴圈：持續監聽鍵盤輸入並處理
        while(1):
            # 獲取使用者鍵盤輸入
            key = get_key(settings)
            
            # 運動模式切換處理
            #切换是否为全向移动模式，全向轮/麦轮小车可以加入全向移动模式
            if key=='b':               
                # 位元運算切換模式狀態 (0 變非0，非0 變 0)
                Omni=~Omni
                if Omni:  # 切換到全向移動模式
                    print("Switch to OmniMode")
                    # 修改按鍵映射：全向模式下，'.' 和 'm' 對應橫向移動
                    moveBindings['.']=[-1,-1]  # 後退+左移
                    moveBindings['m']=[-1, 1]  # 後退+右移
                else:     # 切換到一般移動模式  
                    print("Switch to CommonMode")
                    # 恢復按鍵映射：一般模式下，'.' 和 'm' 對應轉向
                    moveBindings['.']=[-1, 1]  # 後退+左轉
                    moveBindings['m']=[-1,-1]  # 後退+右轉
            # 運動方向按鍵處理
            #判断键值是否在移动/转向方向键值内
            if key in moveBindings.keys():
                # 從按鍵映射表獲取運動方向
                x  = moveBindings[key][0]  # 前後運動分量 (-1, 0, 1)
                th = moveBindings[key][1]  # 左右運動分量 (-1, 0, 1)
                count = 0  # 重置無效按鍵計數器

            # 速度調整按鍵處理
            #判断键值是否在速度增量键值内
            elif key in speedBindings.keys():
                # 根據按鍵調整基準速度
                speed = speed * speedBindings[key][0]  # 調整線性速度倍率
                turn  = turn  * speedBindings[key][1]  # 調整角速度倍率
                count = 0  # 重置無效按鍵計數器
                print(print_vels(speed,turn)) #速度发生变化，打印出来

            # 緊急停止按鍵處理
            #空键值/'k',相关变量置0
            elif key == ' ' or key == 'k' :
                # 立即停止：所有運動變數歸零
                x  = 0              # 前後運動歸零
                th = 0.0            # 左右運動歸零
                control_speed = 0.0      # 前後控制速度歸零
                control_turn  = 0.0      # 轉向控制速度歸零
                HorizonMove   = 0.0      # 橫向移動歸零

            # 未知按鍵處理 (安全機制)
            #长期识别到不明键值，相关变量置0
            else:
                count = count + 1  # 累計無效按鍵次數
                if count > 4:      # 連續 4 次無效按鍵後自動停止運動
                    x  = 0         # 停止前後運動
                    th = 0.0       # 停止轉向運動
                # 檢查是否為 Ctrl+C (退出信號)
                if (key == '\x03'):  # \x03 是 Ctrl+C 的 ASCII 碼
                    break            # 跳出主迴圈，結束程式

           # 目標速度計算區段
           # 根據使用者輸入和當前基準速度計算各軸向的目標速度
           #根据速度与方向计算目标速度
            target_speed = speed * x           # 前後目標速度 = 基準速度 × 方向係數
            target_turn  = turn * th           # 轉向目標速度 = 基準角速度 × 轉向係數  
            target_HorizonMove = speed*th      # 橫向移動目標速度 = 基準速度 × 橫向係數

            # 平滑速度控制演算法區段
            # 使用增量式控制避免速度突變，提供更平滑的機器人運動
            
            # 前後運動平滑控制
            #平滑控制，计算前进后退实际控制速度
            if target_speed > control_speed:
                # 目標速度大於當前控制速度：逐步加速
                # min() 確保不會超過目標速度，0.1 是加速度限制
                control_speed = min( target_speed, control_speed + 0.1 )
            elif target_speed < control_speed:
                # 目標速度小於當前控制速度：逐步減速
                # max() 確保不會低於目標速度，0.1 是減速度限制  
                control_speed = max( target_speed, control_speed - 0.1 )
            else:
                # 目標速度等於當前控制速度：保持當前速度
                control_speed = target_speed

            # 轉向運動平滑控制
            #平滑控制，计算转向实际控制速度
            if target_turn > control_turn:
                # 轉向加速：限制角加速度為 0.5 rad/s²
                control_turn = min( target_turn, control_turn + 0.5 )
            elif target_turn < control_turn:
                # 轉向減速：限制角減速度為 0.5 rad/s²
                control_turn = max( target_turn, control_turn - 0.5 )
            else:
                # 保持轉向速度
                control_turn = target_turn

            # 橫向移動平滑控制 (僅全向移動模式使用)
            #平滑控制，计算横向移动实际控制速度
            if target_HorizonMove > control_HorizonMove:
                # 橫向加速：限制加速度為 0.1 m/s²
                control_HorizonMove = min( target_HorizonMove, control_HorizonMove + 0.1 )
            elif target_HorizonMove < control_HorizonMove:
                # 橫向減速：限制減速度為 0.1 m/s²
                control_HorizonMove = max( target_HorizonMove, control_HorizonMove - 0.1 )
            else:
                # 保持橫向速度
                control_HorizonMove = target_HorizonMove
         
            # Twist 訊息建立和發布區段
            # 根據當前運動模式組裝適當的速度命令訊息
            twist = Twist() #创建ROS速度话题变量  # 建立 ROS2 Twist 訊息物件
            
            if Omni==0:  # 一般運動模式 (差動驅動、阿克曼轉向等)
                # 設定線性速度 (僅 X 軸，前後運動)
                twist.linear.x  = control_speed    # 前後速度 (m/s)
                twist.linear.y = 0.0               # Y 軸速度固定為 0
                twist.linear.z = 0.0               # Z 軸速度固定為 0
                
                # 設定角速度 (僅 Z 軸，偏航角轉動)
                twist.angular.x = 0.0              # X 軸角速度固定為 0 (滾轉)
                twist.angular.y = 0.0              # Y 軸角速度固定為 0 (俯仰)
                twist.angular.z = control_turn     # Z 軸角速度 (偏航轉向)
            else:        # 全向運動模式 (全向輪、麥輪等)
                # 設定線性速度 (X 和 Y 軸皆可控制)
                twist.linear.x  = control_speed         # 前後速度 (m/s)
                twist.linear.y = control_HorizonMove    # 橫向速度 (m/s)
                twist.linear.z = 0.0                    # Z 軸速度固定為 0
                
                # 設定角速度 (全向模式下通常不轉向)
                twist.angular.x = 0.0              # X 軸角速度固定為 0
                twist.angular.y = 0.0              # Y 軸角速度固定為 0
                twist.angular.z = 0.0              # Z 軸角速度固定為 0 (不轉向)
            
            # 發布速度命令到 ROS2 系統
            # 訊息將透過 'cmd_vel' topic 傳送給機器人控制器
            pub.publish(twist)

    # 例外處理區段
    # 捕捉程式執行過程中的任何異常，確保資源正確釋放
    except Exception as e:
        print(e)  # 印出錯誤訊息供除錯使用

    # 資源清理區段 (finally 區塊保證一定會執行)
    finally:
        # 發送停止命令確保機器人安全停止
        # 建立全零速度的 Twist 訊息
        twist = Twist()
        # 線性速度歸零
        twist.linear.x = 0.0   # 前後速度歸零
        twist.linear.y = 0.0   # 橫向速度歸零  
        twist.linear.z = 0.0   # 垂直速度歸零

        # 角速度歸零
        twist.angular.x = 0.0  # 滾轉角速度歸零
        twist.angular.y = 0.0  # 俯仰角速度歸零
        twist.angular.z = 0.0  # 偏航角速度歸零

        # 發布停止命令
        pub.publish(twist)

        # 恢復終端機設定 (僅 Linux 平台)
        if os.name != 'nt':
            # 恢復程式開始前儲存的終端機設定
            # TCSADRAIN: 等待輸出緩衝區清空後再改變設定
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

# 程式進入點
# 當此檔案被直接執行時 (而非被其他模組匯入)，執行主函數
if __name__ == '__main__':
    main()  # 呼叫主要控制函數
