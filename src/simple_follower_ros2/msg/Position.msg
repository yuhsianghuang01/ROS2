# Position.msg - 位置資訊訊息定義
# =====================================
# 
# 技術背景知識：
# 1. ROS2 訊息系統：
#    - 定義自定義資料結構用於節點間通訊
#    - .msg 檔案會自動生成對應的 Python/C++ 類別
#    - 支援基本型別 (int32, float32, bool) 和複合型別
#
# 2. 機器人座標系統：
#    - angleX: 水平角度偏差 (弧度或度數)
#    - angleY: 垂直角度偏差 (弧度或度數) 
#    - distance: 目標物體距離 (公尺或毫米)
#
# 3. 應用場景：
#    - 視覺追蹤：相機偵測到的目標相對位置
#    - 雷射掃描：LiDAR 偵測到的物體位置資訊
#    - 路徑規劃：導航系統中的目標點資訊
#
# 使用範例：
#   from turn_on_wheeltec_robot.msg import Position
#   pos = Position()
#   pos.angleX = 0.5    # 目標在機器人右側 0.5 弧度
#   pos.angleY = 0.0    # 目標與機器人同一水平面
#   pos.distance = 1.2  # 目標距離機器人 1.2 公尺

float32 angleX     # 水平角度偏差 (弧度)：正值表示目標在右側，負值表示在左側
float32 angleY     # 垂直角度偏差 (弧度)：正值表示目標在上方，負值表示在下方  
float32 distance   # 目標距離 (公尺)：機器人中心到目標物體的直線距離
