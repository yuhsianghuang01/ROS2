# WheelTec Robot Message Package 技術說明文件

## 概述
此套件 `wheeltec_robot_msg` 是一個 ROS2 自定義訊息包，用於定義銀河輪(WheelTec)機器人系統中使用的訊息類型。

## 核心技術概念

### 1. ROS2 訊息系統 (Message System)
ROS2 使用訊息傳遞來實現節點間的通訊。訊息是強型別的資料結構，定義了節點間交換資料的格式。

#### 訊息類型層級：
- **原始類型**: bool, int8, uint8, int16, uint16, int32, uint32, int64, uint64, float32, float64, string
- **陣列類型**: 固定長度陣列 `[N]` 或動態陣列 `[]`
- **巢狀類型**: 可包含其他訊息類型或標準訊息

### 2. ROS IDL (Interface Definition Language)
ROS2 使用 IDL 來定義介面，支援三種類型：
- **.msg 檔案**: 定義訊息結構 (用於 Topic 通訊)
- **.srv 檔案**: 定義服務請求/回應 (用於 Service 通訊)
- **.action 檔案**: 定義動作目標/結果/回饋 (用於 Action 通訊)

### 3. CMake 在 ROS2 中的角色
CMake 是 ROS2 的建置系統基礎，負責：
- 管理套件相依性
- 程式碼編譯和連結
- 安裝規則定義
- 測試框架整合

### 4. Ament 建置系統
Ament 是 ROS2 專用的建置系統，擴展了 CMake：
- **ament_cmake**: 提供 CMake 巨集和函數
- **ament_python**: 支援純 Python 套件
- **colcon**: 工作空間建置工具

## 當前訊息定義分析

### Data.msg 結構
```plaintext
float32 x  # X 軸座標 (公尺)
float32 y  # Y 軸座標 (公尺)  
float32 z  # Z 軸座標 (公尺)
```

這是一個簡單的 3D 位置訊息，常用於：
- 機器人位置回報
- 目標點設定
- 路徑規劃節點
- 感測器資料傳輸

### 生成的程式碼結構
執行 `colcon build` 後，系統會生成：

#### C++ 介面:
```cpp
// install/wheeltec_robot_msg/include/wheeltec_robot_msg/msg/data.hpp
namespace wheeltec_robot_msg {
namespace msg {
struct Data {
  float x;
  float y; 
  float z;
};
}} // namespace
```

#### Python 介面:
```python
# install/wheeltec_robot_msg/lib/python3.x/site-packages/wheeltec_robot_msg/msg/_data.py
class Data:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
```

## 建置流程詳解

### 1. 程式碼生成階段
```bash
# rosidl_generate_interfaces() 觸發以下步驟：
# 1. 解析 .msg 檔案
# 2. 驗證訊息定義語法
# 3. 生成多語言介面程式碼
# 4. 建立型別支援函數 (序列化/反序列化)
```

### 2. 編譯階段
```bash
# CMake 編譯流程：
# 1. 處理相依性 (find_package)
# 2. 設定編譯選項
# 3. 編譯生成的程式碼
# 4. 連結必要的函式庫
```

### 3. 安裝階段
```bash
# ament_package() 執行：
# 1. 安裝標頭檔和函式庫
# 2. 設定 CMake 查找檔案
# 3. 匯出套件資訊
# 4. 註冊到 ROS2 環境
```

## 使用範例

### 發布者 (Publisher) - C++
```cpp
#include "rclcpp/rclcpp.hpp"
#include "wheeltec_robot_msg/msg/data.hpp"

class DataPublisher : public rclcpp::Node {
public:
    DataPublisher() : Node("data_publisher") {
        publisher_ = this->create_publisher<wheeltec_robot_msg::msg::Data>("robot_position", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DataPublisher::publish_data, this)
        );
    }

private:
    void publish_data() {
        auto message = wheeltec_robot_msg::msg::Data();
        message.x = 1.0;
        message.y = 2.0;
        message.z = 0.5;
        publisher_->publish(message);
    }

    rclcpp::Publisher<wheeltec_robot_msg::msg::Data>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

### 訂閱者 (Subscriber) - Python
```python
import rclpy
from rclpy.node import Node
from wheeltec_robot_msg.msg import Data

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')
        self.subscription = self.create_subscription(
            Data,
            'robot_position',
            self.data_callback,
            10
        )

    def data_callback(self, msg):
        self.get_logger().info(f'接收位置: x={msg.x}, y={msg.y}, z={msg.z}')

def main():
    rclpy.init()
    node = DataSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
```

## 進階概念

### 1. 訊息版本控制
ROS2 支援訊息版本相容性：
- 新增欄位 (向後相容)
- 修改欄位類型 (需版本升級)
- 刪除欄位 (可能造成不相容)

### 2. 序列化機制
ROS2 使用 CDR (Common Data Representation) 進行訊息序列化：
- 跨平台相容性
- 高效能編碼/解碼
- 網路傳輸最佳化

### 3. QoS (Quality of Service)
ROS2 提供 QoS 設定來控制訊息傳遞行為：
- **可靠性**: RELIABLE vs BEST_EFFORT
- **持久性**: VOLATILE vs TRANSIENT_LOCAL
- **歷史**: KEEP_LAST vs KEEP_ALL

## 相關工具和指令

### 建置相關
```bash
# 建置特定套件
colcon build --packages-select wheeltec_robot_msg

# 清除建置快取
colcon build --cmake-clean-cache

# 僅建置測試
colcon test --packages-select wheeltec_robot_msg
```

### 訊息檢查
```bash
# 查看訊息定義
ros2 interface show wheeltec_robot_msg/msg/Data

# 列出所有訊息類型
ros2 interface list -m

# 測試訊息發布
ros2 topic pub /test_topic wheeltec_robot_msg/msg/Data "{x: 1.0, y: 2.0, z: 3.0}"
```

### 除錯工具
```bash
# 監聽訊息
ros2 topic echo /robot_position

# 檢查 Topic 資訊
ros2 topic info /robot_position

# 查看 Topic 頻率
ros2 topic hz /robot_position
```

## 最佳實務建議

1. **命名規範**: 使用描述性的訊息名稱和欄位名稱
2. **文件化**: 在 .msg 檔案中添加註解說明
3. **版本控制**: 謹慎修改現有訊息結構
4. **測試**: 編寫單元測試驗證訊息功能
5. **效能**: 避免過於複雜的巢狀結構

## 故障排除

### 常見問題
1. **找不到訊息類型**: 確認已正確建置和 source 工作空間
2. **編譯錯誤**: 檢查 CMakeLists.txt 中的相依性設定
3. **執行時錯誤**: 驗證 QoS 設定是否匹配

### 除錯步驟
1. 檢查建置輸出是否有錯誤
2. 確認套件已正確安裝
3. 使用 `ros2 interface` 指令驗證訊息可用性
4. 檢查網路和 DDS 設定

這個技術說明涵蓋了 ROS2 訊息系統的核心概念，有助於深入理解 wheeltec_robot_msg 套件的設計和使用。
