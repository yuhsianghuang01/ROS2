/*
【技術背景知識】
1. ROS2 (Robot Operating System 2):
   - 機器人作業系統第二版，是分散式的機器人軟體框架
   - 採用 DDS (Data Distribution Service) 作為通訊中介軟體
   - 支援即時系統和多平台部署

2. 發布者-訂閱者模式 (Publisher-Subscriber Pattern):
   - ROS2 的核心通訊機制
   - 發布者 (Publisher) 發送資料到特定話題 (Topic)
   - 訂閱者 (Subscriber) 從話題接收資料
   - 解耦合設計：發布者和訂閱者不需直接了解對方

3. 節點 (Node):
   - ROS2 系統中的基本執行單元
   - 每個節點都是獨立的程序，可執行特定功能
   - 節點間透過話題、服務、動作進行通訊

4. 智慧指標 (Smart Pointer):
   - C++11 引入的記憶體管理機制
   - 自動管理記憶體配置和釋放，避免記憶體洩漏
   - shared_ptr: 多個指標可共享同一物件的所有權
*/

#include <memory>                      // 引入智慧指標相關功能 - 用於自動記憶體管理
#include <string.h>                    // 引入C語言字串處理函式庫 - 提供 strlen, strcpy 等函式
#include <string>                      // 引入C++字串類別 - 提供 std::string 類別及其方法
#include "rclcpp/rclcpp.hpp"           // 引入ROS2 C++客戶端核心功能 - 提供節點、發布者、訂閱者等基礎類別
#include "geometry_msgs/msg/twist.hpp" // 引入幾何訊息的Twist類型 - 用於機器人速度控制 (線速度+角速度)
using std::placeholders::_1;           // 佔位符，用於回調函式參數綁定 - std::bind() 的參數佔位符

/**************************************
【類別設計概念】
功能: 最小訂閱者類別，用於接收和處理速度指令
設計模式: 觀察者模式 (Observer Pattern) 的實作

【程式邏輯架構】：
1. 繼承自rclcpp::Node建立ROS2節點 - 獲得節點的基礎功能
2. 在建構函式中建立topic訂閱者接收geometry_msgs::Twist訊息
3. 透過回調函式機制處理接收到的速度指令

【應用場景】:
- 作為測試節點驗證通訊是否正常
- 簡單的速度指令監聽器
- 機器人運動控制的前端介面

【Twist訊息格式說明】:
geometry_msgs::Twist 包含：
- linear:  線性速度 (x: 前後, y: 左右, z: 上下)
- angular: 角速度 (x: 翻滚, y: 俯仰, z: 偏航)
對於地面機器人，通常只使用 linear.x (前進/後退) 和 angular.z (左轉/右轉)
***************************************/
class MinimalSubscriber : public rclcpp::Node // 繼承 ROS2 節點基礎類別，獲得節點通訊能力
{
public:
  /*
  【建構函式設計說明】
  - 初始化列表語法: 在函式體執行前先初始化基底類別
  - Node("minimal_subscriber"): 呼叫父類別建構函式，設定節點名稱
  - 節點名稱在 ROS2 系統中必須唯一，用於節點識別和通訊路由
  */
  MinimalSubscriber()
      : Node("minimal_subscriber") // 建構函式：初始化節點名稱為"minimal_subscriber"，此名稱在ROS2網路中作為節點識別ID
  {
    /*
    【訂閱者建立詳細說明】
    create_subscription<訊息類型>() 是 ROS2 的範本函式：
    - 參數1: "topic" - 要訂閱的話題名稱，必須與發布者發布的話題名稱一致
    - 參數2: 10 - QoS (Quality of Service) 佇列大小，決定能暫存多少未處理的訊息
    - 參數3: std::bind() - 將成員函式綁定為回調函式

    【std::bind 詳細解析】:
    std::bind(&MinimalSubscriber::topic_callback, this, _1)
    - &MinimalSubscriber::topic_callback: 成員函式指標
    - this: 當前物件指標，指定哪個物件的成員函式被呼叫
    - _1: 佔位符，代表回調函式的第一個參數(接收到的訊息)
    */
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(      // 建立Twist類型的話題訂閱者，用於接收機器人速度控制指令
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1)); // 訂閱"topic"話題，佇列大小10，綁定回調函式處理接收到的訊息
  }

private:
  /*
  【回調函式機制說明】
  - 事件驅動程式設計模式的實作
  - 當有新訊息到達時，ROS2會自動呼叫此函式
  - 參數是智慧指標，自動管理記憶體，無需手動釋放

  【SharedPtr 說明】:
  - std::shared_ptr 的別名，用於共享所有權
  - 多個指標可同時指向同一個物件
  - 當最後一個指標被銷毀時，物件自動釋放

  【實際應用場景】:
  在真實機器人系統中，這裡可能會：
  - 解析速度指令並控制馬達
  - 記錄運動軌跡
  - 安全檢查（速度限制、碰撞偵測）
  - 將指令轉發給硬體驅動程式
  */
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux) // 話題回調函式，接收速度指令並處理
  {
    /*
    【日誌系統說明】
    RCLCPP_INFO: ROS2 的日誌輸出巨集
    - 自動添加時間戳記和節點資訊
    - 支援不同日誌等級：DEBUG, INFO, WARN, ERROR, FATAL
    - 日誌會輸出到終端機和 ROS2 日誌系統

    【"not akm" 的含意】:
    akm 指的是 Ackermann（阿克曼轉向）
    - Ackermann steering: 前輪轉向的車輛轉向機構
    - 此訊息表示接收到的是一般差動輪式機器人的速度指令
    - 而非阿克曼轉向車輛的轉向角度指令
    */
    RCLCPP_INFO(this->get_logger(), "not akm"); // 記錄資訊：表示接收到非阿克曼類型的速度指令，用於系統狀態追蹤

    /*
    【可擴展的處理邏輯】
    實際應用中，這裡可以添加：

    // 取得速度資料
    double linear_x = twist_aux->linear.x;    // 前進/後退速度 (m/s)
    double angular_z = twist_aux->angular.z;  // 左轉/右轉角速度 (rad/s)

    // 安全檢查
    if (abs(linear_x) > MAX_LINEAR_SPEED) {
        RCLCPP_WARN(this->get_logger(), "Linear speed exceeds limit!");
    }

    // 控制硬體
    send_to_motor_controller(linear_x, angular_z);
    */
  }

  /*
  【成員變數說明】
  SharedPtr: std::shared_ptr 的縮寫
  - 智慧指標類型，自動管理物件生命週期
  - 多個指標可共享同一物件的所有權
  - 引用計數歸零時自動釋放記憶體

  【訂閱者物件生命週期】:
  - 建構函式中建立訂閱者
  - 在物件存活期間持續監聽話題
  - 解構函式自動清理資源（無需手動釋放）
  */
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_; // 話題訂閱者的智慧指標，負責管理與 ROS2 話題的連接和訊息接收
};

/**************************************
【主函式設計架構】
功能: 程式進入點，負責 ROS2 系統的完整生命週期管理

【ROS2 程式執行流程】：
1. rclcpp::init() - 初始化 ROS2 執行環境和通訊系統
2. 建立節點物件 - 實例化自訂的 ROS2 節點
3. rclcpp::spin() - 進入事件循環，等待和處理訊息
4. rclcpp::shutdown() - 清理 ROS2 資源和關閉通訊

【事件驅動架構說明】:
- ROS2 採用事件驅動的程式設計模式
- spin() 函式會阻塞主執行緒，持續監聽事件
- 當有訊息、服務呼叫或定時器事件時，會觸發對應的回調函式
- 直到收到終止信號（如 Ctrl+C）才會結束循環

【記憶體管理最佳實務】:
- 使用 std::make_shared 建立物件，確保記憶體安全
- 智慧指標自動管理物件生命週期
- 避免原始指標和手動記憶體管理的風險
***************************************/
int main(int argc, char *argv[])
{
  /*
  【ROS2 初始化詳細說明】
  rclcpp::init(argc, argv) 執行以下操作：
  - 解析命令列參數（如節點名稱、日誌等級、網路設定）
  - 初始化 DDS (Data Distribution Service) 通訊中介軟體
  - 設定訊號處理程式（處理 Ctrl+C 等終止信號）
  - 建立全域的 ROS2 執行環境上下文

  【常用命令列參數】:
  - __node:=<name>: 覆寫節點名稱
  - __log_level:=<level>: 設定日誌等級 (DEBUG, INFO, WARN, ERROR)
  - __params_file:=<file>: 載入參數檔案
  */
  rclcpp::init(argc, argv); // 初始化ROS2環境，處理命令列參數並設定通訊系統

  /*
  【節點生命週期管理】
  std::make_shared<MinimalSubscriber>():
  - 建立 MinimalSubscriber 物件的共享指標
  - 比直接使用 new 更安全，避免記憶體洩漏
  - 自動呼叫建構函式，建立訂閱者並開始監聽話題

  【spin() 函式運作機制】:
  - 進入無限迴圈，持續檢查事件佇列
  - 處理到達的訊息、服務請求、定時器事件
  - 呼叫對應的回調函式處理事件
  - 支援多執行緒執行（可使用 spin_some() 或 MultiThreadedExecutor）
  */
  rclcpp::spin(std::make_shared<MinimalSubscriber>()); // 建立節點實例並進入事件迴圈，持續監聽並處理話題訊息

  /*
  【資源清理說明】
  rclcpp::shutdown() 執行清理工作：
  - 關閉所有開啟的發布者和訂閱者
  - 釋放 DDS 通訊資源
  - 清理全域 ROS2 上下文
  - 確保程式優雅地終止

  【程式終止流程】:
  1. 用戶按下 Ctrl+C 或程式收到終止信號
  2. spin() 函式檢測到終止信號並退出迴圈
  3. 執行 shutdown() 清理資源
  4. 主函式返回，程式正常結束
  */
  rclcpp::shutdown(); // 關閉ROS2環境，清理通訊資源和全域狀態
  return 0;           // 程式正常結束，返回成功狀態碼給作業系統
}

/*
【補充：輪趣機器人系統架構概述】

本程式是輪趣(Wheeltec)機器人控制系統的一部分，該系統採用分層架構：

1. 【硬體抽象層】:
   - STM32 微控制器：負責底層馬達控制、感測器資料採集
   - MPU6050 IMU：提供三軸加速度計和陀螺儀資料
   - 編碼器：提供輪子轉速回饋，用於里程計算
   - 串口通訊：115200 baud，使用自定義協議與上位機通訊

2. 【ROS2 驅動層】:
   - wheeltec_robot_node：主控制節點，處理串口通訊和感測器融合
   - 本程式 (cpp_parameters_node)：作為測試或擴展節點
   - 話題系統：/cmd_vel (速度控制), /odom (里程計), /imu (慣性資料)

3. 【應用層】:
   - 導航系統：路徑規劃、避障、定位
   - 機器人控制：遙控、自主巡航、跟隨功能

【相關技術深入說明】

▼ ROS2 vs ROS1 主要差異：
- 通訊架構：ROS1 基於 TCP/IP + XML-RPC，ROS2 基於 DDS
- 即時性：ROS2 支援硬即時系統，ROS1 僅支援軟即時
- 安全性：ROS2 內建加密和認證機制
- 跨平台：ROS2 原生支援 Windows、macOS、嵌入式系統

▼ DDS (Data Distribution Service) 詳解：
- 工業級分散式通訊中介軟體
- 支援多種 QoS 策略：可靠性、持久性、延遲控制
- 自動探索機制：節點間無需中央名稱伺服器
- 內建冗餘和容錯能力

▼ 機器人運動學模型：
1. 差動輪式 (Differential Drive)：
   - 兩個獨立驅動輪 + 萬向輪/腳輪
   - 控制參數：linear.x (前進速度), angular.z (旋轉角速度)
   - 優點：結構簡單、成本低、可原地旋轉
   - 應用：清掃機器人、服務機器人

2. 阿克曼轉向 (Ackermann Steering)：
   - 前輪轉向，後輪驅動（類似汽車）
   - 控制參數：speed (速度), steering_angle (轉向角)
   - 優點：高速穩定、輪胎磨損小
   - 應用：無人車、大型運輸機器人

▼ 四元數姿態表示法：
- 避免歐拉角的萬向節鎖 (Gimbal Lock) 問題
- 數學表示：q = w + xi + yj + zk
- 旋轉運算效率高，插值平滑
- 在機器人學中用於表示 3D 旋轉

【開發與除錯建議】

▼ 常用 ROS2 指令：
```bash
# 列出所有節點
ros2 node list

# 查看節點資訊
ros2 node info /minimal_subscriber

# 列出所有話題
ros2 topic list

# 監聽話題內容
ros2 topic echo /topic

# 發布測試訊息
ros2 topic pub /topic geometry_msgs/msg/Twist "
linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"

# 查看話題頻率
ros2 topic hz /topic

# 查看 TF 變換樹
ros2 run tf2_tools view_frames.py
```

▼ 程式碼最佳實務：
1. 使用智慧指標管理記憶體
2. 適當設定 QoS 參數（佇列大小、可靠性等）
3. 添加參數驗證和錯誤處理
4. 使用 ROS2 日誌系統而非 printf/cout
5. 遵循 ROS2 命名規範（蛇底式命名）

▼ 效能最佳化：
1. 選擇合適的執行器：
   - SingleThreadedExecutor：單執行緒，適用簡單應用
   - MultiThreadedExecutor：多執行緒，適用高負載系統
   - StaticSingleThreadedExecutor：靜態分配，減少記憶體碎片

2. QoS 調整：
   - Best Effort vs Reliable：可靠性與效能權衡
   - Volatile vs Transient Local：是否保存歷史資料
   - 佇列深度：根據處理能力和延遲需求調整

【故障排除指南】

▼ 常見問題：
1. 節點無法啟動：
   - 檢查 ROS2 環境是否正確載入（source setup.bash）
   - 確認相依套件是否安裝完成

2. 話題通訊異常：
   - 使用 ros2 topic list 檢查話題是否存在
   - 檢查 QoS 相容性（發布者與訂閱者的 QoS 設定）

3. 高延遲問題：
   - 調整 DDS 廠商設定（RTI, Fast-DDS, Cyclone DDS）
   - 最佳化網路設定和防火牆規則

4. 記憶體洩漏：
   - 使用 valgrind 進行記憶體檢測
   - 確保智慧指標正確使用，避免循環參照

【延伸學習資源】

▼ 建議學習順序：
1. C++11/14 現代特性（智慧指標、Lambda、範本程式設計）
2. ROS2 基礎概念（節點、話題、服務、動作）
3. 機器人運動學和動力學
4. 控制理論（PID、狀態空間、卡爾曼濾波）
5. 即時系統和嵌入式程式設計

▼ 相關標準和協議：
- REP-105：ROS 座標系統約定
- REP-103：ROS 度量單位約定
- DDS-RTPS：即時發布訂閱協議
- IEEE 754：浮點數標準
- CAN bus：車用網路協議

此程式雖然簡單，但體現了 ROS2 系統的核心設計理念，
是理解複雜機器人系統的重要基礎。
*/
