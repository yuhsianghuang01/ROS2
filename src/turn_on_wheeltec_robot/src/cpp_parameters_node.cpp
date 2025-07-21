#include <memory>                      // 引入智慧指標相關功能
#include <string.h>                    // 引入字串處理函式庫
#include <string>                      // 引入C++字串類別
#include "rclcpp/rclcpp.hpp"           // 引入ROS2 C++客戶端核心功能
#include "geometry_msgs/msg/twist.hpp" // 引入幾何訊息的Twist類型，用於速度控制
using std::placeholders::_1;           // 佔位符，用於回調函式參數綁定

/**************************************
功能: 最小訂閱者類別，用於接收和處理速度指令
程式邏輯：
1. 繼承自rclcpp::Node建立ROS2節點
2. 建立topic訂閱者接收geometry_msgs::Twist訊息
3. 在回調函式中處理接收到的速度指令
用途: 作為測試節點或簡單的速度指令處理器
***************************************/
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber") // 建構函式：初始化節點名稱為"minimal_subscriber"
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(      // 建立Twist類型的話題訂閱者
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1)); // 訂閱"topic"話題，佇列大小10，綁定回調函式
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux) // 話題回調函式，接收速度指令
  {
    RCLCPP_INFO(this->get_logger(), "not akm"); // 記錄資訊：表示接收到非阿克曼類型的速度指令
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_; // 話題訂閱者的智慧指標
};

/**************************************
功能: 主函式，程式進入點
程式流程：
1. 初始化ROS2環境
2. 建立MinimalSubscriber節點實例
3. 啟動節點運行（進入事件迴圈）
4. 程式結束時清理資源
***************************************/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);                            // 初始化ROS2環境，處理命令列參數
  rclcpp::spin(std::make_shared<MinimalSubscriber>()); // 建立節點實例並進入事件迴圈，等待話題訊息
  rclcpp::shutdown();                                  // 關閉ROS2環境，清理資源
  return 0;                                            // 程式正常結束
}
