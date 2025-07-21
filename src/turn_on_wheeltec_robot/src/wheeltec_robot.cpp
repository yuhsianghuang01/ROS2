#include "t/**************************************
Date : January 28, 2021 Function : The main function, ROS initialization, creates the Robot_control object through the Turn_on_robot class and automatically calls the constructor initialization 功能 : 主函數，ROS初始化，通過turn_on_robot類創建Robot_control對象並自動調用構造函數初始化 程式流程說明： 1. 初始化 ROS2 環境 2. 建立 turn_on_robot 物件（自動呼叫建構函式） 3. 進入主要控制迴圈 4. 程式結束時釋放資源 *************************************** /

                                                                                                                                                                                                         int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv); // 初始化 ROS2 環境，設定命令列參數
  // auto node= std::make_shared<turn_on_robot>();

  turn_on_robot Robot_Control; // 建立輪趣機器人控制物件，自動呼叫建構函式進行初始化
  Robot_Control.Control();     // 呼叫主要控制函式，開始機器人控制迴圈
  return 0;                    // 程式正常結束
}
_robot / wheeltec_robot.h "        // 引入輪趣機器人控制類的標頭檔
#include "rclcpp/rclcpp.hpp"                              // 引入 ROS2 C++ 客戶端函式庫的核心功能
#include "turn_on_wheeltec_robot/Quaternion_Solution.h"   // 引入四元數解算函式庫，用於 IMU 姿態計算
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp" // 引入阿克曼轉向驅動訊息型別，支援阿克曼小車控制
#include "wheeltec_robot_msg/msg/data.hpp"                // 引入自訂義的輪趣機器人資料訊息型別

         // sensor_msgs::Imu Mpu6050;//Instantiate an IMU object //实例化IMU对象
         sensor_msgs::msg::Imu Mpu6050;        // 全域變數：MPU6050 IMU感測器的資料結構，用於儲存慣性測量單元資料
using std::placeholders::_1;                   // 佔位符，用於函式回調綁定中的第一個參數
using namespace std;                           // 使用標準命名空間，簡化程式碼書寫
rclcpp::Node::SharedPtr node_handle = nullptr; // ROS2 節點句柄的共享指標，用於節點管理
/**************************************
Date: January 28, 2021
Function: The main function, ROS initialization, creates the Robot_control object through the Turn_on_robot class and automatically calls the constructor initialization
功能: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  // auto node= std::make_shared<turn_on_robot>();

  turn_on_robot Robot_Control;
  Robot_Control.Control();
  return 0;
}
/**************************************
Date: January 28, 2021
Function: Data conversion function
功能: 數據轉換函數
概念說明：將兩個8位元組的數據合併成一個16位元組的短整數
用途：將串口接收的高低位元組數據重組為完整的IMU數據
***************************************/

short turn_on_robot::IMU_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  short transition_16;             // 宣告16位元短整數變數，用於儲存合併後的數據
  transition_16 = 0;               // 初始化數據為0，確保無垃圾數據干擾
  transition_16 |= Data_High << 8; // 將高位元組左移8位，放置在高8位
  transition_16 |= Data_Low;       // 將低位元組直接放置在低8位，形成完整的16位數據
  return transition_16;            // 返回合併後的16位IMU原始數據
}
float turn_on_robot::Odom_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  float data_return;                                                     // 宣告浮點數變數，用於儲存轉換後的速度數據
  short transition_16;                                                   // 宣告16位元短整數變數，用於中間數據處理
  transition_16 = 0;                                                     // 初始化數據為0
  transition_16 |= Data_High << 8;                                       // 將高位元組左移8位，放置在高8位 //获取数据的高8位
  transition_16 |= Data_Low;                                             // 將低位元組直接放置在低8位，形成完整的16位數據 //获取数据的低8位
  data_return = (transition_16 / 1000) + (transition_16 % 1000) * 0.001; // 將mm/s單位轉換為m/s：整數部分除以1000，餘數乘以0.001 // The speed unit is changed from mm/s to m/s //速度单位从mm/s转换为m/s
  return data_return;                                                    // 返回轉換後的速度數據（單位：m/s）
}

/**************************************
Date: January 28, 2021
Function: The speed topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
功能: 速度話題訂閱回調函數Callback，根據訂閱的指令通過串口發指令控制下位機
程式邏輯說明：
1. 接收阿克曼轉向驅動指令
2. 將速度和轉向角度轉換為串口通訊格式
3. 組裝數據封包並發送至下位機
前置概念：阿克曼轉向模型適用於前輪轉向的車輛，如汽車模型
***************************************/
void turn_on_robot::Akm_Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl)
{
  short transition; // 中間變數，用於數據類型轉換 //intermediate variable //中间变量
  // if(akm_cmd_vel=="ackermann_cmd") {RCLCPP_INFO(this->get_logger(),"is akm");} //Prompt message //提示信息
  Send_Data.tx[0] = FRAME_HEADER; // 設定封包帧頭為0x7B，標示數據封包開始 //frame head 0x7B //帧头0X7BAkm_Cmd_Vel_Sub
  Send_Data.tx[1] = 0;            // 預留位元組，供未來擴展使用 //set aside //预留位
  Send_Data.tx[2] = 0;            // 預留位元組，供未來擴展使用 //set aside //预留位

  // The target velocity of the X-axis of the robot
  // 機器人x軸的目標線速度（前進後退速度）
  transition = 0;                           // 重置中間變數
  transition = akm_ctl->drive.speed * 1000; // 將浮點數速度放大1000倍轉為整數，方便串口傳輸 //将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;             // 取速度數據的低8位元組 //取数据的低8位
  Send_Data.tx[3] = transition >> 8;        // 取速度數據的高8位元組（右移8位） //取数据的高8位

  // The target velocity of the Y-axis of the robot
  // 機器人y軸的目標線速度（阿克曼模型通常不使用y軸速度）
  // transition=0;
  // transition = twist_aux->linear.y*1000;
  // Send_Data.tx[6] = transition;
  // Send_Data.tx[5] = transition>>8;

  // The target angular velocity of the robot's Z axis
  // 機器人z軸的目標角速度（轉向角度）
  transition = 0;                                        // 重置中間變數
  transition = akm_ctl->drive.steering_angle * 1000 / 2; // 將轉向角度放大1000倍並除以2進行調整
  Send_Data.tx[8] = transition;                          // 取角速度數據的低8位元組
  Send_Data.tx[7] = transition >> 8;                     // 取角速度數據的高8位元組

  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK); // 計算並設定BBC校驗位，確保數據傳輸正確性 //For the BBC check bits, see the Check_Sum function //BBC校验位，规则参见Check_Sum函数
  Send_Data.tx[10] = FRAME_TAIL;                   // 設定封包帧尾為0x7D，標示數據封包結束 //frame tail 0x7D //帧尾0X7D

  try
  {
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx)); // Sends data to the downloader via serial port //通过串口向下位机发送数据
  }
  catch (serial::IOException &e)
  {
    RCLCPP_ERROR(this->get_logger(), ("Unable to send data through serial port")); // If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }
}

// void turn_on_robot::Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl)
/**************************************
Function: 一般差動輪式機器人速度控制回調函數
功能: 處理geometry_msgs::Twist類型的速度指令，適用於全向移動機器人
程式邏輯說明：
1. 接收三軸線速度和角速度指令
2. 將浮點數速度轉換為整數格式
3. 組裝串口數據封包發送至下位機控制器
前置概念：Twist訊息包含linear(x,y,z)和angular(x,y,z)六軸速度資訊
***************************************/
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
  short transition; // 中間變數，用於浮點數到整數的轉換 //intermediate variable //中间变量
  // if(akm_cmd_vel=="none") {RCLCPP_INFO(this->get_logger(),"not akm");} //Prompt message //提示信息
  Send_Data.tx[0] = FRAME_HEADER; // 設定封包帧頭為0x7B //frame head 0x7B //帧头0X7BAkm_Cmd_Vel_Sub
  Send_Data.tx[1] = 0;            // 預留位元組 //set aside //预留位
  Send_Data.tx[2] = 0;            // 預留位元組 //set aside //预留位

  // The target velocity of the X-axis of the robot
  // 機器人x軸的目標線速度（前進後退方向）
  transition = 0;                          // 重置轉換變數
  transition = twist_aux->linear.x * 1000; // 將x軸線速度(m/s)放大1000倍轉為mm/s整數格式 //将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;            // 儲存速度數據的低8位元組 //取数据的低8位
  Send_Data.tx[3] = transition >> 8;       // 儲存速度數據的高8位元組 //取数据的高8位

  // The target velocity of the Y-axis of the robot
  // 機器人y軸的目標線速度（左右平移方向，全向輪使用）
  transition = 0;                          // 重置轉換變數
  transition = twist_aux->linear.y * 1000; // 將y軸線速度轉換為整數格式
  Send_Data.tx[6] = transition;            // 儲存y軸速度的低8位元組
  Send_Data.tx[5] = transition >> 8;       // 儲存y軸速度的高8位元組

  // The target angular velocity of the robot's Z axis
  // 機器人z軸的目標角速度（自轉方向，左轉右轉）
  transition = 0;                           // 重置轉換變數
  transition = twist_aux->angular.z * 1000; // 將z軸角速度(rad/s)放大1000倍轉為整數格式
  Send_Data.tx[8] = transition;             // 儲存角速度數據的低8位元組
  Send_Data.tx[7] = transition >> 8;        // 儲存角速度數據的高8位元組

  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK); // 計算封包校驗碼確保數據完整性 //For the BBC check bits, see the Check_Sum function //BBC校验位，规则参见Check_Sum函数
  Send_Data.tx[10] = FRAME_TAIL;                   // 設定封包帧尾為0x7D //frame tail 0x7D //帧尾0X7D

  try
  {
    if (akm_cmd_vel == "none")
    {
      Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
    } // Sends data to the downloader via serial port //通过串口向下位机发送数据
  }
  catch (serial::IOException &e)
  {
    RCLCPP_ERROR(this->get_logger(), ("Unable to send data through serial port")); // If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }
}

/**************************************
Date: January 28, 2021
Function: Publish the IMU data topic
功能: 發布IMU慣性測量單元數據話題
程式邏輯說明：
1. 建立IMU資料結構並填入時間戳記和座標框架
2. 填入四元數姿態、角速度、線性加速度數據
3. 設定協方差矩陣表示數據不確定性
4. 發布IMU話題供導航系統使用
前置概念：IMU包含陀螺儀、加速度計，提供機器人姿態和運動狀態
***************************************/

void turn_on_robot::Publish_ImuSensor()
{
  sensor_msgs::msg::Imu Imu_Data_Pub;                                 // 建立IMU話題數據結構 //Instantiate IMU topic data //实例化IMU话题数据
  Imu_Data_Pub.header.stamp = rclcpp::Node::now();                    // 設定當前時間戳記，用於資料同步
  Imu_Data_Pub.header.frame_id = gyro_frame_id;                       // 設定IMU對應的TF座標框架ID，用於座標變換 //IMU corresponds to TF coordinates, which is required to use the robot_pose_ekf feature pack
                                                                      // IMU对应TF坐标，使用robot_pose_ekf功能包需要设置此项
  Imu_Data_Pub.orientation.x = Mpu6050.orientation.x;                 // 填入四元數姿態的x分量，表示三軸姿態 //A quaternion represents a three-axis attitude //四元数表达三轴姿态
  Imu_Data_Pub.orientation.y = Mpu6050.orientation.y;                 // 填入四元數姿態的y分量
  Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;                 // 填入四元數姿態的z分量
  Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;                 // 填入四元數姿態的w分量（實數部分）
  Imu_Data_Pub.orientation_covariance[0] = 1e6;                       // 設定三軸姿態協方差矩陣對角線元素，表示X軸姿態不確定性 //Three-axis attitude covariance matrix //三轴姿态协方差矩阵
  Imu_Data_Pub.orientation_covariance[4] = 1e6;                       // Y軸姿態協方差，數值越大表示不確定性越高
  Imu_Data_Pub.orientation_covariance[8] = 1e-6;                      // Z軸姿態協方差，較小值表示Z軸姿態較為準確
  Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x;       // 填入x軸角速度數據(rad/s) //Triaxial angular velocity //三轴角速度
  Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;       // 填入y軸角速度數據
  Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;       // 填入z軸角速度數據
  Imu_Data_Pub.angular_velocity_covariance[0] = 1e6;                  // 設定三軸角速度協方差矩陣 //Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
  Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;                  // Y軸角速度協方差
  Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;                 // Z軸角速度協方差
  Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x; // 填入x軸線性加速度數據(m/s²) //Triaxial acceleration //三轴线性加速度
  Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y; // 填入y軸線性加速度數據
  Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z; // 填入z軸線性加速度數據

  imu_publisher->publish(Imu_Data_Pub); // 發布IMU數據話題，供其他節點訂閱使用
}

/**************************************
Date: January 28, 2021
Function: Publish the odometer topic, Contains position, attitude, triaxial velocity, angular velocity about triaxial, TF parent-child coordinates, and covariance matrix
功能: 發布里程計話題，包含位置、姿態、三軸速度、繞三軸角速度、TF父子座標、協方差矩陣
程式邏輯說明：
1. 將Z軸旋轉角度轉換為四元數表示法
2. 建立里程計數據結構並填入位置、速度資訊
3. 發布機器人姿態和速度話題供導航使用
前置概念：里程計提供機器人在世界座標系中的位置和速度估計
***************************************/

void turn_on_robot::Publish_Odom()
{
  // Convert the Z-axis rotation Angle into a quaternion for expression
  // 將Z軸旋轉角度轉換為四元數進行表達

  tf2::Quaternion q;                                        // 建立TF2四元數物件用於姿態表示
  q.setRPY(0, 0, Robot_Pos.Z);                              // 設定Roll=0, Pitch=0, Yaw=Robot_Pos.Z的歐拉角轉四元數
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q); // 將TF2四元數轉換為ROS訊息格式

  wheeltec_robot_msg::msg::Data robotpose; // 建立自訂義的機器人位置資料結構
  wheeltec_robot_msg::msg::Data robotvel;  // 建立自訂義的機器人速度資料結構
  nav_msgs::msg::Odometry odom;            // 建立里程計話題數據結構 //Instance the odometer topic data //实例化里程计话题数据

  odom.header.stamp = rclcpp::Node::now(); // 設定當前時間戳記
  odom.header.frame_id = odom_frame_id;    // 設定里程計TF父座標框架（通常為"odom"） // Odometer TF parent coordinates //里程计TF父坐标
  odom.child_frame_id = robot_frame_id;    // 設定里程計TF子座標框架（通常為"base_link"） // Odometer TF subcoordinates //里程计TF子坐标

  odom.pose.pose.position.x = Robot_Pos.X; // 設定機器人在X軸方向的位置（公尺） //Position //位置
  odom.pose.pose.position.y = Robot_Pos.Y; // 設定機器人在Y軸方向的位置
  odom.pose.pose.position.z = Robot_Pos.Z; // 設定機器人在Z軸方向的位置
  odom.pose.pose.orientation = odom_quat;  // 設定機器人姿態，使用Z軸轉角轉換的四元數 //Posture, Quaternion converted by Z-axis rotation //姿态，通过Z轴转角转换的四元数

  odom.twist.twist.linear.x = Robot_Vel.X;  // 設定機器人X方向線速度（m/s） //Speed in the X direction //X方向速度
  odom.twist.twist.linear.y = Robot_Vel.Y;  // 設定機器人Y方向線速度 //Speed in the Y direction //Y方向速度
  odom.twist.twist.angular.z = Robot_Vel.Z; // 設定機器人繞Z軸的角速度（rad/s） //Angular velocity around the Z axis //绕Z轴角速度

  robotpose.x = Robot_Pos.X; // 填入自訂義位置結構的X座標
  robotpose.y = Robot_Pos.Y; // 填入自訂義位置結構的Y座標
  robotpose.z = Robot_Pos.Z; // 填入自訂義位置結構的Z座標

  robotvel.x = Robot_Vel.X; // 填入自訂義速度結構的X速度
  robotvel.y = Robot_Vel.Y; // 填入自訂義速度結構的Y速度
  robotvel.z = Robot_Vel.Z; // 填入自訂義速度結構的Z角速度

  /*   geometry_msgs::msg::TransformStamped odom_tf;

     odom_tf.header = odom.header;
     odom_tf.child_frame_id = odom.child_frame_id;
     odom_tf.header.stamp = rclcpp::Node::now();

     odom_tf.transform.translation.x = odom.pose.pose.position.x;
     odom_tf.transform.translation.y = odom.pose.pose.position.y;
     odom_tf.transform.translation.z = odom.pose.pose.position.z;
     odom_tf.transform.rotation = odom.pose.pose.orientation;

     tf_bro->sendTransform(odom_tf);

 */
  // There are two types of this matrix, which are used when the robot is at rest
  // and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
  // 這個矩陣有兩種，分別在機器人靜止和運動的時候使用。擴展卡爾曼濾波官方提供的2個矩陣，用於robot_pose_ekf功能包
  // tf_pub_->publish(odom_tf);
  odom_publisher->publish(odom);           // 發布標準里程計話題，供導航系統使用 //Pub odometer topic //发布里程计话题
  robotpose_publisher->publish(robotpose); // 發布自訂義機器人位置話題 //Pub odometer topic //发布里程计话题
  robotvel_publisher->publish(robotvel);   // 發布自訂義機器人速度話題 //Pub odometer topic //发布里程计话题
}

/**************************************
Date: January 28, 2021
Function: Publish voltage-related information
功能: 发布电压相关信息
***************************************/

void turn_on_robot::Publish_Voltage()
{
  std_msgs::msg::Float32 voltage_msgs; // Define the data type of the power supply voltage publishing topic //定义电源电压发布话题的数据类型
  static float Count_Voltage_Pub = 0;
  if (Count_Voltage_Pub++ > 10)
  {
    Count_Voltage_Pub = 0;
    voltage_msgs.data = Power_voltage;        // The power supply voltage is obtained //电源供电的电压获取
    voltage_publisher->publish(voltage_msgs); // Post the power supply voltage topic unit: V, volt //发布电源电压话题单位：V、伏特
  }
} /**************************************
 Date: January 28, 2021
 Function: Serial port communication check function, packet n has a byte, the NTH -1 byte is the check bit, the NTH byte bit frame end.Bit XOR results from byte 1 to byte n-2 are compared with byte n-1, which is a BBC check
 Input parameter: Count_Number: Check the first few bytes of the packet
 功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BBC校验
 输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
 ***************************************/

unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number, unsigned char mode)
{
  unsigned char check_sum = 0, k;

  if (mode == 0) // Receive data mode //接收数据模式
  {
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Receive_Data.rx[k]; // By bit or by bit //按位异或
    }
  }
  if (mode == 1) // Send data mode //发送数据模式
  {
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Send_Data.tx[k]; // By bit or by bit //按位异或
    }
  }
  return check_sum; // Returns the bitwise XOR result //返回按位异或结果
}

/**************************************
Date: January 28, 2021
Function: The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
功能: 通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
***************************************/

bool turn_on_robot::Get_Sensor_Data()
{
  short transition_16 = 0, j = 0, Header_Pos = 0, Tail_Pos = 0; // Intermediate variable //中间变量
  uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE] = {0};             // Temporary variable to save the data of the lower machine //临时变量，保存下位机数据
  Stm32_Serial.read(Receive_Data_Pr, sizeof(Receive_Data_Pr));  // Read the data sent by the lower computer through the serial port //通过串口读取下位机发送过来的数据
  // Record the position of the head and tail of the frame //记录帧头帧尾位置
  for (j = 0; j < 24; j++)
  {
    if (Receive_Data_Pr[j] == FRAME_HEADER)
      Header_Pos = j;
    else if (Receive_Data_Pr[j] == FRAME_TAIL)
      Tail_Pos = j;
  }

  if (Tail_Pos == (Header_Pos + 23))
  {
    // If the end of the frame is the last bit of the packet, copy the packet directly to receive_data.rx
    // 如果帧尾在数据包最后一位，直接复制数据包到Receive_Data.rx
    //  ROS_INFO("1----");
    memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
  }
  else if (Header_Pos == (1 + Tail_Pos))
  {
    // 如果帧头在帧尾后面，纠正数据位置后复制数据包到Receive_Data.rx
    //  If the header is behind the end of the frame, copy the packet to receive_data.rx after correcting the data location
    //  ROS_INFO("2----");
    for (j = 0; j < 24; j++)
      Receive_Data.rx[j] = Receive_Data_Pr[(j + Header_Pos) % 24];
  }
  else
  {
    // 其它情况则认为数据包有错误
    //  In other cases, the packet is considered to be faulty
    //  ROS_INFO("3----");
    return false;
  }

  Receive_Data.Frame_Header = Receive_Data.rx[0]; // The first part of the data is the frame header 0X7B //数据的第一位是帧头0X7B
  Receive_Data.Frame_Tail = Receive_Data.rx[23];  // The last bit of data is frame tail 0X7D //数据的最后一位是帧尾0X7D

  if (Receive_Data.Frame_Header == FRAME_HEADER) // Judge the frame header //判断帧头
  {
    if (Receive_Data.Frame_Tail == FRAME_TAIL) // Judge the end of the frame //判断帧尾
    {
      // BBC check passes or two packets are interlaced //BBC校验通过或者两组数据包交错
      if (Receive_Data.rx[22] == Check_Sum(22, READ_DATA_CHECK) || (Header_Pos == (1 + Tail_Pos)))
      {
        Receive_Data.Flag_Stop = Receive_Data.rx[1]; // set aside //预留位
        // Get the speed of the moving chassis in the X direction //获取运动底盘X方向速度
        Robot_Vel.X = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]);
        // Get the speed of the moving chassis in the Y direction, The Y speed is only valid in the omnidirectional mobile robot chassis
        Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]);
        // 获取运动底盘Y方向速度，Y速度仅在全向移动机器人底盘有效
        Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]); // Get the speed of the moving chassis in the Z direction //获取运动底盘Z方向速度

        // MPU6050 stands for IMU only and does not refer to a specific model. It can be either MPU6050 or MPU9250
        // Mpu6050仅代表IMU，不指代特定型号，既可以是MPU6050也可以是MPU9250
        Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8], Receive_Data.rx[9]);   // Get the X-axis acceleration of the IMU     //获取IMU的X轴加速度
        Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10], Receive_Data.rx[11]); // Get the Y-axis acceleration of the IMU     //获取IMU的Y轴加速度
        Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12], Receive_Data.rx[13]); // Get the Z-axis acceleration of the IMU     //获取IMU的Z轴加速度
        Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14], Receive_Data.rx[15]);  // Get the X-axis angular velocity of the IMU //获取IMU的X轴角速度
        Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16], Receive_Data.rx[17]);  // Get the Y-axis angular velocity of the IMU //获取IMU的Y轴角速度
        Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18], Receive_Data.rx[19]);  // Get the Z-axis angular velocity of the IMU //获取IMU的Z轴角速度
        // Linear acceleration unit conversion is related to the range of IMU initialization of STM32, where the range is ±2g=19.6m/s^2
        // 线性加速度单位转化，和STM32的IMU初始化的时候的量程有关,这里量程±2g=19.6m/s^2
        Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;
        // The gyroscope unit conversion is related to the range of STM32's IMU when initialized. Here, the range of IMU's gyroscope is ±500°/s
        // Because the robot generally has a slow Z-axis speed, reducing the range can improve the accuracy
        // 陀螺仪单位转化，和STM32的IMU初始化的时候的量程有关，这里IMU的陀螺仪的量程是±500°/s
        // 因为机器人一般Z轴速度不快，降低量程可以提高精度
        Mpu6050.angular_velocity.x = Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.y = Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.z = Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

        // Get the battery voltage
        // 获取电池电压
        transition_16 = 0;
        transition_16 |= Receive_Data.rx[20] << 8;
        transition_16 |= Receive_Data.rx[21];
        Power_voltage = transition_16 / 1000 + (transition_16 % 1000) * 0.001; // Unit conversion millivolt(mv)->volt(v) //单位转换毫伏(mv)->伏(v)

        return true;
      }
    }
  }
  return false;
}
/**************************************
Date: January 28, 2021
Function: Loop access to the lower computer data and issue topics
功能: 循环获取下位机数据与发布话题
***************************************/

void turn_on_robot::Control()
{

  rclcpp::Time current_time, last_time;
  current_time = rclcpp::Node::now();
  last_time = rclcpp::Node::now();
  while (rclcpp::ok())
  {
    current_time = rclcpp::Node::now();
    // Retrieves time interval, which is used to integrate velocity to obtain displacement (mileage)
    // 获取时间间隔，用于积分速度获得位移(里程)
    Sampling_Time = (current_time - last_time).seconds();

    // The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
    // 通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
    if (true == Get_Sensor_Data())

    {
      // Calculate the displacement in the X direction, unit: m //计算X方向的位移，单位：m
      Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
      // Calculate the displacement in the Y direction, unit: m //计算Y方向的位移，单位：m
      Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
      // The angular displacement about the Z axis, in rad //绕Z轴的角位移，单位：rad
      Robot_Pos.Z += Robot_Vel.Z * Sampling_Time;

      // Calculate the three-axis attitude from the IMU with the angular velocity around the three-axis and the three-axis acceleration
      // 通过IMU绕三轴角速度与三轴加速度计算三轴姿态
      Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,
                          Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);
      Publish_ImuSensor(); // Pub the IMU topic //发布IMU话题
      Publish_Voltage();   // Pub the topic of power supply voltage //发布电源电压话题
      Publish_Odom();

      rclcpp::spin_some(this->get_node_base_interface());
    }

    last_time = current_time; // Record the time and use it to calculate the time interval //记录时间，用于计算时间间隔
  }
}

/**************************************
Date: January 28, 2021
Function: Constructor, executed only once, for initialization
功能: 构造函数, 只执行一次，用于初始化
***************************************/
turn_on_robot::turn_on_robot()
    : rclcpp::Node("wheeltec_robot")
{
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data));
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  int serial_baud_rate = 115200;

  this->declare_parameter<int>("serial_baud_rate");
  this->declare_parameter<std::string>("usart_port_name", "/dev/ttyCH343USB0");
  this->declare_parameter<std::string>("cmd_vel", "cmd_vel");
  this->declare_parameter<std::string>("akm_cmd_vel", "ackermann_cmd");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("robot_frame_id", "base_link");
  this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");

  this->get_parameter("serial_baud_rate", serial_baud_rate);
  this->get_parameter("usart_port_name", usart_port_name);
  this->get_parameter("cmd_vel", cmd_vel);
  this->get_parameter("akm_cmd_vel", akm_cmd_vel);
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->get_parameter("robot_frame_id", robot_frame_id);
  this->get_parameter("gyro_frame_id", gyro_frame_id);

  odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  // odom_timer = create_wall_timer(1s/50, [=]() { Publish_Odom(); });

  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("mobile_base/sensors/imu_data", 10); // CHANGE
  // imu_timer = create_wall_timer(1s/100, [=]() { Publish_ImuSensor(); });

  voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);
  // voltage_timer = create_wall_timer(1s/100, [=]() { Publish_Voltage(); });
  // tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);
  robotpose_publisher = create_publisher<wheeltec_robot_msg::msg::Data>("robotpose", 10);
  // robotpose_timer = create_wall_timer(1s/50, [=]() { Publish_Odom(); });

  robotvel_publisher = create_publisher<wheeltec_robot_msg::msg::Data>("robotvel", 10);
  // robotvel_timer = create_wall_timer(1s/50, [=]() { Publish_Odom(); });
  tf_bro = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel, 2, std::bind(&turn_on_robot::Cmd_Vel_Callback, this, _1));

  Akm_Cmd_Vel_Sub = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      akm_cmd_vel, 2, std::bind(&turn_on_robot::Akm_Cmd_Vel_Callback, this, _1));

  try
  {
    // Attempts to initialize and open the serial port //尝试初始化与开启串口
    Stm32_Serial.setPort(usart_port_name);                        // Select the serial port number to enable //选择要开启的串口号
    Stm32_Serial.setBaudrate(serial_baud_rate);                   // Set the baud rate //设置波特率
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000); // Timeout //超时等待
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open(); // Open the serial port //开启串口
  }
  catch (serial::IOException &e)
  {
    RCLCPP_ERROR(this->get_logger(), "wheeltec_robot can not open serial port,Please check the serial port cable! "); // If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
  }
  if (Stm32_Serial.isOpen())
  {
    RCLCPP_INFO(this->get_logger(), "wheeltec_robot serial port opened"); // Serial port opened successfully //串口开启成功提示
  }
}

/**************************************
Date: January 28, 2021
Function: Destructor, executed only once and called by the system when an object ends its life cycle
功能: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/

turn_on_robot::~turn_on_robot()
{
  // Sends the stop motion command to the lower machine before the turn_on_robot object ends
  // 对象turn_on_robot结束前向下位机发送停止运动命令
  Send_Data.tx[0] = FRAME_HEADER;
  Send_Data.tx[1] = 0;
  Send_Data.tx[2] = 0;

  // The target velocity of the X-axis of the robot //机器人X轴的目标线速度
  Send_Data.tx[4] = 0;
  Send_Data.tx[3] = 0;

  // The target velocity of the Y-axis of the robot //机器人Y轴的目标线速度
  Send_Data.tx[6] = 0;
  Send_Data.tx[5] = 0;

  // The target velocity of the Z-axis of the robot //机器人Z轴的目标角速度
  Send_Data.tx[8] = 0;
  Send_Data.tx[7] = 0;
  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK); // Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
  Send_Data.tx[10] = FRAME_TAIL;

  try
  {
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx)); // Send data to the serial port //向串口发数据
  }
  catch (serial::IOException &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); // If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
  }
  Stm32_Serial.close();                             // Close the serial port //关闭串口
  RCLCPP_INFO(this->get_logger(), "Shutting down"); // Prompt message //提示信息
}
