
#include "turn_on_wheeltec_robot/Quaternion_Solution.h" // 引入四元數解算相關的標頭檔
#define SAMPLING_FREQ 20.0f                             // 定義採樣頻率為20Hz，用於積分計算 // 采样频率
/**************************************
Date: May 31, 2020
Function: 平方根倒數 求四元數用到
功能: 快速平方根倒數計算，用於向量單位化
演算法說明: 使用牛頓-拉夫遜法的快速近似算法
數學原理: 計算 1/√x，避免除法運算提高效率
***************************************/
float InvSqrt(float number)
{
  volatile long i;               // 用於位元操作的長整數變數
  volatile float x, y;           // 中間計算變數
  volatile const float f = 1.5F; // 牛頓-拉夫遜法的常數
  x = number * 0.5F;             // 取輸入數值的一半
  y = number;                    // 暫存原始數值
  i = *((long *)&y);             // 將浮點數按位元解釋為整數
  i = 0x5f375a86 - (i >> 1);     // 神奇數字算法，快速估算倒數平方根
  y = *((float *)&i);            // 將整數按位元解釋為浮點數
  y = y * (f - (x * y * y));     // 牛頓-拉夫遜法迭代一次提高精度

  return y; // 返回 1/√number 的近似值
}
/**************************************
Date: May 31, 2020
Function: 四元數解算
功能: 使用IMU數據進行四元數姿態解算
演算法: Madgwick AHRS 濾波器算法
輸入參數: gx,gy,gz - 三軸陀螺儀數據(rad/s)
         ax,ay,az - 三軸加速度計數據(m/s²)
數學原理:
1. 利用加速度計測量重力向量校正陀螺儀誤差
2. 通過互補濾波融合陀螺儀和加速度計數據
3. 使用四元數表示法避免萬向節鎖問題
***************************************/
volatile float twoKp = 1.0f;                                               // 比例增益的兩倍 (Kp×2)，控制收斂速度
volatile float twoKi = 0.0f;                                               // 積分增益的兩倍 (Ki×2)，消除穩態誤差
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                 // 四元數，表示感測器相對於輔助座標系的姿態
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // 積分誤差項，由Ki縮放用於消除漂移
void Quaternion_Solution(float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;              // 正規化倒數，用於向量單位化
  float halfvx, halfvy, halfvz; // 估計的重力方向向量的一半
  float halfex, halfey, halfez; // 誤差向量的一半
  float qa, qb, qc;             // 四元數的暫存變數
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  { // 只有當加速度計數據有效時才計算回饋（避免正規化時出現NaN）
    // 首先把加速度計採集到的值(三維向量)轉化為單位向量，即向量除以模
    recipNorm = InvSqrt(ax * ax + ay * ay + az * az); // 計算加速度向量模長的倒數
    ax *= recipNorm;                                  // 將加速度計數據正規化為單位向量
    ay *= recipNorm;                                  // Y軸加速度正規化
    az *= recipNorm;                                  // Z軸加速度正規化
    // 把四元數換算成方向餘弦中的第三行的三個元素（重力方向的估計值）
    halfvx = q1 * q3 - q0 * q2;        // 從四元數計算估計的重力方向向量X分量的一半
    halfvy = q0 * q1 + q2 * q3;        // 從四元數計算估計的重力方向向量Y分量的一半
    halfvz = q0 * q0 - 0.5f + q3 * q3; // 從四元數計算估計的重力方向向量Z分量的一半
    // 誤差是估計的重力方向和測量的重力方向的向量外積
    halfex = (ay * halfvz - az * halfvy); // 計算誤差向量X分量：測量值與估計值的向量外積
    halfey = (az * halfvx - ax * halfvz); // 計算誤差向量Y分量
    halfez = (ax * halfvy - ay * halfvx); // 計算誤差向量Z分量
    // 計算並應用積分回饋（如果啟用）
    if (twoKi > 0.0f)
    {                                                         // 如果積分增益大於0，則累積誤差積分
      integralFBx += twoKi * halfex * (1.0f / SAMPLING_FREQ); // X軸積分誤差，由Ki縮放 // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / SAMPLING_FREQ); // Y軸積分誤差累積
      integralFBz += twoKi * halfez * (1.0f / SAMPLING_FREQ); // Z軸積分誤差累積
      gx += integralFBx;                                      // 將積分回饋加到陀螺儀X軸數據 // apply integral feedback
      gy += integralFBy;                                      // 將積分回饋加到陀螺儀Y軸數據
      gz += integralFBz;                                      // 將積分回饋加到陀螺儀Z軸數據
    }
    else
    {
      integralFBx = 0.0f; // 防止積分飽和，重置X軸積分項 // prevent integral windup
      integralFBy = 0.0f; // 重置Y軸積分項
      integralFBz = 0.0f; // 重置Z軸積分項
    }
    // Apply proportional feedback
    gx += twoKp * halfex; // 應用比例回饋到陀螺儀X軸
    gy += twoKp * halfey; // 應用比例回饋到陀螺儀Y軸
    gz += twoKp * halfez; // 應用比例回饋到陀螺儀Z軸
  }
  // Integrate rate of change of quaternion
  // 積分四元數的變化率，更新四元數狀態
  gx *= (0.5f * (1.0f / SAMPLING_FREQ)); // 將陀螺儀數據預乘公共因子：0.5×採樣週期 // pre-multiply common factors
  gy *= (0.5f * (1.0f / SAMPLING_FREQ)); // Y軸陀螺儀數據預處理
  gz *= (0.5f * (1.0f / SAMPLING_FREQ)); // Z軸陀螺儀數據預處理
  qa = q0;                               // 暫存當前四元數q0分量
  qb = q1;                               // 暫存當前四元數q1分量
  qc = q2;                               // 暫存當前四元數q2分量
  q0 += (-qb * gx - qc * gy - q3 * gz);  // 更新四元數q0分量：使用四元數微分方程
  q1 += (qa * gx + qc * gz - q3 * gy);   // 更新四元數q1分量
  q2 += (qa * gy - qb * gz + q3 * gx);   // 更新四元數q2分量
  q3 += (qa * gz + qb * gy - qc * gx);   // 更新四元數q3分量
  // Normalise quaternion
  // 四元數正規化，確保其模長為1
  recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3); // 計算四元數模長的倒數
  q0 *= recipNorm;                                            // 正規化四元數q0分量
  q1 *= recipNorm;                                            // 正規化四元數q1分量
  q2 *= recipNorm;                                            // 正規化四元數q2分量
  q3 *= recipNorm;                                            // 正規化四元數q3分量
  Mpu6050.orientation.w = q0;                                 // 將計算結果存入全域MPU6050結構的四元數w分量
  Mpu6050.orientation.x = q1;                                 // 存入四元數x分量
  Mpu6050.orientation.y = q2;                                 // 存入四元數y分量
  Mpu6050.orientation.z = q3;                                 // 存入四元數z分量
}
