//DengFOC V0.3 —— Lesson8 电流力矩闭环（移植自 Arduino 工程 DengFOC_Lib_Lesson8_Current_Sense）
//灯哥开源，遵循GNU协议，转载请著名版权！
//GNU开源协议（GNU General Public License, GPL）是一种自由软件许可协议，保障用户能够自由地使用、研究、分享和修改软件。
//该协议的主要特点是，要求任何修改或衍生的作品必须以相同的方式公开发布，即必须开源。此外，该协议也要求在使用或分发软件时，必须保留版权信息和许可协议。GNU开源协议是自由软件基金会（FSF）制定和维护的一种协议，常用于GNU计划的软件和其他自由软件中。

#include <Arduino.h>
#include "DengFOC.h"

//================= 用户可调参数 =================
int Sensor_DIR = 1;  // 编码器计数方向：1 = 与电机相序同向，-1 = 反向。
                      // 若上电校准后电机原地抖动或反转，把该值取反即可
int Motor_PP = 7;     // 电机极对数（PP, Pole Pairs）：常见 2208/2804 云台电机为 7，
                      // 填错会导致电角度换算错误、电机发热或不转

//================= 初始化 =================
void setup() {
  Serial.begin(115200);        // 与上位机/串口助手的波特率保持一致（platformio.ini 中 monitor_speed 已同步）

  pinMode(12, OUTPUT);         // GPIO12 为 DengFOC V4 驱动板的电机使能脚
  digitalWrite(12, HIGH);      // 拉高使能功率级，开始允许输出 PWM

  DFOC_Vbus(12.6);             // 设定驱动器供电电压(V)：决定 PWM 占空比与实际相电压的换算关系
                               // 同时完成 PWM(30kHz/8bit)、AS5600(I2C 400kHz) 与电流采样校准的初始化

  DFOC_alignSensor(Motor_PP, Sensor_DIR);  // 电角度对齐：给固定力矩把转子吸到已知位置，
                                           // 记录零电角度 zero_electric_angle，供后续 FOC 换相使用
}

int count = 0;  // 分频计数：数值越大，向上位机回传电流的频率越低

//================= 主循环 =================
void loop()
{
  runFOC();                                  // 刷新传感器数据：读取 AS5600 机械角度 + 采样 A/B 相电流
  DFOC_M0_SET_CURRENT_PID(5, 200, 0, 100000);  // 设置电流环 PID：P=5, I=200, D=0, 输出斜坡=100000
  DFOC_M0_setTorque(serial_motor_target());  // 力矩闭环：目标值取自串口下发的 Iq，闭环后输出电压矢量

  count++;
  if (count > 30)                            // 每约 31 个循环打印一次实测 Iq，避免串口刷屏拖慢控制环
  {
    count = 0;
    Serial.printf("%f\n", DFOC_M0_Current());  // 回传滤波后的 q 轴电流(A)，可直接用串口绘图器观察
  }

  // 接收串口指令（非阻塞：有数据才解析，不阻塞 FOC 主循环）
  serialReceiveUserCommand();
}