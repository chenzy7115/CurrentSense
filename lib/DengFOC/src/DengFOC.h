//灯哥开源，遵循GNU协议，转载请著名版权！
//GNU开源协议（GNU General Public License, GPL）是一种自由软件许可协议，保障用户能够自由地使用、研究、分享和修改软件。
//该协议的主要特点是，要求任何修改或衍生的作品必须以相同的方式公开发布，即必须开源。此外，该协议也要求在使用或分发软件时，必须保留版权信息和许可协议。GNU开源协议是自由软件基金会（FSF）制定和维护的一种协议，常用于GNU计划的软件和其他自由软件中。
//仅在DengFOC官方硬件上测试过，欢迎硬件购买/支持作者，淘宝搜索店铺：灯哥开源
//你的支持将是接下来做视频和持续开源的经费，灯哥在这里先谢谢大家了
//函数声明

#ifndef DENG_FOC_H
#define DENG_FOC_H

#include <Arduino.h>

/*
 * DengFOC V0.3 对外接口总览
 * ---------------------------------------------------------------------------
 * 使用顺序（与 main.cpp 一致）：
 *   1. DFOC_Vbus(V)          设定供电电压，并初始化 PWM / 编码器 / 电流采样
 *   2. DFOC_alignSensor(PP,DIR)  电角度对齐，得到 zero_electric_angle
 *   3. loop 中：runFOC() 刷新传感器 -> 调用某一个闭环接口输出力矩
 * ---------------------------------------------------------------------------
 */

//========================= 底层：SVPWM 输出 =========================
// 把 A/B/C 三相桥臂电压(V)换算成占空比并写入三路 PWM。
// 入参为相电压绝对值，内部按 voltage_power_supply 归一化后限幅到 0~1。
void setPwm(float Ua, float Ub, float Uc);

// 给定 q 轴电压 Uq 与电角度 angle_el，执行 逆帕克 + 逆克拉克 变换后输出 SVPWM。
// Uq 被限幅在 ±voltage_power_supply/2；angle_el 会先归一化到 [0,2π]。
// 注意：原 DengFOC 头文件误写为 float 返回值，实际实现无返回值，此处已统一为 void。
void setTorque(float Uq, float angle_el);

// 把任意角度归一化到 [0, 2π)，用于电角度与目标角度的环绕处理。
float _normalizeAngle(float angle);

//========================= 初始化 =========================
// 设定供电电压并完成外设初始化：
//   - 3 路 LEDC PWM（30kHz / 8bit）
//   - AS5600 编码器（I2C：SDA=19, SCL=18, 400kHz）
//   - 电流采样零漂校准（1000 次均值，约 1s，期间电机不可转动）
void DFOC_Vbus(float power_supply);

// 电角度对齐：先给一个固定力矩把转子锁定到已知电角度，记录零电角度后松劲。
// 该步骤会把转子吸住约 1s，属于正常现象。
void DFOC_alignSensor(int _PP, int _DIR);

// 由机械角度换算当前电角度：_normalizeAngle(DIR * PP * θ_mech - zero_electric_angle)
float _electricalAngle();

//========================= 串口通信 =========================
// 读取串口下发的力矩目标（以 '\n' 为一帧结束符），解析失败不会改动原目标值。
float serial_motor_target();

// 非阻塞解析串口：把收到的字符累积到静态缓冲区，遇到 '\n' 就解析成新的力矩目标。
String serialReceiveUserCommand();

//========================= 传感器读取 =========================
// 机械角速度(rad/s)，带一阶低通滤波（Tf = 10ms），已乘上方向 DIR。
float DFOC_M0_Velocity();

// 机械角度(rad)，含圈数累计，已乘上方向 DIR。
float DFOC_M0_Angle();

// q 轴电流(A)：由 A/B 两相采样值经克拉克 + 帕克变换求得，带低通滤波（Tf = 50ms）。
float DFOC_M0_Current();

//========================= PID 参数设置 =========================
// 三组 PID 只修改 P/I/D 与输出斜坡；limit（输出限幅）在初始化时确定，不在此处修改。
void DFOC_M0_SET_ANGLE_PID(float P, float I, float D, float ramp);
void DFOC_M0_SET_VEL_PID(float P, float I, float D, float ramp);
void DFOC_M0_SET_CURRENT_PID(float P, float I, float D, float ramp);

// PID 运算接口，入参为误差值，返回控制器输出。
float DFOC_M0_VEL_PID(float error);
float DFOC_M0_ANGLE_PID(float error);

//========================= 闭环接口 =========================
// 位置闭环（角度环 -> 速度环 串级）
void DFOC_M0_set_Velocity_Angle(float Target);

// 速度闭环（注意：目标值取自串口变量，入参 Target 未被使用，保持与原例程一致）
void DFOC_M0_setVelocity(float Target);

// 力位闭环（角度环单独输出力矩电压）
void DFOC_M0_set_Force_Angle(float Target);

// 力矩闭环：Iq 目标 -> 电流环 PID -> 输出电压矢量，是本章（Lesson8）的核心接口。
void DFOC_M0_setTorque(float Target);

//========================= 实时任务 =========================
// 必须放在 loop 中最先调用：更新 AS5600 角度/圈数/速度，并采样两相电流。
void runFOC();

#endif