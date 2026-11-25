#include "AS5600.h"
#include "Wire.h"
#include <Arduino.h>

#define _2PI 6.28318530718f

// AS5600 相关
//---------------------------------------------------------------------------
// 读取 AS5600 的 RAW ANGLE 寄存器（0x0C 高字节 / 0x0D 低字节），
// 该寄存器为 12 位有效数据，右对齐，因此只取高字节的低 4 位 + 低字节 8 位。
// 返回值：机械角度，单位 rad，范围 [0, 2π)
//---------------------------------------------------------------------------
double Sensor_AS5600::getSensorAngle() {
  uint8_t angle_reg_msb = 0x0C;   // RAW ANGLE 高字节寄存器地址

  byte readArray[2];              // 存放读回的两个字节
  uint16_t readValue = 0;         // 12 位原始角度值

  // 发起一次 I2C 读：先写寄存器地址，再重复起始读 2 字节
  wire->beginTransmission(0x36);
  wire->write(angle_reg_msb);
  wire->endTransmission(false);

  wire->requestFrom(0x36, (uint8_t)2);
  for (byte i = 0; i < 2; i++) {
    readArray[i] = wire->read();
  }

  // ---- 按 12 位分辨率取出有效位 ----
  int _bit_resolution = 12;       // AS5600 角度分辨率为 12 位
  int _bits_used_msb = 11 - 7;    // 高字节中实际有效的位数 = 4
  float cpr = pow(2, _bit_resolution);          // 每圈计数 = 4096
  int lsb_used = _bit_resolution - _bits_used_msb;  // 低字节贡献的位数 = 8

  uint8_t lsb_mask = (uint8_t)((2 << lsb_used) - 1);        // 0xFF
  uint8_t msb_mask = (uint8_t)((2 << _bits_used_msb) - 1);  // 0x0F

  readValue = (readArray[1] & lsb_mask);                    // 低 8 位
  readValue += ((readArray[0] & msb_mask) << lsb_used);      // 高 4 位左移 8 位拼上

  // 计数 -> 弧度
  return (readValue / (float)cpr) * _2PI;
}

//AS5600 相关

//=========角度处理相关=============
// 构造函数：仅记录电机编号，硬件初始化在 Sensor_init() 中完成
Sensor_AS5600::Sensor_AS5600(int Mot_Num) {
  _Mot_Num = Mot_Num;  //使得 Mot_Num 可以统一在该文件调用
}

// 初始化：绑定 I2C 总线，读几次角度建立速度/角度基准
void Sensor_AS5600::Sensor_init(TwoWire* _wire) {
  wire = _wire;
  // 若外部已通过 begin(sda, scl, freq) 完成初始化（例如 DFOC_Vbus），
  // 这里重复 begin() 只会打印告警并直接返回，不会改动引脚，因此保留以兼容单独使用场景
  wire->begin();   //电机Sensor
  delay(500);      // 等待 AS5600 上电稳定，跳过前几次不可靠读数

  // 连续读 4 次，丢弃最早的一次，为角度/速度计算建立基准时间戳
  getSensorAngle();
  delayMicroseconds(1);
  vel_angle_prev = getSensorAngle();   // 速度计算的起始角度
  vel_angle_prev_ts = micros();        // 速度计算的起始时间
  delay(1);
  getSensorAngle();
  delayMicroseconds(1);
  angle_prev = getSensorAngle();       // 角度计算的起始角度
  angle_prev_ts = micros();            // 角度计算的起始时间
}

// 每个控制周期调用一次：更新单圈角度与累计圈数
void Sensor_AS5600::Sensor_update() {
  float val = getSensorAngle();
  angle_prev_ts = micros();
  float d_angle = val - angle_prev;
  // 圈数检测：单步跳变超过 0.8 圈，说明跨过了 0/2π 边界
  // d_angle 为正说明是反向越过（0 -> 2π），圈数 -1；为负说明正向越过，圈数 +1
  if (abs(d_angle) > (0.8f * _2PI)) full_rotations += (d_angle > 0) ? -1 : 1;
  angle_prev = val;
}

// 当前单圈机械角度(rad)，直接返回最近一次采样值
float Sensor_AS5600::getMechanicalAngle() {
  return angle_prev;
}

// 多圈累计机械角度(rad)：圈数 × 2π + 单圈角度
float Sensor_AS5600::getAngle() {
  return (float)full_rotations * _2PI + angle_prev;
}

// 机械角速度(rad/s)：用与上次调用之间的角度增量除以时间增量
// 注意：调用频率决定了速度的更新率，本工程在每个 loop 中通过 DFOC_M0_Velocity() 间接调用
float Sensor_AS5600::getVelocity() {
  // 计算采样时间（秒）
  float Ts = (angle_prev_ts - vel_angle_prev_ts) * 1e-6;
  // 快速修复奇怪的情况（微溢出）
  if (Ts <= 0) Ts = 1e-3f;
  // 速度计算：圈数差 × 2π + 角度差，再除以时间
  float vel = (((float)(full_rotations - vel_full_rotations) * _2PI + (angle_prev - vel_angle_prev))) / Ts;
  // 保存变量以待将来使用
  vel_angle_prev = angle_prev;
  vel_full_rotations = full_rotations;
  vel_angle_prev_ts = angle_prev_ts;
  return vel;
}