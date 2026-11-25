#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include "Wire.h"

/*
 * AS5600 磁编码器驱动（12 位，I2C 地址 0x36）
 * ---------------------------------------------------------------------------
 * 角度数据来自寄存器 0x0C/0x0D（RAW ANGLE），每圈 4096 个计数。
 * 内部维护：
 *   - angle_prev      : 最近一次的角度（0~2π），单位 rad
 *   - full_rotations  : 累计圈数（带符号），用于得到多圈绝对角度
 *   - vel_* 系列变量  : 计算角速度所需的上一拍快照
 * ---------------------------------------------------------------------------
 */
class Sensor_AS5600
{
  public:
    // Mot_Num 仅用于标识电机编号；本工程只用 0 号电机
    Sensor_AS5600(int Mot_Num);

    // 绑定 I2C 总线并做首次读数，初始化角度基准与时间戳。
    // 若外部已调用过 wire->begin(sda, scl, freq)，这里的 begin() 只会告警而不会改引脚。
    void Sensor_init(TwoWire* _wire = &Wire);

    // 读取一次原始角度并更新圈数，需在每个控制周期调用
    void Sensor_update();

    // 多圈累计机械角度(rad) = full_rotations * 2π + 当前单圈角度
    float getAngle();

    // 机械角速度(rad/s)，基于两次 getVelocity 之间的角度差与时间差
    float getVelocity();

    // 当前单圈机械角度(rad)，范围 [0, 2π)
    float getMechanicalAngle();

    // 直接读取 12 位原始角度并换算为 [0, 2π)，不修改内部状态
    double getSensorAngle();

  private:
    int _Mot_Num;                 // 电机编号（预留多电机扩展）
    //AS5600 变量定义
    //int sensor_direction=1;     //编码器旋转方向定义
    float angle_prev = 0;         // 最后一次调用 getSensorAngle() 的输出结果，用于得到完整的圈数和速度
    long angle_prev_ts = 0;       // 上次调用 getAngle 的时间戳(us)
    float vel_angle_prev = 0;     // 最后一次调用 getVelocity 时的角度
    long vel_angle_prev_ts = 0;   // 最后速度计算时间戳(us)
    int32_t full_rotations = 0;   // 总圈数计数
    int32_t vel_full_rotations = 0; //用于速度计算的先前完整旋转圈数
    TwoWire* wire = &Wire;        // 实际使用的 I2C 总线指针
};

#endif