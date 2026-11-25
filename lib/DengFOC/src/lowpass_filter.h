#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include <Arduino.h>

/**
  * 一阶低通滤波器（RC 型离散实现）
  * ---------------------------------------------------------------------------
  * 传递函数近似 1 / (1 + Tf·s)，用 micros() 实时测得 dt 后按
  *   alpha = Tf / (Tf + dt)
  *   y     = alpha·y_prev + (1 - alpha)·x
  * 计算输出，因此采样周期变化时滤波特性依然稳定。
  * Tf 越大越平滑、相位滞后越大。
  * ---------------------------------------------------------------------------
  */

class LowPassFilter
{
public:
    /**
     * @Tf - 低通滤波时间常数
     */
    LowPassFilter(float Tf);
    ~LowPassFilter() = default;

    // 重载 () 直接调用：输入新采样值，返回滤波后的值
    float operator() (float x);
    float Tf; //!< 低通滤波时间常数

protected:
    unsigned long timestamp_prev;  //!< 最后执行时间戳(us)
    float y_prev;                  //!< 上一个循环中的过滤后的值
};

#endif