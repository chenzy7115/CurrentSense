#include "lowpass_filter.h"

/*
 * 构造函数
 *   time_constant : 滤波时间常数 Tf(s)，越大越平滑、相位滞后越大
 * y_prev 初值为 0，时间戳取当前 micros()。
 */
LowPassFilter::LowPassFilter(float time_constant)
    : Tf(time_constant)
    , y_prev(0.0f)
{
    timestamp_prev = micros();
}


// 输入一个新采样值，返回一阶低通滤波后的值
float LowPassFilter::operator() (float x)
{
    unsigned long timestamp = micros();
    // 本次与上次调用的间隔（秒）
    float dt = (timestamp - timestamp_prev)*1e-6f;

    if (dt < 0.0f ) dt = 1e-3f;        // 时间回绕/异常，用默认周期
    else if(dt > 0.3f) {
        // 间隔过长（例如串口阻塞、断点），历史值已无参考意义，直接用新值重置
        y_prev = x;
        timestamp_prev = timestamp;
        return x;
    }

    // 离散化系数：Tf 越大，alpha 越接近 1，输出越"黏"着历史值，滤波更强
    float alpha = Tf/(Tf + dt);
    float y = alpha*y_prev + (1.0f - alpha)*x;
    y_prev = y;
    timestamp_prev = timestamp;
    return y;
}