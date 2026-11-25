#include "pid.h"
#include <Arduino.h>

// 通用限幅宏：把 amt 约束到 [low, high]
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


/*
 * 构造函数
 *   P/I/D  : 三个增益，后续可通过公共成员直接修改（热调参）
 *   ramp   : 输出变化速率上限（单位/秒），<= 0 表示不做斜率限制
 *   limit  : 输出与积分项共用的对称限幅（±limit）
 * 历史量全部清零，时间戳取当前 micros()，避免首拍 Ts 被算成超大值。
 */
PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)    // PID控制器加速度限幅
    , limit(limit)         // PID控制器输出限幅
    , error_prev(0.0f)
    , output_prev(0.0f)
    , integral_prev(0.0f)
{
    timestamp_prev = micros();
}

// PID 控制器函数：入参为本次误差，返回限幅后的控制量
float PIDController::operator() (float error){
    // 计算两次循环中间的间隔时间
    unsigned long timestamp_now = micros();
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f;
    // Ts 异常（首拍、溢出或主循环被长时间阻塞）时用一个安全的默认周期
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    // P环：直接与误差成正比，决定响应快慢
    float proportional = P * error;
    // Tustin 散点积分（I环）：用梯形法近似积分，消除稳态误差
    float integral = integral_prev + I*Ts*0.5f*(error + error_prev);
    // 积分限幅，防止积分饱和（windup）导致超调与回不来
    integral = _constrain(integral, -limit, limit);
    // D环（微分环节）：误差变化率，抑制超调（本工程三组 PID 的 D 均为 0）
    float derivative = D*(error - error_prev)/Ts;

    // 将P,I,D三环的计算值加起来
    float output = proportional + integral + derivative;
    // 输出限幅：电压环限到 Vbus/2，电流环限到 12.6A 等
    output = _constrain(output, -limit, limit);

    if(output_ramp > 0){
        // 对PID的变化速率进行限制：输出每秒变化不超过 output_ramp
        float output_rate = (output - output_prev)/Ts;
        if (output_rate > output_ramp)
            output = output_prev + output_ramp*Ts;      // 上升过快，按斜率走
        else if (output_rate < -output_ramp)
            output = output_prev - output_ramp*Ts;      // 下降过快，按斜率走
    }
    // 保存值（为了下一次循环）
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    return output;
}