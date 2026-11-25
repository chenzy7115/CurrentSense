#ifndef PID_H
#define PID_H

/*
 * 通用 PID 控制器（位置式 + Tustin 离散积分 + 输出斜率限幅）
 * ---------------------------------------------------------------------------
 * 特性：
 *   - 用 micros() 自动测量采样周期 Ts，无需外部传入
 *   - 积分项与输出项都限幅在 ±limit
 *   - output_ramp 限制输出的变化速率，避免电压矢量突变冲击电机
 * 用法：PIDController current_loop(1.2f, 0, 0, 100000, 12.6f);
 *       float u = current_loop(目标值 - 实测值);
 * ---------------------------------------------------------------------------
 */
class PIDController
{
public:
    // P/I/D 为增益，ramp 为输出斜率上限(单位/秒)，limit 为输出与积分的双向限幅
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;

    // 重载 () 直接调用：入参为误差，返回控制器输出
    float operator() (float error);

    float P;              //!< 比例增益(P环增益)
    float I;              //!< 积分增益（I环增益）
    float D;              //!< 微分增益（D环增益）
    float output_ramp;    //!< 输出变化速率上限
    float limit;          //!< 输出限幅（对称 ±limit）
protected:
    float error_prev;             //!< 最后的跟踪误差值
    float output_prev;            //!< 最后一个 pid 输出值
    float integral_prev;          //!< 最后一个积分分量值
    unsigned long timestamp_prev; //!< 上次执行时间戳(us)
};

#endif