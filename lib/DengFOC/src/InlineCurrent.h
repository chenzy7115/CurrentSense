#ifndef INLINE_CURRENT_H
#define INLINE_CURRENT_H

#include <Arduino.h>

/*
 * 在线（串联采样电阻 + 运放）电流检测
 * ---------------------------------------------------------------------------
 * 硬件参数（DengFOC V4 板载）：
 *   - 采样电阻 0.01 Ω
 *   - 运放增益 50 倍
 *   - 电流 = (ADC 电压 - 零漂电压) * (1 / R_shunt / gain) = 差值 * 2 A/V
 * 引脚分配：
 *   - 0 号电机：A 相 = GPIO39，B 相 = GPIO36（均为 ADC1，输入专用脚）
 *   - 1 号电机：A 相 = GPIO35，B 相 = GPIO34
 *   - C 相为可选项，本工程未接（pinC = PIN_NOT_SET）
 * ---------------------------------------------------------------------------
 */
class CurrSense
{
  public:
    // Mot_Num 选择引脚组（0 或 1），同时写入分流电阻与运放增益
    CurrSense(int Mot_Num);

    // 读取指定引脚的一次 ADC 电压(V)
    float readADCVoltageInline(const int pinA);

    // 把 A/B/C 三相引脚配置为输入（C 相未使用时自动跳过）
    void configureADCInline(const int pinA,const int pinB, const int pinC);

    // 零漂校准：连续采样 1000 次求均值，作为"零电流"基准电压
    void calibrateOffsets();

    // 对外初始化入口：配置 ADC 引脚 + 零漂校准
    void init();

    // 刷新三相电流瞬时值(A)，需在每个控制周期调用
    void getPhaseCurrents();

    float current_a,current_b,current_c;  // 三相电流瞬时值(A)
    int pinA;                             // A 相 ADC 引脚
    int pinB;                             // B 相 ADC 引脚
    int pinC;                             // C 相 ADC 引脚，未使用时为 PIN_NOT_SET
    float offset_ia;                      // A 相零漂电压(V)
    float offset_ib;                      // B 相零漂电压(V)
    float offset_ic;                      // C 相零漂电压(V)
    float _shunt_resistor;                // 分流电阻(Ω)
    float amp_gain;                       // 运放增益

    float volts_to_amps_ratio;            // 电压->电流换算比(A/V)

    float gain_a;                         // A 相换算增益
    float gain_b;                         // B 相换算增益
    float gain_c;                         // C 相换算增益
  private:
    int _Mot_Num;                         // 电机编号
};

#endif