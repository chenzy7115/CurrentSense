#include <Arduino.h>
#include "InlineCurrent.h"


//  - shunt_resistor  - 分流电阻值
//  - gain  - 电流检测运算放大器增益
//  - phA   - A 相 adc 引脚
//  - phB   - B 相 adc 引脚
//  - phC   - C 相 adc 引脚（可选）


#define _ADC_VOLTAGE 3.3f            //ADC 电压（满量程参考）
#define _ADC_RESOLUTION 4095.0f      //ADC 分辨率（ESP32 默认 12 位，0~4095）

// ADC 计数到电压转换比率求解：每个 ADC 计数对应的电压
#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )

// pinC 未使用时的哨兵值（原版此处未初始化，会参与比较，这里显式赋值）
#define PIN_NOT_SET (-1)
#define _isset(a) ( (a) != (PIN_NOT_SET) )

/*
 * 构造函数
 * 按电机编号选择采样引脚组，并写入该板卡的采样电阻与运放增益。
 * 同时把 pinC 置为"未使用"，避免后续 _isset() 读到随机值。
 */
CurrSense::CurrSense(int Mot_Num)
{
  // 默认无 C 相采样引脚
  pinC = PIN_NOT_SET;
  current_a = 0;
  current_b = 0;
  current_c = 0;
  offset_ia = 0;
  offset_ib = 0;
  offset_ic = 0;

  if(Mot_Num==0)
  {
    pinA = 39;          // A 相：GPIO39（ADC1_CH3，仅输入脚）
    pinB = 36;          // B 相：GPIO36（ADC1_CH0，仅输入脚）
    //int pinC;
    _shunt_resistor = 0.01;   // 板载采样电阻 0.01 Ω
    amp_gain  = 50;           // 电流检测运放增益 50 倍

    volts_to_amps_ratio = 1.0f /_shunt_resistor / amp_gain; // volts to amps（= 2 A/V）

    // gains for each phase
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
  }
  if(Mot_Num==1)
  {
    pinA = 35;          // 1 号电机 A 相：GPIO35
    pinB = 34;          // 1 号电机 B 相：GPIO34
    //int pinC;
    _shunt_resistor = 0.01;
    amp_gain  = 50;

    volts_to_amps_ratio = 1.0f /_shunt_resistor / amp_gain; // volts to amps

    // gains for each phase
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
  }
}

// 读取一次 ADC 原始值并换算成电压(V)
float CurrSense::readADCVoltageInline(const int pinA){
  uint32_t raw_adc = analogRead(pinA);
  return raw_adc * _ADC_CONV;
}

// 把采样引脚配置为输入；pinC 未使用时自动跳过，避免对非法引脚调用 pinMode
void CurrSense::configureADCInline(const int pinA,const int pinB, const int pinC){
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);
}

// 查找 ADC 零偏移量的函数
// 原理：电机静止且不输出电压时，运放输出应等于 VCC/2 的静态偏置，
// 由于电阻、运放、ADC 都有误差，实测这个偏置并保存为 offset，
// 之后每次采样减去它，才是真正代表电流的电压。
void CurrSense::calibrateOffsets(){
    const int calibration_rounds = 1000;   // 采样次数，越多越稳，耗时约 1s（含 delay(1)）

    // 查找0电流时候的电压
    offset_ia = 0;
    offset_ib = 0;
    offset_ic = 0;
    // 读数1000次
    for (int i = 0; i < calibration_rounds; i++) {
        offset_ia += readADCVoltageInline(pinA);
        offset_ib += readADCVoltageInline(pinB);
        if(_isset(pinC)) offset_ic += readADCVoltageInline(pinC);
        delay(1);   // 拉开采样间隔，避免读到同一拍噪声
    }
    // 求平均，得到误差
    offset_ia = offset_ia / calibration_rounds;
    offset_ib = offset_ib / calibration_rounds;
    if(_isset(pinC)) offset_ic = offset_ic / calibration_rounds;
}

// 初始化入口：先配置引脚，再做零漂校准
// 注意：校准期间电机必须静止且不输出，否则会把电流误当成零漂
void CurrSense::init(){
    // 配置函数
    configureADCInline(pinA,pinB,pinC);
    // 校准
    calibrateOffsets();
}


// 读取全部三相电流
// 公式：电流(A) = (实测电压 - 零漂电压) × 换算增益

void CurrSense::getPhaseCurrents(){
    current_a = (readADCVoltageInline(pinA) - offset_ia)*gain_a;// amps
    current_b = (readADCVoltageInline(pinB) - offset_ib)*gain_b;// amps
    current_c = (!_isset(pinC)) ? 0 : (readADCVoltageInline(pinC) - offset_ic)*gain_c; // amps
}