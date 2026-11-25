#include <Arduino.h>
#include "AS5600.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "InlineCurrent.h"   //引入在线电流检测
#include "DengFOC.h"

// 通用限幅宏：把 amt 约束到 [low, high]
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//================= 全局状态变量 =================
float voltage_power_supply;                  // 驱动器供电电压(V)，由 DFOC_Vbus() 设定
float Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0;         // 克拉克/帕克变换的中间量与三相输出电压
#define _3PI_2 4.71238898038f                // 3π/2（电角度对齐时使用的固定角度）
float zero_electric_angle=0;                 // 零电角度偏移，由 DFOC_alignSensor() 测得
int PP=1,DIR=1;                              // PP：极对数；DIR：编码器方向（在 DFOC_alignSensor 中赋值）
int pwmA = 32;                               // A 相 PWM 引脚
int pwmB = 33;                               // B 相 PWM 引脚
int pwmC = 25;                               // C 相 PWM 引脚

//低通滤波初始化
LowPassFilter M0_Vel_Flt = LowPassFilter(0.01); // Tf = 10ms   //M0速度环
LowPassFilter M0_Curr_Flt = LowPassFilter(0.05); // Tf = 5ms   //M0电流环
//PID
//注意：原工程使用 `PIDController{.P=..., .ramp=...}` 指定初始化器，该写法与本类
//（有用户构造函数、成员名为 output_ramp）不兼容，此处改为按构造函数参数初始化。
//速度环的 limit 依赖供电电压，在 DFOC_Vbus() 中再更新为 voltage_power_supply/2
PIDController vel_loop_M0(2, 0, 0, 100000, 12.6f / 2);    // 速度环：P=2，限幅 ±Vbus/2（单位 rad/s 误差->电压）
PIDController angle_loop_M0(2, 0, 0, 100000, 100);        // 角度环：P=2，限幅 ±100（单位 度 误差->速度）
PIDController current_loop_M0(1.2, 0, 0, 100000, 12.6);   // 电流环：P=1.2，限幅 ±12.6（单位 A 误差->电压）
//AS5600
Sensor_AS5600 S0=Sensor_AS5600(0);           // 0 号电机编码器
TwoWire S0_I2C = TwoWire(0);                 // 使用 I2C bus 0（引脚 19/18）

#define _1_SQRT3 0.57735026919f              // 1/√3，用于克拉克变换
#define _2_SQRT3 1.15470053838f              // 2/√3，用于克拉克变换

//初始化电流闭环
CurrSense CS_M0= CurrSense(0);               // 0 号电机电流采样（A=39, B=36）



//=================PID 设置函数=================
// 说明：这里只改增益与输出斜率，不改 limit（输出限幅在对象构造/DFOC_Vbus 中确定）
//速度PID
void DFOC_M0_SET_VEL_PID(float P,float I,float D,float ramp)   //M0角度环PID设置
{
  vel_loop_M0.P=P;
  vel_loop_M0.I=I;
  vel_loop_M0.D=D;
  vel_loop_M0.output_ramp=ramp;
}
//角度PID
void DFOC_M0_SET_ANGLE_PID(float P,float I,float D,float ramp)   //M0角度环PID设置
{
  angle_loop_M0.P=P;
  angle_loop_M0.I=I;
  angle_loop_M0.D=D;
  angle_loop_M0.output_ramp=ramp;
}
void DFOC_M0_SET_CURRENT_PID(float P,float I,float D,float ramp)    //M0电流环PID设置
{
  current_loop_M0.P=P;
  current_loop_M0.I=I;
  current_loop_M0.D=D;
  current_loop_M0.output_ramp=ramp;
}


//M0速度PID接口
float DFOC_M0_VEL_PID(float error)   //M0速度环
{
   return vel_loop_M0(error);

}
//M0角度PID接口
float DFOC_M0_ANGLE_PID(float error)
{
  return angle_loop_M0(error);
}


//初始变量及函数定义
//宏定义实现的一个约束函数,用于限制一个值的范围。
//具体来说，该宏定义的名称为 _constrain，接受三个参数 amt、low 和 high，分别表示要限制的值、最小值和最大值。该宏定义的实现使用了三元运算符，根据 amt 是否小于 low 或大于 high，返回其中的最大或最小值，或者返回原值。
//换句话说，如果 amt 小于 low，则返回 low；如果 amt 大于 high，则返回 high；否则返回 amt。这样，_constrain(amt, low, high) 就会将 amt 约束在 [low, high] 的范围内。


// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle){
  float a = fmod(angle, 2*PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI);
  //三目运算符。格式：condition ? expr1 : expr2
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}


// 设置PWM到控制器输出
// 入参为三相桥臂的目标电压(V)，内部按 Vbus 归一化成占空比后写入 LEDC
void setPwm(float Ua, float Ub, float Uc) {
  // 限制上限
  Ua = _constrain(Ua, 0.0f, voltage_power_supply);
  Ub = _constrain(Ub, 0.0f, voltage_power_supply);
  Uc = _constrain(Uc, 0.0f, voltage_power_supply);
  // 计算占空比
  // 限制占空比从0到1
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  //写入PWM到三路通道（Arduino-ESP32 3.x：ledcWrite 直接以引脚号为参数）
  //8 位分辨率，占空比 0~1 对应 0~255
  ledcWrite(pwmA, (uint32_t)(dc_a*255));
  ledcWrite(pwmB, (uint32_t)(dc_b*255));
  ledcWrite(pwmC, (uint32_t)(dc_c*255));
}

// 核心输出函数：给定 q 轴电压 Uq 与电角度 angle_el，输出对应的三相 PWM
// 注意 d 轴电压 Ud 固定为 0（表贴式电机的最优选择，不弱磁）
void setTorque(float Uq,float angle_el) {
  // Uq 限幅：不得超过供电电压的一半，否则无法合成完整的电压圆
  Uq=_constrain(Uq,-(voltage_power_supply)/2,(voltage_power_supply)/2);
  float Ud=0;
  (void)Ud;   // d 轴电压暂固定为 0（表贴式电机），此处显式忽略以消除未使用变量告警
  angle_el = _normalizeAngle(angle_el);
  // 帕克逆变换：把旋转坐标系(d,q)的电压还原到静止坐标系(α,β)
  Ualpha =  -Uq*sin(angle_el);
  Ubeta =   Uq*cos(angle_el);

  // 克拉克逆变换：由(α,β)得到三相电压，再叠加 Vbus/2 的直流偏置
  //（因为桥臂只能输出 0~Vbus，用中心点偏置来实现正负电压）
  Ua = Ualpha + voltage_power_supply/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
  setPwm(Ua,Ub,Uc);
}

// 初始化：供电电压 + PWM + 编码器 + 电流采样
void DFOC_Vbus(float power_supply)
{
  voltage_power_supply=power_supply;     // 保存供电电压，供占空比换算与限幅使用

  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  //Arduino-ESP32 3.x：ledcAttach 一次性完成 通道绑定 + 频率 + 分辨率（8 位 -> 0~255）
  //30kHz 超出人耳听觉范围，也便于降低电流纹波
  ledcAttach(pwmA, 30000, 8);
  ledcAttach(pwmB, 30000, 8);
  ledcAttach(pwmC, 30000, 8);
  Serial.println("完成PWM初始化设置");

  //AS5600
  S0_I2C.begin(19,18, 400000UL);         // SDA=19, SCL=18, 400kHz 快速模式
  S0.Sensor_init(&S0_I2C);   //初始化编码器0
  Serial.println("编码器加载完毕");

  //PID 加载
  vel_loop_M0.limit = voltage_power_supply/2;   // 速度环输出是电压，因此限幅跟随 Vbus
  //初始化电流传感器（内部含 1000 次零漂校准，需约 1s，此时电机必须静止）
  CS_M0.init();
 }


// 由机械角度计算电角度：电角度 = 极对数 × 机械角度 × 方向 - 零电角度偏移
float _electricalAngle(){
  return  _normalizeAngle((float)(DIR *  PP) * S0.getMechanicalAngle()-zero_electric_angle);
}


// 电角度对齐（上电必做）
// 思路：给一个固定在 3π/2 电角度的电压矢量，转子会被吸到与该矢量对齐的位置；
// 此时读出机械角度换算出的电角度，即为零电角度偏移量。
void DFOC_alignSensor(int _PP,int _DIR)
{
  PP=_PP;
  DIR=_DIR;
  setTorque(3, _3PI_2);  //起劲：3V 的电压矢量把转子吸到 3π/2 电角度处
  delay(1000);           // 等待转子稳定（会听到/看到电机被"锁定"）
  S0.Sensor_update();  //更新角度，方便下面电角度读取
  zero_electric_angle=_electricalAngle();   // 记录零电角度
  setTorque(0, _3PI_2);  //松劲（解除校准）
  Serial.print("0电角度：");Serial.println(zero_electric_angle);
}

// 机械角度(rad)，乘上方向后对应用户视角的正方向
float DFOC_M0_Angle()
{
  return DIR*S0.getAngle();
}


//=========================电流读取=========================

//通过Ia,Ib,Ic计算Iq,Id(目前仅输出Iq)
// 克拉克变换：Iα = Ia，Iβ = (Ia + 2Ib)/√3（三相无中线，Ic = -Ia-Ib）
// 帕克变换：把(α,β)投影到随转子旋转的(q)轴，得到真正产生力矩的电流分量
float cal_Iq_Id(float current_a,float current_b,float angle_el)
{
  float I_alpha=current_a;
  float I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

  float ct = cos(angle_el);
  float st = sin(angle_el);
  //float I_d = I_alpha * ct + I_beta * st;   // d 轴电流（本工程暂不使用）
  float I_q = I_beta * ct - I_alpha * st;
  return I_q;
}

// q 轴电流(A)：采样 -> 坐标变换 -> 低通滤波
// 必须先调用 runFOC() 刷新 CS_M0.current_a/b，否则读到的是上一拍数据
float DFOC_M0_Current()
{
  float I_q_M0_ori=cal_Iq_Id(CS_M0.current_a,CS_M0.current_b,_electricalAngle());
  float I_q_M0_flit=M0_Curr_Flt(I_q_M0_ori);
  return I_q_M0_flit;
}

//=========================电流读取=========================
//有滤波
float DFOC_M0_Velocity()
{
  //获取速度数据并滤波
  float vel_M0_ori=S0.getVelocity();
  float vel_M0_flit=M0_Vel_Flt(DIR*vel_M0_ori);
  return vel_M0_flit;   //考虑方向
}

//==============串口接收==============
// 协议：上位机发送 ASCII 数字 + '\n'（例如 "1.5\n" 表示目标 Iq = 1.5A）
float motor_target;    // 当前力矩目标（电流环的给定值）
int commaPosition;
String serialReceiveUserCommand() {

  // a string to hold incoming data
  static String received_chars;   // 静态缓冲区，跨调用累积未收完的字符

  String command = "";

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;

    // end of user input
    if (inChar == '\n') {

      // execute the user command
      command = received_chars;

      commaPosition = command.indexOf('\n');//检测字符串中的换行符
      if(commaPosition != -1)//如果有换行符存在就向下执行
      {
          motor_target = command.substring(0,commaPosition).toDouble();            //电机力矩目标值
          Serial.println(motor_target);                                          // 回显，便于确认下发成功
      }
      // reset the command buffer
      received_chars = "";
    }
  }
  return command;
}

// 供闭环接口读取当前力矩目标
float serial_motor_target()
{
  return motor_target;
}



//================简易接口函数================
// 位置闭环：角度环输出速度目标，速度环再输出力矩电压
void DFOC_M0_set_Velocity_Angle(float Target)
{
 setTorque(DFOC_M0_VEL_PID(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI)),_electricalAngle());   //角度闭环
}

// 速度闭环：目标值取自串口（入参 Target 未使用，与原例程保持一致）
void DFOC_M0_setVelocity(float Target)
{
  setTorque(DFOC_M0_VEL_PID((serial_motor_target()-DFOC_M0_Velocity())*180/PI),_electricalAngle());   //速度闭环
}

// 力位闭环：角度误差经角度 PID 直接输出电压
void DFOC_M0_set_Force_Angle(float Target)   //力位
{
  setTorque(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI),_electricalAngle());
}


// 力矩闭环（本课核心）：Iq 给定与实测的误差经电流环 PID 输出电压矢量
void DFOC_M0_setTorque(float Target)
{
  setTorque(current_loop_M0(Target-DFOC_M0_Current()),_electricalAngle());
}


// 实时任务：每个控制周期刷新传感器数据
// 必须最先调用，保证本拍闭环使用的是最新角度与电流
void runFOC()
{
  //====传感器更新====
  S0.Sensor_update();          // 读取机械角度、累计圈数、更新速度计算基准
  CS_M0.getPhaseCurrents();    // 采样 A/B 相电流并扣除零漂

  //====传感器更新====
}