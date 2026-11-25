# ESP32 有感 FOC —— 电流（力矩）闭环

基于 **DengFOC V0.3**（灯哥开源 FOC 库，Lesson8 电流力矩闭环）移植到 **PlatformIO + Arduino-ESP32 3.x** 的工程。

通过 AS5600 磁编码器获取转子电角度，通过板载采样电阻 + 运放获取 A/B 相电流，
再经 Clarke/Park 变换得到 q 轴电流 Iq，与上位机下发的目标电流构成电流环闭环，
从而直接控制电机的输出力矩。

---

## 1. 目录结构

```
CurrentSense/
├── platformio.ini              PlatformIO 工程配置（lolin32_lite / arduino）
├── README.md                   本文件
├── src/
│   └── main.cpp                应用入口：初始化、电角度对齐、主循环
└── lib/
    └── DengFOC/
        ├── library.json
        └── src/
            ├── DengFOC.h/.cpp         FOC 主逻辑：SVPWM 输出、电角度、电流环、串口协议
            ├── AS5600.h/.cpp          AS5600 磁编码器驱动（I2C）
            ├── InlineCurrent.h/.cpp   在线电流采样（ADC + 零漂校准）
            ├── pid.h/.cpp             PID 控制器（Tustin 积分 + 斜率限幅）
            └── lowpass_filter.h/.cpp  一阶低通滤波器
```

---

## 2. 硬件连接

默认引脚定义（DengFOC V4 驱动板 + ESP32）：

| 功能            | GPIO | 说明                                   |
| --------------- | ---- | -------------------------------------- |
| A 相 PWM        | 32   | LEDC，30 kHz，8 bit                    |
| B 相 PWM        | 33   | LEDC，30 kHz，8 bit                    |
| C 相 PWM        | 25   | LEDC，30 kHz，8 bit                    |
| 电机使能 EN     | 12   | 输出高电平使能功率级                   |
| AS5600 SDA      | 19   | I2C，400 kHz                           |
| AS5600 SCL      | 18   | I2C，400 kHz，器件地址 0x36            |
| A 相电流采样    | 39   | ADC1，输入专用脚                       |
| B 相电流采样    | 36   | ADC1，输入专用脚                       |

电流采样参数：分流电阻 `0.01 Ω`，运放增益 `50`，换算比 `2 A/V`。
C 相电流未采样（`pinC = -1`），由 `Ia + Ib + Ic = 0` 隐含。

---

## 3. 关键参数

在 [src/main.cpp](src/main.cpp) 中修改：

| 参数         | 默认值 | 说明                                             |
| ------------ | ------ | ------------------------------------------------ |
| `Motor_PP`   | `7`    | 电机极对数，2208/2804 云台电机通常为 7           |
| `Sensor_DIR` | `-1`   | 编码器方向，电机反转或抖动时取反                 |
| `DFOC_Vbus`  | `12.6` | 供电电压(V)，必须与实际电源一致                  |

在 [src/main.cpp](src/main.cpp) 的 `loop()` 中设置电流环 PID：`DFOC_M0_SET_CURRENT_PID(5, 200, 0, 100000)`，
即 `P = 5`、`I = 200`、`D = 0`、输出斜率 `100000`。

---

## 4. 编译与烧录

```powershell
# 编译
pio run

# 烧录（自动选择串口）
pio run -t upload

# 打开串口监视器（115200）
pio device monitor
```

若 `pio` 不在 PATH 中，可直接使用：

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run
```

---

## 5. 串口协议

波特率 **115200**，一帧以 `\n` 结束：

| 方向 | 内容                | 说明                          |
| ---- | ------------------- | ----------------------------- |
| 下发 | `1.5\n`             | 设定目标 q 轴电流（A）        |
| 回显 | `1.500000`          | 目标值回显，确认下发成功      |
| 上报 | `0.012345`          | 滤波后的实测 Iq（A），每约 31 个循环一次 |

实测电流可直接用串口绘图器（Serial Plotter）观察阶跃响应。

---

## 6. 运行现象

1. 上电后串口先打印 `完成PWM初始化设置`、`编码器加载完毕`。
2. 电流零漂校准期间约 1 秒，电机必须静止。
3. 电角度对齐：电机被吸住约 1 秒后松开，串口打印 `0电角度：x.xxxx`。
4. 之后电机输出力矩为 0，等待串口下发目标电流。
5. 下发正/负值可改变力矩方向；数值超出安全范围时会被驱动能力与 PID 限幅约束。

---

## 7. 与原始 Arduino 例程的差异

源工程：`DengFOC_Lib_Lesson8_Current_Sense`（Arduino IDE 工程，core 2.x）。
本工程在保持控制逻辑一致的前提下，做了以下适配：

1. **LEDC API**
   Arduino-ESP32 3.x 已移除 `ledcSetup()` / `ledcAttachPin(channel, …)` / `ledcWrite(channel, …)`，
   改为 `ledcAttach(pin, freq, resolution)` 与 `ledcWrite(pin, duty)`。

2. **PID 对象初始化**
   原代码使用 `PIDController{.P = 2, .I = 0, .D = 0, .ramp = 100000, .limit = …}` 指定初始化器，
   该写法对含用户构造函数、成员名为 `output_ramp` 的类不合法，改为按构造函数参数初始化；
   速度环依赖 `Vbus/2` 的限幅仍在 `DFOC_Vbus()` 中重新设置。

3. **`pinC` 未初始化**
   原版 `CurrSense::pinC` 未赋值即参与 `_isset()` 判断（未定义行为），现显式初始化为哨兵值 `-1`。

4. **工程结构**
   原 `.ino` 单目录工程拆分为 PlatformIO 结构：算法库放入 `lib/DengFOC`，示例代码放入 `src/main.cpp`，
   头文件补充 include guard 与接口注释。

---

## 8. 常见问题

- **上电后电机剧烈抖动或堵转不转**
  多半是 `Motor_PP` 或 `Sensor_DIR` 填错；也有可能是供电电压 `DFOC_Vbus` 与实际电源不一致。
- **实测电流始终为 0 或恒定偏移**
  检查采样引脚是否接对，以及校准时电机是否在转动（转动会导致零漂校准错误）。
- **电机能在开环转动但闭环立刻失控**
  检查编码器方向与相序是否匹配，尝试把 `Sensor_DIR` 取反后重新对齐电角度。

---

## 9. 版权

原始库版权归 **灯哥开源（DengFOC）** 所有，遵循 GNU General Public License。
转载或修改请保留原有版权信息。