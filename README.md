# STM32F103 FOC 无刷电机控制项目

> 基于 **STM32F103C8T6（BluePill / 最小系统板）** 移植 **灯哥（ToanTech）DengFOC 开源库** 的磁场定向控制（FOC）项目，支持速度环、角度环、电流环三环控制，并通过串口与上位机/调试助手实时交互。

本仓库代码在 **Keil MDK (uvprojx)** 工程中开发，使用 **STM32 标准外设库 (Standard Peripheral Library, SPL 3.5)**。

---

## 目录

- [项目简介](#项目简介)
- [参考开源项目](#参考开源项目)
- [硬件方案](#硬件方案)
- [控制原理](#控制原理)
- [USART 实时控制系统（本仓库亮点）](#usart-实时控制系统本仓库亮点)
- [工程结构](#工程结构)
- [快速开始](#快速开始)
- [串口通信协议](#串口通信协议)
- [引脚分配](#引脚分配)
- [调参建议](#调参建议)
- [已知问题与注意事项](#已知问题与注意事项)
- [License](#license)

---

## 项目简介

这是一个典型的 **无感/有传感器 FOC 三环控制** 移植工程：

- 硬件主控：STM32F103C8T6（72MHz，Cortex-M3）
- 电机：带 **AS5600 磁编码器** 的云台/小型无刷电机（7 对极示例，可配置）
- 驱动：三相 PWM（TIM1 输出三路互补/独立 PWM）+ 采样电阻 + 运放放大电流信号
- 控制：电流环 → 速度环 → 角度环 级联（SimpleFOC 经典结构），底层算法源自 DengFOC / SimpleFOC
- 交互：**自研 USART2 实时控制小系统** —— 串口指令控制、**PID 在线整定**、运行状态实时回传（115200 波特率），无需重新编译即可远程调参

默认主循环中执行 `RUN_FOC()` + `Cmd_operation()`，通过串口指令在**速度环 / 位置环 / 力矩环**等模式间切换；工程同时保留了 `velocityOpenloop()`（速度开环）等参考实现。

> ✨ **本项目与灯哥原库最大的不同**：不只是一份 FOC 算法移植，而是额外开发了一套
> **完整可用的 USART 指令控制子系统**（见 [USART 实时控制系统](#usart-实时控制系统本仓库亮点)），
> 让电机控制从"改代码、重新烧录"变成了"发一条指令即可"，非常适合快速调试与二次开发。

---

## 参考开源项目

- **灯哥 FOC（DengFOC）** 官方仓库：[ToanTech/DengFOC_Lib](https://github.com/ToanTech/DengFOC_Lib)
  - 灯哥在 B 站有完整的 **《手把手教你写 FOC 算法》** 系列视频，本项目中的
    `FOC.c / PID.c / AS5600.c / lowpast_filter.c` 等模块即在该库思路基础上移植到
    STM32F1 + 标准外设库实现。
- **SimpleFOC**（DengFOC 的算法原型）：[simplefoc/Arduino-FOC](https://github.com/simplefoc/Arduino-FOC)
  - 本项目中的 `setPhaseVoltage / cal_Iq_Id / 开环速度 / 三环级联` 结构与 SimpleFOC 一脉相承。

> 说明：灯哥原库以 **Arduino/ESP32** 为主，本工程将其核心 FOC 算法完整搬到
> **STM32F103 + 标准外设库**，并在其基础上增加了**串口指令解析、PID 在线整定、
> 运行状态回传**等适合嵌入式调试的功能 —— 即下文重点介绍的
> [USART 实时控制系统](#usart-实时控制系统本仓库亮点)。

---

## 硬件方案

| 部件 | 型号/说明 |
| --- | --- |
| 主控 | STM32F103C8T6（72MHz，64KB Flash，20KB RAM） |
| 电机 | 小型云台无刷电机（示例为 7 对极，通过 `FOC_M0_alignSensor(PP, DIR)` 配置） |
| 编码器 | AS5600 磁编码器（12bit，IIC 接口） |
| 驱动 | 三相全桥驱动 + 双电阻采样，运放放大后经 ADC 采样 |
| 电流采样 | 2 路（A/B 相），0.01Ω 采样电阻 + 50 倍运放（`AD.c` 中可配置） |
| 供电 | 12V 电机电源（`power_supply` 宏，位置在 `main.c` / `FOC.c`） |

### 电流采样电路参数

`AD.c` 中通过 `CurrSense()` 配置：

```c
shunt_resistor = 0.01;   // 采样电阻 10mΩ
amp_gain       = 50;     // 运放放大倍数
volts_2_amps_ratio = 1.0f / shunt_resistor / amp_gain;
```

并在上电时执行 `Take_offset()`（1000 次采样求平均）得到零漂补偿。

---

## 控制原理

采用 **SimpleFOC/DengFOC 经典双环/三环级联结构**：

```
角度目标 ──► [角度环 PID] ──► 速度目标 ──► [速度环 PID] ──► 电流目标 ──► [电流环 PID] ──► 力矩(电压Uq) ──► SVPWM ──► 电机
                                                                                             ▲
                                              AS5600 角度 ──► 速度计算(getVelocity)           │
                                              电流采样(A/D) ──► Clark/Iq 提取(cal_Iq_Id) ─────┘
```

对应 `FOC.c` 中的核心函数：

| 函数 | 作用 |
| --- | --- |
| `FOC_M0_setTorque()` | 电流（力矩）环：目标电流 → Iq 反馈 → PID → 输出 Uq |
| `FOC_M0_setVelocity()` | 速度环：目标速度 → 速度反馈（编码器微分） → PID → 电流目标 |
| `FOC_M0_set_Velcocity_Angle()` | 位置环：目标角度 → 角度反馈 → PID → 速度目标（三环级联） |
| `FOC_M0_set_Force_Angle()` | 位置-力矩混合模式（角度环 + 电流环，无速度环） |
| `velocityOpenloop()` | 速度开环（不依赖编码器反馈） |
| `cal_Iq_Id()` | 相电流 Clark 变换提取 q 轴电流 |
| `S0_electricalAngle()` | 电角度计算（机械角 × 极对数 × 方向） |
| `FOC_M0_alignSensor()` | 上电编码器对齐（校准零电角度） |

### PID 控制器（`PID.c`）

- 增量式 PID + 梯形积分 + 输出限幅 + **输出变化率限制（output_ramp）**
- 时间基准为 `micros()`（TIM3 扩展 64 位微秒计数）
- 积分项、微分项均支持独立的抗饱和处理

### 低通滤波器（`lowpast_filter.c`）

一阶低通（指数滑动平均），用于速度 / 电流反馈信号平滑，时间常数在 `main.c` 中配置：

```c
LowPassFilter_Init(0.01 , &M0_Vel_Flt);   // 速度滤波
LowPassFilter_Init(0.05 , &M0_Curr_Flt);  // 电流滤波
```

### 实时时钟（`REALTIME.c`）

TIM3 以 72MHz/72=1MHz 计数（0xFFFF 周期溢出），中断累加 `timer3_overflow`，
`micros()` 返回 64 位微秒时间戳，作为 PID / 滤波 / 速度计算的时间基准。

---

## USART 实时控制系统（本仓库亮点）

> 这是作者在灯哥 FOC 移植之外**自主设计并实现**的一套实时控制小系统：
> 用一条串口线就能完成 **模式切换、PID 在线整定、状态监控**，全程无需重新编译烧录，
> 大幅提升电机调试效率。核心代码集中在 `Hardware/Serial.c` 与 `Hardware/FOC.c`。

### 系统架构

```
                    ┌────────────────────────────────────────────────────────┐
                    │                     USART2 (115200)                     │
                    │   PA3(RX)                                          PA2(TX)│
                    └─────────┬──────────────────────────────────┬───────────┘
                              │                                  │
                     USART2_IRQHandler()              Serial_SendString/Printf
                    （中断收帧，不丢数据）                  （状态/参数回传）
                              │
                              ▼
                    Array_Cut_Operation()
                    （拆包：命令 + 最多5个参数）
                              │
                              ▼
                    Cmd_Compare() ──► 指令编号(1~10)
                              │
                              ▼
                    Cmd_operation()  ──► 状态机调度（flag 机制）
                              │
                ┌─────────────┼─────────────┐
                ▼             ▼             ▼
       速度环/位置环/力矩环   PID 在线整定    PID/速度/角度/电流查询
       (FOC_M0_set*)       (SetPID)        (Show_*)
```

### 工作流程

1. **中断收帧**：`USART2_IRQHandler()` 在中断里完成协议解析，从 `/` 起始、`\r\n` 结束的
   数据帧中提取命令与参数，保证主循环不被串口阻塞、不丢数据；
2. **指令拆包**：`Array_Cut_Operation()` 将命令与最多 5 个参数拆出，`Cmd_Compare()` 将
   字符串命令映射为指令编号；
3. **状态机调度**：`Cmd_operation()` 通过 `flag` 机制区分"持续控制指令"与
   "一次性查询/整定指令"——持续指令（如 `/setvel`）每周期执行，查询/整定指令执行后
   自动回到之前的运行状态，互不干扰；
4. **在线整定**：`/setpidvel /setpidang /setpidcur` 可在电机运行中直接修改三环 PID，
   参数即时生效，调试无需重新编译。

### 支持的指令总览

| 类别 | 指令 | 说明 |
| --- | --- | --- |
| 持续控制 | `/setvel N` `/setang N` `/setprog N` | 速度/位置/程序控制 |
| 在线整定 | `/setpidvel` `/setpidang` `/setpidcur` | 三环 PID 参数实时修改 |
| 状态查询 | `/showpid` `/showvel` `/showang` `/showcur` | 参数与运行状态回传 |

> 完整协议见 [串口通信协议](#串口通信协议) 一节；指令清单文件见工程根目录
> `FOC_SYS_command.txt`。

---

## 工程结构

基于 Keil MDK 的 `Project.uvprojx`，目录分组如下：

```
├── Project.uvprojx        Keil 工程文件
├── Start/                 CMSIS 启动文件（startup_stm32f10x_md.s）+ core_cm3 + system_stm32f10x
├── Library/               STM32F10x 标准外设库（SPL 3.5）
├── System/                系统层
│   ├── system.h           GPIO 位带操作宏（PAout/PBout/...）
│   ├── SysTick.c          delay_us / delay_ms
│   ├── REALTIME.c         高精度微秒时基（TIM3）→ 放于 Hardware 亦可
│   └── usart.c            USART1（预留调试串口）
├── Hardware/              硬件驱动层
│   ├── FOC.c/.h           ★ FOC 核心算法（对齐、三环、SVPWM）+ 串口命令调度（Cmd_operation）
│   ├── Motor.c/.h         电机 PWM 输出封装（3 路）
│   ├── PWM.c/.h           TIM1 三路 PWM 初始化（PA8/PA9/PA10，约 3kHz）
│   ├── AD.c/.h            电流采样（ADC1 + DMA 双通道 + 零漂校准）
│   ├── AS5600.c/.h        AS5600 磁编码器（IIC，角度/速度）
│   ├── iic.c/.h           软件 IIC（PB8=SCL，PB9=SDA）
│   ├── Serial.c/.h        ★ USART 实时控制系统（USART2 中断收帧 + 指令解析 + 浮点打印）
│   ├── lowpast_filter.c/.h 一阶低通滤波
│   └── OLED.c/.h          OLED 驱动（工程内保留，默认未编译进工程）
└── User/                  用户层
    ├── main.c             ★ 主程序（初始化 + 控制循环）
    ├── PID.c/.h           PID 控制器
    └── stm32f10x_it.c     中断处理模板
```

> 注意：`OLED.c / OLED_Data.c / Store.c / MyFLASH.c / Timer.c / Delay.c` 等文件保留在
> 工程目录中但**未加入当前 `Project.uvprojx`**（为历史调试遗留），如需使用可自行加入工程。

### 编译环境

- IDE：Keil MDK-ARM（uVision5）
- 编译器：AC5（armcc）或 AC6，均兼容
- 芯片：STM32F103C8（High-density 亦可）
- 宏定义：`USE_STDPERIPH_DRIVER`
- 包含路径：`.\Start; .\Library; .\User; .\System; .\Hardware`
- 工程使用 `startup_stm32f10x_md.s` 启动文件

---

## 快速开始

1. **接线**
   - TIM1 PWM：PA8 / PA9 / PA10 → 三相驱动输入
   - 电流采样：PA4 / PA5（ADC1 通道 4/5）→ 运放输出
   - AS5600：PB8（SCL）/ PB9（SDA），接 IIC
   - 串口：PA2（USART2_TX）/ PA3（USART2_RX），115200-8-N-1

2. **编译下载**
   - 使用 Keil 打开 `Project.uvprojx`
   - 选择 `STM32F103C8` 目标，编译后下载到板子

3. **上电对齐**
   - 程序上电后会执行 `FOC_M0_alignSensor(7, 1)`，电机会施加一个固定电压并
     停留约 1 秒以对齐编码器零位（期间请勿用手掰动电机）
   - 极对数 `7` 与方向 `1` 需与你的电机实际匹配，否则需修改 `FOC_M0_alignSensor()` 参数

4. **串口实时控制**
   - 打开任意串口助手，115200，以 `/` 开头、`\r\n` 结尾发送指令（见下节协议）
   - 上电后可直接用 `/setvel 30` 让电机转起来，用 `/setpidvel` 等指令在线整定 PID
   - 这一整套指令交互就来自本项目自研的 [USART 实时控制系统](#usart-实时控制系统本仓库亮点)

---

## 串口通信协议

指令格式：`/命令 参数1 参数2 ...` （命令与参数、参数之间用**空格**分隔，以 `\r\n` 结尾）

### 控制指令

| 指令 | 参数 | 说明 |
| --- | --- | --- |
| `/setvel N` | N: 目标速度 (rad/s) | 速度环控制 |
| `/setang N` | N: 目标角度 (rad) | 位置（角度）环控制（三环级联） |
| `/setprog N` | N: 程序编号 | 预留程序控制（当前为空实现） |
| `/setpidvel KP KI KD Ramp Limit` | 5 个参数 | 在线整定速度环 PID |
| `/setpidang KP KI KD Ramp Limit` | 5 个参数 | 在线整定角度环 PID |
| `/setpidcur KP KI KD Ramp Limit` | 5 个参数 | 在线整定电流环 PID |

### 查询指令

| 指令 | 说明 |
| --- | --- |
| `/showpid` | 显示三环 PID 参数（Kp/Ki/Kd/Ramp/Limit） |
| `/showvel` | 显示当前速度 |
| `/showang` | 显示当前角度 |
| `/showcur` | 显示当前 Iq 电流 |

### 示例

```
/setvel  30          → 设定速度 30 rad/s
/setang  3.14        → 设定角度 180°（3.14 rad）
/setpidvel 0.02 1 0 100000 30   → 在线设置速度环 PID
/showpid              → 打印当前三环参数
```

> 协议说明：命令以 `/` 开始，数据帧以 `\r\n` 结束。`/setpid*`（1 类指令）与
> `/show*`（2 类指令）为"非持续性指令"，执行后保持之前的运行状态不变；
> 连续控制指令（如 `/setvel`）会持续执行直到收到下一条指令或复位。

---

## 引脚分配

| 功能 | 引脚 | 外设/说明 |
| --- | --- | --- |
| 三相 PWM | PA8 / PA9 / PA10 | TIM1_CH1/CH2/CH3（PWM 模式 1，约 3kHz，见下文计算） |
| 电流采样 | PA4 / PA5 | ADC1 通道 4/5，双通道 DMA 循环采样 |
| AS5600 SCL | PB8 | 软件 IIC |
| AS5600 SDA | PB9 | 软件 IIC |
| 调试串口 | PA2 (TX) / PA3 (RX) | USART2，115200 |
| 预留串口 | PA9 (TX) / PA10 (RX) | USART1（`usart.c` 保留） |

### 关键配置位置

| 参数 | 位置 | 说明 |
| --- | --- | --- |
| 供电电压 | `main.c` / `FOC.c` 中 `power_supply` | 12.0V，需与母线电压一致 |
| 极对数/方向 | `FOC_M0_alignSensor(7, 1)` | main.c 中调用 |
| 电流采样参数 | `AD.c` `CurrSense()` | 采样电阻、运放倍数 |
| PWM 频率 | `PWM_Init(999, 23)` | Motor.c，约 3kHz（72MHz/((999+1)×(23+1))） |
| PID 初值 | `main.c` | vel/angle/current 三环参数 |

---

## 调参建议

1. **先电流环**：将电流环 P 设小、I 设大（如 `0.5 / 50`），观察 Iq 响应，保证电流不振荡；
2. **再速度环**：在电流环稳定的基础上调速度环（如 `0.1 / 2`），逐步增大 Kp 直到出现轻微振荡再回调；
3. **最后角度环**：角度环通常需要较大 Kp（如 `10`），Kd 用于阻尼；
4. **利用在线整定**：运行时直接发送 `/setpidvel` 等指令实时修改参数，无需重新编译
   （这是本仓库 [USART 实时控制系统](#usart-实时控制系统本仓库亮点) 的核心价值）；
5. **注意输出限幅**：速度环输出（电流目标）限幅 `OutMax` 应小于电流环能力；电流环输出限幅等于母线电压。

---

## 已知问题与注意事项

- **上电对齐**：`FOC_M0_alignSensor()` 期间电机会通电，请确保电机可自由转动、负载不要过大；
- **电压限制**：`power_supply` 宏须与实际母线电压一致，否则 PWM 占空比映射会失真；
- **极对数**：对齐参数 `7` 为示例值，务必按实际电机修改，否则电角度错误会导致失控；
- **浮点打印**：`Serial_Print_float_number()` 为自实现浮点转字符串，最多保留 6 位小数；
- **电流采样**：零漂校准依赖上电时电机静止，若电机在 `Take_offset()` 期间转动会导致偏置错误；
- **OLED/存储等**：工程目录中的 OLED、Flash 存储模块未编入当前工程，属于历史功能。

---

## License

本项目基于开源项目 **DengFOC（ToanTech）** 与 **SimpleFOC** 的思路进行移植与二次开发，
具体版权请遵循上游开源协议。代码仅供学习交流使用。

---

*项目维护：见 Git 提交历史 · 如有问题欢迎提交 Issue*
