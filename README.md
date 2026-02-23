# MIC_FFT 项目文档

## 项目概述

这是一个基于 STM32F103C8T6 微控制器的麦克风音频信号 FFT（快速傅里叶变换）分析项目。该项目通过 ADC 采集麦克风音频信号，使用 ARM Cortex-M3 的 DSP 库进行 FFT 运算，并在 OLED 显示屏上实时显示时域波形（谐波模式）和频域频谱（频谱模式）。

### 主要技术栈

- **微控制器**: STM32F103C8T6 (Cortex-M3, 72MHz)
- **开发环境**: STM32CubeMX + Keil MDK-ARM V5
- **HAL 库**: STM32F1xx HAL Driver
- **DSP 库**: STM32 官方 DSP 库（汇编优化的 FFT 实现）
- **外设接口**:
  - ADC1（PA1 引脚）：麦克风信号采集
  - DMA1_Channel1：ADC 数据自动传输
  - I2C1（PB6-SCL, PB7-SDA）：OLED 显示屏通信
  - TIM3：ADC 触发定时器
  - TIM4：按键消抖定时器
  - GPIO：按键输入（PB1）和 LED 指示（PC13）

### 硬件配置

- **系统时钟**: 72MHz（HSE 外部晶振 + PLL ×9）
- **ADC 配置**:
  - 采样时间：1.5 周期（谐波模式）/ 239.5 周期（频谱模式）
  - 触发方式：软件触发（谐波模式）/ TIM3_TRGO 触发（频谱模式）
  - 分辨率：12 位（0-4095）
- **定时器配置**:
  - TIM3：1632 周期，用于触发 ADC 采样
  - TIM4：10000 周期（预分频 8），用于按键消抖

## 项目结构

```
MIC_FFT/
├── Core/                          # STM32CubeMX 生成的核心代码
│   ├── Inc/                       # 头文件
│   │   ├── adc.h                  # ADC 配置头文件
│   │   ├── dma.h                  # DMA 配置头文件
│   │   ├── gpio.h                 # GPIO 配置头文件
│   │   ├── i2c.h                  # I2C 配置头文件
│   │   ├── main.h                 # 主程序头文件
│   │   ├── stm32f1xx_hal_conf.h   # HAL 库配置
│   │   ├── stm32f1xx_it.h         # 中断服务程序头文件
│   │   └── tim.h                  # 定时器配置头文件
│   └── Src/                       # 源文件
│       ├── adc.c                  # ADC 初始化和配置
│       ├── dma.c                  # DMA 初始化和配置
│       ├── gpio.c                 # GPIO 初始化和配置
│       ├── i2c.c                  # I2C 初始化和配置
│       ├── main.c                 # 主程序逻辑
│       ├── stm32f1xx_hal_msp.c    # HAL MSP 初始化
│       ├── stm32f1xx_it.c         # 中断服务程序
│       ├── system_stm32f1xx.c     # 系统初始化
│       └── tim.c                  # 定时器初始化和配置
├── Drivers/                       # STM32 HAL 和 CMSIS 驱动
│   ├── CMSIS/                     # ARM CMSIS 核心
│   └── STM32F1xx_HAL_Driver/      # STM32F1xx HAL 驱动
├── MDK-ARM/                       # Keil 项目文件
│   ├── MIC.uvprojx                # Keil 项目文件
│   ├── MIC.uvoptx                 # Keil 选项文件
│   ├── DSP/                       # DSP 库文件
│   │   ├── cr4_fft_256_stm32.s    # 256 点 FFT 汇编实现
│   │   ├── cr4_fft_1024_stm32.s   # 1024 点 FFT 汇编实现
│   │   ├── stm32_dsp.h            # DSP 函数声明
│   │   └── table_fft.h            # FFT 查找表
│   └── MyDrivers/                 # 自定义驱动
│       ├── OLED.c/h               # OLED 显示屏驱动
│       ├── OLED_Data.c/h          # OLED 字库和显示数据
│       └── key.c/h                # 按键驱动
├── MIC.ioc                        # STM32CubeMX 项目配置文件
├── .mxproject                     # STM32CubeMX 元数据
└── 硬件电路.png                   # 硬件电路图

```

## 编译和烧录

### 编译项目

本项目使用 Keil MDK-ARM 开发环境进行编译。请在 Keil IDE 中打开 `MDK-ARM/MIC.uvprojx` 项目文件。

**编译步骤**：
1. 打开 Keil MDK-ARM
2. 打开项目文件 `MDK-ARM/MIC.uvprojx`
3. 选择目标：`MIC`
4. 点击 "Project" → "Build Target" 或按 F7 编译项目

**编译产物**：
- `MDK-ARM/MIC/MIC.axf` - 调试可执行文件
- `MDK-ARM/MIC/MIC.hex` - Intel HEX 格式固件（用于烧录）

### 烧录固件

可以使用以下方式烧录生成的 HEX 文件：

1. **ST-Link Utility**：使用 ST-LINK 调试器
2. **Keil MDK Flash Download**：在 Keil IDE 中直接烧录
3. **J-Link**：使用 SEGGER J-Link 调试器
4. **USB 串口**：使用 USB 串口和 ISP 模式烧录（需要 Bootloader）

## 功能说明

### 工作模式

系统支持两种显示模式，通过按键（PB1）切换：

#### 模式 0：谐波显示模式（时域波形）
- 使用 ADC 连续转换模式
- 软件触发采样
- 最小采样时间（1.5 周期）以获得最高采样率
- 在 OLED 上实时显示音频信号的时域波形
- 波形范围：X 轴 0-127，Y 轴 0-63（对应 ADC 值 0-4095）

#### 模式 1：频谱显示模式（频域频谱）
- 使用 ADC 单次转换模式
- TIM3 定时器触发采样（固定采样率）
- 最大采样时间（239.5 周期）以确保采样精度
- 使用 DMA 自动采集 256 个采样点
- 对采集的数据进行 256 点 FFT 运算
- 计算各频率分量的幅值
- 在 OLED 上显示 64 个频率点的频谱图

### 关键算法

#### FFT 运算
使用 STM32 优化的 Radix-4 复数 FFT 算法：
- 函数：`cr4_fft_256_stm32(lBufOutArray, AD_Value_buf[data_select], 256)`
- 输入：256 个 int16_t 类型的采样点（实数）
- 输出：256 个 uint32_t 类型的复数结果（高 16 位为虚部，低 16 位为实部）

#### 幅值计算
FFT 输出是复数形式，需要计算其模（幅值）：
```c
real = (lBufOutArray[i] << 16) >> 16;  // 提取实部
imag = (lBufOutArray[i] >> 16);        // 提取虚部
magnitude[i] = sqrt(real * real + imag * imag) >> 3;  // 计算幅值并缩放
```

### 双缓冲机制

使用两个 256 点的缓冲区交替工作：
- `AD_Value_buf[0]` 和 `AD_Value_buf[1]`
- ADC 完成一次采集后，通过 DMA 中断切换到另一个缓冲区
- 主循环从已完成的缓冲区读取数据进行 FFT 运算

## 开发约定

### 代码风格
- 使用 STM32CubeMX 生成的基础框架
- 用户代码放在 `/* USER CODE BEGIN */` 和 `/* USER CODE END */` 标记之间
- 使用 HAL 库函数进行外设操作
- 变量命名采用驼峰命名法

### 中断处理
- ADC 转换完成中断：`HAL_ADC_ConvCpltCallback`
- 定时器中断：`HAL_TIM_PeriodElapsedCallback`
- 按键消抖使用 TIM4 中断处理

### 内存配置
- 堆大小：0x200
- 栈大小：0x400
- RAM 使用：从 0x20000000 到 0x20004FFF（20KB）

### 注意事项
1. **时序配置**：谐波模式追求高采样率，频谱模式追求高精度
2. **FFT 输入**：必须是 2 的幂次方长度（项目使用 256 点）
3. **显示刷新**：OLED 使用缓冲区机制，需要调用 `OLED_Update()` 刷新显示
4. **按键防抖**：使用定时器中断实现硬件防抖，避免误触发

## 扩展开发

### 添加新的显示模式
在 `main.c` 的主循环中添加新的模式分支，配置相应的 ADC 参数和显示逻辑。

### 修改 FFT 点数
1. 在 `MDK-ARM/DSP/` 目录下使用相应的 FFT 汇编文件
2. 修改缓冲区大小
3. 调整采样点数和显示分辨率

### 自定义 OLED 显示
使用 `MyDrivers/OLED.c/h` 中提供的绘图函数：
- `OLED_DrawPoint()` - 画点
- `OLED_DrawLine()` - 画线
- `OLED_ShowString()` - 显示字符串
- `OLED_ShowNum()` - 显示数字

## 故障排除

### 编译错误
- 确保已安装 STM32F1xx_DFP 包
- 检查 DSP 库文件是否正确添加到项目中
- 检查包含路径设置

### 运行问题
- 确认系统时钟配置正确（72MHz）
- 检查 ADC 和 DMA 初始化顺序
- 验证 I2C 连接和 OLED 地址

### 显示问题
- 检查 OLED 供电和 I2C 连接
- 确认 OLED_Init() 初始化成功
- 验证 OLED_Update() 是否被调用

## 参考资料

- [STM32F103x8 数据手册](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf)
- [STM32F1xx HAL 驱动用户手册](https://www.st.com/resource/en/user_manual/dm00105879-description-of-stm32f1xx-hal-drivers-stmicroelectronics.pdf)
- [STM32 DSP 库文档](https://www.st.com/resource/en/user_manual/cd00258089-stm32f10x-dsp-library-standard-library-stmicroelectronics.pdf)
- [OLED 驱动文档](https://江协科技.com)

## 许可证

本项目使用 STMicroelectronics HAL 库，遵循相应的许可协议。
