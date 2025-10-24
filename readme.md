# FastRC飞行控制器
> Fast for rocket control
## 硬件架构：
### 主控板
主控电源设计思想：
5V电源：直接来自USB端口，或者来自降压的锂电池电源。注意做好电源隔离
3V3电源：由5V电源经过LDO降压解决，分为两路，一路给stm32F405,传感器及其一些外设端口供电，另一路给SD，LED外设供电

#### 主要器件型号
|器件|型号|
|----|----|
|主控|stm32F405RGT6|
|陀螺仪|ICM-42688-P|
|气压计|SPL06|
|磁力计|LIS3MDLTR|
#### 开关设计：
BUTTON1:切换多旋翼/火箭模式
BOOT0:
#### 接口资源
`六路UART串口`
|  串口  |  作用  |
|--|--|
|UART1|接收机：支持ELRS和SBUS|
|UART2|GPS接口：支持串口通信和IIC的GPS|
|UART3|空闲|
|UART4|空闲|
|UART5|空闲|
|UART6|空闲|

`6路GPIO`
|  GPIO  |  作用  |推荐模式|
|--|--|--|
|GPIO1|点火端口|GPIO_ouput|
|GPIO2|安全端口|GPIO_input|
|GPIO3|空闲|---|
|GPIO4|空闲|---|
|GPIO5|空闲|---|
|GPIO6|空闲|---|

`9路PWM`
| PWM | 作用 |
|---|---|
|TIM1_CH1/GPIO7|蜂鸣器控制|
|TIM1_CH2|空闲|
|TIM1_CH3|空闲|
|TIM2_CH1|空闲|
|TIM2_CH2|空闲|
|TIM3_CH1|电机/舵机|
|TIM3_CH2|电机/舵机|
|TIM3_CH3|电机/舵机|
|TIM3_CH4|电机/舵机|

其他:
|Interface|作用|
|--|--|
|拓展IIC|空闲|
|typec|提供电源，传输数据|

### 电源板
#### 设计思想
11.7V->5V在电源板上完成
5v->3.3v只需要在需要的期间上引出3V3给LDO的OUT就行
电源板还需要完成：
9路舵机供电
两个点火端口
### 信号板
放置GPS和接收机

## 设计目标
1. 正常计算姿态
2. 存储数据
3. 控制点火，开伞和舵机
### 拓展设计目标
1. 数据回传
2. 上位机通信

### 软件架构
>软件架构仿照如下：
```
ANO_PioneerPro-088/
├── Libraries/                                  # 第三方库文件
│   ├── CMSIS/                                  # ARM Cortex-M内核支持库
│   │   ├── DSP_Lib/                            # 数字信号处理库
│   │   ├── Include/                            # CMSIS头文件
│   │   └── ST/                                 # ST官方CMSIS支持
│   ├── STM32F4xx_StdPeriph_Driver/             # ST标准外设驱动库
│   │   ├── inc/                                # 头文件
│   │   └── src/                                # 源文件
│   └── USBStack/                               # USB协议栈
│       ├── INC/                                # USB头文件
│       └── SRC/                                # USB源文件
└── SRC/                                        # 飞控主源码
    ├── AnoImu/                                 # IMU相关模块
    │   ├── Ano_Imu_Calibration.c/.h            # IMU校准
    │   ├── Ano_Imu_Data.c/.h                   # IMU数据处理
    │   ├── Ano_Imu_Task.c/.h                   # IMU任务管理
    ├── applications/                           # 应用层代码
    │   ├── Ano_DT.c/.h                         # 数据传输
    │   ├── Ano_FlyCtrl.c/.h                    # 飞行控制
    │   ├── Ano_OF.c/.h                         # 光流处理
    │   ├── Ano_OF_DecoFusion.c/.h              # 光流数据融合
    │   ├── Ano_OPMV_CBTracking_Ctrl.c/.h       # OpenMV色块追踪控制
    │   ├── Ano_OPMV_Ctrl.c/.h                  # OpenMV控制
    │   ├── Ano_OPMV_LineTracking_Ctrl.c/.h     # OpenMV线追踪控制
    │   ├── Ano_Parameter.c/.h                  # 参数管理
    │   ├── Ano_ProgramCtrl_User.c/.h           # 用户程序控制
    │   ├── Ano_RC.c/.h                         # 遥控器处理
    │   ├── Ano_Scheduler.c/.h                  # 任务调度
    │   ├── Ano_USB.c/.h                        # USB通信
    │   ├── Ano_UWB.c/.h                        # UWB定位
    │   ├── BSP_Init.c/.h                       # 板级初始化
    │   ├── main.c                              # 主程序入口
    │   ├── stm32f4xx_conf.h                    # STM32配置
    │   ├── stm32f4xx_it.c                      # 中断处理
    │   └── usb_config.c                        # USB配置
    ├── drivers/                                # 驱动层代码
    │   ├── Drv_BSP.c/.h                        # 板级支持包
    │   ├── Drv_OpenMV.c/.h                     # OpenMV驱动
    │   ├── Drv_RcIn.c/.h                       # 遥控输入驱动
    │   ├── Drv_Servo.c/.h                      # 舵机驱动
    │   ├── Drv_UP_Flow.c/.h                    # 光流传感器驱动
    │   ├── Drv_adc.c/.h                        # ADC驱动
    │   ├── Drv_ak09915.c/.h                    # AK09915磁力计驱动
    │   ├── Drv_ak8975.c/.h                     # AK8975磁力计驱动
    │   ├── Drv_bmi088.c/.h                     # BMI088传感器驱动
    │   ├── Drv_gps.c/.h                        # GPS驱动
    │   ├── Drv_heating.c/.h                    # 加热控制驱动
    │   ├── Drv_i2c_soft.c/.h                   # 软件I2C驱动
    │   ├── Drv_icm20602.c/.h                   # ICM20602传感器驱动
    │   ├── Drv_laser.c/.h                      # 激光传感器驱动
    │   ├── Drv_led.c/.h                        # LED驱动
    │   ├── Drv_pwm_out.c/.h                    # PWM输出驱动
    │   ├── Drv_spi.c/.h                        # SPI驱动
    │   ├── Drv_spl06.c/.h                      # SPL06气压计驱动
    │   ├── Drv_time.c/.h                       # 时间管理驱动
    │   ├── Drv_usart.c/.h                      # USART驱动
    │   ├── Drv_w25qxx.c/.h                     # W25QXX FLASH驱动
    │   └── usbd_STM32F4xx_FS.c                 # STM32 USB设备驱动
    ├── fc_general/                             # 通用飞控模块
    │   ├── Ano_FcData.c/.h                     # 飞控数据管理
    │   ├── Ano_Filter.c/.h                     # 滤波器
    │   ├── Ano_Imu.c/.h                        # IMU处理
    │   ├── Ano_Math.c/.h                       # 数学运算
    │   ├── Ano_MotionCal.c/.h                  # 运动校准
    │   ├── Ano_Navigate.c/.h                   # 导航算法
    │   ├── Ano_Pid.c/.h                        # PID控制
    │   ├── Ano_Sensor_Basic.c/.h               # 基础传感器处理
    └── fc_specific/                            # 特定飞控功能模块
        ├── Ano_AltCtrl.c/.h                    # 高度控制
        ├── Ano_AttCtrl.c/.h                    # 姿态控制
        ├── Ano_FlightCtrl.c/.h                 # 飞行控制
        ├── Ano_FlightDataCal.c/.h              # 飞行数据计算
        ├── Ano_LocCtrl.c/.h                    # 位置控制
        ├── Ano_MagProcess.c/.h                 # 磁力计处理
        ├── Ano_MotorCtrl.c/.h                  # 电机控制
        ├── Ano_Power.c/.h                      # 电源管理
        └── config.h                            # 配置文件
```
### 开发流程
>软件开发
#### 1. 基础通信层

|模块通信驱动|通信方式|负责人|
|--|--|--|
|陀螺仪ICM-42688-P|SPI|黄|
|磁力计LIS3MDLTR|IIC|黄|
|气压计SPL06|IIC|程|
|USB通信|虚拟串口，USB|程|
