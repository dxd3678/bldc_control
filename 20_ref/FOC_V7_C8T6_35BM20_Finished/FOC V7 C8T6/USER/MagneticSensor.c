#include "MagneticSensor.h" 
#include "foc_utils.h"
#include "gpio.h"

/******************************************************************************/
/**
 * @brief 全局变量定义
 * 
 * 这些变量用于存储传感器的角度数据、旋转计数、速度计算时间戳等信息。
 */
long cpr;                        // 每转的角度计数分辨率（Counts Per Revolution）
float full_rotation_offset;      // 全转角度偏移量，用于扩展角度范围
long angle_data, angle_data_prev;// 当前和前一次的角度数据
unsigned long velocity_calc_timestamp; // 速度计算的时间戳
float angle_prev;                // 前一次的角度值，用于速度计算

/*
iic_delay(): 实现大约2微秒的延时，确保I2C信号的稳定性。
iic_start(): 发送I2C启动信号，按照I2C协议的规定，形成通信的开始条件。
iic_stop(): 发送I2C停止信号，按照I2C协议的规定，结束通信。
iic_ack(): 发送I2C确认信号（ACK），表示成功接收数据。
iic_nack(): 发送I2C非确认信号（NACK），表示未成功接收数据或不再接收数据。
iic_wait_ack(): 等待从设备发送ACK信号，如果在规定时间内未收到ACK，则认为通信失败。
iic_read_byte(uint8_t ack): 读取一个字节的数据，并根据参数决定是否发送ACK或NACK信号。
iic_send_byte(uint8_t data): 发送一个字节的数据，通过I2C总线发送给从设备。


shaft_angle: 存储当前电机轴的角度值（弧度）。
electrical_angle: 存储当前电气角度值（弧度）。
shaft_velocity: 存储当前轴的速度值（弧度每秒）。
current_sp: 存储目标电流设定值。
shaft_velocity_sp: 存储目标轴速度设定值。
shaft_angle_sp: 存储目标轴角度设定值。
voltage: 存储DQ坐标系下的电压分量。
current: 存储DQ坐标系下的电流分量。
torque_controller: 存储用于控制电机力矩的控制器类型。
controller: 存储用于控制电机整体运动的控制器类型。
sensor_offset: 存储传感器的偏移量，用于校正角度值。
zero_electric_angle: 存储零电气角度，用于电气角度的计算基准。



shaftAngle(): 计算并返回当前轴的实际角度值。如果没有连接传感器（开环控制），则返回之前保存的角度值。使用传感器方向和偏移量进行校正。
shaftVelocity(): 计算并返回当前轴的速度值。如果没有连接传感器（开环控制），则返回之前保存的速度值。通过低通滤波器处理获取的速度值，减少噪声影响。
electricalAngle(): 计算并返回当前的电气角度值。电气角度 = （轴角度 + 传感器偏移量）× 极对数 - 零电气角度，并进行归一化处理（0到2π之间）。


I2C通信协议：
本代码实现了I2C通信协议的基本操作，包括启动、停止、发送和接收字节、发送ACK/NACK信号等功能。通过GPIO端口手动控制SCL和SDA线，实现与AS5600磁编码器传感器的通信。

AS5600磁编码器传感器：
AS5600是一款无接触式磁旋转位置传感器，能够提供高分辨率和精确的角度数据。本代码通过I2C接口读取传感器的角度计数值，并转换为实际角度和速度。

角度和速度计算：
通过读取AS5600的原始计数值，将其转换为实际的轴角度。考虑了全转次数和偏移量，确保角度值能够持续累积，避免因传感器的2π限制而溢出。
速度通过计算当前角度与前一次角度的变化量与采样时间差得到。同时，使用低通滤波器平滑速度信号，减少噪声影响，提高控制精度。

低通滤波器（LPF）：
在shaftVelocity()函数中使用了低通滤波器LPFoperator，用于处理获取的速度信号，减少高频噪声对速度测量的影响，提升速度计算的稳定性和可靠性。

极对数（pole_pairs）：
electricalAngle()函数中使用了极对数pole_pairs，用于将机械角度转换为电气角度。极对数表示电机的极对数（例如，4极电机有2个极对）。

角度归一化：
所有角度值在计算后都会调用_normalizeAngle()函数，将角度值归一化到0到2π之间，确保角度值在合理范围内，避免因角度溢出导致的计算错误。

*/
/******************************************************************************/

// I2C通信的GPIO端口和引脚定义
#define IIC_SCL_GPIO_PORT               GPIOB    // I2C时钟线(GPIOB端口)
#define IIC_SCL_GPIO_PIN                GPIO_PIN_6 // I2C时钟线(GPIOB端口6号引脚)
#define IIC_SDA_GPIO_PORT               GPIOB    // I2C数据线(GPIOB端口)
#define IIC_SDA_GPIO_PIN                GPIO_PIN_7 // I2C数据线(GPIOB端口7号引脚)

// 宏定义用于设置I2C的时钟线（SCL）状态，高电平或低电平
#define IIC_SCL(x)        do{ x ? \
                                  HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_SET) : \
                                  HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_RESET); \
                              }while(0)       /* 设置SCL线为高电平或低电平 */

// 宏定义用于设置I2C的数据线（SDA）状态，高电平或低电平
#define IIC_SDA(x)        do{ x ? \
                                  HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_SET) : \
                                  HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_RESET); \
                              }while(0)       /* 设置SDA线为高电平或低电平 */

// 宏定义用于读取I2C的数据线（SDA）状态
#define IIC_READ_SDA     HAL_GPIO_ReadPin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN) /* 读取SDA线状态 */

/**
 * @brief I2C通信的延时函数
 * 
 * 通过简单的空循环实现大约2微秒的延时，用于保证I2C信号的稳定性。
 */
static void iic_delay(void)
{
    for(uint32_t j = 0; j < 10; j++); // 简单的延时循环
}

/**
 * @brief 发送I2C启动信号
 * 
 * 按照I2C协议的规定，先将SDA和SCL拉高，然后拉低SDA，再拉低SCL，形成启动条件。
 */
void iic_start(void)
{
    IIC_SDA(1);       // 将SDA线拉高
    IIC_SCL(1);       // 将SCL线拉高
    iic_delay();      // 延时
    IIC_SDA(0);       // 将SDA线拉低，准备发送数据
    iic_delay();      // 延时
    IIC_SCL(0);       // 将SCL线拉低，开始数据传输
    iic_delay();      // 延时
}

/**
 * @brief 发送I2C停止信号
 * 
 * 按照I2C协议的规定，先将SDA拉低，然后将SCL拉高，最后将SDA拉高，形成停止条件。
 */
void iic_stop(void)
{
    IIC_SDA(0);       // 将SDA线拉低
    iic_delay();      // 延时
    IIC_SCL(1);       // 将SCL线拉高
    iic_delay();      // 延时
    IIC_SDA(1);       // 将SDA线拉高，结束通信
    iic_delay();      // 延时
}

/**
 * @brief 发送I2C确认信号（ACK）
 * 
 * 主设备发送ACK信号，表示接收端成功接收到数据。
 */
void iic_ack(void)
{
    IIC_SDA(0);       // 主设备将SDA线拉低，表示ACK
    iic_delay();      // 延时
    IIC_SCL(1);       // 将SCL线拉高，发送ACK信号
    iic_delay();      // 延时
    IIC_SCL(0);       // 将SCL线拉低，完成ACK发送
    iic_delay();      // 延时
    IIC_SDA(1);       // 主设备释放SDA线
    iic_delay();      // 延时
}

/**
 * @brief 发送I2C非确认信号（NACK）
 * 
 * 主设备发送NACK信号，表示接收端未成功接收到数据或不再接收数据。
 */
void iic_nack(void)
{
    IIC_SDA(1);       // 主设备将SDA线拉高，表示NACK
    iic_delay();      // 延时
    IIC_SCL(1);       // 将SCL线拉高，发送NACK信号
    iic_delay();      // 延时
    IIC_SCL(0);       // 将SCL线拉低，完成NACK发送
    iic_delay();      // 延时
}

/**
 * @brief 等待I2C确认信号
 * 
 * 主设备在发送数据后，等待从设备发送ACK。如果在规定时间内未收到ACK，则认为通信失败。
 * 
 * @return uint8_t 返回通信是否失败（1表示失败，0表示成功）
 */
uint8_t iic_wait_ack(void)
{
    uint8_t waittime = 0; // 等待计数器
    uint8_t rack = 0;      // ACK状态标志

    IIC_SDA(1);            // 主设备释放SDA线，准备接收ACK
    iic_delay();           // 延时
    IIC_SCL(1);            // 将SCL线拉高，准备读取ACK信号
    iic_delay();           // 延时

    // 等待从设备拉低SDA线表示ACK
    while (IIC_READ_SDA)   
    {
        waittime++; // 增加等待计数

        if (waittime > 250) // 如果等待时间超过阈值
        {
            iic_stop(); // 发送停止信号
            rack = 1;   // 设置ACK失败标志
            break;      // 退出等待循环
        }
    }

    IIC_SCL(0);            // 将SCL线拉低，完成ACK读取
    iic_delay();           // 延时
    return rack;           // 返回ACK状态（0表示成功，1表示失败）
}

/**
 * @brief 读取I2C字节数据
 * 
 * 读取从设备发送的一个字节数据，并根据参数决定是否发送ACK或NACK。
 * 
 * @param ack 是否发送ACK信号（1发送ACK，0发送NACK）
 * @return uint8_t 读取到的字节数据
 */
uint8_t iic_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0; // 循环计数器和接收数据变量

    for (i = 0; i < 8; i++ )    
    {
        receive <<= 1;         // 将接收数据左移一位
        IIC_SCL(1);            // 将SCL线拉高，准备读取数据位
        iic_delay();           // 延时

        if (IIC_READ_SDA)      // 如果SDA线为高电平
        {
            receive++;         // 接收数据位为1，累加到接收数据中
        }
        
        IIC_SCL(0);            // 将SCL线拉低，准备读取下一个数据位
        iic_delay();           // 延时
    }

    if (!ack)                  // 如果不需要发送ACK
    {
        iic_nack();            // 发送NACK信号
    }
    else
    {
        iic_ack();             // 发送ACK信号
    }

    return receive;            // 返回接收的数据字节
}

/**
 * @brief 发送I2C字节数据
 * 
 * 将一个字节数据通过I2C总线发送给从设备。
 * 
 * @param data 要发送的字节数据
 */
void iic_send_byte(uint8_t data)
{
    uint8_t t; // 循环计数器

    for (t = 0; t < 8; t++)
    {
        // 将数据的最高位（第7位）发送到SDA线上
        IIC_SDA((data & 0x80) >> 7);    
        iic_delay();           // 延时
        IIC_SCL(1);            // 将SCL线拉高，发送数据位
        iic_delay();           // 延时
        IIC_SCL(0);            // 将SCL线拉低，准备发送下一个数据位
        data <<= 1;            // 数据左移一位，准备发送下一位
    }
    IIC_SDA(1);                // 结束发送后释放SDA线
}

/**
 * @brief 读取AS5600传感器的两个字节数据
 * 
 * 通过I2C通信读取AS5600传感器的指定寄存器地址的两个字节数据。
 * 
 * @param readAddr 要读取的寄存器地址
 * @return uint16_t 读取到的16位数据
 */
uint16_t AS5600_ReadTwoByte(uint16_t readAddr)
{
    uint16_t temp = 0xFFFF; // 临时变量，用于存储读取的数据

    iic_start();                           // 发送I2C启动信号
    iic_send_byte((0X36 << 1) | 0x00);     // 发送AS5600的写地址（左移1位，加0表示写操作）
    iic_wait_ack();                        // 等待ACK信号
    iic_send_byte(readAddr);               // 发送要读取的寄存器地址
    iic_wait_ack();                        // 等待ACK信号
    iic_start();                           // 发送I2C重复启动信号
    iic_send_byte((0X36 << 1) | 0x01);     // 发送AS5600的读地址（左移1位，加1表示读操作）
    iic_wait_ack();                        // 等待ACK信号
    temp = iic_read_byte(1);               // 读取高位字节，并发送ACK信号
    temp = temp << 8 | iic_read_byte(0);   // 读取低位字节，不发送ACK信号
    iic_stop();                            // 发送I2C停止信号
    return temp;                           // 返回读取到的16位数据
}

/******************************************************************************/
#define  AS5600_Address  0x36      // AS5600传感器的I2C地址
#define  RAW_Angle_Hi    0x0C      // AS5600传感器的原始角度高字节寄存器地址
#define  AS5600_CPR      4096       // AS5600传感器的全转计数（Counts Per Revolution）

/**
 * @brief 获取AS5600传感器的原始计数值
 * 
 * 通过读取AS5600传感器的原始角度高字节寄存器（0x0C），获取16位原始计数值。
 * 
 * @return unsigned short 原始计数值
 */
unsigned short I2C_getRawCount()
{
    return AS5600_ReadTwoByte(0x0C); // 读取AS5600的原始角度数据
}

/******************************************************************************/
/**
 * @brief 初始化磁编码器传感器
 * 
 * 初始化过程中，设置CPR（每转计数分辨率）、读取初始角度数据、重置全转偏移量和速度计算时间戳。
 */
void MagneticSensor_Init(void)
{
    cpr = AS5600_CPR;                             // 设置每转计数分辨率
    angle_data = angle_data_prev = I2C_getRawCount(); // 读取初始角度数据并保存为当前和前一次的角度数据
    full_rotation_offset = 0;                     // 初始化全转偏移量为0
    velocity_calc_timestamp = 0;                  // 初始化速度计算时间戳为0
}
/******************************************************************************/

/**
 * @brief 获取当前轴的实际角度
 * 
 * 通过读取AS5600传感器的计数值，计算并返回当前轴的实际角度。考虑了全转偏移量，确保角度范围基本无限扩展。
 * 
 * @return float 当前轴角度（弧度）
 */
float getAngle(void)
{
    float d_angle; // 角度变化值

    angle_data = I2C_getRawCount(); // 读取当前的角度计数值

    // 计算角度变化量，用于检测全转
    d_angle = angle_data - angle_data_prev;

    // 如果角度变化量超过阈值（80%的CPR），判断为发生了溢出（全转）
    if(fabs(d_angle) > (0.8 * cpr)) 
        full_rotation_offset += d_angle > 0 ? -_2PI : _2PI; // 根据方向调整全转偏移量

    // 保存当前角度计数值，为下次计算做准备
    angle_data_prev = angle_data;

    // 计算并返回完整的轴角度，包括全转偏移量和当前计数值对应的角度
    return (full_rotation_offset + (angle_data * 1.0 / cpr) * _2PI);
}

/**
 * @brief 获取当前轴的速度
 * 
 * 通过计算当前角度与前一次角度的变化量与时间差，计算并返回当前轴的速度。使用低通滤波器平滑速度值。
 * 
 * @return float 当前轴速度（弧度每秒）
 */
float getVelocity(void)
{
    unsigned long now_us;    // 当前时间（微秒）
    float Ts, angle_now, vel; // 采样时间、当前角度、速度

    // 获取当前的系统滴答计数值
    now_us = SysTick->VAL; //_micros(); // 可能需要根据具体系统调整

    // 计算采样时间差（秒）
    if(now_us < velocity_calc_timestamp)
        Ts = (float)(velocity_calc_timestamp - now_us) / 14 * 1e-6; // 假设SysTick时钟周期为9ns
    else
        Ts = (float)(0xFFFFFF - now_us + velocity_calc_timestamp) / 14 * 1e-6; // 处理SysTick溢出的情况

    // 快速修复异常的采样时间（如为0或过长）
    if(Ts == 0 || Ts > 0.5) Ts = 1e-3; // 设置为1毫秒

    // 获取当前的完整轴角度
    angle_now = getAngle();

    // 计算速度：角度变化量除以时间差
    vel = (angle_now - angle_prev) / Ts;

    // 保存当前角度和时间戳，为下次计算做准备
    angle_prev = angle_now;
    velocity_calc_timestamp = now_us;

    return vel; // 返回计算得到的速度
}


