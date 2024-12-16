

#include "FOCMotor.h"
#include "BLDCmotor.h"
#include "MagneticSensor.h" 
#include "lowpass_filter.h" 
#include "Kalman.h"


// 全局变量定义
float shaft_angle;                    //!< 当前电机轴角度（弧度）
float electrical_angle;               //!< 当前电气角度（弧度）
float shaft_velocity;                 //!< 当前轴速度
float current_sp;                     //!< 目标电流
float shaft_velocity_sp;              //!< 目标轴速度
float shaft_angle_sp;                 //!< 目标轴角度
DQVoltage_s voltage;                  //!< DQ坐标系下的电压
DQCurrent_s current;                  //!< DQ坐标系下的电流

TorqueControlType torque_controller;   //!< 力矩控制器类型
MotionControlType controller;         //!< 运动控制器类型

float sensor_offset = 0;              //!< 传感器偏移量
float zero_electric_angle;            //!< 零电气角度
/******************************************************************************/
/**
 * @brief 计算并返回当前轴角度
 * 
 * 如果没有连接传感器，则返回之前的轴角度值（开环控制情况下）。
 * 
 * @return float 当前轴角度（弧度）
 */
float shaftAngle(void)
{
    // 如果没有传感器，则返回之前的轴角度值（开环控制）
    // if(!sensor) return shaft_angle;
    return sensor_direction * getAngle() - sensor_offset;
}

/**
 * @brief 计算并返回当前轴速度
 * 
 * 如果没有连接传感器，则返回之前的轴速度值（开环控制情况下）。
 * 
 * @return float 当前轴速度
 */
float shaftVelocity(void)
{
    // 如果没有传感器，则返回之前的轴速度值（开环控制）
    // if(!sensor) return shaft_velocity;
    // 使用低通滤波器处理速度信号，减少噪声
    return sensor_direction * LPFoperator(&LPF_velocity, getVelocity());
    // return sensor_direction * getVelocity();
}
/******************************************************************************/
/**
 * @brief 计算并返回当前电气角度
 * 
 * 电气角度 = （轴角度 + 传感器偏移量） * 极对数 - 零电气角度，并进行归一化处理。
 * 
 * @return float 当前电气角度（弧度）
 */
float electricalAngle(void)
{
    return _normalizeAngle((shaft_angle + sensor_offset) * pole_pairs - zero_electric_angle);
}
/******************************************************************************/
