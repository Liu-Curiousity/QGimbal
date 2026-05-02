/**
 * @file        Gimbal_config.h
 * @brief 		用于定义QGimbal的配置常量
 * @detail
 * @author      Liu-Curiousity (2675794963@qq.com)
 * @date        26-4-28
 * @version 	V1.0.0
 * @note 		
 * @warning	    
 * @par 		历史版本
                V1.0.0创建于26-4-28
 * @copyright   (c) 2026 QDrive
 * */

#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

#define GIMBAL_SOFTWARE_VERSION "2.0.0"

/*==========================云台参数==========================*/
#define GIMBAL_MAX_CURRENT      1.65f   // 最大电流,单位A

/*==========================配置参数==========================*/
#define GIMBAL_MAX_SPEED        1000.0f   // 最大转速,单位rpm

#define GIMBAL_SPEED_KP_YAW     1.3e-2f
#define GIMBAL_SPEED_KI_YAW     3.9e-4f
#define GIMBAL_SPEED_KD_YAW     0.0f
#define GIMBAL_SPEED_KP_PITCH   3e-3f
#define GIMBAL_SPEED_KI_PITCH   3.9e-4f
#define GIMBAL_SPEED_KD_PITCH   0.0f
#define GIMBAL_ANGLE_KP_YAW     5.0f
#define GIMBAL_ANGLE_KI_YAW     0.1f
#define GIMBAL_ANGLE_KD_YAW     110.0f
#define GIMBAL_ANGLE_KP_PITCH   4.6f
#define GIMBAL_ANGLE_KI_PITCH   0.17f
#define GIMBAL_ANGLE_KD_PITCH   30.0f

#endif //GIMBAL_CONFIG_H
