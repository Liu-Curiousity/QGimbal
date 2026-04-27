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

/*==========================配置参数==========================*/
#define GIMBAL_MAX_SPEED           1000.0f   // 最大转速,单位rpm

#define GIMBAL_SPEED_KP            3e-3f
#define GIMBAL_SPEED_KI            7.8e-5f
#define GIMBAL_SPEED_KD            0.0f
#define GIMBAL_ANGLE_KP            1200.0f
#define GIMBAL_ANGLE_KI            0.0f
#define GIMBAL_ANGLE_KD            0.0f

#endif //GIMBAL_CONFIG_H
