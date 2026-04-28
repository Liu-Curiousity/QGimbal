/**
 * @brief 		FreeRTOS任务公共头文件
 * @detail
 * @author 	    Haoqi Liu
 * @date        24-11-24
 * @version 	V1.0.0
 * @note        任务函数必须在此文件定义,否则在app_freertos.c中找不到该函数符号
 * @warning
 * @par 		历史版本
                V1.0.0创建于24-11-24
 * */

#ifndef TASK_PUBLIC_H
#define TASK_PUBLIC_H

#ifdef __cplusplus
extern "C" {
#endif

/**=======================================C头文件内容======================================**/
#include "cmsis_os2.h"

//任务句柄和任务函数声明
//任务函数必须在此声明(在extern "C"块中),否则在app_freertos.c中找不到该函数符号
void StartDebugTask(void *argument);
void StartGimbalTask(void *argument);
void StartTransmitTask(void *argument);
void StartReceiveTask(void *argument);
void StartStartShell(void *argument);
void StartImuTask(void *argument);

extern osThreadId_t DebugTaskHandle;
extern osThreadId_t GimbalTaskHandle;
extern osThreadId_t TransmitTaskHandle;
extern osThreadId_t ReceiveTaskHandle;
extern osThreadId_t StartShellHandle;
extern osThreadId_t ImuTaskHandle;
/**======================================================================================**/

#ifdef __cplusplus
};
#endif

#endif //TASK_PUBLIC_H
