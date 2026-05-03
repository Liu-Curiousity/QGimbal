#include "task_public.h"
#include "main.h"
#include "cmsis_os.h"
#include "Gimbal.h"
#include "sys_public.h"

extern Gimbal gimbal;

void StartDebugTask(void *argument) {
    // 1.等待gimbal使能
    while (!gimbal.enabled)
        delay_ms(10);
    gimbal.start();
    gimbal.Ctrl(Gimbal::CtrlType::AngleCtrl, {0, 0});
    // 等待陀螺仪初始化完成
    osDelay(pdMS_TO_TICKS(2000));
    gimbal.reset_imu(); // 重新设置陀螺仪零点
    osDelay(pdMS_TO_TICKS(100));
    gimbal.enable_stability();
    HAL_GPIO_WritePin(Laser_En_GPIO_Port,Laser_En_Pin, GPIO_PIN_SET); // 使能激光
    gimbal.Ctrl(Gimbal::CtrlType::AngleCtrl, {0, 0});
    while (true) {
        osDelay(pdMS_TO_TICKS(2000));
    }
}
