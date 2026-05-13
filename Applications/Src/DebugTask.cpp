#include "task_public.h"
#include "main.h"
#include "cmsis_os.h"
#include "QGimbal.h"
#include "sys_public.h"

extern QGimbal qgimbal;

void StartDebugTask(void *argument) {
    // // 1.等待gimbal使能
    // while (!qgimbal.enabled)
    //     delay_ms(10);
    // qgimbal.start();
    // qgimbal.Ctrl(Gimbal::CtrlType::AngleCtrl, {0, 0});
    // // 等待陀螺仪初始化完成
    // osDelay(pdMS_TO_TICKS(2000));
    // qgimbal.reset_imu(); // 重新设置陀螺仪零点
    // osDelay(pdMS_TO_TICKS(100));
    // qgimbal.enable_stability();
    // qgimbal.enable_laser();
    // qgimbal.Ctrl(Gimbal::CtrlType::AngleCtrl, {0, 0});
    while (true) {
        osDelay(pdMS_TO_TICKS(2000));
    }
}
