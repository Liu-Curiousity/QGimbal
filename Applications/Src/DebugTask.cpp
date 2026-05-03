#include "task_public.h"
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "QD4310.h"
#include "Gimbal.h"

extern QD4310 YawMotor;   // 云台偏航电机
extern QD4310 PitchMotor; // 云台俯仰电机
extern Gimbal gimbal;

constexpr static float yaw_center = 0.0f;   // 云台偏航中心位置,单位: rad
constexpr static float pitch_center = 0.0f; // 云台俯仰中心位置,单位: rad

void StartDebugTask(void *argument) {
    osDelay(50);
    // 上电复位云台角度
    gimbal.enable();
    gimbal.Ctrl(Gimbal::CtrlType::AngleCtrl, {0, 0});
    // 等待陀螺仪初始化完成
    osDelay(pdMS_TO_TICKS(2000));
    gimbal.enable_stability();
    HAL_GPIO_WritePin(Laser_En_GPIO_Port,Laser_En_Pin, GPIO_PIN_SET); // 使能激光
    gimbal.Ctrl(Gimbal::CtrlType::LowSpeedCtrl, {0, 0});
    while (true) {
        osDelay(pdMS_TO_TICKS(2000));
    }
}
