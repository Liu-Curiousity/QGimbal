//
// Created by 26757 on 2025/12/28.
//
#include "task_public.h"
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "QD4310.h"
#include "PID.h"
#include "Gimbal.h"

constexpr static float yaw_center = 0.0f;   // 云台偏航中心位置,单位: rad
constexpr static float pitch_center = 0.0f; // 云台俯仰中心位置,单位: rad

extern float INS_angle[3]; // yaw,pitch,roll

QD4310 YawMotor(&hcan1, 0x00);   // 云台偏航电机
QD4310 PitchMotor(&hcan1, 0x01); // 云台俯仰电机

Gimbal gimbal(
    {YawMotor, PitchMotor},
    {yaw_center, pitch_center},
    {
        PID{
            PID::PID_type::position_type,
            0.0f, 0.f, 0.0f,
        },
        PID{
            PID::PID_type::position_type,
            0.0f, 0.0f, 0.0f,
        }
    },
    {
        PID{
            PID::PID_type::position_type,
            5.0f, 0.1f, 110.0f,
            1.8f, -1.8f,
            1, -1
        },
        PID{
            PID::PID_type::position_type,
            4.6f, 0.17f, 30.0f,
            1.8f, -1.8f,
            1, -1
        }
    },
    0.001f
);

void CAN_InterfaceInit();

void StartGimbalTask(void *argument) {
    CAN_InterfaceInit();
    YawMotor.enable();
    PitchMotor.enable();

    // 上电复位云台角度
    YawMotor.setAngle(yaw_center);
    PitchMotor.setAngle(pitch_center);
    // 等待陀螺仪初始化完成
    osDelay(pdMS_TO_TICKS(2000));
    YawMotor.setAngle(yaw_center);
    PitchMotor.setAngle(pitch_center);

    HAL_GPIO_WritePin(Laser_En_GPIO_Port,Laser_En_Pin, GPIO_PIN_SET); // 使能激光
    gimbal.enable();
    osDelay(50);
    gimbal.enable_stability();
    while (true) {
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {}
        gimbal.Ctrl_ISR({INS_angle[0], INS_angle[1]});
        xTaskNotifyGive((TaskHandle_t)TransmitTaskHandle); // 通知发送任务发送数据
    }
}

void CAN_InterfaceInit() {
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) Error_Handler();
    if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        if (rx_header.StdId >= 0x500 && rx_header.StdId <= 0x508) {
            if (rx_header.StdId == 0x500) {
                YawMotor.update(rx_data);
            } else if (rx_header.StdId == 0x501) {
                PitchMotor.update(rx_data);
            }
        }
    }
}
