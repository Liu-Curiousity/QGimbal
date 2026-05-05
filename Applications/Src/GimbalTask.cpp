//
// Created by 26757 on 2025/12/28.
//
#include "task_public.h"
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "QD4310.h"
#include "PID.h"
#include "QGimbal.h"
#include "Gimbal_config.h"
#include "BMI088.h"
#include "Storage_EmbeddedFlash.h"
#include "adc.h"

extern BMI088 bmi088;

QD4310 YawMotor(&hcan1, 0x00);   // 云台偏航电机
QD4310 PitchMotor(&hcan1, 0x01); // 云台俯仰电机

QGimbal qgimbal(
    {YawMotor, PitchMotor},
    {
        PID{
            PID::PID_type::position_type,
            GIMBAL_SPEED_KP_YAW,
            GIMBAL_SPEED_KI_YAW,
            GIMBAL_SPEED_KD_YAW,
            2e3f,
            -2e3f,
            GIMBAL_MAX_CURRENT,
            -GIMBAL_MAX_CURRENT
        },
        PID{
            PID::PID_type::position_type,
            GIMBAL_SPEED_KP_PITCH,
            GIMBAL_SPEED_KI_PITCH,
            GIMBAL_SPEED_KD_PITCH,
            2e3f,
            -2e3f,
            GIMBAL_MAX_CURRENT,
            -GIMBAL_MAX_CURRENT
        }
    },
    {
        PID{
            PID::PID_type::position_type,
            GIMBAL_ANGLE_KP_YAW,
            GIMBAL_ANGLE_KI_YAW,
            GIMBAL_ANGLE_KD_YAW,
            1.8f, -1.8f,
            GIMBAL_MAX_CURRENT,
            -GIMBAL_MAX_CURRENT
        },
        PID{
            PID::PID_type::position_type,
            GIMBAL_ANGLE_KP_PITCH,
            GIMBAL_ANGLE_KI_PITCH,
            GIMBAL_ANGLE_KD_PITCH,
            1.8f, -1.8f,
            GIMBAL_MAX_CURRENT,
            -GIMBAL_MAX_CURRENT
        }
    },
    0.001f, storage
);

void CAN_InterfaceInit();

void StartGimbalTask(void *argument) {
    CAN_InterfaceInit();
    HAL_ADC_Start(&hadc3);

    // 等待第一次接收到数据，表明IMU初始化完成，此时启动gimbal
    while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {}
    qgimbal.init();
    qgimbal.enable();
    while (true) {
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {}
        qgimbal.Ctrl_ISR({bmi088.yaw, bmi088.pitch});

        if (__HAL_ADC_GET_FLAG(&hadc3, ADC_FLAG_EOC)) {
            qgimbal.updateVoltage(HAL_ADC_GetValue(&hadc3) / 4095.0f * 3.3f / 22 * 222);
            HAL_ADC_Start(&hadc3);
        }
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
