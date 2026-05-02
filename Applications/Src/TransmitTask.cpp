//
// Created by 26757 on 2025/12/29.
//
#include <algorithm>
#include <cstring>
#include "task_public.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "Gimbal.h"
#include "queue.h"

// 左下角为正方向
struct TransmitPackage {
    float imu_angles[3];       // yaw, pitch, roll, in rad
    float yaw_imu_angle;       // end point yaw angle in rad
    float pitch_imu_angle;     // end point pitch angle in rad
    float yaw_motor_angle;     // yaw motor angle in rad
    float pitch_motor_angle;   // pitch motor angle in rad
    uint8_t laser_enabled;     // 0: disabled, 1: enabled
    uint8_t enabled;           // 0: disabled, 1: enabled
    uint8_t stability_enabled; // 0: disabled, 1: enabled
    uint8_t check_sum;         // checksum
} transmit_package;

struct ReceivePackage {
    float yaw_speed;           // in rpm
    float pitch_speed;         // in rpm
    uint8_t laser_enabled;     // 0: disable, 1: enable, other: no action
    uint8_t enabled;           // 0: disable, 1: enable, other: no action
    uint8_t stability_enabled; // 0: disable, 1: enable, other: no action
    uint8_t check_sum;         // checksum
};

xQueueHandle receive_package_queue;

extern float INS_angle[3]; // yaw,pitch,roll
extern Gimbal gimbal;      // 云台
extern QD4310 YawMotor;    // 云台偏航电机
extern QD4310 PitchMotor;  // 云台俯仰电机

uint8_t UART6_RxBuffer[sizeof(ReceivePackage)];

void StartTransmitTask(void *argument) {
    while (true) {
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {}
        transmit_package.laser_enabled = HAL_GPIO_ReadPin(Laser_En_GPIO_Port, Laser_En_Pin);
        transmit_package.enabled = gimbal.enabled;
        transmit_package.stability_enabled = gimbal.stability_enabled;
        transmit_package.imu_angles[0] = INS_angle[0];
        transmit_package.imu_angles[1] = INS_angle[1];
        transmit_package.imu_angles[2] = INS_angle[2];
        transmit_package.yaw_imu_angle = gimbal.imu_angle.yaw;
        transmit_package.pitch_imu_angle = gimbal.imu_angle.pitch;
        transmit_package.yaw_motor_angle = YawMotor.angle;
        transmit_package.pitch_motor_angle = PitchMotor.angle;
        // 计算校验和
        uint8_t checksum = 0;
        for (size_t i = 0; i < sizeof(TransmitPackage) - sizeof(uint8_t); ++i) {
            checksum += reinterpret_cast<uint8_t *>(&transmit_package)[i];
        }
        transmit_package.check_sum = checksum;
        HAL_UART_Transmit_DMA(&huart6, reinterpret_cast<const uint8_t *>(&transmit_package), sizeof(TransmitPackage));
    }
}

void StartReceiveTask(void *argument) {
    receive_package_queue = xQueueCreate(5, sizeof(ReceivePackage));
    ReceivePackage receive_package{};
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, UART6_RxBuffer, sizeof(ReceivePackage));
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT); // 关闭DMA半传输中断
    while (true) {
        xQueueReceive(receive_package_queue, &receive_package, portMAX_DELAY);
        // 校验和
        uint8_t checksum = 0;
        for (size_t i = 0; i < sizeof(ReceivePackage) - sizeof(uint8_t); ++i) {
            checksum += UART6_RxBuffer[i];
        }
        if (checksum == reinterpret_cast<ReceivePackage *>(UART6_RxBuffer)->check_sum) {
            if (receive_package.laser_enabled == 0 || receive_package.laser_enabled == 1)
                HAL_GPIO_WritePin(Laser_En_GPIO_Port, Laser_En_Pin,
                                  receive_package.laser_enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
            if (receive_package.enabled == 0 || receive_package.enabled == 1) {
                receive_package.enabled ? gimbal.enable() : gimbal.disable();
            }
            if (receive_package.stability_enabled == 0 || receive_package.stability_enabled == 1) {
                receive_package.stability_enabled ? gimbal.enable_stability() : gimbal.disable_stability();
            }
            gimbal.Ctrl(Gimbal::CtrlType::LowSpeedCtrl,
                        {std::clamp(receive_package.yaw_speed, -50.0f, 50.0f),
                        std::clamp(receive_package.pitch_speed, -50.0f, 50.0f)});
        }
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    BaseType_t xHigherPriorityTaskWoken;
    if (huart->Instance == huart6.Instance) {
        if (Size == sizeof(ReceivePackage)) {
            xQueueSendToBackFromISR(receive_package_queue, UART6_RxBuffer, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        std::fill_n(UART6_RxBuffer, sizeof(UART6_RxBuffer), 0);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, UART6_RxBuffer, sizeof(ReceivePackage));
        __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT); // 关闭DMA半传输中断
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart6) {
        __HAL_UNLOCK(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_RxBuffer, sizeof(UART6_RxBuffer));
        __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT); // 关闭DMA半传输中断
    }
}

/**
 * @brief 反转字节(按bit反转)
 * */
static uint8_t ReverseBits(uint8_t data) {
    data = ((data & 0x55) << 1) | ((data & 0xAA) >> 1);
    data = ((data & 0x33) << 2) | ((data & 0xCC) >> 2);
    data = ((data & 0x0F) << 4) | ((data & 0xF0) >> 4);
    return data;
}

uint8_t CRC8(const uint8_t *data, uint32_t len, uint8_t polynomial, uint8_t init,
             uint8_t xor_out, bool input_invert, bool output_invert) {
    /**1.校验参数**/
    assert_param(data != nullptr);
    assert_param(len > 0);
    /**2.变量定义**/
    uint8_t crc = init;
    /**3.计算**/
    do {
        crc ^= input_invert ? ReverseBits(*(data++)) : *(data++);
        for (uint8_t i = 0; i < 8; ++i) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
    } while (--len);
    return output_invert ? ReverseBits(crc ^ xor_out) : (crc ^ xor_out);
}
