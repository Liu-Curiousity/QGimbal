#include <algorithm>
#include <cstring>
#include "task_public.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "Gimbal.h"
#include "queue.h"
#include "sys_public.h"

// 左下角为正方向
struct __attribute__((packed)) TxPackage {
    uint8_t status;                           // bit 0: enabled, bit 1:stability_enabled, bit 2: laser_enabled
    Gimbal::gimbal_pair<float> imu_speed;     // end point speed, in rpm
    Gimbal::gimbal_pair<float> imu_angle;     // end point angle, in rad
    Gimbal::gimbal_pair<float> motor_current; // motor current, in A
    Gimbal::gimbal_pair<float> motor_speed;   // motor speed, in rpm
    Gimbal::gimbal_pair<float> motor_angle;   // motor angle, in rad
    uint8_t crc8;                             // CRC8
} tx_package;

enum class CmdType : uint8_t {
    NOP = 0x00,              // 无操作
    Enable = 0x01,           // 使能
    Disable = 0x02,          // 失能
    CurrentCtrl = 0x03,      // 电流控制
    SpeedCtrl = 0x04,        // 速度控制
    AngleCtrl = 0x05,        // 角度控制
    LowSpeedCtrl = 0x06,     // 低速控制
    StepAngleCtrl = 0x07,    // 角度递增控制
    EnableStability = 0xFF,  // 使能自稳
    DisableStability = 0xFE, // 失能自稳
    EnableLaser = 0xFD,      // 使能激光
    DisableLaser = 0xFC,     // 失能激光
    ResetIMU = 0xFB,         // 复位IMU(角度调零)
};

struct __attribute__((packed)) RxPackage {
    CmdType cmd_type;                  // 命令类型
    Gimbal::gimbal_pair<int16_t> data; // 命令数据
};

extern Gimbal gimbal;     // 云台

xQueueHandle receive_package_queue;
uint8_t UART6_RxBuffer[sizeof(RxPackage)];

uint8_t CRC8(const uint8_t *data, uint32_t len, uint8_t polynomial, uint8_t init,
             uint8_t xor_out, bool input_invert, bool output_invert);

void StartCommunicateTask(void *argument) {
    receive_package_queue = xQueueCreate(5, sizeof(RxPackage));
    RxPackage rx_package{};

    // 1.等待gimbal使能
    while (!gimbal.enabled)
        delay_ms(10);

    // 2.打开串口
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, UART6_RxBuffer, sizeof(UART6_RxBuffer));
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT); // 关闭DMA半传输中断

    while (true) {
        xQueueReceive(receive_package_queue, &rx_package, portMAX_DELAY);

        switch (rx_package.cmd_type) {
            case CmdType::NOP: // NOP指令,只发送反馈报文
                break;
            case CmdType::Enable: // 使能指令
                gimbal.start();
                break;
            case CmdType::Disable: // 失能指令
                gimbal.stop();
                break;
            case CmdType::CurrentCtrl: // 电流控制
                gimbal.Ctrl(Gimbal::CtrlType::CurrentCtrl,
                            {
                                rx_package.data.yaw * 10.0f / INT16_MAX,
                                rx_package.data.pitch * 10.0f / INT16_MAX
                            });
                break;
            case CmdType::SpeedCtrl: // 速度控制
                gimbal.Ctrl(Gimbal::CtrlType::SpeedCtrl,
                            {
                                rx_package.data.yaw * 1000.0f / INT16_MAX,
                                rx_package.data.pitch * 1000.0f / INT16_MAX
                            });
                break;
            case CmdType::AngleCtrl: // 角度控制
                gimbal.Ctrl(Gimbal::CtrlType::AngleCtrl,
                            {
                                rx_package.data.yaw * 2 * std::numbers::pi_v<float> / UINT16_MAX,
                                rx_package.data.pitch * 2 * std::numbers::pi_v<float> / UINT16_MAX
                            });
                break;
            case CmdType::LowSpeedCtrl: // 低速控制
                gimbal.Ctrl(Gimbal::CtrlType::LowSpeedCtrl,
                            {
                                rx_package.data.yaw * 1000.0f / INT16_MAX,
                                rx_package.data.pitch * 1000.0f / INT16_MAX
                            });
                break;
            case CmdType::StepAngleCtrl: // 角度递增
                gimbal.Ctrl(Gimbal::CtrlType::StepAngleCtrl,
                            {
                                rx_package.data.yaw * 2 * std::numbers::pi_v<float> / INT16_MAX,
                                rx_package.data.pitch * 2 * std::numbers::pi_v<float> / INT16_MAX
                            });
                break;
            case CmdType::EnableStability: // 使能自稳
                gimbal.enable_stability();
                break;
            case CmdType::DisableStability: // 失能自稳
                gimbal.disable_stability();
                break;
            case CmdType::EnableLaser: // 使能激光
                HAL_GPIO_WritePin(Laser_En_GPIO_Port, Laser_En_Pin, GPIO_PIN_SET);
                break;
            case CmdType::DisableLaser: // 失能激光
                HAL_GPIO_WritePin(Laser_En_GPIO_Port, Laser_En_Pin, GPIO_PIN_RESET);
                break;
            case CmdType::ResetIMU: // 复位IMU(角度调零)
                gimbal.reset_imu();
                break;
            default:
                break;
        }
        // 是合法命令则发送反馈报文
        if (rx_package.cmd_type <= CmdType::StepAngleCtrl ||
            rx_package.cmd_type >= CmdType::ResetIMU) {
            tx_package.status = gimbal.enabled | gimbal.stability_enabled << 1 |
                                HAL_GPIO_ReadPin(Laser_En_GPIO_Port, Laser_En_Pin) << 2;
            tx_package.imu_speed = gimbal.imu_speed;
            tx_package.imu_angle = gimbal.imu_angle;
            tx_package.motor_current = gimbal.motor_current;
            tx_package.motor_speed = gimbal.motor_speed;
            tx_package.motor_angle = gimbal.motor_angle;
            tx_package.crc8 = CRC8(reinterpret_cast<const uint8_t *>(&tx_package), sizeof(tx_package) - 1,
                                   0x07, 0x00, 0x00, false, false);
            HAL_UART_Transmit_DMA(&huart6, reinterpret_cast<const uint8_t *>(&tx_package),
                                  sizeof(TxPackage));
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    BaseType_t xHigherPriorityTaskWoken;
    if (huart->Instance == huart6.Instance) {
        // 如果数据长度匹配且CRC8校验通过,进行处理
        if (Size == sizeof(RxPackage) + 1 &&
            CRC8(UART6_RxBuffer, 4, 0x07, 0x00, 0x00, false, false) == UART6_RxBuffer[4]) {
            // cmd:1 byte, data:4 bytes, crc8:1 byte
            xQueueSendToBackFromISR(receive_package_queue, UART6_RxBuffer, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        std::fill_n(UART6_RxBuffer, sizeof(UART6_RxBuffer), 0);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, UART6_RxBuffer, sizeof(UART6_RxBuffer));
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
