#include <algorithm>
#include "task_public.h"
#include "main.h"
#include "cmsis_os.h"

#include "usart.h"

uint8_t UART6_RxBuffer[16];
float offset_x, offset_y;

void DebugTask(void *argument) {
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, UART6_RxBuffer, 16);
    while (true) {
        osDelay(pdMS_TO_TICKS(2000));
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == huart6.Instance) {
        if (Size == 13) {
            // 右下为正方向
            offset_x = std::strtof(reinterpret_cast<const char *>(UART6_RxBuffer), nullptr);
            offset_y = std::strtof(reinterpret_cast<const char *>(UART6_RxBuffer + 7), nullptr);
        }
        std::fill_n(UART6_RxBuffer, 16, 0);
        HAL_UARTEx_ReceiveToIdle_IT(&huart6, UART6_RxBuffer, 16);
    }
}
