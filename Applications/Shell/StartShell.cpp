/**
 * @file        StartShell.cpp
 * @brief       启动shell
 * @details
 * @author      Liu-Curiousity (2675794963@qq.com)
 * @date        2025-7-8
 * @version     V1.1.0
 * @note
 * @warning
 * @par         历史版本:
 *		        V1.0.0创建于2025-6-21
 *		        V1.1.0创建于2025-7-8
 * @copyright   (c) 2025 QDrive
 */

#include "task_public.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "shell_cpp.h"
#include "retarget.h"
#include "FreeRTOS.h"
#include "task.h"
#include "qgimbal.h"

Shell shell;
char shellBuffer[1024];
extern QGimbal qgimbal; // 云台

void USB_Disconnected() {
    __HAL_RCC_USB_OTG_FS_FORCE_RESET();
    osDelay(200);
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();

    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_Initure.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_PULLDOWN;
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_Initure);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    osDelay(300);
}

void StartStartShell(void *argument) {
    // 1.等待gimbal使能
    while (!qgimbal.enabled)
        osDelay(10);

    // 2.启动shell
    USB_Disconnected(); //USB重枚举
    MX_USB_DEVICE_Init();
    shell.read = shellRead;
    shell.write = shellWrite;
    shellInit(&shell, shellBuffer, sizeof(shellBuffer));
    xTaskCreate(shellTask, "LetterShellTask", 256, &shell, osPriorityNormal, nullptr);
    vTaskDelete(nullptr);
}
