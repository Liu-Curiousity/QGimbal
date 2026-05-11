/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       IMU_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include <numbers>
#include "IMU_Task.h"
#include "task_public.h"
#include "cmsis_os.h"
#include "task.h"
#include "bsp_spi.h"
#include "BMI088Driver/BMI088driver.h"
#include "MahonyAHRS.h"

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
static void imu_cmd_spi_dma(void);

void get_angle(float quat[4], float *yaw, float *pitch, float *roll);

// TODO: 临时解决方案
static float wrap(float value, const float min, const float max) {
    value = std::fmod(value - min, max - min);
    return value < 0 ? value + max : value + min;
}

float yaw_bias = 0.0f;
float delta_yaw_bias = -4.9e-7f;

extern SPI_HandleTypeDef hspi1;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t temp_dma_rx_buf[SPI_DMA_TEMP_LENGHT];
uint8_t temp_dma_tx_buf[SPI_DMA_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};


volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

bmi088_real_data_t bmi088_real_data;

float INS_angle[3]; //euler angle, unit rad.

/**
  * @brief          imu task, init bmi088, calculate the euler angle
  * @param[in]      argument: NULL
  * @retval         none
  */
void InsTask(void *argument) {
    osDelay(InsTask_INIT_TIME);

    while (BMI088_init()) {
        osDelay(100);
    }

    MahonyAHRS AHRS{1000};

    SPI1_DMA_init();

    imu_start_dma_flag = 1;
    while (true) {
        //wait spi DMA tansmit done
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {}

        if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }
        if (accel_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel,
                                   &bmi088_real_data.time);
        }

        if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
                                         &bmi088_real_data.temp);
        }

        // 倒拿检测与修正
        if (bmi088_real_data.accel[2] < 0) {  // 假设 z 轴向下为正
            bmi088_real_data.accel[0] = -bmi088_real_data.accel[0];  // 反转 x
            bmi088_real_data.accel[1] = -bmi088_real_data.accel[1];  // 反转 y
            bmi088_real_data.accel[2] = -bmi088_real_data.accel[2];  // 反转 z
        }

        AHRS.update(bmi088_real_data.gyro[0], bmi088_real_data.gyro[1], bmi088_real_data.gyro[2],
                    bmi088_real_data.accel[0], bmi088_real_data.accel[1], bmi088_real_data.accel[2]);
        get_angle(AHRS.q.data(), INS_angle, INS_angle + 1, INS_angle + 2);
        yaw_bias += delta_yaw_bias;
        INS_angle[0] = wrap(INS_angle[0] - yaw_bias, -std::numbers::pi_v<float>, std::numbers::pi_v<float>);
        xTaskNotifyGive((TaskHandle_t)GimbalTaskHandle);
    }
}

void get_angle(float q[4], float *yaw, float *pitch, float *roll) {
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == INT1_ACCEL_Pin) {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag) {
            imu_cmd_spi_dma();
        }
    } else if (GPIO_Pin == INT1_GYRO_Pin) {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag) {
            imu_cmd_spi_dma();
        }
    }
}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
static void imu_cmd_spi_dma() {
    // 开启陀螺仪的DMA传输
    if ((gyro_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(
            hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))) {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        return;
    }
    // 开启加速度计的DMA传输
    if ((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(
            hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))) {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        return;
    }

    if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(
            hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS))) {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)temp_dma_tx_buf, (uint32_t)temp_dma_rx_buf, SPI_DMA_TEMP_LENGHT);
        return;
    }
}

extern "C" void DMA2_Stream0_IRQHandler() {
    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET) {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        if (gyro_update_flag & (1 << IMU_SPI_SHFITS)) {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }

        //accel read over
        if (accel_update_flag & (1 << IMU_SPI_SHFITS)) {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS)) {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
                static BaseType_t xHigherPriorityTaskWoken;
                vTaskNotifyGiveFromISR((TaskHandle_t)InsHandle, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
}
