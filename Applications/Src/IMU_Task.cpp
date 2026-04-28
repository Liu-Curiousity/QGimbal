#include "task_public.h"
#include "cmsis_os.h"
#include "task.h"
#include "BMI088.h"
#include "MahonyAHRS.h"
#include "spi.h"

#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_TEMP_LENGHT       4


#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS    2

uint8_t gyro_dma_rx_buf[8];
uint8_t gyro_dma_tx_buf[8] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[8];
uint8_t accel_dma_tx_buf[8] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t temp_dma_rx_buf[4];
uint8_t temp_dma_tx_buf[4] = {0xA2, 0xFF, 0xFF, 0xFF};

volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;

BMI088 bmi088;

/**
  * @brief          imu task, init bmi088, calculate the euler angle
  * @param[in]      argument: NULL
  * @retval         none
  */
void StartImuTask(void *argument) {
    while (bmi088.init()) {
        osDelay(100);
    }

    MahonyAHRS AHRS{1000};

    while (true) {
        //wait spi tansmit done
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {}

        if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            bmi088.update_gyro(gyro_dma_rx_buf + 1);
        }
        if (accel_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            bmi088.update_accel(accel_dma_rx_buf + 2);
        }

        if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            bmi088.update_temperature(temp_dma_rx_buf + 2);
        }

        bmi088.update_gyro(gyro_dma_rx_buf + 1);
        bmi088.update_accel(accel_dma_rx_buf + 2);
        bmi088.update_temperature(temp_dma_rx_buf + 2);

        AHRS.update(bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2],
                    bmi088.accel[0], bmi088.accel[1], bmi088.accel[2]);
        bmi088.update_euler_angle(AHRS.q);
        xTaskNotifyGive((TaskHandle_t)GimbalTaskHandle);
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
                vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(ImuTaskHandle), &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (bmi088.initialized) {
        if (GPIO_Pin == INT1_ACCEL_Pin) {
            accel_update_flag |= 1 << IMU_DR_SHFITS;
            accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
            imu_cmd_spi_dma();
        } else if (GPIO_Pin == INT1_GYRO_Pin) {
            gyro_update_flag |= 1 << IMU_DR_SHFITS;
            imu_cmd_spi_dma();
        }
    }
}