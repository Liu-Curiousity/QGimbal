#include "BMI088.h"
#include "gpio.h"
#include "spi.h"
#include "cmsis_os2.h"

void BMI088::delay_ms(uint16_t ms) {
    osDelay(ms);
}

void BMI088::delay_us(uint16_t us)

 {
    osDelay(us / 1000 + 1);
}

void BMI088::gpio_init() {}

void BMI088::com_init() {
    accel_ns_h();
    gyro_ns_h();
}

void BMI088::accel_ns_l() {
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

void BMI088::accel_ns_h() {
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088::gyro_ns_l() {
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}

void BMI088::gyro_ns_h() {
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088::read_write_byte(uint8_t tx_data) {
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, 1, 1000);
    return rx_data;
}
