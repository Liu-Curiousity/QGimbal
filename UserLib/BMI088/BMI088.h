#pragma once

#include <cstdint>

class BMI088 {
public:
    virtual ~BMI088() = default;

    enum ERROR_CODE: uint8_t {
        NO_ERROR = 0x00,
        ACC_PWR_CTRL_ERROR = 0x01,
        ACC_PWR_CONF_ERROR = 0x02,
        ACC_CONF_ERROR = 0x03,
        ACC_SELF_TEST_ERROR = 0x04,
        ACC_RANGE_ERROR = 0x05,
        INT1_IO_CTRL_ERROR = 0x06,
        INT_MAP_DATA_ERROR = 0x07,
        GYRO_RANGE_ERROR = 0x08,
        GYRO_BANDWIDTH_ERROR = 0x09,
        GYRO_LPM1_ERROR = 0x0A,
        GYRO_CTRL_ERROR = 0x0B,
        GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
        GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,
        SELF_TEST_ACCEL_ERROR = 0x80,
        SELF_TEST_GYRO_ERROR = 0x40,
        NO_SENSOR = 0xFF,
    };

    BMI088() = default;

    bool initialized = false;
    float yaw, pitch, roll; // euler angle, unit rad.
    float accel[3]{};
    float gyro[3]{};
    float temperate{};
    float time{};

    // need to be implemented by user
    void delay_ms(uint16_t ms);
    void delay_us(uint16_t us);
    void gpio_init();
    void com_init();
    void accel_ns_l();
    void accel_ns_h();
    void gyro_ns_l();
    void gyro_ns_h();
    uint8_t read_write_byte(uint8_t tx_data);

    ERROR_CODE init();
    ERROR_CODE accel_self_test();
    ERROR_CODE gyro_self_test();
    ERROR_CODE accel_init();
    ERROR_CODE gyro_init();
    void read();
    void get_sensor_time();
    void get_temperate();
    void get_gyro();
    void get_accel();

    void update_euler_angle(float q[4]);
    void update_accel(const uint8_t rx_buf[6]);
    void update_gyro(const uint8_t rx_buf[6]);
    void update_temperature(const uint8_t rx_buf[2]);

private:
    void write_single_reg(uint8_t reg, uint8_t data);
    void read_single_reg(uint8_t reg, uint8_t *data);
    void write_muli_reg(uint8_t reg, const uint8_t *buf, uint8_t len);
    void read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

    void accel_write_single_reg(uint8_t reg, uint8_t data);
    void accel_read_single_reg(uint8_t reg, uint8_t *data);
    void accel_write_muli_reg(uint8_t reg, const uint8_t *data, uint8_t len);
    void accel_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);

    void gyro_write_single_reg(uint8_t reg, uint8_t data);
    void gyro_read_single_reg(uint8_t reg, uint8_t *data);
    void gyro_write_muli_reg(uint8_t reg, const uint8_t *data, uint8_t len);
    void gyro_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
};
