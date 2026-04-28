#include <cmath>
#include "BMI088.h"
#include "BMI088reg.h"

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_LONG_DELAY_TIME      80
#define BMI088_COM_WAIT_SENSOR_TIME 150

//3/32768*9.8
#define BMI088_ACCEL_3G_SEN     0.0008974358974f
#define BMI088_ACCEL_6G_SEN     0.00179443359375f
#define BMI088_ACCEL_12G_SEN    0.0035888671875f
#define BMI088_ACCEL_24G_SEN    0.007177734375f

//2000/32768/180*pi
#define BMI088_GYRO_2000_SEN    0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN    0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN     0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN     0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN     0.000066579027251980956150958662738366f

float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

static uint8_t write_accel_reg_data_error[][3] =
{
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088::ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088::ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088::ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088::ACC_RANGE_ERROR},
    {
        BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW,
        BMI088::INT1_IO_CTRL_ERROR
    },
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088::INT_MAP_DATA_ERROR}
};

static uint8_t write_gyro_reg_data_error[][3] =
{
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088::GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088::GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088::GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088::GYRO_CTRL_ERROR},
    {
        BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW,
        BMI088::GYRO_INT3_INT4_IO_CONF_ERROR
    },
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088::GYRO_INT3_INT4_IO_MAP_ERROR}
};

auto BMI088::init() -> ERROR_CODE {
    uint8_t error = NO_ERROR;
    // GPIO and SPI Init
    gpio_init();
    com_init();

    // self test pass and init
    error |= accel_self_test() != NO_ERROR ? SELF_TEST_ACCEL_ERROR : accel_init();
    error |= gyro_self_test() != NO_ERROR ? SELF_TEST_GYRO_ERROR : gyro_init();

    if (error == NO_ERROR) {
        initialized = true;
    }
    return static_cast<ERROR_CODE>(error);
}

auto BMI088::accel_init() -> ERROR_CODE {
    uint8_t res = 0;

    //check commiunication is normal
    accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    //accel software reset
    accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        return NO_SENSOR;
    }

    //set accel sonsor config and check
    for (const auto write_list : write_accel_reg_data_error) {
        accel_write_single_reg(write_list[0], write_list[1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        accel_read_single_reg(write_list[0], &res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_list[1]) {
            return static_cast<ERROR_CODE>(write_list[2]);
        }
    }
    return NO_ERROR;
}

auto BMI088::gyro_init() -> ERROR_CODE {
    uint8_t res = 0;

    //check commiunication is normal
    accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    //reset the gyro sensor
    gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        return NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (const auto write_list : write_gyro_reg_data_error) {
        gyro_write_single_reg(write_list[0], write_list[1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        gyro_read_single_reg(write_list[0], &res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_list[1]) {
            return static_cast<ERROR_CODE>(write_list[2]);
        }
    }
    return NO_ERROR;
}

auto BMI088::accel_self_test() -> ERROR_CODE {
    int16_t self_test_accel[2][3];

    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    uint8_t res = 0;

    static const uint8_t write_BMI088_ACCEL_self_test_Reg_Data_Error[6][3] =
    {
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, ACC_CONF_ERROR},
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G, ACC_RANGE_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, ACC_PWR_CONF_ERROR},
        {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL, ACC_PWR_CONF_ERROR},
        {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL, ACC_PWR_CONF_ERROR}
    };

    //check commiunication is normal
    accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // reset bmi088 accel sensor and wait for > 50ms
    accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal
    accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        return NO_SENSOR;
    }

    // set the accel register
    for (uint8_t write_reg_num = 0; write_reg_num < 4; write_reg_num++) {
        accel_write_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0],
                               write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        accel_read_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], &res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1]) {
            return static_cast<ERROR_CODE>(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][2]);
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        delay_ms(BMI088_LONG_DELAY_TIME);
    }

    // self test include postive and negative
    for (uint8_t write_reg_num = 0; write_reg_num < 2; write_reg_num++) {
        accel_write_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0],
                               write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        accel_read_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], &res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1]) {
            return static_cast<ERROR_CODE>(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][2]);
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        delay_ms(BMI088_LONG_DELAY_TIME);

        // read response accel
        accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

        self_test_accel[write_reg_num][0] = (int16_t)((buf[1]) << 8) | buf[0];
        self_test_accel[write_reg_num][1] = (int16_t)((buf[3]) << 8) | buf[2];
        self_test_accel[write_reg_num][2] = (int16_t)((buf[5]) << 8) | buf[4];
    }

    //set self test off
    accel_write_single_reg(BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_OFF);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    accel_read_single_reg(BMI088_ACC_SELF_TEST, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != (BMI088_ACC_SELF_TEST_OFF)) {
        return ACC_SELF_TEST_ERROR;
    }

    //reset the accel sensor
    accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);

    if ((self_test_accel[0][0] - self_test_accel[1][0] < 1365) ||
        (self_test_accel[0][1] - self_test_accel[1][1] < 1365) ||
        (self_test_accel[0][2] - self_test_accel[1][2] < 680)) {
        return SELF_TEST_ACCEL_ERROR;
    }

    accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    return NO_ERROR;
}

auto BMI088::gyro_self_test() -> ERROR_CODE {
    uint8_t res = 0;
    uint8_t retry = 0;
    //check commiunication is normal
    gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    //reset the gyro sensor
    gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    gyro_write_single_reg(BMI088_GYRO_SELF_TEST, BMI088_GYRO_TRIG_BIST);
    delay_ms(BMI088_LONG_DELAY_TIME);

    do {
        gyro_read_single_reg(BMI088_GYRO_SELF_TEST, &res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        retry++;
    } while (!(res & BMI088_GYRO_BIST_RDY) && retry < 10);

    if (retry == 10) {
        return SELF_TEST_GYRO_ERROR;
    }

    if (res & BMI088_GYRO_BIST_FAIL) {
        return SELF_TEST_GYRO_ERROR;
    }

    return NO_ERROR;
}

void BMI088::update_temperature(const uint8_t rx_buf[2]) {
    auto temperate_raw_temp = static_cast<int16_t>(rx_buf[0] << 3 | rx_buf[1] >> 5);
    if (temperate_raw_temp > 1023) temperate_raw_temp -= 2048;
    temperate = temperate_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

void BMI088::update_euler_angle(float q[4]) {
    yaw = std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    pitch = std::asin(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll = std::atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

void BMI088::update_accel(const uint8_t rx_buf[6]) {
    accel[0] = ((int16_t)(rx_buf[1] << 8) | rx_buf[0]) * BMI088_ACCEL_SEN;
    accel[1] = ((int16_t)(rx_buf[3] << 8) | rx_buf[2]) * BMI088_ACCEL_SEN;
    accel[2] = ((int16_t)(rx_buf[5] << 8) | rx_buf[4]) * BMI088_ACCEL_SEN;
}

void BMI088::update_gyro(const uint8_t rx_buf[6]) {
    gyro[0] = ((int16_t)(rx_buf[1] << 8) | rx_buf[0]) * BMI088_GYRO_SEN;
    gyro[1] = ((int16_t)(rx_buf[3] << 8) | rx_buf[2]) * BMI088_GYRO_SEN;
    gyro[2] = ((int16_t)(rx_buf[5] << 8) | rx_buf[4]) * BMI088_GYRO_SEN;
}

void BMI088::read() {
    get_accel();
    get_gyro();
    get_temperate();
}

void BMI088::get_sensor_time() {
    uint8_t buf[3];
    accel_read_muli_reg(BMI088_SENSORTIME_DATA_L, buf, 3);
    time = (buf[2] << 16 | buf[1] << 8 | buf[0]) * 39.0625f;
}

void BMI088::get_temperate() {
    uint8_t buf[2];
    accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
    auto temperate_raw_temp = static_cast<int16_t>(buf[0] << 3 | buf[1] >> 5);
    if (temperate_raw_temp > 1023) temperate_raw_temp -= 2048;
    temperate = temperate_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

void BMI088::get_gyro() {
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};

    gyro_read_muli_reg(BMI088_GYRO_X_L, buf, 6);
    gyro[0] = ((int16_t)(buf[1] << 8) | buf[0]) * BMI088_GYRO_SEN;
    gyro[1] = ((int16_t)(buf[3] << 8) | buf[2]) * BMI088_GYRO_SEN;
    gyro[2] = ((int16_t)(buf[5] << 8) | buf[4]) * BMI088_GYRO_SEN;
}

void BMI088::get_accel() {
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};

    accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
    accel[0] = ((int16_t)(buf[1] << 8) | buf[0]) * BMI088_ACCEL_SEN;
    accel[1] = ((int16_t)(buf[3] << 8) | buf[2]) * BMI088_ACCEL_SEN;
    accel[2] = ((int16_t)(buf[5] << 8) | buf[4]) * BMI088_ACCEL_SEN;
}

void BMI088::write_single_reg(const uint8_t reg, const uint8_t data) {
    read_write_byte(reg);
    read_write_byte(data);
}

void BMI088::read_single_reg(const uint8_t reg, uint8_t *data) {
    read_write_byte(reg | 0x80);
    *data = read_write_byte(0x55);
}

void BMI088::write_muli_reg(const uint8_t reg, const uint8_t *buf, uint8_t len) {
    read_write_byte(reg);
    while (len != 0) {
        read_write_byte(*buf);
        buf++;
        len--;
    }
}

void BMI088::read_muli_reg(const uint8_t reg, uint8_t *buf, uint8_t len) {
    read_write_byte(reg | 0x80);

    while (len != 0) {
        *buf = read_write_byte(0x55);
        buf++;
        len--;
    }
}

void BMI088::accel_write_single_reg(const uint8_t reg, const uint8_t data) {
    accel_ns_l();
    write_single_reg(reg, data);
    accel_ns_h();
}

void BMI088::accel_read_single_reg(const uint8_t reg, uint8_t *data) {
    accel_ns_l();
    read_write_byte(reg | 0x80);
    read_write_byte(0x55);
    *data = read_write_byte(0x55);
    accel_ns_h();
}

void BMI088::accel_write_muli_reg(const uint8_t reg, const uint8_t *data, const uint8_t len) {
    accel_ns_l();
    write_muli_reg(reg, data, len);
    accel_ns_h();
}

void BMI088::accel_read_muli_reg(const uint8_t reg, uint8_t *data, const uint8_t len) {
    accel_ns_l();
    read_write_byte(reg | 0x80);
    read_muli_reg(reg, data, len);
    accel_ns_h();
}

void BMI088::gyro_write_single_reg(const uint8_t reg, const uint8_t data) {
    gyro_ns_l();
    write_single_reg(reg, data);
    gyro_ns_h();
}

void BMI088::gyro_read_single_reg(const uint8_t reg, uint8_t *data) {
    gyro_ns_l();
    read_single_reg(reg, data);
    gyro_ns_h();
}

void BMI088::gyro_write_muli_reg(const uint8_t reg, const uint8_t *data, const uint8_t len) {
    gyro_ns_l();
    write_muli_reg(reg, data, len);
    gyro_ns_h();
}

void BMI088::gyro_read_muli_reg(const uint8_t reg, uint8_t *data, const uint8_t len) {
    gyro_ns_l();
    read_muli_reg(reg, data, len);
    gyro_ns_h();
}
