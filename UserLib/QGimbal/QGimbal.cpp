/**
 * @file        QGimbal.cpp
 * @brief       QGimbal电机控制库
 * @details
 * @author      Liu-Curiousity (2675794963@qq.com)
 * @date        2026-5-5
 * @version     V1.0.0
 * @note
 * @warning
 * @par         历史版本:
 *		        V1.0.0创建于2026-5-5, 将Gimbal非核心功能剥离,使用QGimbal类集Gimbal实现解耦
 * @copyright   (c) 2026 QDrive
 */

#include "QGimbal.h"
#include "Gimbal_config.h"
#include <algorithm>
#include <numbers>
#include "usart.h"

using namespace std;

void QGimbal::init() {
    // 1.初始化Gimbal
    Gimbal::init();
    // 2.初始化flash
    if (!storage.initialized)
        storage.init();
    // 3.从flash中读取校准数据
    load_storage_calibration();
}

void QGimbal::start() {
    Gimbal::start();
    // TODO: 添加灯效
    // if (started) HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
}

void QGimbal::stop() {
    Gimbal::stop();
    // TODO: 添加灯效
    // HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
}

void QGimbal::enable_laser() {
    HAL_GPIO_WritePin(Laser_En_GPIO_Port, Laser_En_Pin, GPIO_PIN_SET);
    laser_enabled = true;
}

void QGimbal::disable_laser() {
    HAL_GPIO_WritePin(Laser_En_GPIO_Port, Laser_En_Pin, GPIO_PIN_RESET);
    laser_enabled = false;
}

// TODO: 后续添加IMU校准
// void QGimbal::calibrate() {
//     if (!enabled) return; // 如果没有使能,则不能校准
//     if (started) return;  // 如果已经启动,则不能校准
//     Gimbal::calibrate();
//     if (calibrated)                                            // 如果基础校准成功
//         freeze_storage_calibration(STORAGE_BASE_CALIBRATE_OK); // 保存基础校准数据
// }

void QGimbal::updateVoltage(const float voltage) {
    if (voltage == 0) return;
    this->voltage = voltage;
}


void QGimbal::Ctrl(const CtrlType ctrl_type, gimbal_pair<float> value) {
    if (ctrl_type == CtrlType::AngleCtrl) {
        value = {
            wrap((value + zero_pos).yaw, 0, 2 * numbers::pi_v<float>),
            wrap((value + zero_pos).pitch, 0, 2 * numbers::pi_v<float>)
        };
    }
    Gimbal::Ctrl(ctrl_type, value);
}

bool QGimbal::setPID(const gimbal_pair<float> pid_speed_kp, const gimbal_pair<float> pid_speed_ki,
                     const gimbal_pair<float> pid_speed_kd, const gimbal_pair<float> pid_angle_kp,
                     const gimbal_pair<float> pid_angle_ki, const gimbal_pair<float> pid_angle_kd) {
    if (!isnan(pid_speed_kp.yaw)) pid_speed.yaw.kp = pid_speed_kp.yaw;
    if (!isnan(pid_speed_ki.yaw)) pid_speed.yaw.ki = pid_speed_ki.yaw;
    if (!isnan(pid_speed_kd.yaw)) pid_speed.yaw.kd = pid_speed_kd.yaw;
    if (!isnan(pid_angle_kp.yaw)) pid_angle.yaw.kp = pid_angle_kp.yaw;
    if (!isnan(pid_angle_ki.yaw)) pid_angle.yaw.ki = pid_angle_ki.yaw;
    if (!isnan(pid_angle_kd.yaw)) pid_angle.yaw.kd = pid_angle_kd.yaw;
    if (!isnan(pid_speed_kp.pitch)) pid_speed.pitch.kp = pid_speed_kp.pitch;
    if (!isnan(pid_speed_ki.pitch)) pid_speed.pitch.ki = pid_speed_ki.pitch;
    if (!isnan(pid_speed_kd.pitch)) pid_speed.pitch.kd = pid_speed_kd.pitch;
    if (!isnan(pid_angle_kp.pitch)) pid_angle.pitch.kp = pid_angle_kp.pitch;
    if (!isnan(pid_angle_ki.pitch)) pid_angle.pitch.ki = pid_angle_ki.pitch;
    if (!isnan(pid_angle_kd.pitch)) pid_angle.pitch.kd = pid_angle_kd.pitch;
    return true;
}

bool QGimbal::setLimit(const gimbal_pair<float> current_limit) {
    if (!isnan(current_limit.yaw)) {
        pid_speed.yaw.output_limit_p = current_limit.yaw;
        pid_speed.yaw.output_limit_n = -current_limit.yaw;
        pid_angle.yaw.output_limit_p = current_limit.yaw;
        pid_angle.yaw.output_limit_n = -current_limit.yaw;
    }
    if (!isnan(current_limit.pitch)) {
        pid_speed.pitch.output_limit_p = current_limit.pitch;
        pid_speed.pitch.output_limit_n = -current_limit.pitch;
        pid_angle.pitch.output_limit_p = current_limit.pitch;
        pid_angle.pitch.output_limit_n = -current_limit.pitch;
    }
    return true;
}

bool QGimbal::setZeroPosition(const gimbal_pair<float> position) {
    zero_pos = {
        wrap((zero_pos + position).yaw, 0, 2 * numbers::pi_v<float>),
        wrap((zero_pos + position).pitch, 0, 2 * numbers::pi_v<float>)
    };
    freeze_storage_calibration(STORAGE_ZERO_POS_OK);
    return true;
}

bool QGimbal::setUartBaudRate(const uint32_t baud_rate) {
    if (baud_rate < 50'000 || baud_rate > 10'000'000) return false; // 波特率必须在50'000-10'000'000之间
    uart_baud_rate = baud_rate;
    // TODO: 重写配置UART
    HAL_UART_DeInit(&huart6);
    huart6.Init.BaudRate = baud_rate;
    if (HAL_UART_Init(&huart6) != HAL_OK) {
        Error_Handler();
    }
    return true;
}

void QGimbal::restore_calibration() {
    setPID(
        {GIMBAL_SPEED_KP_YAW, GIMBAL_SPEED_KP_PITCH},
        {GIMBAL_SPEED_KI_YAW, GIMBAL_SPEED_KI_PITCH},
        {GIMBAL_SPEED_KD_YAW, GIMBAL_SPEED_KI_PITCH},
        {GIMBAL_ANGLE_KP_YAW, GIMBAL_SPEED_KI_PITCH},
        {GIMBAL_ANGLE_KI_YAW, GIMBAL_SPEED_KI_PITCH},
        {GIMBAL_ANGLE_KD_YAW, GIMBAL_SPEED_KI_PITCH}
    );
    setLimit({GIMBAL_MAX_CURRENT, GIMBAL_MAX_CURRENT});
    setUartBaudRate(115200);

    freeze_storage_calibration(
        static_cast<StorageStatus>(STORAGE_PID_PARAMETER_OK | // 储存PID参数
                                   STORAGE_LIMIT_OK |         // 储存限制参数
                                   STORAGE_PLUG_OK)           // 储存ID
    );
}

void QGimbal::load_storage_calibration() {
    uint8_t storage_magic;
    storage.read(0x000, &storage_magic, sizeof(storage_magic));
    if (storage_magic != STORAGE_MAGIC) { return; } // 如果魔术字不对,则说明没有校准数据

    StorageStatus storage_status;
    storage.read(0x010, &storage_status, sizeof(storage_status));
    if ((storage_status & STORAGE_CALIBRATE_OK) == STORAGE_CALIBRATE_OK) {
        // 如果基础校准数据正常,则读取
    }
    if ((storage_status & STORAGE_PID_PARAMETER_OK) == STORAGE_PID_PARAMETER_OK) {
        storage.read(0x200, &pid_speed.yaw.kp, sizeof(pid_speed.yaw.kp));
        storage.read(0x210, &pid_speed.yaw.ki, sizeof(pid_speed.yaw.ki));
        storage.read(0x220, &pid_speed.yaw.kd, sizeof(pid_speed.yaw.kd));
        storage.read(0x230, &pid_angle.yaw.kp, sizeof(pid_angle.yaw.kp));
        storage.read(0x240, &pid_angle.yaw.ki, sizeof(pid_angle.yaw.ki));
        storage.read(0x250, &pid_angle.yaw.kd, sizeof(pid_angle.yaw.kd));
        storage.read(0x260, &pid_speed.yaw.kp, sizeof(pid_speed.yaw.kp));
        storage.read(0x270, &pid_speed.yaw.ki, sizeof(pid_speed.yaw.ki));
        storage.read(0x280, &pid_speed.yaw.kd, sizeof(pid_speed.yaw.kd));
        storage.read(0x290, &pid_angle.yaw.kp, sizeof(pid_angle.yaw.kp));
        storage.read(0x2A0, &pid_angle.yaw.ki, sizeof(pid_angle.yaw.ki));
        storage.read(0x2B0, &pid_angle.yaw.kd, sizeof(pid_angle.yaw.kd));
    }
    if ((storage_status & STORAGE_LIMIT_OK) == STORAGE_LIMIT_OK) {
        storage.read(0x300, &pid_angle.yaw.output_limit_p, sizeof(pid_angle.yaw.output_limit_p));
        pid_angle.yaw.output_limit_n = -pid_angle.yaw.output_limit_p;
        storage.read(0x310, &pid_speed.yaw.output_limit_p, sizeof(pid_speed.yaw.output_limit_p));
        pid_speed.yaw.output_limit_n = -pid_speed.yaw.output_limit_p;
        storage.read(0x320, &pid_angle.pitch.output_limit_p, sizeof(pid_angle.pitch.output_limit_p));
        pid_angle.pitch.output_limit_n = -pid_angle.pitch.output_limit_p;
        storage.read(0x330, &pid_speed.pitch.output_limit_p, sizeof(pid_speed.pitch.output_limit_p));
        pid_speed.pitch.output_limit_n = -pid_speed.pitch.output_limit_p;
    }
    if ((storage_status & STORAGE_PLUG_OK) == STORAGE_PLUG_OK) {
        storage.read(0x400, &uart_baud_rate, sizeof(uart_baud_rate));
        setUartBaudRate(uart_baud_rate); // 配置UART波特率
    }
    if ((storage_status & STORAGE_ZERO_POS_OK) == STORAGE_ZERO_POS_OK) {
        storage.read(0x500, &zero_pos, sizeof(zero_pos));
    }
}

/**
 * @brief 储存校准数据
 * @param storage_type 储存数据类型
 */
void QGimbal::freeze_storage_calibration(const StorageStatus storage_type) {
    uint8_t storage_magic;
    StorageStatus storage_status;
    storage.read(0x000, &storage_magic, sizeof(storage_magic));
    // 如果魔术字不对,则清零所有储存标志
    if (storage_magic != STORAGE_MAGIC) {
        storage_magic = STORAGE_MAGIC;
        storage.write(0x000, &storage_magic, sizeof(storage_magic));
        storage_status = STORAGE_NONE;
        storage.write(0x010, &storage_status, sizeof(storage_status));
    }

    storage.read(0x010, &storage_status, 1);
    if ((storage_type & STORAGE_CALIBRATE_OK) == STORAGE_CALIBRATE_OK) {
        // 储存基础校准数据
    }
    if ((storage_type & STORAGE_PID_PARAMETER_OK) == STORAGE_PID_PARAMETER_OK) {
        // 储存PID参数
        storage.write(0x200, &pid_speed.yaw.kp, sizeof(pid_speed.yaw.kp));
        storage.write(0x210, &pid_speed.yaw.ki, sizeof(pid_speed.yaw.ki));
        storage.write(0x220, &pid_speed.yaw.kd, sizeof(pid_speed.yaw.kd));
        storage.write(0x230, &pid_angle.yaw.kp, sizeof(pid_angle.yaw.kp));
        storage.write(0x240, &pid_angle.yaw.ki, sizeof(pid_angle.yaw.ki));
        storage.write(0x250, &pid_angle.yaw.kd, sizeof(pid_angle.yaw.kd));
        storage.write(0x260, &pid_speed.yaw.kp, sizeof(pid_speed.yaw.kp));
        storage.write(0x270, &pid_speed.yaw.ki, sizeof(pid_speed.yaw.ki));
        storage.write(0x280, &pid_speed.yaw.kd, sizeof(pid_speed.yaw.kd));
        storage.write(0x290, &pid_angle.yaw.kp, sizeof(pid_angle.yaw.kp));
        storage.write(0x2A0, &pid_angle.yaw.ki, sizeof(pid_angle.yaw.ki));
        storage.write(0x2B0, &pid_angle.yaw.kd, sizeof(pid_angle.yaw.kd));
    }
    if ((storage_type & STORAGE_LIMIT_OK) == STORAGE_LIMIT_OK) {
        // 储存限幅参数
        storage.write(0x300, &pid_angle.yaw.output_limit_p, sizeof(pid_angle.yaw.output_limit_p));
        storage.write(0x310, &pid_speed.yaw.output_limit_p, sizeof(pid_speed.yaw.output_limit_p));
        storage.write(0x320, &pid_angle.pitch.output_limit_p, sizeof(pid_angle.pitch.output_limit_p));
        storage.write(0x330, &pid_speed.pitch.output_limit_p, sizeof(pid_speed.pitch.output_limit_p));
    }
    if ((storage_type & STORAGE_PLUG_OK) == STORAGE_PLUG_OK) {
        // 储存波特率
        storage.write(0x400, &uart_baud_rate, sizeof(uart_baud_rate));
    }
    if ((storage_type & STORAGE_ZERO_POS_OK) == STORAGE_ZERO_POS_OK) {
        // 储存位置零点
        storage.write(0x500, &zero_pos, sizeof(zero_pos));
    }

    // 更新储存状态
    storage_status = static_cast<StorageStatus>(storage_status | storage_type);
    storage.write(0x010, &storage_status, sizeof(storage_status));
}
