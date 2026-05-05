/**
 * @file        QGimbal.cpp
 * @brief       QGimbal电机控制库
 * @details
 * @author      Liu-Curiousity (2675794963@qq.com)
 * @date        2026-5-5
 * @version     V1.0.0
 * @note        此库为中间层库,与硬件完全解耦
 * @warning
 * @par         历史版本:
 *		        V1.0.0创建于2026-5-5, 将Gimbal非核心功能剥离,使用QGimbal类集Gimbal实现解耦
 * @copyright   (c) 2026 QDrive
 */

#ifndef FOC_QGIMBAL_QGIMBAL_H
#define FOC_QGIMBAL_QGIMBAL_H

#include "Gimbal.h"
#include "Storage.h"
#include "main.h"

class QGimbal : public Gimbal {
public:
    /**
     * @brief Gimbal构造函数
     * @param motor yaw轴和pitch轴电机对象引用
     * @param pid_speed yaw轴和pitch轴速度PID
     * @param pid_angle yaw轴和pitch轴角度PID
     * @param ctrl_ts 控制周期,单位:s
     * @param storage 存储器
     */
    QGimbal(const gimbal_pair<QD4310&> motor,
            const gimbal_pair<PID>& pid_speed,
            const gimbal_pair<PID>& pid_angle,
            const float ctrl_ts, Storage& storage) :
        Gimbal(motor, pid_speed, pid_angle, ctrl_ts), storage(storage) {}

    bool laser_enabled{false};
    uint32_t uart_baud_rate{115200}; // UART波特率
    float voltage{0.0f};             // 电压,单位V

    void init();
    void start();
    void stop();
    void enable_laser();
    void disable_laser();
    // void calibrate();
    void updateVoltage(float voltage);

    /**
     * @brief QGimbal控制设置函数
     * @param ctrl_type 控制类型
     * @param value 控制值
     */
    void Ctrl(CtrlType ctrl_type, gimbal_pair<float> value);

    /**
     * @brief 设置PID参数
     * @param pid_speed_kp 速度环比例系数,若为NAN则不更新
     * @param pid_speed_ki 速度环积分系数,若为NAN则不更新
     * @param pid_speed_kd 速度环微分系数,若为NAN则不更新
     * @param pid_angle_kp 角度环比例系数,若为NAN则不更新
     * @param pid_angle_ki 角度环积分系数,若为NAN则不更新
     * @param pid_angle_kd 角度环微分系数,若为NAN则不更新
     * @return 设置成功返回true,失败返回false
     */
    bool setPID(gimbal_pair<float> pid_speed_kp, gimbal_pair<float> pid_speed_ki, gimbal_pair<float> pid_speed_kd,
                gimbal_pair<float> pid_angle_kp, gimbal_pair<float> pid_angle_ki, gimbal_pair<float> pid_angle_kd);

    /**
     * @brief 设置速度和电流限制
     * @param current_limit 电流限制,单位A
     * @return 设置成功返回true,失败返回false
     */
    bool setLimit(gimbal_pair<float> current_limit);

    /**
     * @brief 设置位置零点
     * @param position 位置零点,单位rad
     * @return 设置成功返回true,失败返回false
     */
    bool setZeroPosition(gimbal_pair<float> position);

    /**
     * @brief 设置UART波特率
     * @param baud_rate 波特率,单位bps
     * @return 设置成功返回true,失败返回false
     */
    bool setUartBaudRate(uint32_t baud_rate);

private:
    friend void gimbal_config_list();
    friend void gimbal_store();
    friend void gimbal_restore();

    enum StorageStatus:uint8_t {
        STORAGE_NONE = 0b0000'0000,
        STORAGE_CALIBRATE_OK = 0b0000'0001,
        STORAGE_PID_PARAMETER_OK = 0b0000'0010,
        STORAGE_LIMIT_OK = 0b0000'0100,
        STORAGE_PLUG_OK = 0b0000'1000,
        STORAGE_ZERO_POS_OK = 0b0001'0000,
        STORAGE_ALL_OK = STORAGE_CALIBRATE_OK |
                         STORAGE_PID_PARAMETER_OK |
                         STORAGE_LIMIT_OK |
                         STORAGE_PLUG_OK |
                         STORAGE_ZERO_POS_OK,
    };

    static constexpr uint8_t STORAGE_MAGIC = 0xAA; // 存储器魔术字,储存在0x000

    Storage& storage; //存储器

    void restore_calibration();
    void load_storage_calibration();
    void freeze_storage_calibration(StorageStatus storage_type);
};

#endif //FOC_QGIMBAL_QGIMBAL_H
