/**
 * @file        QD4310.h
 * @brief       基于HAL库的QD4310电机CAN总线控制库
 * @details
 * @author      Liu-Curiousity (2675794963@qq.com)
 * @date        2026-6-15
 * @version     V1.2.0
 * @note
 * @warning
 * @par         历史版本:
 *		        V1.1.0创建于2026-1-25
 *		        V1.1.1创建于2026-4-28, 修复clamp限幅失效的问题
 *		        V1.2.0创建于2026-6-15, 添加零点设置和重启设备
 * @copyright   (c) 2026 QDrive
 */

#ifndef __QD4310_H
#define __QD4310_H

#include <cstdint>
#include "can.h"

class QD4310 {
public:
    explicit QD4310(CAN_HandleTypeDef *hcan, const uint8_t id) :
        id(id), hcan(hcan) {}

    void enable() const { SendCommand(Command::Enable, 0x0000); }
    void disable() const { SendCommand(Command::Disable, 0x0000); }
    void reboot() const { SendCommand(Command::Reboot, 0x0000); }
    void setZeroPos() const { SendCommand(Command::SetZeroPos, 0x0000); }
    void update(const uint8_t feedback[8]);

    void nop() const;
    /**
     * @brief 设置电机角度
     * @param _angle 设置的角度,[0,2pi]
     */
    void setAngle(float _angle) const;
    /**
     * @brief 设置电机角度
     * @param step_angle_ 设置的角度,[-2pi,2pi]
     */
    void setStepAngle(float step_angle_) const;
    /**
     * @brief 设置电机转速
     * @param speed_ 设置的转速,[-1000,1000]
     */
    void setSpeed(float speed_) const;
    /**
     * @brief 设置电机转速
     * @param speed_ 设置的转速,[-1000,1000]
     */
    void setLowSpeed(float speed_) const;
    /**
     * @brief 设置电机电流
     * @param current_ 设置的转速,[-10,10]
     */
    void setCurrent(float current_) const;

    bool enabled{};
    uint8_t id;      // CAN id
    float speed{};   // in rpm
    float angle{};   // in rad
    float current{}; // in A
private:
    enum class Command :uint8_t {
        NOP = 0x00,           // 无操作
        Enable = 0x01,        // 使能
        Disable = 0x02,       // 失能
        CurrentCtrl = 0x03,   // 电流控制
        SpeedCtrl = 0x04,     // 速度控制
        AngleCtrl = 0x05,     // 角度控制
        LowSpeedCtrl = 0x06,  // 低速控制
        StepAngleCtrl = 0x07, // 角度步进控制

        Reboot = 0xFF,     // 重启
        SetZeroPos = 0xFE, // 设置零点
        ClearError = 0xFB, // 清除错误
    };

    CAN_HandleTypeDef *hcan{};

    void SendCommand(Command cmd, int16_t value) const;
};

#endif
