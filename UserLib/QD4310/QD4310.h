/**
 * @file        QD4310.h
 * @brief       基于HAL库的QD4310电机CAN总线控制库
 * @details
 * @author      Liu-Curiousity (2675794963@qq.com)
 * @date        2026-4-28
 * @version     V1.1.1
 * @note
 * @warning
 * @par         历史版本:
 *		        V1.1.0创建于2026-1-25
 *		        V1.1.1创建于2026-4-28, 修复clamp限幅失效的问题
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

    void enable() const { SendCommand(Command::ENABLE, 0x0000); }
    void disable() const { SendCommand(Command::DISABLE, 0x0000); }
    void update(const uint8_t feedback[8]);

    void nop() const;
    /**
     * @brief 设置电机角度
     * @param _angle 设置的角度,[0,2pi]
     */
    void setAngle(float _angle) const;
    /**
     * @brief 设置电机角度
     * @param _step_angle 设置的角度,[-2pi,2pi]
     */
    void setStepAngle(float _step_angle) const;
    /**
     * @brief 设置电机转速
     * @param _speed 设置的转速,[-1000,1000]
     */
    void setSpeed(float _speed) const;
    /**
     * @brief 设置电机转速
     * @param _speed 设置的转速,[-1000,1000]
     */
    void setLowSpeed(float _speed) const;
    /**
     * @brief 设置电机电流
     * @param _current 设置的转速,[-10,10]
     */
    void setCurrent(float _current) const;

    bool enabled{};
    uint8_t id;      // CAN id
    float speed{};   // in rpm
    float angle{};   // in rad
    float current{}; // in A
private:
    enum class Command :uint8_t {
        NOP = 0x00,
        ENABLE = 0x01,
        DISABLE = 0x02,
        CURRENT = 0x03,
        SPEED = 0x04,
        ANGLE = 0x05,
        LOW_SPEED = 0x06,
        STEP_ANGLE = 0x07
    };

    CAN_HandleTypeDef *hcan{};

    void SendCommand(Command cmd, int16_t value) const;
};

#endif
