/**
 * @file        QD4310.cpp
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

#include <algorithm>
#include "QD4310.h"

void QD4310::SendCommand(const Command cmd, const int16_t value) const {
    static uint8_t TxBuffer[3];
    TxBuffer[0] = static_cast<uint8_t>(cmd);
    *reinterpret_cast<int16_t *>(TxBuffer + 1) = value;

    uint32_t txMailbox = CAN_TX_MAILBOX0;
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = 0x400 + id;
    TxHeader.ExtId = 0x400 + id;
    TxHeader.TransmitGlobalTime = DISABLE;
    TxHeader.DLC = 3;
    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxBuffer, &txMailbox);
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);
}

void QD4310::update(const uint8_t feedback[8]) {
    enabled = feedback[0] & 0x01;
    current = *(int16_t *)(feedback + 2) * 10.0f / INT16_MAX;
    speed = *(int16_t *)(feedback + 4) * 1000.0f / INT16_MAX;
    angle = *(uint16_t *)(feedback + 6) * 2 * std::numbers::pi_v<float> / UINT16_MAX;
}

void QD4310::nop() const {
    SendCommand(Command::NOP, 0x0000);
}

void QD4310::setAngle(float _angle) const {
    _angle = std::clamp(_angle, 0.0f, 2 * std::numbers::pi_v<float>); // 限制角度在[0, 2pi]范围内
    SendCommand(Command::ANGLE, _angle / 2 / std::numbers::pi_v<float> * UINT16_MAX);
}

void QD4310::setStepAngle(float _step_angle) const {
    // 限制角度在[-2pi, 2pi]范围内
    _step_angle = std::clamp(_step_angle, -2 * std::numbers::pi_v<float>, 2 * std::numbers::pi_v<float>);
    SendCommand(Command::STEP_ANGLE, _step_angle / 2 / std::numbers::pi_v<float> * INT16_MAX);
}

void QD4310::setSpeed(float _speed) const {
    _speed = std::clamp(_speed, -1000.0f, 1000.0f); // 限制速度在[-1000, 1000]范围内
    SendCommand(Command::SPEED, _speed / 1000 * INT16_MAX);
}

void QD4310::setLowSpeed(float _speed) const {
    _speed = std::clamp(_speed, -1000.0f, 1000.0f); // 限制速度在[-1000, 1000]范围内
    SendCommand(Command::LOW_SPEED, _speed / 1000 * INT16_MAX);
}

void QD4310::setCurrent(float _current) const {
    _current = std::clamp(_current, -10.0f, 10.0f);
    SendCommand(Command::CURRENT, _current / 10 * INT16_MAX);
}
