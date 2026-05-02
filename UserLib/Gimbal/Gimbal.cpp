//
// Created by 26757 on 2025/10/3.
//

#include "Gimbal.h"

void Gimbal::enable() {
    if (enabled) return; // 如果已经使能,则不重复使能
    if (!motor.yaw.enabled) motor.yaw.enable();
    if (!motor.pitch.enabled) motor.pitch.enable();
    if (motor.yaw.enabled && motor.pitch.enabled)
        enabled = true;
}

void Gimbal::disable() {
    if (!enabled) return; // 如果已经失能,则不重复失能
    motor.yaw.setCurrent(0);
    motor.pitch.setCurrent(0);
    if (motor.yaw.enabled) motor.yaw.disable();
    if (motor.yaw.enabled) motor.pitch.disable();
    if (!motor.yaw.enabled && !motor.pitch.enabled) {
        disable_stability();
        enabled = false;
    }
}

void Gimbal::enable_stability() {
    if (!enabled) return;          // 如果没有使能,则不能开启稳定模式
    if (stability_enabled) return; // 如果已经开启稳定模式,则不重复开启
    pid_angle.yaw.target = imu_angle.yaw;
    pid_angle.pitch.target = imu_angle.pitch + angle.pitch;
    stability_enabled = true;
}

void Gimbal::disable_stability() {
    if (!enabled) return;           // 如果没有使能,则不能关闭稳定模式
    if (!stability_enabled) return; // 如果已经关闭稳定模式,则不重复关闭
    stability_enabled = false;
}

void Gimbal::Ctrl(const CtrlType ctrl_type, const gimbal_pair<float> value) {
    this->ctrl_type = ctrl_type;
    switch (ctrl_type) {
        case CtrlType::LowSpeedCtrl:
            break;
        case CtrlType::AngleCtrl:
            break;
        case CtrlType::StepAngleCtrl:
            break;
        case CtrlType::SpeedCtrl:
            target_speed = value;
            break;
        case CtrlType::CurrentCtrl:
            target_current = value;
            break;
    }
}

void Gimbal::Ctrl_ISR(const gimbal_pair<float> imu_angle_) {
    static gimbal_pair<float> previous_imu_angle;

    if (!enabled) return;

    /** 1.更新状态 **/
    angle = {motor.yaw.angle, motor.pitch.angle};
    speed = {motor.yaw.speed, motor.pitch.speed};
    current = {motor.yaw.current, motor.pitch.current};

    imu_angle = {imu_angle_.yaw, imu_angle_.pitch + angle.pitch};

    auto pitch_clamp = [*this](const float value) {
        if ((value > 0 && wrap((angle - center).pitch) > pitch_max) ||
            (value < 0 && wrap((angle - center).pitch) < -pitch_max))
            return 0.0f;
        return value;
    };

    /**1.速度闭环控制**/
    switch (ctrl_type) {
        case CtrlType::LowSpeedCtrl:
            if (!stability_enabled) {
                motor.yaw.setSpeed(target_speed.yaw);
                motor.pitch.setSpeed(pitch_clamp(target_speed.pitch));
            } else {
                pid_angle.yaw.target += target_speed.yaw * Ts * 2 * std::numbers::pi_v<float> / 60;
                if ((previous_imu_angle - imu_angle).yaw > std::numbers::pi_v<float>)
                    pid_angle.yaw.target -= 2 * std::numbers::pi_v<float>;
                else if ((previous_imu_angle - imu_angle).yaw < -std::numbers::pi_v<float>)
                    pid_angle.yaw.target += 2 * std::numbers::pi_v<float>;
                target_current.yaw = pid_angle.yaw.calc(imu_angle.yaw);

                pid_angle.pitch.target += pitch_clamp(target_speed.pitch) * Ts * 2 * std::numbers::pi_v<float> / 60;
                if ((previous_imu_angle - imu_angle).pitch > std::numbers::pi_v<float>)
                    pid_angle.pitch.target -= 2 * std::numbers::pi_v<float>;
                else if ((previous_imu_angle - imu_angle).pitch < -std::numbers::pi_v<float>)
                    pid_angle.pitch.target += 2 * std::numbers::pi_v<float>;
                target_current.pitch = pid_angle.pitch.calc(imu_angle.pitch);

                motor.yaw.setCurrent(target_current.yaw);
                motor.pitch.setCurrent(target_current.pitch);
            }
            break;
        case CtrlType::AngleCtrl:
            break;
        case CtrlType::StepAngleCtrl:
            break;
        case CtrlType::SpeedCtrl:
            if (stability_enabled) {
                target_current = {};
            } else {
                target_current = {pid_speed.yaw.calc(speed.yaw), pid_speed.pitch.calc(speed.pitch)};
            }
            break;
        case CtrlType::CurrentCtrl:
            motor.yaw.setCurrent(target_current.yaw);
            motor.pitch.setCurrent(target_current.pitch);
            break;
    }
    previous_imu_angle = imu_angle;
}
