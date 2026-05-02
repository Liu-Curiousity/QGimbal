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
            target_speed = value;
            break;
        case CtrlType::AngleCtrl:
            target_angle = value;
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
    static gimbal_pair<float> previous_angle;

    if (!enabled) return;

    /** 1.更新状态 **/
    angle = {motor.yaw.angle, motor.pitch.angle};
    speed = {motor.yaw.speed, motor.pitch.speed};
    current = {motor.yaw.current, motor.pitch.current};

    imu_angle = {imu_angle_.yaw, imu_angle_.pitch + angle.pitch};
    imu_speed = {};

    auto pitch_clamp = [*this](const float value) {
        if ((value > 0 && wrap((angle - center).pitch) > pitch_max) ||
            (value < 0 && wrap((angle - center).pitch) < -pitch_max))
            return 0.0f;
        return value;
    };

    // 根据稳定模式选择反馈量, 稳定模式下使用IMU角度和速度, 非稳定模式下使用电机角度和速度
    const auto angle_ = stability_enabled ? imu_angle : angle;
    const auto speed_ = stability_enabled ? imu_speed : speed;

    /** 2.速度闭环控制 **/
    switch (ctrl_type) {
        case CtrlType::LowSpeedCtrl:
            target_angle.yaw += target_speed.yaw * Ts * 2 * std::numbers::pi_v<float> / 60;
            target_angle.pitch += pitch_clamp(target_speed.pitch) * Ts * 2 * std::numbers::pi_v<float> / 60;
        case CtrlType::AngleCtrl:
        case CtrlType::StepAngleCtrl:
            if ((previous_angle - angle_).yaw > std::numbers::pi_v<float>)
                target_angle.yaw -= 2 * std::numbers::pi_v<float>;
            else if ((previous_angle - angle_).yaw < -std::numbers::pi_v<float>)
                target_angle.yaw += 2 * std::numbers::pi_v<float>;

            if ((previous_angle - angle_).pitch > std::numbers::pi_v<float>)
                target_angle.pitch -= 2 * std::numbers::pi_v<float>;
            else if ((previous_angle - angle_).pitch < -std::numbers::pi_v<float>)
                target_angle.pitch += 2 * std::numbers::pi_v<float>;

            pid_angle.yaw.target = target_angle.yaw;
            pid_angle.pitch.target = target_angle.pitch;

            target_current.yaw = pid_angle.yaw.calc(angle_.yaw);
            target_current.pitch = pid_angle.pitch.calc(angle_.pitch);

            motor.yaw.setCurrent(target_current.yaw);
            motor.pitch.setCurrent(target_current.pitch);
            break;
        case CtrlType::SpeedCtrl:
            pid_speed.yaw.target = pitch_clamp(target_speed.yaw);
            pid_speed.pitch.target = pitch_clamp(target_speed.pitch);
            target_current = {pid_speed.yaw.calc(speed_.yaw), pid_speed.pitch.calc(speed_.pitch)};
            motor.yaw.setCurrent(target_current.yaw);
            motor.pitch.setCurrent(target_current.pitch);
            break;
        case CtrlType::CurrentCtrl:
            motor.yaw.setCurrent(target_current.yaw);
            motor.pitch.setCurrent(target_current.pitch);
            break;
    }
    previous_angle = angle_;
}
