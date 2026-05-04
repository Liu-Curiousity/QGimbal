#include "Gimbal.h"
#include "sys_public.h"
#include "MahonyAHRS.h"

extern MahonyAHRS AHRS;

float Gimbal::wrap(float value, const float min, const float max) {
    value = std::fmod(value - min, max - min);
    return value < 0 ? value + max : value + min;
}

void Gimbal::update_attitude(gimbal_pair<float> imu_angle) {
    static gimbal_pair<float> previous_imu_angle = imu_angle;
    this->motor_angle = {motor.yaw.angle, motor.pitch.angle};
    this->motor_speed = {motor.yaw.speed, motor.pitch.speed};
    this->motor_current = {motor.yaw.current, motor.pitch.current};
    this->imu_angle = {imu_angle.yaw, imu_angle.pitch + motor_angle.pitch};
    this->imu_speed = {
        wrap((imu_angle - previous_imu_angle).yaw) / Ts * 60.0f * std::numbers::inv_pi_v<float> * 0.5f,
        wrap((imu_angle - previous_imu_angle).pitch) / Ts * 60.0f * std::numbers::inv_pi_v<float> * 0.5f
    };
    previous_imu_angle = imu_angle;
}

void Gimbal::init() {
    initialized = true;
}

void Gimbal::enable() {
    if (!initialized) return; // 如果没有初始化,则不能使能
    if (enabled) return;      // 如果已经使能,则不重复使能
    if (!motor.yaw.enabled) motor.yaw.enable();
    if (!motor.pitch.enabled) motor.pitch.enable();
    delay_ms(1); // 等待电机响应
    if (motor.yaw.enabled && motor.pitch.enabled)
        enabled = true;
}

void Gimbal::disable() {
    if (!enabled) return; // 如果已经失能,则不重复失能
    motor.yaw.setCurrent(0);
    motor.pitch.setCurrent(0);
    if (motor.yaw.enabled) motor.yaw.disable();
    if (motor.yaw.enabled) motor.pitch.disable();
    delay_ms(1); // 等待电机响应
    if (!motor.yaw.enabled && !motor.pitch.enabled) {
        disable_stability();
        enabled = false;
    }
}

void Gimbal::start() {
    if (enabled) started = true;
}

void Gimbal::stop() {
    started = false;
}

void Gimbal::reset_imu() {
    AHRS.reset();
}

void Gimbal::enable_stability() {
    if (!enabled) return;          // 如果没有使能,则不能开启稳定模式
    if (stability_enabled) return; // 如果已经开启稳定模式,则不重复开启
    target_angle = imu_angle;
    target_speed = imu_speed;
    stability_enabled = true;
}

void Gimbal::disable_stability() {
    if (!enabled) return;           // 如果没有使能,则不能关闭稳定模式
    if (!stability_enabled) return; // 如果已经关闭稳定模式,则不重复关闭
    stability_enabled = false;
}

void Gimbal::Ctrl(const CtrlType ctrl_type, const gimbal_pair<float> value) {
    const auto angle_ = stability_enabled ? imu_angle : motor_angle;
    switch (ctrl_type) {
        case CtrlType::LowSpeedCtrl:
            target_low_speed = value;         // 设置低速控制速度
            if (this->ctrl_type != ctrl_type) // 当前控制模式不是低速控制时, 才设置角度
                target_angle = angle_;        // 使用当前角度为低速控制起始角度
            break;
        case CtrlType::StepAngleCtrl:
            if (this->ctrl_type == ctrl_type)
                target_angle += value;
            else
                target_angle = angle_ + value;
            break;
        case CtrlType::AngleCtrl:
            // 使云台始终沿差值小于pi的方向转动
            target_angle = {
                angle_.yaw + wrap((value - angle_).yaw),
                angle_.pitch + wrap((value - angle_).pitch)
            };
            break;
        case CtrlType::SpeedCtrl:
            target_speed = value;
            break;
        case CtrlType::CurrentCtrl:
            target_current = value;
            break;
    }
    this->ctrl_type = ctrl_type;
}

void Gimbal::Ctrl_ISR(const gimbal_pair<float> imu_angle_) {
    static gimbal_pair<float> previous_angle;
    static gimbal_pair<float> previous_imu_angle;
    if (!enabled) {
        // 实时刷新状态
        motor.yaw.nop();
        motor.pitch.nop();
        return;
    }

    auto pitch_clamp = [*this](const float value) {
        if ((value > 0 && wrap((motor_angle - center).pitch) > pitch_max) ||
            (value < 0 && wrap((motor_angle - center).pitch) < -pitch_max))
            return 0.0f;
        return value;
    };

    /** 1.更新状态 **/
    update_attitude(imu_angle_);
    // 根据稳定模式选择反馈量, 稳定模式下使用IMU角度和速度, 非稳定模式下使用电机角度和速度
    const auto angle_ = stability_enabled ? imu_angle : motor_angle;
    const auto previous_angle_ = stability_enabled ? previous_imu_angle : previous_angle;
    const auto speed_ = stability_enabled ? imu_speed : motor_speed;

    /** 2.速度闭环控制 **/
    if (started) {
        switch (ctrl_type) {
            case CtrlType::LowSpeedCtrl:
                target_angle.yaw += target_low_speed.yaw * Ts * 2 * std::numbers::pi_v<float> / 60;
                target_angle.pitch += pitch_clamp(target_low_speed.pitch) * Ts * 2 * std::numbers::pi_v<float> / 60;
            case CtrlType::AngleCtrl:
            case CtrlType::StepAngleCtrl:
                if ((previous_angle_ - angle_).yaw > std::numbers::pi_v<float>)
                    target_angle.yaw -= 2 * std::numbers::pi_v<float>;
                else if ((previous_angle_ - angle_).yaw < -std::numbers::pi_v<float>)
                    target_angle.yaw += 2 * std::numbers::pi_v<float>;

                if ((previous_angle_ - angle_).pitch > std::numbers::pi_v<float>)
                    target_angle.pitch -= 2 * std::numbers::pi_v<float>;
                else if ((previous_angle_ - angle_).pitch < -std::numbers::pi_v<float>)
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
    } else {
        motor.yaw.setCurrent(0);
        motor.pitch.setCurrent(0);
    }
    previous_angle = motor_angle;
    previous_imu_angle = imu_angle;
}
