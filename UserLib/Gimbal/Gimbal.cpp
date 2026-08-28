#include "Gimbal.h"
#include "sys_public.h"
#include "MahonyAHRS.h"

extern MahonyAHRS AHRS;

float Gimbal::wrap(float value, const float min, const float max) {
    value = std::fmod(value - min, max - min);
    return value < 0 ? value + max : value + min;
}

void Gimbal::update_attitude(const gimbal_pair<float> imu_angle_raw) {
    static gimbal_pair<float> previous_imu_angle_raw = imu_angle_raw;
    static gimbal_pair<float> previous_imu_angle = imu_angle_raw;
    this->motor_angle = {.yaw = motor.yaw.angle, .pitch = motor.pitch.angle};
    this->motor_speed = {.yaw = motor.yaw.speed, .pitch = motor.pitch.speed};
    this->motor_current = {.yaw = motor.yaw.current, .pitch = motor.pitch.current};
    this->imu_angle = {
        .yaw = wrap(imu_angle_raw.yaw, 0, 2 * std::numbers::pi),
        .pitch = wrap(imu_angle_raw.pitch + motor_angle.pitch - imu_pitch_zero_pos, 0, 2 * std::numbers::pi)
    };
    this->imu_speed = {
        .yaw = wrap((this->imu_angle - previous_imu_angle).yaw) / Ts * 60.0f * std::numbers::inv_pi_v<float> * 0.5f,
        .pitch = wrap((imu_angle_raw - previous_imu_angle_raw).pitch) / Ts * 60.0f * std::numbers::inv_pi_v<float> *
                 0.5f + motor_speed.pitch
    };
    previous_imu_angle_raw = imu_angle_raw;
    previous_imu_angle = this->imu_angle;
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

void Gimbal::reboot() {
    disable();
    motor.yaw.reboot();
    motor.pitch.reboot();
    NVIC_SystemReset();
}

void Gimbal::start() {
    if (enabled) started = true;
}

void Gimbal::stop() {
    pid_speed.yaw.reset();
    pid_speed.pitch.reset();
    pid_angle.yaw.reset();
    pid_angle.pitch.reset();
    started = false;
}

void Gimbal::reset_imu() {
    imu_pitch_zero_pos = motor_angle.pitch;
    AHRS.reset();
}

void Gimbal::enable_stability() {
    if (!enabled) return;          // 如果没有使能,则不能开启稳定模式
    if (stability_enabled) return; // 如果已经开启稳定模式,则不重复开启
    stability_enabled = true;

    const gimbal_pair<float> target_angle_ = {
        .yaw = wrap(target_angle.yaw + imu_angle.yaw - motor_angle.yaw, 0, 2 * std::numbers::pi_v<float>),
        .pitch = wrap(target_angle.pitch + imu_angle.pitch - motor_angle.pitch, 0, 2 * std::numbers::pi_v<float>)
    };
    if (ctrl_type.type == CtrlType::LowSpeedCtrl) {
        Ctrl({.type = CtrlType::AngleCtrl, .value = target_angle_});
        load_ctrl(target_angle_);
        Ctrl({.type = CtrlType::LowSpeedCtrl, .value = target_low_speed});
    } else if (ctrl_type.type == CtrlType::StepAngleCtrl) {
        Ctrl({.type = CtrlType::AngleCtrl, .value = target_angle_});
        load_ctrl(target_angle_);
        Ctrl({.type = CtrlType::StepAngleCtrl, .value = {.yaw = 0, .pitch = 0}});
    } else if (ctrl_type.type == CtrlType::AngleCtrl) {
        Ctrl({.type = CtrlType::AngleCtrl, .value = target_angle_});
    }
}

void Gimbal::disable_stability() {
    if (!enabled) return;           // 如果没有使能,则不能关闭稳定模式
    if (!stability_enabled) return; // 如果已经关闭稳定模式,则不重复关闭
    stability_enabled = false;

    const gimbal_pair<float> target_angle_ = {
        .yaw = wrap(target_angle.yaw - imu_angle.yaw + motor_angle.yaw, 0, 2 * std::numbers::pi_v<float>),
        .pitch = wrap(target_angle.pitch - imu_angle.pitch + motor_angle.pitch, 0, 2 * std::numbers::pi_v<float>)
    };
    if (ctrl_type.type == CtrlType::LowSpeedCtrl) {
        Ctrl({.type = CtrlType::AngleCtrl, .value = target_angle_});
        load_ctrl(target_angle_);
        Ctrl({.type = CtrlType::LowSpeedCtrl, .value = target_low_speed});
    } else if (ctrl_type.type == CtrlType::StepAngleCtrl) {
        Ctrl({.type = CtrlType::AngleCtrl, .value = target_angle_});
        load_ctrl(target_angle_);
        Ctrl({.type = CtrlType::StepAngleCtrl, .value = {.yaw = 0, .pitch = 0}});
    } else if (ctrl_type.type == CtrlType::AngleCtrl) {
        Ctrl({.type = CtrlType::AngleCtrl, .value = target_angle_});
    }
}

void Gimbal::Ctrl(const CtrlType ctrl_type) {
    // 数据同步问题，必须先赋值value，再赋值type，否则被打断后会出问题
    pre_ctrl_type.value = ctrl_type.value;
    pre_ctrl_type.type = ctrl_type.type;
}

void Gimbal::load_ctrl(const gimbal_pair<float> angle) {
    if (pre_ctrl_type.type == CtrlType::NoCtrl) return;
    switch (pre_ctrl_type.type) {
        case CtrlType::LowSpeedCtrl:
            target_low_speed = pre_ctrl_type.value; // 设置低速控制速度
            if (ctrl_type.type != pre_ctrl_type.type) {
                // 当前控制模式不是低速控制时, 才设置角度
                pid_angle.yaw.reset();   // 重置PID角度环
                pid_angle.pitch.reset(); // 重置PID角度环
                target_angle = angle;    // 使用当前角度为低速控制起始角度
            }
            break;
        case CtrlType::StepAngleCtrl:
            if (ctrl_type.type != pre_ctrl_type.type) {
                pid_angle.yaw.reset();   // 重置PID角度环
                pid_angle.pitch.reset(); // 重置PID角度环
                target_angle = angle + pre_ctrl_type.value;
            } else {
                target_angle += pre_ctrl_type.value;
            }
            break;
        case CtrlType::AngleCtrl:
            if (ctrl_type.type != pre_ctrl_type.type) {
                pid_angle.yaw.reset();   // 重置PID角度环
                pid_angle.pitch.reset(); // 重置PID角度环
            }
            // 使云台始终沿差值小于pi的方向转动
            target_angle = {
                .yaw = angle.yaw + wrap((pre_ctrl_type.value - angle).yaw),
                .pitch = angle.pitch + wrap((pre_ctrl_type.value - angle).pitch)
            };
            break;
        case CtrlType::SpeedCtrl:
            if (ctrl_type.type != pre_ctrl_type.type) {
                pid_speed.yaw.reset();   // 重置PID角度环
                pid_speed.pitch.reset(); // 重置PID角度环
            }
            target_speed = pre_ctrl_type.value;
            break;
        case CtrlType::CurrentCtrl:
            target_current = pre_ctrl_type.value;
            break;
        case CtrlType::NoCtrl:
            break;
    }
    ctrl_type = pre_ctrl_type;
    pre_ctrl_type = {.type = CtrlType::NoCtrl, .value = {}};
}

void Gimbal::Ctrl_ISR(const gimbal_pair<float> imu_angle_) {
    static gimbal_pair<float> angle;
    static gimbal_pair<float> previous_angle;
    static gimbal_pair<float> previous_imu_angle;
    if (!enabled) {
        // 实时刷新状态
        motor.yaw.nop();
        motor.pitch.nop();
        return;
    }

    auto pitch_clamp = [*this](const float value) {
        if ((value > 0 && wrap(motor_angle.pitch) > pitch_max) ||
            (value < 0 && wrap(motor_angle.pitch) < -pitch_max))
            return 0.0f;
        return value;
    };

    /** 1.更新状态 **/
    update_attitude(imu_angle_);

    /*应用控制命令*/
    load_ctrl(angle);

    // 根据稳定模式选择反馈量, 稳定模式下使用IMU角度和速度, 非稳定模式下使用电机角度和速度
    angle = stability_enabled ? imu_angle : motor_angle;
    const auto speed = stability_enabled ? imu_speed : motor_speed;

    /** 2.速度闭环控制 **/
    if (started) {
        switch (ctrl_type.type) {
            case CtrlType::LowSpeedCtrl:
                target_angle.yaw += target_low_speed.yaw * Ts * 2 * std::numbers::pi_v<float> / 60;
                target_angle.pitch += pitch_clamp(target_low_speed.pitch) * Ts * 2 * std::numbers::pi_v<float> / 60;
            case CtrlType::AngleCtrl:
            case CtrlType::StepAngleCtrl:
                if ((previous_angle - angle).yaw > std::numbers::pi_v<float>)
                    target_angle.yaw -= 2 * std::numbers::pi_v<float>;
                else if ((previous_angle - angle).yaw < -std::numbers::pi_v<float>)
                    target_angle.yaw += 2 * std::numbers::pi_v<float>;

                if ((previous_angle - angle).pitch > std::numbers::pi_v<float>)
                    target_angle.pitch -= 2 * std::numbers::pi_v<float>;
                else if ((previous_angle - angle).pitch < -std::numbers::pi_v<float>)
                    target_angle.pitch += 2 * std::numbers::pi_v<float>;

                pid_angle.yaw.target = target_angle.yaw;
                pid_angle.pitch.target = target_angle.pitch;

                // 利用C/C++短路机制
                if (!stability_enabled) {
                    if ((ctrl_type.type != CtrlType::LowSpeedCtrl || target_low_speed.yaw == 0) &&
                        fabsf(target_angle.yaw - angle.yaw) < std::numbers::pi_v<float> / 32768.0f)
                        target_angle.yaw = angle.yaw;
                    if ((ctrl_type.type != CtrlType::LowSpeedCtrl || target_low_speed.pitch == 0) &&
                        fabsf(target_angle.pitch - angle.pitch) < std::numbers::pi_v<float> / 32768.0f)
                        target_angle.pitch = angle.pitch;
                }

                target_current.yaw = pid_angle.yaw.calc(angle.yaw);
                target_current.pitch = pid_angle.pitch.calc(angle.pitch);

                motor.yaw.setCurrent(target_current.yaw);
                motor.pitch.setCurrent(target_current.pitch);
                break;
            case CtrlType::SpeedCtrl:
                pid_speed.yaw.target = target_speed.yaw;
                pid_speed.pitch.target = pitch_clamp(target_speed.pitch);
                target_current = {pid_speed.yaw.calc(speed.yaw), pid_speed.pitch.calc(speed.pitch)};
                motor.yaw.setCurrent(target_current.yaw);
                motor.pitch.setCurrent(target_current.pitch);
                break;
            case CtrlType::CurrentCtrl:
                motor.yaw.setCurrent(target_current.yaw);
                motor.pitch.setCurrent(target_current.pitch);
                break;
            case CtrlType::NoCtrl:
                break;
        }
    } else {
        motor.yaw.setCurrent(0);
        motor.pitch.setCurrent(0);
    }
    previous_angle = angle;
}
