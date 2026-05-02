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
    pid_imu.yaw.target = imu_angle.yaw;
    pid_imu.pitch.target = imu_angle.pitch + motor.pitch.angle;
    stability_enabled = true;
}

void Gimbal::disable_stability() {
    if (!enabled) return;           // 如果没有使能,则不能关闭稳定模式
    if (!stability_enabled) return; // 如果已经关闭稳定模式,则不重复关闭
    stability_enabled = false;
}

void Gimbal::Ctrl(const CtrlType ctrl_type, const gimbal_pair<float> speed) {
    this->ctrl_type = ctrl_type;
    target_speed = speed;
}

void Gimbal::Ctrl_ISR(const gimbal_pair<float> imu_angle_) {
    static float current_to_ctrl = 0;

    if (!enabled) return;

    imu_angle.yaw = imu_angle_.yaw;
    imu_angle.pitch = imu_angle_.pitch + motor.pitch.angle;
    if (!stability_enabled) {
        motor.yaw.setSpeed(target_speed.yaw);
        /*==============pitch轴限位===============*/
        if ((target_speed.pitch > 0 && wrap(motor.pitch.angle - center.pitch, -std::numbers::pi_v<float>,
                                            std::numbers::pi_v<float>) > pitch_max) ||
            (target_speed.pitch < 0 && wrap(motor.pitch.angle - center.pitch, -std::numbers::pi_v<float>,
                                            std::numbers::pi_v<float>) < -pitch_max))
            motor.pitch.setSpeed(0);
        else motor.pitch.setSpeed(target_speed.pitch);
    } else {
        pid_imu.yaw.target += target_speed.yaw * Ts * 2 * std::numbers::pi_v<float> / 60;
        pid_imu.yaw.target = wrap(pid_imu.yaw.target, 0, 2 * std::numbers::pi_v<float>);
        // 过零点处理
        if (pid_imu.yaw.target - imu_angle.yaw > std::numbers::pi_v<float>) {
            current_to_ctrl = pid_imu.yaw.calc(imu_angle.yaw + 2 * std::numbers::pi_v<float>);
        } else if (pid_imu.yaw.target - imu_angle.yaw < -std::numbers::pi_v<float>) {
            current_to_ctrl = pid_imu.yaw.calc(imu_angle.yaw - 2 * std::numbers::pi_v<float>);
        } else {
            current_to_ctrl = pid_imu.yaw.calc(imu_angle.yaw);
        }

        motor.yaw.setCurrent(current_to_ctrl);
        /*==============pitch轴限位===============*/
        if ((target_speed.pitch > 0 && wrap(motor.pitch.angle - center.pitch, -std::numbers::pi_v<float>,
                                            std::numbers::pi_v<float>) > pitch_max) ||
            (target_speed.pitch < 0 && wrap(motor.pitch.angle - center.pitch, -std::numbers::pi_v<float>,
                                            std::numbers::pi_v<float>) < -pitch_max))
            pid_imu.pitch.target += 0;
        else
            pid_imu.pitch.target += target_speed.pitch * Ts * 2 * std::numbers::pi_v<float> / 60;
        pid_imu.pitch.target = wrap(pid_imu.pitch.target, 0, 2 * std::numbers::pi_v<float>);
        // 过零点处理
        if (pid_imu.pitch.target - imu_angle.pitch > std::numbers::pi_v<float>) {
            current_to_ctrl = pid_imu.pitch.calc(imu_angle.pitch + 2 * std::numbers::pi_v<float>);
        } else if (pid_imu.pitch.target - imu_angle.pitch < -std::numbers::pi_v<float>) {
            current_to_ctrl = pid_imu.pitch.calc(imu_angle.pitch - 2 * std::numbers::pi_v<float>);
        } else {
            current_to_ctrl = pid_imu.pitch.calc(imu_angle.pitch);
        }
        motor.pitch.setCurrent(current_to_ctrl);
    }
}
