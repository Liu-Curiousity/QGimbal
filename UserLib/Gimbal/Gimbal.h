/**
 * @brief 		Gimbal.h库文件
 * @detail
 * @author 	    Haoqi Liu
 * @date        2025/10/3
 * @version 	V1.0.0
 * @note
 * @warning
 * @par 		历史版本
                V1.0.0创建于2025/10/3
 * */

#ifndef QGIMBAL_GIMBAL_H
#define QGIMBAL_GIMBAL_H

#include "QD4310.h"
#include "PID.h"

class Gimbal {
public:
    Gimbal(QD4310& yawMotor, QD4310& pitchMotor, const float yaw_center, const float pitch_center,
           const PID& PID_yaw_imu, const PID& PID_pitch_imu, const float ctrl_ts) :
        yawMotor(yawMotor), pitchMotor(pitchMotor), PID_yaw_imu(PID_yaw_imu), PID_pitch_imu(PID_pitch_imu), Ts(ctrl_ts),
        yaw_center(yaw_center), pitch_center(pitch_center) {}

    bool enabled{false};
    bool stability_enabled{false};
    float yaw_imu_angle{0};   // in rad
    float pitch_imu_angle{0}; // in rad

    void enable() {
        if (enabled) return; // 如果已经使能,则不重复使能
        if (!yawMotor.enabled) yawMotor.enable();
        if (!pitchMotor.enabled) pitchMotor.enable();
        if (yawMotor.enabled && pitchMotor.enabled)
            enabled = true;
    }

    void disable() {
        if (!enabled) return; // 如果已经失能,则不重复失能
        yawMotor.setCurrent(0);
        pitchMotor.setCurrent(0);
        if (yawMotor.enabled) yawMotor.disable();
        if (yawMotor.enabled) pitchMotor.disable();
        if (!yawMotor.enabled && !pitchMotor.enabled) {
            disable_stability();
            enabled = false;
        }
    }

    void enable_stability() {
        if (!enabled) return;          // 如果没有使能,则不能开启稳定模式
        if (stability_enabled) return; // 如果已经开启稳定模式,则不重复开启
        PID_yaw_imu.target = yaw_imu_angle;
        PID_pitch_imu.target = pitch_imu_angle + pitchMotor.angle;
        stability_enabled = true;
    }

    void disable_stability() {
        if (!enabled) return;           // 如果没有使能,则不能关闭稳定模式
        if (!stability_enabled) return; // 如果已经关闭稳定模式,则不重复关闭
        stability_enabled = false;
    }

    /**
     * @param yaw_speed yaw轴速度,单位:rpm
     * @param pitch_speed pitch轴速度,单位:rpm
     */
    void Ctrl(const float yaw_speed, const float pitch_speed) {
        target_yaw_speed = yaw_speed;
        target_pitch_speed = pitch_speed;
    }

    /**
     * @param yaw_imu_angle_ imu测量的偏航角度,单位:rad
     * @param pitch_imu_angle_ imu测量的俯仰角度,单位:rad
     */
    void Ctrl_ISR(const float yaw_imu_angle_, const float pitch_imu_angle_) {
        static float speed_to_ctrl = 0;

        if (!enabled) return;

        yaw_imu_angle = yaw_imu_angle_;
        pitch_imu_angle = pitch_imu_angle_ + pitchMotor.angle;
        if (!stability_enabled) {
            yawMotor.setSpeed(target_yaw_speed);
            /*==============pitch轴限位===============*/
            if ((target_pitch_speed > 0 && wrap(pitchMotor.angle - pitch_center, -std::numbers::pi_v<float>,
                                                std::numbers::pi_v<float>) > pitch_max) ||
                (target_pitch_speed < 0 && wrap(pitchMotor.angle - pitch_center, -std::numbers::pi_v<float>,
                                                std::numbers::pi_v<float>) < -pitch_max))
                pitchMotor.setSpeed(0);
            else pitchMotor.setSpeed(target_pitch_speed);
        } else {
            PID_yaw_imu.target += target_yaw_speed * Ts * 2 * std::numbers::pi_v<float> / 60;
            PID_yaw_imu.target = wrap(PID_yaw_imu.target, 0, 2 * std::numbers::pi_v<float>);
            // 过零点处理
            if (PID_yaw_imu.target - yaw_imu_angle > std::numbers::pi_v<float>) {
                speed_to_ctrl = PID_yaw_imu.calc(yaw_imu_angle + 2 * std::numbers::pi_v<float>);
            } else if (PID_yaw_imu.target - yaw_imu_angle < -std::numbers::pi_v<float>) {
                speed_to_ctrl = PID_yaw_imu.calc(yaw_imu_angle - 2 * std::numbers::pi_v<float>);
            } else {
                speed_to_ctrl = PID_yaw_imu.calc(yaw_imu_angle);
            }

            yawMotor.setCurrent(speed_to_ctrl);
            /*==============pitch轴限位===============*/
            if ((target_pitch_speed > 0 && wrap(pitchMotor.angle - pitch_center, -std::numbers::pi_v<float>,
                                                std::numbers::pi_v<float>) > pitch_max) ||
                (target_pitch_speed < 0 && wrap(pitchMotor.angle - pitch_center, -std::numbers::pi_v<float>,
                                                std::numbers::pi_v<float>) < -pitch_max))
                PID_pitch_imu.target += 0;
            else
                PID_pitch_imu.target += target_pitch_speed * Ts * 2 * std::numbers::pi_v<float> / 60;
            PID_pitch_imu.target = wrap(PID_pitch_imu.target, 0, 2 * std::numbers::pi_v<float>);
            // 过零点处理
            if (PID_pitch_imu.target - pitch_imu_angle > std::numbers::pi_v<float>) {
                speed_to_ctrl = PID_pitch_imu.calc(pitch_imu_angle + 2 * std::numbers::pi_v<float>);
            } else if (PID_pitch_imu.target - pitch_imu_angle < -std::numbers::pi_v<float>) {
                speed_to_ctrl = PID_pitch_imu.calc(pitch_imu_angle - 2 * std::numbers::pi_v<float>);
            } else {
                speed_to_ctrl = PID_pitch_imu.calc(pitch_imu_angle);
            }
            pitchMotor.setCurrent(speed_to_ctrl);
        }
    }

private:
    QD4310& yawMotor;
    QD4310& pitchMotor;
    PID PID_yaw_imu;
    PID PID_pitch_imu;

    static constexpr float pitch_max = 0.5f; // pitch轴最大仰角限制,单位:rad

    float Ts;
    float target_yaw_speed{0};   // in rpm
    float target_pitch_speed{0}; // in rpm

    float yaw_center;
    float pitch_center;

    static float wrap(float value, const float min, const float max) {
        value = std::fmod(value - min, max - min);
        return value < 0 ? value + max : value + min;
    }
};

#endif //QGIMBAL_GIMBAL_H
