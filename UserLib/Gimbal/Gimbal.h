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
    enum class CtrlType {
        CurrentCtrl = 0,
        SpeedCtrl = 1,
        AngleCtrl = 2,
        StepAngleCtrl = 3,
        LowSpeedCtrl = 4,
    };

    template <typename T>
    class gimbal_pair {
    public:
        T yaw;
        T pitch;

        gimbal_pair<float> operator-(const gimbal_pair& gimbal_pair) const {
            return {
                yaw - gimbal_pair.yaw,
                pitch - gimbal_pair.pitch
            };
        }
    };

    Gimbal(const gimbal_pair<QD4310&> motor, const gimbal_pair<float> center,
           const gimbal_pair<PID>& pid_speed, const gimbal_pair<PID>& pid_angle,
           const float ctrl_ts) :
        Ts(ctrl_ts), center(center),
        motor(motor), pid_speed(pid_speed), pid_angle(pid_angle) {}

    bool enabled{false};
    bool stability_enabled{false};
    gimbal_pair<float> imu_angle{0, 0}; // 单位:rad
    gimbal_pair<float> imu_speed{0, 0}; // 单位:rad
    gimbal_pair<float> angle{0, 0};     // 单位:rad
    gimbal_pair<float> speed{0, 0};     // 单位:rpm
    gimbal_pair<float> current{0, 0};   // 单位:A

    void enable();
    void disable();
    void enable_stability();
    void disable_stability();

    /**
     * @brief Gimbal控制设置函数
     * @param ctrl_type 控制类型
     * @param value yaw轴和pitch轴控制量
     */
    void Ctrl(CtrlType ctrl_type, gimbal_pair<float> value);

    /**
     * @brief Gimbal控制中断服务函数
     * @param imu_angle_ imu测量的偏航和俯仰角度,单位:rad
     */
    void Ctrl_ISR(gimbal_pair<float> imu_angle_);

private:
    CtrlType ctrl_type{CtrlType::CurrentCtrl}; // 当前控制类型

    float Ts;                                // 控制周期,单位:s
    gimbal_pair<float> target_angle{0, 0};   // 单位:rad
    gimbal_pair<float> target_speed{0, 0};   // 单位:rpm
    gimbal_pair<float> target_current{0, 0}; // 单位:A
    gimbal_pair<float> center{0, 0};         // 云台中心位置,单位:rad
    gimbal_pair<QD4310&> motor;
    gimbal_pair<PID> pid_speed;
    gimbal_pair<PID> pid_angle;

    static constexpr float pitch_max = 0.5f; // pitch轴最大仰角限制,单位:rad

    static float wrap(float value,
                      const float min = -std::numbers::pi_v<float>,
                      const float max = std::numbers::pi_v<float>) {
        value = std::fmod(value - min, max - min);
        return value < 0 ? value + max : value + min;
    }
};

#endif //QGIMBAL_GIMBAL_H
