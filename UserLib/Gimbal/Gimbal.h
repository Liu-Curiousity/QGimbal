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
        SpeedCtrl = 0,
        AngleCtrl = 1,
        StepAngleCtrl = 2,
        LowSpeedCtrl = 3,
    };

    template <typename T>
    struct gimbal_pair {
        T yaw;
        T pitch;
    };

    Gimbal(const gimbal_pair<QD4310&> motor, const gimbal_pair<float> center,
           const gimbal_pair<PID>& pid_imu, const float ctrl_ts) :
        Ts(ctrl_ts), center(center),
        motor(motor), pid_imu(pid_imu) {}

    bool enabled{false};
    bool stability_enabled{false};
    gimbal_pair<float> imu_angle{0, 0}; // 单位:rad

    void enable();
    void disable();
    void enable_stability();
    void disable_stability();

    /**
     * @brief Gimbal控制设置函数
     * @param ctrl_type 控制类型
     * @param speed yaw轴和pitch轴速度,单位:rpm
     */
    void Ctrl(CtrlType ctrl_type, gimbal_pair<float> speed);

    /**
     * @brief Gimbal控制中断服务函数
     * @param imu_angle_ imu测量的偏航和俯仰角度,单位:rad
     */
    void Ctrl_ISR(gimbal_pair<float> imu_angle_);

private:
    CtrlType ctrl_type{CtrlType::SpeedCtrl}; // 当前控制类型

    float Ts;                              // 控制周期,单位:s
    gimbal_pair<float> target_speed{0, 0}; // 单位:rpm
    gimbal_pair<float> center{0, 0};       // 云台中心位置,单位:rad
    gimbal_pair<QD4310&> motor;
    gimbal_pair<PID> pid_imu;

    static constexpr float pitch_max = 0.5f; // pitch轴最大仰角限制,单位:rad

    static float wrap(float value, const float min, const float max) {
        value = std::fmod(value - min, max - min);
        return value < 0 ? value + max : value + min;
    }
};

#endif //QGIMBAL_GIMBAL_H
