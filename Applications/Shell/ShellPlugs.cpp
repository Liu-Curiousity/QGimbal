/**
 * @file        ShellPlugs.cpp
 * @brief       shell 接口函数
 * @details
 * @author      Liu-Curiousity (2675794963@qq.com)
 * @date        2026-4-28
 * @version     V1.0.0
 * @note
 * @warning
 * @par         历史版本:
 *		        V1.0.0创建于2026-4-28
 * @copyright   (c) 2026 QDrive
 */

#include <algorithm>

#include "QGimbal.h"
#include "Gimbal_config.h"
#include "shell_cpp.h"
#include "usbd_cdc_if.h"
#include "retarget.h"
#include "sys_public.h"

extern QGimbal qgimbal;
extern Shell shell;

static float atof_lite(const char *s) {
    if (!s) return 0.0f;

    // 可选符号
    int sign = 1;
    if (*s == '+') {
        ++s;
    } else if (*s == '-') {
        sign = -1;
        ++s;
    }

    // 解析整数部分
    float int_part = 0.0f;
    bool has_digit = false;
    while (*s >= '0' && *s <= '9') {
        has_digit = true;
        int_part = int_part * 10.0f + static_cast<float>(*s - '0');
        ++s;
    }

    // 解析小数部分
    float frac_part = 0.0f;
    float scale = 1.0f;
    if (*s == '.') {
        ++s;
        while (*s >= '0' && *s <= '9') {
            has_digit = true;
            frac_part = frac_part * 10.0f + static_cast<float>(*s - '0');
            scale *= 10.0f;
            ++s;
        }
    }

    if (!has_digit) return 0.0f;

    const float result = int_part + (frac_part / scale);
    return (sign < 0) ? -result : result;
}

// 打印单行
#define PRINT(...)                          \
    do {                                    \
        printf(__VA_ARGS__);                \
        printf("\r\n");                     \
    } while (0)

void print_version() {
    PRINT("Software version %s", GIMBAL_SOFTWARE_VERSION);
}

void gimbal_status() {
    PRINT("Gimbal Status:");
    PRINT("  Enabled            : %s", qgimbal.started ? "Yes" : "No");
    PRINT("  Stability Enabled  : %s", qgimbal.stability_enabled ? "Yes" : "No");
    PRINT("  Laser Enabled      : %s", qgimbal.laser_enabled ? "Yes" : "No");
    PRINT("  CtrlMode           : %s",
          qgimbal.getCtrlType() == Gimbal::CtrlType::CurrentCtrl ? "CurrentCtrl" :
          qgimbal.getCtrlType() == Gimbal::CtrlType::SpeedCtrl ? "SpeedCtrl" :
          qgimbal.getCtrlType() == Gimbal::CtrlType::AngleCtrl ? "AngleCtrl" :
          qgimbal.getCtrlType() == Gimbal::CtrlType::StepAngleCtrl ? "StepAngleCtrl" :
          qgimbal.getCtrlType() == Gimbal::CtrlType::LowSpeedCtrl ? "LowSpeedCtrl" : "Unknown");
    PRINT("  IMU Angle          : yaw:%.2f rad, pitch:%.2f rad", qgimbal.imu_angle.yaw, qgimbal.imu_angle.pitch);
    PRINT("  IMU Speed          : yaw:%.2f rpm, pitch:%.2f rpm", qgimbal.imu_speed.yaw, qgimbal.imu_speed.pitch);
    PRINT("  Angle              : yaw:%.2f rad, pitch:%.2f rad", qgimbal.motor_angle.yaw, qgimbal.motor_angle.pitch);
    PRINT("  Speed              : yaw:%.2f rpm, pitch:%.2f rpm", qgimbal.motor_speed.yaw, qgimbal.motor_speed.pitch);
    PRINT("  Current            : yaw:%.2f A  , pitch:%.2f A  ", qgimbal.motor_current.yaw,
          qgimbal.motor_current.pitch);
    PRINT("  Voltage            : %.2f V", qgimbal.voltage);
}

void gimbal_config_help() {
    PRINT("Usage: config [--list | PARAM_PATH VALUE | key=value]");
    PRINT("");
    PRINT("Examples:");
    PRINT("  config pid.speed.kp.yaw 0.1");
    PRINT("  config pid.speed.ki.pitch=0.1");
    PRINT("  config --help");
    PRINT("  config --list");
    PRINT("");
    PRINT("Configuration Parameters:");
    PRINT("  pid.speed.kp.[yaw|pitch]   : Speed PID proportional gain");
    PRINT("  pid.speed.ki.[yaw|pitch]   : Speed PID integral gain");
    PRINT("  pid.speed.kd.[yaw|pitch]   : Speed PID derivative gain");
    PRINT("  pid.angle.kp.[yaw|pitch]   : Angle PID proportional gain");
    PRINT("  pid.angle.ki.[yaw|pitch]   : Angle PID integral gain");
    PRINT("  pid.angle.kd.[yaw|pitch]   : Angle PID derivative gain");
    PRINT("  limit.speed.[yaw|pitch]    : Speed limit in rpm");
    PRINT("  limit.current.[yaw|pitch]  : Current limit in A");
    PRINT("  center.[yaw|pitch]         : Center position offset in rad");
}

void gimbal_config_list() {
    PRINT("Current Configuration:");
    PRINT("pid.speed.kp.yaw = %.3g", qgimbal.pid_speed.yaw.kp);
    PRINT("pid.speed.ki.yaw = %.3g", qgimbal.pid_speed.yaw.ki);
    PRINT("pid.speed.kd.yaw = %.3g", qgimbal.pid_speed.yaw.kd);
    PRINT("pid.angle.kp.yaw = %.3g", qgimbal.pid_angle.yaw.kp);
    PRINT("pid.angle.ki.yaw = %.3g", qgimbal.pid_angle.yaw.ki);
    PRINT("pid.angle.kd.yaw = %.3g", qgimbal.pid_angle.yaw.kd);

    PRINT("pid.speed.kp.pitch = %.3g", qgimbal.pid_speed.pitch.kp);
    PRINT("pid.speed.ki.pitch = %.3g", qgimbal.pid_speed.pitch.ki);
    PRINT("pid.speed.kd.pitch = %.3g", qgimbal.pid_speed.pitch.kd);
    PRINT("pid.angle.kp.pitch = %.3g", qgimbal.pid_angle.pitch.kp);
    PRINT("pid.angle.ki.pitch = %.3g", qgimbal.pid_angle.pitch.ki);
    PRINT("pid.angle.kd.pitch = %.3g", qgimbal.pid_angle.pitch.kd);

    PRINT("limit.current.yaw = %.3g A", qgimbal.pid_speed.yaw.output_limit_p);
    PRINT("limit.current.pitch = %.3g A", qgimbal.pid_speed.pitch.output_limit_p);

    PRINT("uart.baud_rate = %u", qgimbal.uart_baud_rate);
}

void gimbal_config(int argc, char *argv[]) {
    if (argc < 2 || strcmp(argv[1], "--help") == 0) {
        gimbal_config_help();
        return;
    }

    if (strcmp(argv[1], "--list") == 0) {
        gimbal_config_list();
        return;
    }

    const char *key = argv[1];
    const char *value = nullptr;

    if (strchr(key, '=') != nullptr) {
        // 解析 key=value 格式
        static char keybuf[128];
        strncpy(keybuf, key, sizeof(keybuf) - 1);
        keybuf[sizeof(keybuf) - 1] = '\0';

        char *eq = strchr(keybuf, '=');
        *eq = '\0';
        key = keybuf;
        value = eq + 1;
    } else if (argc >= 3) {
        value = argv[2];
    }

    if (strcmp(key, "zero_pos") == 0) {
        if (value && strcmp(value, "--imu") == 0) {
            if (qgimbal.stability_enabled) {
                PRINT("Cannot reset IMU zero position while stability is enabled. Please disable stability first.");
                return;
            }
            qgimbal.reset_imu();
            PRINT("Setting config [zero_pos] for IMU");
        } else {
            qgimbal.setZeroPosition(qgimbal.motor_angle);
            PRINT("Setting config [zero_pos]");
        }
    } else if (value) {
        const float valf = atof_lite(value);
        if (strcmp(key, "pid.speed.kp.yaw") == 0)
            qgimbal.setPID({valf, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN});
        else if (strcmp(key, "pid.speed.ki.yaw") == 0)
            qgimbal.setPID({NAN, NAN}, {valf, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN});
        else if (strcmp(key, "pid.speed.kd.yaw") == 0)
            qgimbal.setPID({NAN, NAN}, {NAN, NAN}, {valf, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN});
        else if (strcmp(key, "pid.angle.kp.yaw") == 0)
            qgimbal.setPID({NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {valf, NAN}, {NAN, NAN}, {NAN, NAN});
        else if (strcmp(key, "pid.angle.ki.yaw") == 0)
            qgimbal.setPID({NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {valf, NAN}, {NAN, NAN});
        else if (strcmp(key, "pid.angle.kd.yaw") == 0)
            qgimbal.setPID({NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {valf, NAN});
        else if (strcmp(key, "pid.speed.kp.pitch") == 0)
            qgimbal.setPID({NAN, valf}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN});
        else if (strcmp(key, "pid.speed.ki.pitch") == 0)
            qgimbal.setPID({NAN, NAN}, {NAN, valf}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN});
        else if (strcmp(key, "pid.speed.kd.pitch") == 0)
            qgimbal.setPID({NAN, NAN}, {NAN, NAN}, {NAN, valf}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN});
        else if (strcmp(key, "pid.angle.kp.pitch") == 0)
            qgimbal.setPID({NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, valf}, {NAN, NAN}, {NAN, NAN});
        else if (strcmp(key, "pid.angle.ki.pitch") == 0)
            qgimbal.setPID({NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, valf}, {NAN, NAN});
        else if (strcmp(key, "pid.angle.kd.pitch") == 0)
            qgimbal.setPID({NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, NAN}, {NAN, valf});
        else if (strcmp(key, "limit.current.yaw") == 0)
            qgimbal.setLimit({valf,NAN});
        else if (strcmp(key, "limit.current.pitch") == 0)
            qgimbal.setLimit({NAN, valf});
        else if (strcmp(key, "uart.baud_rate") == 0) {
            if (!qgimbal.setUartBaudRate(valf)) {
                PRINT("Invalid UART baud rate: %d, must be between 10'000 and 5'000'000", static_cast<int>(valf));
                return;
            }
            PRINT("UART baud rate will be set after storing and rebooting");
        } else {
            PRINT("Unknown config target: %s", key);
            return;
        }
        if (valf == 0) {
            PRINT("Setting config [%s] = 0.000", key);
        } else {
            PRINT("Setting config [%s] = %.3g", key, valf);
        }
    } else {
        PRINT("Missing value for config [%s]", key);
    }
}

void gimbal_ctrl_help() {
    PRINT("Usage: ctrl [current Y P | low_speed Y P | speed Y P | step_angle Y P | angle Y P | key=y,p]");
    PRINT("");
    PRINT("Examples:");
    PRINT("  ctrl speed 100 0");
    PRINT("  ctrl speed=100,0");
    PRINT("  ctrl --help");
    PRINT("");
    PRINT("Control Parameters:");
    PRINT("  current           : Set current (A)");
    PRINT("  low_speed         : Set speed by increasing angle (rpm)");
    PRINT("  speed             : Set speed (rpm)");
    PRINT("  angle             : Set angle (rad)");
    PRINT("  step_angle        : Step an specific angle (rad)");
}

void gimbal_ctrl(int argc, char *argv[]) {
    if (argc < 2 || strcmp(argv[1], "--help") == 0) {
        gimbal_ctrl_help();
        return;
    }

    const char *key = argv[1];
    float y_val = 0;
    float p_val = 0;
    bool has_val = false;

    if (strchr(key, '=') != nullptr) {
        static char keybuf[128];
        strncpy(keybuf, key, sizeof(keybuf) - 1);
        keybuf[sizeof(keybuf) - 1] = '\0';
        char *eq = strchr(keybuf, '=');
        *eq = '\0';
        key = keybuf;
        char *comma = strchr(eq + 1, ',');
        if (comma) {
            *comma = '\0';
            y_val = atof_lite(eq + 1);
            p_val = atof_lite(comma + 1);
            has_val = true;
        }
    } else if (argc >= 4) {
        y_val = atof_lite(argv[2]);
        p_val = atof_lite(argv[3]);
        has_val = true;
    }

    if (has_val) {
        const Gimbal::gimbal_pair vals = {y_val, p_val};
        if (strcmp(key, "current") == 0) {
            PRINT("Setting current Y:%.2f P:%.2f A", y_val, p_val);
            qgimbal.Ctrl(Gimbal::CtrlType::CurrentCtrl, vals);
        } else if (strcmp(key, "speed") == 0) {
            PRINT("Setting speed Y:%.2f P:%.2f rpm", y_val, p_val);
            qgimbal.Ctrl(Gimbal::CtrlType::SpeedCtrl, vals);
        } else if (strcmp(key, "angle") == 0) {
            PRINT("Setting angle Y:%.2f P:%.2f rad", y_val, p_val);
            qgimbal.Ctrl(Gimbal::CtrlType::AngleCtrl, vals);
        } else if (strcmp(key, "step_angle") == 0) {
            PRINT("Stepping angle Y:%.2f P:%.2f rad", y_val, p_val);
            qgimbal.Ctrl(Gimbal::CtrlType::StepAngleCtrl, vals);
        } else if (strcmp(key, "low_speed") == 0) {
            PRINT("Setting low_speed Y:%.2f P:%.2f rpm", y_val, p_val);
            qgimbal.Ctrl(Gimbal::CtrlType::LowSpeedCtrl, vals);
        } else {
            PRINT("Unknown ctrl target: %s", key);
            gimbal_ctrl_help();
        }
    } else {
        PRINT("Missing value for ctrl [%s], format: val_yaw val_pitch", key);
    }
}

void gimbal_enable() {
    qgimbal.start();
    if (qgimbal.started) {
        PRINT("QGimbal enabled");
    } else
        PRINT("enable failed, please calibrate first");
}

void gimbal_disable() {
    qgimbal.stop();
    PRINT("QGimbal disabled");
}

void gimbal_enable_stability() {
    qgimbal.enable_stability();
    if (qgimbal.stability_enabled) {
        PRINT("QGimbal stability enabled");
    } else {
        PRINT("enable failed");
    }
}

void gimbal_disable_stability() {
    qgimbal.disable_stability();
    PRINT("QGimbal stability control disabled");
}

void gimbal_enable_laser() {
    qgimbal.enable_laser();
    if (qgimbal.laser_enabled) {
        PRINT("Laser enabled");
    } else {
        PRINT("enable failed");
    }
}

void gimbal_disable_laser() {
    qgimbal.disable_laser();
    PRINT("Laser disabled");
}

void gimbal_restore() {
    PRINT("Are you sure you want to restore factory settings? (y/n)");
    char response;
    while (!shellRead(&response, 1)) {
        delay_ms(1); // Wait for input
    }
    if (response != 'y' && response != 'Y') {
        PRINT("Factory restore cancelled");
        return;
    }
    qgimbal.restore_calibration(); // 恢复出厂设置
    PRINT("QGimbal factory restore completed");
    gimbal_config_list();
}

void gimbal_store() {
    if (qgimbal.started) {
        PRINT("QGimbal is running, please disable it first");
        return;
    }
    gimbal_config_list();
    PRINT("Are you sure you want to store configurations? (y/n)");
    char response;
    while (!shellRead(&response, 1)) {
        delay_ms(1); // Wait for input
    }
    if (response != 'y' && response != 'Y') {
        PRINT("Store operation cancelled");
        return;
    }
    qgimbal.freeze_storage_calibration(
        static_cast<QGimbal::StorageStatus>(QGimbal::STORAGE_PID_PARAMETER_OK | // 储存PID参数
                                            QGimbal::STORAGE_LIMIT_OK |         // 储存限制参数
                                            QGimbal::STORAGE_PLUG_OK)           // 储存ID
    );
    PRINT("Store configuration completed");
}

void shell_reboot() {
    NVIC_SystemReset();
}

SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    version, print_version, Show version info
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    reboot, shell_reboot, reboot system
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    store, gimbal_store, Store configurations
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    restore, gimbal_restore, Factory restore
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    ctrl, gimbal_ctrl, Set control targets
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    config, gimbal_config, Configure system parameters
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    disable, gimbal_disable, Disable FOC control
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    enable, gimbal_enable, Enable FOC control
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    enable_stability, gimbal_enable_stability, Enable gimbal stability control
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    disable_stability, gimbal_disable_stability, Disable gimbal stability control
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    enable_laser, gimbal_enable_laser, Enable laser
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    disable_laser, gimbal_disable_laser, Disable laser
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    status, gimbal_status, Show current motor status
);
