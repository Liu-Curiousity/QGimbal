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

#define PROMPT_DISABLE_FIRST "QGimbal is running, please disable it first"
#define PROMPT_ENABLE_FIRST "QGimbal is not running, please enable it first"
#define PROMPT_UNKNOW_TARGET(cmd, key) "Unknown "#cmd" target: %s", key
#define PROMPT_MISSING_VALUE_FOR(cmd, key) ("Missing value for "#cmd" [%s]", key)

class ShellPlugs : public QGimbal {
public:
    static void print_version() {
        print_len("Software version %s", GIMBAL_SOFTWARE_VERSION);
    }

    static void gimbal_status() {
        print_len("Gimbal Status:");
        print_len("  Enabled            : %s", qgimbal.started ? "Yes" : "No");
        print_len("  Stability Enabled  : %s", qgimbal.stability_enabled ? "Yes" : "No");
        print_len("  Laser Enabled      : %s", qgimbal.laser_enabled ? "Yes" : "No");
        print_len("  CtrlMode           : %s",
                  qgimbal.getCtrlType() == CtrlType::CurrentCtrl ? "CurrentCtrl" :
                  qgimbal.getCtrlType() == CtrlType::SpeedCtrl ? "SpeedCtrl" :
                  qgimbal.getCtrlType() == CtrlType::AngleCtrl ? "AngleCtrl" :
                  qgimbal.getCtrlType() == CtrlType::StepAngleCtrl ? "StepAngleCtrl" :
                  qgimbal.getCtrlType() == CtrlType::LowSpeedCtrl ? "LowSpeedCtrl" : "Unknown");
        print_len("  IMU Angle          : yaw:%.3f rad, pitch:%.3f rad",
                  qgimbal.imu_angle.yaw, qgimbal.imu_angle.pitch);
        print_len("  IMU Speed          : yaw:%.3f rpm, pitch:%.3f rpm",
                  qgimbal.imu_speed.yaw, qgimbal.imu_speed.pitch);
        print_len("  Angle              : yaw:%.3f rad, pitch:%.3f rad",
                  qgimbal.motor_angle.yaw, qgimbal.motor_angle.pitch);
        print_len("  Speed              : yaw:%.3f rpm, pitch:%.3f rpm",
                  qgimbal.motor_speed.yaw, qgimbal.motor_speed.pitch);
        print_len("  Current            : yaw:%.3f A  , pitch:%.3f A  ",
                  qgimbal.motor_current.yaw, qgimbal.motor_current.pitch);
        print_len("  Voltage            : %.3f V", qgimbal.voltage);
    }

    static void gimbal_config_help() {
        print_len("Usage: config [--list | PARAM_PATH VALUE | key=value]");
        print_len("");
        print_len("Examples:");
        print_len("  config pid.speed.kp.yaw 0.1");
        print_len("  config pid.speed.ki.pitch=0.1");
        print_len("  config --help");
        print_len("  config --list");
        print_len("");
        print_len("Configuration Parameters:");
        print_len("  pid.speed.kp.[yaw|pitch]   : Speed PID proportional gain");
        print_len("  pid.speed.ki.[yaw|pitch]   : Speed PID integral gain");
        print_len("  pid.speed.kd.[yaw|pitch]   : Speed PID derivative gain");
        print_len("  pid.angle.kp.[yaw|pitch]   : Angle PID proportional gain");
        print_len("  pid.angle.ki.[yaw|pitch]   : Angle PID integral gain");
        print_len("  pid.angle.kd.[yaw|pitch]   : Angle PID derivative gain");
        print_len("  limit.speed.[yaw|pitch]    : Speed limit in rpm");
        print_len("  limit.current.[yaw|pitch]  : Current limit in A");
        print_len("  center.[yaw|pitch]         : Center position offset in rad");
    }

    static void gimbal_config_list() {
        print_len("Current Configuration:");
        print_len("pid.speed.kp.yaw = %.3g", qgimbal.pid_speed.yaw.kp);
        print_len("pid.speed.ki.yaw = %.3g", qgimbal.pid_speed.yaw.ki);
        print_len("pid.speed.kd.yaw = %.3g", qgimbal.pid_speed.yaw.kd);
        print_len("pid.angle.kp.yaw = %.3g", qgimbal.pid_angle.yaw.kp);
        print_len("pid.angle.ki.yaw = %.3g", qgimbal.pid_angle.yaw.ki);
        print_len("pid.angle.kd.yaw = %.3g", qgimbal.pid_angle.yaw.kd);

        print_len("pid.speed.kp.pitch = %.3g", qgimbal.pid_speed.pitch.kp);
        print_len("pid.speed.ki.pitch = %.3g", qgimbal.pid_speed.pitch.ki);
        print_len("pid.speed.kd.pitch = %.3g", qgimbal.pid_speed.pitch.kd);
        print_len("pid.angle.kp.pitch = %.3g", qgimbal.pid_angle.pitch.kp);
        print_len("pid.angle.ki.pitch = %.3g", qgimbal.pid_angle.pitch.ki);
        print_len("pid.angle.kd.pitch = %.3g", qgimbal.pid_angle.pitch.kd);

        print_len("limit.current.yaw = %.3g A", qgimbal.pid_speed.yaw.output_limit_p);
        print_len("limit.current.pitch = %.3g A", qgimbal.pid_speed.pitch.output_limit_p);

        print_len("uart.baud_rate = %u", qgimbal.uart_baud_rate);
    }

    static void gimbal_config(int argc, char *argv[]) {
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
                    print_len(
                        "Cannot reset IMU zero position while stability is enabled. Please disable stability first.");
                    return;
                }
                qgimbal.reset_imu();
                print_len("Setting config [zero_pos] for IMU");
            } else {
                qgimbal.setZeroPosition(qgimbal.motor_angle);
                print_len("Setting config [zero_pos]");
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
                    print_len("Invalid UART baud rate: %d, must be between 10'000 and 5'000'000",
                              static_cast<int>(valf));
                    return;
                }
                print_len("UART baud rate will be set after storing and rebooting");
            } else {
                print_len("Unknown config target: %s", key);
                return;
            }
            if (valf == 0) {
                print_len("Setting config [%s] = 0.000", key);
            } else {
                print_len("Setting config [%s] = %.3g", key, valf);
            }
        } else {
            print_len("Missing value for config [%s]", key);
        }
    }

    static void gimbal_ctrl_help() {
        print_len("Usage: ctrl [current Y P | low_speed Y P | speed Y P | step_angle Y P | angle Y P | key=y,p]");
        print_len("");
        print_len("Examples:");
        print_len("  ctrl %s 100 0", CtrlItems[1].name);
        print_len("  ctrl %s=100,0", CtrlItems[1].name);
        print_len("  ctrl --help");
        print_len("");
        print_len("Control Parameters:");
        for (const auto& item : CtrlItems) {
            print_len("  %-14s : %s (%s)", item.name, item.description, item.unit);
        }
    }

    static void gimbal_ctrl(const int argc, char *argv[]) {
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
            const gimbal_pair vals = {y_val, p_val};
            if (strcmp(key, "current") == 0) {
                print_len("Setting current Y:%.3f P:%.3f A", y_val, p_val);
                qgimbal.Ctrl(CtrlType::CurrentCtrl, vals);
            } else if (strcmp(key, "speed") == 0) {
                print_len("Setting speed Y:%.3f P:%.3f rpm", y_val, p_val);
                qgimbal.Ctrl(CtrlType::SpeedCtrl, vals);
            } else if (strcmp(key, "angle") == 0) {
                print_len("Setting angle Y:%.3f P:%.3f rad", y_val, p_val);
                qgimbal.Ctrl(CtrlType::AngleCtrl, vals);
            } else if (strcmp(key, "step_angle") == 0) {
                print_len("Stepping angle Y:%.3f P:%.3f rad", y_val, p_val);
                qgimbal.Ctrl(CtrlType::StepAngleCtrl, vals);
            } else if (strcmp(key, "low_speed") == 0) {
                print_len("Setting low_speed Y:%.3f P:%.3f rpm", y_val, p_val);
                qgimbal.Ctrl(CtrlType::LowSpeedCtrl, vals);
            } else {
                print_len("Unknown ctrl target: %s", key);
                gimbal_ctrl_help();
            }
        } else {
            print_len("Missing value for ctrl [%s], format: val_yaw val_pitch", key);
        }
    }

    static void gimbal_enable() {
        qgimbal.start();
        if (qgimbal.started) {
            print_len("QGimbal enabled");
        } else
            print_len("enable failed, please calibrate first");
    }

    static void gimbal_disable() {
        qgimbal.stop();
        print_len("QGimbal disabled");
    }

    static void gimbal_enable_stability() {
        qgimbal.enable_stability();
        if (qgimbal.stability_enabled) {
            print_len("QGimbal stability enabled");
        } else {
            print_len("enable failed");
        }
    }

    static void gimbal_disable_stability() {
        qgimbal.disable_stability();
        print_len("QGimbal stability control disabled");
    }

    static void gimbal_enable_laser() {
        qgimbal.enable_laser();
        if (qgimbal.laser_enabled) {
            print_len("Laser enabled");
        } else {
            print_len("enable failed");
        }
    }

    static void gimbal_disable_laser() {
        qgimbal.disable_laser();
        print_len("Laser disabled");
    }

    static void gimbal_restore() {
        print_len("Are you sure you want to restore factory settings? (y/n)");
        char response;
        while (!shellRead(&response, 1)) {
            delay_ms(1); // Wait for input
        }
        if (response != 'y' && response != 'Y') {
            print_len("Factory restore cancelled");
            return;
        }
        qgimbal.restore_calibration(); // 恢复出厂设置
        print_len("QGimbal factory restore completed");
        gimbal_config_list();
    }

    static void gimbal_store() {
        if (qgimbal.started) {
            print_len("QGimbal is running, please disable it first");
            return;
        }
        gimbal_config_list();
        print_len("Are you sure you want to store configurations? (y/n)");
        char response;
        while (!shellRead(&response, 1)) {
            delay_ms(1); // Wait for input
        }
        if (response != 'y' && response != 'Y') {
            print_len("Store operation cancelled");
            return;
        }
        qgimbal.freeze_storage_calibration(
            static_cast<StorageStatus>(STORAGE_PID_PARAMETER_OK | // 储存PID参数
                                       STORAGE_LIMIT_OK |         // 储存限制参数
                                       STORAGE_PLUG_OK)           // 储存ID
        );
        print_len("Store configuration completed");
    }

    static void shell_reboot() {
        qgimbal.reboot();
    }

private:
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

    static void print_len(const char *format, ...) {
        if (shell.write != silent) {
            va_list args;
            va_start(args, format);
            vprintf(format, args);
            va_end(args);
            printf("\r\n");
        }
    }

    static void print(const char *format, ...) {
        if (shell.write != silent) {
            va_list args;
            va_start(args, format);
            vprintf(format, args);
            va_end(args);
        }
    }

    static signed short silent(char *data, unsigned short len) {
        (void)data;
        (void)len;
        return 0;
    }

    // 解析 key=value 格式
    static char* parse_key_value_arg(char *arg) {
        if (!arg) return nullptr;
        char *eq = strchr(arg, '=');
        if (!eq) return nullptr;
        *eq = '\0';
        return eq + 1;
    }


    class Item {
    public:
        const char *name;
        const char *description;
        const char *unit;
        const char *format;
        void (*print_value)(const Item&);
        bool (*set_value)(gimbal_pair<float>);

        template <size_t N>
        static const Item* find_item(const Item (&items)[N], const char *key) {
            return find_item(items, N, key);
        }

        static const Item* find_item(const Item items[], const size_t N, const char *key) {
            for (size_t i = 0; i < N; ++i) {
                if (strcmp(items[i].name, key) == 0) return &items[i];
            }
            return nullptr;
        }
    };

    //
    // inline static const Item ConfigItems[] = {
    //     {
    //         "pid.speed.kp.yaw", "Speed PID proportional gain", nullptr, "%.3g",
    //         [](const Item& self) {
    //             print(self.format, qd4310.PID_Speed.kp);
    //         },
    //         [](float value) {
    //             qgimbal.setPID(value, std::nullopt, std::nullopt,
    //                            std::nullopt, std::nullopt, std::nullopt);
    //             return true;
    //         }
    //     },
    //     {
    //         "pid.speed.ki.yaw", "Speed PID integral gain", nullptr, "%.3g",
    //         [](const Item& self) {
    //             print(self.format, qd4310.PID_Speed.ki);
    //         },
    //         [](float value) {
    //             qd4310.setPID(std::nullopt, value, std::nullopt,
    //                           std::nullopt, std::nullopt, std::nullopt);
    //             return true;
    //         }
    //     },
    //     {
    //         "pid.speed.kd.yaw", "Speed PID derivative gain", nullptr, "%.3g",
    //         [](const Item& self) {
    //             print(self.format, qd4310.PID_Speed.kd);
    //         },
    //         [](float value) {
    //             qd4310.setPID(std::nullopt, std::nullopt, value,
    //                           std::nullopt, std::nullopt, std::nullopt);
    //             return true;
    //         }
    //     },
    //     {
    //         "pid.angle.kp", "Angle PID proportional gain", nullptr, "%.3g",
    //         [](const Item& self) {
    //             print(self.format, qd4310.PID_Angle.kp);
    //         },
    //         [](float value) {
    //             qd4310.setPID(std::nullopt, std::nullopt, std::nullopt,
    //                           value, std::nullopt, std::nullopt);
    //             return true;
    //         }
    //     },
    //     {
    //         "pid.angle.ki", "Angle PID integral gain", nullptr, "%.3g",
    //         [](const Item& self) {
    //             print(self.format, qd4310.PID_Angle.ki);
    //         },
    //         [](float value) {
    //             qd4310.setPID(std::nullopt, std::nullopt, std::nullopt,
    //                           std::nullopt, value, std::nullopt);
    //             return true;
    //         }
    //     },
    //     {
    //         "pid.angle.kd", "Angle PID derivative gain", nullptr, "%.3g",
    //         [](const Item& self) {
    //             print(self.format, qd4310.PID_Angle.kd);
    //         },
    //         [](float value) {
    //             qd4310.setPID(std::nullopt, std::nullopt, std::nullopt,
    //                           std::nullopt, std::nullopt, value);
    //             return true;
    //         }
    //     },
    //     {
    //         "limit.speed", "Speed limit in rpm", "rpm", "%.3g",
    //         [](const Item& self) {
    //             if (!qd4310.PID_Angle.output_limit_p)
    //                 print("no limit");
    //             else
    //                 print(self.format, qd4310.PID_Angle.output_limit_p.value());
    //         },
    //         [](float value) {
    //             return qd4310.setLimit(value, std::nullopt);
    //         }
    //     },
    //     {
    //         "limit.current", "Current limit in A", "A", "%.3g",
    //         [](const Item& self) {
    //             if (!qd4310.PID_Speed.output_limit_p)
    //                 print("no limit");
    //             else
    //                 print(self.format, qd4310.PID_Speed.output_limit_p.value());
    //         },
    //         [](const float value) {
    //             return qd4310.setLimit(std::nullopt, value);
    //         }
    //     },
    //     {
    //         "can.id", "CAN ID of the motor (0-7)", nullptr, "%03u",
    //         [](const Item& self) {
    //             print(self.format, qd4310.ID);
    //         },
    //         [](const float value) {
    //             if (!qd4310.setID(static_cast<uint8_t>(value))) {
    //                 print_len("Invalid CAN ID: %d, must be between 0 and 7", static_cast<int>(value));
    //                 return false;
    //             }
    //             return true;
    //         }
    //     },
    //     {
    //         "timeout", "Communication timeout", "s", "%.3g",
    //         [](const Item& self) {
    //             print(self.format, qd4310.getTimeout());
    //         },
    //         [](const float value) { return qd4310.setTimeout(value); }
    //     },
    //     {
    //         "uart.baud_rate", "UART BaudRate of the motor (50K-10M)", "bps", "%u",
    //         [](const Item& self) {
    //             print(self.format, qd4310.uart_baud_rate);
    //         },
    //         [](const float value) {
    //             if (!qd4310.setUartBaudRate(static_cast<uint32_t>(value))) {
    //                 print_len("Invalid UART baud rate: %d, must be between 10'000 and 10'000'000",
    //                           static_cast<int>(value));
    //                 return false;
    //             }
    //             print_len("UART baud rate will be set after storing and rebooting");
    //             return true;
    //         }
    //     },
    //     {
    //         "zero_pos", "Position zero offset in rad", nullptr, nullptr,
    //         nullptr,
    //         [](const float value) {
    //             if (qd4310.started) {
    //                 print_len(PROMPT_DISABLE_FIRST);
    //                 return false;
    //             }
    //             return qd4310.setZeroPosition(std::isnan(value) ? qd4310.getAngle() : value);
    //         }
    //     },
    //     {
    //         "can.baud_rate", "CAN bus baud rate (fixed)", "bps", "%u",
    //         [](const Item& self) {
    //             (void)self;
    //             print("1'000'000");
    //         },
    //         nullptr
    //     },
    // };

    inline static const Item CtrlItems[] = {
        {
            "current", "Set current", "A", "%.3g",
            nullptr,
            [](const gimbal_pair<float> value) {
                qgimbal.Ctrl(CtrlType::CurrentCtrl, value);
                return true;
            }
        },
        {
            "speed", "Set speed", "rpm", "%.3g",
            nullptr,
            [](const gimbal_pair<float> value) {
                qgimbal.Ctrl(CtrlType::SpeedCtrl, value);
                return true;
            }
        },
        {
            "angle", "Set angle", "rad", "%.3g",
            nullptr,
            [](const gimbal_pair<float> value) {
                qgimbal.Ctrl(CtrlType::AngleCtrl, value);
                return true;
            }
        },
        {
            "step_angle", "Step an specific angle", "rad", "%.3g",
            nullptr,
            [](const gimbal_pair<float> value) {
                qgimbal.Ctrl(CtrlType::StepAngleCtrl, value);
                return true;
            }
        },
        {
            "low_speed", "Set speed by increasing angle", "rpm", "%.3g",
            nullptr,
            [](const gimbal_pair<float> value) {
                qgimbal.Ctrl(CtrlType::LowSpeedCtrl, value);
                return true;
            }
        },
    };
};

SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    version, ShellPlugs::print_version, Show version info
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    reboot, ShellPlugs::shell_reboot, reboot system
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    store, ShellPlugs::gimbal_store, Store configurations
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    restore, ShellPlugs::gimbal_restore, Factory restore
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    ctrl, ShellPlugs::gimbal_ctrl, Set control targets
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    config, ShellPlugs::gimbal_config, Configure system parameters
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    disable, ShellPlugs::gimbal_disable, Disable FOC control
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    enable, ShellPlugs::gimbal_enable, Enable FOC control
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    enable_stability, ShellPlugs::gimbal_enable_stability, Enable gimbal stability control
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    disable_stability, ShellPlugs::gimbal_disable_stability, Disable gimbal stability control
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    enable_laser, ShellPlugs::gimbal_enable_laser, Enable laser
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    disable_laser, ShellPlugs::gimbal_disable_laser, Disable laser
);
SHELL_EXPORT_CMD(
    SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
    status, ShellPlugs::gimbal_status, Show current motor status
);
