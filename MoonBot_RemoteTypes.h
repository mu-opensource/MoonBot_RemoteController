/*
 * MoonBot_RemotTypes.h
 *
 *  Created on: 2019年4月3日
 *      Author: ysq
 */

#ifndef MOONBOT_MOONBOT_REMOTCONTROLLER_MOONBOT_REMOTTYPES_H_
#define MOONBOT_MOONBOT_REMOTCONTROLLER_MOONBOT_REMOTTYPES_H_

#include <stdio.h>

#define MR_DEBUG                        0
#if MR_DEBUG != 0
#define IPRINTF(s,...)                  printf("\e[0;32mI:" s "\e[0m", ##__VA_ARGS__)
#define WPRINTF(s,...)                  printf("\e[0;33min file:%s, function:%s, line: %d\nW:" s "\e[0m",\
                                                __FILE__, \
                                                __PRETTY_FUNCTION__,\
                                                __LINE__, \
                                                ##__VA_ARGS__)
#define EPRINTF(s,...)                  printf("\e[0;31min file:%s, function:%s, line: %d\nE:" s "\e[0m",\
                                                __FILE__,\
                                                __PRETTY_FUNCTION__,\
                                                __LINE__,\
                                                ##__VA_ARGS__)
#else
// XXX sometimes raises "unused variable" warning
#define IPRINTF(s,...)
#define WPRINTF(s,...)
#define EPRINTF(s,...)
#endif

#define MOONBOT_REMOT_VERSION           0x01
#define MOONBOT_REMOT_PRODUCT_ID        0x03
#define MOONBOT_REMOT_FORM_COMMON       0x00
#define MOONBOT_REMOT_FORM_ROBOT        0x01
#define MOONBOT_REMOT_FORM_MECH         0x02
#define MOONBOT_REMOT_FORM_ROVER        0x03

#define MR_CMD_BUFFER_LEN               50
// port init type
#define MR_PORT_NOT_USE                 0x00
#define MR_PORT_COMMON_DIN              0x01
#define MR_PORT_COMMON_DOUT             0x02
#define MR_PORT_COMMON_AIN              0x03
#define MR_PORT_COMMON_AOUT             0x04
#define MR_PORT_COMMON_PWM              0x06
#define MR_PORT_SPECIAL_DEVICED_TYPE    0xA0
#define MR_PORT_SPECIAL_MU              0x01
#define MR_PORT_SPECIAL_LED             0x02
#define MR_PORT_SPECIAL_SPEAKER         0x03
#define MR_PORT_SPECIAL_SERVO           0x04
#define MR_PORT_SPECIAL_MOTOR           0x05
#define MR_PORT_SPECIAL_ENCODER         0x06
// CMD define
#define MR_CMD_CONTROLLER               0x70
#define MR_CMD_LED                      0x71
#define MR_CMD_SERVO                    0x72
#define MR_CMD_MOTOR                    0x73
#define MR_CMD_WHEEL                    0x74
#define MR_CMD_BUZZER                   0x75
#define MR_CMD_SPEAKER                  0x76
#define MR_CMD_PORT                     0x77
#define MR_CMD_BUTTON                   0x78
#define MR_CMD_IMU                      0x79
#define MR_CMD_APP_BUTTON               0x80
// SUB-CMD define
// Controller:    0x70
#define MR_CMD_CONTROLLER_PORT_CONF     0x01
#define MR_CMD_CONTROLLER_FORM_CHECK    0x10
#define MR_CMD_CONTROLLER_READ_FORM     0x11
#define MR_CMD_CONTROLLER_EXIT          0x20
// LED:           0x71
#define MR_CMD_LED_WRITE                0x01
#define MR_CMD_LED_SET_COLOR            0x02
#define MR_CMD_LED_SHOW                 0x03
#define MR_CMD_LED_CLEAR                0x04
#define MR_CMD_LED_MATRIX               0x05
#define MR_CMD_LED_SIMPLE_WRITE         0xA1
#define MR_CMD_LED_EYES_EMOTION         0xA2
// Servo:         0x72
#define MR_CMD_SERVO_CALIBRATE          0x01
#define MR_CMD_SERVO_WRITE              0x02
#define MR_CMD_SERVO_READ               0x03
#define MR_CMD_SERVO_SET_TARGET_ANGLE   0x04
#define MR_CMD_SERVO_MOVE_ALL_SERVO     0x05
#define MR_CMD_SERVO_WRITE_ALL          0x06
#define MR_CMD_SERVO_READ_ALL           0x07
// Motor:         0x73
#define MR_CMD_MOTOR_WRITE_RPM          0x01
#define MR_CMD_MOTOR_READ_RPM           0x02
#define MR_CMD_MOTOR_WRITE              0x03
#define MR_CMD_MOTOR_READ               0x04
// TankeBase:     0x74
#define MR_CMD_WHEEL_CALIBRATE          0x01
#define MR_CMD_WHEEL_WRITE_RPM          0x02
#define MR_CMD_WHEEL_READ_RPM           0x03
#define MR_CMD_WHEEL_FORWARD            0x04
#define MR_CMD_WHEEL_BACKWARD           0x05
#define MR_CMD_WHEEL_TURN_LEFT          0x06
#define MR_CMD_WHEEL_TURN_RIHGHT        0x07
#define MR_CMD_WHEEL_STOP               0x08
#define MR_CMD_WHEEL_SET_DISTANCE_STEP  0x09
#define MR_CMD_WHEEL_SET_ANGLE_STEP     0x0A
// Buzzer:        0x75
#define MR_CMD_BUZZER_WRITE             0x01
#define MR_CMD_BUZZER_PLAY              0x02
#define MR_CMD_BUZZER_SET_TEMPO         0x03
// Speaker:       0x76
#define MR_CMD_SPEAKER_PLAY             0x01
#define MR_CMD_SPEAKER_SET              0x02
#define MR_CMD_SPEAKER_SET_VOLUME       0x03
#define MR_CMD_SPEAKER_SET_PLAY_MODE    0x04
#define MR_CMD_SPEAKER_MOTION           0x05
#define MR_CMD_SPEAKER_RATATION_READ    0x06
// Port:          0x77
#define MR_CMD_PORT_DIGITAL_WRITE       0x01
#define MR_CMD_PORT_DIGITAL_READ        0x02
#define MR_CMD_PORT_ANALOG_WRITE        0x03
#define MR_CMD_PORT_ANALOG_READ         0x04
// Button:        0x78
#define MR_CMD_BUTTON_READ              0x01
// IMU:           0x79
#define MR_CMD_IMU_CALIBRATE            0x01
#define MR_CMD_IMU_READ_MAG             0x02
#define MR_CMD_IMU_READ_ACC             0x03
#define MR_CMD_IMU_TEMPERATURE          0x04
// AppButton:     0x80
#define MR_CMD_APP_BUTTON_CLICK         0x01
#define MR_CMD_APP_BUTTON_DRAG          0x03

#endif /* MOONBOT_MOONBOT_REMOTCONTROLLER_MOONBOT_REMOTTYPES_H_ */
