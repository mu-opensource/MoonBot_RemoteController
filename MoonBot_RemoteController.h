/*
 * MoonBot_RemotController.h
 *  Created on: 2019年4月3日
 *      Author: ysq
 *
 * MooBot remote framework
 * ┌───────────────────────┐
 * │ [1] Physical Layer    │
 * ├───────────────────────┤
 * │ [2] Data Link Layer   │
 * ├───────────────────────┤
 * │ [3] Protocol Layer    │
 * ├───────────────────────┤
 * │ [4] Application Layer │
 * └───────────────────────┘
 * Class Stream                                     // Physical Layer
 *  └─ Class MuVisionSensorUart                     // Data Link Layer
 *    └─ Class MoonBotRemoteProtocolAnalysis        // Protocol Layer
 *      └─ Class MoonBotRemoteController            // Common Application Layer
 *        ├─ Class MoonBotMechRemoteController      // MECH Application Layer
 *        ├─ Class MoonBotRoverRemoteController     // Rover Application Layer
 *        └─ Class MoonBotRobotRemoteController     // Robot Application Layer
 *
 */

#ifndef MOONBOT_MOONBOT_REMOTCONTROLLER_MOONBOT_REMOTCONTROLLER_H_
#define MOONBOT_MOONBOT_REMOTCONTROLLER_MOONBOT_REMOTCONTROLLER_H_

#include <Stream.h>
#include <MoonBot.h>
#include <MoonBot_RemotePotocolAnalysis.h>

// match command MICRO
#define MR_ERROR_CHECK(err) \
  do {\
    error_ = err;\
    if (error_ == MU_OK) {\
      response(cmd_, sub_cmd_);\
      return;\
    }\
  } while(0)
#define MACTCH_BEGIN        \
  switch (cmd_) {
#define MATCH_END           \
    default:\
      break;\
  }
#define MACTCH_CMD(cmd)     \
    case cmd:\
      switch(sub_cmd_) {
#define MACTCH_SUB_CMD(sub_cmd, fn_p, ...)     \
      case sub_cmd: \
        MR_ERROR_CHECK(fn_p(##__VA_ARGS__));\
        break

class MoonBotRemoteController : public MoonBotRemoteProtocolAnalysis {
 public:
  MoonBotRemoteController(MuUart::hw_port_t uart,
                         uint32_t address,
                         bool response_enable = true);
  virtual ~MoonBotRemoteController(void);
  MoonBotRemoteController& operator=(const MoonBotRemoteController &) = delete;
  MoonBotRemoteController(const MoonBotRemoteController&) = delete;

  virtual void run(void);
  bool exit_ = false;

 private:
 protected:
  // command match
  virtual void CommandMatcher(void);
  virtual void RunEvent(void);
  // device functions
  void commonPinInit(uint8_t pin, uint8_t mode);
  void specialPortInit(moonbot_port_t port, uint8_t dev_id);
  inline uint8_t portCFG(void);
  inline uint8_t controllerFormCheck(void);
  inline uint8_t controllerReadForm(void);
  inline uint8_t controllerExit(void);
  // LED
  inline uint8_t ledWrite(void);
  inline uint8_t ledSetColor(void);
  inline uint8_t ledShow(void);
  inline uint8_t ledClear(void);
  inline uint8_t ledMatrix(void);
  inline uint8_t ledSimpleWrite(void);
         uint8_t ledAction(uint8_t num);
  inline uint8_t ledEyesEmotion(void);
  // Servo
  inline uint8_t servoCalibrate(void);
  inline uint8_t servoWrite(void);
  inline uint8_t servoRead(void);
  inline uint8_t servoSetTargetAngle(void);
  inline uint8_t servoMoveAllServo(void);
  inline uint8_t servoWriteAll(void);
  inline uint8_t servoReadAll(void);
         void    setAllServoPower(bool state);
  // Motor
  inline uint8_t motorWriteRpm(void);
  inline uint8_t motorReadRpm(void);
  inline uint8_t motorWrite(void);
  inline uint8_t motorRead(void);
  // Wheel
  inline uint8_t wheelCalibrate(void);
  inline uint8_t wheelWriteRpm(void);
  inline uint8_t wheelReadRpm(void);
  inline uint8_t wheelForward(void);
  inline uint8_t wheelBackward(void);
  inline uint8_t wheelTurnLeft(void);
  inline uint8_t wheelTurnRight(void);
  inline uint8_t wheelStop(void);
  inline uint8_t wheelSetDistanceStep(void);
  inline uint8_t wheelSetAngleStep(void);
  // Buzzer
  inline uint8_t buzzerWrite(void);
         uint8_t buzzerPlayMusic(uint32_t music_id);
  inline uint8_t buzzerPlay(void);
  inline uint8_t buzzerSetTempo(void);
  // Speaker
  inline uint8_t speakerPlay(void);
  inline uint8_t speakerSet(void);
  inline uint8_t speakerSetVolume(void);
  inline uint8_t speakerSetPlayMode(void);
  // Port
  inline uint8_t portDigitalWrite(void);
  inline uint8_t portDigitalRead(void);
  inline uint8_t portAnalogWrite(void);
  inline uint8_t portAnalogRead(void);
  // Button
  inline uint8_t buttonRead(void);
  // IMU
  inline uint8_t imuCalibrate(void);
  inline uint8_t imuReadCompass(void);
  inline uint8_t imuReadAcceleration(void);
  inline uint8_t imuTemperature(void);
  inline uint8_t imuMotion(void);
  inline uint8_t imuRatationRead(void);
  // AppButton
  inline uint8_t appButtonClick(void);
  inline uint8_t appButtonDrag(void);

  void enableAllServoPower(bool state);
  void enableTankBaseStopEvent(unsigned long time2stop = 15000) {
    tank_base_stop_event_enable_ = true;
    time2stop_tank_base_ = millis()+time2stop;
  }

  void clearBuffer();

  SoftwareSerial* sw_serial_ = NULL;
  enum moonbot_remot_common_event_t {
    kMoonBotRemoteCommonEventNone,
    kMoonBotRemoteCommonEventNum,
  };
  unsigned long time2stop_tank_base_ = 0;
  bool tank_base_stop_event_enable_ = false;
  uint8_t remote_event_ = kMoonBotRemoteCommonEventNone;
  uint8_t form_ = MOONBOT_REMOT_FORM_COMMON;
  uint8_t error_ = MU_SLAVE_UNKNOW_COMMAND;
  uint8_t eyes_color_[2][3] = { { 0 }, { 0 } };
  bool motor_encoder_enable_[kMotorNum] = {false};
  static const uint8_t color_map_[][3];
  static const uint8_t beat_map_[];
  static const uint16_t tone_map_[];
  uint16_t tempo_ = 120;
};



#endif /* MOONBOT_MOONBOT_REMOTCONTROLLER_MOONBOT_REMOTCONTROLLER_H_ */
