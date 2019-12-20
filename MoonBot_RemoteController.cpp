/*
 * MoonBot_RemotController.cpp
 *
 *  Created on: 2019年4月3日
 *      Author: ysq
 */

#include <MoonBot_RemoteController.h>
const uint8_t MoonBotRemoteController::color_map_[][3] = {
  //  R    G     B
  { 0x00, 0x00, 0x00 },   // close
  { 0x00, 0x00, 0xFF },   // blue
  { 0x00, 0xFF, 0x00 },   // green
  { 0x00, 0xFF, 0xFF },   // cyan
  { 0xFF, 0x00, 0x00 },   // red
  { 0xFF, 0x00, 0xFF },   // purple
  { 0xFF, 0xFF, 0x00 },   // yellow
  { 0xFF, 0xFF, 0xFF },   // red
};
const uint16_t MoonBotRemoteController::tone_map_[] = {
NOTE_0,
NOTE_C4,
NOTE_D4,
NOTE_E4,
NOTE_F4,
NOTE_G4,
NOTE_A4,
NOTE_B4,
NOTE_C5,
NOTE_D5,
NOTE_E5,
NOTE_F5,
NOTE_G5,
NOTE_A5,
NOTE_B5, };
const uint8_t MoonBotRemoteController::beat_map_[] = {
    BEAT_FRACTION_BREVE,
    BEAT_FRACTION_DOUBLE,
    BEAT_FRACTION_WHOLE,
    BEAT_FRACTION_THREE_QUARTER,
    BEAT_FRACTION_HALF,
    BEAT_FRACTION_QUARTER,
    BEAT_FRACTION_EIGHTH,
    BEAT_FRACTION_SIXTEENTH,};

MoonBotRemoteController::MoonBotRemoteController(MuUart::hw_port_t uart,
                                                 uint32_t address,
                                                 bool response_enable)
    : MoonBotRemoteProtocolAnalysis(uart,address,response_enable) {
  IMU.enable();
  onBoardLED.begin();
  onBoardLED.clear();
  onBoardLED.show();
  pinMode(MOONBOT_PIN_BUTTON_A, INPUT);
  pinMode(MOONBOT_PIN_BUTTON_B, INPUT);
  pinMode(MOONBOT_PIN_BUZZER_SIG, OUTPUT);
  pinMode(MOONBOT_PIN_BUZZER_SHDW, OUTPUT);
  digitalWrite(MOONBOT_PIN_BUZZER_SHDW, LOW);
  for (int i = 0; i < kServoNum; ++i) {
    m_servo[i].attach((moonbot_servo_t)i);
    m_servo[i].write(90);
    m_servo[i].power(false);
  }

  // TODO default setting
  pinMode(moonbotPortToPin(kPort3, kPortPin1), INPUT);
  pinMode(moonbotPortToPin(kPort5, kPortPin1), INPUT);
  pinMode(moonbotPortToPin(kPort7, kPortPin1), INPUT);
  pinMode(moonbotPortToPin(kPort8, kPortPin1), INPUT);
  moonbot_eyes.begin();
  moonbot_eyes.clear();
  moonbot_eyes.show();
  speaker.begin(Serial2);
  TankBase.begin();
}

MoonBotRemoteController::~MoonBotRemoteController(void) {
  for (int i = 0; i < kServoNum; ++i) {
    m_servo[i].detach();
  }
}

void MoonBotRemoteController::run(void) {
  if (capture() == MU_OK) {
    remote_event_ = kMoonBotRemoteCommonEventNone;      // new command coming, clean event.
    error_ = MU_SLAVE_UNKNOW_COMMAND;                   // reset error to error command
    CommandMatcher();
  }
  RunEvent();
}

void MoonBotRemoteController::CommandMatcher(void) {
  // Not available command: break; otherwise: return;
  MACTCH_BEGIN
    //==============Controller==============
    MACTCH_CMD(MR_CMD_CONTROLLER)
      MACTCH_SUB_CMD(MR_CMD_CONTROLLER_PORT_CONF, portCFG);
      MACTCH_SUB_CMD(MR_CMD_CONTROLLER_FORM_CHECK, controllerReadForm);
      MACTCH_SUB_CMD(MR_CMD_CONTROLLER_READ_FORM, controllerFormCheck);
      MACTCH_SUB_CMD(MR_CMD_CONTROLLER_EXIT, controllerExit);
    MATCH_END
    break;
    //==============LED==============
    MACTCH_CMD(MR_CMD_LED)
      MACTCH_SUB_CMD(MR_CMD_LED_WRITE, ledWrite);
      MACTCH_SUB_CMD(MR_CMD_LED_SET_COLOR, ledSetColor);
      MACTCH_SUB_CMD(MR_CMD_LED_SHOW, ledShow);
      MACTCH_SUB_CMD(MR_CMD_LED_CLEAR, ledClear);
      MACTCH_SUB_CMD(MR_CMD_LED_MATRIX, ledMatrix);
      MACTCH_SUB_CMD(MR_CMD_LED_SIMPLE_WRITE, ledSimpleWrite);
      MACTCH_SUB_CMD(MR_CMD_LED_EYES_EMOTION, ledEyesEmotion);
    MATCH_END
    break;
    //==============Servo==============
    MACTCH_CMD(MR_CMD_SERVO)
      MACTCH_SUB_CMD(MR_CMD_SERVO_CALIBRATE, servoCalibrate);
      MACTCH_SUB_CMD(MR_CMD_SERVO_WRITE, servoWrite);
      MACTCH_SUB_CMD(MR_CMD_SERVO_READ, servoRead);
      MACTCH_SUB_CMD(MR_CMD_SERVO_SET_TARGET_ANGLE, servoSetTargetAngle);
      MACTCH_SUB_CMD(MR_CMD_SERVO_MOVE_ALL_SERVO, servoMoveAllServo);
      MACTCH_SUB_CMD(MR_CMD_SERVO_WRITE_ALL, servoWriteAll);
      MACTCH_SUB_CMD(MR_CMD_SERVO_READ_ALL, servoReadAll);
    MATCH_END
    break;
    //==============Motor==============
    MACTCH_CMD(MR_CMD_MOTOR)
      MACTCH_SUB_CMD(MR_CMD_MOTOR_WRITE_RPM, motorWriteRpm);
      MACTCH_SUB_CMD(MR_CMD_MOTOR_READ_RPM, motorReadRpm);
      MACTCH_SUB_CMD(MR_CMD_MOTOR_WRITE, motorWrite);
      MACTCH_SUB_CMD(MR_CMD_MOTOR_READ, motorRead);
    MATCH_END
    break;
    //==============TankBase==============
    MACTCH_CMD(MR_CMD_WHEEL)
      MACTCH_SUB_CMD(MR_CMD_WHEEL_CALIBRATE, wheelCalibrate);
      MACTCH_SUB_CMD(MR_CMD_WHEEL_WRITE_RPM, wheelWriteRpm);
      MACTCH_SUB_CMD(MR_CMD_WHEEL_READ_RPM, wheelReadRpm);
      MACTCH_SUB_CMD(MR_CMD_WHEEL_FORWARD, wheelForward);
      MACTCH_SUB_CMD(MR_CMD_WHEEL_BACKWARD, wheelBackward);
      MACTCH_SUB_CMD(MR_CMD_WHEEL_TURN_LEFT, wheelTurnLeft);
      MACTCH_SUB_CMD(MR_CMD_WHEEL_TURN_RIHGHT, wheelTurnRight);
      MACTCH_SUB_CMD(MR_CMD_WHEEL_STOP, wheelStop);
      MACTCH_SUB_CMD(MR_CMD_WHEEL_SET_DISTANCE_STEP, wheelSetDistanceStep);
      MACTCH_SUB_CMD(MR_CMD_WHEEL_SET_ANGLE_STEP, wheelSetAngleStep);
    MATCH_END
    break;
    //==============Buzzer==============
    MACTCH_CMD(MR_CMD_BUZZER)
      MACTCH_SUB_CMD(MR_CMD_BUZZER_WRITE, buzzerWrite);
      MACTCH_SUB_CMD(MR_CMD_BUZZER_PLAY, buzzerPlay);
      MACTCH_SUB_CMD(MR_CMD_BUZZER_SET_TEMPO, buzzerSetTempo);
    MATCH_END
    break;
    //==============Speaker==============
    MACTCH_CMD(MR_CMD_SPEAKER)
      MACTCH_SUB_CMD(MR_CMD_SPEAKER_PLAY, speakerPlay);
      MACTCH_SUB_CMD(MR_CMD_SPEAKER_SET, speakerSet);
      MACTCH_SUB_CMD(MR_CMD_SPEAKER_SET_VOLUME, speakerSetVolume);
      MACTCH_SUB_CMD(MR_CMD_SPEAKER_SET_PLAY_MODE, speakerSetPlayMode);
    MATCH_END
    break;
    //==============Port==============
    MACTCH_CMD(MR_CMD_PORT)
      MACTCH_SUB_CMD(MR_CMD_PORT_DIGITAL_WRITE, portDigitalWrite);
      MACTCH_SUB_CMD(MR_CMD_PORT_DIGITAL_READ, portDigitalRead);
      MACTCH_SUB_CMD(MR_CMD_PORT_ANALOG_WRITE, portAnalogWrite);
      MACTCH_SUB_CMD(MR_CMD_PORT_ANALOG_READ, portAnalogRead);
    MATCH_END
    break;
    //==============Button==============
    MACTCH_CMD(MR_CMD_BUTTON)
      MACTCH_SUB_CMD(MR_CMD_BUTTON_READ, buttonRead);
    MATCH_END
    break;
    //==============IMU==============
    MACTCH_CMD(MR_CMD_IMU)
      MACTCH_SUB_CMD(MR_CMD_IMU_CALIBRATE, imuCalibrate);
      MACTCH_SUB_CMD(MR_CMD_IMU_READ_MAG, imuReadCompass);
      MACTCH_SUB_CMD(MR_CMD_IMU_READ_ACC, imuReadAcceleration);
      MACTCH_SUB_CMD(MR_CMD_IMU_TEMPERATURE, imuTemperature);
      MACTCH_SUB_CMD(MR_CMD_SPEAKER_MOTION, imuMotion);
      MACTCH_SUB_CMD(MR_CMD_SPEAKER_RATATION_READ, imuRatationRead);
    MATCH_END
    break;
    //==============AppButton==============
    MACTCH_CMD(MR_CMD_APP_BUTTON)
      MACTCH_SUB_CMD(MR_CMD_APP_BUTTON_CLICK, appButtonClick);
      MACTCH_SUB_CMD(MR_CMD_APP_BUTTON_DRAG, appButtonDrag);
    MATCH_END
    break;
  MATCH_END
  EPRINTF("Not available command: 0x%02x, sub-command: 0x%02x\n",
          cmd_, sub_cmd_);
  responseError(error_);
}
void MoonBotRemoteController::RunEvent(void) {
  if (tank_base_stop_event_enable_ && millis() >= time2stop_tank_base_) {
    tank_base_stop_event_enable_ = false;
    TankBase.write(0, 0);
  }
}

void MoonBotRemoteController::enableAllServoPower(bool state) {
  for (int i = 0; i < kServoNum; ++i) {
    m_servo[i].power(state);
  }
}

void MoonBotRemoteController::clearBuffer() {
  while (MoonBotRemoteProtocolAnalysis::available()) {
    MoonBotRemoteProtocolAnalysis::receive();
  }
}

// device functions
static bool swSerialInit(SoftwareSerial* sw_serial,
                               uint8_t rx, uint8_t tx) {
  sw_serial = new SoftwareSerial(rx, tx);
  sw_serial->begin(9600);
  return sw_serial->isListening();
}
static void swSerialDeinit (SoftwareSerial* sw_serial) {
  if (sw_serial) delete sw_serial;
  sw_serial = NULL;
}

// port config function
void MoonBotRemoteController::commonPinInit(uint8_t pin, uint8_t mode) {
  switch (mode) {
    case MR_PORT_COMMON_DIN:
      pinMode(pin, INPUT_PULLUP);
      break;
    case MR_PORT_COMMON_AIN:
      pinMode(pin, INPUT);
      break;
    case MR_PORT_COMMON_DOUT:
    case MR_PORT_COMMON_AOUT:
    case MR_PORT_COMMON_PWM:
      pinMode(pin, OUTPUT);
      break;
    default:
      return;
  }
}
void MoonBotRemoteController::specialPortInit(moonbot_port_t port, uint8_t dev_id) {
  switch(dev_id) {
    case MR_PORT_SPECIAL_MU:
      IPRINTF("MoonBot Mu enable on port:%d\n", port);
      break;
    case MR_PORT_SPECIAL_LED:
#ifdef MOONBOT_PORT_SERIAL
      if (port == MOONBOT_PORT_SERIAL) {
        Serial.end();
      }
#endif
#ifdef MOONBOT_PORT_SERIAL1
        if (port == MOONBOT_PORT_SERIAL1) {
          Serial1.end();
        }
#endif
#ifdef MOONBOT_PORT_SERIAL2
        if (port == MOONBOT_PORT_SERIAL2) {
          Serial2.end();
        }
#endif
#ifdef MOONBOT_PORT_SERIAL3
        if (port == MOONBOT_PORT_SERIAL3) {
          Serial3.end();
        }
#endif
      moonbot_eyes.setPin(moonbotPortToPin(port, kPortPin1));
      moonbot_eyes.begin();
      moonbot_eyes.clear();
      moonbot_eyes.show();
      IPRINTF("MoonBot eyes led enable on port:%d\n", port);
      break;
    case MR_PORT_SPECIAL_SPEAKER:
      switch(port) {
#ifdef MOONBOT_PORT_SERIAL
        case MOONBOT_PORT_SERIAL:
          speaker.begin(Serial);
          break;
#endif
#ifdef MOONBOT_PORT_SERIAL1
        case MOONBOT_PORT_SERIAL1:
          speaker.begin(Serial1);
          break;
#endif
#ifdef MOONBOT_PORT_SERIAL2
        case MOONBOT_PORT_SERIAL2:
          speaker.begin(Serial2);
          break;
#endif
#ifdef MOONBOT_PORT_SERIAL3
        case MOONBOT_PORT_SERIAL3:
          speaker.begin(Serial3);
          break;
#endif
        default:
          if (sw_serial_) {
            // software serial is occupied by other port
            WPRINTF("Software serial is occupied by other port\n");
            return;
          }
          if (swSerialInit(sw_serial_,
                           moonbotPortToPin(port, kPortPin1),
                           moonbotPortToPin(port, kPortPin2)) == false) {
            // not available software serial port
            swSerialDeinit(sw_serial_);
            EPRINTF("Port:%d is not an available software serial port\n", port);
            return;
          }
          speaker.begin(*sw_serial_);
          break;
      }
      IPRINTF("MoonBot speaker enable on port:%d\n", port);
      break;
    case MR_PORT_SPECIAL_ENCODER:
      if (port == kPort4) {
        motor_encoder_enable_[0] = true;
      } else if (port == kPort6) {
        motor_encoder_enable_[1] = true;
      }
      IPRINTF("MoonBot Encoder enable on port:%d\n", port);
      break;
    default:
      break;
  }
}
uint8_t MoonBotRemoteController::portCFG(void) {
  resetParameterIndex();
  uint8_t port[2];
  // kPortNum of ports
  for (int i = 0; i < kPortNum; ++i) {
    port[0] = getParameter<uint8_t>();
    port[1] = getParameter<uint8_t>();
    if (port[0] == MR_PORT_SPECIAL_DEVICED_TYPE) {
      specialPortInit((moonbot_port_t)i, port[1]);
      continue;
    }
    for (int j = 0; j < kPortPinNum; ++j) {
      commonPinInit(moonbotPortToPin((moonbot_port_t) i, (port_pin_t) j),
                    port[j]);
    }
  }
  // kServoNum of servos
  for (int i = 0; i < kServoNum; ++i) {
    port[0] = getParameter<uint8_t>();
    port[1] = getParameter<uint8_t>();
    if (port[0] == MR_PORT_NOT_USE) {
      IPRINTF("Servo%d not use\n", i);
      continue;
    }
    if (port[0] == MR_PORT_SPECIAL_DEVICED_TYPE
        && port[1] == MR_PORT_SPECIAL_SERVO) {
      m_servo[i].power(true);
      m_servo[i].write(90);
      IPRINTF("Servo port%d: enable\n", i);
      continue;
    }
    EPRINTF("Servo port:%d, unsupported device ID: %hu,%hu\n",
            i, port[0],port[1]);
  }
  // kServoNum of servos
  for (int i = 0; i < kMotorNum; ++i) {
    port[0] = getParameter<uint8_t>();
    port[1] = getParameter<uint8_t>();
    if (port[0] == MR_PORT_NOT_USE) {
      IPRINTF("Motor%d not use\n", i);
      continue;
    }
    if (port[0] == MR_PORT_SPECIAL_DEVICED_TYPE
        && port[1] == MR_PORT_SPECIAL_MOTOR) {
      switch(i) {
        case kMotor1:
          Motor1.begin(true, motor_encoder_enable_[0]);
          break;
        case kMotor2:
          Motor2.begin(false, motor_encoder_enable_[1]);
          break;
        default:
          EPRINTF("Unsupported motor number:%d\n", i);
          break;
      }
      IPRINTF("Motor%d: enable\n", i);
    }
  }
  onBoardLED.setBrightness(255);
  onBoardLED.clear();
  onBoardLED.show();
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::controllerFormCheck(void) {
  resetParameterIndex();
  uint8_t product_id = getParameter<uint8_t>();
  uint8_t form = getParameter<uint8_t>();
  uint8_t version = getParameter<uint8_t>();
  uint8_t is_form = false;
  if (product_id == MOONBOT_REMOT_PRODUCT_ID
      && form == form_
      && version == MOONBOT_REMOT_VERSION) {
    is_form = true;
  }

  resetParameterIndex();
  setParameter<uint8_t>(is_form);
  return MU_OK;
}
uint8_t MoonBotRemoteController::controllerReadForm(void) {
  resetParameterIndex();
  setParameter<uint8_t>(MOONBOT_REMOT_PRODUCT_ID);
  setParameter<uint8_t>(form_);
  setParameter<uint8_t>(MOONBOT_REMOT_VERSION);
  return MU_OK;
}
uint8_t MoonBotRemoteController::controllerExit(void) {
  exit_ = true;
  resetParameterIndex();
  return MU_OK;
}
// LED
// test CMD:  FF0C60710101008080805EED    // set on board led to rgb(0x80, 0x80, 0x80)
//            FF0C6071010100000000DEED    // close on board led
uint8_t MoonBotRemoteController::ledWrite(void) {
  resetParameterIndex();
  uint8_t led_id = getParameter<uint8_t>();
  uint8_t r = getParameter<uint8_t>();
  uint8_t g = getParameter<uint8_t>();
  uint8_t b = getParameter<uint8_t>();
  IPRINTF("ledSetRGB: id = %hu, led_id = %hu, r = %hu, g = %hu, b = %hu\n",
         id_, led_id, r, g, b);
  Adafruit_NeoPixel* led;
  switch(id_) {
    case 1:
      led = &onBoardLED;
      break;
    case 2:
      led = &moonbot_eyes;
      break;
    default:
      EPRINTF("Not available id:%d\n", id_);
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  if (led_id == 0) {
    led_id = led->numPixels();
    for (uint8_t i = 0; i < led_id; ++i) {
      led->setPixelColor(i, r, g, b);
    }
  } else {
    led->setPixelColor(led_id-1, r, g, b);
  }
  enableAllServoPower(false);
  led->show();
  while (!led->canShow());
  delay(4);           // to fixes bug: servo will shake if write quickly
  enableAllServoPower(true);

  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::ledSetColor(void) {
  resetParameterIndex();
  uint8_t led_id = getParameter<uint8_t>();
  uint8_t r = getParameter<uint8_t>();
  uint8_t g = getParameter<uint8_t>();
  uint8_t b = getParameter<uint8_t>();
  Adafruit_NeoPixel* led;
  switch(id_) {
    case 1:
      led = &onBoardLED;
      break;
    case 2:
      led = &moonbot_eyes;
      break;
    default:
      EPRINTF("Not available id:%d\n", id_);
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  if (led_id) {
    led->setPixelColor(led_id-1, r, g, b);
  } else {
    led->fill(Adafruit_NeoPixel::Color(r, g, b), 0, 0);
  }

  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::ledShow(void) {
  Adafruit_NeoPixel* led;
  switch(id_) {
    case 1:
      led = &onBoardLED;
      break;
    case 2:
      led = &moonbot_eyes;
      break;
    default:
      EPRINTF("Not available id:%d\n", id_);
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  enableAllServoPower(false);
  led->show();
  while(!led->canShow());
  delay(4);               // to fixes bug: servo will shake if write quickly
  enableAllServoPower(true);

  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::ledClear(void) {
  Adafruit_NeoPixel* led;
  switch(id_) {
    case 1:
      led = &onBoardLED;
      break;
    case 2:
      led = &moonbot_eyes;
      break;
    default:
      EPRINTF("Not available id:%d\n", id_);
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  led->clear();

  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::ledMatrix(void) {
  resetParameterIndex();
  Adafruit_NeoPixel* led;
  switch(id_) {
    case 1:
      led = &onBoardLED;
      break;
    case 2:
      led = &moonbot_eyes;
      break;
    default:
      EPRINTF("Not available id:%d\n", id_);
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  uint8_t led_n = getParameter<uint8_t>();
  for (uint8_t i = 0; i < led_n; ++i) {
    uint8_t led_id = getParameter<uint8_t>();
    uint8_t color = getParameter<uint8_t>();
    uint8_t brightness = getParameter<uint8_t>();
    size_t color_map_len = sizeof(color_map_)/3;
    if (color > color_map_len) {
      color = random(color_map_len);
    }
    led->setPixelColor(led_id,
                       color_map_[color][0]*brightness/255,
                       color_map_[color][1]*brightness/255,
                       color_map_[color][2]*brightness/255);
  }
  enableAllServoPower(false);
  led->show();
  while(!led->canShow());
  delay(4);             // to fixes bug: servo will shake if write quickly
  enableAllServoPower(true);

  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::ledSimpleWrite(void) {
  resetParameterIndex();
  uint8_t led_id = getParameter<uint8_t>();
  uint8_t color = getParameter<uint8_t>();
  uint8_t brightness = getParameter<uint8_t>();

  Adafruit_NeoPixel* led;
  switch(id_) {
    case 1:
      led = &onBoardLED;
      break;
    case 2:
      led = &moonbot_eyes;
      break;
    default:
      EPRINTF("Not available id:%d\n", id_);
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  size_t color_map_len = sizeof(color_map_)/3;
  if (color > color_map_len) {
    color = random(color_map_len);
  }
  if (led_id == 0) {
    led->fill(Adafruit_NeoPixel::Color(color_map_[color][0]*brightness/255,
                                       color_map_[color][1]*brightness/255,
                                       color_map_[color][2]*brightness/255),
              0, 0);
  } else {
    led->fill(Adafruit_NeoPixel::Color(color_map_[color][0]*brightness/255,
                                       color_map_[color][1]*brightness/255,
                                       color_map_[color][2]*brightness/255),
              (led_id-1)*led->numPixels()/2,
              led->numPixels()/2);
  }
  enableAllServoPower(false);
  led->show();
  while(!led->canShow());
  delay(4);       // to fixes bug: servo will shake if write quickly
  enableAllServoPower(true);

  // set color for eyes emotion
  if (id_ == 2) {
    switch (led_id) {
      case 0:
        // left eye
        eyes_color_[0][0] = color_map_[color][0]*brightness/255;
        eyes_color_[0][1] = color_map_[color][1]*brightness/255;
        eyes_color_[0][2] = color_map_[color][2]*brightness/255;
        // right eye
        eyes_color_[1][0] = color_map_[color][0]*brightness/255;
        eyes_color_[1][1] = color_map_[color][1]*brightness/255;
        eyes_color_[1][2] = color_map_[color][2]*brightness/255;
        break;
      case 1:
        // left eye
        eyes_color_[0][0] = color_map_[color][0]*brightness/255;
        eyes_color_[0][1] = color_map_[color][1]*brightness/255;
        eyes_color_[0][2] = color_map_[color][2]*brightness/255;
        break;
      case 2:
        // right eye
        eyes_color_[1][0] = color_map_[color][0]*brightness/255;
        eyes_color_[1][1] = color_map_[color][1]*brightness/255;
        eyes_color_[1][2] = color_map_[color][2]*brightness/255;
        break;
      default:
        break;
    }
  }
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::ledAction(uint8_t num) {
  enableAllServoPower(false);
  if ((eyes_color_[0][0] == 0)
      & (eyes_color_[0][1] == 0)
      & (eyes_color_[0][2] == 0)
      & (eyes_color_[1][0] == 0)
      & (eyes_color_[1][1] == 0)
      & (eyes_color_[1][2] == 0)) {
    eyes_color_[0][1] = 0x20;
    eyes_color_[0][2] = 0x20;
    eyes_color_[1][1] = 0x20;
    eyes_color_[1][2] = 0x20;
  }
  switch (num) {
    case 0x00: {
      // controller alarm
      for (int i = 0; i < 5; ++i) {
        onBoardLED.setPixelColor(0, 0x20, 0, 0);
        onBoardLED.setPixelColor(1, 0, 0, 0x20);
        onBoardLED.show();
        delay(200);
        onBoardLED.setPixelColor(1, 0x20, 0, 0);
        onBoardLED.setPixelColor(0, 0, 0, 0x20);
        onBoardLED.show();
        delay(200);
      }
      onBoardLED.clear();
      onBoardLED.show();
    }
      break;
    case 0x01: {
      // controller flash
      for (int i = 0; i < 5; ++i) {
        onBoardLED.setPixelColor(0, 0x20, 0x20, 0x20);
        onBoardLED.setPixelColor(1, 0x20, 0x20, 0x20);
        onBoardLED.show();
        delay(200);
        onBoardLED.clear();
        onBoardLED.show();
        delay(200);
      }
    }
      break;
    case 0x02: {
      // sensor on off
      resetParameterIndex();
      uint8_t click_type = getParameter<uint8_t>();
      if (click_type == 0x01) {
        Mu.LedSetColor(kLed1, kLedWhite, kLedWhite, 4);
        Mu.LedSetColor(kLed2, kLedWhite, kLedWhite, 4);
        Mu.LedSetMode(kLed1, 1, 1);
        Mu.LedSetMode(kLed2, 1, 1);
      } else if (click_type == 0x02) {
        Mu.LedSetColor(kLed1, kLedBlue, kLedRed, 1);
        Mu.LedSetColor(kLed2, kLedBlue, kLedRed, 1);
        Mu.LedSetMode(kLed1, 0, 0);
        Mu.LedSetMode(kLed2, 0, 0);
      }
    }
      break;
    case 0x03:      // eyes happy
    {
      moonbot_eyes.setPixelColor(0, 0x003030);
      moonbot_eyes.setPixelColor(1, 0x003030);
      moonbot_eyes.setPixelColor(2, 0x000000);
      moonbot_eyes.setPixelColor(3, 0x000000);
      moonbot_eyes.setPixelColor(4, 0x000000);
      moonbot_eyes.setPixelColor(5, 0x003030);
      moonbot_eyes.setPixelColor(6, 0x003030);
      moonbot_eyes.setPixelColor(7, 0x003030);
      moonbot_eyes.setPixelColor(8, 0x000000);
      moonbot_eyes.setPixelColor(9, 0x000000);
      moonbot_eyes.setPixelColor(10, 0x000000);
      moonbot_eyes.setPixelColor(11, 0x003030);
      moonbot_eyes.show();
      break;
    }
    case 0x04:      // eyes sad
    {
      moonbot_eyes.clear();
      for (int j = 1; j <= 1; j = j + (1)) {
        for (int i = 50; i <= 150; i = i + (1)) {
          moonbot_eyes.setBrightness(i);
          moonbot_eyes.setPixelColor(2, 0x00ffff);
          moonbot_eyes.setPixelColor(10, 0x00ffff);
          moonbot_eyes.show();
          delay(4);
        }
        for (int i = 150; i >= 50; i = i + (-1)) {
          moonbot_eyes.setBrightness(i);
          moonbot_eyes.setPixelColor(2, 0x00ffff);
          moonbot_eyes.setPixelColor(10, 0x00ffff);
          moonbot_eyes.show();
          delay(4);
        }
      }
      for (int i = 50; i <= 150; i = i + (1)) {
        moonbot_eyes.setBrightness(i);
        moonbot_eyes.setPixelColor(2, 0x00ffff);
        moonbot_eyes.setPixelColor(10, 0x00ffff);
        moonbot_eyes.show();
        delay(4);
      }
      moonbot_eyes.setBrightness(255);
      break;
    }
    case 0x05:      // eyes angry
    {
      for (int j = 1; j <= 1; j = j + (1)) {
        for (int i = 0; i <= 200; i = i + (4)) {
          moonbot_eyes.setBrightness(i);
          moonbot_eyes.fill(0xff0000, 0, 0);
          moonbot_eyes.show();
          delay(2);
        }
        for (int i = 200; i >= 0; i = i + (-4)) {
          moonbot_eyes.setBrightness(i);
          moonbot_eyes.fill(0xff0000, 0, 0);
          moonbot_eyes.show();
          delay(1);
        }
      }
      for (int i = 0; i <= 200; i = i + (4)) {
        moonbot_eyes.setBrightness(i);
        moonbot_eyes.fill(0xff0000, 0, 0);
        moonbot_eyes.show();
        delay(2);
      }
      memset(eyes_color_, 0, sizeof(eyes_color_));
      eyes_color_[0][0] = moonbot_eyes.getPixels()[0];
      eyes_color_[1][0] = moonbot_eyes.getPixels()[0];
      moonbot_eyes.setBrightness(255);
      break;
    }
    case 0x06: {
      // eyes blink
      unsigned int action_delay = 30;
      moonbot_eyes.setPixelColor(0, 0);
      moonbot_eyes.setPixelColor(6, 0);
      moonbot_eyes.show();
      delay(action_delay);
      moonbot_eyes.setPixelColor(1, 0);
      moonbot_eyes.setPixelColor(5, 0);
      moonbot_eyes.setPixelColor(7, 0);
      moonbot_eyes.setPixelColor(11, 0);
      moonbot_eyes.show();
      delay(action_delay);
      moonbot_eyes.setPixelColor(2, 0);
      moonbot_eyes.setPixelColor(4, 0);
      moonbot_eyes.setPixelColor(8, 0);
      moonbot_eyes.setPixelColor(10, 0);
      moonbot_eyes.show();
      delay(action_delay);
      moonbot_eyes.setPixelColor(3, 0);
      moonbot_eyes.setPixelColor(9, 0);
      moonbot_eyes.show();
      delay(action_delay);
      moonbot_eyes.setPixelColor(3, eyes_color_[0][0], eyes_color_[0][1], eyes_color_[0][2]);
      moonbot_eyes.setPixelColor(9, eyes_color_[1][0],eyes_color_[1][1],eyes_color_[1][2]);
      moonbot_eyes.show();
      delay(action_delay);
      moonbot_eyes.setPixelColor(2, eyes_color_[0][0],eyes_color_[0][1],eyes_color_[0][2]);
      moonbot_eyes.setPixelColor(4, eyes_color_[0][0],eyes_color_[0][1],eyes_color_[0][2]);
      moonbot_eyes.setPixelColor(8, eyes_color_[1][0],eyes_color_[1][1],eyes_color_[1][2]);
      moonbot_eyes.setPixelColor(10, eyes_color_[1][0],eyes_color_[1][1],eyes_color_[1][2]);
      moonbot_eyes.show();
      delay(action_delay);
      moonbot_eyes.setPixelColor(1, eyes_color_[0][0],eyes_color_[0][1],eyes_color_[0][2]);
      moonbot_eyes.setPixelColor(5, eyes_color_[0][0],eyes_color_[0][1],eyes_color_[0][2]);
      moonbot_eyes.setPixelColor(7, eyes_color_[1][0],eyes_color_[1][1],eyes_color_[1][2]);
      moonbot_eyes.setPixelColor(11, eyes_color_[1][0],eyes_color_[1][1],eyes_color_[1][2]);
      moonbot_eyes.show();
      delay(action_delay);
      moonbot_eyes.setPixelColor(0, eyes_color_[0][0],eyes_color_[0][1],eyes_color_[0][2]);
      moonbot_eyes.setPixelColor(6, eyes_color_[1][0],eyes_color_[1][1],eyes_color_[1][2]);
      moonbot_eyes.show();
    }
      break;
    case 0x07: {
      // eyes circle
      setAllServoPower(false);
      colorWipe(moonbot_eyes, (0), 40);
      colorWipe(moonbot_eyes, Adafruit_NeoPixel::Color(eyes_color_[0][0], eyes_color_[0][1], eyes_color_[0][2]), 40);
      setAllServoPower(true);
    }
      break;
    case 0x08: {    // eyes flash
      setAllServoPower(false);
      theaterChase(moonbot_eyes, ((((uint32_t)eyes_color_[0][0])<<16)|(eyes_color_[0][1]<<8)|eyes_color_[0][2]), 40);
      colorFade(moonbot_eyes, eyes_color_[0][0], eyes_color_[0][1], eyes_color_[0][2],3);
      setAllServoPower(true);
      break;
    }
    case 0x09: {    // eyes rainbow
      setAllServoPower(false);
      rainbowCycle(moonbot_eyes, 3);
      colorFade(moonbot_eyes, eyes_color_[0][0], eyes_color_[0][1], eyes_color_[0][2],3);
      setAllServoPower(true);
      break;
    }
    case 0x0A:      // eyes close
    {
      unsigned int action_delay = 30;
      moonbot_eyes.setPixelColor(0, 0);
      moonbot_eyes.setPixelColor(6, 0);
      moonbot_eyes.show();
      delay(action_delay);
      moonbot_eyes.setPixelColor(1, 0);
      moonbot_eyes.setPixelColor(5, 0);
      moonbot_eyes.setPixelColor(7, 0);
      moonbot_eyes.setPixelColor(11, 0);
      moonbot_eyes.show();
      delay(action_delay);
      moonbot_eyes.setPixelColor(2, 0);
      moonbot_eyes.setPixelColor(4, 0);
      moonbot_eyes.setPixelColor(8, 0);
      moonbot_eyes.setPixelColor(10, 0);
      moonbot_eyes.show();
      delay(action_delay);
      moonbot_eyes.setPixelColor(3, 0);
      moonbot_eyes.setPixelColor(9, 0);
      moonbot_eyes.show();
      break;
    }
    default:
      enableAllServoPower(true);
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  enableAllServoPower(true);
  clearBuffer();
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::ledEyesEmotion(void) {
  resetParameterIndex();
  uint8_t emotion = getParameter<uint8_t>();
  return ledAction(emotion+2);
}

// Servo
uint8_t MoonBotRemoteController::servoCalibrate(void) {
  resetParameterIndex();
  int8_t angle_offset = getParameter<int8_t>();
  m_servo[id_-1].correction(angle_offset);

  resetParameterIndex();
  return MU_OK;
}
// test command:  FF09607202026442ED      // servo2 write 100
//                FF096072020220FEED      // servo2 write 32
//                FF09607202025A38ED      // servo2 write 90
uint8_t MoonBotRemoteController::servoWrite(void) {
  resetParameterIndex();
  uint8_t mode = getParameter<uint8_t>();
  uint8_t angle = getParameter<uint8_t>();
  uint8_t angle_now = m_servo[id_-1].read();
  switch (mode) {
    case 1: // Set
      break;
    case 2: // Inc
      angle = angle_now+angle;
      angle = angle>180 ? 180:angle;
      break;
    case 3: // Dec
      if (angle_now>angle) {
        angle = angle_now-angle;
      } else {
        angle = 0;
      }
      break;
    default:
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  m_servo[id_-1].write(angle);

  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::servoRead(void) {
  uint8_t angle = m_servo[id_-1].read();

  resetParameterIndex();
  setParameter<uint8_t>(angle);
  return MU_OK;
}
uint8_t MoonBotRemoteController::servoSetTargetAngle(void) {
  resetParameterIndex();
  uint8_t mode = getParameter<uint8_t>();
  uint8_t angle = getParameter<uint8_t>();
  uint8_t speed = getParameter<uint8_t>();
  uint8_t angle_now = m_servo[id_-1].read();
  switch (mode) {
    case 1: // Set
      break;
    case 2: // Inc
      angle = angle_now+angle;
      angle = angle>180 ? 180:angle;
      break;
    case 3: // Dec
      if (angle_now>angle) {
        angle = angle_now-angle;
      } else {
        angle = 0;
      }
      break;
    default:
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  m_servo[id_-1].setTargetAngle(angle ,speed);

  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::servoMoveAllServo(void) {
  MoonBotServo::moveAllServoToTarget();

  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::servoWriteAll(void) {
  resetParameterIndex();
  uint8_t num = getParameter<uint8_t>();
  for (uint8_t i = 0; i < num; ++i) {
    uint8_t angle = getParameter<uint8_t>();
    if (angle <= 180) {
      m_servo[i].write(angle);
    }
  }
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::servoReadAll(void) {
  resetParameterIndex();
  setParameter<uint8_t>(kServoNum);
  for (uint8_t i = 0; i < kServoNum; ++i) {
    setParameter<uint8_t>((uint8_t)m_servo[i].read());
  }
  return MU_OK;
}
void MoonBotRemoteController::setAllServoPower(bool state) {
  for (unsigned int i = 0; i < kServoNum; ++i) {
    m_servo[i].power(state);
  }
}

// Motor
uint8_t MoonBotRemoteController::motorWriteRpm(void) {
  resetParameterIndex();
  int8_t rpm = getParameter<int8_t>();
  switch (id_) {
    case 1:
      Motor1.writeRpm(rpm);
      break;
    case 2:
      Motor2.writeRpm(rpm);
      break;
    default:
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }

  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::motorReadRpm(void) {
  int8_t rpm = 0;
  switch (id_) {
    case 1:
      rpm = Motor1.readRpm();
      break;
    case 2:
      rpm = Motor1.readRpm();
      break;
    default:
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }

  resetParameterIndex();
  setParameter<int8_t>(rpm);
  return MU_OK;
}
uint8_t MoonBotRemoteController::motorWrite(void) {
  resetParameterIndex();
  uint8_t mode = getParameter<uint8_t>();
  int vol = getParameter<uint8_t>();
  Motor* motor = nullptr;
  int vol_now = 0;

  switch (id_) {
    case 1:
      motor = &Motor1;
      break;
    case 2:
      motor = &Motor2;
      break;
    default:
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  vol_now = motor->read();
  switch (mode) {
    case 0:   // stop
      vol = 0;
      break;
    case 1:   // +
      vol = vol_now+vol;
      break;
    case 2:   // -
      vol = vol_now-vol;
      break;
    case 3:   // forward
      break;
    case 4:   // backward
      vol = -vol;
      break;
    default:
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  motor->write(vol);

  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::motorRead(void) {
  int16_t vol = 0;
  switch (id_) {
    case 1:
      vol = Motor1.read();
      break;
    case 2:
      vol = Motor1.read();
      break;
    default:
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }

  resetParameterIndex();
  setParameter<int16_t>(vol);
  return MU_OK;
}
// TankBase
uint8_t MoonBotRemoteController::wheelCalibrate(void) {
  resetParameterIndex();
  uint8_t percent = getParameter<uint8_t>();
  switch (id_) {
    case 1:
      TankBase.rpmCorrection(percent);
      break;
    default:
      EPRINTF("Not available id:%d\n", id_);
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::wheelWriteRpm(void) {
  resetParameterIndex();
  uint8_t left_mode = getParameter<uint8_t>();
  int left_rpm = getParameter<uint8_t>();
  uint8_t right_mode = getParameter<uint8_t>();
  int right_rpm = getParameter<uint8_t>();

  int left_rpm_now = TankBase.readRpm(kLeftMotor);
  switch (left_mode) {
    case 0:   // stop
      left_rpm = 0;
      break;
    case 1:   // +
      left_rpm = left_rpm_now+left_rpm;
      break;
    case 2:   // -
      left_rpm = left_rpm_now-left_rpm;
      break;
    case 3:   // forward
      break;
    case 4:   // backward
      left_rpm = -left_rpm;
      break;
    default:
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  int right_rpm_now = TankBase.readRpm(kLeftMotor);
  switch (right_mode) {
    case 0:   // stop
      right_rpm = 0;
      break;
    case 1:   // +
      right_rpm = right_rpm_now+right_rpm;
      break;
    case 2:   // -
      right_rpm = right_rpm_now-right_rpm;
      break;
    case 3:   // forward
      break;
    case 4:   // backward
      right_rpm = -right_rpm;
      break;
    default:
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  switch (id_) {
    case 1:
      TankBase.writeRpm(left_rpm, right_rpm);
      break;
    default:
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::wheelReadRpm(void) {
  int8_t left_rpm;
  int8_t right_rpm;
  switch (id_) {
    case 1:
      left_rpm = TankBase.readRpm(kLeftMotor);
      right_rpm = TankBase.readRpm(kRightMotor);
      break;
    default:
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }

  resetParameterIndex();
  setParameter<int8_t>(left_rpm);
  setParameter<int8_t>(right_rpm);
  return MU_OK;
}
uint8_t MoonBotRemoteController::wheelForward(void) {
  resetParameterIndex();
  uint16_t distance = getParameter<uint16_t>();
  uint8_t speed = getParameter<uint8_t>();
  switch (id_) {
    case 1:
      TankBase.forward(distance, speed);
      break;
    default:
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  while(TankBase.read(kLeftMotor)&&TankBase.read(kRightMotor));
  delay(100);
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::wheelBackward(void) {
  resetParameterIndex();
  uint16_t distance = getParameter<uint16_t>();
  uint8_t speed = getParameter<uint8_t>();
  switch (id_) {
    case 1:
      TankBase.backward(distance, speed);
      break;
    default:
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  while(TankBase.read(kLeftMotor)&&TankBase.read(kRightMotor));
  delay(100);
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::wheelTurnLeft(void) {
  resetParameterIndex();
  uint16_t angle = getParameter<uint16_t>();
  uint8_t speed = getParameter<uint8_t>();
  switch (id_) {
    case 1:
      TankBase.turnLeft(angle, speed);
      break;
    default:
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  while(TankBase.read(kLeftMotor) || TankBase.read(kRightMotor));
  delay(100);
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::wheelTurnRight(void) {
  resetParameterIndex();
  uint16_t angle = getParameter<uint16_t>();
  uint8_t speed = getParameter<uint8_t>();
  switch (id_) {
    case 1:
      TankBase.turnRight(angle, speed);
      break;
    default:
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  while(TankBase.read(kLeftMotor) || TankBase.read(kRightMotor));
  delay(100);
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::wheelStop(void) {
  switch (id_) {
    case 1:
      TankBase.stop();
      break;
    default:
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::wheelSetDistanceStep(void) {
  resetParameterIndex();
  uint8_t percent = getParameter<uint8_t>();
  switch (id_) {
    case 1:
      TankBase.distanceCorrection(percent);
      break;
    default:
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }

  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::wheelSetAngleStep(void) {
  resetParameterIndex();
  uint8_t percent = getParameter<uint8_t>();
  switch (id_) {
    case 1:
      TankBase.wheelSpacingSet(percent);
      break;
    default:
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }

  resetParameterIndex();
  return MU_OK;
}
// Buzzer
uint8_t MoonBotRemoteController::buzzerWrite(void) {
  resetParameterIndex();
  uint8_t tone_value = getParameter<uint8_t>();
  uint8_t beat = getParameter<uint8_t>();
  switch (id_) {
    case 1:
      if (tone_value) {
        tone(MOONBOT_PIN_BUZZER_SIG, tone_map_[tone_value]);
      }
      delay((uint32_t)60000*beat_map_[beat]/BEAT_FRACTION_WHOLE/tempo_);
      noTone(MOONBOT_PIN_BUZZER_SIG);
      break;
    default:
      EPRINTF("Not available id:%d\n", id_);
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::buzzerPlayMusic(uint32_t music_id) {
  switch(music_id) {
    case 1:   // button_music_1
      tone(MOONBOT_PIN_BUZZER_SIG, 500);
      delay(200);
      noTone(MOONBOT_PIN_BUZZER_SIG);
      break;
    case 2:   // button_music_2
      tone(MOONBOT_PIN_BUZZER_SIG, 1000);
      delay(200);
      noTone(MOONBOT_PIN_BUZZER_SIG);
      break;
    case 3:   // button_music_3
      tone(MOONBOT_PIN_BUZZER_SIG, 1500);
      delay(200);
      noTone(MOONBOT_PIN_BUZZER_SIG);
      break;
    case 4:   // button_music_4
      tone(MOONBOT_PIN_BUZZER_SIG, 2000);
      delay(200);
      noTone(MOONBOT_PIN_BUZZER_SIG);
      break;
    case 5:   // alarm_1
      tone(MOONBOT_PIN_BUZZER_SIG, 750);
      delay(1000);
      noTone(MOONBOT_PIN_BUZZER_SIG);
      break;
    case 6:   // alarm_2
      for (int j = 1; j <= 2; j = j + (1)) {
        for (int i = 500; i <= 1000; i = i + (5)) {
          tone(MOONBOT_PIN_BUZZER_SIG, i, 0);
          delay(1);
        }
        for (int i = 1000; i >= 500; i = i + (-5)) {
          tone(MOONBOT_PIN_BUZZER_SIG, i, 0);
          delay(1);
        }
      }
      noTone(MOONBOT_PIN_BUZZER_SIG);
      break;
    case 7:   // sound_1
      for (int i = 1; i <= 2; i = i + (1)) {
        tone(MOONBOT_PIN_BUZZER_SIG, 500, 0);
        delay(100);
        tone(MOONBOT_PIN_BUZZER_SIG, 1000, 0);
        delay(100);
      }
      noTone(MOONBOT_PIN_BUZZER_SIG);
      break;
    case 8:   // sound_2
      tone(MOONBOT_PIN_BUZZER_SIG, NOTE_C5);
      delay(60000/tempo_*BEAT_FRACTION_QUARTER/BEAT_FRACTION_WHOLE);
      noTone(MOONBOT_PIN_BUZZER_SIG);
      tone(MOONBOT_PIN_BUZZER_SIG, NOTE_F5);
      delay(60000/tempo_*BEAT_FRACTION_QUARTER/BEAT_FRACTION_WHOLE);
      noTone(MOONBOT_PIN_BUZZER_SIG);
      tone(MOONBOT_PIN_BUZZER_SIG, NOTE_C5);
      delay(60000/tempo_*BEAT_FRACTION_QUARTER/BEAT_FRACTION_WHOLE);
      noTone(MOONBOT_PIN_BUZZER_SIG);
      tone(MOONBOT_PIN_BUZZER_SIG, NOTE_A5);
      delay(60000/tempo_*BEAT_FRACTION_HALF/BEAT_FRACTION_WHOLE);
      noTone(MOONBOT_PIN_BUZZER_SIG);
      break;
    case 9:   // sound_3
      for (int i = 750; i >= 250; i = i + (-6)) {
        tone(MOONBOT_PIN_BUZZER_SIG, i, 0);
        delay(2);
      }
      noTone(MOONBOT_PIN_BUZZER_SIG);
      break;
    case 10:   // sound_4
      for (int i = 1000; i <= 1500; i = i + (10)) {
        tone(MOONBOT_PIN_BUZZER_SIG, i, 0);
        delay(4);
      }
      noTone(MOONBOT_PIN_BUZZER_SIG);
      break;
    case 11:       // Ambulances
      for (int i = 0; i < 2; ++i) {
        tone(MOONBOT_PIN_BUZZER_SIG, 700);
        delay(400);
        tone(MOONBOT_PIN_BUZZER_SIG, 950);
        delay(600);
        noTone(MOONBOT_PIN_BUZZER_SIG);
      }
      break;
    case 12:       // Whistle
      for (int j = 0; j < 3; ++j) {
        for (int i = 0; i < 25; ++i) {
          tone(MOONBOT_PIN_BUZZER_SIG, 700+i*32);
          delay(10);
        }
        for (int i = 0; i < 10; ++i) {
          tone(MOONBOT_PIN_BUZZER_SIG, 1500-i*80);
          delay(10);
        }
        noTone(MOONBOT_PIN_BUZZER_SIG);
      }
      break;
    case 13: {     // Fire Truck
      for (unsigned int i = 0; i < 160; ++i) {
        tone(MOONBOT_PIN_BUZZER_SIG, 700+i*5);
        delay(10);
      }
      for (unsigned int i = 0; i < 400; ++i) {
        tone(MOONBOT_PIN_BUZZER_SIG, 1500-i*2);
        delay(10);
      }
      Serial.println();
      noTone(MOONBOT_PIN_BUZZER_SIG);
    }
      break;
    case 14:
      buzzerPlayerPGM(notationJingleBells,
                      sizeJingleBells,
                      60000/tempo_);
      break;
    case 15:
      buzzerPlayerPGM(notationHappyBirthday,
                      sizeHappyBirthday,
                      60000/tempo_);
      break;
    default:
      EPRINTF("Not available music_id:%lu\n", music_id);
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  clearBuffer();
  return MU_OK;
}
uint8_t MoonBotRemoteController::buzzerPlay(void) {
  resetParameterIndex();
  uint8_t music_id = getParameter<uint16_t>();
  resetParameterIndex();
  return buzzerPlayMusic(music_id);
}
uint8_t MoonBotRemoteController::buzzerSetTempo(void) {
  resetParameterIndex();
  tempo_ = getParameter<uint16_t>();
  resetParameterIndex();
  return MU_OK;
}
// Speaker
uint8_t MoonBotRemoteController::speakerPlay(void) {
  resetParameterIndex();
  char name[4];
  for (size_t i = 0; i < sizeof(name); ++i) {
    name[i] = getParameter<char>();
  }
  switch (id_) {
    case 1:
      speaker.play(name);
      break;
    default:
      EPRINTF("Not available id:%d\n", id_);
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::speakerSet(void) {
  resetParameterIndex();
  uint8_t mode = getParameter<uint8_t>();
  switch (id_) {
    case 1:
      switch (mode) {
        case 0:     // stop
          speaker.stop();
          break;
        case 1:     // play/pause
          speaker.pause();
          break;
        case 2:     // next
          speaker.playNext();
          break;
        case 3:     // previous
          speaker.playPrevious();
          break;
        default:
          return MU_SLAVE_UNKNOW_REG_VALUE;
      }
      speaker.stop();
      break;
    default:
      EPRINTF("Not available id:%d\n", id_);
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::speakerSetVolume(void) {
  resetParameterIndex();
  uint8_t volume = getParameter<uint8_t>();
  switch(id_) {
    case 1:
      speaker.setVolume(volume);
      break;
    default:
      EPRINTF("Not available id:%d\n", id_);
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::speakerSetPlayMode(void) {
  resetParameterIndex();
  uint8_t mode = getParameter<uint8_t>();
  switch(id_) {
    case 1:
      speaker.setPlayMode(mode);
      break;
    default:
      EPRINTF("Not available id:%d\n", id_);
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  resetParameterIndex();
  return MU_OK;
}
// Port
uint8_t MoonBotRemoteController::portDigitalWrite(void) {
  resetParameterIndex();
  uint8_t pin1 = getParameter<uint8_t>();
  uint8_t pin2 = getParameter<uint8_t>();
  if (pin1<2) {
  digitalWrite(moonbotPortToPin(moonbot_port_t(id_-1), kPortPin1), pin1);
  }
  if (pin2<2) {
  digitalWrite(moonbotPortToPin(moonbot_port_t(id_-1), kPortPin2), pin2);
  }
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::portDigitalRead(void) {
  uint8_t pin1 = digitalRead(moonbotPortToPin(moonbot_port_t(id_-1), kPortPin1));
  uint8_t pin2 = digitalRead(moonbotPortToPin(moonbot_port_t(id_-1), kPortPin2));
  resetParameterIndex();
  setParameter<uint8_t>(pin1);
  setParameter<uint8_t>(pin2);
  return MU_OK;
}
uint8_t MoonBotRemoteController::portAnalogWrite(void) {
  resetParameterIndex();
  uint8_t pin = getParameter<uint8_t>();
  uint8_t value = getParameter<uint16_t>() & 0xFF;
  analogWrite(moonbotPortToPin(moonbot_port_t(id_-1), port_pin_t(pin)), value);
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::portAnalogRead(void) {
  resetParameterIndex();
  uint8_t pin = getParameter<uint8_t>();
  uint16_t value = (uint16_t)analogRead(moonbotPortToPin(moonbot_port_t(id_-1), port_pin_t(pin)));
  resetParameterIndex();
  setParameter<uint16_t>(value);
  return MU_OK;
}
// Button
uint8_t MoonBotRemoteController::buttonRead(void) {
  resetParameterIndex();
  uint8_t state = 0;
  switch (id_) {
    case 1:
      state = !digitalRead(MOONBOT_PIN_BUTTON_A);
      break;
    case 2:
      state = !digitalRead(MOONBOT_PIN_BUTTON_B);
      break;
    default:
      state = (!digitalRead(MOONBOT_PIN_BUTTON_A))
          && (!digitalRead(MOONBOT_PIN_BUTTON_B));
      break;
  }
  resetParameterIndex();
  setParameter<uint8_t>(state);
  return MU_OK;
}
// IMU
uint8_t MoonBotRemoteController::imuCalibrate(void) {
  switch (id_) {
    case 1:
      IMU.calibrateMag();
      break;
    default:
      EPRINTF("Not available id:%d\n", id_);
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  resetParameterIndex();
  return MU_OK;
}
uint8_t MoonBotRemoteController::imuReadCompass(void) {
  resetParameterIndex();
  uint16_t angle = 0;
  switch (id_) {
    case 1: {
      angle = IMU.getMagAngle();
      break;
    }
    default:
      EPRINTF("Not available id:%d\n", id_);
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  IPRINTF("Mag Angle = %u\n", angle);
  resetParameterIndex();
  setParameter<uint16_t>(angle);
  return MU_OK;
}
uint8_t MoonBotRemoteController::imuReadAcceleration(void) {
  resetParameterIndex();
  uint8_t type = getParameter<uint8_t>();
  int32_t axes[3];
  int16_t retval = 0;
  switch (id_) {
    case 1:
      if (type && type < 4) {
        IMU.Acc.GetAxes(axes);
        retval = axes[type - 1];
      } else {
        retval = IMU.getAcceleration()*1024;
      }
      break;
    default:
      EPRINTF("Not available id:%d\n", id_);
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  resetParameterIndex();
  setParameter<int16_t>(retval);
  return MU_OK;
}
uint8_t MoonBotRemoteController::imuTemperature(void) {
  resetParameterIndex();
  int16_t temperature = 0;
  switch (id_) {
    case 1:
      temperature = IMU.temperatureC();
      break;
    default:
      EPRINTF("Not available id:%d\n", id_);
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  resetParameterIndex();
  setParameter<int16_t>(temperature);
  return MU_OK;
}
uint8_t MoonBotRemoteController::imuMotion(void) {
  resetParameterIndex();
  uint8_t motion = getParameter<uint8_t>();
  uint8_t retval = 0;
  switch (motion) {
    case 0:
      retval = IMU.on(kIMUShake);
      break;
    case 1:
      retval = IMU.on(kIMUFreeFall);
      break;
    case 2:     // x up
    {
      int32_t axes[3];
      IMU.Acc.GetAxes(axes);
      retval = axes[0]>700;
      break;
    }
    case 3:     // x down
    {
      int32_t axes[3];
      IMU.Acc.GetAxes(axes);
      retval = axes[0]<-700;
      break;
    }
    case 4:     // y up
    {
      int32_t axes[3];
      IMU.Acc.GetAxes(axes);
      retval = axes[1]>700;
      break;
    }
    case 5:     // y down
    {
      int32_t axes[3];
      IMU.Acc.GetAxes(axes);
      retval = axes[1]<-700;
      break;
    }
    case 6:     // z up
    {
      int32_t axes[3];
      IMU.Acc.GetAxes(axes);
      retval = axes[2]>700;
      break;
    }
    case 7:     // z down
    {
      int32_t axes[3];
      IMU.Acc.GetAxes(axes);
      retval = axes[2]<-700;
      break;
    }
    case 8:     // 3g
    {
      retval = IMU.getAcceleration()>3;
      break;
    }
    case 9:     // 6g
    {
      retval = IMU.getAcceleration()>6;
      break;
    }
    case 10:     // 8g
    {
      retval = IMU.getAcceleration()>8;
      break;
    }
    default:
      EPRINTF("Not available motion type: %d\n", motion);
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }

  resetParameterIndex();
  setParameter<uint8_t>(retval);
  return MU_OK;
}
uint8_t MoonBotRemoteController::imuRatationRead(void) {
  resetParameterIndex();
  uint8_t type = getParameter<uint8_t>();
  int16_t retval = 0;
  switch (type) {
    case 0:
    case 1:
      retval = IMU.getAccAngle(lsm303_acc_angle_t(type));
      break;
    default:
      EPRINTF("Not available type: %d\n", type);
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }

  resetParameterIndex();
  setParameter<int16_t>(retval);
  return MU_OK;
}
// App Button
uint8_t MoonBotRemoteController::appButtonClick(void) {
//  resetParameterIndex();
//  uint8_t click_type = getParameter<uint8_t>();
//
//  switch (id_) {
//    default:
//      break;
//  }
  return MU_SLAVE_UNKNOW_REG_VALUE;
}
// test command:    FF0C6080030100FF00604EED      // angle: 255, speed: 96
uint8_t MoonBotRemoteController::appButtonDrag(void) {
  resetParameterIndex();
  uint16_t angle = getParameter<uint16_t>();
  uint16_t speed = getParameter<uint16_t>();
  IPRINTF("appButtonDrag: id = 0x%02x, angle = %u, speed = %u\n",
         id_, angle, speed);
  switch(id_) {
    case 1: {
      int left_speed, right_speed;
      speed = speed>100 ? 100:speed;
      angle = angle>360 ? 360:angle;
      if (angle < 90) {
        left_speed = 255*(int)speed/100;
        right_speed = map(angle, 0, 90, 255, -255)*speed/100;
      } else if (angle < 180) {
        right_speed = -255*(int)speed/100;
        left_speed = map(angle, 90, 180, 255, -255)*speed/100;
      } else if (angle < 270) {
        left_speed = -255*(int)speed/100;
        right_speed = map(angle, 180, 270, -255, 255)*speed/100;
      } else {
        right_speed = 255*(int)speed/100;
        left_speed = map(angle, 270, 360, -255, 255)*speed/100;
      }
      TankBase.write(left_speed, right_speed);
      enableTankBaseStopEvent();
      IPRINTF("appButtonDrag: left_speed = %d, right_speed = %d\n",
             left_speed, right_speed);
      break;
    }
    default:
      EPRINTF("Not available id:%d\n", id_);
      resetParameterIndex();
      return MU_SLAVE_UNKNOW_REG_VALUE;
  }
  resetParameterIndex();
  return MU_OK;
}


