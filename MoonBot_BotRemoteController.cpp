/*
 * MoonBot_BotRemoteController.cpp
 *
 *  Created on: 2019年5月20日
 *      Author: ysq
 */

#include "MoonBot_BotRemoteController.h"

MoonBotBotRemoteController::MoonBotBotRemoteController(MuVsUart* uart,
                                                       uint32_t address,
                                                       bool response_enable)
    : MoonBotRemoteController(uart, address, response_enable),
      moonbot_(Mu, m_servo[kServo3], m_servo[kServo4], m_servo[kServo1]) {
  form_ = MOONBOT_REMOT_FORM_MECH;
  Mu.begin(uart);
  moonbot_eyes.setPin(moonbotPortToPin(kPort1, kPortPin1));
  moonbot_eyes.updateLength(MOONBOT_EXTERNAL_LED_NUM);
  moonbot_eyes.updateType(NEO_GRB + NEO_KHZ800);
  moonbot_eyes.begin();
  eyes_color_[0][0] = 0;
  eyes_color_[0][1] = eyes_brightness_;
  eyes_color_[0][2] = eyes_brightness_;
  eyes_color_[1][0] = 0;
  eyes_color_[1][1] = eyes_brightness_;
  eyes_color_[1][2] = eyes_brightness_;
  setEyesColor(eyes_color_[0]);
  speaker.begin(Serial2);
  for (int i = 0; i < kServoNum; ++i) {
    m_servo[i].power(true);
  }
  m_servo[kServo1].reverse(true);    // right arm
  m_servo[kServo3].reverse(true);    // head
  TankBase.begin();
  // buzzer begin
  pinMode(MOONBOT_PIN_BUZZER_SIG, OUTPUT);
  pinMode(MOONBOT_PIN_BUZZER_SHDW, OUTPUT);
  digitalWrite(MOONBOT_PIN_BUZZER_SHDW, LOW);
}

MoonBotBotRemoteController::~MoonBotBotRemoteController(void) {}

void MoonBotBotRemoteController::CommandMatcher(void) {
  // Not available command: break; otherwise: return;
  MACTCH_BEGIN
    //==============AppButton==============
    MACTCH_CMD(MR_CMD_APP_BUTTON)
      MACTCH_SUB_CMD(MR_CMD_APP_BUTTON_CLICK, botAppButtonClick);
    MATCH_END
  MATCH_END
  MoonBotRemoteController::CommandMatcher();
}

void MoonBotBotRemoteController::RunEvent(void) {
  switch (remote_event_) {
    default:
      MoonBotRemoteController::RunEvent();
      break;
  }
}

uint8_t MoonBotBotRemoteController::botAppButtonClick(void) {
  resetParameterIndex();
  uint8_t click_type = getParameter<uint8_t>();
  resetParameterIndex();
  switch (id_) {
//    case 0x81:
//    case 0x82:
    // Dance
    case 0x90:
    case 0x91:
    case 0x92:
    case 0x93:
    case 0x94:
    case 0x95:
    case 0x96:
    case 0x97:
    case 0x98:
    case 0x99:
    case 0x9A:
    case 0x9B:
    case 0x9C:
    case 0x9D:
    case 0x9E:
    case 0x9F:
      if (click_type == 0x02) {
        return MU_OK;
      }
      return dance(id_&0x0F);
    // Buzzer
    case 0xB0:
    case 0xB1:
    case 0xB2:
    case 0xB3:
    case 0xB4:
    case 0xB5:
    case 0xB6:
    case 0xB7:
    case 0xB8:
    case 0xB9:
    case 0xBA:
    case 0xBB:
    case 0xBC:
    case 0xBD:
    case 0xBE:
    case 0xBF: {
      if (click_type == 0x02) {
        return MU_OK;
      }
      uint8_t click_id2music_id[] = {7,8,9,11,12};
      return buzzerPlayMusic(click_id2music_id[id_&0x0F]);
    }
    // LED action
    case 0xC0:
    case 0xC1:
      if (click_type == 0x02) {
        LED.clear();
        LED.show();
        return MU_OK;
      }
      return ledAction(id_&0x0F);
    case 0xC2:      // MuVisionSensor On/Off
      return ledAction(id_&0x0F);
    case 0xC3:
    case 0xC4:
    case 0xC5:
    case 0xC6:
    case 0xC7:
    case 0xC8:
    case 0xC9:
    case 0xCA:
    case 0xCB:
    case 0xCC:
    case 0xCD:
    case 0xCE:
    case 0xCF:
      if (click_type == 0x02) {
        moonbot_eyes.clear();
        moonbot_eyes.show();
        return MU_OK;
      }
      return ledAction(id_&0x0F);
    default:
      break;
  }

  return MU_ERROR_REG_VALUE;
}

uint8_t MoonBotBotRemoteController::dance(uint8_t num) {
  switch (num) {
    case 0:
      speaker.play((char *)"0408");     // city piano
      for (int i = 0; i<3; ++i) {
        moonbot_.swing(kMoonBotLeftMotor, 80, 100);
      }
      break;
    case 1:
      for (int i = 0; i<3; ++i) {
        moonbot_.bodyShake(60, 500);
      }
      break;
    case 2: {
      switch (random(3)) {
        case 0:
          speaker.play((char *)"0201");     // hello
          break;
        case 1:
          speaker.play((char *)"0202");     // hi
          break;
        case 2:
          speaker.play((char *)"0203");     // how r u
          break;
        default:
          break;
      }
      moonbot_humannoid_arm_t arm_type = (moonbot_humannoid_arm_t)random(3);
      for (int i = 0; i<3; ++i) {
        moonbot_.armShake(arm_type, 15, 150);
      }
    }
      break;
    default:
      return MU_ERROR_REG_VALUE;
  }
  resetParameterIndex();
  return MU_OK;
}

void MoonBotBotRemoteController::setEyesColor(uint8_t eyes_color[3]) {
  memcpy(eyes_color_[0], eyes_color, 3);
  memcpy(eyes_color_[1], eyes_color, 3);
  for (unsigned int i = 0; i < moonbot_eyes.numPixels(); ++i) {
    moonbot_eyes.setPixelColor(i, eyes_color[0], eyes_color[1], eyes_color[2]);
  }
  moonbot_eyes.show();
}





