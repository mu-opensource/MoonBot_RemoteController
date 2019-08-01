/*
 * MoonBot_RoverRemoteController.cpp
 *
 *  Created on: 2019年4月28日
 *      Author: ysq
 */

#include "MoonBot_RoverRemoteController.h"

MoonBotRoverRemoteController::MoonBotRoverRemoteController(MuVsUart* uart,
                                                       uint32_t address,
                                                       bool response_enable)
    : MoonBotRemoteController(uart, address, response_enable),
      rover_(Mu) {
  form_ = MOONBOT_REMOT_FORM_ROVER;
  Mu.begin(uart);
  for (int i = 0; i < kServoNum; ++i) {
    m_servo[i].power(true);
  }
  m_servo[kServo3].reverse(true);
  // Init IO;
  pinMode(left_avoid_pin[0], INPUT_PULLUP);
  pinMode(left_avoid_pin[1], INPUT_PULLUP);
  pinMode(right_avoid_pin[0], INPUT_PULLUP);
  pinMode(right_avoid_pin[1], INPUT_PULLUP);
  TankBase.begin();
}

MoonBotRoverRemoteController::~MoonBotRoverRemoteController(void) {}

void MoonBotRoverRemoteController::CommandMatcher(void) {
  // Not available command: break; otherwise: return;
  MACTCH_BEGIN
  //==============AppButton==============
    MACTCH_CMD(MR_CMD_APP_BUTTON)
      MACTCH_SUB_CMD(MR_CMD_APP_BUTTON_CLICK, roverAppButtonClick);
    MATCH_END
  MATCH_END

  MoonBotRemoteController::CommandMatcher();
}

void MoonBotRoverRemoteController::RunEvent(void) {
  switch (remote_event_) {
    case kMoonBotRemoteRoverEventFollowBall: {
      static unsigned long ball_count = 0;
      if (ball_count++%500 == 0) {
        rover_.runFollowBall();
      }
    }
      break;
    case kMoonBotRemoteRoverEventCardDetect:
      static unsigned long count = 0;
      if (count++%500 == 0) {
        rover_.runTrafficNumber();
      }
      break;
    default:
      break;
  }
  if (avoid_enable_) {
    if (digitalRead(left_avoid_pin[0]) == LOW
        || digitalRead(left_avoid_pin[1]) == LOW) {
      int left_speed = TankBase.read(kLeftMotor);
      int right_speed = TankBase.read(kRightMotor);
      if (left_speed != 0 || right_speed != 0) {
        TankBase.write(80, -80);
        do {
          delay(1000);
        } while (digitalRead(left_avoid_pin[0]) == LOW
            || digitalRead(left_avoid_pin[1]) == LOW);
        TankBase.write(left_speed, right_speed);
      }
    } else if (digitalRead(right_avoid_pin[0]) == LOW
        || digitalRead(right_avoid_pin[1]) == LOW) {
      int left_speed = TankBase.read(kLeftMotor);
      int right_speed = TankBase.read(kRightMotor);
      if (left_speed != 0 || right_speed != 0) {
        TankBase.write(-80, 80);
        do {
          delay(1000);
        } while (digitalRead(right_avoid_pin[0]) == LOW
            || digitalRead(right_avoid_pin[1]) == LOW);
        TankBase.write(left_speed, right_speed);
      }
    }
  }
  MoonBotRemoteController::RunEvent();
}

uint8_t MoonBotRoverRemoteController::roverAppButtonClick(void) {
  resetParameterIndex();
  uint8_t click_type = getParameter<uint8_t>();
  resetParameterIndex();
  switch (id_) {
    case 0x81: {
      switch (click_type) {
        case 0x01:
          remote_event_ = kMoonBotRemoteRoverEventFollowBall;
          rover_.followBallBegin();
          break;
        case 0x02:
          remote_event_ = kMoonBotRemoteCommonEventNone;
          rover_.followBallEnd();
          break;
        default:
          return MU_ERROR_REG_VALUE;
      }
      resetParameterIndex();
      return MU_OK;
    }
    case 0x82: {
      switch (click_type) {
        case 0x01:
          avoid_enable_ = true;
//          remote_event_ = kMoonBotRemoteRoverEventAvoid;
          break;
        case 0x02:
          avoid_enable_ = false;
//          remote_event_ = kMoonBotRemoteCommonEventNone;
          break;
        default:
          break;
      }
      return MU_OK;
    }
    case 0x83: {
      switch (click_type) {
        case 0x01:
          if (rover_.begin() == MU_OK) {
            remote_event_ = kMoonBotRemoteRoverEventCardDetect;
          } else {
            EPRINTF("Enable rover NumTraffic card failure\n");
          }
          break;
        case 0x02:
          remote_event_ = kMoonBotRemoteCommonEventNone;
          rover_.end();
          break;
        default:
          break;
      }
      return MU_OK;
    }
    // Dance
//    case 0x90:
//    case 0x91:
//    case 0x92:
//    case 0x93:
//    case 0x94:
//    case 0x95:
//    case 0x96:
//    case 0x97:
//    case 0x98:
//    case 0x99:
//    case 0x9A:
//    case 0x9B:
//    case 0x9C:
//    case 0x9D:
//    case 0x9E:
//    case 0x9F:
//      return dance(id_&0x0F);
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

//uint8_t MoonBotRoverRemoteController::buzzerPlay(uint8_t num) {
//  resetParameterIndex();
//  switch (num) {
//    case 0x00:
//      buzzerPlayerPGM(notationJingleBells, sizeJingleBells);
//      break;
//    case 0x01:
//      tone(MOONBOT_PIN_BUZZER_SIG, 200, 500);
//      break;
//    case 0x02:
//      tone(MOONBOT_PIN_BUZZER_SIG, 300, 500);
//      break;
//    case 0x03:
//      tone(MOONBOT_PIN_BUZZER_SIG, 400, 500);
//      break;
//    case 0x04:
//      tone(MOONBOT_PIN_BUZZER_SIG, 500, 500);
//      break;
//    default:
//      return MU_ERROR_REG_VALUE;
//  }
//  return MU_OK;
//}

//uint8_t MoonBotRoverRemoteController::ledAction(uint8_t num) {
//  switch (num) {
//    case 0x00: {
//      for (int i = 0; i < 5; ++i) {
//        onBoardLED.setPixelColor(0, 0x20, 0, 0);
//        onBoardLED.setPixelColor(1, 0, 0, 0x20);
//        onBoardLED.show();
//        delay(200);
//        onBoardLED.setPixelColor(1, 0x20, 0, 0);
//        onBoardLED.setPixelColor(0, 0, 0, 0x20);
//        onBoardLED.show();
//        delay(200);
//      }
//      onBoardLED.clear();
//      onBoardLED.show();
//    }
//      break;
//    case 0x01: {
//      for (int i = 0; i < 5; ++i) {
//        onBoardLED.setBrightness(0);
//        onBoardLED.setPixelColor(0, 0x20, 0x20, 0x20);
//        onBoardLED.setPixelColor(1, 0x20, 0x20, 0x20);
//        for (int i = 0; i < 255; ++i) {
//          onBoardLED.setBrightness(i);
//          onBoardLED.setPixelColor(0, 0x20, 0x20, 0x20);
//          onBoardLED.setPixelColor(1, 0x20, 0x20, 0x20);
//          onBoardLED.show();
//          delay(1);
//        }
//        for (int i = 255; i > 0; --i) {
//          onBoardLED.setBrightness(i);
//          onBoardLED.setPixelColor(0, 0x20, 0x20, 0x20);
//          onBoardLED.setPixelColor(1, 0x20, 0x20, 0x20);
//          onBoardLED.show();
//          delay(1);
//        }
//        onBoardLED.setBrightness(255);
//        onBoardLED.clear();
//        onBoardLED.show();
//      }
//    }
//      break;
//    case 0x02: {
//      resetParameterIndex();
//      uint8_t click_type = getParameter<uint8_t>();
//      if (click_type == 0x01) {
//        Mu.LedSetColor(kLed1, kLedWhite, kLedWhite, 4);
//        Mu.LedSetColor(kLed2, kLedWhite, kLedWhite, 4);
//      } else if (click_type == 0x02) {
//        Mu.LedSetColor(kLed1, kLedRed, kLedBlue, 1);
//        Mu.LedSetColor(kLed2, kLedRed, kLedBlue, 1);
//      }
//    }
//      break;
//    default:
//      return MU_ERROR_REG_VALUE;
//  }
//  resetParameterIndex();
//  return MU_OK;
//}

