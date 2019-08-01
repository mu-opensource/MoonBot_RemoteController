/*
 * MoonBot_MechRemotController.cpp
 *
 *  Created on: 2019年4月15日
 *      Author: ysq
 */

#include <MoonBot_MechRemoteController.h>

MoonBotMechRemoteController::MoonBotMechRemoteController(MuVsUart* uart,
                                                         uint32_t address,
                                                         bool response_enable)
    : MoonBotRemoteController(uart, address, response_enable),
      mech_(Mu, m_servo[kServo1], m_servo[kServo3], m_servo[kServo4]) {
  form_ = MOONBOT_REMOT_FORM_MECH;
  Mu.begin(uart);
  speaker.begin(Serial1);
  for (int i = 0; i < kServoNum; ++i) {
    m_servo[i].power(true);
  }
  m_servo[kServo4].reverse(true);        // MECH upper arm
  m_servo[kServo3].reverse(true);        // MECH lower arm
  TankBase.begin();
  // buzzer begin
  pinMode(MOONBOT_PIN_BUZZER_SIG, OUTPUT);
  pinMode(MOONBOT_PIN_BUZZER_SHDW, OUTPUT);
  digitalWrite(MOONBOT_PIN_BUZZER_SHDW, LOW);
}

MoonBotMechRemoteController::~MoonBotMechRemoteController(void) {}

void MoonBotMechRemoteController::CommandMatcher(void) {
  // Not available command: break; otherwise: return;
  MACTCH_BEGIN
    //==============AppButton==============
    MACTCH_CMD(MR_CMD_APP_BUTTON)
      MACTCH_SUB_CMD(MR_CMD_APP_BUTTON_CLICK, mechAppButtonClick);
    MATCH_END
  MATCH_END

  MoonBotRemoteController::CommandMatcher();
}

void MoonBotMechRemoteController::RunEvent(void) {
  switch (remote_event_) {
    case kMoonBotRemoteMechEventSearchGrabBall:
      static uint32_t ball_count = 0;
      ball_count++;
      if (ball_count%500) {
        break;
      }
      switch (mech_basketball_state_) {
        case kSearchBall:
          if (mech_.searchBall()) {
            mech_basketball_state_ = kGrabBall;
          }
          break;
        case kGrabBall:
          switch (mech_.grabBall()) {
            case kGrabedBall:
              remote_event_ = kMoonBotRemoteCommonEventNone;
              mech_.end();
              break;
            case kUndetectBall:
              mech_basketball_state_ = kSearchBall;
              break;
            default:
              break;
          }
          break;
        default:
          break;
      };
      break;
    case kMoonBotRemoteMechEventSearchShootBall:
      static uint32_t ball_shoot_count = 0;
      ball_shoot_count++;
      if (ball_shoot_count%500) {
        break;
      }
      switch (mech_basketball_state_) {
        case kSearchCard:
          if(mech_.searchCard()) {
            mech_basketball_state_ = kShootBall;
          }
          break;
        case kShootBall:
          switch(mech_.shootBall()) {
            case kShootedBall:
              remote_event_ = kMoonBotRemoteCommonEventNone;
              mech_.end();
              break;
            case kUndetectCard:
              mech_basketball_state_ = kSearchCard;
              break;
            default:
              break;
          }
          break;
        default:
          break;
      }
      break;
    default:
      MoonBotRemoteController::RunEvent();
      break;
  }
}

uint8_t MoonBotMechRemoteController::mechAppButtonClick(void) {
  resetParameterIndex();
  uint8_t click_type = getParameter<uint8_t>();
  resetParameterIndex();
  switch (id_) {
    case 0x81: {
      switch (click_type) {
        case 0x01:
          if (mech_.begin() == MU_OK) {
            remote_event_ = kMoonBotRemoteMechEventSearchGrabBall;
            mech_basketball_state_ = kSearchBall;
          } else {
            EPRINTF("Enable MECH search grab ball failure\n");
          }
          break;
        case 0x02:
          remote_event_ = kMoonBotRemoteCommonEventNone;
          mech_.end();
          break;
        default:
          break;
      }
      return MU_OK;
    }
    case 0x82: {
      switch (click_type) {
        case 0x01:
          if (mech_.begin() == MU_OK) {
            remote_event_ = kMoonBotRemoteMechEventSearchShootBall;
            mech_basketball_state_ = kSearchCard;
          } else {
            EPRINTF("Enable MECH search shoot ball failure\n");
          }
          break;
        case 0x02:
          remote_event_ = kMoonBotRemoteCommonEventNone;
          mech_.end();
          break;
        default:
          break;
      }
      return MU_OK;
    }
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

uint8_t MoonBotMechRemoteController::dance(uint8_t num) {
  resetParameterIndex();
  switch (num) {
    case 0x00:
      break;
    default:
      return MU_ERROR_REG_VALUE;
  }
  return MU_OK;
}
