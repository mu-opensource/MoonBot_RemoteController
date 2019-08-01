/*
 * MoonBotMECH.cpp
 *
 *  Created on: 2019年1月21日
 *      Author: ysq
 */

#include <moonbot_mech.h>
#include <Wire.h>

MoonBotMECHDemo::MoonBotMECHDemo(void)
    : MoonBotMECH(Mu, m_servo[kServo1], m_servo[kServo3], m_servo[kServo4]),
      claw_(m_servo[kServo1]),
      lower_arm_(m_servo[kServo3]),
      upper_arm_(m_servo[kServo4]) {
}

MoonBotMECHDemo::~MoonBotMECHDemo(void) {
  end();
}

int MoonBotMECHDemo::begin(void) {
  // 1
//  ball_center_x_ = ?;         // 跟踪球的中心
//  card_center_x_ = 42;          // 卡片跟踪中心
//  shoot_card_width_ += 4;
//  claw_.correction(-2);         // 爪子松紧
//  lower_arm_.correction(-1);    // 防止爪子贴地
  // 2
////  ball_center_x_ = ?;         // 跟踪球的中心
//  card_center_x_ = 42;          // 卡片跟踪中心
//  shoot_card_width_ += 4;
//  claw_.correction(-5);         // 爪子松紧
//  lower_arm_.correction(-1);    // 防止爪子贴地
  // 3
//  //  ball_center_x_ = ?;         // 跟踪球的中心
    card_center_x_ = 42;          // 卡片跟踪中心
//    shoot_card_width_ += 4;
//    claw_.correction(-2);         // 爪子松紧
////    lower_arm_.correction(3);    // 防止爪子贴地

  card_type_ = VISION_SHAPE_CARD_DETECT;

  claw_.attach(kServo1);
  lower_arm_.attach(kServo3, true);
  upper_arm_.attach(kServo4, true);
  TankBase.begin(false, false);
  Mu.begin(&Serial3);
  MoonBotMECH::begin();

  return true;
}

int MoonBotMECHDemo::end(void) {
  claw_.detach();
  lower_arm_.detach();
  upper_arm_.detach();
  MoonBotMECH::end();
  return true;
}

bool MoonBotMECHDemo::CheckBall(void) {
  return true;
//  lower_arm_.setTargetAngle(lower_arm_grabbed_, 3);
//  upper_arm_.setTargetAngle(upper_arm_grabbed_, 3);
//  int detect_count = 0;
//  int undetect_count = 0;
//  while (MoonBotServo::moveAllServoToTarget(0)) {
//    UpdateResult(VISION_BALL_DETECT);
//    if (Mu_->read(VISION_BALL_DETECT, kStatus)) {
//      detect_count++;
//    } else {
//      undetect_count++;
//    }
//  }
//  if (detect_count*10/(undetect_count+detect_count)>5) {
//    return true;
//  }
//  return false;
}
void MoonBotMECHDemo::run(void) {
  switch (mech_basketball_state_) {
    case kSearchBall:
      if (searchBall()) {
        mech_basketball_state_ = kGrabBall;
      }
      break;
    case kGrabBall:
      switch (grabBall()) {
        case kGrabedBall:
          if (CheckBall()) {
            mech_basketball_state_ = kSearchCard;
          } else {
            claw_.setTargetAngle(claw_open_);
            upper_arm_.setTargetAngle(upper_arm_init_, 2);
            lower_arm_.setTargetAngle(lower_arm_init_, 2);
            MoonBotServo::moveAllServoToTarget();
            mech_basketball_state_ = kSearchBall;
          }
          break;
        case kUndetectBall:
          mech_basketball_state_ = kSearchBall;
          break;
        default:
          break;
      }
      break;
    case kSearchCard:
      if(searchCard()) {
        mech_basketball_state_ = kShootBall;
      }
      break;
    case kShootBall:
      switch(shootBall()) {
        case kShootedBall:
          mech_basketball_state_ = kSearchBall;
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
}

