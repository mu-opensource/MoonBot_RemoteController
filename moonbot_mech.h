/*
 * moonbot_mech.h
 *
 *  Created on: 2019年2月21日
 *      Author: ysq
 */

#ifndef PRODUCTS_MOONBOT_MECH_H_
#define PRODUCTS_MOONBOT_MECH_H_

#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#include "MoonBot.h"

class MoonBotMECHDemo : public MoonBotMECH {
 public:
  MoonBotMECHDemo(void);
  virtual ~MoonBotMECHDemo(void);
  MoonBotMECHDemo& operator=(const MoonBotMECHDemo &) = delete;
  MoonBotMECHDemo(const MoonBotMECHDemo&) = delete;

  int begin(void);
  void run(void);
  int end(void);

  void test_code(void);
  MoonBotServo& claw_;
  MoonBotServo& lower_arm_;
  MoonBotServo& upper_arm_;

 private:
  bool CheckBall(void);
    enum mech_basketball_t {
      kSearchBall,
      kGrabBall,
      kSearchCard,
      kShootBall
    };
    mech_basketball_t mech_basketball_state_ = kSearchBall;

};

#endif /* PRODUCTS_MOONBOT_MECH_H_ */
