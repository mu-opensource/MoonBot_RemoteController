/*
 * moobot_rover_demo.cpp
 *
 *  Created on: 2019年4月24日
 *      Author: ysq
 */

#include <moonbot_rover_demo.h>


MoonBotRoverDemo::MoonBotRoverDemo(void)
    : MoonBotRover(Mu) {
}

MoonBotRoverDemo::~MoonBotRoverDemo(void) {
  end();
}

int MoonBotRoverDemo::begin(void) {
  moonbot_eyes.begin();
  Mu.begin(&Serial3);
  TankBase.begin(false);
  onBoardLED.begin();
  pinMode(left_avoid_pin[0], INPUT_PULLUP);
  pinMode(left_avoid_pin[1], INPUT_PULLUP);
  pinMode(right_avoid_pin[0], INPUT_PULLUP);
  pinMode(right_avoid_pin[1], INPUT_PULLUP);

  return MoonBotRover::begin();
}

void MoonBotRoverDemo::run(void) {
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
  }
  if (digitalRead(right_avoid_pin[0]) == LOW
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
  runTrafficNumber();
}
