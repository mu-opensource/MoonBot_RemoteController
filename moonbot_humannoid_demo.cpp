/*
 * moonbot_humannoid_demo.cpp
 *
 *  Created on: 2019年7月12日
 *      Author: ysq
 */

#include "moonbot_humannoid_demo.h"

MoonBot_HumannoidDemo::MoonBot_HumannoidDemo(void)
    : mu_uart_(&Serial3, 0x60) {
  randomSeed(analogRead(A0));
  onBoardLED.begin();
  Mu.begin(&Serial3);
  speaker.begin(Serial2);
  moonbot_eyes.begin();
  TankBase.begin();
  TankBase.wheelSpacingSet(133);
  TankBase.distanceCorrection(120);
  time2greeting_.start(10000, AsyncDelay::MILLIS);
  time2search_.start(2000, AsyncDelay::MILLIS);
  time2response_.start(3000, AsyncDelay::MILLIS);
  pinMode(left_touch_, INPUT);
  pinMode(right_touch_, INPUT);
  larm_servo_.attach(kServo4);
  rarm_servo_.attach(kServo1, true);
  head_servo_.attach(kServo3, true);
}

MoonBot_HumannoidDemo::~MoonBot_HumannoidDemo(void) {
  end();
}

bool MoonBot_HumannoidDemo::begin() {
  moonbot_eyes.clear();
  moonbot_eyes.show();
  Mu.VisionBegin(VISION_BODY_DETECT);
  Mu.VisionSetLevel(VISION_BODY_DETECT, kLevelAccuracy);
  rarm_servo_.write(90);
  larm_servo_.write(90);
  head_servo_.write(110);
  return true;
}

void MoonBot_HumannoidDemo::end() {
  Mu.VisionEnd(VISION_BODY_DETECT);
  rarm_servo_.detach();
  larm_servo_.detach();
  head_servo_.detach();
  TankBase.write(0, 0);
}

void MoonBot_HumannoidDemo::run() {
  if (Mu.UpdateResult(VISION_BODY_DETECT, false)&VISION_BODY_DETECT) {
    MuVsVisionState* vision_state = Mu.GetVisionState(VISION_BODY_DETECT);
    if (vision_state->detect) {
      time2search_.restart();
      if (vision_state->vision_result[0].x_value < 45) {
        TankBase.write(-100, 100);
      } else if (vision_state->vision_result[0].x_value > 55) {
        TankBase.write(100, -100);
      } else {
        TankBase.write(0, 0);
        if (time2greeting_.isExpired()) {
          time2greeting_.restart();
          switch (random(3)) {
            case 0:
              speaker.play((char *)"0201");   // hello
              break;
            case 1:
              speaker.play((char *)"0202");   // hi
              break;
            case 2:
              speaker.play((char *)"0204");   // nice to meet you
              break;
            default:
              break;
          }
          for (unsigned int i = 0; i < kServoNum; ++i) {
            m_servo[i].power(false);
          }
          switch (random(3)) {
            case 0:
              theaterChase(moonbot_eyes, 0x3030, 15);
              colorFade(moonbot_eyes, 0, 0x30, 0x30,2);
              break;
            case 1:
              colorWipe(moonbot_eyes, 0, 30);
              colorWipe(moonbot_eyes, 0x3030, 30);
              break;
            case 2:
              moonbot_eyes.setBrightness(0x30);
              rainbowCycle(moonbot_eyes, 1);
              moonbot_eyes.setBrightness(0xFF);
              break;
            default:
              break;
          }
          for (unsigned int i = 0; i < kServoNum; ++i) {
            m_servo[i].power(true);
          }
        }
      }
      int head_angle = head_servo_.read();
      if (vision_state->vision_result[0].y_value < 45
          && head_angle < 130) {
        head_servo_.write(head_angle+(50-vision_state->vision_result[0].y_value)/4);
      } else if (vision_state->vision_result[0].y_value > 55
          && head_angle > 90) {
        head_servo_.write(head_angle-(vision_state->vision_result[0].y_value-50)/4);
      }
    } else {
      if (time2search_.isExpired()) {
        if (TankBase.read(kLeftMotor) == 0) {
          head_servo_.write(110);
          if (Mu.read(VISION_BODY_DETECT, kXValue) > 50) {
            TankBase.write(100, -100);
          } else {
            TankBase.write(-100, 100);
          }
        }
      } else {
        TankBase.write(0, 0);
      }
    }
    // TODO change this to IR close
  //  if (digitalRead(right_touch) == HIGH) {
    if (time2response_.isExpired()) {
      uint8_t ir_value;
      mu_uart_.Get(0x50, &ir_value);
      if (ir_value > 10) {
        time2response_.restart();
        time2greeting_.restart();
        TankBase.write(0, 0);
        switch (random(3)) {
          case 0:
            speaker.play((char *)"0201");   // hello
            break;
          case 1:
            speaker.play((char *)"0202");   // hi
            break;
          case 2:
            speaker.play((char *)"0204");   // nice to meet you
            break;
          default:
            break;
        }
        int right_angle = larm_servo_.read();
        larm_servo_.write(160);
        delay(300);
        for (int i = 0; i < 5; ++i) {
          larm_servo_.write(150);
          delay(100);
          larm_servo_.write(160);
          delay(100);
        }
        larm_servo_.write(right_angle);
      }
    }
  }
}

