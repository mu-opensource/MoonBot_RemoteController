/*
 * moonbot_humannoid_demo.h
 *
 *  Created on: 2019年7月12日
 *      Author: ysq
 */

#ifndef MOONBOT_REMOTECONTROLLER_MOONBOT_HUMANNOID_DEMO_H_
#define MOONBOT_REMOTECONTROLLER_MOONBOT_HUMANNOID_DEMO_H_

#include <MoonBot.h>

class MoonBot_HumannoidDemo {
 public:
  MoonBot_HumannoidDemo(void);
  virtual ~MoonBot_HumannoidDemo(void);
  MoonBot_HumannoidDemo& operator=(const MoonBot_HumannoidDemo &) = delete;
  MoonBot_HumannoidDemo(const MoonBot_HumannoidDemo&) = delete;

  bool begin();
  void end();
  void run();

 private:
  AsyncDelay time2greeting_;
  AsyncDelay time2search_;
  AsyncDelay time2response_;
  MoonBotServo& larm_servo_ = m_servo[3];
  MoonBotServo& rarm_servo_ = m_servo[0];
  MoonBotServo& head_servo_ = m_servo[2];
  int left_touch_ = moonbotPortToPin(kPort7, kPortPin1);
  int right_touch_ = moonbotPortToPin(kPort3, kPortPin2);
  int r_angle_ = 90;
  int l_angle_ = 90;
  int head_angle_ = 90;
  MuVisionSensorUart mu_uart_;      // TODO temporary function
};

#endif /* MOONBOT_REMOTECONTROLLER_MOONBOT_HUMANNOID_DEMO_H_ */
