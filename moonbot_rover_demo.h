/*
 * moobot_rover_demo.h
 *
 *  Created on: 2019年4月24日
 *      Author: ysq
 */

#ifndef MOONBOT_REMOTECONTROLLER_MOONBOT_ROVER_DEMO_H_
#define MOONBOT_REMOTECONTROLLER_MOONBOT_ROVER_DEMO_H_


#include "MoonBot.h"

class MoonBotRoverDemo : public MoonBotRover {
 public:
  MoonBotRoverDemo(void);
  virtual ~MoonBotRoverDemo(void);
  MoonBotRoverDemo& operator=(const MoonBotRoverDemo &) = delete;
  MoonBotRoverDemo(const MoonBotRoverDemo&) = delete;

  int begin(void);
  void run(void);

 protected:
  MoonBotServo& head_ = m_servo[0];
  uint8_t left_avoid_pin[2] = {
      moonbotPortToPin(kPort7, kPortPin1),
      moonbotPortToPin(kPort7, kPortPin2)
  };
  uint8_t right_avoid_pin[2] = {
      moonbotPortToPin(kPort3, kPortPin1),
      moonbotPortToPin(kPort3, kPortPin2)
  };
};


#endif /* MOONBOT_REMOTECONTROLLER_MOONBOT_ROVER_DEMO_H_ */
