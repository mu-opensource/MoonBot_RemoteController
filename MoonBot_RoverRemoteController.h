/*
 * MoonBot_RoverRemoteController.h
 *
 *  Created on: 2019年4月28日
 *      Author: ysq
 */

#ifndef MOONBOT_REMOTCONTROLLER_MOONBOT_ROVERREMOTECONTROLLER_H_
#define MOONBOT_REMOTCONTROLLER_MOONBOT_ROVERREMOTECONTROLLER_H_

#include <MoonBot_Rover.h>
#include <MoonBot_RemoteController.h>

class MoonBotRoverRemoteController : public MoonBotRemoteController {
 public:
  MoonBotRoverRemoteController(MuVsUart* uart,
                               uint32_t address,
                               bool response_enable = false);
  virtual ~MoonBotRoverRemoteController(void);
  MoonBotRoverRemoteController& operator=(const MoonBotRoverRemoteController &) = delete;
  MoonBotRoverRemoteController(const MoonBotRoverRemoteController&) = delete;

 protected:
  // command match
  virtual void CommandMatcher(void) override;
  virtual void RunEvent(void) override;
  inline uint8_t roverAppButtonClick(void);

  enum moonbot_remote_rover_event_t {
    kMoonBotRemoteRoverEventFollowBall      = kMoonBotRemoteCommonEventNum,
    kMoonBotRemoteRoverEventAvoid,
    kMoonBotRemoteRoverEventCardDetect,
    kMoonBotRemoteRoverEventNum,
  };
  MoonBotRover rover_;
  uint8_t left_avoid_pin[2] = {
      moonbotPortToPin(kPort7, kPortPin1),
      moonbotPortToPin(kPort7, kPortPin2)
  };
  uint8_t right_avoid_pin[2] = {
      moonbotPortToPin(kPort3, kPortPin1),
      moonbotPortToPin(kPort3, kPortPin2)
  };
  bool avoid_enable_ = false;
};



#endif /* MOONBOT_REMOTCONTROLLER_MOONBOT_ROVERREMOTECONTROLLER_H_ */
