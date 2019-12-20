/*
 * MoonBot_MechRemotController.h
 *
 *  Created on: 2019年4月15日
 *      Author: ysq
 */

#ifndef MOONBOT_REMOTCONTROLLER_MOONBOT_MECHREMOTECONTROLLER_H_
#define MOONBOT_REMOTCONTROLLER_MOONBOT_MECHREMOTECONTROLLER_H_

#include <MoonBot_MECH.h>
#include <MoonBot_RemoteController.h>

class MoonBotMechRemoteController : public MoonBotRemoteController {
 public:
  MoonBotMechRemoteController(MuUart::hw_port_t uart,
                             uint32_t address,
                             bool response_enable = false);
  virtual ~MoonBotMechRemoteController(void);
  MoonBotMechRemoteController& operator=(const MoonBotMechRemoteController &) = delete;
  MoonBotMechRemoteController(const MoonBotMechRemoteController&) = delete;

 protected:
  // command match
  virtual void CommandMatcher(void) override;
  virtual void RunEvent(void) override;
  inline uint8_t mechAppButtonClick(void);

  uint8_t dance(uint8_t id);

  MoonBotMECH mech_;
  enum moonbot_remot_mech_event_t {
    kMoonBotRemoteMechEventSearchGrabBall      = kMoonBotRemoteCommonEventNum,
    kMoonBotRemoteMechEventSearchShootBall,
    kMoonBotRemoteMechEventNum,
  };

  enum mech_t {
    kSearchBall,
    kGrabBall,
    kSearchCard,
    kShootBall
  };
  mech_t mech_basketball_state_ = kSearchBall;
};

#endif /* MOONBOT_REMOTCONTROLLER_MOONBOT_MECHREMOTECONTROLLER_H_ */
