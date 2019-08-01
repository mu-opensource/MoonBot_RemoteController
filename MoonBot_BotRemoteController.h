/*
 * MoonBot_BotRemoteController.h
 *
 *  Created on: 2019年5月20日
 *      Author: ysq
 */

#ifndef MOONBOT_REMOTECONTROLLER_MOONBOT_BOTREMOTECONTROLLER_H_
#define MOONBOT_REMOTECONTROLLER_MOONBOT_BOTREMOTECONTROLLER_H_

#include <MoonBot_Humannoid.h>
#include <MoonBot_RemoteController.h>

class MoonBotBotRemoteController : public MoonBotRemoteController {
 public:
  MoonBotBotRemoteController(MuVsUart* uart,
                             uint32_t address,
                             bool response_enable = false);
  virtual ~MoonBotBotRemoteController(void);
  MoonBotBotRemoteController& operator=(const MoonBotBotRemoteController &) = delete;
  MoonBotBotRemoteController(const MoonBotBotRemoteController&) = delete;

 protected:
  // command match
  virtual void CommandMatcher(void) override;
  virtual void RunEvent(void) override;
  inline uint8_t botAppButtonClick(void);

  uint8_t dance(uint8_t num);
  uint8_t buzzerPlay(uint8_t num);
//  uint8_t ledAction(uint8_t num);

  void setEyesColor(uint8_t eyes_color[3]);

  const uint8_t eyes_brightness_ = 0x20;
//  uint8_t eyes_color_[3] = {0, eyes_brightness_, eyes_brightness_};

  MoonBotHumannoid moonbot_;
  enum moonbot_bot_mech_event_t {
    kMoonBotRemoteBotEventNum      = kMoonBotRemoteCommonEventNum,
  };
};



#endif /* MOONBOT_REMOTECONTROLLER_MOONBOT_BOTREMOTECONTROLLER_H_ */
