#include <moonbot_mech.h>
#include <moonbot_rover_demo.h>
#include <moonbot_humannoid_demo.h>
#include <MoonBot_RemoteController.h>
#include <MoonBot_MechRemoteController.h>
#include <MoonBot_RoverRemoteController.h>
#include <MoonBot_BotRemoteController.h>

#define BUTTON_A_SHORT_PRESS      0x01
#define BUTTON_B_SHORT_PRESS      0x02
#define MOONMECH_MODE             0x10
#define MOONROVER_MODE            0x11
#define MOONBOT_MODE              0x12
#define BLOCKLY_MODE              0x20

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  LED.begin();
  LED.clear();
  LED.show();
  TankBase.distanceCorrection(112);
  TankBase.wheelSpacingSet(146);
}

void loop() {
  switch (ModeCheck()) {
    case BLOCKLY_MODE: {
      MoonBotRemoteController common_remote(&Serial3, MOONBOT_REMOT_DEFAULT_ADDRESS);
      for (;;) {
        common_remote.run();
        if (common_remote.exit_) {
          return;
        }
      }
    }
      break;
    case BUTTON_B_SHORT_PRESS: {
      for(;;) {
        if (Serial.available()) {
          Serial3.write(Serial.read());
        }
        if (Serial3.available()) {
          Serial.write(Serial3.read());
        }
      }
    }
      break;
    case MOONMECH_MODE: {
      MoonBotMechRemoteController mech_remot(&Serial3, MOONBOT_REMOT_DEFAULT_ADDRESS);
      for(;;) {
        mech_remot.run();
        if (mech_remot.exit_) {
          return;
        }
      }
    }
      break;
    case MOONROVER_MODE: {
      MoonBotRoverRemoteController rover_remote(&Serial3, MOONBOT_REMOT_DEFAULT_ADDRESS);
      for(;;) {
        rover_remote.run();
        if (rover_remote.exit_) {
          return;
        }
      }
    }
      break;
    case MOONBOT_MODE: {
      MoonBotBotRemoteController bot_remot(&Serial3, MOONBOT_REMOT_DEFAULT_ADDRESS);
      for(;;) {
        bot_remot.run();
        if (bot_remot.exit_) {
          return;
        }
      }
    }
      break;
    default:
      break;
  }
}

uint8_t ModeCheck() {
  static bool start = false;
  MoonBotRemoteProtocolAnalysis protocol(&Serial3, MOONBOT_REMOT_DEFAULT_ADDRESS);
  for(;;) {
    if (!start) {
      start = true;
      protocol.responseError(0xEA);
    }
    // Blockly mode
    if (isButtonPressed(MOONBOT_PIN_BUTTON_A)) {
      tone(MOONBOT_PIN_BUZZER_SIG, 500, 300);
      LED.setPixelColor(0, 0x002020);
      LED.show();
      delay(300);
      LED.clear();
      LED.show();
      protocol.transmit({0x03,{0x01,0xB1,0x01}});
      return BLOCKLY_MODE;
    }
    // MuVisionSensor firmware update
    if (isButtonPressed(MOONBOT_PIN_BUTTON_B)) {
      tone(MOONBOT_PIN_BUZZER_SIG, 1000, 300);
      LED.setPixelColor(1, 0x2000);
      LED.show();
      return BUTTON_B_SHORT_PRESS;
      break;
    }
    // Remote mode
    if (protocol.capture() == MU_OK) {
      if (protocol.getCMD() == MR_CMD_CONTROLLER
          && protocol.getSubCMD() == MR_CMD_CONTROLLER_FORM_CHECK) {
        protocol.resetParameterIndex();
        uint8_t product = protocol.getParameter<uint8_t>();
        uint8_t form = protocol.getParameter<uint8_t>();
        if (product == MOONBOT_REMOT_PRODUCT_ID) {
          switch (form) {
            case MOONBOT_REMOT_FORM_ROBOT: {
              tone(MOONBOT_PIN_BUZZER_SIG, 500, 200);
              LED.fill(0x200020, 0, 0);
              LED.show();
              delay(300);
              LED.clear();
              LED.show();
              return MOONBOT_MODE;
            }
            case MOONBOT_REMOT_FORM_MECH: {
              tone(MOONBOT_PIN_BUZZER_SIG, 1000, 200);
              LED.fill(0x2020, 0, 0);
              LED.show();
              delay(300);
              LED.clear();
              LED.show();
              return MOONMECH_MODE;
            }
            case MOONBOT_REMOT_FORM_ROVER: {
              tone(MOONBOT_PIN_BUZZER_SIG, 1500, 200);
              LED.fill(0x202000, 0, 0);
              LED.show();
              delay(300);
              LED.clear();
              LED.show();
              return MOONROVER_MODE;
            }
            case 0x10: {
              tone(MOONBOT_PIN_BUZZER_SIG, 500, 300);
              LED.setPixelColor(0, 0x002020);
              LED.show();
              delay(300);
              LED.clear();
              LED.show();
              protocol.transmit({0x03,{0x01,0xB1,0x01}});
              return BLOCKLY_MODE;
            }
            default:
              break;
          }
        }
      }

      if (protocol.getCMD() == MR_CMD_CONTROLLER
          && protocol.getSubCMD() == 0x12) {    // controller demo
        protocol.resetParameterIndex();
        uint8_t demo_type = protocol.getParameter<uint8_t>();
        switch (demo_type) {
          case MOONBOT_REMOT_FORM_ROBOT: {
            tone(MOONBOT_PIN_BUZZER_SIG, 500, 200);
            LED.fill(0x200020, 0, 0);
            LED.show();
            delay(300);
            LED.clear();
            LED.show();
            MoonBot_HumannoidDemo moonbot;
            moonbot.begin();
            uint32_t count = 0;
            for(;;) {
              count++;
              if (count%500 == 0) {
                moonbot.run();
              }
              if (protocol.capture() == MU_OK) {
                break;
              }
            }
          }
            break;
          case MOONBOT_REMOT_FORM_MECH: {
            tone(MOONBOT_PIN_BUZZER_SIG, 1000, 200);
            LED.fill(0x2020, 0, 0);
            LED.show();
            delay(300);
            LED.clear();
            LED.show();
            MoonBotMECHDemo mech;
            mech.begin();
            uint32_t count = 0;
            for(;;) {
              count++;
              if (count%500 == 0) {
                mech.run();
              }
              if (protocol.capture() == MU_OK) {
                break;
              }
            }
          }
            break;
          case MOONBOT_REMOT_FORM_ROVER: {
            tone(MOONBOT_PIN_BUZZER_SIG, 1500, 200);
            LED.fill(0x202000, 0, 0);
            LED.show();
            delay(300);
            LED.clear();
            LED.show();
            MoonBotRoverDemo rover;
            rover.begin();
            uint32_t count = 0;
            for(;;) {
              count++;
              if (count%500 == 0) {
                rover.run();
              }
              if (protocol.capture() == MU_OK) {
                break;
              }
            }
          }
            break;
          default:
            break;
        }
      }
    }
  }
  return 0;
}


