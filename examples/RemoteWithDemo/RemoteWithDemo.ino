#include <moonbot_mech.h>
#include <moonbot_rover_demo.h>
#include <moonbot_humannoid_demo.h>
#include <MoonBot_RemoteController.h>
#include <MoonBot_MechRemoteController.h>
#include <MoonBot_RoverRemoteController.h>
#include <MoonBot_BotRemoteController.h>

MoonBotRemoteProtocolAnalysis protocol(&Serial3, MU_VISION_SENSOR_DEFAULT_ADDRESS);

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  protocol.responseError(0xEA);
  LED.begin();
  LED.clear();
  LED.show();
  TankBase.rpmCorrection(114);
  TankBase.distanceCorrection(112);
  TankBase.wheelSpacingSet(146);
}

void loop() {
  // Blockly mode
  if (isButtonPressed(MOONBOT_PIN_BUTTON_A)) {
    tone(MOONBOT_PIN_BUZZER_SIG, 500, 300);
    LED.setPixelColor(0, 0x002020);
    LED.show();
    delay(300);
    LED.clear();
    LED.show();
    MoonBotRemoteController common_remote(&Serial3, MU_VISION_SENSOR_DEFAULT_ADDRESS);
    MuVisionSensorUart vision_sensor(&Serial3, MU_VISION_SENSOR_DEFAULT_ADDRESS);
    vision_sensor.Set(0xB1, 0x01);
    for (;;) {
      common_remote.run();
    }
  }
  // MuVisionSensor firmware update
  if (isButtonPressed(MOONBOT_PIN_BUTTON_B)) {
    tone(MOONBOT_PIN_BUZZER_SIG, 1000, 300);
    LED.setPixelColor(1, 0x2000);
    LED.show();
    for(;;) {
      if (Serial.available()) {
        Serial3.write(Serial.read());
      }
      if (Serial3.available()) {
        Serial.write(Serial3.read());
      }
    }
  }
  // Remote mode
  if (protocol.capture() == MU_OK) {
    if (protocol.getCMD() == MR_CMD_CONTROLLER
        && protocol.getSubCMD() == MR_CMD_CONTROLLER_FORM_CHECK) {
      protocol.resetParameterIndex();
      uint8_t product = protocol.getParameter<uint8_t>();
      uint8_t form = protocol.getParameter<uint8_t>();
      if (product == MOONBOT_REMOT_PRODUCT_ID) {
        tone(MOONBOT_PIN_BUZZER_SIG, 500, 300);
        switch (form) {
          case MOONBOT_REMOT_FORM_ROBOT: {
            tone(MOONBOT_PIN_BUZZER_SIG, 500, 200);
            LED.fill(0x200020, 0, 0);
            LED.show();
            delay(300);
            LED.clear();
            LED.show();
            MoonBotBotRemoteController bot_remot(&Serial3, MU_VISION_SENSOR_DEFAULT_ADDRESS);
            for(;;) {
              bot_remot.run();
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
            MoonBotMechRemoteController mech_remot(&Serial3, MU_VISION_SENSOR_DEFAULT_ADDRESS);
            for(;;) {
              mech_remot.run();
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
            MoonBotRoverRemoteController rover_remote(&Serial3, MU_VISION_SENSOR_DEFAULT_ADDRESS);
            for(;;) {
              rover_remote.run();
            }
          }
            break;
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


