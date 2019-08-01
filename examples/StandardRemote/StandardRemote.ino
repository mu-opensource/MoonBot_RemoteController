/*
 * StandardRemote.ino
 *
 *  Created on: 2019年6月11日
 *      Author: ysq
 */

#include <MoonBot_RemoteController.h>
#include <MoonBot_MechRemoteController.h>
#include <MoonBot_RoverRemoteController.h>
#include <MoonBot_BotRemoteController.h>

MoonBotRemoteProtocolAnalysis protocol(&Serial3, MU_VISION_SENSOR_DEFAULT_ADDRESS);

int serial_putc( char c, struct __file * )
{
  Serial.write( c );
  return c;
}
void setup() {
  fdevopen( &serial_putc, 0 );
  Serial.begin(115200);
  Serial3.begin(115200);
  protocol.responseError(0xEA);
  LED.begin();
  LED.clear();
  LED.show();
}

void loop() {
  // Blockly mode
  if (isButtonPressed(MOONBOT_PIN_BUTTON_A)) {
    MoonBotRemoteController common_remote(&Serial3, MU_VISION_SENSOR_DEFAULT_ADDRESS);
    MuVisionSensorUart vision_sensor(&Serial3, MU_VISION_SENSOR_DEFAULT_ADDRESS);
    vision_sensor.Set(0xB1, 0x01);
    for (;;) {
      common_remote.run();
      if (isButtonPressed(MOONBOT_PIN_BUTTON_A)) {
        vision_sensor.Set(0xB1, 0x01);
      }
    }
  }
  // MuVisionSensor firmware update
  if (isButtonPressed(MOONBOT_PIN_BUTTON_B)) {
    LED.fill(0x2000, 0, 0);
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
        switch (form) {
          case MOONBOT_REMOT_FORM_ROBOT: {
            MoonBotBotRemoteController bot_remot(&Serial3, MU_VISION_SENSOR_DEFAULT_ADDRESS);
            for(;;) {
              bot_remot.run();
            }
          }
            break;
          case MOONBOT_REMOT_FORM_MECH: {
            MoonBotMechRemoteController mech_remot(&Serial3, MU_VISION_SENSOR_DEFAULT_ADDRESS);
            for(;;) {
              mech_remot.run();
            }
          }
            break;
          case MOONBOT_REMOT_FORM_ROVER: {
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
  }
}

