/*
 * StandardBlockly.ino
 *
 *  Created on: 2019年6月11日
 *      Author: ysq
 */

#include <MoonBot_RemoteController.h>

MoonBotRemoteController common_remote(&Serial3, MU_VISION_SENSOR_DEFAULT_ADDRESS);
MuVisionSensorUart vision_sensor(&Serial3, MU_VISION_SENSOR_DEFAULT_ADDRESS);

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
}

void loop() {
  common_remote.run();
  if (isButtonPressed(MOONBOT_PIN_BUTTON_A)) {
    vision_sensor.Set(0xB1, 0x01);
  }
  if (isButtonPressed(MOONBOT_PIN_BUTTON_B)) {
    for(;;) {
      if (Serial.available()) {
        Serial3.write(Serial.read());
      }
      if (Serial3.available()) {
        Serial.write(Serial.read());
      }
    }
  }
}

