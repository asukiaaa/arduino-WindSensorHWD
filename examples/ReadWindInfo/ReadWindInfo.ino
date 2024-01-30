#include <WindSensorHWD_asukiaaa.hpp>

#ifndef SERIAL_FOR_WINDSENSOR
#define SERIAL_FOR_WINDSENSOR Serial1
#endif
WindSensorHWD_asukiaaa::Core windSensor(&SERIAL_FOR_WINDSENSOR);

void setup() {
  Serial.begin(115200);
  windSensor.beginWithSerial();
}

void loop() {
  auto infoStateSystem = windSensor.readInfoStateSystem();
  if (infoStateSystem.readResult == 0) {
    infoStateSystem.printToStream(&Serial);
  } else {
    Serial.println("Error infoStateSystem.readResult: " +
                   String(infoStateSystem.readResult));
  }

  auto infoSensor = windSensor.readInfoSensor();
  if (infoSensor.readResult == 0) {
    infoSensor.printToStream(&Serial);
  } else {
    Serial.println("Error infoSensor.readResult: " +
                   String(infoSensor.readResult));
  }

  Serial.println("at " + String(millis()));
  delay(1000);
}
