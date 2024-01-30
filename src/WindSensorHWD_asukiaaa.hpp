#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

namespace WindSensorHWD_asukiaaa {

namespace ReadResult {
static const uint8_t NoProblem = 0x00;
static const uint8_t NotHandled = 0x01;
static const uint8_t CannotReceiveData = 0x02;
static const uint8_t InvalidFormat = 0x03;
static const uint8_t InvalidResponseCommand = 0x04;
static const uint8_t UnmatchChecksum = 0x05;
}  // namespace ReadResult

class InfoSensor {
 public:
  uint8_t readResult = ReadResult::NotHandled;
  float windSpeed = 0;
  int windDirection = 0;
  int windAngleOfDepression = 0;
  // float vecX = 0;
  // float vecY = 0;
  // float vecZ = 0;
  float temperature = 0;
  unsigned long readAt = 0;

  void updateFromBytes(char* bytes) {
    char* endP;
    windSpeed = strtod(&bytes[4], &endP);
    windDirection = strtol(endP + 1, &endP, 10);
    windAngleOfDepression = strtol(endP + 1, &endP, 10);
    // vecX = strtod(endP + 1, &endP);
    // vecY = strtod(endP + 1, &endP);
    // vecZ = strtod(endP + 1, &endP);
    temperature = strtod(endP + 1, &endP);
  }

  void printToStream(Stream* serial) const {
    serial->print("readResult: ");
    serial->println(readResult);
    serial->print("windSpeed: ");
    serial->println(String(windSpeed, 3));
    serial->print("windDirection: ");
    serial->println(windDirection);
    serial->print("windAngleOfDepression: ");
    serial->println(windAngleOfDepression);
    // serial->print("vecX: ");
    // serial->println(String(vecX, 3));
    // serial->print("vecY: ");
    // serial->println(String(vecY, 3));
    // serial->print("vecZ: ");
    // serial->println(String(vecZ, 3));
    serial->print("temperature: ");
    serial->println(String(temperature, 1));
    serial->print("readAt: ");
    serial->println(readAt);
  }
};

class InfoStateSystem {
 public:
  uint8_t readResult = ReadResult::NotHandled;
  unsigned long readAt = 0;
  uint16_t stateSystem;
  uint16_t stateHardware;
  uint16_t stateSoftware;

  void updateFromBytes(char* bytes) {
    char* endP;
    stateSystem = strtoul(&bytes[4], &endP, 10);
    stateHardware = strtoul(endP + 1, &endP, 10);
    stateSoftware = strtoul(endP + 1, &endP, 10);
  }

  void printToStream(Stream* serial) const {
    serial->print("readResult: ");
    serial->println(readResult);
    serial->print("stateSystem: 0b");
    serial->println(String(stateSystem, BIN));
    serial->print("stateHardware: 0b");
    serial->println(String(stateHardware, BIN));
    serial->print("stateSoftware: 0b");
    serial->println(String(stateSoftware, BIN));
    serial->print("readAt: ");
    serial->println(readAt);
  }
};

class Core {
 public:
  static const auto ConfigSerial = SERIAL_8N1;
  static const auto BaudrateSerial = 38400;

  Core(HardwareSerial* serial) : serial(serial) {}
  void beginWithSerial() {
    serial->begin(BaudrateSerial, ConfigSerial);
    beginWithoutSerial();
  }
  void beginWithoutSerial() {}

  uint8_t readInfoStateSystemToP(InfoStateSystem* infoModule) {
    clearBufferOnSerial();
    buildBytesForCommand(bytes, "RS");
    serial->print(bytes);
    auto result = infoModule->readResult = waitResponseForCommand("AS");
    infoModule->readAt = millis();
    // for (size_t i = 0; i < indexBytes; ++i) {
    //   Serial.print((char)bytes[i]);
    // }
    infoModule->updateFromBytes(bytes);
    if (result != 0) {
      return result;
    }
    return 0;
  }

  InfoStateSystem readInfoStateSystem() {
    InfoStateSystem infoModule;
    readInfoStateSystemToP(&infoModule);
    return infoModule;
  }

  uint8_t readInfoSensorToP(InfoSensor* infoSensor) {
    clearBufferOnSerial();
    buildBytesForCommand(bytes, "RM");
    serial->print(bytes);
    auto result = infoSensor->readResult = waitResponseForCommand("AM");
    infoSensor->readAt = millis();
    // Serial.println(result);
    if (result != 0) {
      return result;
    }
    infoSensor->updateFromBytes(bytes);
    // infoSensor.printToStream(&Serial);
    return 0;
  }

  InfoSensor readInfoSensor() {
    InfoSensor infoSensor;
    readInfoSensorToP(&infoSensor);
    return infoSensor;
  }

 private:
  static const size_t lenBytes = 106;
  char bytes[lenBytes];
  size_t indexBytes;
  HardwareSerial* serial;
  unsigned long msTimeout = 70;

  uint8_t waitResponseForCommand(String strCommand) {
    if (!receiveOneLine()) {
      return ReadResult::CannotReceiveData;
    }
    if (!hasCorrectFormat()) {
      return ReadResult::InvalidFormat;
    }
    if (!hasCommand(strCommand)) {
      return ReadResult::InvalidResponseCommand;
    }
    if (!hasCorrectChecksum()) {
      return ReadResult::UnmatchChecksum;
    }
    return 0;
  }

  bool receiveOneLine() {
    indexBytes = 0;
    auto waitFrom = millis();
    while (millis() - waitFrom < msTimeout) {
      if (serial->available()) {
        waitFrom = millis();
        bytes[indexBytes++] = serial->read();
        if (indexBytes > 2 && bytes[indexBytes - 2] == '\r' &&
            bytes[indexBytes - 1] == '\n') {
          return true;
        }
      }
    }
    return false;
  }

  bool hasCorrectFormat() {
    return bytes[0] == '<' && bytes[indexBytes - 5] == '>';
  }

  bool hasCorrectChecksum() {
    auto receivedChecksum = strtol(&bytes[indexBytes - 4], 0, 16);
    // Serial.println("receivedChecksum " + String(receivedChecksum, HEX));
    auto checksum = buildCheckSum(bytes, indexBytes - 4);
    // Serial.println("checksum " + String(checksum, HEX));
    return receivedChecksum == checksum;
  }

  bool hasCommand(String strCommand) {
    return bytes[1] == strCommand[0] && bytes[2] == strCommand[1];
  }

  void clearBufferOnSerial() {
    while (serial->available()) {
      serial->read();
    }
  }

  static void buildBytesForCommand(char* bytes, String strCommand) {
    size_t index = 0;
    bytes[index++] = '<';
    bytes[index++] = strCommand[0];
    bytes[index++] = strCommand[1];
    bytes[index++] = ',';
    bytes[index++] = '>';
    auto checksum = buildCheckSum(bytes, index);
    String strChecksum = String(checksum, HEX);
    if (strChecksum.length() == 1) {
      strChecksum = "0" + strChecksum;
    }
    strChecksum.toUpperCase();
    bytes[index++] = strChecksum[0];
    bytes[index++] = strChecksum[1];
    bytes[index++] = '\r';
    bytes[index++] = '\n';
    bytes[index++] = '\0';
  }

  static uint8_t buildCheckSum(char* bytes, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; ++i) {
      checksum += bytes[i];
    }
    return (checksum ^ 0xff) + 1;
  }
};

}  // namespace WindSensorHWD_asukiaaa
