#ifndef CO2_H
#define CO2_H

#include "../GasSensor.h"
#include "../../lib/carbon-dioxide/SensirionI2CScd4x.h"
#include <Arduino.h>
#include <Wire.h>
#include <string>

class CO2Sensor : public GasSensor
{
public:
  CO2Sensor();

  std::string getName() const override;
  std::map<std::string, SensorData> getData() override;
  void init() override;

private:
  SensirionI2CScd4x scd4x;

  bool checkDataReady();
  void safeRead();
  void printUint16Hex(uint16_t value);
  void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2);
};

#endif