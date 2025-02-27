#ifndef UV_H
#define UV_H

#include "../GasSensor.h"
#include "../../lib/ultraviolet/Adafruit_LTR390.h"
#include <Arduino.h>
#include <Wire.h>
#include <string>

class UVSensor : public GasSensor
{
public:
  UVSensor();

  std::string getName() const override;
  std::map<std::string, SensorData> getData() override;
  void init() override;

private:
  Adafruit_LTR390 ltr;
};

#endif