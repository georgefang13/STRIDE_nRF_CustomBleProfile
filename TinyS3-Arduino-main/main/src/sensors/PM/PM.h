#ifndef PM_H
#define PM_H

#include "../GasSensor.h"
#include "../../lib/particulate-matter/Adafruit_PM25AQI.h"
#include <Arduino.h>
#include <Wire.h>
#include <string>

class PMSensor : public GasSensor
{
public:
  PMSensor();

  std::string getName() const override;
  std::map<std::string, SensorData> getData() override;
  void init() override;

private:
  Adafruit_PM25AQI aqi;
};

#endif