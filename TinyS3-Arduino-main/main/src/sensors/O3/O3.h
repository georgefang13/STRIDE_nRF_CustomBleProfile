#ifndef O3_H
#define O3_H

#include "../GasSensor.h"
#include "../../lib/ozone/DFRobot_OzoneSensor.h"
#include <Arduino.h>
#include <Wire.h>
#include <string>

#define COLLECT_NUMBER 20 // collect number, the collection range is 1-100
#define Ozone_IICAddress OZONE_ADDRESS_3

class O3Sensor : public GasSensor
{
public:
  O3Sensor();

  std::string getName() const override;
  std::map<std::string, SensorData> getData() override;
  void init() override;

private:
  DFRobot_OzoneSensor Ozone;
};

#endif