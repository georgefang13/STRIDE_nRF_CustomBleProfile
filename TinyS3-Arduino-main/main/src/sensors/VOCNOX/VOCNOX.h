#ifndef VOCNOX_H
#define VOCNOX_H

#include "../GasSensor.h"
#include <Arduino.h>
#include "../../lib/vocnox/SensirionI2CSvm41.h"
#include <Wire.h>
#include <string>

class VOCNOXSensor : public GasSensor
{
public:
  VOCNOXSensor();

  std::string getName() const override;
  std::map<std::string, SensorData> getData() override;
  void init() override;

private:
  SensirionI2CSvm41 svm41;
};

#endif