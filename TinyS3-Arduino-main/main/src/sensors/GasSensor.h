#ifndef GASSENSOR_H
#define GASSENSOR_H

#include <string>
#include <limits>
#include <memory>
#include <map>
#include "sensorData.h"
#include "../bluetooth/BLE.h"

#define NEGATIVE_INFINITY (-std::numeric_limits<float>::max())

class GasSensor
{
public:
  GasSensor(){};
  ~GasSensor(){};

  virtual std::string getName() const = 0;
  virtual std::map<std::string, SensorData> getData() = 0;
  virtual void init() = 0;

  bool isInitialized() const { return initialized; }

protected:
  std::map<std::string, SensorData> data;
  bool initialized = false;
};

#endif