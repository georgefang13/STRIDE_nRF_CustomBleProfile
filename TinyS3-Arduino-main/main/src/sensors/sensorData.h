#include <limits>

#define NEGATIVE_INFINITY (-std::numeric_limits<float>::max())

#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

struct SensorData
{
  float value;
  std::string units;

  SensorData(float value = NEGATIVE_INFINITY, std::string units = "")
      : value(value), units(units) {}
  SensorData(const SensorData &data)
      : value(data.value), units(data.units) {}
  SensorData(std::string units) : value(NEGATIVE_INFINITY), units(units) {}
};

#endif // SENSOR_DATA_H