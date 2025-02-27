#include "O3.h"

O3Sensor::O3Sensor()
    : GasSensor()
{
  init();
}

std::string O3Sensor::getName() const
{
  return "O3";
}

std::map<std::string, SensorData> O3Sensor::getData()
{
  data["O3"] = SensorData(Ozone.readOzoneData(COLLECT_NUMBER), "ppb");
  return data;
}

void O3Sensor::init()
{
  delay(250);
  if (!Ozone.begin(Ozone_IICAddress))
  {
    Serial.println("O3 not found!");
    return;
  }
  Serial.println("O3 connected!");
  Ozone.setModes(MEASURE_MODE_PASSIVE);
  initialized = true;
}