#include "PM.h"

PMSensor::PMSensor()
    : GasSensor()
{
  init();
}

std::string PMSensor::getName() const
{
  return "PM2.5";
}

std::map<std::string, SensorData> PMSensor::getData()
{
  PM25_AQI_Data pmData;

  if (!aqi.read(&pmData))
  {
    Serial.println("Could not read from AQI");
    delay(50); // try again in a bit!
    return data;
  }
  data["PM1.0"] = SensorData(pmData.pm10_env, "ug/m3");
  data["PM2.5"] = SensorData(pmData.pm25_env, "ug/m3");
  data["PM10"] = SensorData(pmData.pm100_env, "ug/m3");
  return data;
}

void PMSensor::init()
{
  aqi = Adafruit_PM25AQI();
  delay(250);
  if (!aqi.begin_I2C())
  {
    Serial.println("PM2.5 not found!");
    return;
  }

  data["PM1.0"] = SensorData("ug/m3");
  data["PM2.5"] = SensorData("ug/m3");
  data["PM10"] = SensorData("ug/m3");

  Serial.println("PM2.5 connected!");
  initialized = true;
}