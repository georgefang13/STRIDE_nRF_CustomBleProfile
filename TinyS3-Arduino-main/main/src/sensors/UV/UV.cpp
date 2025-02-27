#include "UV.h"

UVSensor::UVSensor()
    : GasSensor()
{
  init();
}

std::string UVSensor::getName() const
{
  return "UV";
}

std::map<std::string, SensorData> UVSensor::getData()
{
  if (!ltr.newDataAvailable())
  {
    Serial.println("No new UV data available");
    return data;
  }
  float uvReading = ltr.readUVS();
  float uvIndex = (uvReading / 2300 * 1.0) * (18 / ltr.getGain());
  data["UV"] = SensorData(uvReading, "UVI");
  return data;
}

void UVSensor::init()
{
  ltr = Adafruit_LTR390();

  delay(250);

  if (!ltr.begin())
  {
    Serial.println("UV not found!");
    return;
  }
  Serial.println("UV connected!");

  ltr.setMode(LTR390_MODE_UVS);
  ltr.setGain(LTR390_GAIN_3);
  if (ltr.getGain() != LTR390_GAIN_3)
  {
    Serial.println("\n\n\n\n\nUV gain not set!\n\n\n\n\n");
    return;
  }
  ltr.setResolution(LTR390_RESOLUTION_18BIT);
  ltr.configInterrupt(true, LTR390_MODE_UVS);

  data["UV"] = SensorData("UVI");
  initialized = true;
}