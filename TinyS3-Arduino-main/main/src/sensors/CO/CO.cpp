#include "CO.h"

void COSensor::init()
{
  gas = DFRobot_GAS_I2C(&Wire, I2C_ADDRESS);
  delay(500);
  if (!gas.begin())
  {
    Serial.println("CO not found!");
    return;
  }
  Serial.println("CO connected!");

  initialized = true;

  gas.changeAcquireMode(gas.PASSIVITY);
  delay(500);
  gas.setTempCompensation(gas.OFF);
}

COSensor::COSensor()
    : GasSensor()
{
  init();
}

std::string COSensor::getName() const
{
  return "CO";
}

std::map<std::string, SensorData> COSensor::getData()
{
  if (gas.dataIsAvailable())
  {
    data["Temperature"] = SensorData(gas.readTempC(), "C");
    data["CO"] = SensorData(gas.readGasConcentrationPPM(), "ppm");
  }
  else
  {
    Serial.println("CO data not available");
  }
  if (data["CO"].value < 0)
  {
    Serial.println("CO data not available; bad reading\n\n");
  }
  return data;
}