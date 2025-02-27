#include "CO2.h"

CO2Sensor::CO2Sensor()
    : GasSensor()
{
  init();
}

std::string CO2Sensor::getName() const
{
  return "CO2";
}

std::map<std::string, SensorData> CO2Sensor::getData()
{
  safeRead();
  return data;
}

bool CO2Sensor::checkDataReady()
{
  char errorMessage[256];
  bool isDataReady = false;
  uint16_t e = scd4x.getDataReadyFlag(isDataReady);
  if (e)
  {
    Serial.print("Error trying to execute getDataReadyFlag(): ");
    errorToString(e, errorMessage, 256);
    Serial.println(errorMessage);
    return false;
  }
  return isDataReady;
}

void CO2Sensor::safeRead()
{
  char errorMessage[256];
  uint16_t error;
  uint16_t co2;
  float temperature;
  float humidity;
  if (checkDataReady())
  {
    error = scd4x.readMeasurement(co2, temperature, humidity);
    if (error)
    {
      Serial.print("Error trying to execute readMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
    }
    else if (co2 <= 0)
    {
      Serial.println("Invalid sample detected, skipping.");
    }
    else
    {
      data["CO2"] = SensorData(co2, "ppm");
      data["Temperature"] = SensorData(temperature, "C");
      data["Humidity"] = SensorData(humidity, "%");
    }
  }
  else
  {
    Serial.println("CO2 Data not ready, skipping.");
  }
}

void CO2Sensor::printUint16Hex(uint16_t value)
{
  Serial.print(value < 4096 ? "0" : "");
  Serial.print(value < 256 ? "0" : "");
  Serial.print(value < 16 ? "0" : "");
  Serial.print(value, HEX);
}

void CO2Sensor::printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2)
{
  Serial.print("Serial: 0x");
  printUint16Hex(serial0);
  printUint16Hex(serial1);
  printUint16Hex(serial2);
  Serial.println();
}

void CO2Sensor::init()
{
  Wire.begin();

  uint16_t error;
  char errorMessage[256];

  scd4x.begin(Wire);

  // stop potentially previously started measurement
  error = scd4x.stopPeriodicMeasurement();
  if (error)
  {
    Serial.println("CO2 not found!");
    // Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
    // errorToString(error, errorMessage, 256);
    // Serial.println(errorMessage);
    return;
  }

  uint16_t serial0;
  uint16_t serial1;
  uint16_t serial2;
  error = scd4x.getSerialNumber(serial0, serial1, serial2);
  if (error)
  {
    Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    return;
  }
  else
  {
    Serial.println("SCD4x connected!");
    printSerialNumber(serial0, serial1, serial2);
  }
  error = scd4x.startLowPowerPeriodicMeasurement();
  if (error)
  {
    Serial.print("Error trying to execute startLowPowerPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    return;
  }
  initialized = true;
}