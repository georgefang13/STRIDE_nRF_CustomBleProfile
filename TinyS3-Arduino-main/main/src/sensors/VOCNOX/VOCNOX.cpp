#include "VOCNOX.h"

VOCNOXSensor::VOCNOXSensor()
    : GasSensor()
{
  init();
}

std::string VOCNOXSensor::getName() const
{
  return "SGP41";
}

std::map<std::string, SensorData> VOCNOXSensor::getData()
{
  svm41.startMeasurement();
  delay(55); // Delay to let the sensor take a reading
  uint16_t error;
  char errorMessage[256];

  // Read Measurement
  float humidity;
  float temperature;
  float vocIndex;
  float noxIndex;
  error = svm41.readMeasuredValues(humidity, temperature, vocIndex, noxIndex);
  if (error)
  {
    Serial.print("Error trying to execute readMeasuredValues(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  else
  {
    if (temperature == 0)
    {
      Serial.println("Temperature is 0?!?!?!");
    }
    if (temperature > 0 && temperature < 100)
    {
      data["Temperature"] = SensorData(temperature, "C");
    }
    if (humidity > 0 && humidity < 100)
    {
      data["Humidity"] = SensorData(humidity, "%");
    }
    data["VOC"] = SensorData(vocIndex, "ppb");
    data["NOx"] = SensorData(noxIndex, "ppb");
  }

  return data;
}

void VOCNOXSensor::init()
{
  Wire.begin();

  uint16_t error;
  char errorMessage[256];

  svm41.begin(Wire);

  error = svm41.deviceReset();
  if (error)
  {
    Serial.println("SGP41 not found!");
    // Serial.print("Error trying to execute deviceReset(): ");
    // errorToString(error, errorMessage, 256);
    // Serial.println(errorMessage);
    return;
  }

  // Delay to let the serial monitor catch up
  delay(500);

  uint8_t serialNumber[32];
  uint8_t serialNumberSize = 32;
  error = svm41.getSerialNumber(serialNumber, serialNumberSize);
  if (error)
  {
    Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    return;
  }
  else
  {
    Serial.print("SGP41 SerialNumber:");
    Serial.println((char *)serialNumber);
  }

  uint8_t firmwareMajor;
  uint8_t firmwareMinor;
  bool firmwareDebug;
  uint8_t hardwareMajor;
  uint8_t hardwareMinor;
  uint8_t protocolMajor;
  uint8_t protocolMinor;
  error = svm41.getVersion(firmwareMajor, firmwareMinor, firmwareDebug,
                           hardwareMajor, hardwareMinor, protocolMajor,
                           protocolMinor);

  if (error)
  {
    Serial.print("Error trying to execute getVersion(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    return;
  }
  else
  {
    Serial.print("Firmware version: ");
    Serial.print(firmwareMajor);
    Serial.print(".");
    Serial.print(firmwareMinor);
    Serial.print("\t");
    Serial.print("FirmwareDebug: ");
    Serial.println(firmwareDebug);
    Serial.print("Hardware version: ");
    Serial.print(hardwareMajor);
    Serial.print(".");
    Serial.println(hardwareMinor);
    Serial.print("Protocol version: ");
    Serial.print(protocolMajor);
    Serial.print(".");
    Serial.print(protocolMinor);
    Serial.println();
  }
  data["VOC"] = SensorData("ppb");
  data["NOx"] = SensorData("ppb");
  data["Temperature"] = SensorData("C");
  data["Humidity"] = SensorData("%");

  Serial.println("SGP41 connected!");
  initialized = true;
}