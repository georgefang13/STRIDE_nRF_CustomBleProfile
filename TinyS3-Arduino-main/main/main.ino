#include "src/sensors/sensors.h"
#include "src/bluetooth/BLE.h"
#include "src/lib/esp32s3-arduino-helper-1.0.1/src/UMS3.h"
#include <string>
#include <vector>
#include <memory>
#include <map>

#define BLE_DELAY 5000
#define SENSOR_DELAY 16000
#define BATTERY_MAX 4.20 // maximum voltage of battery
#define BATTERY_MIN 3.4  // minimum voltage of battery before shutdown

std::vector<std::unique_ptr<GasSensor>> sensors;
std::map<std::string, SensorData> allSensorData;
int MAX_LOOP_COUNT = SENSOR_DELAY / BLE_DELAY;
int loopCount = MAX_LOOP_COUNT;
BLE *ble;
UMS3 ums3;

template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args &&...args)
{
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

float getBatteryPercent()
{
  float batteryVoltage = ums3.getBatteryVoltage();

  // round value by two precision
  float voltage = roundf(batteryVoltage * 100) / 100;
  float output = ((voltage - BATTERY_MIN) / (BATTERY_MAX - BATTERY_MIN)) * 100;
  if (output < 100)
    return output;
  else
    return 100.0f;
}

std::map<std::string, SensorData> getAllSensorData()
{
  float tempAvg = 0;
  float humdAvg = 0;
  int tempCount = 0;
  int humdCount = 0;
  std::map<std::string, SensorData> allSensorData;
  for (const auto &sensor : sensors)
  {
    if (!sensor->isInitialized())
    {
      Serial.println(String(sensor->getName().c_str()) + " not initialized, trying again...");
      sensor->init();
      continue;
    }
    std::map<std::string, SensorData> sensorData = sensor->getData();
    for (const auto &data : sensorData)
    {
      if (data.first == "Temperature" && data.second.value > NEGATIVE_INFINITY)
      {
        Serial.println(String(sensor->getName().c_str()) + " Temp: " + String(data.second.value) + data.second.units.c_str());
        tempAvg += data.second.value;
        tempCount++;
      }
      else if (data.first == "Humidity" && data.second.value > NEGATIVE_INFINITY)
      {
        humdAvg += data.second.value;
        humdCount++;
      }
      else
      {
        allSensorData[data.first] = data.second;
      }
    }
  }
  if (tempCount > 0)
  {
    allSensorData["Temperature"] = SensorData(tempAvg / tempCount, "C");
    Serial.println("Average Temp: " + String(allSensorData["Temperature"].value) + allSensorData["Temperature"].units.c_str());
  }
  if (humdCount > 0)
  {
    allSensorData["Humidity"] = SensorData(humdAvg / humdCount, "%");
    Serial.println("Average Humd: " + String(allSensorData["Humidity"].value) + allSensorData["Humidity"].units.c_str());
  }

  allSensorData["Battery"] = SensorData(getBatteryPercent(), "%");
  return allSensorData;
}

void setup()
{
  Serial.begin(115200);

  ums3.begin();

  ble = new BLE();

  delay(20); // Ensure all sensors are powered

  sensors.push_back(make_unique<CO2Sensor>());
  sensors.push_back(make_unique<COSensor>());
  sensors.push_back(make_unique<O3Sensor>());
  sensors.push_back(make_unique<UVSensor>());
  sensors.push_back(make_unique<VOCNOXSensor>());
  sensors.push_back(make_unique<PMSensor>());

  Serial.println("All Sensors Initialized!");

  delay(5000); // Allow sensors to acclimate
}

void loop()
{
  if (loopCount == MAX_LOOP_COUNT)
  {
    allSensorData = getAllSensorData();
    loopCount = 0;
  }
  ble->send_data(allSensorData);
  loopCount++;
  delay(BLE_DELAY);
}