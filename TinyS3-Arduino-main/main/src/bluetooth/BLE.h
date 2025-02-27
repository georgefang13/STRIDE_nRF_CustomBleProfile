#ifndef BLE_H
#define BLE_H

#include <Arduino.h>
#include "UUIDs.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "../sensors/sensorData.h"
#include <map>
#include <string>

#define DEVICE_NAME "TinyS3 Arduino"

using UUID = std::string;
using SENSOR_NAME = std::string;

void _start_advertising();

extern std::map<SENSOR_NAME, UUID> sensor_uuid_map;

class CustomServerCallbacks : public BLEServerCallbacks
{
public:
  void onConnect(BLEServer *pServer);
  void onDisconnect(BLEServer *pServer);
};

class BLE
{
public:
  BLE();
  BLECharacteristic *get_characteristic(std::string name);
  void send_data(std::map<std::string, SensorData> data);

private:
  std::map<UUID, BLECharacteristic *> characteristic_map;

  void _init_characteristics(BLEService *pService);
  BLECharacteristic *create_characteristic(std::string sensor_name, BLEService *pService);
};

#endif // BLE_H