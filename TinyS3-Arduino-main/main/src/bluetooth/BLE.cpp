#include "BLE.h"

std::map<SENSOR_NAME, UUID> sensor_uuid_map = {
    {"CO", CHARACTERISTIC_CO_UUID},
    {"CO2", CHARACTERISTIC_CO2_UUID},
    {"UV", CHARACTERISTIC_UV_UUID},
    {"VOC", CHARACTERISTIC_VOC_UUID},
    {"O3", CHARACTERISTIC_O3_UUID},
    {"Temperature", CHARACTERISTIC_TEMP_UUID},
    {"Humidity", CHARACTERISTIC_HUMD_UUID},
    {"PM2.5", CHARACTERISTIC_PM25_UUID},
    {"PM10", CHARACTERISTIC_PM100_UUID},
    {"PM1.0", CHARACTERISTIC_PM10_UUID},
    {"NOx", CHARACTERISTIC_NOX_UUID},
    {"Battery", CHARACTERISTIC_BATTERY_PERCENT_UUID}};

void _start_advertising()
{
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

void CustomServerCallbacks::onConnect(BLEServer *pServer)
{
  Serial.println("Client connected");
}

void CustomServerCallbacks::onDisconnect(BLEServer *pServer)
{
  BLEDevice::stopAdvertising();
  Serial.println("Client disconnected");
  delay(800); // give the client time to get the last notification
  _start_advertising();
  Serial.println("Advertising started");
}

BLE::BLE()
{
  BLEDevice::init(DEVICE_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  CustomServerCallbacks *myServerCallbacks = new CustomServerCallbacks();
  pServer->setCallbacks(myServerCallbacks);
  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID), 40);
  _init_characteristics(pService);
  pService->start();
  _start_advertising();
}

BLECharacteristic *BLE::get_characteristic(std::string name)
{
  UUID uuid = sensor_uuid_map[name];
  return characteristic_map[uuid];
}

void BLE::_init_characteristics(BLEService *pService)
{
  std::vector<std::string> sensor_names;
  for (const auto &pair : sensor_uuid_map)
  {
    sensor_names.push_back(pair.first);
  }
  for (auto sensor : sensor_names)
  {
    UUID sensor_uuid = sensor_uuid_map[sensor];
    BLECharacteristic *characteristic = create_characteristic(sensor, pService);
    characteristic_map[sensor_uuid] = characteristic;
    Serial.println("Initialized characteristic for " + String(sensor.c_str()) + " with UUID: " + sensor_uuid.c_str());
  }
}

BLECharacteristic *BLE::create_characteristic(std::string sensor_name, BLEService *pService)
{
  UUID uuid = sensor_uuid_map[sensor_name];
  BLECharacteristic *characteristic = pService->createCharacteristic(uuid, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  BLEUUID cccdUUID(BLEUUID((uint16_t)0x2902)); // Standard UUID for CCCD
  BLEDescriptor *pDescriptor = new BLEDescriptor(cccdUUID);
  pDescriptor->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
  characteristic->addDescriptor(pDescriptor);
  float zero = 0.0;
  byte bytes[sizeof(float)];
  memcpy(bytes, &zero, sizeof(float));
  characteristic->setValue(bytes, sizeof(bytes));
  return characteristic;
}

void BLE::send_data(std::map<std::string, SensorData> data)
{
  for (const auto &data : data)
  {
    // Convert to list of bytes
    byte bytes[sizeof(float)];
    memcpy(bytes, &data.second.value, sizeof(float));

    // Set the characteristic value and notify
    BLECharacteristic *characteristic = get_characteristic(data.first);

    if (characteristic == nullptr)
    {
      Serial.println("Characteristic " + String(data.first.c_str()) + " not found");
    }
    else
    {
      // Serial.println("Sending data for " + String(data.first.c_str()) + ": " + String(data.second.value) + data.second.units.c_str() + " on UUID: " + characteristic->getUUID().toString().c_str());
      characteristic->setValue(bytes, sizeof(bytes));
      characteristic->notify();
      delay(10);
    }
  }
}