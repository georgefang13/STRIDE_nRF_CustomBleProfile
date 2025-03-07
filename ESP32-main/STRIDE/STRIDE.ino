/*-----------------------------------------------------------------------------

███████╗████████╗██████╗ ██╗██████╗ ███████╗    
██╔════╝╚══██╔══╝██╔══██╗██║██╔══██╗██╔════╝    
███████╗   ██║   ██████╔╝██║██║  ██║█████╗      
╚════██║   ██║   ██╔══██╗██║██║  ██║██╔══╝      
███████║   ██║   ██║  ██║██║██████╔╝███████╗    
╚══════╝   ╚═╝   ╚═╝  ╚═╝╚═╝╚═════╝ ╚══════╝   

 @authors: Luke Andresen

 @date: 3/7/2025

 @brief: This file is the primary driver for the STRIDE platform, built on an ESP32-s3 
------------------------------------------------------------------------------*/

#include <config.h>

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLE2901 *descriptor_2901 = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

// FSR Data Storage
uint16_t pad_data[NUM_PADS][MAX_PAD_SAMPLES]; // Store raw FSR data
uint16_t selected_data[NUM_PADS * TRANSMIT_POINTS];    // Store downsampled data
uint8_t sample_index = 0;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

void setup() {
  Serial.begin(115200);
  
  // Set pins
  pinMode(PAD1_PIN, INPUT);
  pinMode(PAD2_PIN, INPUT);
  pinMode(PAD3_PIN, INPUT);
  pinMode(PAD4_PIN, INPUT);
  pinMode(PAD5_PIN, INPUT);
  pinMode(PAD6_PIN, INPUT);
  pinMode(PAD7_PIN, INPUT);
  pinMode(PAD8_PIN, INPUT);

  // Set up BLE and begin advertisement
  BLE_init();

}

void loop() {
  
  // notify changed value
  if (deviceConnected) {
    if(analogRead(PAD1_PIN) > 50 || \
       analogRead(PAD2_PIN) > 50 || \
       analogRead(PAD3_PIN) > 50 || \
       analogRead(PAD4_PIN) > 50 || \
       analogRead(PAD5_PIN) > 50 || \
       analogRead(PAD6_PIN) > 50 || \
       analogRead(PAD7_PIN) > 50 || \
       analogRead(PAD8_PIN) > 50){
    
      collect_FSR();
      if(sample_index > TRANSMIT_POINTS){
        process_and_transmit_data();
      }
    }
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}

void collect_FSR(){
  bool collecting = true;
  uint16_t pad_sample[NUM_PADS];
  while(collecting){
    pad_sample[0] = analogRead(PAD1_PIN);
    pad_sample[1] = analogRead(PAD2_PIN);
    pad_sample[2] = analogRead(PAD3_PIN);
    pad_sample[3] = analogRead(PAD4_PIN);
    pad_sample[4] = analogRead(PAD5_PIN);
    pad_sample[5] = analogRead(PAD6_PIN);
    pad_sample[6] = analogRead(PAD7_PIN);
    pad_sample[7] = analogRead(PAD8_PIN);

    if(pad_sample[0] > 50 || pad_sample[1] > 50 || \
       pad_sample[2] > 50 || pad_sample[3] > 50 || \
       pad_sample[4] > 50 || pad_sample[5] > 50 || \
       pad_sample[6] > 50 || pad_sample[2] > 50){

      for(int i = 0; i < NUM_PADS; i++){
        // Serial.print(i+1); Serial.print(": ");
        // Serial.println(pad_sample[i]);
        pad_data[i][sample_index] = pad_sample[i];
      }

      sample_index++;

      if(sample_index >= MAX_PAD_SAMPLES){
        collecting = false;
      }
    }
    else{
      collecting = false;
    }
    delay(10);
  }
}


void process_and_transmit_data() {
  // Calculate step size based on the total number of samples collected
  int step_size = sample_index / TRANSMIT_POINTS;
  for (int i = 0; i < TRANSMIT_POINTS; i++) {
    int sample_index_for_point = i * step_size;
    for (int j = 0; j < NUM_PADS; j++) {
      // Select the sample based on the step size
      selected_data[i*NUM_PADS+j] = pad_data[j][sample_index_for_point];
    }
  }
  sample_index = 0;
  
  // Split the selected data into 8 parts (each part will have 160/8 = 20 bytes)
  uint8_t chunk_size = sizeof(selected_data) / 4;
  for (int chunk = 0; chunk < 4; chunk++) {
    // Define the chunk data
    uint8_t chunk_data[chunk_size];
    memcpy(chunk_data, selected_data + chunk * chunk_size, chunk_size);

    // Transmit the chunk of data
    pCharacteristic->setValue(chunk_data, chunk_size);
    pCharacteristic->notify();
    Serial.println("Data chunk transmitted");
  }
  Serial.println("All data chunks transmitted");
}


void BLE_init(){
  
  // Create the BLE Device
  BLEDevice::init(DEVICE_NAME);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE
  );

  // Creates BLE Descriptor 0x2902: Client Characteristic Configuration Descriptor (CCCD)
  pCharacteristic->addDescriptor(new BLE2902());
  // Adds also the Characteristic User Description - 0x2901 descriptor
  descriptor_2901 = new BLE2901();
  descriptor_2901->setDescription(CHARACTERISTIC_DESC);
  descriptor_2901->setAccessPermissions(ESP_GATT_PERM_READ);  // enforce read only - default is Read|Write
  pCharacteristic->addDescriptor(descriptor_2901);

  // Start the service
  pService->start();

  // **Optimize BLE settings**
  BLEDevice::setMTU(23);  // Increase MTU for larger payloads (ESP32 max is 517)
  
  // Adjust advertising settings for fast connection
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x00);  // Lower = faster connections
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}
