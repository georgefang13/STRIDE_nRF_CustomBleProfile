/*-----------------------------------------------------------------------------

███████╗████████╗██████╗ ██╗██████╗ ███████╗    
██╔════╝╚══██╔══╝██╔══██╗██║██╔══██╗██╔════╝    
███████╗   ██║   ██████╔╝██║██║  ██║█████╗      
╚════██║   ██║   ██╔══██╗██║██║  ██║██╔══╝      
███████║   ██║   ██║  ██║██║██████╔╝███████╗    
╚══════╝   ╚═╝   ╚═╝  ╚═╝╚═╝╚═════╝ ╚══════╝   

 @authors: Luke Andresen

 @date: 3/26/2025

 @brief: This file is the primary driver for the STRIDE platform, built on an ESP32-s3 
------------------------------------------------------------------------------*/

#include <config.h>

Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLE2901 *descriptor_2901 = NULL;

// Data Transmission bools
bool deviceConnected = false;
bool oldDeviceConnected = false;

// FSR Data Storage
uint16_t pad_data[NUM_PADS][MAX_PAD_SAMPLES]; // Store raw FSR data
uint8_t data_packet[PACKET_SIZE];             // Store processed data
uint16_t sample_index = 0;
uint16_t max_value = 1;

// IMU Data Storage
uint16_t acc_data[NUM_IMU_AXES][MAX_IMU_SAMPLES];   // Store raw IMU data (lin acc | rotation)
uint16_t imu_packet[IMU_PACKET_SIZE];
unsigned long lastTime = 0;

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
  Serial.println("Welcome to STRIDE");

  Wire1.begin(SDA, SCL, 400000); 
  Serial.println("I2C Initialized"); 

  delay(100);

  if (!bno08x.begin_I2C(0x4A, &Wire1)) {
      Serial.println("Failed to initialize BNO08x!");
      while (1);
  }
  Serial.println("BNO085 initialized!");

  set_reports();

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
    // FSR readings take place if any pad reads a value over the threshold
    if(analogRead(PAD1_PIN) > 50 || \
       analogRead(PAD2_PIN) > 50 || \
       analogRead(PAD3_PIN) > 50 || \
       analogRead(PAD4_PIN) > 50 || \
       analogRead(PAD5_PIN) > 50 || \
      /*  analogRead(PAD6_PIN) > 50 || \ */
       analogRead(PAD7_PIN) > 50 || \
       analogRead(PAD8_PIN) > 50) {
        
      collect_FSR();
      Serial.print("Max Value: ");
      Serial.println(max_value);

      // after this, we want to process_and_transmit_data with prev IMU and FSR data
      process_and_transmit_data();
    }
    // if we're not stepping, collect IMU data sequentially
    collect_IMU_datapoint();
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
    // pad_sample[5] = analogRead(PAD6_PIN);
    pad_sample[5] = 0;
    pad_sample[6] = analogRead(PAD7_PIN);
    pad_sample[7] = analogRead(PAD8_PIN);

    if(pad_sample[0] > 50 || pad_sample[1] > 50 || \
       pad_sample[2] > 50 || pad_sample[3] > 50 || \
       pad_sample[4] > 50 || pad_sample[5] > 50 || \
       pad_sample[6] > 50 || pad_sample[7] > 50){

      for(int i = 0; i < NUM_PADS; i++){
        pad_data[i][sample_index] = pad_sample[i];
        if(pad_sample[i] > max_value){
          max_value = pad_sample[i];
        }
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

void collect_IMU_datapoint() {
    // collect event datapoint and add it to acc_data[][];
    unsigned long currentTime = millis();
    float deltaT = (currentTime - lastTime) / 1000.0; 
    lastTime = currentTime;

    float ax = 0;
    float ay = 0;
    float az = 0;
    float qw = 0;
    float qx = 0;
    float qy = 0;
    float qz = 0;
    float vel_forward;
    float vel_upward;
    float vel_side;
    float pos_forward;
    float pos_upward;
    float pos_side;

    if (bno08x.getSensorEvent(&sensorValue)) {
      // get linear acceleration values
      if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
        ax = sensorValue.un.linearAcceleration.x;
        ay = sensorValue.un.linearAcceleration.y;
        az = sensorValue.un.linearAcceleration.z;
      }

      // get rotation values
      if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        qw = sensorValue.un.rotationVector.real;
        qx = sensorValue.un.rotationVector.i;
        qy = sensorValue.un.rotationVector.j;
        qz = sensorValue.un.rotationVector.k;
      }

      // calculate rotation around z-axis (this is axis into ankle; main axis of rotation)
      float theta_z = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

      // get components of acceleration to calculate forward and upward acc
      // TODO: the sign of these will need to change based on default orientation of IMU
      float a_forward = ax * cos(theta_z) + ay * sin(theta_z);
      float a_upward = -ax * sin(theta_z) + ay * cos(theta_z);
      float a_side = az;

      // integrate acc --> vel
      vel_forward += a_forward * deltaT;
      vel_upward += a_upward * deltaT;
      vel_side += a_side * deltaT;

      // integrate vel --> pos
      pos_forward += vel_forward * deltaT;
      pos_upward += vel_upward * deltaT;
      pos_side += vel_side * deltaT;

      // print final acc
      Serial.print("Upward Accel: "); Serial.print(a_upward);
      Serial.print(" | Forward Accel: "); Serial.print(a_forward);
      Serial.print(" | Sideways Accel: "); Serial.println(a_side);

      // print final position
      Serial.print("Upward Pos: "); Serial.print(pos_forward);
      Serial.print(" | Forward Pos: "); Serial.print(pos_upward);
      Serial.print(" | Sideways Pos: "); Serial.println(pos_side);
    }
  // delay by sample frequency
  delay(IMU_SAMPLE_FREQ);
}

//TODO: processing and packaging for IMU data
void process_and_transmit_data() {
  uint8_t data_indices[3] = {sample_index / 5, sample_index / 2, sample_index* 4 / 5};
  for (int i = 0; i < 24; i++) {
    uint8_t data_index = data_indices[i/8];
    uint8_t pad_idx = i % 8;  

    uint8_t scaled_value = map(pad_data[pad_idx][data_index], 0, max_value, 0, 15);

    Serial.print(pad_idx+1);
    Serial.print(": ");
    Serial.println(scaled_value);


    if (i % 2 == 0) {
        data_packet[i / 2] = (scaled_value << 4);  // Store high nibble
    } else {
        data_packet[i / 2] |= scaled_value;  // Store low nibble
    }
  }

  // Store the max value, scaled to fit in a single byte (0-255)
  data_packet[12] = map(max_value, 0, 4095, 0, 255);

  // Store IMU in data packet from 13 to 19

  // Transmit the collected data
  pCharacteristic->setValue((uint8_t *)data_packet, 13);
  pCharacteristic->notify();
  Serial.println("Data transmitted");

  // Reset sample index for next collection
  sample_index = 0;
  max_value = 1;
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

// Enable necessary sensor reports
void set_reports(void) {
    Serial.println("Setting desired reports...");
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
        Serial.println("Could not enable linear acceleration");
    }
    if (!bno08x.enableReport(SH2_STEP_COUNTER)) {
      Serial.println("Could not enable step counter");
    }
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
      Serial.println("Could not enable rotation vector");
    }
    Serial.println("Reports set");
}