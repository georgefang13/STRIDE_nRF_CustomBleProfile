/*-----------------------------------------------------------------------------

███████╗████████╗██████╗ ██╗██████╗ ███████╗    
██╔════╝╚══██╔══╝██╔══██╗██║██╔══██╗██╔════╝    
███████╗   ██║   ██████╔╝██║██║  ██║█████╗      
╚════██║   ██║   ██╔══██╗██║██║  ██║██╔══╝      
███████║   ██║   ██║  ██║██║██████╔╝███████╗    
╚══════╝   ╚═╝   ╚═╝  ╚═╝╚═╝╚═════╝ ╚══════╝   

 @authors: Luke Andresen, Gavin Kitch

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
uint16_t fsr_sample_index = 0;
uint16_t max_value = 1;

// IMU Data Storage
float acc_data[MAX_IMU_SAMPLES][NUM_IMU_AXES];   // Store unfiltered IMU data
float pos_data[MAX_IMU_SAMPLES][3];              // 3 axis integrated acc data
uint16_t imu_sample_index = 0;
unsigned long lastTime = 0;
float forward_offset = 0;
float upward_offset = 0;
// temp storage as global variable—avoid local ovf
float temp[MAX_IMU_SAMPLES][3];
int temp_indices[MAX_IMU_SAMPLES];
float filtered_data[MAX_IMU_SAMPLES][3];

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
  delay(100);
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
       analogRead(PAD6_PIN) > 50 || \
       analogRead(PAD7_PIN) > 50 || \
       analogRead(PAD8_PIN) > 50) {
        
      collect_FSR();

      // Serial.print("Max Value: ");
      // Serial.println(max_value);

      // after this, we want to process_and_transmit_data with prev IMU and FSR data
      process_and_transmit_data();
    }
    // if we're not stepping, collect IMU data sequentially
    if (imu_sample_index < MAX_IMU_SAMPLES) {
      collect_IMU_datapoint();
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
    pad_sample[5] = 0;
    pad_sample[6] = analogRead(PAD7_PIN);
    pad_sample[7] = analogRead(PAD8_PIN);

    if(pad_sample[0] > 50 || pad_sample[1] > 50 || \
       pad_sample[2] > 50 || pad_sample[3] > 50 || \
       pad_sample[4] > 50 || pad_sample[5] > 50 || \
       pad_sample[6] > 50 || pad_sample[7] > 50){

      for(int i = 0; i < NUM_PADS; i++){
        pad_data[i][fsr_sample_index] = pad_sample[i];
        if(pad_sample[i] > max_value){
          max_value = pad_sample[i];
        }
      }

      fsr_sample_index++;

      if(fsr_sample_index >= MAX_PAD_SAMPLES){
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

    float ax = 0;
    float ay = 0;
    float az = 0;
    float qw = 0;
    float qx = 0;
    float qy = 0;
    float qz = 0;

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
      // float theta_z = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

      // get components of acceleration to calculate forward and upward acc
      // default orientation: x is forward (+ rot), y is upward (+ rot), z is into ankle from exterior (axis of rotation)
      // float a_forward = -1 * ax * cos(theta_z) + ay * sin(theta_z);
      // float a_upward = -1 * ax * sin(theta_z) + -1 * ay * cos(theta_z);
      float a_side = az;

      float theta_x = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
      float a_forward = -1 * ax * cos(theta_x) + ay * sin(theta_x);
      float a_upward = -1 * (ax * sin(theta_x) + ay * cos(theta_x));

      // store acc value
      acc_data[imu_sample_index][0] = a_forward;
      acc_data[imu_sample_index][1] = a_upward;
      acc_data[imu_sample_index][2] = a_side;
      imu_sample_index++;
    }
  // delay by sample frequency
  delay(IMU_SAMPLE_FREQ);
}

// FILTERS FOR IMU DATA SMOOTHING
// Low-pass filter helper (exponential smoothing)
float alpha = 0.6; // tune between 0 (no change) and 1 (very fast)
float lowPassFilter(float current, float previous) {
    return alpha * current + (1 - alpha) * previous;
}

// High-pass filter
float highPassFilter(float current_vel, float prev_accel, float current_accel, float alpha) {
    // alpha should be between 0 (no filtering) and 1 (very aggressive filtering)
    return alpha * (current_vel + current_accel - prev_accel);
}


//TODO: processing and packaging for IMU data
void process_and_transmit_data() {
  Serial.println("Processing and Transmitting data...");
  delay(100);
  uint8_t data_indices[3] = {fsr_sample_index / 5, fsr_sample_index / 2, fsr_sample_index* 4 / 5};
  for (int i = 0; i < 24; i++) {
    uint8_t data_index = data_indices[i/8];
    uint8_t pad_idx = i % 8;  

    uint8_t scaled_value = map(pad_data[pad_idx][data_index], 0, max_value, 0, 15);

    if (i % 2 == 0) {
        data_packet[i / 2] = (scaled_value << 4);  // Store high nibble
    } else {
        data_packet[i / 2] |= scaled_value;  // Store low nibble
    }
  }

  // Store the max value, scaled to fit in a single byte (0-255)
  data_packet[12] = map(max_value, 0, 4095, 0, 255);

  // IMU PROCESSING
  // 1. Apply low-pass filter to raw data
  filtered_data[0][0] = acc_data[0][0];
  filtered_data[0][1] = acc_data[0][1];
  filtered_data[0][2] = acc_data[0][2];
  for (int i = 1; i < imu_sample_index; i++) {
      filtered_data[i][0] = lowPassFilter(acc_data[i][0], filtered_data[i - 1][0]);
      filtered_data[i][1] = lowPassFilter(acc_data[i][1], filtered_data[i - 1][1]);
      filtered_data[i][2] = lowPassFilter(acc_data[i][2], filtered_data[i - 1][2]);
  }

  // 2. Threshold filtered acceleration
  int pos_idx = 0;
  float thresh = 0.05;

  for (int i = 0; i < imu_sample_index && pos_idx < MAX_IMU_SAMPLES; i++) {
      float a_forward = filtered_data[i][0];
      float a_upward = filtered_data[i][1];
      float a_side = filtered_data[i][2];

      if (fabs(a_forward) > thresh || fabs(a_upward) > thresh || fabs(a_side) > thresh) {
          temp[pos_idx][0] = a_forward;
          temp[pos_idx][1] = a_upward;
          temp[pos_idx][2] = a_side;
          temp_indices[pos_idx] = i;
          pos_idx++;
      }
  }

  // 3. Integration
  float forward_pos = 0, upward_pos = 0, side_pos = 0;
  float forward_vel = 0, upward_vel = 0, side_vel = 0;
  float prev_ax = 0, prev_ay = 0, prev_az = 0;
  float dt = 0;
  int prev_index = 0;

  float alpha = 0.9; // adjust between 0.8–0.98 based on filtering strength

  // track max values
  int minX_idx = 0;
  int maxY_idx = 0;
  int maxX_idx = pos_idx-1;

  if (pos_idx >= MAX_IMU_SAMPLES) {
      Serial.println("⚠️ pos_idx reached MAX_IMU_SAMPLES — data may be truncated.");  // --- FIXED
  }
  
  for (int j = 0; j < pos_idx && j < MAX_IMU_SAMPLES; j++) {
      if (pos_idx <= 0 || pos_idx > MAX_IMU_SAMPLES) {
        Serial.println("Invalid pos_idx, aborting transmission.");
        return;
      }

      int cur_index = temp_indices[j];
      dt = (cur_index - prev_index) * IMU_SAMPLE_FREQ * 0.001; // convert ms to s
      prev_index = cur_index;

      float a_forward = temp[j][0];
      float a_upward = temp[j][1];
      float a_side = temp[j][2];

      // Integrate acceleration -> velocity
      forward_vel += a_forward * dt;
      upward_vel += a_upward * dt;
      side_vel += a_side * dt;

      // Apply high-pass filter to remove drift in velocity
      forward_vel = highPassFilter(forward_vel, prev_ax, a_forward, alpha);
      upward_vel = highPassFilter(upward_vel, prev_ay, a_upward, alpha);
      side_vel = highPassFilter(side_vel, prev_az, a_side, alpha);

      // Save current accel for next high-pass filter round
      prev_ax = a_forward;
      prev_ay = a_upward;
      prev_az = a_side;

      // Integrate velocity -> position
      forward_pos += forward_vel * dt;
      upward_pos += upward_vel * dt;
      side_pos += side_vel * dt;

      // TODO: POSSIBLY NEED CLAMPING OF J HERE
      pos_data[j][0] = forward_pos;
      pos_data[j][1] = upward_pos;
      pos_data[j][2] = side_pos;

      // update max indices
      if (forward_pos < pos_data[minX_idx][0]) {
        minX_idx = j;
      }
      if (forward_pos > pos_data[maxX_idx][0]) {
        maxX_idx = j;
      }
      if (upward_pos > pos_data[maxY_idx][1]) {
        maxY_idx = j;
      }

      // Serial.printf("(Data point %d - Forward: %f", j, forward_pos);
      // Serial.printf(" | Upward: %f", upward_pos);
      // Serial.printf(" | Side: %f", side_pos);
      // Serial.println();
  }

  // package [ {MIN_X, MAX_Y, 75th, MAX_X} | MAX ] into top 7 bytes
  // float minX_x = 0, maxY_x = 0, midstep_x = 0, maxX_x = 0;
  // float minX_y = 0, maxY_y = 0, midstep_y = 0;
  
  // TODO: verify this index
  // int midstep_idx = (int) round(3 * pos_idx / 4);
  int midstep_idx = min((int) round(4 * pos_idx / 5), MAX_IMU_SAMPLES - 1);  // --- FIXED


  // calculating 3 point rolling average w/ out of bounds checks
  // if (minX_idx > 0 && minX_idx < pos_idx) {
  //     minX_x = abs((pos_data[minX_idx][0] + pos_data[minX_idx - 1][0] + pos_data[minX_idx + 1][0]) / 3);
  //     minX_y = (pos_data[minX_idx][1] + pos_data[minX_idx - 1][1] + pos_data[minX_idx + 1][1]) / 3;
  // }
  // else {
  //     minX_x = abs(pos_data[minX_idx][0]);
  //     minX_y = pos_data[minX_idx][1];
  // }
  // if (maxY_idx > 0 && maxY_idx < pos_idx) {
  //     maxY_x = (pos_data[maxY_idx][0] + pos_data[maxY_idx - 1][0] + pos_data[maxY_idx + 1][0]) / 3;
  //     maxY_y = (pos_data[maxY_idx][1] + pos_data[maxY_idx - 1][1] + pos_data[maxY_idx + 1][1]) / 3;
  // }
  // else {
  //     maxY_x = pos_data[maxY_idx][0];
  //     maxY_y = pos_data[maxY_idx][1];
  // }
  // if (midstep_idx > 0 && midstep_idx < pos_idx) {
  //     midstep_x = (pos_data[midstep_idx][0] + pos_data[midstep_idx - 1][0] + pos_data[midstep_idx + 1][0]) / 3;
  //     midstep_y = (pos_data[midstep_idx][1] + pos_data[midstep_idx - 1][1] + pos_data[midstep_idx + 1][1]) / 3;
  // }
  // else {
  //     midstep_x = pos_data[midstep_idx][0];
  //     midstep_y = pos_data[midstep_idx][1];
  // }
  // // implied y value of 0
  // if (maxX_idx > 0 && maxX_idx < pos_idx) {
  //     maxX_x = (pos_data[maxX_idx][0] + pos_data[maxX_idx - 1][0] + pos_data[maxX_idx + 1][0]) / 3;
  // }
  // else {
  //     maxX_x = pos_data[maxX_idx][0];
  // }

  int8_t minX_x = min(pos_data[minX_idx][0] * IMU_SCALE + IMU_ADJ, MAX_IMU_VALUE);
  int8_t minX_y = min(pos_data[minX_idx][1] * IMU_SCALE + IMU_ADJ, MAX_IMU_VALUE);
  int8_t maxY_x = min(pos_data[maxY_idx][0] * IMU_SCALE + IMU_ADJ, MAX_IMU_VALUE);
  int8_t maxY_y = min(pos_data[maxY_idx][1] * IMU_SCALE + IMU_ADJ, MAX_IMU_VALUE);
  int8_t midstep_x = min(pos_data[midstep_idx][0] * IMU_SCALE + IMU_ADJ, MAX_IMU_VALUE);
  int8_t midstep_y = min(pos_data[midstep_idx][1] * IMU_SCALE + IMU_ADJ, MAX_IMU_VALUE);
  int8_t maxX_x = min(pos_data[maxX_idx][0] * IMU_SCALE + IMU_ADJ, MAX_IMU_VALUE);

  // forward_offset = pos_data[0][0];
  // upward_offset = pos_data[0][1];
  // Serial.printf("Forward Offset: %f | Upward Offset : %f", forward_offset, upward_offset);
  // Serial.println("");

  // print out all the values
  Serial.println("POINTS TO TRANSMIT:");
  Serial.printf("Min X: (%d, %d)", minX_x, minX_y); Serial.println("");
  Serial.printf("Max Y: (%d, %d)", maxY_x, maxY_y); Serial.println("");
  Serial.printf("Midstep: (%d, %d)", midstep_x, midstep_y); Serial.println("");
  Serial.printf("Max X: (%d, 0.0)", maxX_x); Serial.println("");

  // validate data
  //  minXx < maxYx < midXx < maxXx
  //  minXy < maxYy and midy < maxYy
  //  no negative values (besides minX_x, maxY_x) - omitted are redundancies
  if ((minX_x <= maxY_x && maxY_x <= midstep_x && midstep_x <= maxX_x) && 
      (minX_y <= maxY_y && midstep_y <= maxY_y) &&
      (minX_y >= 0 && midstep_x > 0 && midstep_y > 0)) {
        data_packet[13] = minX_x;
        data_packet[14] = minX_y;
        data_packet[15] = maxY_x;
        data_packet[16] = maxY_y;
        data_packet[17] = midstep_x;
        data_packet[18] = midstep_y;
        data_packet[19] = maxX_x;
  }
  else {
      // pad with 0s so we know to not save this imu data
      memset(&data_packet[13], 0, 7);
  }

  // print out arr
  for (int i = 13; i < 20; i++) Serial.print(String(data_packet[i], DEC) + " ");


  // Transmit the collected data
  pCharacteristic->setValue((uint8_t *)data_packet, PACKET_SIZE);
  pCharacteristic->notify();
  Serial.println("Data transmitted");

  // Reset sample index for next collection
  fsr_sample_index = 0;
  max_value = 1;
  imu_sample_index = 0;
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