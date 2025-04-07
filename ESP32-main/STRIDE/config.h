/*-----------------------------------------------------------------------------

 config.h:

 @authors: Luke Andresen

 @date: 3/7/2025

 @brief: This file contains static and mutable configurations for the STRIDE platofrm, including pins, libraries, and key variables
------------------------------------------------------------------------------*/

/*************KEY CHANGABLE VARIABLES*************/  

// STRIDE_MODE = 1 for LEFT, 2 for RIGHT
#define STRIDE_MODE 1

// Define buffer sizes based on estimated data needs
#define NUM_PADS 8
#define MAX_PAD_SAMPLES 200
#define NUM_IMU_AXES 3             // Only Up-Forward-Side
#define MAX_IMU_SAMPLES 200

#define PRESSURE_THRESHOLD 50      // Minimum pressure to start collecting data
#define PACKET_SIZE        20      // Number of points to transmit

#define IMU_PACKET_SIZE     7      // Number of points available for IMU
#define IMU_SAMPLE_FREQ    10      // Sample frequency

// BLE values
#if STRIDE_MODE == 1
  #define DEVICE_NAME         "STRIDE-LEFT"
  #define SERVICE_UUID        "90ca8ba1-ad86-407a-80ce-f2f1d256c27e" // generated STRIDE-LEFT service uuid
  #define CHARACTERISTIC_UUID "c04a0c51-1f6d-44ea-bd08-675a0a578e41" // generated STRIDE-LEFT characteristic uuid
  #define CHARACTERISTIC_DESC "Data channel for STRIDE-LEFT"
#else
  #define DEVICE_NAME         "STRIDE-RIGHT"
  #define SERVICE_UUID        "9549d68d-c01b-413b-afe0-38f7dbe49651" // generated STRIDE-RIGHT service uuid
  #define CHARACTERISTIC_UUID "8023c288-db87-43b0-afd8-d7d3063c7a4c" // generated STRIDE-RIGHT characteristic uuid
  #define CHARACTERISTIC_DESC "Data channel for STRIDE-RIGHT"
#endif

/*************PINS*************/
#define PAD1_PIN 7
#define PAD2_PIN 9
#define PAD3_PIN 6
#define PAD4_PIN 8
#define PAD5_PIN 17
#define PAD6_PIN 18
#define PAD7_PIN 5
#define PAD8_PIN 16

#define SDA      41  // SDA for I2C Bus 1 (STEMMA QT default)
#define SCL      40  // SCL for I2C Bus 1 (STEMMA QT default)

/*************LIBRARIES*************/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLE2901.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
