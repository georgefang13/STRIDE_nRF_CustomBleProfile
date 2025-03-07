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
#define MAX_PAD_SAMPLES 500
#define IMU_SAMPLES 25
#define BYTES_PER_VALUE 2  // Assume uint16_t for pressure, float16 possible for IMU

#define PRESSURE_THRESHOLD 50  // Minimum pressure to start collecting data
#define TRANSMIT_POINTS 5     // Number of evenly spaced points to transmit

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
#define PAD1_PIN 6
#define PAD2_PIN 5
#define PAD3_PIN 17
#define PAD4_PIN 9
#define PAD5_PIN 8
#define PAD6_PIN 7
#define PAD7_PIN 16
#define PAD8_PIN 18

/*************LIBRARIES*************/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLE2901.h>