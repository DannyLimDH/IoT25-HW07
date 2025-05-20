#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define DEVICE_NAME "ESP32_Advertiser"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"  // Example UUID

void setup() {
  Serial.begin(115200);

  // Initialize BLE
  BLEDevice::init(DEVICE_NAME);
  BLEServer *pServer = BLEDevice::createServer();  // Required even without connections

  // Set up service UUID (optional)
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pService->start();

  // Configure advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);  // Include service UUID
  pAdvertising->setScanResponse(true);         // Include additional scan response data
  pAdvertising->setMinPreferred(0x06);         // Optimize connection interval
  pAdvertising->setMinPreferred(0x12);         // Optimize connection interval

  // Start advertising
  BLEDevice::startAdvertising();
  Serial.println("BLE Advertising started (Advertiser only mode).");
}

void loop() {
  // Nothing to do in loop; advertising runs in background
  delay(1000);
}
