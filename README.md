# IoT25-HW07: BLE-Based Distance Estimation with ESP32

This project demonstrates how to estimate distance between two ESP32 boards using Bluetooth Low Energy (BLE) signal strength (RSSI). One ESP32 acts as a BLE **Advertiser (Server)** and the other as a **Scanner (Client)** that reads RSSI values and estimates the distance using a path loss model. Additionally, the estimated distances are logged and visualized.

---

## 🧾 Objectives

- Establish BLE advertisement between two ESP32 boards
- Scan and measure RSSI signal strength from the advertiser
- Estimate physical distance using RSSI and TX power values
- Display distance and status via serial monitor and RGB LED
- Record multiple distance measurements for different positions
- Visualize results with graphs and tables

---

## 🧰 Components

- 2× ESP32 DevKit v1 boards  
- RGB LED (connected to GPIO 25, 26, 27)  
- Jumper wires  
- Breadboard  
- Laptop (for Arduino IDE and visualization)

---

## 💡 Overview

- **BLE Server (`IoT25_HW07_server.ino`)**
  - Initializes BLE advertising with custom service UUID
  - No connection required, just advertises presence
  - Advertiser name: `ESP32_Advertiser`

- **BLE Client (`IoT25_HW07_client.ino`)**
  - Scans for BLE devices matching a target MAC address
  - Reads RSSI values of the advertisement packets
  - Estimates distance using the log-distance path loss model:
    ```
    distance = 10 ^ ((txPower - RSSI) / (10 * n))
    ```
    where `txPower` is assumed, and `n` is environmental factor
  - Displays status via RGB LED (color-coded by distance)
  - Logs distance to serial monitor

---

## 📄 Key Code Features

- **BLE Scan & RSSI**
  - Uses `BLEScan` and `BLEAdvertisedDevice` classes
  - Scans for 3 seconds at intervals and logs RSSI

- **Distance Estimation Model**
  - Calibrated with known txPower and path loss exponent `n`
  - Distances calculated for 0.5m, 1m, 2m, 3m, 4m

- **RGB LED Color Mapping**
  - Red: Close distance (e.g., < 1m)
  - Green: Medium distance
  - Blue: Far distance (e.g., > 3m)

---

## 📊 Results

![Result Graph](./media/HW07_chart.png)

---

## 📸 Photos

- ![Photo 1](./media/hw%207-1.jpg)
- ![Photo 2](./media/hw%207-2.jpg)
- ![Photo 3](./media/hw%207-3.jpg)
- ![Photo 4](./media/hw%207-4.jpg)

---

## ▶ Demo

### BLE Scanning and RGB Feedback

- ![GIF 1](./media/IoT25-HW07_1.gif)
- ![GIF 2](./media/IoT25-HW07_2.gif)

---

## ✅ Observations

- RSSI decreases with distance, and the estimated distance increases
- Results are consistent across repeated measurements
- Some fluctuation observed due to BLE signal variability
- Visualized graph shows correlation between actual and estimated distance

---

## 📁 File Structure

```
IoT25-HW07/
├── code/
│   ├── IoT25_HW07_server.ino
│   ├── IoT25_HW07_client.ino
│   └── index.html  (optional future web interface)
├── media/
│   ├── Photos (hw 7-*.jpg)
│   ├── Graph (HW07_chart.png)
│   ├── Demo GIFs & Videos
```

---

## 🔗 References

- [ESP32 BLE Advertiser & Scanner Guide](https://randomnerdtutorials.com/esp32-ble-server-client/)
- [RSSI to Distance Estimation Theory](https://www.novelbits.io/bluetooth-rssi-distance-measurement/)
