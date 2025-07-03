# IoT Sanitary Control

An intelligent sanitary control system developed for the Eng. Alcínio Miguel Auditorium, using Internet of Things (IoT) technologies to monitor and automate environmental variables like temperature, humidity, air quality, and occupancy.

---

## 📌 Table of Contents

* [Description](#description)
* [Features](#features)
* [Architecture](#architecture)
* [Equipment Used](#equipment-used)
* [How to Run](#how-to-run)
* [Node-RED Dashboard](#node-red-dashboard)




---

## 📖 Description

This project demonstrates the practical application of IoT for sanitary control in public spaces. Physical (simulated) sensors monitor the room conditions and occupancy. The data is sent via MQTT to a Node-RED dashboard that displays real-time parameters and runs automation rules for ventilation, air conditioning, and lighting.

---

## ✨ Features

* **Monitoring of temperature, humidity, light level, and CO₂**
* **Real-time people counting** with ultrasonic sensors
* **Automatic opening of windows and curtains** with servo motors
* **Automatic lighting** with simulated LED
* **Visual and sound alerts** when maximum occupancy is reached
* **Interactive dashboard** in Node-RED with historical charts

---

## 🗂️ Architecture

* **Microcontroller:** ESP32 (simulated on [Wokwi](https://wokwi.com/))
* **Communication:** MQTT protocol (public broker)
* **Visualization:** Node-RED dashboard

---

## 🔧 Equipment Used

* **ESP32:** Microcontroller with Wi-Fi
* **DHT22:** Temperature and humidity sensor
* **LDR:** Light sensor
* **HC-SR04 (x2):** Ultrasonic sensors for entry/exit counting
* **Servo Motors:** For windows and curtains
* **LED:** To simulate auditorium lighting

---

## ▶️ How to Run

1. **Clone the repository**

   ```bash
   git clone https://github.com/NunoAraujo55/Controlo---Sanitrio.git
   cd Controlo---Sanitrio
   ```

2. **Open the project on [Wokwi](https://wokwi.com/)**
   Import or recreate the setup with ESP32, sensors, and actuators.


3. **Set up InfluxDB**
  
    * Make sure your InfluxDB instance is running and accessible.
    * Adjust the database name, bucket, and organization in the Node-RED InfluxDB nodes to match your setup.  
    * Verify that data from MQTT is written correctly to InfluxDB.

4. **Import the Node-RED flow**

   * Open Node-RED, click the top-right menu → Import → Clipboard.
   * Paste the content of the provided `flows.json`.
   * Deploy the flow and ensure the MQTT broker and InfluxDB match your configuration.

5. **Run the simulation**

   * Program the ESP32 (`main.cpp` example) to publish data via MQTT.
   * Check the indicators on the Node-RED dashboard.

---

## 📊 Node-RED Dashboard

The dashboard shows:

* Occupancy, temperature, humidity, light level, and air quality indicators.
* Historical CO₂ charts.
* Visual and sound alerts when occupancy reaches the defined limit.
* Automatic control of windows, curtains, and air conditioning.

👉 **Note:** The complete flow configuration is available in `flows.json`. Import this file to get all nodes, gauges, charts, and logic ready to use.
