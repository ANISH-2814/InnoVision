# InnoVision

# 🛠️ AI-Powered Image Analysis of Industrial Objects for Quality Assurance

An end-to-end automated quality inspection system using **YOLOv8**, **Python**, and **Raspberry Pi**, capable of identifying defective industrial components in real-time and rejecting them automatically with a servo-controlled mechanism.

---

## 📸 Demo

![System Demo](media/finaldemo.gif)  
> *(Defected Product (Gear) detection and automatic ejection demonstration)*

---

## 🚀 Features

- 🔍 Real-time object detection and defect classification using YOLOv8
- 📷 Live video feed processing from a USB camera
- 🤖 Automated rejection system using a servo motor
- ⚙️ Conveyor control using 12V DC motor and ultrasonic sensor
- 🧠 Edge device deployment on Raspberry Pi 4
- 💡 Cost-effective solution for industrial automation

---

## 🧠 How It Works

1. **Object Detection**  
   A USB camera captures a live feed as objects move on a conveyor belt.

2. **YOLOv8 Analysis**  
   Each frame is processed using YOLOv8 to classify the object as `GOOD` or `DEFECTIVE`.

3. **Automation Logic**  
   - If `GOOD`: The object continues on the conveyor.
   - If `DEFECTIVE`: A servo motor pushes it off the belt.

4. **Ultrasonic Sensor**  
   Detects the presence of an object to trigger image capture and model inference.

---

## 🔧 Hardware Used

| Component          | Description                          |
|-------------------|--------------------------------------|
| Raspberry Pi 4     | Main controller                      |
| USB Camera         | Live feed acquisition                |
| 12V DC Motor       | Conveyor drive                       |
| Ultrasonic Sensor  | Object detection trigger             |
| Servo Motor        | Defective object rejection           |
| L298N Motor Driver | Controls DC and servo motors         |
| Conveyor Belt      | Transports objects                   |

---

## 💻 Software Stack

- **Python 3.9+**
- **YOLOv8 (Ultralytics)**
- **OpenCV**
- **GPIO Zero / RPi.GPIO**
- **Linux (Raspberry Pi OS)**

---
