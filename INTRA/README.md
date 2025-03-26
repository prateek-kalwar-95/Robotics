# 🚀 Line Following Robot with PID Control

## 📌 Overview  
The **Line Following Robot** is an autonomous vehicle designed to navigate a predefined track by detecting and following a black line on a white surface. This project utilizes an **8-channel IR sensor array** to track the line and implements a **PID (Proportional-Integral-Derivative) control system** to maintain stability and accuracy while following the path. 

The PID control optimizes motor speed and direction based on sensor readings, minimizing deviation and enhancing performance. This documentation provides an in-depth explanation of the **hardware, circuit design, code logic, and troubleshooting techniques** to help understand the working principle of the robot.

---

## 🛠 Hardware Components & Description  

The following components are used in this project:

### 🔹 1. **Arduino Uno R3**  
   - **Function**: Acts as the **brain of the robot**, processing sensor inputs and controlling motor outputs.  
   - **Specifications**:
     - Microcontroller: ATmega328P  
     - Operating Voltage: 5V  
     - Digital I/O Pins: 14 (of which 6 support PWM)  
     - Analog Input Pins: 6  
     - Flash Memory: 32 KB  
   - **Why Arduino?**  
     - Easy to program using the Arduino IDE  
     - Large community support  
     - Compatible with multiple sensors and modules  

### 🔹 2. **L298N Motor Driver Module**  
   - **Function**: Controls the speed and direction of two DC motors by varying PWM signals.  
   - **Specifications**:
     - Input Voltage: 5V–35V  
     - Dual H-Bridge motor controller  
     - Maximum current: 2A per channel  
   - **Why L298N?**  
     - Provides bidirectional motor control  
     - Can control motor speed using PWM signals  
     - Has built-in overheating protection  

### 🔹 3. **8-Channel IR Sensor Array**  
   - **Function**: Detects the black line by measuring light reflectivity and sends digital output to Arduino.  
   - **Specifications**:
     - 8 individual IR sensors  
     - Output: Digital (0 or 1)  
     - Operating Voltage: 5V  
   - **Why IR Sensor Array?**  
     - High precision in detecting line deviations  
     - Faster response time compared to single IR sensors  

### 🔹 4. **LM2596 Buck Converter (Voltage Regulator)**  
   - **Function**: Steps down battery voltage to **5V** for the sensor array.  
   - **Why Needed?**  
     - Ensures stable voltage supply  
     - Protects components from voltage fluctuations  

### 🔹 5. **N20 DC Gear Motors (3V–12V)**  
   - **Function**: Drives the wheels of the robot.  
   - **Why N20 Motors?**  
     - Compact, high torque, and efficient  
     - Suitable for precise control in robotic applications  

### 🔹 6. **Chassis & Wheels**  
   - **Function**: Provides structure and mobility to the robot.  

### 🔹 7. **Power Supply (Rechargeable Battery 12V)**  
   - **Function**: Provides power to Arduino, motor driver, and sensors.  

---

## 🛠 Circuit Design & Connections  

### 🔌 **Motor Driver (L298N) to Arduino Connections**  

| L298N Pin | Arduino Pin | Function |
|-----------|-------------|--------------------------|
| IN1 | 4 | Left Motor Direction Control |
| IN2 | 5 | Left Motor Direction Control |
| ENA | 6 | Left Motor Speed Control (PWM) |
| IN3 | 7 | Right Motor Direction Control |
| IN4 | 8 | Right Motor Direction Control |
| ENB | 9 | Right Motor Speed Control (PWM) |

### 🔌 **IR Sensor Array to Arduino Connections**  

| Sensor Index | Arduino Pin |
|--------------|-------------|
| Sensor 1 | 2 |
| Sensor 2 | 3 |
| Sensor 3 | 10 |
| Sensor 4 | 11 |
| Sensor 5 | 12 |
| Sensor 6 | A0 |
| Sensor 7 | A1 |
| Sensor 8 | A2 |

### 🔌 **Power Connections**  
- **Arduino Uno** → Powered via USB or an external battery pack  
- **L298N Motor Driver** → Connected to **12V battery** to drive motors  
- **IR Sensor Array** → Receives **5V** from the LM2596 Buck Converter  

🔴 **Note:** All components must have a common **GND** to prevent electrical issues.

---

## 🧠 Code Logic & PID Control  

### **What is PID Control?**  
PID (Proportional-Integral-Derivative) control helps adjust motor speed based on sensor input to maintain the robot’s stability.  

### **PID Formula Used**  
\[
\text{Correction} = (K_p \times \text{Error}) + (K_d \times \text{Derivative}) + (K_i \times \text{Integral})
\]
Where:  
- **Kp (Proportional Gain)** → Reacts to current error  
- **Ki (Integral Gain)** → Corrects accumulated past errors (set to 0 in our case)  
- **Kd (Derivative Gain)** → Predicts future error  

### **PID Tuning Parameters**  

| Parameter | Value | Description |
|-----------|-------|-------------|
| Kp | 12 | Determines reaction to error |
| Ki | 0 | Not used in this setup |
| Kd | 36 | Helps stabilize robot |

### **Code Flow**  
1. **Sensor Reading**: IR sensor array detects black line positions.  
2. **Error Calculation**: The error is computed using a weighted sum approach.  
3. **PID Computation**: Adjusts motor speed based on error.  
4. **Motor Control**: Updates PWM values to correct deviations.  

### **Key Functions**  
- `readSensors()`: Reads IR sensor values.  
- `computeError()`: Calculates deviation from line.  
- `computePID()`: Uses the PID formula to generate correction values.  
- `adjustMotorSpeeds()`: Modifies motor speed based on correction output.  

---

## 🛠 Assembly & Testing  

### **Steps to Build the Robot**  
1. **Chassis Setup**: Attach motors to the chassis and fix wheels.  
2. **Component Placement**: Secure Arduino, L298N, and sensors on the chassis.  
3. **Wiring**: Connect components as per the circuit diagram.  
4. **Power Setup**: Connect the battery pack and ensure proper voltage distribution.  
5. **Testing**:  
   - Upload the code to Arduino.  
   - Test sensor readings and motor responses.  
   - Place the robot on the track and fine-tune PID values if needed.  

---

## 🔧 Troubleshooting  

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Robot not moving | Loose wiring | Check power connections |
| Motors spinning incorrectly | Wrong connections to L298N | Swap motor wires |
| Robot not following the line | Incorrect PID values | Adjust Kp and Kd |
| Erratic movements | Sensor misalignment | Ensure sensors are positioned correctly |

---

## 🚀 Future Improvements  
- **Adaptive PID Control**: Dynamic tuning based on real-time conditions.  
- **Obstacle Detection**: Integrate ultrasonic sensors for navigation.  
- **Wireless Control**: Add Bluetooth or Wi-Fi for remote debugging.  
- **Camera Integration**: Implement Computer Vision for enhanced line tracking.  

---

## 🏆 Conclusion  
This **Line Following Robot** showcases the integration of **PID control**, **motor control**, and **sensor-based decision making**, making it a fundamental project in **robotics and embedded systems**. The knowledge gained from this project can be extended to advanced applications like **autonomous navigation, warehouse automation, and industrial robotics**. 🚀  

---

Hope this documentation helps! 😊 Feel free to contribute or ask questions.  

