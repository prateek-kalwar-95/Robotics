# 🚗 Arduino Line Following Robot with PID Control

A simple **Arduino-based Line Following Robot** that evolves from basic workshop projects to intra-college competitions and finally to edge-level PID control.

---

## 📌 Journey
- **Workshop:** LED blinking, IR sensor basics, motor driver control.
- **Intra-college:** 2-sensor line follower with if-else logic.
- **Edge-level:** 8-sensor array with **PID control** for smooth, accurate navigation.

---

## 🧰 Components
- Arduino Uno R3
- L298N Motor Driver
- 2 DC Motors + Wheels + Chassis
- 8-channel IR Sensor Array
- Battery Pack (7.4V/9V)
- Jumper wires & Breadboard

---

## ⚡ Circuit (Summary)
- **Motors** → Controlled via L298N (PWM pins on Arduino)
- **Sensors** → A0–A7 inputs on Arduino
- **Power** → Common GND, battery to motors + Arduino

---

## 📐 PID Control (Basic Formula)
```
correction = (Kp * error) + (Kd * (error - lastError));
```
- **Kp:** proportional gain
- **Kd:** derivative gain
- **Ki:** not used here

Motors adjust speed based on correction.

---

## 🛠 Assembly & Testing
1. Mount motors, wheels, and sensors on chassis.
2. Connect wiring (common GND).
3. Upload Arduino code.
4. Test robot on track and tune Kp, Kd values.

---

## 🔧 Troubleshooting
- **Not moving:** check power/GND.
- **Wrong direction:** swap motor wires.
- **Zig-zag:** reduce Kp or increase Kd.
- **Off track:** align sensors properly.

---

## 🚀 Future Scope
- Checkpoint detection
- Obstacle avoidance
- Wireless debugging
- IoT integration

---

💡 *A concise guide to help you go from basics → competitions → advanced PID robotics.*
