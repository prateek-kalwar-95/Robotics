# Line Following Robot with PID Control

A sophisticated line following robot built with Arduino Nano, featuring PID control for smooth line tracking and Bluetooth connectivity for real-time parameter tuning.

## 🤖 Features

- **PID Control System**: Smooth and accurate line following with adjustable parameters
- **Bluetooth Tuning**: Real-time PID parameter adjustment via HC-05 module
- **8-Channel Sensor Array**: Precise line detection and positioning
- **Special Track Elements**: Handles checkpoints, crossroads, and finish lines
- **Intelligent Recovery**: Automatic line recovery when off-track
- **Visual Feedback**: LED indicators for checkpoints and status

## 🔧 Hardware Requirements

### Core Components
- **Arduino Nano** - Main microcontroller
- **TB6612FNG Motor Driver** - Dual motor control
- **HC-05 Bluetooth Module** - Wireless PID tuning
- **8-Channel IR Sensor Array (RLSO8)** - Line detection
- **LM2596 Buck Converter** - Power regulation
- **Two DC Motors** - Robot locomotion
- **LED** - Status indication

### Pin Configuration

#### Motor Driver Connections
| Pin | Connection | Description |
|-----|------------|-------------|
| 5   | PWMA       | Left Motor PWM (Speed) |
| 3   | AIN1       | Left Motor Direction 1 |
| 4   | AIN2       | Left Motor Direction 2 |
| 6   | STBY       | Motor Driver Standby |
| 9   | PWMB       | Right Motor PWM (Speed) |
| 10  | BIN1       | Right Motor Direction 1 |
| 12  | BIN2       | Right Motor Direction 2 |

#### Sensor Array Connections
| Pin | Sensor | Position |
|-----|--------|----------|
| A0  | SENSOR1| Rightmost |
| A1  | SENSOR2| - |
| A2  | SENSOR3| - |
| A3  | SENSOR4| Center-Right |
| A4  | SENSOR5| Center-Left |
| A5  | SENSOR6| - |
| A6  | SENSOR7| - |
| A7  | SENSOR8| Leftmost |

#### Other Connections
| Pin | Component | Description |
|-----|-----------|-------------|
| 13  | LED       | Status Indicator |
| 0   | HC-05 RX  | Bluetooth Communication |
| 1   | HC-05 TX  | Bluetooth Communication |

## 📋 Installation & Setup

### 1. Hardware Assembly
1. Mount the sensor array at the front of the robot chassis
2. Connect motors to the TB6612FNG driver
3. Wire all components according to the pin configuration table
4. Ensure proper power distribution using the LM2596 converter

### 2. Software Installation
1. Install Arduino IDE
2. Upload the provided code to Arduino Nano
3. Pair your device with HC-05 Bluetooth module (default: PIN 1234)

### 3. Calibration
1. Place robot on the track
2. Adjust `SENSOR_THRESHOLD` value (default: 500) based on your sensors
3. Set `BLACK_LINE` to 1 for black lines, 0 for white lines
4. Fine-tune PID parameters using Bluetooth commands

## 🎛️ PID Tuning

### Bluetooth Commands
Connect to HC-05 and send these commands:

| Command | Function | Example |
|---------|----------|---------|
| `p[value]` | Set Proportional gain | `p30` |
| `i[value]` | Set Integral gain | `i5` |
| `d[value]` | Set Derivative gain | `d80` |

### Default PID Values
- **Kp (Proportional)**: 25
- **Ki (Integral)**: 0
- **Kd (Derivative)**: 75

### Tuning Guidelines
1. **Start with Kp**: Increase until robot oscillates, then reduce by 20%
2. **Add Kd**: Increase to reduce oscillations and improve stability
3. **Fine-tune Ki**: Add small amounts to eliminate steady-state error
4. **Test thoroughly**: Verify performance on different track sections

## 🛤️ Track Features

### Supported Elements
- **Straight Lines**: Basic line following
- **Curves**: Smooth cornering with PID control
- **Checkpoints**: Middle 4 sensors detect line (LED turns on)
- **Crossroads**: Center + outer sensors detect line (robot goes straight)
- **Finish Line**: All 8 sensors detect line (robot stops)

### Track Requirements
- Line width: Optimized for 8-sensor array spacing
- Surface contrast: High contrast between line and background
- Lighting: Consistent ambient lighting for IR sensors

## ⚙️ Configuration Parameters

### Speed Settings
```cpp
int baseSpeed = 90;           // Base motor speed (0-255)
int maxSpeed = 130;           // Maximum speed limit
int turnSpeedReduction = 30;  // Speed reduction % when turning
```

### Sensor Settings
```cpp
#define SENSOR_THRESHOLD 500  // Analog threshold for line detection
#define BLACK_LINE 1          // 1 for black line, 0 for white line
```

### Timing Parameters
```cpp
int brakeTime = 50;                    // Brake duration (ms)
const unsigned long FEEDBACK_INTERVAL = 1000; // PID feedback interval (ms)
```

## 🔍 Troubleshooting

### Common Issues

**Robot not following line smoothly**
- Check PID parameters (start with Kd=0, tune Kp first)
- Verify sensor alignment and height
- Adjust `SENSOR_THRESHOLD` value

**Robot loses line frequently**
- Increase `baseSpeed` for better momentum
- Check sensor array for dust/damage
- Ensure consistent lighting conditions

**Bluetooth connection issues**
- Verify HC-05 pairing (PIN: 1234)
- Check baud rate (9600)
- Ensure proper TX/RX connections

**Motors not responding**
- Check motor driver connections
- Verify power supply voltage
- Test STBY pin activation

## 📊 Performance Monitoring

The robot sends real-time feedback via Bluetooth every second:
```
PID Values: Kp=25, Ki=0, Kd=75
```

Additional status messages:
- `"Finish line detected - Stopped"` - When finish line is reached
- Individual PID parameter updates when changed via Bluetooth

## 🚀 Advanced Features

### Line Recovery Algorithm
When the robot loses the line:
1. Apply short brake to prevent overshooting
2. Turn in the direction of last known error
3. Continue turning until line is reacquired
4. Timeout after 2 seconds to prevent infinite loops

### Integral Windup Prevention
The integral term is constrained to prevent windup:
```cpp
integral = constrain(integral, -10000, 10000);
```

## 📝 Code Structure

- **Setup**: Hardware initialization and Bluetooth setup
- **Main Loop**: Sensor reading, PID calculation, motor control
- **Sensor Functions**: Line detection and error calculation
- **PID Controller**: Three-term control algorithm
- **Motor Control**: Direction and speed management
- **Bluetooth Interface**: Real-time parameter tuning
- **Special Conditions**: Checkpoint, crossroad, and finish line handling



## 🆘 Support

For technical support or questions:
- Check the troubleshooting section
- Review pin connections
- Verify PID tuning procedure
- Test individual components separately

---

**Happy Robot Building! 🤖**
