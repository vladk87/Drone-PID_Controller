# Drone PID Controller – STM32 Project

This project implements a real-time PID-based flight controller for a quadcopter using an STM32 Nucleo-L432KC microcontroller. It features motor control via PWM, altitude measurement using a Time-of-Flight (ToF) distance sensor (VL53L0X), and orientation stabilization with a 3-axis gyroscope (LSM9DS1). The system aims to achieve stable lift, autonomous hover, and basic altitude control.

---

## ✈️ Features

- Real-time PID control for pitch, roll, and altitude
- PWM-based motor control using hardware timers
- Sensor integration (IMU and ToF distance sensor)
- Altitude hold and hover mode
- Autonomous takeoff, hover, and landing cycle
- Serial debugging over UART
- Modular, interrupt-driven C code for STM32

---

## 🧰 Hardware Used

| Component                 | Description                          |
|--------------------------|--------------------------------------|
| STM32 Nucleo-L432KC      | Microcontroller (ARM Cortex-M4)      |
| VL53L0X                  | Time-of-Flight (ToF) distance sensor |
| LSM9DS1                  | 3-axis Gyroscope/Accelerometer       |
| X-NUCLEO-IHM04A1         | Motor driver shield (for brushed DC) |
| 4x 3V DC Motors          | Drone propulsion                     |
| AMS1117 Buck Converter   | Voltage regulation from battery      |
| 3.7V LiPo Battery        | Power source                         |

---

## 📐 System Overview

```plaintext
      [ Gyro (SPI) ]          [ ToF Sensor (I2C) ]
             |                        |
             +--------+--------------+
                      |
                [ STM32 Nucleo ]
                      |
          +-----------+------------+
          |     PWM Outputs to     |
          |    MOSFET/Motor Driver |
          +------------------------+

