## Sensors
| Component No | Description |
| --- | --- |
| ST GY-VL53L0XV2 | Time of Flight Distance Sensor |
| MPU9250 | 9-DOF 3-Axis Acceleration Gyroscope Magnetometer |
| N/A | 5-Bit Line Hunting Sensor Follower Module for Arduino |

## Controllers/Actuators
| Component No | Description |
| --- | --- |
| TB6612FNG | Dual DC Stepper Motor Driver |
| N20 | Gear Motor 6V 300RPM Shaft 10mm with Hall Sensor Encoder |
| ESP32-S3 DevKitC-1 | ESP32 Dev Board N16R8 WiFiBluetooth IoT |

## Power
| Component No | Description |
| --- | --- |
| 18650 | Flat Top 3.7V 3200mA Li-ion Rechargeable Battery |
| H961-u v6 | 18650 Battery Charging Module |

## Pin Connections

### N20 Motor Right (Port Side)
| N20 Pin | Component | Pin | Remark |
| --- | --- | --- | --- |
| M1 | TB6612FNG | B2 | Motor Terminal 1 (Input) |
| M2 | TB6612FNG | B1 | Motor Terminal 1 (Input) |
| C1 | ESP32-S3 | 36 (GPIO 36) | Encoder 1 (Output) |
| C2 | ESP32-S3 | 37 (GPIO 37) | Encoder 2 (Output) |

### N20 Motor Left (Starboard Side)
| N20 Pin | Component | Pin | Remark |
| --- | --- | --- | --- |
| M1 | TB6612FNG | A2 | Motor Terminal 1 (Input) |
| M2 | TB6612FNG | A1 | Motor Terminal 1 (Input) |
| C1 | ESP32-S3 | 38 (GPIO 38) | Encoder 1 (Output) |
| C2 | ESP32-S3 | 39 (GPIO 39) | Encoder 2 (Output) |

### TB6612FNG
| TB6612FNG Pin | Component | Pin | Remark |
| --- | --- | --- | --- |
| VM | H961-u v6 | +5v | Motor Power |
| VCC | ESP32-S3 | +3.3v | Logic Power |
| GND | H961-u v6 | GND | Common Ground |
| A1 | N20 Motor Left | M2 | DC MotorA Terminal 1 (Output) |
| A2 | N20 Motor Left | M1 | DC MotorA Terminal 2 (Output) |
| B1 | N20 Motor Right | M2 | DC MotorB Terminal 1 (Output) |
| B2 | N20 Motor Right | M1 | DC MotorB Terminal 2 (Output) |
| AIN1 | ESP32-S3 | 10 (GPIO 10) | DC MotorA Direction Control Pin 1 (Input) |
| AIN2 | ESP32-S3 | 11 (GPIO 11) | DC MotorA Direction Control Pin 2 (Input) |
| BIN1 | ESP32-S3 | 47 (GPIO 47) | DC MotorB Direction Control Pin 1 (Input) |
| BIN2 | ESP32-S3 | 48 (GPIO 48) | DC MotorB Direction Control Pin 2 (Input) |
| PWMA | ESP32-S3 | 4 (GPIO 4) | DC MotorA PWM (Speed) Control (Input) |
| PWMB | ESP32-S3 | 5 (GPIO 5) | DC MotorB PWM (Speed) Control (Input) |
| STBY | ESP32-S3 | 1 (GPIO 1) | Motor (A and B) enable pin (Input / Pull high to enable / Default disabled) |
