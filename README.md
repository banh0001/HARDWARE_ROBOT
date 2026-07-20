# LUNA Hardware Firmware

Firmware สำหรับ LUNA Mecanum Robot รันบน ESP32-S3 ใช้ micro-ROS ในการสื่อสารกับ ROS 2

![LUNA Robot](docs/images/LUNA_ROBOT.png)

### System Overview

![System Overview](docs/images/overviewcon.png)

---

## โครงสร้างโปรเจกต์

```
LUNA_BOT/
├── config/
│   └── luna_robot.h          # ค่า config หลักของหุ่นยนต์ (pin, PID, wheel)
├── main/
│   ├── platformio.ini        # config สำหรับ firmware หลัก
│   ├── src/
│   │   └── firmware.ino      # firmware หลัก (micro-ROS publisher/subscriber)
│   └── lib/
│       ├── encoder/          # อ่านค่า encoder
│       ├── imu/              # driver MPU6050
│       ├── kinematics/       # คำนวณ mecanum kinematics
│       ├── motor/            # driver มอเตอร์ (LUNA_MOTOR_DRIVE)
│       ├── odometry/         # คำนวณ odometry
│       └── pid/              # PID controller
└── test/
    ├── platformio.ini        # config สำหรับ test firmware
    └── src/
        ├── firmware.ino      # test firmware (ไม่ต้องใช้ micro-ROS Agent)
        ├── cmd_spin.h        # คำสั่ง spin / sample
        ├── cmd_ticks.h       # คำสั่ง ticks
        ├── cmd_test.h        # คำสั่ง test
        ├── cmd_testall.h     # คำสั่ง testall
        └── cmd_imu.h         # คำสั่ง imu
```

---

## 1. ติดตั้ง PlatformIO

### วิธีที่ 1 — VS Code Extension (แนะนำ)

1. ติดตั้ง [Visual Studio Code](https://code.visualstudio.com/)
2. เปิด VS Code → ไปที่ Extensions (`Ctrl+Shift+X`)
3. ค้นหา **PlatformIO IDE** แล้วกด Install
4. รอ PlatformIO Core ติดตั้งเสร็จ (ประมาณ 2–5 นาที) แล้ว Reload window

### วิธีที่ 2 — Command Line (Linux/macOS)

```bash
pip install platformio
```

ตรวจสอบว่าติดตั้งสำเร็จ:

```bash
pio --version
```

### ติดตั้ง udev rules (Linux เท่านั้น)

ให้สิทธิ์เชื่อมต่อ USB โดยไม่ต้อง sudo:

```bash
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo udevadm control --reload-rules
sudo usermod -aG dialout $USER
sudo usermod -aG plugdev $USER
```

> หลังรันคำสั่งนี้ต้อง **logout แล้ว login ใหม่** หรือ reboot

---

## 2. Clone โปรเจกต์

```bash
git clone https://github.com/banh0001/luna_mecanum_robot_hardware.git
cd luna_mecanum_robot_hardware
```

---

## 3. ตั้งค่าหุ่นยนต์ (`config/luna_robot.h`)

ไฟล์นี้คือหัวใจของ config ทั้งหมด แก้ค่าให้ตรงกับฮาร์ดแวร์ก่อนอัพโหลด

**การเชื่อมต่อฮาร์ดแวร์ทั้งหมด:**

![Hardware Connections](docs/images/hardwarecontenct.png)

**บอร์ด LUNA:**

![LUNA Board](docs/images/LUNA.png)

**มุมมองด้านข้าง:**

![Side View](docs/images/SF.png)

```cpp
// Hardware ที่ใช้
#define LUNA_MOTOR_DRIVER       // LUNA_DRIVER
#define IMU_MPU6050             // IMU MPU6050
#define LUNA_BASE MECANUM       // ประเภทฐานล้อ

// I2C pins
#define SDA_PIN 2
#define SCL_PIN 1

// PID gains สำหรับแต่ละมอเตอร์
#define K_P1 0.01
#define K_I1 1.0
#define K_D1 0

// ข้อมูลล้อและมอเตอร์
#define MOTOR_MAX_RPM           60      // RPM สูงสุดของมอเตอร์
#define WHEEL_DIAMETER          0.065   // เส้นผ่านศูนย์กลางล้อ (เมตร)
#define LR_WHEELS_DISTANCE      0.1975  // ระยะห่างระหว่างล้อซ้าย-ขวา (เมตร)

// Encoder counts per revolution (วัดจริงด้วยคำสั่ง sample)
#define COUNTS_PER_REV1 2250
#define COUNTS_PER_REV2 2102
#define COUNTS_PER_REV3 2276
#define COUNTS_PER_REV4 2139
```

**การจัดวางมอเตอร์:**
```
        FRONT
  MOTOR1    MOTOR2
  MOTOR3    MOTOR4
        BACK
```

![Motor Direction](docs/images/mecanum_motion_direction.png)

---

## 4. Firmware หลัก (`main/`)

Firmware สำหรับใช้งานจริงกับ ROS 2 ผ่าน micro-ROS

### Firmware Flowchart

```mermaid
flowchart TD
    START([START]) --> SETUP["SETUP\nSerial.begin 921600\npinMode LED OUTPUT\nWire.begin SDA/SCL"]
    SETUP --> IMU{imu.init OK?}
    IMU -->|No| FLASH["flashLED(3) loop"]
    FLASH --> END_STATE([END])
    IMU -->|Yes| TRANSPORT["set_microros_serial_transport\n/dev/ttyACM0"]
    TRANSPORT --> BUFTIMER["buffer + timer init"]
    BUFTIMER --> PING1{PING agent\nevery 500 ms}
    PING1 -->|No| WAIT1["WAITING_AGENT"]
    WAIT1 --> PING1
    PING1 -->|Yes| CREATEENT[["CreateEntities()\nnode / topic / timer 20ms"]]
    CREATEENT --> CREATEOK{CreateEntities\nOK?}
    CREATEOK -->|No| DESTROY1[["destroyEntities"]]
    DESTROY1 --> WAIT1
    CREATEOK -->|Yes| PING2{ping_Agent\nevery 200 ms}
    PING2 -->|No| FULLSTOP["fullStop + destroyEntities"]
    FULLSTOP --> WAIT1
    PING2 -->|Yes| EXEC[["rclc_executor_spin_some\nCONTROL_TIMER 20ms"]]
    EXEC --> MB[["moveBase"]]
    EXEC --> PD[["publishData"]]
    MB --> CMDTO{cmd_timeout\n> 200 ms?}
    CMDTO -->|No| ZEROV["linear.x/y = 0\nangular.z = 0"]
    CMDTO -->|Yes| GETRPM["kinematics.getRPM\nlinear.x/y, angular.z"]
    ZEROV --> GETRPM
    GETRPM --> ENCRPM["encoder.getRPM"]
    ENCRPM --> CMDZERO{cmd_vel == 0?}
    CMDZERO -->|Yes| PIDM["PID motor\nPWM / readRPM"]
    CMDZERO -->|No| BRAKEM["motor.brake"]
    PIDM --> GETVEL["getVelocities\ncurrent_rpm 1–4"]
    BRAKEM --> GETVEL
    GETVEL --> ODOUP["odometry_update\ndt, vx, vy, az"]
    PD --> MSGS["odom_msg + imu_msg"]
    MSGS --> QUAT["compute quaternion\nx, y, z, w"]
    QUAT --> GETT[["getTime"]]
    GETT --> STAMP["stamp header.stamp = time_stamp"]
    STAMP --> RPMDATA["rpm_data[0–3] = rpm[1–4]"]
    RPMDATA --> PUB[("publish\n/imu/data\n/odom_unfiltered\n/motor/rpm")]
```

### Build และ Upload

**ใน VS Code:**
1. เปิดโฟลเดอร์ `main/` ด้วย PlatformIO
2. กดปุ่ม **Build** (✓) หรือ **Upload** (→) ที่ status bar ด้านล่าง

**ใน Terminal:**
```bash
cd main
pio run                        # แค่ build
pio run -e luna_robot -t upload        # build + upload
```

### Topics ที่ firmware หลัก Publish/Subscribe

| Topic | Type | ทิศทาง | คำอธิบาย |
|---|---|---|---|
| `cmd_vel` | `geometry_msgs/Twist` | Subscribe | รับคำสั่งความเร็ว |
| `odom/unfiltered` | `nav_msgs/Odometry` | Publish | ข้อมูล odometry |
| `imu/data` | `sensor_msgs/Imu` | Publish | ข้อมูล accelerometer + gyroscope |
| `motor/rpm` | `std_msgs/Float32MultiArray` | Publish | RPM ของมอเตอร์ทั้ง 4 |

### เชื่อมต่อ micro-ROS Agent (บน PC)

```bash
# ติดตั้ง agent (ครั้งแรกครั้งเดียว) ดูคลิปนี้
https://youtu.be/F4KXbHpUiv4?si=xijBHGEX6vcmlbPy

# รัน agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600
```

---

## 5. Test Firmware (`test/`)

Test firmware ใช้สำหรับทดสอบฮาร์ดแวร์ก่อนใช้งานจริง **ไม่ต้องใช้ micro-ROS Agent**
สื่อสารผ่าน Serial Monitor ธรรมดา baud rate **115200**

### Build และ Upload

```bash
cd test
pio run -e luna_robot -t upload
pio device monitor -e luna_robot
```

หรือใน VS Code เปิดโฟลเดอร์ `test/` แล้วกด Upload เหมือนเดิม

### คำสั่งที่ใช้ได้ใน Serial Monitor

เมื่อ upload เสร็จและเปิด Serial Monitor จะเห็น:

```
Type 'spin'    - spin motors at max RPM
Type 'sample'  - spin motors and show summary
Type 'ticks'   - measure ticks per revolution
Type 'test'    - test PID with cmd_vel
Type 'testall' - run all 4 motors and plot data
Type 'imu'     - stream IMU accel/gyro data
Press enter to clear command.
```

---

### คำสั่ง `spin` — หมุนมอเตอร์ทดสอบ

```
> spin
```

- หมุนมอเตอร์แต่ละตัวทีละตัวที่ PWM สูงสุด เป็นเวลา **5 วินาที**
- ใช้ตรวจสอบทิศทางการหมุนของมอเตอร์ว่าถูกต้องหรือไม่
- ต้องยกหุ่นยนต์ขึ้นให้ล้อลอยพ้นพื้น

---

### คำสั่ง `sample` — วัด RPM และ Counts Per Revolution จริง

```
> sample
```

- หมุนมอเตอร์แต่ละตัวทีละตัว แล้วพิมพ์ผลสรุป:
  - Encoder counts ที่อ่านได้
  - Counts per revolution ที่คำนวณได้
  - Max linear velocity และ angular velocity ของหุ่นยนต์
- **นำค่า COUNTS PER REVOLUTION ที่ได้ไปอัพเดทใน `config/luna_robot.h`**

ตัวอย่าง output ของ LUNA:
```
================MOTOR ENCODER READINGS================
FRONT LEFT - M1: 12310  FRONT RIGHT - M2: 10568
REAR LEFT  - M3: 11440  REAR RIGHT  - M4: 10700

================COUNTS PER REVOLUTION=================
FRONT LEFT - M1: 2443   FRONT RIGHT - M2: 2102
REAR LEFT  - M3: 2276   REAR RIGHT  - M4: 2139

====================MAX VELOCITIES====================
Linear Velocity:  +- 0.16 m/s
Angular Velocity: +- 1.65 rad/s
```

---

### คำสั่ง `ticks` — หา Counts Per Revolution แบบ manual

```
> ticks
```

ใช้เมื่ออยากวัดด้วยตัวเอง:

1. มอเตอร์แต่ละตัวจะหมุนช้า ๆ
2. **นับจำนวนรอบที่ล้อหมุนด้วยตาตัวเอง** แล้วจด
3. อ่านค่า `final_tick_count` ที่พิมพ์ออกมา
4. คำนวณ: `COUNTS_PER_REV = final_tick_count ÷ จำนวนรอบที่นับ`
5. นำค่าที่ได้ไปใส่ใน `config/luna_robot.h`

---

### คำสั่ง `test` — ทดสอบ PID ทีละมอเตอร์

```
> test
```

- ทดสอบ PID controller โดยสั่ง `cmd_vel` ที่ `linear.x = 0.5 m/s`
- วิ่งมอเตอร์ทีละตัว เป็นเวลา **5 วินาที** ต่อตัว
- พิมพ์ค่า `req_rpm`, `current_rpm`, `pwm` ทุก 100ms

ตัวอย่าง output:
```
req_rpm:: 24.39  current_rpm:: 23.80  pwm:: 145
req_rpm:: 24.39  current_rpm:: 24.41  pwm:: 142
```

ถ้า `current_rpm` ห่างจาก `req_rpm` มาก ให้ปรับค่า PID ใน `config/luna_robot.h`:
- เพิ่ม `K_P` ถ้า response ช้าเกินไป
- เพิ่ม `K_I` ถ้ามี steady-state error
- เพิ่ม `K_D` ถ้า overshoot มาก

---

### คำสั่ง `testall` — ทดสอบ 4 มอเตอร์พร้อมกัน + Serial Plotter

```
> testall
```

- รันมอเตอร์ทั้ง 4 พร้อมกันเป็นเวลา **5 วินาที**
- พิมพ์ CSV format: `M1_req, M1_cur, M1_pwm, M2_req, M2_cur, M2_pwm, ...`
- กดปุ่มใดก็ได้เพื่อหยุด

**ดูกราฟใน Serial Plotter:**
1. ใน VS Code PlatformIO → เปิด **Serial Plotter** (ไม่ใช่ Serial Monitor)
2. พิมพ์ `testall` แล้ว Enter
3. จะเห็นกราฟ RPM ของแต่ละมอเตอร์แบบ realtime

---

### `plot_motors.py` — วิเคราะห์ PID Step Response

Script Python สำหรับดูกราฟ PID response ของมอเตอร์ทั้ง 4 พร้อมกัน พร้อมคำนวณ metrics อัตโนมัติ

**ติดตั้ง dependencies (ครั้งแรกครั้งเดียว):**
```bash
pip install pyserial numpy matplotlib
```

**วิธีใช้งาน:**

แบบที่ 1 — เชื่อมต่อ serial อัตโนมัติ (upload test firmware ก่อน):
```bash
cd test
python3 plot_motors.py
```
Script จะส่งคำสั่ง `testall` ให้บอร์ดเองและเก็บข้อมูลอัตโนมัติ

แบบที่ 2 — อ่านจากไฟล์ที่บันทึกไว้:
```bash
python3 plot_motors.py motor_log.txt
```

**ผลลัพธ์ที่ได้:**
- กราฟ RPM step response ของ Motor 1–4 แยกกัน
- คำนวณ metrics ทุก motor:
  - **Rise Time** — เวลาที่ RPM ไต่จาก 10% → 90% ของ target
  - **Peak Time** — เวลาที่ RPM ถึงจุดสูงสุด
  - **Settling Time** — เวลาที่ RPM เข้าสู่ช่วง ±5% ของ target
  - **Overshoot** — % ที่ RPM เกิน target
  - **Steady State Error** — % ความคลาดเคลื่อนตอนนิ่ง

ใช้ผลนี้ตัดสินใจปรับค่า PID ใน `config/luna_robot.h`

**ตัวอย่างกราฟ:**

![PID Response](docs/images/pid_response.png)

**ผลตอบสนอง PID ที่ดีที่สุด (Control Performance):**

![Best PID Response](docs/images/goodpidresponse.png)

---

### คำสั่ง `imu` — ทดสอบ IMU

```
> imu
```

- Initialize MPU6050 บน I2C (SDA=2, SCL=1)
- Calibrate gyroscope อัตโนมัติ (วางนิ่ง ๆ ประมาณ 2 วินาที)
- Stream ข้อมูล 20 Hz ในรูปแบบ CSV:

```
Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z
-0.0123,0.0456,9.8100,0.0001,-0.0002,0.0000
```

หน่วย:
- Accelerometer → **m/s²**
- Gyroscope → **rad/s** (หักค่า bias แล้ว)

กดปุ่มใดก็ได้ใน Serial Monitor เพื่อหยุด

**ตรวจสอบผล:**
- วางนิ่งบนพื้นราบ: `Accel_Z` ควรใกล้ **9.81** และ `Accel_X`, `Accel_Y` ใกล้ **0**
- Gyro ทั้ง 3 แกนควรใกล้ **0** เมื่อไม่มีการหมุน
- ถ้า IMU init ล้มเหลว ให้ตรวจสาย SDA/SCL และ I2C address

---

## 6. ลำดับการ Setup หุ่นยนต์ครั้งแรก

```
1. ติดตั้ง PlatformIO
2. แก้ config/luna_robot.h ให้ตรงกับ pin และ hardware
3. Upload test firmware → ทดสอบ IMU ด้วยคำสั่ง imu
4. Upload test firmware → ทดสอบมอเตอร์ด้วยคำสั่ง spin
5. วัด COUNTS_PER_REV ด้วยคำสั่ง sample หรือ ticks
6. อัพเดทค่า COUNTS_PER_REV ใน config/luna_robot.h
7. ปรับ PID ด้วยคำสั่ง test และ testall จนกว่า current_rpm ตาม req_rpm ได้ดี
8. Upload main firmware และเชื่อมต่อ micro-ROS Agent
```

---

## 7. แก้ปัญหาที่พบบ่อย

| ปัญหา | สาเหตุ | แนวทางแก้ |
|---|---|---|
| Upload ไม่ได้ — `Permission denied /dev/ttyACM0` | ยังไม่มีสิทธิ์ USB | รัน `sudo usermod -aG dialout $USER` แล้ว reboot |
| Upload ไม่ได้ — port ไม่เจอ | ยังไม่ได้กด Boot button | กดค้าง **BOOT** บน ESP32-S3 ตอนกด Upload |
| `imu` command พิมพ์ ERROR | สาย I2C ผิด หรือ address ผิด | ตรวจ SDA/SCL pin ใน `luna_robot.h` และตรวจสายต่อ |
| micro-ROS Agent ต่อไม่ได้ | baud rate ไม่ตรง | ตรวจ `monitor_speed` ใน `main/platformio.ini` ต้องเป็น 921600 |
| Build error `#include errors` | library ยังไม่ได้ดาวน์โหลด | รัน `pio pkg install` หรือ Build ครั้งแรกรอสักครู่ |
| LED ไม่กะพริบหลัง upload | micro-ROS Agent ยังไม่ได้รัน | รัน agent บน PC ก่อน ESP32 จะเชื่อมต่อและ LED จะกะพริบ |
