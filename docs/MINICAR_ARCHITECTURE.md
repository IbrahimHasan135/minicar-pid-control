# Mini Car Control System Architecture
## ESP32 DevKit V1 — ESP-IDF + C++ + FreeRTOS

## 1. Purpose

Dokumen ini mendefinisikan arsitektur software untuk mini car yang digunakan sebagai platform pengambilan data pada project indoor navigation.

Fokus dokumen ini **bukan** algoritma indoor navigation, melainkan fondasi mini car yang harus mampu:

- bergerak maju dengan jarak yang terukur,
- menjaga kecepatan sesuai target,
- menjaga arah agar tidak drifting,
- melakukan belokan dengan sudut yang presisi,
- menerima command secara modular,
- mudah dikembangkan,
- mudah diuji,
- tidak memiliki code yang saling bertumpuk,
- tidak memiliki akses hardware yang tersebar,
- dapat ditambahkan komunikasi serial/Zigbee tanpa merusak control architecture.

Framework yang digunakan:

- **ESP-IDF**
- **C++**
- **FreeRTOS**

Paradigma utama:

```text
Application
    ↓
Logic / FreeRTOS
    ↓
Service / OOP
    ↓
Driver / OOP
    ↓
Hardware
```

Prinsip terpenting:

> Task menentukan kapan sesuatu dijalankan.  
> Service menentukan bagaimana sesuatu dilakukan.  
> Driver hanya berkomunikasi dengan hardware.

---

# 2. Design Goals

Architecture harus memenuhi aturan berikut.

1. Driver dan Service menggunakan konsep OOP.
2. Logic menggunakan FreeRTOS Task.
3. `ControlTask` hanya mengenal `MotionService`.
4. `MotionService` menjadi satu-satunya facade/pintu kontrol gerakan.
5. Satu hardware driver hanya boleh dimiliki atau diakses oleh satu service.
6. Task tidak boleh mengakses driver secara langsung.
7. Application tidak boleh mengakses driver atau low-level service.
8. PID tidak berada di Logic.
9. PID merupakan bagian/helper pada Service Layer.
10. Odometry tidak boleh membaca driver secara langsung.
11. Logging harus melewati Telemetry Queue.
12. Control loop tidak boleh blocking karena logging.
13. Motion command dikirim melalui Queue.
14. Komunikasi serial/Zigbee tidak boleh langsung mengontrol motor.
15. Semua motion harus dapat diperintah melalui interface sederhana.
16. Tidak boleh ada "sandwich code", yaitu satu function/class yang memuat seluruh proses sensor, PID, hardware, state machine, logging, dan command sekaligus.
17. Dependency harus bergerak turun sesuai layer dan tidak lompat antar branch.

---

# 3. High-Level Architecture

```text
┌──────────────────────────────────────────────┐
│                  APPLICATION                 │
│                                              │
│ Mission Definition                           │
│                                              │
│ MOVE 1.0 m @ 1.0 m/s                         │
│ TURN RIGHT 90 deg                            │
│ MOVE 2.0 m @ 0.5 m/s                         │
└─────────────────────┬────────────────────────┘
                      │
                      │ push MotionCommand
                      ▼
┌──────────────────────────────────────────────┐
│                 LOGIC LAYER                  │
│                 FreeRTOS                     │
│                                              │
│ MotionTask                                   │
│ ControlTask                                  │
│ SerialTask                                   │
│ TelemetryTask                                │
└─────────────────────┬────────────────────────┘
                      │
                      ▼
┌──────────────────────────────────────────────┐
│                SERVICE LAYER                 │
│                                              │
│ MotionService  ← main facade                 │
│ MotorControlService                          │
│ HeadingService                               │
│ OdometryService                              │
│ PIDController                                │
│ CommunicationService                         │
│ TelemetryService                             │
└─────────────────────┬────────────────────────┘
                      │
                      ▼
┌──────────────────────────────────────────────┐
│                 DRIVER LAYER                 │
│                                              │
│ MotorEncoderDriver                           │
│ IMUDriver                                    │
│ SerialDriver                                 │
│ ConsoleDriver                                │
└─────────────────────┬────────────────────────┘
                      │
                      ▼
                  HARDWARE
```

---

# 4. Dependency Direction

Dependency utama harus selalu:

```text
Application
    ↓
Logic
    ↓
Service
    ↓
Driver
    ↓
Hardware
```

Dependency berikut **dilarang**:

```text
Driver      → Service
Driver      → Logic

Service     → Logic

Application → Driver
Application → PIDController
Application → IMUDriver
Application → MotorEncoderDriver

Task        → Hardware Driver
```

Service boleh menggunakan service lain **hanya jika hubungan tersebut sudah ditentukan secara eksplisit oleh architecture**.

---

# 5. Application Layer

Application Layer hanya mendeskripsikan mission atau command yang diinginkan.

Contoh:

```cpp
Mission::pushMove(1.0f, 1.0f);
Mission::pushTurn(90.0f);
Mission::pushMove(2.0f, 0.5f);
```

Application tidak mengetahui:

- PWM,
- encoder ticks,
- GPIO,
- PCNT,
- IMU register,
- PID,
- UART,
- heading correction,
- FreeRTOS synchronization internal,
- implementation detail motor.

Application hanya menghasilkan `MotionCommand`.

---

# 6. Motion Command Model

Gunakan struktur data bersama pada folder `model`.

Contoh:

```cpp
enum class MotionType : uint8_t {
    MOVE_DISTANCE,
    TURN_ANGLE,
    STOP
};

struct MotionCommand {
    MotionType type;

    float value;
    float speed;
};
```

Interpretasi:

```text
MOVE_DISTANCE:
    value = distance meter
    speed = meter per second

TURN_ANGLE:
    value = relative angle degree
    speed = optional angular control parameter

STOP:
    value = unused
    speed = unused
```

Contoh mission:

```text
MOVE_DISTANCE
value = 1.0
speed = 1.0
```

kemudian:

```text
TURN_ANGLE
value = +90.0
```

kemudian:

```text
MOVE_DISTANCE
value = 2.0
speed = 0.5
```

---

# 7. Logic Layer

Logic Layer berisi FreeRTOS task dan orchestration antar thread.

Task utama:

```text
ControlTask
MotionTask
SerialTask
TelemetryTask
```

Tidak perlu membuat task terpisah untuk:

```text
EncoderTask
MotorTask
PIDTask
IMUTask
HeadingTask
OdometryTask
```

Hal tersebut akan menambah synchronization complexity secara tidak perlu.

---

# 8. ControlTask

## Responsibility

`ControlTask` menjalankan control loop periodik.

ControlTask **hanya boleh memanggil MotionService**.

Contoh:

```cpp
motion.update(dt);
```

ControlTask tidak boleh memanggil:

```cpp
motorDriver.update();
imuDriver.read();
headingService.update();
motorControlService.setVelocity();
pid.update();
```

semua detail tersebut menjadi responsibility dari Service Layer.

## Suggested Frequency

Initial recommendation:

```text
100 Hz
10 ms cycle
```

Contoh konseptual:

```cpp
void ControlTask(void* arg)
{
    TickType_t lastWake = xTaskGetTickCount();

    while (true)
    {
        motionService.update(0.01f);

        vTaskDelayUntil(
            &lastWake,
            pdMS_TO_TICKS(10)
        );
    }
}
```

Final frequency dapat disesuaikan setelah testing motor, encoder, dan IMU.

---

# 9. MotionTask

`MotionTask` menangani Motion Command Queue.

Flow:

```text
Application
      │
      ▼
MotionCommandQueue
      │
      ▼
MotionTask
      │
      ▼
MotionService
```

MotionTask tidak melakukan PID.

MotionTask tidak menghitung odometry.

MotionTask tidak membaca sensor.

MotionTask hanya:

1. menunggu command,
2. membaca status MotionService,
3. mengirim command baru saat MotionService siap.

Concept:

```cpp
while (true)
{
    if (!motionService.isBusy())
    {
        if (xQueueReceive(
            motionQueue,
            &command,
            portMAX_DELAY
        ))
        {
            execute(command);
        }
    }
}
```

Kemudian:

```cpp
switch (command.type)
{
case MotionType::MOVE_DISTANCE:
    motionService.startMove(
        command.value,
        command.speed
    );
    break;

case MotionType::TURN_ANGLE:
    motionService.startTurn(
        command.value
    );
    break;

case MotionType::STOP:
    motionService.stop();
    break;
}
```

---

# 10. SerialTask

SerialTask disiapkan untuk komunikasi dengan ESP32 lain, termasuk kemungkinan node Zigbee.

SerialTask tidak boleh:

```text
UART → motor
UART → PWM
UART → MotionService direct command bypass queue
```

Flow yang benar:

```text
External ESP32 / Zigbee
        │
       UART
        │
        ▼
    SerialTask
        │
        ▼
CommunicationService
        │
        ▼
parsed MotionCommand
        │
        ▼
 MotionCommandQueue
        │
        ▼
    MotionTask
```

Dengan cara ini sumber command dapat berasal dari:

```text
Application Mission ─┐
                     │
Serial / Zigbee ─────┼──→ MotionCommandQueue
                     │
Future Navigation ───┘
```

Control architecture tidak perlu berubah.

---

# 11. TelemetryTask

Semua log yang tidak membutuhkan output immediate harus masuk melalui Telemetry Queue.

Flow:

```text
Service / Task
     │
     │ push telemetry
     ▼
TelemetryQueue
     │
     ▼
TelemetryTask
     │
     ▼
TelemetryService
     │
     ▼
ConsoleDriver
     │
     ▼
UART / Console
```

Control loop tidak boleh menunggu telemetry.

Jika queue penuh, log non-critical boleh dibuang.

Prinsip:

> Telemetry boleh drop. Control cycle tidak boleh terlambat karena telemetry.

---

# 12. Service Layer Overview

Service utama:

```text
Service
├── MotorControlService
├── HeadingService
├── OdometryService
├── MotionService
├── CommunicationService
└── TelemetryService
```

Helper:

```text
PIDController
```

`PIDController` bukan Task.

`PIDController` juga tidak wajib menjadi child dari Service.

Secara konsep:

```text
MotorControlService
├── leftSpeedPID
└── rightSpeedPID

HeadingService
└── headingPID
```

---

# 13. Base Service

Base class dapat digunakan sebagai interface umum.

Contoh:

```cpp
class Service {
public:
    virtual esp_err_t init() = 0;
    virtual void reset() = 0;

    virtual ~Service() = default;
};
```

Base class tidak boleh memiliki terlalu banyak logic.

Tujuannya hanya memberikan lifecycle/interface yang konsisten.

---

# 14. MotorEncoderDriver

Motor dan encoder digabung menjadi satu driver karena merupakan satu subsystem hardware.

Driver:

```text
MotorEncoderDriver
```

Responsibility:

- configure motor GPIO,
- configure PWM,
- configure motor direction,
- initialize encoder counter,
- read encoder ticks,
- calculate or expose encoder raw values,
- provide motor output command,
- stop motor hardware.

Driver tidak mengetahui:

- PID,
- distance meter,
- heading,
- turn 90°,
- MOVE command,
- navigation,
- odometry pose.

Concept interface:

```cpp
class MotorEncoderDriver {
public:
    esp_err_t init();

    void setLeftOutput(float output);
    void setRightOutput(float output);

    int32_t getLeftTicks() const;
    int32_t getRightTicks() const;

    float getLeftRPM() const;
    float getRightRPM() const;

    void stop();
};
```

---

# 15. One Driver — One Service Rule

Satu driver hanya boleh diakses oleh satu service.

## Motor / Encoder

```text
MotorEncoderDriver
       │
       ▼
MotorControlService
```

Hanya `MotorControlService` yang boleh memiliki reference/pointer ke `MotorEncoderDriver`.

Tidak boleh:

```text
MotorEncoderDriver
   ├── MotorControlService
   ├── OdometryService
   └── MotionService
```

---

# 16. IMU Driver

Driver:

```text
IMUDriver
```

Responsibility:

- initialize I2C/SPI sensor,
- read gyro,
- read accelerometer jika diperlukan,
- read magnetometer jika tersedia,
- expose raw/calibrated sensor values.

IMUDriver tidak melakukan:

- turn state machine,
- heading PID,
- motion command,
- motor correction.

Ownership:

```text
IMUDriver
    │
    ▼
HeadingService
```

Hanya `HeadingService` yang boleh mengakses IMUDriver.

---

# 17. Sensor Heading Strategy

Untuk indoor mini car, magnetometer tidak disarankan sebagai satu-satunya heading source karena dapat terganggu oleh:

- motor DC,
- permanent magnet pada motor,
- kabel arus tinggi,
- rangka logam,
- meja atau struktur besi,
- perangkat elektronik sekitar.

Recommended architecture:

```text
Gyroscope
   +
Encoder information
   +
optional magnetometer correction
        │
        ▼
HeadingService
```

Pada implementasi awal, gyroscope yaw dapat menjadi sumber utama heading relatif.

---

# 18. MotorControlService

`MotorControlService` adalah satu-satunya service yang berkomunikasi dengan `MotorEncoderDriver`.

Structure:

```text
MotorControlService
│
├── MotorEncoderDriver&
│
├── PIDController leftSpeedPID
│
└── PIDController rightSpeedPID
```

Responsibility:

- convert wheel target velocity menjadi motor command,
- membaca feedback encoder melalui driver,
- menghitung actual wheel velocity,
- menjalankan PID roda kiri,
- menjalankan PID roda kanan,
- expose encoder/velocity state ke MotionService.

Concept:

```cpp
class MotorControlService : public Service {
protected:
    MotorEncoderDriver& driver_;

    PIDController leftSpeedPID_;
    PIDController rightSpeedPID_;

    void setVelocity(
        float leftMps,
        float rightMps
    );

    void updateMotorControl(float dt);

    float getLeftVelocity() const;
    float getRightVelocity() const;

    int32_t getLeftTicks() const;
    int32_t getRightTicks() const;

    void stopMotor();
};
```

Methods yang hanya digunakan oleh `MotionService` sebaiknya dibuat `protected`.

---

# 19. Speed PID

Mini car differential drive memiliki dua wheel velocity controller.

```text
Target Left Velocity
        │
        ▼
    Left PID
        │
        ▼
 Left Motor PWM
        │
        ▼
 Left Encoder
        │
        └──── feedback


Target Right Velocity
        │
        ▼
    Right PID
        │
        ▼
Right Motor PWM
        │
        ▼
Right Encoder
        │
        └──── feedback
```

PID berada di Service Layer.

PID bukan Task.

---

# 20. PIDController Helper

PID dibuat reusable dan tidak mengetahui motor maupun heading.

Contoh:

```cpp
class PIDController {
public:
    PIDController(
        float kp,
        float ki,
        float kd
    );

    float update(
        float setpoint,
        float measurement,
        float dt
    );

    void reset();

    void setOutputLimit(
        float minOutput,
        float maxOutput
    );
};
```

MotorControlService dapat memiliki:

```cpp
PIDController leftSpeedPID_;
PIDController rightSpeedPID_;
```

HeadingService dapat memiliki:

```cpp
PIDController headingPID_;
```

---

# 21. HeadingService

`HeadingService` adalah satu-satunya service yang mengakses IMUDriver.

Structure:

```text
HeadingService
│
├── IMUDriver&
│
└── PIDController headingPID
```

Responsibility:

- membaca sensor heading,
- menghitung heading relative,
- menangani angle normalization,
- memberikan current heading,
- menghitung heading correction,
- melakukan reset heading reference.

Concept:

```cpp
class HeadingService : public Service {
protected:
    IMUDriver& imu_;

    PIDController headingPID_;

    void updateHeading(float dt);

    float getHeading() const;

    float calculateHeadingCorrection(
        float targetHeading,
        float dt
    );

    void resetHeadingReference();
};
```

---

# 22. Heading Normalization

Angle harus dinormalisasi agar error tidak salah saat melewati 0° / 360°.

Contoh:

```text
Current = 359°
Target  = 1°
```

Error seharusnya:

```text
+2°
```

bukan:

```text
-358°
```

Gunakan helper normalization ke range seperti:

```text
[-180°, +180°]
```

---

# 23. OdometryService

OdometryService adalah pure computational service.

OdometryService **tidak memiliki driver**.

Input berasal dari MotionService.

Concept:

```cpp
class OdometryService : public Service {
protected:
    void updateOdometry(
        int32_t leftTicks,
        int32_t rightTicks,
        float heading
    );

    float getDistance() const;

    Pose2D getPose() const;

    void resetOdometry();
};
```

Flow:

```text
MotorEncoderDriver
       │
       ▼
MotorControlService
       │
       │ encoder state
       ▼
MotionService
       │
       ▼
OdometryService
```

Heading:

```text
IMUDriver
   │
   ▼
HeadingService
   │
   ▼
MotionService
   │
   ▼
OdometryService
```

Tidak boleh:

```text
OdometryService ───→ MotorEncoderDriver
```

---

# 24. Pose2D

Odometry dapat menyimpan:

```cpp
struct Pose2D {
    float x;
    float y;
    float heading;
};
```

Walaupun navigation belum menjadi fokus saat ini, abstraction ini disiapkan agar architecture tidak perlu diubah ketika sistem berkembang.

---

# 25. MotionService

`MotionService` adalah facade utama seluruh motion subsystem.

Public API harus sederhana.

Contoh:

```cpp
class MotionService :
    protected MotorControlService,
    protected HeadingService,
    protected OdometryService
{
public:
    esp_err_t init();

    void update(float dt);

    bool startMove(
        float distanceMeter,
        float speedMps
    );

    bool startTurn(
        float angleDegree
    );

    void stop();

    bool isBusy() const;
    bool isCompleted() const;

    MotionState getState() const;
};
```

Protected inheritance digunakan agar low-level API tidak terekspos ke application/task.

Contoh yang **tidak boleh bisa dilakukan dari luar**:

```cpp
motionService.setVelocity(...);
motionService.updateOdometry(...);
motionService.calculateHeadingCorrection(...);
```

Public API cukup berupa motion-level command.

---

# 26. Why MotionService Is Not a God Class

MotionService bukan tempat implementasi semua algoritma.

MotionService hanya:

1. orchestration,
2. motion state machine,
3. menghubungkan output antar service,
4. menentukan target sesuai motion state.

Jangan membuat:

```cpp
void MotionService::update()
{
    // 500 lines:
    // read encoder
    // calculate RPM
    // calculate PID
    // read IMU
    // integrate gyro
    // heading PID
    // odometry
    // state machine
    // PWM
    // logging
}
```

Gunakan:

```cpp
void MotionService::update(float dt)
{
    updateHeading(dt);

    updateOdometryState();

    updateMotionState(dt);

    updateMotorControl(dt);
}
```

Masing-masing detail tetap berada di service/helper yang sesuai.

---

# 27. Motion State Machine

Recommended states:

```cpp
enum class MotionState : uint8_t {
    IDLE,
    MOVING,
    TURNING,
    STOPPING,
    COMPLETED,
    ERROR
};
```

Possible transition:

```text
          ┌─────────┐
          │  IDLE   │
          └────┬────┘
               │ command
        ┌──────┴──────┐
        ▼             ▼
    MOVING          TURNING
        │             │
        │ target done │ target done
        └──────┬──────┘
               ▼
          COMPLETED
               │
               ▼
             IDLE
```

Emergency stop dapat mengarah ke:

```text
STOPPING
```

Error dapat mengarah ke:

```text
ERROR
```

---

# 28. Move Distance Control

Saat command:

```text
MOVE 1 meter @ 1 meter/second
```

MotionService:

1. reset/start distance reference,
2. capture initial heading,
3. save target distance,
4. save target speed,
5. change state to `MOVING`.

Saat setiap update:

```text
Current Heading
      │
      ▼
Heading PID
      │
      ▼
steering correction
      │
      ├───────────────┐
      ▼               ▼
Left target       Right target
velocity          velocity
      │               │
      ▼               ▼
Left Speed PID    Right Speed PID
      │               │
      ▼               ▼
Motor Left        Motor Right
```

Example:

```text
base velocity = 1.0 m/s

heading correction = 0.08 m/s

left target  = 0.92 m/s
right target = 1.08 m/s
```

Exact sign convention mengikuti motor orientation.

---

# 29. Cascade Control

Recommended structure:

```text
                 Heading Target
                       │
                       ▼
                  Heading PID
                       │
                       ▼
              Steering Correction
                  /           \
                 /             \
                ▼               ▼
        Left Velocity      Right Velocity
             Target             Target
                │                 │
                ▼                 ▼
         Left Speed PID     Right Speed PID
                │                 │
                ▼                 ▼
          Left Motor         Right Motor
```

Heading control adalah outer loop.

Wheel speed control adalah inner loop.

---

# 30. Turn Control

Untuk differential drive, initial implementation dapat menggunakan pivot turn:

```text
Left wheel  → forward
Right wheel → reverse
```

atau sebaliknya.

Saat `TURN +90°`:

```text
initial heading = current heading
target heading  = initial + 90°
```

Heading PID menentukan turn command.

Semakin dekat dengan target, command harus berkurang agar overshoot lebih kecil.

Turn selesai jika kondisi seperti berikut terpenuhi:

```text
abs(angle_error) < angle_tolerance
AND
abs(yaw_rate) < angular_velocity_tolerance
```

Dengan demikian mobil tidak dianggap selesai hanya karena melewati target sesaat.

---

# 31. Motion Update Flow

Control cycle:

```text
ControlTask
     │
     ▼
MotionService::update(dt)
     │
     ├── updateHeading(dt)
     │       │
     │       └── HeadingService → IMUDriver
     │
     ├── collect encoder state
     │       │
     │       └── MotorControlService → MotorEncoderDriver
     │
     ├── updateOdometry(...)
     │       │
     │       └── OdometryService
     │
     ├── updateMotionState(dt)
     │       │
     │       ├── MOVING
     │       ├── TURNING
     │       ├── STOPPING
     │       └── IDLE
     │
     └── updateMotorControl(dt)
             │
             └── MotorControlService
```

---

# 32. Complete Dependency Tree

```text
                         Application
                             │
                             ▼
                      MotionCommandQueue
                             │
                             ▼
                         MotionTask
                             │
                             ▼
                      MotionService
                             │
          ┌──────────────────┼──────────────────┐
          │                  │                  │
          ▼                  ▼                  ▼
 MotorControlService    HeadingService    OdometryService
          │                  │
          ▼                  ▼
 MotorEncoderDriver        IMUDriver
```

Auxiliary branch:

```text
External ESP32
     │
     ▼
SerialDriver
     │
     ▼
CommunicationService
     │
     ▼
SerialTask
     │
     ▼
MotionCommandQueue
```

Telemetry:

```text
Any allowed producer
        │
        ▼
TelemetryQueue
        │
        ▼
TelemetryTask
        │
        ▼
TelemetryService
        │
        ▼
ConsoleDriver
```

Control:

```text
ControlTask
     │
     └────→ MotionService::update(dt)
```

---

# 33. Queue Architecture

Recommended queues:

```text
MotionCommandQueue
TelemetryQueue
```

Optional future queues:

```text
CommunicationRxQueue
NavigationCommandQueue
SystemEventQueue
```

Jangan membuat queue jika direct synchronous call masih lebih sederhana dan aman.

Queue digunakan terutama saat data berpindah antar Task.

---

# 34. Telemetry Message

Hindari penggunaan string besar dan dynamic allocation.

Recommended:

```cpp
enum class LogLevel : uint8_t {
    DEBUG,
    INFO,
    WARNING,
    ERROR
};

struct TelemetryMessage {
    uint32_t timestamp;
    LogLevel level;

    char tag[16];
    char message[96];
};
```

Approximate size sekitar 120 byte tergantung alignment.

Dengan queue 20 item:

```text
~2.4 KB
```

Dengan queue 50 item:

```text
~6 KB
```

Masih masuk akal untuk ESP32 DevKit V1 selama task stack dan subsystem lain juga dikontrol.

---

# 35. Telemetry Non-Blocking Rule

Producer sebaiknya menggunakan:

```cpp
xQueueSend(
    telemetryQueue,
    &message,
    0
);
```

Jika queue penuh:

```text
drop DEBUG/INFO telemetry
```

Jangan menggunakan:

```cpp
portMAX_DELAY
```

dari ControlTask.

Critical system fault dapat memiliki jalur berbeda jika diperlukan.

---

# 36. FreeRTOS Priority Concept

Initial suggestion:

```text
ControlTask       highest application priority
MotionTask        medium-high
SerialTask        medium
TelemetryTask     low
```

Contoh awal:

```text
ControlTask     priority 10
MotionTask      priority 7
SerialTask      priority 5
TelemetryTask   priority 2
```

Nilai final harus diverifikasi bersama task ESP-IDF internal dan subsystem lain.

---

# 37. Suggested Task Rates

Initial suggestion:

```text
ControlTask
    100 Hz

MotionTask
    event-driven / queue-based

SerialTask
    event-driven or blocking UART receive

TelemetryTask
    queue-driven
```

MotionTask tidak perlu polling cepat jika dapat menggunakan Event/Queue synchronization.

---

# 38. Thread Safety

Karena MotionTask dan ControlTask sama-sama mengakses MotionService:

```text
MotionTask
    └── startMove / startTurn / stop

ControlTask
    └── update
```

shared state harus dirancang dengan hati-hati.

Pilihan implementasi:

1. critical section kecil,
2. mutex,
3. atomic field,
4. internal command handoff.

Preferensi:

- hindari mutex lama di ControlTask,
- critical section harus sesingkat mungkin,
- jangan melakukan blocking I/O saat lock aktif.

---

# 39. Configuration Layer

Semua hardware dan tuning parameter tidak boleh tersebar sebagai magic number.

Gunakan:

```text
config/
├── RobotConfig.hpp
├── PIDConfig.hpp
└── FreeRTOSConfig.hpp
```

Contoh `RobotConfig.hpp`:

```cpp
namespace RobotConfig {

constexpr float WHEEL_DIAMETER_M = ...;
constexpr float WHEEL_BASE_M = ...;
constexpr int ENCODER_TICKS_PER_REV = ...;

}
```

PID:

```cpp
namespace PIDConfig {

constexpr float SPEED_KP = ...;
constexpr float SPEED_KI = ...;
constexpr float SPEED_KD = ...;

constexpr float HEADING_KP = ...;
constexpr float HEADING_KI = ...;
constexpr float HEADING_KD = ...;

}
```

---

# 40. Recommended Folder Tree

```text
main/
│
├── app_main.cpp
│
├── application/
│   ├── Mission.cpp
│   └── Mission.hpp
│
├── logic/
│   ├── ControlTask.cpp
│   ├── ControlTask.hpp
│   ├── MotionTask.cpp
│   ├── MotionTask.hpp
│   ├── SerialTask.cpp
│   ├── SerialTask.hpp
│   ├── TelemetryTask.cpp
│   └── TelemetryTask.hpp
│
├── service/
│   ├── base/
│   │   └── Service.hpp
│   │
│   ├── control/
│   │   ├── PIDController.cpp
│   │   └── PIDController.hpp
│   │
│   ├── motion/
│   │   ├── MotionService.cpp
│   │   └── MotionService.hpp
│   │
│   ├── motor/
│   │   ├── MotorControlService.cpp
│   │   └── MotorControlService.hpp
│   │
│   ├── heading/
│   │   ├── HeadingService.cpp
│   │   └── HeadingService.hpp
│   │
│   ├── odometry/
│   │   ├── OdometryService.cpp
│   │   └── OdometryService.hpp
│   │
│   ├── telemetry/
│   │   ├── TelemetryService.cpp
│   │   └── TelemetryService.hpp
│   │
│   └── communication/
│       ├── CommunicationService.cpp
│       └── CommunicationService.hpp
│
├── driver/
│   ├── motor/
│   │   ├── MotorEncoderDriver.cpp
│   │   └── MotorEncoderDriver.hpp
│   │
│   ├── imu/
│   │   ├── IMUDriver.cpp
│   │   └── IMUDriver.hpp
│   │
│   ├── telemetry/
│   │   ├── ConsoleDriver.cpp
│   │   └── ConsoleDriver.hpp
│   │
│   └── communication/
│       ├── SerialDriver.cpp
│       └── SerialDriver.hpp
│
├── model/
│   ├── MotionCommand.hpp
│   ├── MotionState.hpp
│   ├── TelemetryMessage.hpp
│   ├── RobotState.hpp
│   └── Pose2D.hpp
│
└── config/
    ├── RobotConfig.hpp
    ├── PIDConfig.hpp
    └── FreeRTOSConfig.hpp
```

---

# 41. Future ESP-IDF Components

Setelah project stabil, folder dapat dipindahkan menjadi native ESP-IDF components:

```text
project/
│
├── main/
│   └── app_main.cpp
│
└── components/
    ├── logic/
    ├── motion_service/
    ├── motor_service/
    ├── heading_service/
    ├── odometry_service/
    ├── telemetry_service/
    ├── communication_service/
    ├── motor_encoder_driver/
    └── imu_driver/
```

Tahap awal boleh menggunakan satu component `main` agar development lebih cepat.

---

# 42. Startup Flow

Recommended startup:

```text
app_main()
   │
   ├── create hardware drivers
   │
   ├── create services
   │
   ├── initialize MotionService hierarchy
   │
   ├── create queues
   │
   ├── create tasks
   │
   └── enqueue initial mission
```

Setelah scheduler aktif:

```text
ControlTask
    └── continuously updates motion

MotionTask
    └── waits for motion commands

SerialTask
    └── waits for external communication

TelemetryTask
    └── waits for logs
```

---

# 43. Example Mission Flow

Mission:

```text
1. Move 1 meter @ 1 m/s
2. Turn right 90°
3. Move 2 meter @ 0.5 m/s
```

Application:

```text
push MOVE(1.0, 1.0)
push TURN(+90)
push MOVE(2.0, 0.5)
```

MotionTask consumes first command.

MotionService:

```text
state = MOVING
target_distance = 1.0
target_speed = 1.0
heading_target = current_heading
```

ControlTask continuously calls:

```text
MotionService.update()
```

MotionService:

```text
heading update
        ↓
encoder feedback
        ↓
odometry update
        ↓
heading correction
        ↓
wheel targets
        ↓
speed PID
        ↓
motor output
```

Saat distance selesai:

```text
state = COMPLETED
```

MotionTask mengambil:

```text
TURN(+90)
```

dan seterusnya.

---

# 44. Error Handling Concept

Service sebaiknya dapat melaporkan:

```text
NOT_INITIALIZED
SENSOR_ERROR
ENCODER_ERROR
CONTROL_ERROR
INVALID_COMMAND
TIMEOUT
```

Gunakan `esp_err_t` untuk initialization/hardware lifecycle jika sesuai.

Runtime state dapat memakai enum internal.

MotionService dapat berpindah ke:

```text
MotionState::ERROR
```

dan memerintahkan motor stop.

---

# 45. Safety Defaults

Jika terjadi:

- IMU tidak terbaca,
- encoder invalid,
- control timeout,
- motion command invalid,
- internal service error,

default behavior:

```text
STOP MOTOR
```

Jangan mempertahankan PWM terakhir tanpa valid feedback.

---

# 46. Architecture Summary

Architecture final:

```text
Application
    │
    ▼
MotionCommandQueue
    │
    ▼
MotionTask
    │
    ▼
MotionService
    │
    ├── MotorControlService
    │       │
    │       ├── Left Speed PID
    │       ├── Right Speed PID
    │       │
    │       ▼
    │   MotorEncoderDriver
    │
    ├── HeadingService
    │       │
    │       ├── Heading PID
    │       │
    │       ▼
    │     IMUDriver
    │
    └── OdometryService
            │
            └── pure computation
```

Parallel infrastructure:

```text
ControlTask
    └── MotionService.update()

SerialTask
    └── CommunicationService
            └── SerialDriver
                    ↓
             MotionCommandQueue

Telemetry producers
    ↓
TelemetryQueue
    ↓
TelemetryTask
    ↓
TelemetryService
    ↓
ConsoleDriver
```

Core principle:

> One entry point for motion.  
> One service owner per driver.  
> No direct hardware access outside its owning service.  
> Logic schedules.  
> Service controls.  
> Driver talks to hardware.
