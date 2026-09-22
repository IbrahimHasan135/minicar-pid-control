# Mini Car Control System Architecture
## ESP32 DevKit V1 - ESP-IDF 5.4.4 + C++ + FreeRTOS

## 1. Purpose

Dokumen ini menjelaskan arsitektur software mini car untuk platform pengambilan data indoor navigation.

Fokus architecture ini adalah fondasi motion control yang:

- menerima command secara modular,
- menjalankan control loop yang stabil,
- menjaga boundary hardware tetap jelas,
- mudah diganti driver-nya tanpa mengubah bahasa layer atas,
- tidak mencampur state machine, PID, driver, queue, dan log dalam satu tempat,
- cocok dikerjakan beberapa developer/AI dengan tanggung jawab berbeda.

Framework wajib:

```text
ESP-IDF 5.4.4
C++
FreeRTOS
```

## 2. Layer Definition

Layer resmi project:

```text
Application
    -> Logic / Task
        -> Service
            -> Driver
                -> Hardware
```

Arti setiap layer:

```text
Application
    Mendefinisikan mission atau sumber command level tinggi.

Logic / Task
    Memiliki flow runtime, queue, state machine, timing, sequencing, dan fail-safe decision.

Service
    Komponen pasif/helper dengan bahasa unit yang stabil. Service menghitung, memformat,
    mengubah raw data menjadi engineering unit, dan memberi API yang rapi ke Logic.

Driver
    Satu-satunya layer yang menyentuh hardware/peripheral ESP-IDF low-level.

Hardware
    Motor, encoder, IMU, UART, console, peripheral fisik.
```

Rule inti:

> Logic owns behavior. Service provides capability. Driver touches hardware.

## 3. High-Level Architecture

```text
Application Mission / External Command
        |
        v
MotionCommandQueue
        |
        v
logic/motion_control
    |-- MotionCommandTask   event-driven queue consumer
    |-- MotionLoopTask      fixed-rate control loop
    `-- MotionControlContext shared motion state
        |
        v
Service Layer
    |-- MotorControlService
    |-- HeadingService
    |-- OdometryService
    |-- PIDController
    |-- CommunicationService
    `-- TelemetryService
        |
        v
Driver Layer
    |-- MotorEncoderDriver
    |-- IMUDriver
    |-- SerialDriver
    `-- ConsoleDriver
        |
        v
Hardware
```

## 4. Why There Is No MotionService

Architecture ini sengaja tidak memakai `MotionService` sebagai facade aktif.

Alasannya:

- motion state machine adalah runtime behavior,
- command sequencing adalah responsibility Task/Logic,
- completion rule dan fail-safe adalah control flow,
- periodic update harus jelas berada di loop task,
- service sebaiknya pasif agar tetap menjadi bahasa stabil di atas driver.

Jadi motion logic berada di:

```text
main/logic/motion_control/
```

Bukan di service.

## 5. Application Layer

Application hanya menghasilkan command level tinggi.

Contoh:

```cpp
Mission::enqueueDemoMission(motionQueue);
```

Application boleh tahu:

- `MotionCommand`,
- motion queue,
- urutan mission.

Application tidak boleh tahu:

- PWM,
- GPIO,
- PCNT,
- encoder ticks detail,
- IMU register,
- PID,
- motor output,
- FreeRTOS task internal,
- driver object.

Application flow:

```text
Application
    -> MotionCommandQueue
```

## 6. Motion Command Model

Shared model berada di `main/model`.

```cpp
enum class MotionType : uint8_t {
    MOVE_DISTANCE,
    TURN_ANGLE,
    STOP,
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
    value = distance_m
    speed = speed_mps

TURN_ANGLE:
    value = relative angle_deg
    speed = unused for now

STOP:
    value = unused
    speed = unused
```

Sign convention awal:

```text
MOVE +distance_m = forward
MOVE -distance_m = reverse
TURN +angle_deg = right
TURN -angle_deg = left
```

## 7. Logic / Task Layer

Logic layer memiliki flow runtime.

Task utama:

```text
logic/motion_control/MotionCommandTask
logic/motion_control/MotionLoopTask
logic/SerialTask
logic/TelemetryTask
```

### 7.1 MotionControl Module

Motion control logic berada dalam satu module:

```text
main/logic/motion_control/
```

Module ini adalah jawaban utama jika ditanya:

> Siapa yang handle motion logic?

Jawabannya:

> Motion control module. Di dalamnya ada satu task event queue dan satu task periodic loop.

### 7.2 MotionCommandTask

`MotionCommandTask` bersifat event-driven.

Responsibility:

- wait `MotionCommandQueue`,
- menerima command dari Application/Serial/Future Navigation,
- submit command ke `MotionControlContext`,
- menjaga urutan command,
- tidak menjalankan PID,
- tidak membaca driver,
- tidak menghitung odometry.

Blocking `portMAX_DELAY` diperbolehkan di sini karena task ini memang menunggu event command.

### 7.3 MotionLoopTask

`MotionLoopTask` adalah control loop periodik.

Initial rate:

```text
100 Hz
10 ms period
```

Responsibility:

- update heading service,
- refresh motor feedback,
- update odometry service,
- start pending command dari context,
- menjalankan move/turn state handling,
- menentukan target wheel velocity,
- memanggil motor control apply,
- completion detection,
- fail-safe stop.

`MotionLoopTask` boleh memanggil service, tetapi tidak boleh memanggil driver langsung.

Control loop menggunakan:

```cpp
vTaskDelayUntil(...)
```

### 7.4 MotionControlContext

`MotionControlContext` menyimpan shared motion state antara `MotionCommandTask` dan `MotionLoopTask`.

Isi konsep:

- active state,
- active command,
- pending command,
- stop request,
- target distance,
- target speed,
- start distance,
- target heading.

Akses context harus thread-safe dengan critical section pendek. Jangan melakukan driver I/O, serial I/O, atau log blocking ketika lock aktif.

## 8. Service Layer

Service adalah component/capability pasif.

Service boleh:

- menggunakan driver yang dimilikinya,
- menyimpan state internal yang menjadi bagian capability,
- menghitung unit engineering,
- menjalankan PID helper,
- expose API kecil untuk logic.

Service tidak boleh:

- membuat task,
- punya command queue,
- memiliki mission sequencing,
- menjadi motion state machine owner,
- mengakses driver milik service lain,
- melakukan delay/busy wait.

### 8.1 MotorControlService

Owner tunggal `MotorEncoderDriver`.

Responsibility:

- refresh encoder/motor feedback,
- convert RPM/ticks menjadi engineering unit,
- menyimpan target left/right wheel speed,
- menjalankan speed PID kiri/kanan,
- apply output motor via driver,
- stop motor.

Tidak boleh:

- membaca IMU,
- tahu `MotionCommandQueue`,
- menentukan move/turn selesai,
- menjalankan motion sequencing.

### 8.2 HeadingService

Owner tunggal `IMUDriver`.

Responsibility:

- update heading relatif,
- expose heading/yaw rate,
- normalize angle,
- calculate heading correction dengan PID.

Tidak boleh:

- akses motor driver,
- menjalankan turn state machine,
- tahu mission queue.

### 8.3 OdometryService

Pure computational service.

Tidak punya driver.

Input diberikan oleh Logic dari data service lain:

```cpp
updateOdometry(left_ticks, right_ticks, heading_deg);
```

Responsibility:

- hitung travelled distance,
- hitung `Pose2D`,
- reset odometry state.

### 8.4 PIDController

Helper generic.

Tidak tahu:

- motor,
- encoder,
- IMU,
- degree,
- navigation,
- task.

Wajib ada:

- output clamp,
- integral clamp,
- reset.

### 8.5 CommunicationService

Owner tunggal `SerialDriver`.

Responsibility:

- parse protocol/frame external command,
- validate command format,
- menghasilkan `MotionCommand`.

Tidak boleh langsung mengontrol motor.

### 8.6 TelemetryService

Owner tunggal `ConsoleDriver`.

Responsibility:

- format telemetry message,
- publish via console driver.

Telemetry tetap asynchronous melalui `TelemetryTask` dan `TelemetryQueue`.

## 9. Driver Layer

Driver hanya hardware.

Driver boleh:

- configure GPIO,
- configure LEDC/PWM,
- configure PCNT,
- configure UART,
- configure I2C/SPI,
- read/write peripheral,
- expose raw/hardware-near data.

Driver tidak boleh:

- menjalankan PID,
- membaca `MotionCommand`,
- membuat queue,
- menjalankan motion state machine,
- menentukan target distance/heading,
- menyimpan mission.

Driver skeleton saat ini disediakan sebagai kontrak API. Implementasi hardware detail dikerjakan setelah pin/peripheral final jelas.

## 10. Ownership Rule

Satu driver hanya dimiliki satu service.

```text
MotorEncoderDriver -> MotorControlService
IMUDriver          -> HeadingService
SerialDriver       -> CommunicationService
ConsoleDriver      -> TelemetryService
```

Dilarang:

```text
MotionLoopTask -> MotorEncoderDriver
TelemetryTask  -> MotorEncoderDriver
OdometryService -> MotorEncoderDriver
```

## 11. Queue Architecture

Queue utama:

```text
MotionCommandQueue
TelemetryQueue
```

Motion command flow:

```text
Application / Serial / Future Navigation
    -> MotionCommandQueue
    -> MotionCommandTask
    -> MotionControlContext
    -> MotionLoopTask
    -> Services
    -> Drivers
```

Telemetry flow:

```text
Producer
    -> TelemetryQueue
    -> TelemetryTask
    -> TelemetryService
    -> ConsoleDriver
```

Telemetry producer high-priority harus non-blocking:

```cpp
xQueueSend(queue, &message, 0);
```

## 12. Startup Flow

`app_main()` melakukan:

1. create drivers,
2. create services dengan dependency injection,
3. create `MotionControlContext`,
4. create queues,
5. init services,
6. create tasks,
7. enqueue optional demo mission.

## 13. Folder Tree

```text
main/
|-- app_main.cpp
|-- application/
|-- logic/
|   |-- motion_control/
|   |   |-- MotionCommandTask.*
|   |   |-- MotionLoopTask.*
|   |   `-- MotionControlContext.*
|   |-- SerialTask.*
|   `-- TelemetryTask.*
|-- service/
|-- driver/
|-- model/
`-- config/
```

## 14. Safety Defaults

Jika terjadi:

- driver belum siap,
- command invalid,
- dt invalid,
- IMU/motor feedback tidak valid,
- control path error,

maka default behavior:

```text
STOP MOTOR
set ERROR or IDLE according to condition
```

Jangan mempertahankan output motor terakhir tanpa feedback valid.

## 15. Architecture Summary

Final rule:

```text
Application defines intent.
Logic/Task owns runtime behavior.
Service provides passive capability.
Driver touches hardware.
```

Tidak ada `MotionService` aktif. Motion logic berada di `logic/motion_control`.
