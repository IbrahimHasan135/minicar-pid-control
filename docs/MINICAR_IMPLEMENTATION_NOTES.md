# Mini Car Implementation Notes

Catatan ini adalah pegangan implementasi repo `minicar-pid-control` setelah keputusan arsitektur baru:

```text
Application defines intent.
Logic/Task owns runtime behavior.
Service provides passive capability.
Driver touches hardware.
```

Target framework:

```text
ESP-IDF 5.4.4
C++
FreeRTOS
```

## 1. Current Architecture Decision

Tidak menggunakan `MotionService` sebagai facade aktif.

Motion logic berada di:

```text
main/logic/motion_control/
```

Isi module:

```text
MotionCommandTask
MotionLoopTask
MotionControlContext
```

Alasan:

- command sequencing adalah runtime behavior,
- move/turn state machine adalah logic,
- periodic control harus jelas berada di task loop,
- service harus tetap menjadi alat bantu pasif dengan bahasa/unit stabil,
- driver dapat diganti tanpa mengubah bahasa logic di atasnya.

## 2. Layer Roles

### Application

Folder:

```text
main/application/
```

Peran:

- mendefinisikan mission,
- push `MotionCommand` ke queue,
- tidak tahu detail service/driver.

Contoh:

```text
Mission -> MotionCommandQueue
```

### Logic / Task

Folder:

```text
main/logic/
```

Peran:

- FreeRTOS task,
- queue ownership,
- motion state machine,
- command sequencing,
- control timing,
- fail-safe decision,
- memanggil service.

Logic tidak boleh akses driver langsung.

### Service

Folder:

```text
main/service/
```

Peran:

- passive capability/helper,
- bahasa unit stabil,
- perhitungan domain kecil,
- PID helper,
- owner tunggal driver tertentu.

Service tidak boleh membuat task atau command queue.

### Driver

Folder:

```text
main/driver/
```

Peran:

- satu-satunya layer yang menyentuh hardware API,
- configure/read/write peripheral,
- expose raw/hardware-near data,
- tidak punya motion logic.

## 3. Motion Control Module

Folder:

```text
main/logic/motion_control/
```

### MotionCommandTask

Event-driven task.

Tugas:

- `xQueueReceive(MotionCommandQueue, ..., portMAX_DELAY)`,
- submit command ke `MotionControlContext`,
- menunggu sampai context siap menerima command berikutnya,
- menjaga urutan command.

Tidak melakukan:

- PID,
- odometry,
- sensor update,
- driver access.

### MotionLoopTask

Periodic task.

Tugas:

- berjalan 100 Hz awal,
- update heading,
- refresh motor feedback,
- update odometry,
- ambil pending command dari context,
- start move/turn,
- hitung wheel target,
- apply motor control,
- detect completion,
- fail-safe stop.

Loop ini adalah tempat utama move/turn control logic.

### MotionControlContext

Shared state antara command task dan loop task.

Menyimpan:

- active state,
- active command,
- pending command,
- stop request,
- target distance,
- target speed,
- start distance,
- target heading.

Context harus memakai critical section pendek. Jangan menjalankan I/O saat lock.

## 4. Service Contracts

### MotorControlService

Owner:

```text
MotorEncoderDriver
```

API concept:

```cpp
refreshFeedback();
setVelocityTargets(left_mps, right_mps);
applyControl(dt_s);
stopMotor();
getLeftTicks();
getRightTicks();
getLeftVelocityMps();
getRightVelocityMps();
```

Tugas:

- convert RPM/ticks ke unit service,
- speed PID kiri/kanan,
- clamp output,
- stop motor.

### HeadingService

Owner:

```text
IMUDriver
```

Tugas:

- update heading relatif,
- expose heading/yaw rate,
- normalize angle,
- heading PID correction.

### OdometryService

Tidak punya driver.

Input dari MotionLoopTask:

```cpp
updateOdometry(left_ticks, right_ticks, heading_deg);
```

Tugas:

- travelled distance,
- pose 2D.

### CommunicationService

Owner:

```text
SerialDriver
```

Tugas:

- parse external command menjadi `MotionCommand`.

### TelemetryService

Owner:

```text
ConsoleDriver
```

Tugas:

- format dan publish telemetry.

## 5. Driver Skeleton Boundary

Driver saat ini boleh berisi skeleton/TODO dulu.

Untuk Aranda:

- isi hardware init,
- isi read/write peripheral,
- jaga API tetap sama jika memungkinkan,
- jangan masukkan PID/state machine/queue ke driver.

Driver skeleton yang perlu diisi:

```text
MotorEncoderDriver
IMUDriver
SerialDriver
ConsoleDriver jika perlu dedicated UART
```

## 6. Runtime Flow

Startup:

```text
app_main
  -> create drivers
  -> create services
  -> create MotionControlContext
  -> create queues
  -> init services
  -> start TelemetryTask
  -> start MotionLoopTask
  -> start MotionCommandTask
  -> start SerialTask
  -> enqueue demo mission
```

Motion command flow:

```text
Application / Serial
  -> MotionCommandQueue
  -> MotionCommandTask
  -> MotionControlContext
  -> MotionLoopTask
  -> Services
  -> Drivers
```

Control loop flow:

```text
MotionLoopTask
  -> HeadingService.updateHeading(dt_s)
  -> MotorControlService.refreshFeedback()
  -> OdometryService.updateOdometry(...)
  -> start pending command if available
  -> handle MOVING/TURNING/IDLE/ERROR
  -> MotorControlService.applyControl(dt_s)
```

## 7. Queue Use

`MotionCommandQueue`:

- semua command masuk sini,
- boleh berasal dari Application, Serial, future Navigation.

`TelemetryQueue`:

- log/status asynchronous,
- producer high-priority harus timeout 0,
- telemetry boleh drop.

## 8. File Map

```text
main/application/Mission.*
    Demo mission source.

main/logic/motion_control/MotionCommandTask.*
    Queue command consumer.

main/logic/motion_control/MotionLoopTask.*
    Periodic motion control executor.

main/logic/motion_control/MotionControlContext.*
    Shared motion state.

main/logic/SerialTask.*
    External command ingestion to queue.

main/logic/TelemetryTask.*
    Telemetry queue consumer.

main/service/motor/MotorControlService.*
    Passive motor/wheel speed capability.

main/service/heading/HeadingService.*
    Passive heading capability.

main/service/odometry/OdometryService.*
    Passive odometry computation.

main/service/control/PIDController.*
    Generic PID helper.

main/driver/*
    Hardware-only skeleton/contracts.
```

## 9. Sign and Unit Convention

```text
MOVE +distance_m = forward
MOVE -distance_m = reverse
TURN +angle_deg = right
TURN -angle_deg = left
speed_mps > 0 for MOVE command speed
heading_deg normalized to [-180, 180)
```

Physical variable names should include unit suffix.

## 10. Validation Boundary

Current implementation has driver skeletons. Therefore:

- source/static checks can be done,
- build can be done only when user allows,
- hardware behavior is not validated,
- motor/IMU/serial readiness depends on driver implementation.

Do not claim movement works until real driver and hardware test pass.

## 11. Handoff Notes for Aranda or Next AI

Do not reintroduce `MotionService` as active motion brain.

When editing:

- put command/state behavior in `logic/motion_control`,
- put reusable calculation/capability in `service`,
- put peripheral access in `driver`,
- keep one driver owner per service,
- keep CMake source list updated,
- keep docs consistent if architecture changes again.

## 12. Next Driver Work

### MotorEncoderDriver

Needs:

- motor direction pin setup,
- PWM/LEDC setup,
- encoder counter setup,
- output clamp/application,
- left/right ticks,
- left/right RPM,
- safe stop.

### IMUDriver

Needs:

- I2C/SPI setup,
- IMU init,
- gyro yaw rate read,
- calibration/bias plan,
- health reporting.

### SerialDriver

Needs:

- UART setup,
- non-fragile frame/line read,
- timeout behavior.

### ConsoleDriver

Current stub uses console output. Replace only if telemetry needs dedicated UART.
