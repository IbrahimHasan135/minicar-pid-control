# Mini Car Coding Rules
## ESP32 DevKit V1 - ESP-IDF 5.4.4 + C++

## 1. Mandatory Technology

Project wajib menggunakan:

```text
ESP-IDF 5.4.4
C++
FreeRTOS
```

Jangan mengubah project menjadi Arduino Framework kecuali ada keputusan eksplisit.

## 2. Layer Rule

Layer resmi:

```text
Application
Logic / Task
Service
Driver
Hardware
```

Dependency hanya boleh turun:

```text
Application -> Logic -> Service -> Driver -> Hardware
```

Dilarang shortcut:

- Application -> Driver
- Task -> Driver
- Driver -> Service
- Driver -> Logic
- Service -> Logic
- Service -> driver milik service lain

## 3. Responsibility Rule

### Application

Application hanya membuat command/mission.

Boleh:

- push `MotionCommand` ke queue,
- menentukan urutan mission demo,
- menjadi source command level tinggi.

Tidak boleh:

- akses service,
- akses driver,
- tahu PID,
- tahu GPIO/PWM/encoder/IMU detail.

### Logic / Task

Logic memiliki behavior runtime.

Boleh:

- membuat/consume queue,
- menjalankan FreeRTOS task,
- menyimpan state machine,
- menentukan sequencing,
- menentukan completion rule,
- memanggil service.

Tidak boleh:

- akses driver langsung,
- menaruh GPIO/UART/I2C API,
- melakukan hardware-specific detail,
- membuat god task ratusan baris tanpa helper.

### Service

Service adalah passive capability/component.

Boleh:

- punya satu driver owner,
- menghitung unit engineering,
- menjalankan PID helper,
- menyimpan state internal capability,
- expose API kecil ke logic.

Tidak boleh:

- memiliki mission queue,
- membuat FreeRTOS task,
- menjadi owner motion state machine,
- menjalankan blocking movement,
- akses driver milik service lain.

### Driver

Driver hanya hardware.

Boleh:

- configure peripheral,
- read/write hardware,
- expose raw/hardware-near data,
- return `esp_err_t`.

Tidak boleh:

- PID,
- queue,
- motion state,
- mission sequencing,
- navigation decision.

## 4. Motion Control Rule

Motion logic wajib berada di:

```text
main/logic/motion_control/
```

Module ini berisi:

```text
MotionCommandTask
MotionLoopTask
MotionControlContext
```

Tidak boleh membuat `MotionService` aktif yang mengambil alih state machine motion.

## 5. MotionCommandTask Rule

`MotionCommandTask`:

- consume `MotionCommandQueue`,
- submit command ke `MotionControlContext`,
- boleh blocking menunggu queue,
- tidak menghitung PID,
- tidak update sensor,
- tidak akses service selain context logic.

## 6. MotionLoopTask Rule

`MotionLoopTask`:

- periodic 100 Hz awal,
- menggunakan `vTaskDelayUntil`,
- update service dalam urutan jelas,
- membaca state dari `MotionControlContext`,
- menjalankan move/turn handler,
- memanggil motor apply control,
- melakukan completion detection,
- melakukan fail-safe stop.

Body loop harus tetap kecil. Pecah helper seperti:

```text
updateServices()
startPendingCommand()
handleMove()
handleTurn()
failSafeStop()
```

## 7. MotionControlContext Rule

Context adalah shared state antara command task dan loop task.

Wajib:

- critical section pendek,
- tidak ada driver I/O saat lock,
- tidak ada log blocking saat lock,
- tidak ada dynamic allocation saat fast path.

## 8. No Sandwich Code

Dilarang membuat function yang mencampur:

```text
queue receive
serial parse
encoder read
IMU read
PID
odometry
state machine
PWM write
logging
```

Pisahkan sesuai layer.

## 9. Driver Ownership

Satu driver hanya boleh diakses oleh satu service:

```text
MotorEncoderDriver -> MotorControlService
IMUDriver          -> HeadingService
SerialDriver       -> CommunicationService
ConsoleDriver      -> TelemetryService
```

Task tidak boleh include header driver kecuali di `app_main.cpp` untuk wiring dependency.

## 10. PID Rule

PID berada di Service layer sebagai helper pasif.

`PIDController` harus generic:

```cpp
float update(float setpoint, float measurement, float dt_s);
void reset();
```

Wajib:

- output clamp,
- integral clamp,
- reset saat command baru/state berubah tajam.

## 11. Unit Rule

Gunakan suffix unit:

```text
distance_m
speed_mps
angle_deg
heading_deg
yaw_rate_dps
dt_s
```

Jangan campur degree/radian tanpa suffix.

## 12. FreeRTOS API Rule

FreeRTOS API utama hanya di Logic/infrastructure:

- `xTask*`,
- `vTask*`,
- `xQueue*`,
- critical section.

Service core motion tidak boleh bergantung pada scheduler.

## 13. ESP-IDF Hardware API Rule

API berikut sebisa mungkin hanya muncul di Driver:

```text
gpio_*
ledc_*
pcnt_*
uart_*
i2c_*
spi_*
```

## 14. Non-Blocking Control Rule

Control loop tidak boleh menunggu:

- telemetry queue,
- serial input,
- filesystem,
- network,
- long mutex.

Telemetry producer high-priority harus memakai timeout 0.

## 15. Motion API Rule

Movement asynchronous.

Dilarang:

```cpp
while (distance < target) {}
vTaskDelay(move_time);
```

Yang benar:

```text
command queued
state set
periodic loop progresses motion
completion detected later
```

## 16. Config Rule

Konstanta dipisah:

```text
RobotConfig.hpp
PIDConfig.hpp
FreeRTOSConfig.hpp
```

Tidak boleh magic number penting tersebar di code.

## 17. Naming Rule

- Class: `PascalCase`
- Method/function: `camelCase`
- Member: trailing underscore
- Physical value: suffix unit
- Constants: `UPPER_SNAKE_CASE` atau namespace constexpr konsisten

## 18. Header Rule

Header berisi interface dan dependency minimal.

Gunakan forward declaration jika memungkinkan.

## 19. Error Handling Rule

Init/hardware lifecycle menggunakan `esp_err_t`.

Runtime control error harus fail-safe stop.

Jangan silently ignore error hardware penting.

## 20. Telemetry Rule

Telemetry asynchronous:

```text
Producer -> TelemetryQueue -> TelemetryTask -> TelemetryService -> ConsoleDriver
```

Telemetry boleh drop. Control loop tidak boleh terlambat karena telemetry.

## 21. CMake Rule

Setiap `.cpp` yang aktif harus tercantum eksplisit di `main/CMakeLists.txt`.

Jangan mengandalkan accidental include path.

## 22. Definition of Done

### Driver

Selesai jika:

- init hardware berhasil,
- API dasar bekerja,
- error ditangani,
- tidak ada control logic,
- hanya owning service yang akses.

### Service

Selesai jika:

- responsibility jelas,
- API kecil,
- tidak membuat task,
- tidak akses driver lain,
- bisa dipanggil dari logic.

### Task / Logic

Selesai jika:

- ownership behavior jelas,
- blocking behavior jelas,
- tidak akses hardware langsung,
- timing control stabil,
- state machine tidak bercampur dengan driver detail.

## 23. Implementation Order

Urutan kerja:

1. Models + config
2. Driver skeleton API
3. Passive services
4. MotionControlContext
5. MotionCommandTask
6. MotionLoopTask
7. SerialTask + CommunicationService
8. TelemetryTask + TelemetryService
9. Application mission
10. Driver hardware implementation
11. Hardware validation/tuning

## 24. Forbidden Patterns

Dilarang:

```text
MotionService as active motion brain
God task
Global mutable driver for many users
Task directly accesses driver
Service owns command queue
PID inside driver
GPIO inside service
Blocking move function
Busy wait
Delay-based movement
Serial directly to motor
Telemetry directly from driver
```

## 25. Final Rule

Pegang kalimat ini:

```text
Application defines intent.
Logic/Task owns runtime behavior.
Service provides passive capability.
Driver touches hardware.
```
