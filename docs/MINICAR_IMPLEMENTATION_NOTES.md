# Mini Car Implementation Notes

Catatan ini merangkum cara implementasi repo `minicar-pid-control` berdasarkan:

- `docs/MINICAR_ARCHITECTURE.md`
- `docs/MINICAR_CODING_RULES.md`
- target framework: ESP-IDF 5.4.4

Tujuan catatan ini adalah menjadi pegangan sebelum eksekusi coding. Fokus pekerjaan Codex adalah `application`, `logic/task`, dan `service` layer. Driver dibuat sebagai file/interface/skeleton saja agar implementasi hardware detail dapat dikerjakan oleh Aranda.

## Boundary Pekerjaan

Codex boleh mengerjakan:

- `main/application`
- `main/logic`
- `main/service`
- `main/model`
- `main/config`
- wiring awal di `app_main.cpp`
- update `main/CMakeLists.txt`
- dokumentasi handoff untuk AI/developer lain

Codex tidak mengisi logic hardware driver:

- tidak menulis GPIO/LEDC/PCNT/UART/I2C/SPI detail
- tidak menentukan pin final
- tidak mengakses ESP-IDF hardware API di service/task
- driver cukup berupa class/interface/skeleton dengan method yang dibutuhkan service

Aranda mengerjakan:

- `MotorEncoderDriver`
- `IMUDriver`
- `SerialDriver`
- `ConsoleDriver`
- konfigurasi hardware pin/peripheral final
- validasi hardware real

## Layer Contract

Dependency harus tetap turun:

```text
Application
    -> Logic / FreeRTOS Task
    -> Service
    -> Driver
    -> Hardware
```

Shortcut yang tidak boleh dibuat:

- Application langsung ke Service low-level atau Driver
- Task langsung ke Driver
- Service membaca Driver milik service lain
- Driver memanggil Service/Task
- PID ditempatkan di Task atau Driver

## Implementation Shape

Struktur awal yang direkomendasikan:

```text
main/
|-- app_main.cpp
|-- application/
|-- logic/
|-- service/
|-- driver/
|-- model/
`-- config/
```

Tahap awal tetap satu ESP-IDF component di folder `main`. Belum perlu memecah ke `components/` sampai arsitektur stabil.

## Model First

Buat shared DTO di `main/model`:

- `MotionCommand.hpp`
- `MotionState.hpp`
- `TelemetryMessage.hpp`
- `Pose2D.hpp`
- `RobotState.hpp`

Gunakan suffix unit untuk value fisik:

- `distance_m`
- `speed_mps`
- `angle_deg`
- `heading_deg`
- `yaw_rate_dps`
- `dt_s`

Convention awal yang perlu dijaga konsisten:

- `MOVE +distance_m` = maju
- `MOVE -distance_m` = mundur
- `TURN +angle_deg` = kanan
- `TURN -angle_deg` = kiri
- internal heading menggunakan degree sampai ada keputusan lain

## Config

Pisahkan konstanta di `main/config`:

- `RobotConfig.hpp`: wheel diameter, wheel base, ticks per rev, speed limit, tolerance
- `PIDConfig.hpp`: gain PID speed kiri/kanan dan heading
- `FreeRTOSConfig.hpp`: priority, stack, queue size, control period

Jangan menyebar magic number di service/task.

## Service Layer

Service harus dapat diuji tanpa scheduler sebanyak mungkin. FreeRTOS API jangan masuk core motion calculation.

Service utama:

- `MotorControlService`
- `HeadingService`
- `OdometryService`
- `MotionService`
- `TelemetryService`
- `CommunicationService`

Helper:

- `PIDController`

### PIDController

Generic, tidak tahu motor/heading/navigation.

API minimal:

```cpp
float update(float setpoint, float measurement, float dt_s);
void reset();
void setOutputLimit(float min_output, float max_output);
void setIntegralLimit(float min_integral, float max_integral);
```

Wajib ada:

- output clamp
- integral clamp atau anti-windup sederhana
- `reset()` saat command baru atau state berubah tajam

### MotorControlService

Satu-satunya service yang punya akses ke `MotorEncoderDriver`.

Tanggung jawab:

- baca ticks/RPM dari driver
- ubah feedback ke velocity engineering unit
- PID speed kiri dan kanan
- clamp output motor
- dead-zone compensation jika dibutuhkan
- expose encoder/velocity state untuk `MotionService`

Tidak boleh:

- tahu `MotionCommand`
- menjalankan state machine move/turn
- membaca IMU
- mengirim telemetry blocking

### HeadingService

Satu-satunya service yang punya akses ke `IMUDriver`.

Tanggung jawab:

- update heading relatif dari gyro/IMU
- normalize angle error ke shortest path
- PID heading correction
- expose heading dan yaw rate
- reset reference saat command baru

Tidak boleh:

- akses motor driver
- menentukan command mission
- melakukan turn blocking loop

### OdometryService

Pure computation service. Tidak punya driver.

Input berasal dari `MotionService`:

```cpp
updateOdometry(left_ticks, right_ticks, heading_deg);
```

Tanggung jawab:

- hitung travelled distance
- hitung pose `Pose2D`
- reset distance/pose reference

### MotionService

Facade utama motion subsystem.

Public API harus kecil:

```cpp
esp_err_t init();
void update(float dt_s);
bool startMove(float distance_m, float speed_mps);
bool startTurn(float angle_deg);
void stop();
bool isBusy() const;
bool isCompleted() const;
MotionState getState() const;
```

`MotionService` boleh mengorkestrasi:

- `MotorControlService`
- `HeadingService`
- `OdometryService`

Tapi tidak boleh menjadi god class. `update()` idealnya hanya memanggil langkah kecil:

```cpp
updateHeading(dt_s);
updateOdometryState();
updateMotionState(dt_s);
updateMotorControl(dt_s);
```

State handler dipisah:

- `handleIdle()`
- `handleMove()`
- `handleTurn()`
- `handleStopping()`
- `handleError()`

## Logic / Task Layer

Task hanya scheduling/orchestration. Tidak ada PID, GPIO, encoder calculation, IMU read langsung, atau motion math detail.

### ControlTask

Periodik 100 Hz awal.

Body ideal:

```cpp
motion_service.update(0.01f);
vTaskDelayUntil(...);
```

Tidak boleh blocking telemetry/serial/queue dengan `portMAX_DELAY`.

### MotionTask

Satu-satunya consumer `MotionCommandQueue`.

Tanggung jawab:

- tunggu command
- cek `motion_service.isBusy()`
- panggil `startMove`, `startTurn`, atau `stop`
- command berikutnya jalan setelah motion sebelumnya completed/idle

Tidak boleh:

- hitung PID
- baca encoder/IMU
- akses driver

### SerialTask

External command masuk lewat:

```text
SerialDriver -> CommunicationService -> MotionCommandQueue
```

Tidak boleh langsung ke motor atau langsung bypass queue.

### TelemetryTask

Semua telemetry/log non-critical lewat `TelemetryQueue`.

Producer harus non-blocking:

```cpp
xQueueSend(queue, &message, 0);
```

Jika queue penuh, drop telemetry non-critical.

## Driver Skeleton Boundary

Driver file boleh dibuat agar service bisa compile dan Aranda punya kontrak API.

Namun isi method hardware sebaiknya:

- minimal stub
- return `ESP_ERR_NOT_SUPPORTED` atau value netral bila belum diisi
- diberi TODO spesifik untuk Aranda

Driver skeleton tidak boleh mengandung fake control behavior yang menutupi pekerjaan hardware real.

## Startup Flow

`app_main()` nanti idealnya:

1. create driver object
2. create service object dengan dependency injection
3. call `init()`
4. create queue
5. create task
6. push initial mission dari application layer

Initial mission demo:

```text
MOVE 1.0 m @ 1.0 m/s
TURN +90 deg
MOVE 2.0 m @ 0.5 m/s
STOP
```

## ESP-IDF 5.4.4 Notes

Project harus tetap ESP-IDF native C++.

Konsekuensi implementasi:

- ubah entry dari `main.c` ke `app_main.cpp`
- pastikan `extern "C" void app_main(void)` dipakai
- `main/CMakeLists.txt` mencantumkan semua `.cpp`
- dependency ESP-IDF dicantumkan eksplisit jika diperlukan
- tidak pindah ke Arduino

## Validation Boundary

Saat Codex mengerjakan layer non-driver:

- boleh lakukan static/source check
- boleh lakukan build hanya jika user mengizinkan
- tidak mengklaim hardware valid sebelum diuji di device
- driver behavior dianggap pending sampai Aranda implement dan hardware test

## Implementation Order

Urutan eksekusi yang disarankan:

1. Models + config
2. Driver skeleton API
3. PIDController
4. MotorControlService, HeadingService, OdometryService
5. MotionService + state machine
6. MotionCommandQueue + MotionTask
7. ControlTask
8. TelemetryQueue + TelemetryTask + TelemetryService
9. SerialTask + CommunicationService
10. Application Mission example
11. Handoff `.MD` detail untuk Aranda/AI lain

## Handoff Rule untuk Aranda

Dokumen handoff berikutnya harus menjelaskan:

- file driver mana yang harus diisi
- method mana yang menjadi kontrak service
- apa yang tidak boleh diubah agar layer atas tidak rusak
- convention unit dan sign
- expected return/error behavior
- test hardware minimal untuk setiap driver
- log/telemetry yang boleh dipakai saat bring-up

