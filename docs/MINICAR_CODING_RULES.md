# Mini Car Coding Rules
## ESP32 DevKit V1 — ESP-IDF + C++

Dokumen ini berisi aturan implementasi yang wajib diikuti saat mengembangkan source code mini car.

Tujuan utama:

- modular,
- tidak sandwich,
- mudah dibaca,
- mudah diuji,
- dependency jelas,
- tidak ada akses hardware liar,
- mudah dikembangkan oleh beberapa developer,
- cocok digunakan sebagai specification untuk Codex atau AI coding agent.

---

# 1. Mandatory Technology

Project wajib menggunakan:

```text
ESP-IDF
C++
FreeRTOS
```

Jangan mengubah project menjadi Arduino Framework kecuali ada keputusan eksplisit.

Gunakan API ESP-IDF secara native bila tersedia.

---

# 2. Layer Rule

Layer resmi:

```text
Application
Logic
Service
Driver
Hardware
```

Dependency:

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

Tidak boleh membuat shortcut antar layer.

---

# 3. No Sandwich Code

"Sandwich code" adalah code yang mencampur terlalu banyak responsibility dalam satu class/function/file.

Contoh dilarang:

```cpp
void controlLoop()
{
    readEncoder();

    calculateRPM();

    readIMU();

    calculateHeading();

    calculateOdometry();

    calculateSpeedPID();

    calculateHeadingPID();

    setPWM();

    printLog();

    parseSerial();

    checkMotionCommand();
}
```

Walaupun function tersebut bekerja, architecture ini tidak diterima.

Harus dipisah berdasarkan responsibility.

---

# 4. Single Responsibility Rule

Setiap class harus mempunyai responsibility utama yang jelas.

Contoh:

```text
MotorEncoderDriver
    hardware motor + encoder

MotorControlService
    wheel velocity control

HeadingService
    heading estimation/control

OdometryService
    pose/distance calculation

MotionService
    motion orchestration

TelemetryService
    telemetry output

CommunicationService
    communication parsing/format
```

Jika sebuah class mulai memiliki banyak responsibility yang tidak berkaitan, pecah class tersebut.

---

# 5. Driver Rules

Driver hanya menangani hardware.

Driver boleh:

- configure GPIO,
- configure PWM,
- configure PCNT,
- configure I2C,
- configure SPI,
- configure UART,
- read hardware,
- write hardware.

Driver tidak boleh:

- menjalankan motion state machine,
- menentukan target distance,
- menentukan target heading,
- melakukan navigation decision,
- menyimpan mission,
- membaca MotionCommand,
- mengakses FreeRTOS Motion Queue,
- menjalankan PID.

---

# 6. One Driver — One Service

Setiap driver hanya boleh diakses oleh satu service.

Contoh:

```text
MotorEncoderDriver
       ↓
MotorControlService
```

dan:

```text
IMUDriver
   ↓
HeadingService
```

Dilarang:

```text
MotorEncoderDriver
├── MotorControlService
├── OdometryService
└── MotionService
```

Dilarang:

```text
IMUDriver
├── HeadingService
└── MotionService
```

---

# 7. Driver Ownership

Owning service menyimpan driver sebagai reference atau pointer yang jelas.

Prefer:

```cpp
MotorControlService(
    MotorEncoderDriver& driver
);
```

daripada global variable.

Hindari global mutable hardware object jika dependency injection memungkinkan.

---

# 8. Service Rules

Service menangani business/control logic.

Service boleh:

- menggunakan driver yang dimilikinya,
- menggunakan helper class,
- melakukan control calculation,
- menyimpan state internal,
- menerima input dari MotionService,
- memberikan output yang dibutuhkan MotionService.

Service tidak boleh:

- membuat FreeRTOS task sendiri tanpa kebutuhan kuat,
- mengakses Queue milik Logic secara sembarangan,
- melakukan mission sequencing,
- membaca hardware driver yang bukan miliknya.

---

# 9. MotionService Rule

`MotionService` adalah satu-satunya public facade untuk motion subsystem.

Public API contoh:

```cpp
bool startMove(
    float distanceMeter,
    float speedMps
);

bool startTurn(
    float angleDegree
);

void stop();

void update(float dt);

bool isBusy() const;

MotionState getState() const;
```

Jangan mengekspos low-level methods seperti:

```cpp
setLeftPWM()
setRightPWM()
readEncoder()
calculateHeadingPID()
```

---

# 10. Protected Inheritance Rule

Jika multiple inheritance digunakan:

```cpp
class MotionService :
    protected MotorControlService,
    protected HeadingService,
    protected OdometryService
```

gunakan `protected` atau `private` inheritance sesuai kebutuhan.

Tujuannya:

- MotionService dapat menggunakan kemampuan parent,
- API parent tidak otomatis menjadi public API MotionService.

---

# 11. Inheritance Must Represent Architecture

Jangan membuat inheritance chain hanya agar code dapat diakses.

Dilarang:

```text
PIDController
    ↓
MotorControlService
    ↓
HeadingService
    ↓
OdometryService
    ↓
MotionService
```

Ini menciptakan inheritance sandwich.

Yang diinginkan:

```text
             Service
          /     |      \
         /      |       \
      Motor   Heading  Odometry
         \      |       /
          \     |      /
           MotionService
```

PIDController digunakan sebagai helper/composed member.

---

# 12. Prefer Composition for Helpers

Contoh benar:

```cpp
class MotorControlService {
private:
    PIDController leftPid_;
    PIDController rightPid_;
};
```

Bukan:

```cpp
class MotorControlService : public PIDController
```

MotorControlService bukan PIDController.

MotorControlService **memiliki/menggunakan** PIDController.

---

# 13. PID Location Rule

PID berada di Service Layer.

Tidak boleh:

```text
ControlTask
    └── PID
```

Tidak boleh:

```text
MotorEncoderDriver
    └── PID
```

Yang benar:

```text
MotorControlService
    ├── LeftSpeedPID
    └── RightSpeedPID

HeadingService
    └── HeadingPID
```

---

# 14. PID Generic Rule

PIDController harus generic.

PIDController tidak boleh mengetahui:

- motor,
- encoder,
- IMU,
- degree,
- m/s,
- navigation.

Ideal interface:

```cpp
float update(
    float setpoint,
    float measurement,
    float dt
);
```

---

# 15. PID Reset Rule

Reset PID ketika:

- mulai command baru,
- berpindah dari MOVE ke TURN,
- emergency stop,
- state error,
- target berubah secara discontinuous jika diperlukan.

Tujuannya mencegah integral term lama memengaruhi command baru.

---

# 16. Anti-Windup

PID implementation harus memiliki mekanisme untuk mencegah integral windup.

Minimal:

```text
integral clamp
```

atau conditional integration.

Output juga harus memiliki limit.

---

# 17. Derivative Rule

Derivative harus mempertimbangkan noise sensor.

Jika menggunakan D term:

- jangan asal memperbesar KD,
- pertimbangkan derivative filtering,
- gunakan `dt` yang konsisten,
- hindari derivative spike ketika setpoint berubah.

---

# 18. ControlTask Rule

ControlTask hanya boleh melakukan orchestration waktu.

Ideal body:

```cpp
while (true)
{
    motionService.update(dt);

    vTaskDelayUntil(...);
}
```

Jangan tambahkan:

```cpp
readEncoder();
readIMU();
calculatePID();
setMotorPWM();
```

ke ControlTask.

---

# 19. ControlTask Timing

Gunakan:

```cpp
vTaskDelayUntil()
```

bukan sekadar:

```cpp
vTaskDelay()
```

untuk periodic control loop.

Tujuannya menjaga periode lebih stabil.

---

# 20. dt Rule

Jangan hardcode `dt` di berbagai service.

Ada dua pendekatan:

```text
A. ControlTask menghasilkan dt yang konsisten
B. MotionService menghitung actual elapsed time
```

Pilih satu dan gunakan konsisten.

Untuk awal, periodic fixed loop 100 Hz dapat menggunakan:

```text
dt = 0.01 s
```

tetapi actual timing tetap perlu dimonitor saat tuning.

---

# 21. MotionTask Rule

MotionTask hanya menangani:

- MotionCommand Queue,
- sequencing command,
- status busy/completed,
- pemanggilan public MotionService API.

MotionTask tidak boleh menjalankan control calculation.

---

# 22. Queue-Based Command Rule

Semua sumber motion command harus menuju Queue.

Contoh:

```text
Mission
Serial
Future Navigation
Remote Command
```

semuanya:

```text
→ MotionCommandQueue
```

Tidak boleh satu source bypass langsung menuju motor subsystem.

---

# 23. SerialTask Rule

SerialTask tidak boleh memanggil motor secara langsung.

Dilarang:

```cpp
if (serialCommand == "MOVE")
{
    motorDriver.setPWM(...);
}
```

Yang benar:

```text
Serial
  ↓
Parse
  ↓
MotionCommand
  ↓
MotionCommandQueue
```

---

# 24. Communication Service Rule

CommunicationService bertanggung jawab terhadap:

- frame parsing,
- protocol validation,
- command decoding,
- checksum bila digunakan,
- serialization response.

SerialDriver hanya melakukan UART hardware I/O.

---

# 25. Telemetry Rule

Jangan menyebarkan serial log low-level di seluruh project.

Prefer:

```text
Telemetry producer
        ↓
TelemetryQueue
        ↓
TelemetryTask
        ↓
TelemetryService
        ↓
ConsoleDriver
```

---

# 26. Logging in Control Loop

Hindari direct logging intensif di `ControlTask`.

Dilarang melakukan log 100 Hz berupa string panjang menggunakan output blocking.

Jika membutuhkan debug high-rate:

- sample data,
- throttle log,
- gunakan binary telemetry jika diperlukan,
- atau log setiap N cycle.

---

# 27. Telemetry Queue Must Be Non-Blocking

Dari high-priority task:

```cpp
xQueueSend(
    telemetryQueue,
    &message,
    0
);
```

Jika gagal:

```text
drop telemetry
```

Jangan menunggu telemetry queue kosong.

---

# 28. Avoid Dynamic Allocation in Fast Path

Hindari:

```cpp
new
delete
malloc
free
std::string
```

secara berulang pada control loop.

Gunakan:

- fixed-size structs,
- stack object,
- statically allocated buffer,
- pre-created queue.

---

# 29. Telemetry Message Size

Gunakan bounded string.

Contoh:

```cpp
struct TelemetryMessage {
    uint32_t timestamp;
    LogLevel level;
    char tag[16];
    char message[96];
};
```

Jangan membuat:

```cpp
char message[512];
```

tanpa alasan.

---

# 30. Queue Size Rule

Queue size harus didasarkan pada throughput.

Initial values:

```text
MotionCommandQueue:
    sekitar 8–16 command

TelemetryQueue:
    sekitar 20–50 message
```

Angka final disesuaikan setelah runtime profiling.

---

# 31. No Infinite Blocking in Control Path

ControlTask tidak boleh menggunakan:

```cpp
portMAX_DELAY
```

untuk operasi non-critical.

ControlTask tidak boleh menunggu:

- telemetry,
- serial output,
- command parsing,
- filesystem,
- network.

---

# 32. Task Priority Rule

Priority harus mencerminkan criticality.

Recommended hierarchy:

```text
ControlTask
    highest

MotionTask
    medium-high

SerialTask
    medium

TelemetryTask
    low
```

Telemetry tidak boleh memiliki priority lebih tinggi dari motor control.

---

# 33. Task Stack Rule

Jangan memberi semua task stack besar secara default.

Mulai dari ukuran reasonable kemudian monitor:

```cpp
uxTaskGetStackHighWaterMark()
```

Gunakan hasil runtime untuk tuning stack.

---

# 34. CPU Core Pinning

Jangan melakukan core pinning sebelum ada alasan.

Jika kelak diperlukan:

- control task dapat dipin setelah profiling,
- pertimbangkan Wi-Fi/Bluetooth task ESP-IDF,
- jangan mengunci semua task ke satu core tanpa analisis.

---

# 35. Odometry Rule

OdometryService tidak boleh mengakses driver.

Input harus diberikan oleh MotionService.

Contoh:

```cpp
updateOdometry(
    leftTicks,
    rightTicks,
    heading
);
```

---

# 36. Raw vs Processed Data

Driver mengeluarkan raw/hardware-near data.

Service boleh mengubah menjadi unit engineering.

Contoh:

```text
Encoder Driver:
    ticks
    RPM

MotorControlService:
    meter per second

Heading Driver:
    gyro raw/calibrated

HeadingService:
    yaw / heading degree
```

Pastikan ownership unit jelas.

---

# 37. Unit Rule

Gunakan unit yang konsisten.

Recommended:

```text
distance          meter
linear velocity   meter/second
angle             degree atau radian — pilih satu internal convention
angular velocity  degree/second atau radian/second
time              second
```

Jika memilih radian sebagai internal unit, convert hanya di boundary.

Jangan mencampur degree dan radian tanpa suffix.

---

# 38. Variable Naming with Units

Untuk value fisik, suffix unit sangat dianjurkan:

```cpp
distance_m
speed_mps
angle_deg
yaw_rate_dps
dt_s
wheel_base_m
```

Ini mengurangi bug unit.

---

# 39. State Rule

Motion state hanya dimodifikasi melalui MotionService.

Jangan expose state variable public.

Gunakan getter:

```cpp
MotionState getState() const;
```

---

# 40. No Public Mutable Data

Hindari:

```cpp
public:
    float currentSpeed;
    float heading;
```

Gunakan:

```cpp
float getCurrentSpeed() const;
float getHeading() const;
```

Jika perubahan diperlukan, gunakan method terkontrol.

---

# 41. Header Rule

Header file harus berisi:

- class declaration,
- public interface,
- minimal dependency,
- constants/types jika memang bagian interface.

Jangan menaruh implementation besar di header kecuali template atau alasan performa yang jelas.

---

# 42. Include Rule

Gunakan forward declaration jika memungkinkan.

Contoh:

```cpp
class MotorEncoderDriver;
```

untuk mengurangi coupling/include dependency.

---

# 43. Namespace Rule

Gunakan namespace project agar simbol tidak global.

Contoh:

```cpp
namespace minicar {
namespace service {
...
}
}
```

atau struktur namespace lain yang konsisten.

---

# 44. Naming Convention

Recommended:

Classes:

```text
PascalCase
```

Examples:

```text
MotionService
HeadingService
MotorEncoderDriver
PIDController
```

Methods:

```text
camelCase
```

Examples:

```text
startMove()
getHeading()
updateMotorControl()
```

Member private/protected:

```text
trailing underscore
```

Example:

```cpp
float targetSpeed_;
PIDController headingPid_;
```

Constants:

```text
UPPER_SNAKE_CASE
```

atau `constexpr` style konsisten.

---

# 45. Boolean Naming

Gunakan prefix:

```text
is
has
can
should
```

Contoh:

```cpp
isBusy()
isCompleted()
hasError()
```

---

# 46. No Magic Numbers

Dilarang:

```cpp
if (error < 1.5f)
```

jika 1.5 adalah tuning parameter penting tanpa nama.

Gunakan:

```cpp
constexpr float TURN_TOLERANCE_DEG = 1.5f;
```

atau config.

---

# 47. Config Separation

Hardware constants:

```text
RobotConfig.hpp
```

Control constants:

```text
PIDConfig.hpp
```

RTOS constants:

```text
FreeRTOSConfig.hpp
```

Jangan menaruh semua constant dalam satu file raksasa.

---

# 48. Error Handling

Initialization function dapat menggunakan:

```cpp
esp_err_t
```

Contoh:

```cpp
esp_err_t init();
```

Runtime logic gunakan state/error model yang sesuai.

Jangan silently ignore hardware error.

---

# 49. Fail Safe Rule

Jika subsystem penting gagal:

```text
IMU failure
encoder failure
motor driver failure
control timeout
```

default output:

```text
MOTOR STOP
```

---

# 50. Validation Rule

Public MotionService API harus memvalidasi input.

Contoh:

```text
distance <= 0
invalid speed
NaN
infinite value
angle outside permitted range
```

Invalid command tidak boleh langsung dieksekusi.

---

# 51. Motor Output Clamp

Selalu clamp motor control output.

Contoh conceptual:

```cpp
output = std::clamp(
    output,
    MIN_OUTPUT,
    MAX_OUTPUT
);
```

PWM command tidak boleh melewati hardware range.

---

# 52. Speed Target Clamp

Target velocity harus dibatasi sesuai kemampuan kendaraan.

Contoh:

```text
MAX_LINEAR_SPEED_MPS
MAX_WHEEL_SPEED_MPS
```

---

# 53. Heading Error Normalization

Selalu gunakan shortest-path angle error.

Function helper contoh:

```cpp
float normalizeAngleDeg(float angle);
```

Output:

```text
[-180, 180)
```

---

# 54. Start Move Rule

`startMove()` minimal melakukan:

```text
validate command
reset motion-specific PID if required
capture start distance
capture target heading
store target speed
store target distance
change state to MOVING
```

---

# 55. Start Turn Rule

`startTurn()` minimal melakukan:

```text
validate angle
capture current heading
calculate normalized target
reset heading PID
change state to TURNING
```

---

# 56. Completion Rule for Move

Jangan hanya menggunakan:

```text
distance >= target
```

Pertimbangkan:

```text
distance tolerance
current velocity
overshoot handling
```

Contoh:

```text
abs(distance_error) < tolerance
AND
vehicle sufficiently slow
```

Implementasi final tergantung karakter mekanik.

---

# 57. Completion Rule for Turn

Recommended:

```text
abs(angle_error) < ANGLE_TOLERANCE
AND
abs(yaw_rate) < YAW_RATE_TOLERANCE
```

Ini mencegah false completion saat kendaraan melewati target dengan cepat.

---

# 58. MotionService Update Must Stay Small

Target style:

```cpp
void MotionService::update(float dt)
{
    updateHeading(dt);
    updateOdometryState();
    updateMotionState(dt);
    updateMotorControl(dt);
}
```

Jika `update()` tumbuh terlalu besar, pecah fungsi.

---

# 59. State Handler Separation

Gunakan:

```cpp
handleIdle()
handleMove()
handleTurn()
handleStopping()
handleError()
```

daripada satu switch case berisi puluhan baris setiap case.

---

# 60. Function Size Guideline

Tidak ada limit absolut, namun jika function:

- sulit dilihat dalam satu layar,
- memiliki banyak level nested if,
- melakukan beberapa responsibility,

pecah menjadi function lebih kecil.

---

# 61. Avoid Deep Nesting

Hindari:

```cpp
if (...)
{
    if (...)
    {
        if (...)
        {
            if (...)
            {
            }
        }
    }
}
```

Gunakan:

- guard clauses,
- helper function,
- state machine.

---

# 62. Guard Clause Rule

Prefer:

```cpp
if (!initialized_) {
    return;
}

if (state_ == MotionState::ERROR) {
    stopMotor();
    return;
}
```

daripada nesting panjang.

---

# 63. Hardware Abstraction

Service tidak boleh menggunakan GPIO number langsung.

Dilarang:

```cpp
gpio_set_level(GPIO_NUM_18, 1);
```

di Service.

GPIO hanya di Driver.

---

# 64. ESP-IDF API Location

Low-level API seperti:

```text
gpio_*
ledc_*
pcnt_*
uart_*
i2c_*
spi_*
```

sebisa mungkin hanya muncul dalam Driver Layer.

FreeRTOS API:

```text
xTask*
xQueue*
xSemaphore*
```

sebisa mungkin berada di Logic/infrastructure layer.

---

# 65. FreeRTOS in Service

Service sebaiknya tidak tergantung FreeRTOS kecuali infrastructure service memang membutuhkan primitive tertentu.

Core motion calculation harus sebisa mungkin dapat diuji tanpa scheduler.

---

# 66. Testability Rule

Service harus bisa diuji dengan fake/mock driver.

Karena itu gunakan constructor injection.

Contoh:

```cpp
MotorControlService(
    MotorEncoderDriver& driver
);
```

Future improvement dapat memakai interface:

```cpp
IMotorEncoderDriver
```

jika unit testing membutuhkan mocking penuh.

---

# 67. Do Not Over-Abstract Too Early

Walaupun architecture modular, jangan membuat interface untuk semua hal tanpa kebutuhan.

Hindari puluhan file abstraksi kosong.

Prioritaskan:

```text
clear ownership
clear responsibility
simple API
```

---

# 68. Model Folder Rule

Shared DTO/data structures masuk `model`.

Contoh:

```text
MotionCommand
MotionState
TelemetryMessage
Pose2D
RobotState
```

Jangan menduplikasi struct yang sama di banyak layer.

---

# 69. RobotState

Optional shared snapshot:

```cpp
struct RobotState {
    float leftSpeedMps;
    float rightSpeedMps;

    float headingDeg;

    float travelledDistanceM;

    Pose2D pose;

    MotionState motionState;
};
```

Snapshot harus read-only bagi consumer.

---

# 70. Telemetry Snapshot

Telemetry tidak perlu mengakses driver.

Jika perlu telemetry state:

```text
MotionService / service
    ↓
structured telemetry snapshot
    ↓
TelemetryQueue
```

Bukan:

```text
TelemetryTask
    ↓
read encoder driver
```

---

# 71. No Cross-Branch Driver Access

Tree harus tetap:

```text
MotorEncoderDriver
       ↓
MotorControlService
       ↓
MotionService
```

Tidak boleh:

```text
MotorEncoderDriver
       ↓
TelemetryService
```

Jika telemetry membutuhkan RPM, MotorControlService/MotionService yang memberikan datanya.

---

# 72. Serial Driver Ownership

Jika UART untuk external ESP:

```text
SerialDriver
     ↓
CommunicationService
     ↓
SerialTask
```

Jika UART untuk telemetry:

```text
ConsoleDriver
     ↓
TelemetryService
     ↓
TelemetryTask
```

Jangan satu driver dimiliki dua service.

---

# 73. Separate Communication and Telemetry UART Conceptually

Walaupun saat development keduanya mungkin keluar melalui console yang sama, architecture tetap memisahkan responsibility:

```text
Communication
≠
Telemetry
```

Communication membawa protocol/device data.

Telemetry membawa debug/status/log.

---

# 74. Memory Rule

Untuk ESP32 DevKit V1:

- hindari queue dengan payload besar,
- hindari excessive task count,
- hindari heap allocation pada high-rate path,
- monitor free heap,
- monitor task high-water mark.

Telemetry Queue ukuran beberapa KB adalah normal.

---

# 75. Runtime Monitoring

Saat development, expose diagnostics seperti:

```text
free heap
minimum free heap
task stack high-water mark
control loop execution time
queue high-water mark
dropped telemetry count
```

Namun diagnostics tidak boleh mengganggu real-time control.

---

# 76. Control Loop Execution Time

Measure waktu eksekusi:

```text
MotionService::update()
```

Pastikan worst-case execution time berada jauh di bawah period.

Untuk 10 ms loop:

```text
execution time << 10 ms
```

Idealnya terdapat margin besar.

---

# 77. No Delay Inside Service Update

Dilarang:

```cpp
vTaskDelay(...)
```

di dalam:

```cpp
MotionService::update()
MotorControlService::updateMotorControl()
HeadingService::updateHeading()
```

Control loop tidak boleh berhenti karena delay internal.

---

# 78. No Busy Wait

Dilarang:

```cpp
while (getHeading() != target) {
}
```

Dilarang:

```cpp
while (distance < target) {
}
```

Movement harus asynchronous/state-based.

---

# 79. Non-Blocking Motion API

`startMove()` dan `startTurn()` harus return cepat.

Dilarang:

```cpp
void moveOneMeter()
{
    while (...) {
        ...
    }
}
```

Yang benar:

```text
startMove()
    ↓
state = MOVING
    ↓
return
```

ControlTask menyelesaikan motion secara bertahap.

---

# 80. No Sleep-to-Move

Jangan mengukur jarak dengan:

```text
motor on
delay 1 second
motor off
```

Gunakan feedback encoder/odometry.

---

# 81. Encoder as Feedback

Encoder adalah feedback utama untuk:

```text
wheel speed
travel distance
```

Jangan menghitung velocity hanya dari PWM command.

---

# 82. IMU as Heading Feedback

IMU digunakan untuk heading/turn feedback.

Magnetometer tidak wajib digunakan sebagai primary heading indoor.

Gunakan gyro-based relative heading sebagai baseline.

---

# 83. Calibration Layer

Calibration data dapat ditempatkan di config/service sesuai kebutuhan.

Contoh:

```text
wheel diameter correction
left/right encoder scale
gyro bias
motor dead-zone
```

Jangan hardcode calibration tersebar di banyak file.

---

# 84. Left/Right Motor Difference

Jangan mengasumsikan motor kiri dan kanan identik.

Arsitektur harus mendukung:

```text
left PID tuning
right PID tuning
left scale correction
right scale correction
```

---

# 85. Motor Dead Zone

Motor DC mungkin memiliki PWM minimum untuk mulai bergerak.

Bila diperlukan, dead-zone compensation masuk:

```text
MotorControlService
```

bukan Driver.

Driver tetap hanya menerapkan requested hardware output.

---

# 86. Direction Convention

Tetapkan convention satu kali.

Contoh:

```text
positive velocity = forward
negative velocity = reverse

positive angle = right
```

atau convention lain.

Tuliskan di documentation dan gunakan konsisten.

---

# 87. Motion Command Sign Convention

Contoh:

```text
MOVE +distance = forward
MOVE -distance = reverse

TURN +angle = right
TURN -angle = left
```

Jika memilih convention ini, seluruh project harus mengikuti convention yang sama.

---

# 88. Include Documentation

Setiap public class wajib memiliki comment singkat mengenai responsibility.

Tidak perlu comment untuk baris yang sudah jelas.

Comment harus menjelaskan:

```text
why
constraint
assumption
```

bukan mengulang code.

---

# 89. TODO Rule

TODO harus spesifik.

Buruk:

```cpp
// TODO fix
```

Baik:

```cpp
// TODO: replace fixed gyro bias with startup calibration.
```

---

# 90. No Dead Code

Jangan meninggalkan:

- commented old implementation,
- unused class,
- unused experimental PID,
- duplicate driver.

Gunakan Git sebagai history.

---

# 91. Build Warning Rule

Target:

```text
zero new compiler warnings
```

Jangan mengabaikan warning tanpa alasan.

---

# 92. CMake Rule

Setiap module/component harus mencantumkan dependency secara eksplisit pada `CMakeLists.txt`.

Jangan mengandalkan accidental include path.

---

# 93. ESP-IDF Component Evolution

Tahap awal:

```text
main/
    application/
    logic/
    service/
    driver/
```

Setelah stabil, subsystem dapat dipindahkan ke:

```text
components/
```

tanpa mengubah public architecture.

---

# 94. Definition of Done for a Driver

Driver dianggap selesai jika:

1. hardware init berhasil,
2. API dasar bekerja,
3. error ditangani,
4. tidak mengandung control logic,
5. hanya owning service yang mengakses,
6. interface terdokumentasi.

---

# 95. Definition of Done for a Service

Service dianggap selesai jika:

1. responsibility jelas,
2. tidak mengakses driver selain driver miliknya,
3. tidak berisi FreeRTOS orchestration yang tidak perlu,
4. API kecil dan jelas,
5. state internal private/protected,
6. dapat digunakan melalui MotionService jika bagian motion subsystem.

---

# 96. Definition of Done for a Task

Task dianggap selesai jika:

1. responsibility hanya scheduling/orchestration,
2. tidak mengakses hardware langsung,
3. blocking behavior jelas,
4. priority dan stack terdokumentasi,
5. tidak memiliki business/control logic yang seharusnya berada di Service.

---

# 97. Codex Implementation Order

Recommended development sequence:

```text
Phase 1
Models + Config

Phase 2
MotorEncoderDriver interface/skeleton
IMUDriver interface/skeleton

Phase 3
PIDController

Phase 4
MotorControlService
HeadingService
OdometryService

Phase 5
MotionService + state machine

Phase 6
MotionQueue + MotionTask

Phase 7
ControlTask

Phase 8
TelemetryQueue + TelemetryTask + TelemetryService

Phase 9
SerialTask + CommunicationService

Phase 10
Application Mission example

Phase 11
Runtime diagnostics and tuning
```

---

# 98. First Demo Target

Initial demo command:

```text
MOVE 1.0 m @ 1.0 m/s
TURN +90°
MOVE 2.0 m @ 0.5 m/s
STOP
```

Target behavior:

1. mobil bergerak lurus,
2. kecepatan dikontrol encoder PID,
3. heading dikoreksi selama bergerak,
4. jarak dihitung odometry,
5. turn menggunakan heading feedback,
6. command berikutnya baru berjalan setelah command sebelumnya selesai.

---

# 99. Final Forbidden Patterns

Jangan membuat:

```text
God class
God task
global mutable driver
driver accessed by many services
PID inside task
GPIO inside service
blocking move function
busy wait
delay-based distance control
serial directly to motor
telemetry directly from driver
large dynamic log allocation
nested state logic hundreds of lines
```

---

# 100. Final Architecture Rules

Pegang aturan berikut selama development:

> **One motion facade:** `MotionService`.

> **One driver owner:** satu driver hanya diakses satu service.

> **Tasks schedule; services control; drivers touch hardware.**

> **No task directly controls hardware.**

> **No service bypasses ownership untuk membaca driver lain.**

> **No blocking movement functions.**

> **Motion command always enters through the Motion Queue.**

> **Telemetry is asynchronous and lower priority than control.**

> **PID belongs to Service Layer.**

> **MotionService orchestrates; it does not absorb every algorithm.**

> **Code should remain readable from the dependency tree.**
