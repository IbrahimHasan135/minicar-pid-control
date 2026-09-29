# Mini Car Driver System Notes

Dokumen ini adalah catatan khusus untuk developer Driver Layer.

Target pembaca utama: Aranda atau AI/developer lain yang akan mengisi driver hardware.

## 1. Prinsip Utama

Driver adalah layer paling bawah yang bicara langsung dengan hardware.

Rule singkat:

```text
Application defines intent.
Logic/Task owns runtime behavior.
Service provides passive capability.
Driver touches hardware.
```

Driver tidak boleh tahu mission, queue, PID, move/turn state machine, atau navigation.

## 2. Ownership

Setiap driver hanya boleh diakses oleh owning service.

```text
Left MotorEncoderDriver  -> MotorControlService
Right MotorEncoderDriver -> MotorControlService
IMUDriver                -> HeadingService
SerialDriver             -> CommunicationService
ConsoleDriver            -> TelemetryService
```

Task tidak boleh akses driver langsung, kecuali `app_main.cpp` hanya untuk membuat object dan dependency injection.

## 3. Motor Driver Contract

`MotorEncoderDriver` mewakili satu motor dan satu encoder.

Artinya:

```text
1 MotorEncoderDriver = 1 motor + 1 encoder
```

Untuk mobil differential drive, `app_main.cpp` membuat dua driver:

```cpp
static driver::MotorEncoderDriver left_motor_driver;
static driver::MotorEncoderDriver right_motor_driver;

static service::MotorControlService motor_service(
    left_motor_driver,
    right_motor_driver
);
```

`MotorEncoderDriver` tidak boleh tahu apakah dirinya motor kiri atau kanan. Mapping kiri/kanan dilakukan di `app_main.cpp` dan `MotorControlService`.

## 4. MotorEncoderDriver API

Driver API saat ini:

```cpp
esp_err_t init();

void setOutput(float output);
int32_t getTicks() const;
float getRPM() const;

void stop();
```

Makna API:

```text
init()
    Configure hardware untuk satu motor dan satu encoder.

setOutput(output)
    Apply motor command untuk satu motor.
    Range output mengikuti service config:
        -1.0 = reverse/full negative command
         0.0 = stop/no command
        +1.0 = forward/full positive command

getTicks()
    Return encoder ticks untuk motor ini.

getRPM()
    Return RPM untuk motor ini.
    Sign harus konsisten dengan arah motor.

stop()
    Paksa motor ini masuk safe stop state.
```

## 5. MotorControlService Contract

`MotorControlService` menerima dua driver:

```cpp
MotorControlService(
    MotorEncoderDriver& left_driver,
    MotorEncoderDriver& right_driver
);
```

Service ini yang mengelola:

- left/right feedback,
- left/right velocity target,
- left/right speed PID,
- output ke driver kiri dan kanan.

Driver tidak perlu membuat PID dan tidak perlu tahu differential drive.

## 6. Turn Behavior

Untuk pivot turn:

```text
TURN right:
    left target  = positive
    right target = negative

TURN left:
    left target  = negative
    right target = positive
```

Saat ini logic mengirim target wheel speed ke service:

```cpp
motor_service.setVelocityTargets(left_mps, right_mps);
```

Lalu `MotorControlService` menghitung output PID dan memanggil:

```cpp
left_driver.setOutput(left_output);
right_driver.setOutput(right_output);
```

## 7. IMU Driver Contract

`IMUDriver` dimiliki oleh `HeadingService`.

API saat ini:

```cpp
esp_err_t init();

float getYawRateDps() const;
bool isHealthy() const;
```

Makna API:

```text
init()
    Configure I2C/SPI dan initialize IMU.

getYawRateDps()
    Return yaw rate dalam degree/second.

isHealthy()
    Return true jika IMU siap dan pembacaan valid.
```

`IMUDriver` tidak menghitung turn state machine. Heading integration dilakukan oleh `HeadingService`.

## 8. Serial Driver Contract

`SerialDriver` dimiliki oleh `CommunicationService`.

API saat ini:

```cpp
esp_err_t init();
bool readLine(char* buffer, size_t buffer_size, uint32_t timeout_ms);
```

`CommunicationService` akan parse command seperti:

```text
STOP
MOVE <distance_m> <speed_mps>
TURN <angle_deg>
```

Driver hanya membaca frame/line dari UART. Driver tidak boleh langsung mengontrol motor.

## 9. Console Driver Contract

`ConsoleDriver` dimiliki oleh `TelemetryService`.

API saat ini:

```cpp
esp_err_t init();
esp_err_t writeLine(const char* line, size_t length);
```

Telemetry boleh asynchronous dan boleh drop. Driver console tidak boleh mengganggu control loop.

## 10. Acceptance Output

Driver dianggap siap jika service di atasnya bisa menghasilkan output yang benar.

### Motor/Encoder

Minimal dapat dibuktikan:

```text
motor init OK
setOutput(+x) membuat motor bergerak arah positif
setOutput(-x) membuat motor bergerak arah negatif
stop() mematikan motor
getTicks() berubah saat roda bergerak
getRPM() masuk akal dan sign konsisten
left/right mapping benar saat dipakai oleh MotorControlService
```

### IMU

Minimal dapat dibuktikan:

```text
imu init OK
isHealthy() true setelah init berhasil
yaw_rate_dps sekitar 0 saat diam
yaw_rate_dps berubah saat board diputar
heading dari HeadingService berubah konsisten
```

### Serial

Minimal dapat dibuktikan:

```text
raw serial line terbaca
CommunicationService parse STOP
CommunicationService parse MOVE value/speed
CommunicationService parse TURN angle
```

## 11. What Not To Change

Jangan ubah layer atas kecuali benar-benar perlu.

Hindari:

```text
Driver membuat task
Driver membaca MotionCommand
Driver menjalankan PID
Driver menyimpan mission
Driver langsung mengakses queue
Task langsung memanggil driver
Serial command langsung mengontrol motor
```

Jika driver butuh config hardware, taruh di driver/config yang jelas, bukan di Logic.

## 12. Build Target

Project saat ini memakai:

```text
ESP-IDF 5.4.4
target default: esp32
```

Build check:

```bash
source /home/ibrohim/.espressif/v5.4.4/esp-idf/export.sh
idf.py build
```

