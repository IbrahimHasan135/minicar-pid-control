# Panduan Pengembangan Driver Motor Mini Car

Dokumen ini adalah titik awal untuk fokus mengembangkan motor dan encoder pada
repo `minicar-pid-control`. Target project adalah ESP32 DevKit V1, ESP-IDF
5.4.4, C++, dan FreeRTOS.

## 1. Gambaran Singkat Seluruh Repo

Aliran dependency project adalah:

```text
Application -> Logic/Task -> Service -> Driver -> Hardware
```

| Folder/file | Peran | Hubungan dengan pekerjaan motor |
|---|---|---|
| `main/application/` | Membuat urutan command/mission | Tidak perlu diubah untuk membuat driver |
| `main/logic/motion_control/` | State machine gerak dan control loop 100 Hz | Mengirim target ke service, tidak boleh mengakses GPIO/PWM langsung |
| `main/service/motor/` | Konversi RPM ke m/s dan PID kecepatan roda | Satu-satunya pemilik `MotorEncoderDriver` |
| `main/service/control/` | PID generik | Dipakai oleh `MotorControlService`, bukan oleh driver |
| `main/driver/motor/` | Akses GPIO, PWM/LEDC, dan encoder/PCNT | Fokus implementasi utama |
| `main/config/` | Konstanta robot, PID, dan task | Tempat parameter fisik dan nanti konfigurasi pin/peripheral |
| `main/model/` | Bentuk data bersama | Tidak menyentuh hardware |
| `main/app_main.cpp` | Membuat object dan menghubungkan dependency | Memanggil `motor_service.init()` saat startup |
| `main/CMakeLists.txt` | Daftar source dan komponen ESP-IDF | Perlu diperbarui bila driver membutuhkan komponen baru |
| `docs/` | Arsitektur dan aturan project | Baca sebelum mengubah boundary layer |

Catatan: `main/main.c` tidak tercantum di `main/CMakeLists.txt`, sehingga entry
point aktif adalah `main/app_main.cpp`.

## 2. Alur Motor yang Sudah Ada

```text
MotionLoopTask (setiap 10 ms)
    -> MotorControlService.refreshFeedback()
        -> MotorEncoderDriver.getLeft/RightTicks()
        -> MotorEncoderDriver.getLeft/RightRPM()
    -> MotorControlService.setVelocityTargets(left_mps, right_mps)
    -> PID kiri dan kanan menghasilkan output -1.0 sampai +1.0
    -> MotorControlService.applyControl(dt_s)
        -> MotorEncoderDriver.setLeft/RightOutput(output)
            -> GPIO arah + duty PWM hardware
```

`MotorEncoderDriver` hanya mengurus hardware. Jangan menaruh PID, command
MOVE/TURN, queue, odometry, atau keputusan gerak selesai di dalam driver.

## 3. Kondisi Implementasi Saat Ini

File utama:

```text
main/driver/motor/MotorEncoderDriver.hpp
main/driver/motor/MotorEncoderDriver.cpp
```

Semua API sudah dibuat, tetapi hardware masih berupa skeleton:

- `init()` selalu mengembalikan `ESP_ERR_NOT_SUPPORTED`;
- output hanya disimpan ke variable, belum menjadi sinyal GPIO/PWM;
- ticks dan RPM selalu nol;
- `stop()` belum menghentikan hardware;
- akibatnya `MotorControlService::isInitialized()` selalu `false` dan
  `MotionLoopTask` masuk fail-safe `ERROR`.

Artinya prioritas pertama bukan tuning PID. Prioritas pertama adalah membuat
init, open-loop motor, dan pembacaan encoder benar serta aman.

## 4. Data Hardware yang Harus Diisi Sebelum Coding

Jangan menebak pin atau karakteristik board. Isi tabel ini dari wiring dan
datasheet:

| Parameter | Left | Right | Keterangan |
|---|---:|---:|---|
| Pin PWM/enable | TBD | TBD | GPIO output-capable |
| Pin arah IN1 | TBD | TBD | Sesuai H-bridge |
| Pin arah IN2 (jika ada) | TBD | TBD | Tidak semua driver memerlukannya |
| Pin encoder A | TBD | TBD | Input PCNT |
| Pin encoder B | TBD | TBD | Isi jika quadrature |
| Polaritas motor dibalik? | TBD | TBD | `output > 0` wajib membuat roda maju |
| Polaritas encoder dibalik? | TBD | TBD | Maju wajib menghasilkan ticks/RPM positif |
| LEDC timer/channel | TBD | TBD | Channel kiri dan kanan berbeda |
| PCNT unit/channel | TBD | TBD | Unit kiri dan kanan berbeda |

Parameter bersama yang juga harus dipastikan:

- nama dan tipe H-bridge/motor driver;
- tegangan motor dan batas duty yang aman;
- frekuensi serta resolusi PWM yang didukung;
- mode stop yang diinginkan: coast atau brake;
- encoder single-channel atau quadrature;
- pulses per revolution encoder;
- apakah angka itu dihitung per motor shaft atau output shaft;
- rasio gearbox;
- faktor hitung edge (`x1`, `x2`, atau `x4`).

`ENCODER_TICKS_PER_REV = 360` harus berarti jumlah count yang benar-benar
dilaporkan PCNT untuk satu putaran roda. Jangan langsung menyalin nilai PPR dari
datasheet tanpa memperhitungkan gearbox dan mode edge counting.

## 5. Kontrak API Driver

### `esp_err_t init()`

Urutan yang disarankan:

1. Pastikan output motor berada pada keadaan aman sebelum konfigurasi lanjut.
2. Konfigurasi GPIO arah kiri dan kanan.
3. Konfigurasi LEDC timer dan dua channel PWM.
4. Set duty kedua motor ke nol.
5. Konfigurasi PCNT kiri dan kanan, termasuk glitch filter bila diperlukan.
6. Clear lalu start kedua counter.
7. Inisialisasi state sampling RPM.
8. Kembalikan `ESP_OK` hanya jika seluruh tahap berhasil.

Jika salah satu tahap gagal, panggil safe stop dan kembalikan error ESP-IDF.
Jangan mengembalikan `ESP_OK` untuk hardware yang hanya terinisialisasi sebagian.

### `setLeftOutput(float)` dan `setRightOutput(float)`

Kontrak output dari service adalah normalized command:

```text
-1.0 = putaran reverse maksimum
 0.0 = stop
+1.0 = putaran forward maksimum
```

Implementasi driver harus:

1. menolak nilai non-finite (`NaN`/infinity) dengan safe stop;
2. clamp nilai ke `[-1.0, +1.0]`;
3. tentukan arah dari tanda nilai;
4. ubah `abs(output)` menjadi duty LEDC;
5. terapkan inversi motor per sisi di driver/config, bukan di logic;
6. perlakukan output nol sesuai keputusan brake/coast.

Saat berganti arah, gunakan urutan aman: duty nol terlebih dahulu, ubah pin
arah, kemudian terapkan duty baru. Dead-time tambahan hanya digunakan bila
memang diwajibkan datasheet H-bridge; jangan membuat control loop blocking.

### `getLeftTicks()` dan `getRightTicks()`

Keduanya mengembalikan posisi encoder kumulatif bertanda (`int32_t`):

```text
roda maju   -> ticks bertambah
roda mundur -> ticks berkurang
```

Nilai digunakan oleh `OdometryService`, jadi jangan mengembalikan delta per
sampel. Jika PCNT memakai batas counter 16-bit, driver harus menangani overflow
atau mengakumulasikannya agar API tetap terlihat sebagai counter 32-bit.

### `getLeftRPM()` dan `getRightRPM()`

Keduanya mengembalikan RPM roda bertanda, bukan RPM shaft motor:

```text
rpm = delta_ticks / ticks_per_revolution * 60 / delta_time_seconds
```

Karena getter saat ini tidak menerima `dt_s`, driver perlu menyimpan hasil
sampling/cache beserta timestamp internal. Hindari menghitung RPM kiri dan kanan
dengan interval waktu yang berbeda jauh. Bila feedback belum valid, lebih aman
mengembalikan nol dan mencatat status internal daripada menghasilkan angka acak.

Untuk kecepatan rendah, pertimbangkan filtering ringan atau measurement period
yang lebih panjang. Jangan menaruh PID di sini.

### `stop()`

`stop()` adalah jalur keselamatan dan harus tetap bekerja walaupun init gagal
sebagian:

- duty PWM kiri dan kanan menjadi nol;
- pin arah masuk keadaan brake/coast yang telah dipilih;
- state output internal menjadi nol;
- pemanggilan berulang harus aman (idempotent).

Encoder tidak harus di-reset oleh `stop()`, karena odometry menggunakan count
kumulatif.

## 6. Struktur Config yang Disarankan

Pindahkan pin dan konfigurasi peripheral ke file config tersendiri, misalnya:

```text
main/config/MotorHardwareConfig.hpp
```

Isinya dapat mencakup pin, PWM frequency/resolution, LEDC channel, PCNT unit,
inversi motor/encoder, serta glitch filter. Nilai fisik robot tetap berada di
`RobotConfig.hpp`:

```text
WHEEL_DIAMETER_M
WHEEL_BASE_M
ENCODER_TICKS_PER_REV
MAX_MOTOR_OUTPUT
```

Jangan menyebarkan magic number pin dan PWM di beberapa function.

## 7. Urutan Pengembangan yang Disarankan

### Tahap A — Finalisasi kontrak hardware

- Lengkapi tabel hardware di bagian 4.
- Cocokkan wiring dengan datasheet H-bridge dan batas GPIO ESP32.
- Tentukan brake/coast serta convention tanda.
- Koreksi diameter roda dan ticks per revolution aktual.

### Tahap B — PWM dan arah motor tanpa PID

- Implementasikan GPIO, LEDC, clamp, direction, dan `stop()`.
- Uji tiap roda secara terpisah dengan output kecil.
- Pastikan command positif menggerakkan kendaraan maju.
- Pastikan boot, error init, dan output nol tidak membuat motor bergerak.

Jangan menjalankan roda terangkat tanpa batas waktu atau langsung memakai duty
100%. Sediakan emergency power-off saat pengujian awal.

### Tahap C — Encoder/PCNT

- Implementasikan counter kiri dan kanan.
- Verifikasi tanda count saat roda diputar manual.
- Putar roda tepat satu revolusi dan cocokkan count dengan
  `ENCODER_TICKS_PER_REV`.
- Verifikasi overflow dan pembacaan count yang konsisten.

### Tahap D — Estimasi RPM

- Hitung delta ticks berdasarkan timestamp monotonic.
- Validasi RPM maju, mundur, nol, dan kecepatan rendah.
- Tambahkan filter hanya setelah melihat noise data nyata.

### Tahap E — Integrasi service

- Pastikan `motor_service.init()` menghasilkan `ESP_OK`.
- Pantau target m/s, measured RPM/m/s, dan normalized motor output.
- Uji setpoint kecil sebelum setpoint maksimum.
- Tuning PID kiri dan kanan setelah feedback encoder terbukti benar.

### Tahap F — Integrasi gerak

- Uji STOP terlebih dahulu.
- Uji maju pendek, mundur pendek, kemudian turn.
- Ukur jarak nyata dan arah heading.
- Baru setelah itu tuning toleransi completion dan mission penuh.

## 8. Checklist Pengujian

### Tanpa roda menyentuh lantai

- [ ] Saat boot, kedua output motor nol.
- [ ] `init()` gagal dengan error yang jelas jika peripheral gagal.
- [ ] Output di luar range ter-clamp.
- [ ] Output non-finite memicu stop.
- [ ] Output positif memutar masing-masing roda ke arah maju.
- [ ] Output negatif membalik arah dengan aman.
- [ ] `stop()` selalu menghasilkan duty nol.
- [ ] Encoder maju positif dan mundur negatif.
- [ ] Satu putaran roda menghasilkan ticks yang diharapkan.
- [ ] RPM kembali mendekati nol saat roda berhenti.

### Dengan kendaraan di lantai

- [ ] Kendaraan berhenti saat startup dan ketika feedback tidak valid.
- [ ] Kedua roda mampu mengikuti setpoint rendah tanpa osilasi berbahaya.
- [ ] MOVE positif maju dan MOVE negatif mundur.
- [ ] Tidak ada sisi motor/encoder yang tandanya terbalik.
- [ ] STOP bekerja selama gerakan.
- [ ] Kabel encoder tidak menimbulkan false counts berlebihan.
- [ ] Driver/H-bridge/motor tidak overheat.

## 9. Kriteria Selesai Driver

Driver motor dianggap selesai ketika:

- `init()` mengonfigurasi GPIO, LEDC, dan PCNT serta mengembalikan error nyata;
- output normalized diterjemahkan ke arah dan duty secara benar;
- `stop()` aman pada semua jalur, termasuk init parsial;
- ticks kumulatif dan RPM roda bertanda benar;
- overflow counter dan noise dasar encoder ditangani;
- tidak ada PID, queue, state machine, atau delay gerakan di driver;
- hanya `MotorControlService` yang menggunakan driver;
- build ESP-IDF bersih dan pengujian hardware pada checklist lulus.

## 10. File yang Kemungkinan Diubah

Untuk pekerjaan motor saja, scope normalnya:

```text
main/driver/motor/MotorEncoderDriver.hpp
main/driver/motor/MotorEncoderDriver.cpp
main/config/MotorHardwareConfig.hpp       (baru, disarankan)
main/config/RobotConfig.hpp               (jika parameter fisik dikoreksi)
main/CMakeLists.txt                       (dependency komponen ESP-IDF)
```

Jangan mulai dengan mengubah `MotionLoopTask`, `Mission`, atau PID. Ubah bagian
tersebut hanya setelah driver dan data feedback sudah tervalidasi.

## 11. Hal yang Perlu Diwaspadai di Repo Saat Ini

- `main/CMakeLists.txt` baru mencantumkan `freertos` dan `log`; implementasi
  LEDC/GPIO/PCNT mungkin memerlukan dependency driver ESP-IDF tambahan sesuai
  header/API yang dipilih.
- API getter RPM saat ini tidak membawa `dt_s`; tetapkan satu mekanisme sampling
  internal yang konsisten atau revisi kontraknya secara sadar bersama service.
- PID awal (`SPEED_KP`, `SPEED_KI`, `SPEED_KD`) hanyalah nilai awal, bukan hasil
  tuning hardware.
- Diameter roda dan ticks per revolution memengaruhi sekaligus speed feedback
  dan odometry. Kesalahan di sini terlihat seperti PID atau jarak yang salah.
- Driver harus aman dipanggil dari control loop 100 Hz dan tidak boleh blocking.

Pegangan utamanya:

```text
Driver menghasilkan hardware I/O yang aman dan feedback yang benar.
Service mengubah feedback menjadi unit engineering dan menjalankan PID.
Logic memutuskan kendaraan harus bergerak bagaimana dan kapan selesai.
```
