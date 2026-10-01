# Motor Driver Bench Test dan Roadmap

## Scope Saat Ini

Tahap ini menguji motor kiri melalui jalur project yang benar:

```text
main.c bench -> MotorControlService -> MotorEncoderDriver -> BTS7960 + encoder
```

Targetnya bukan tuning PID final. Targetnya memastikan hardware motor bergerak,
arah output sesuai, encoder memberi feedback, dan log monitor bisa dipakai
sebagai bukti approve awal.

## Asumsi Hardware

Hardware yang dipakai:

- High Torque Motor DC 25GA370 12V 100RPM dengan encoder.
- BTS7960 H-bridge 43A untuk satu motor DC.
- ESP32 DOIT DevKit dengan expansion I/O base.

Hal yang wajib diverifikasi dari wiring/datasheet:

- encoder punya kanal A dan B quadrature;
- output encoder aman untuk GPIO ESP32 3,3 V;
- ground ESP32, encoder, dan BTS7960 tersambung bersama;
- supply motor 12 V berasal dari supply eksternal, bukan dari ESP32;
- arah fisik maju disepakati sebelum mengubah logic lain.

Jangan menghubungkan sinyal encoder 5 V langsung ke ESP32. Gunakan supply
encoder 3,3 V bila modul mendukungnya, atau gunakan level shifter.

## Wiring Bench Motor Kiri

Konfigurasi sementara berada di `main/config/MotorHardwareConfig.hpp`.

### Kabel Dinamo 25GA370

Berdasarkan gambar pinout dinamo:

| Kabel dinamo | Fungsi | Sambungkan ke |
|---|---|---|
| White | Quad encoder B signal | GPIO33 melalui level shifter bila sinyal 5 V |
| Yellow | Quad encoder A signal | GPIO32 melalui level shifter bila sinyal 5 V |
| Blue | Quad encoder +5V VCC | 5 V encoder supply |
| Green | Quad encoder Ground | GND bersama ESP32 dan BTS7960 |
| Red | Motor power terminal (-) | BTS7960 motor output M- / OUT- |
| Black | Motor power terminal (+) | BTS7960 motor output M+ / OUT+ |

Catatan penting: label gambar menyebut encoder `+5V VCC`, jadi jangan langsung
memasukkan sinyal A/B 5 V ke GPIO ESP32. Pakai level shifter 5 V ke 3,3 V, atau
pastikan encoder benar-benar bisa diberi 3,3 V dan output A/B tidak melebihi
3,3 V sebelum disambungkan langsung.

### Pin BTS7960 dan ESP32

| Sinyal | ESP32 default | Catatan |
|---|---:|---|
| BTS7960 GND | GND | Common ground wajib |
| BTS7960 VCC logic | sesuai modul | Cek kebutuhan modul |
| BTS7960 RPWM | GPIO25 | PWM maju default |
| BTS7960 LPWM | GPIO26 | PWM mundur default |
| BTS7960 R_EN | GPIO27 | Di-set HIGH oleh driver |
| BTS7960 L_EN | GPIO14 | Di-set HIGH oleh driver |
| Motor supply | 12 V eksternal | Jangan dari ESP32 |
| Encoder GND | GND | Common ground wajib |
| Encoder VCC | sesuai datasheet | Pastikan level A/B aman |
| Encoder A | GPIO32 | PCNT channel A |
| Encoder B | GPIO33 | PCNT channel B |

`RIGHT_MOTOR` masih memakai `GPIO_NUM_NC`, sehingga service tetap bisa init
tetapi motor kanan tidak mengeluarkan PWM sampai pin kanannya diisi.

Ringkasnya untuk satu motor kiri:

```text
Dinamo black (+)  -> BTS7960 M+ / OUT+
Dinamo red (-)    -> BTS7960 M- / OUT-
BTS7960 B+ / VM   -> supply motor +12 V
BTS7960 B- / GND  -> supply motor ground
BTS7960 GND       -> ESP32 GND

Dinamo blue       -> encoder +5 V
Dinamo green      -> GND bersama
Dinamo yellow A   -> level shifter -> ESP32 GPIO32
Dinamo white B    -> level shifter -> ESP32 GPIO33

ESP32 GPIO25      -> BTS7960 RPWM
ESP32 GPIO26      -> BTS7960 LPWM
ESP32 GPIO27      -> BTS7960 R_EN
ESP32 GPIO14      -> BTS7960 L_EN
```

Jika fase `left_forward_low` membuat motor berputar mundur, jangan langsung
ubah logic. Ubah `invert_motor` menjadi `true` atau tukar terminal motor M+/M-.
Jika arah motor sudah benar tetapi ticks encoder negatif saat maju, ubah
`invert_encoder` menjadi `true`.

## Cara Kerja Program

```text
MotorControlService.setVelocityTargets()
    -> PID speed menghasilkan output normalized -1..+1
    -> MotorEncoderDriver.setOutput()
    -> LEDC menggerakkan BTS7960 RPWM/LPWM
    -> motor bergerak
    -> encoder A/B dibaca PCNT quadrature x4
    -> MotorControlService.refreshFeedback()
    -> log ticks dan velocity
```

Driver tidak berisi PID, command queue, state machine, atau keputusan mission.
PID tetap berada di `MotorControlService` sesuai aturan folder.

## Cara Menjalankan

Gunakan terminal ESP-IDF 5.4.4:

```powershell
idf.py set-target esp32
idf.py build
idf.py -p COMx flash monitor
```

Ganti `COMx` dengan port board.

Sebelum flash:

- pastikan roda terangkat;
- siapkan emergency power-off untuk supply motor;
- cek ulang pin `LEFT_MOTOR`;
- mulai dengan supply motor yang current limit bila tersedia.

## Expected Output Bench

Program `main.c` menjalankan urutan:

| Fase | Durasi | Target | Expected hardware | Expected log approve |
|---|---:|---:|---|---|
| safe stop | 2 s | 0 m/s | motor diam | ticks stabil |
| left_forward_low | 3 s | +0.12 m/s | motor putar maju pelan | left_ticks bertambah |
| stop | 1.5 s | 0 m/s | motor diam | velocity turun mendekati nol |
| left_reverse_low | 3 s | -0.12 m/s | motor putar mundur pelan | left_ticks berkurang |
| stop final | selesai | 0 m/s | motor diam | tidak ada gerak lanjut |

Contoh pola log yang dicari:

```text
PHASE=left_forward_low target_left=0.120 target_right=0.000
phase=left_forward_low left_ticks=... left_mps=...
PHASE=left_forward_low DONE ...
PHASE=left_reverse_low target_left=-0.120 target_right=0.000
phase=left_reverse_low left_ticks=... left_mps=...
PHASE=left_reverse_low DONE ...
```

Approve awal diberikan kalau:

- motor tidak bergerak saat boot dan stop;
- fase forward membuat arah fisik maju;
- fase reverse membuat arah fisik mundur;
- ticks forward positif dan ticks reverse negatif;
- velocity kembali mendekati nol saat stop.

Jika motor forward bergerak ke arah salah, ubah `invert_motor` pada config
motor atau tukar wiring motor. Jika arah fisik benar tetapi ticks terbalik, ubah
`invert_encoder`.

## Interpretasi Masalah

| Gejala | Kemungkinan | Pemeriksaan |
|---|---|---|
| Motor diam terus | Pin RPWM/LPWM/EN salah, supply motor mati, duty terlalu kecil | Cek wiring BTS7960 dan supply 12 V |
| Motor langsung bergerak saat boot | Wiring enable/PWM salah atau modul tidak safe | Cabut supply motor, cek pin config |
| Forward dan reverse terbalik | Polaritas motor/in1-in2 berbeda | Ubah `invert_motor` |
| Ticks selalu nol | Wiring encoder/pin/supply salah | Ukur A/B, cek GPIO32/33 |
| Ticks hanya satu arah | Kanal B tidak terbaca | Cek sinyal B dan common ground |
| Ticks berubah saat diam | Input floating/noise | Cek level logic, pull-up, shielding, filter |
| RPM salah tetapi ticks benar | Counts/rev salah | Ukur satu putaran output shaft |

## Roadmap Pengembangan dan Branch

Aturan project menyatakan perubahan kecil harus di-commit dan push, serta tidak
membuat branch bertingkat. Pekerjaan aktif tetap di `feat/motor-driver` sampai
scope motor/encoder siap direview.

### Fase 1 - Encoder manual

- [x] Isolasi build ke `main.c` dan driver motor.
- [x] Tambahkan konfigurasi pin encoder.
- [x] Implementasikan quadrature PCNT dan pembacaan ticks.
- [x] Tambahkan estimasi RPM untuk tester.
- [ ] Validasi level tegangan A/B.
- [ ] Validasi arah dan noise saat diam.
- [ ] Ukur counts per output revolution aktual.

### Fase 2 - BTS7960 satu motor via service

- [x] Tambahkan config pin BTS7960 per motor.
- [x] Implementasikan LEDC RPWM/LPWM dan safe stop di driver.
- [x] Jalankan bench lewat `MotorControlService`, bukan akses driver langsung.
- [ ] Build dengan ESP-IDF 5.4.4.
- [ ] Validasi motor diam saat boot.
- [ ] Validasi output positif membuat arah maju.
- [ ] Validasi output negatif membuat arah mundur.
- [ ] Validasi ticks maju positif dan mundur negatif.
- [ ] Simpan log monitor sebagai bukti approve.

### Fase 3 - Dua encoder dan dua motor

- Isi pin `RIGHT_MOTOR`.
- Validasi encoder kiri dan kanan secara terpisah.
- Tetapkan convention: kendaraan maju menghasilkan ticks/RPM positif pada
  kedua roda.
- Uji pembacaan bersamaan dan overflow counter.
- Pastikan dua BTS7960 dipakai untuk differential drive dua motor.

### Fase 4 - Driver siap produk

- Tangani kegagalan init parsial dan cleanup resource.
- Finalisasi brake/coast, PWM frequency, dead-time bila diperlukan, filter,
  timeout feedback, dan telemetry error.
- Build bersih dan selesaikan checklist driver pada panduan utama.
- Buat pull request `feat/motor-driver` ke `main`.

### Fase 5 - Integrasi setelah merge

Setelah `feat/motor-driver` di-merge ke `main`, buat branch dari `main` untuk
scope terpisah bila dibutuhkan:

```text
feat/motor-service-integration  -> full two-motor service integration
feat/motor-pid-tuning           -> tuning closed-loop berdasarkan data nyata
fix/motor-encoder-noise         -> hanya jika ada bug/noise spesifik
```

Logic motion, mission, IMU, dan serial tidak diubah selama validasi driver.

## Catatan Hasil Hardware

Isi setelah pengujian:

```text
Tanggal:
Model/vendor motor:
Supply motor:
Supply encoder:
Level output A/B:
GPIO RPWM/LPWM/R_EN/L_EN:
GPIO encoder A/B:
Forward direction fisik:
Ticks forward:
Ticks reverse:
RPM no-load terukur:
Noise saat diam:
Keputusan invert_motor:
Keputusan invert_encoder:
Catatan wiring/filter:
Link/commit bukti:
```
