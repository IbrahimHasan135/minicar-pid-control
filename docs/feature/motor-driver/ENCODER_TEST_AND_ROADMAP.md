# Encoder Test dan Roadmap Motor Driver

## Scope Saat Ini

Tahap pertama hanya membaca encoder pada satu motor 25GA370. BTS7960 belum
diaktifkan dan motor tidak diputar otomatis. Output shaft diputar manual agar
wiring, arah count, counts per revolution, dan estimasi RPM dapat diverifikasi
sebelum mengembangkan PWM motor.

## Asumsi yang Harus Diverifikasi

Nama `25GA370 12V 100RPM` belum cukup untuk menentukan spesifikasi encoder.
Vendor dapat menjual motor dengan PPR dan rasio gearbox berbeda. Pastikan dari
datasheet atau label produk:

- encoder memiliki kanal A dan B (quadrature);
- tegangan supply encoder;
- level tegangan output A/B;
- pulse per revolution pada motor shaft;
- rasio gearbox;
- output encoder open-collector atau push-pull.

ESP32 hanya menerima logika sekitar 3,3 V. Jangan menghubungkan sinyal encoder
5 V langsung ke GPIO ESP32. Gunakan supply encoder 3,3 V jika modul memang
mendukungnya, atau gunakan level shifter/rangkaian yang benar.

## Wiring Tester Satu Encoder

Konfigurasi sementara berada di `main/config/MotorHardwareConfig.hpp`:

| Encoder | ESP32 default | Catatan |
|---|---:|---|
| GND | GND | Ground wajib tersambung bersama |
| VCC | sesuai datasheet | Jangan menebak 3,3 V atau 5 V |
| A | GPIO32 | Bisa diganti di config |
| B | GPIO33 | Bisa diganti di config |

GPIO32 dan GPIO33 dipilih untuk tester karena mendukung input dan internal
pull-up. Hindari GPIO34-39 bila mengandalkan internal pull-up karena pin tersebut
tidak memilikinya pada ESP32 klasik.

BTS7960 dan supply motor 12 V tidak dibutuhkan dalam pengujian manual ini.

## Cara Kerja Program

```text
Encoder A/B
    -> GPIO matrix ESP32
    -> dua channel PCNT quadrature x4
    -> counter bertanda dan terakumulasi
    -> MotorEncoderDriver.getTicks()
    -> delta ticks / waktu / counts-per-rev
    -> MotorEncoderDriver.getRPM()
    -> log setiap 250 ms dari main.c
```

PCNT membaca rising dan falling edge dari kedua kanal. Karena itu satu siklus
quadrature menghasilkan empat count. `accum_count` dan watch point dipakai agar
count tidak berhenti pada batas counter hardware 16-bit.

## Cara Menjalankan

Gunakan terminal ESP-IDF 5.4.4:

```powershell
idf.py set-target esp32
idf.py build
idf.py -p COMx flash monitor
```

Ganti `COMx` dengan port board. Setelah boot:

1. Biarkan shaft diam; ticks harus tetap dan RPM mendekati nol.
2. Putar shaft perlahan ke arah yang dianggap maju; ticks harus bertambah.
3. Jika arah maju negatif, ubah `INVERT_ENCODER_DIRECTION` menjadi `true`.
4. Tandai posisi shaft, putar tepat satu putaran, dan catat delta ticks.
5. Ganti `ENCODER_COUNTS_PER_OUTPUT_REV` dengan hasil pengukuran aktual.
6. Putar beberapa putaran dengan kecepatan relatif konstan dan periksa RPM.

Nilai awal `1496` hanyalah hipotesis `11 PPR * rasio 34 * quadrature x4`, bukan
spesifikasi resmi motor pengguna. Hasil satu putaran output shaft adalah sumber
nilai yang dipakai sampai datasheet terkonfirmasi.

## Interpretasi Hasil

| Gejala | Kemungkinan | Pemeriksaan |
|---|---|---|
| Ticks selalu nol | Wiring/pin/supply salah | Ukur level A dan B, cek pin config |
| Ticks hanya satu arah | Kanal B tidak terbaca | Cek sinyal B dan common ground |
| Ticks berubah saat diam | Input floating/noise | Cek level logic, pull-up, shielding, filter |
| Arah terbalik | Orientasi A/B berbeda | Aktifkan `INVERT_ENCODER_DIRECTION` |
| RPM salah tetapi ticks benar | Counts/rev salah | Ukur ulang satu putaran output shaft |
| Count lompat/hilang | Noise atau pulsa terlalu cepat | Cek wiring, logic analyzer, glitch filter |

## Roadmap Pengembangan dan Branch

Aturan project menyatakan perubahan kecil harus di-commit dan push, serta tidak
membuat branch bertingkat. Pekerjaan aktif tetap di `feat/motor-driver` sampai
scope motor/encoder siap direview. Branch baru nantinya dibuat dari `main` yang
sudah menerima hasil merge, bukan dari feature branch ini.

### Fase 1 — Satu encoder manual (`feat/motor-driver`, sekarang)

- [x] Isolasi build ke `main.c` dan driver motor.
- [x] Tambahkan konfigurasi pin encoder.
- [x] Implementasikan quadrature PCNT dan pembacaan ticks.
- [x] Tambahkan estimasi RPM untuk tester.
- [ ] Build dengan ESP-IDF 5.4.4.
- [ ] Validasi level tegangan A/B.
- [ ] Validasi arah dan noise saat diam.
- [ ] Ukur counts per output revolution aktual.
- [ ] Simpan hasil pengujian hardware di dokumen ini.

### Fase 2 — Dua encoder (`feat/motor-driver`)

- Buat dua instance `MotorEncoderDriver`, masing-masing dengan config/constructor
  pin sendiri; jangan hard-code satu global pin pair.
- Validasi encoder kiri dan kanan secara terpisah.
- Tetapkan convention: kendaraan maju menghasilkan ticks/RPM positif pada
  kedua roda.
- Uji pembacaan bersamaan dan overflow counter.

### Fase 3 — BTS7960 open-loop (`feat/motor-driver`)

- Catat pin `RPWM`, `LPWM`, `R_EN`, dan `L_EN` per motor driver.
- Pastikan ground ESP32 dan BTS7960 common, tetapi supply motor 12 V ditangani
  sesuai wiring daya yang aman.
- Implementasikan LEDC dan safe stop.
- Uji duty kecil dengan roda terangkat: satu motor, satu arah, lalu reverse.
- Uji encoder saat motor digerakkan dan cek konsistensi tanda output/feedback.

Satu BTS7960 mengendalikan satu motor DC. Mobil differential drive dengan dua
motor membutuhkan dua module BTS7960 atau driver dua kanal yang sesuai.

### Fase 4 — Driver siap produk (`feat/motor-driver`)

- Tangani kegagalan init parsial dan cleanup resource.
- Tambahkan validasi output non-finite dan clamp `[-1, 1]`.
- Finalisasi brake/coast, PWM frequency, dead-time bila diperlukan, filter,
  timeout feedback, dan telemetry error.
- Build bersih dan selesaikan checklist driver pada panduan utama.
- Buat pull request `feat/motor-driver` ke `main`.

### Fase 5 — Integrasi setelah merge

Setelah `feat/motor-driver` di-merge ke `main`, buat branch dari `main` untuk
scope terpisah bila dibutuhkan:

```text
feat/motor-service-integration  -> hubungkan dua driver ke MotorControlService
feat/motor-pid-tuning           -> tuning closed-loop berdasarkan data nyata
fix/motor-encoder-noise         -> hanya jika ada bug/noise spesifik
```

Logic motion, mission, IMU, dan serial tidak diubah selama validasi driver.

## Catatan Hasil Hardware

Isi setelah pengujian:

```text
Tanggal:
Model/vendor motor:
Supply encoder:
Level output A/B:
GPIO A/B:
Count satu putaran maju:
Count satu putaran mundur:
RPM no-load terukur:
Noise saat diam:
Keputusan invert direction:
Catatan wiring/filter:
```

