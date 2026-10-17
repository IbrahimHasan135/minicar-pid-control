# Wiring Dua BTS7960, Dua Motor Encoder, ESP32, dan GY-511

Konfigurasi ini ditujukan untuk kendaraan differential drive:

```text
Motor kiri  -> BTS7960 kiri  -> ESP32
Motor kanan -> BTS7960 kanan -> ESP32
GY-511      -> I2C ESP32
```

Satu BTS7960 mengendalikan tepat satu motor. Jangan menggabungkan kedua motor
ke satu output BTS7960 karena saat belok arah/kecepatan kedua roda harus dapat
dikontrol secara independen.

Mapping ini mengasumsikan ESP32 DOIT DevKit V1 berbasis modul ESP32-WROOM.
Jika tulisan pada shield logam adalah `ESP32-WROVER`, verifikasi ulang GPIO16
dan GPIO17 karena varian tersebut dapat memakainya untuk PSRAM.

## Ringkasan Pin ESP32

| Perangkat | Sinyal | GPIO |
|---|---|---:|
| BTS kiri | RPWM | 25 |
| BTS kiri | LPWM | 26 |
| BTS kiri | R_EN | 27 |
| BTS kiri | L_EN | 14 |
| Encoder kiri | A | 32 |
| Encoder kiri | B | 33 |
| BTS kanan | RPWM | 18 |
| BTS kanan | LPWM | 19 |
| BTS kanan | R_EN | 23 |
| BTS kanan | L_EN | 13 |
| Encoder kanan | A | 16 |
| Encoder kanan | B | 17 |
| GY-511 | SDA | 21 |
| GY-511 | SCL | 22 |

## BTS7960 Kiri

### Header kontrol

| BTS7960 kiri | Sambungan |
|---|---|
| RPWM | ESP32 GPIO25 |
| LPWM | ESP32 GPIO26 |
| R_EN | ESP32 GPIO27 |
| L_EN | ESP32 GPIO14 |
| VCC | ESP32 5V/VIN dari USB |
| GND | Common GND |
| R_IS | Tidak dipasang |
| L_IS | Tidak dipasang |

### Terminal daya dan motor

| BTS7960 kiri | Sambungan |
|---|---|
| B+ | Baterai +12 V melalui fuse dan emergency switch |
| B- | Baterai negatif/common GND |
| M+ | Pin 6 Motor+ dinamo kiri |
| M- | Pin 1 Motor- dinamo kiri |

### Encoder kiri

| Pin konektor dinamo kiri | Fungsi | Sambungan |
|---:|---|---|
| 2 | Encoder GND | Common GND |
| 3 | Encoder A | ESP32 GPIO32 |
| 4 | Encoder B | ESP32 GPIO33 |
| 5 | Encoder VCC | ESP32 3V3 |

## BTS7960 Kanan

### Header kontrol

| BTS7960 kanan | Sambungan |
|---|---|
| RPWM | ESP32 GPIO18 |
| LPWM | ESP32 GPIO19 |
| R_EN | ESP32 GPIO23 |
| L_EN | ESP32 GPIO13 |
| VCC | ESP32 5V/VIN dari USB |
| GND | Common GND |
| R_IS | Tidak dipasang |
| L_IS | Tidak dipasang |

### Terminal daya dan motor

| BTS7960 kanan | Sambungan |
|---|---|
| B+ | Baterai +12 V melalui fuse dan emergency switch |
| B- | Baterai negatif/common GND |
| M+ | Pin 6 Motor+ dinamo kanan |
| M- | Pin 1 Motor- dinamo kanan |

### Encoder kanan

| Pin konektor dinamo kanan | Fungsi | Sambungan |
|---:|---|---|
| 2 | Encoder GND | Common GND |
| 3 | Encoder A | ESP32 GPIO16 |
| 4 | Encoder B | ESP32 GPIO17 |
| 5 | Encoder VCC | ESP32 3V3 |

Motor kanan diasumsikan dipasang mirror terhadap motor kiri. Karena itu config
awal memakai `invert_motor=true` dan `invert_encoder=true` untuk sisi kanan.
Jangan langsung menukar kabel ketika arah tidak sesuai; catat dahulu arah roda
dan tanda RPM, lalu koreksi flag secara sistematis.

## GY-511

| GY-511 | Sambungan |
|---|---|
| VCC | ESP32 3V3 |
| GND | Common GND |
| SDA | ESP32 GPIO21 |
| SCL | ESP32 GPIO22 |
| DRDY/INT | Tidak dipasang |

Pasang GY-511 sejauh mungkin dari motor, BTS7960, kabel baterai, rangka besi,
dan magnet. Pasang rata dan kaku pada chassis.

## Distribusi Daya

```text
Baterai +12 V
    -> fuse utama
        -> emergency switch
            +-> B+ BTS kiri
            `-> B+ BTS kanan

Baterai negatif
    +-> B- BTS kiri
    +-> B- BTS kanan
    `-> common GND ESP32, kedua encoder, dan GY-511

ESP32 5V/VIN dari USB
    +-> VCC logika BTS kiri
    `-> VCC logika BTS kanan

ESP32 3V3
    +-> VCC encoder kiri
    +-> VCC encoder kanan
    `-> VCC GY-511
```

Jangan pernah menyambungkan 12 V ke pin 5V, 3V3, GPIO, encoder, GY-511, atau
VCC header kontrol BTS7960. Gunakan kabel daya dan fuse yang sesuai dengan arus
stall kedua motor. Jangan merakit atau mengubah kabel ketika baterai terpasang.

## Urutan Test

Firmware default memakai:

```cpp
constexpr bool RUN_MOTOR_TEST_AFTER_IMU = false;
```

### Tahap 1 - IMU saja

1. Baterai 12 V dilepas.
2. Hubungkan ESP32 melalui USB.
3. Build, flash, dan buka monitor.
4. Pastikan kedua motor diam.
5. Putar chassis kanan/kiri selama fase `imu_read_test`.

### Tahap 2 - Dua motor dengan roda terangkat

1. Ubah `RUN_MOTOR_TEST_AFTER_IMU` menjadi `true`.
2. Build dan flash ketika baterai 12 V masih mati.
3. Angkat kedua roda dan siapkan emergency switch.
4. Nyalakan 12 V lalu tekan EN/RESET ESP32.
5. Firmware menjalankan urutan berikut:

```text
both_forward_low:  kiri +30 RPM, kanan +30 RPM
both_reverse_low:  kiri -30 RPM, kanan -30 RPM
pivot_right_low:   kiri +20 RPM, kanan -20 RPM
pivot_left_low:    kiri -20 RPM, kanan +20 RPM
```

## Kriteria Tanda yang Benar

| Fase | Roda kiri | Roda kanan | RPM yang diharapkan |
|---|---|---|---|
| both_forward_low | kendaraan maju | kendaraan maju | keduanya positif |
| both_reverse_low | kendaraan mundur | kendaraan mundur | keduanya negatif |
| pivot_right_low | maju | mundur | kiri positif, kanan negatif |
| pivot_left_low | mundur | maju | kiri negatif, kanan positif |

Jika arah fisik roda benar tetapi tanda RPM salah, ubah hanya
`invert_encoder` sisi tersebut. Jika arah fisik salah sekaligus tanda RPM
mengikuti arah yang salah, periksa `invert_motor` lalu validasi kembali encoder.
