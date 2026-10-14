# Integrasi GY-511, Motor Differential, dan Track S

## Tujuan

GY-511 memberi feedback arah agar kendaraan:

- mempertahankan heading pada bagian lintasan lurus;
- menyelesaikan belok kanan/kiri berdasarkan sudut, bukan delay;
- mengoreksi perbedaan kecepatan roda kiri dan kanan;
- menyediakan `heading_deg` dan `yaw_dps` pada log pengujian.

Alur software tetap mengikuti arsitektur proyek:

```text
MotionLoopTask
    -> HeadingService
        -> IMUDriver
            -> GY-511 melalui I2C

MotionLoopTask
    -> MotorControlService
        -> MotorEncoderDriver kiri dan kanan
            -> BTS7960 kiri dan kanan
```

## Sensor yang Didukung

Implementasi awal ditujukan untuk GY-511 berbasis LSM303DLHC:

- accelerometer 3-axis pada alamat I2C `0x18` atau `0x19`;
- magnetometer 3-axis pada alamat I2C `0x1E`;
- `WHO_AM_I_A` yang diharapkan adalah `0x33`.

LSM303DLHC bukan gyroscope. `yaw_dps` dihitung dari perubahan heading kompas
terhadap waktu agar tetap kompatibel dengan kontrak `HeadingService` yang ada.
Metode ini lebih mudah terpengaruh motor, kabel arus besar, besi, dan medan
magnet sekitar dibanding IMU yang memiliki gyroscope.

## Wiring GY-511 ke ESP32

| Pin GY-511 | ESP32 | Catatan |
|---|---|---|
| VCC | 3V3 | Gunakan 3,3 V untuk level logika yang aman |
| GND | GND | Harus common ground dengan ESP32 dan BTS7960 |
| SDA | GPIO21 | I2C data |
| SCL | GPIO22 | I2C clock |
| DRDY/INT | Tidak dipasang | Belum digunakan |

Konfigurasi berada di `main/config/IMUHardwareConfig.hpp`. Jangan meletakkan
GY-511 dekat motor, BTS7960, kabel baterai, magnet speaker, atau rangka besi.
Pasang board rata dan kaku pada chassis.

## Konvensi Arah

Konvensi proyek:

```text
heading positif = rotasi kanan / clockwise
TURN +angle      = belok kanan
TURN -angle      = belok kiri
motor positif    = roda bergerak maju
```

Pivot turn yang benar:

| Perintah | Roda kiri | Roda kanan |
|---|---:|---:|
| Kanan | maju | mundur |
| Kiri | mundur | maju |

Ketika bergerak lurus dan heading perlu dikoreksi ke kanan, roda kiri dibuat
lebih cepat daripada roda kanan. Masing-masing roda tetap memiliki speed PID
sendiri, sehingga sinkronisasi memakai feedback encoder, bukan duty PWM yang
dianggap sama.

Jika kendaraan diputar manual ke kanan tetapi `yaw_dps` negatif, ubah
`INVERT_HEADING` menjadi `true` di `IMUHardwareConfig.hpp`.

## Interpretasi Track S

Dengan arah panah pada gambar, urutan gerak konseptualnya adalah:

```text
P0 -> P1: lurus ke kanan
ujung atas: belok kanan sekitar 180 derajat
P2 -> P3: lurus ke kiri
ujung tengah: belok kiri sekitar 180 derajat
P4 -> akhir: lurus ke kanan
```

Urutan command produk nantinya:

```text
MOVE distance_top speed_profile
TURN +180
MOVE distance_middle speed_profile
TURN -180
MOVE distance_bottom speed_profile
STOP
```

Nilai jarak lurus dan bentuk belok belum boleh dikunci sebelum parameter
`Ltrack`, lebar lintasan `W`, dan radius belok `R` tersedia. Untuk kurva yang
halus, command pivot `TURN 180` nantinya perlu dikembangkan menjadi kontrol
arc/curvature; pivot turn hanya tahap pembuktian integrasi motor dan heading.

Satu pengujian lintasan hanya memakai satu profil kecepatan:

- V1: 0,8 m/s untuk seluruh satu lintasan;
- V2: 1,3 m/s untuk seluruh satu lintasan;
- V3: 1,8 m/s untuk seluruh satu lintasan.

Target tersebut tetap tunduk pada batas motor yang dijelaskan di
`TRACK_SPEED_VALIDATION.md`.

## Kalibrasi Magnetometer

Kalibrasi dilakukan setelah sensor terpasang permanen pada mobil:

1. Matikan motor, tetapi biarkan seluruh elektronik berada pada posisi normal.
2. Putar kendaraan perlahan 360 derajat beberapa kali.
3. Catat nilai minimum dan maksimum magnetometer X, Y, dan Z.
4. Hitung offset tiap axis: `(maximum + minimum) / 2`.
5. Hitung half-range tiap axis: `(maximum - minimum) / 2`.
6. Gunakan rata-rata half-range dibagi half-range axis sebagai scale axis.
7. Masukkan hasil ke `MAG_OFFSET_*` dan `MAG_SCALE_*`.
8. Ulangi pengujian dengan motor hidup karena arus motor dapat mengubah medan.

Tanpa kalibrasi, sensor boleh dipakai untuk bring-up tetapi belum boleh menjadi
bukti akurasi belok.

## Urutan Approval Hardware

1. Flash dengan motor power dilepas dan pastikan log menampilkan
   `GY-511/LSM303 ready`.
2. Putar chassis manual ke kanan dan kiri; pastikan tanda `yaw_dps` sesuai.
3. Pastikan `heading_deg` kembali mendekati nilai awal setelah rotasi kembali.
4. Angkat seluruh roda, sambungkan motor power, lalu validasi maju/mundur.
5. Konfigurasikan BTS7960 serta encoder kanan; warning `GPIO_NUM_NC` harus hilang.
6. Uji pivot kanan: roda kiri maju dan roda kanan mundur.
7. Uji pivot kiri: roda kiri mundur dan roda kanan maju.
8. Uji heading hold pada kecepatan rendah di lintasan lurus.
9. Kalibrasikan PID heading dan speed PID berdasarkan log nyata.
10. Baru jalankan track S, mulai dari profil yang aman dan satu lintasan sekali.

Approval belok belum dapat diberikan selama motor kanan masih
`GPIO_NUM_NC`. Jangan menghubungkan hardware kanan ke pin tebakan.

## Referensi Sensor

- STMicroelectronics, `LSM303DLHC datasheet`:
  https://www.st.com/resource/en/datasheet/lsm303dlhc.pdf
- STMicroelectronics, `AN3192 - magnetometer calibration and tilt compensation`:
  https://www.st.com/resource/en/application_note/an3192-3axis-magnetometer-calibration-and-compensation-stmicroelectronics.pdf
