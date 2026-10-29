# Wiring Lengkap Mini Car: ESP32, Dual BTS7960, Encoder, dan GY-511

Dokumen ini menjadi acuan wiring hardware untuk bench test pada branch
`feat/motor-driver`. Konfigurasi pin mengikuti:

- `main/config/MotorHardwareConfig.hpp`;
- `main/config/IMUHardwareConfig.hpp`;
- firmware motor-only bench test di `main/main.c`.

Hardware yang digunakan:

- ESP32 DOIT DevKit berbasis ESP32-WROOM;
- expansion I/O base plate untuk ESP32;
- dua BTS7960, masing-masing untuk satu motor;
- dua motor DC 25GA370 12 V 100 RPM dengan encoder quadrature;
- GY-511 berbasis LSM303DLHC;
- baterai motor 12 V.

> **Peringatan keselamatan:** angkat kedua roda ketika bench test, pasang fuse
> dekat baterai, siapkan emergency switch, dan putuskan baterai sebelum
> mengubah kabel. Jangan pernah memasukkan 12 V ke pin ESP32, expansion board,
> encoder, GY-511, atau VCC logic BTS7960.

## 1. Diagram Besar Sistem

```text
                         USB / regulated 5 V
                                  |
                              ESP32 + expansion
                                  |
              +-------------------+-------------------+
              |                   |                   |
       GPIO kontrol kiri   GPIO kontrol kanan      I2C 3,3 V
              |                   |                   |
         BTS7960 kiri        BTS7960 kanan          GY-511
              |                   |
          Motor kiri          Motor kanan
          + encoder           + encoder

Baterai +12 V -> fuse -> emergency switch -> B+ kedua BTS7960
Baterai negatif ---------------------------> B- kedua BTS7960
Baterai negatif ---------------------------> common GND logic
```

Satu BTS7960 hanya mengendalikan satu motor. Kedua motor tidak boleh dipasang
paralel pada satu BTS7960 karena kecepatan dan arah roda harus dapat dikontrol
secara independen ketika kendaraan berbelok.

## 2. Ringkasan Semua Pin ESP32

| Perangkat | Sinyal | Sambungan |
|---|---|---:|
| BTS7960 kiri | RPWM | 25 |
| BTS7960 kiri | LPWM | 26 |
| BTS7960 kiri | R_EN | 5 V langsung, bukan GPIO |
| BTS7960 kiri | L_EN | 5 V langsung, bukan GPIO |
| Encoder kiri | Phase A | 32 |
| Encoder kiri | Phase B | 33 |
| BTS7960 kanan | RPWM | 22 (sementara untuk diagnosis Arduino-equivalent) |
| BTS7960 kanan | LPWM | 23 (sementara untuk diagnosis Arduino-equivalent) |
| BTS7960 kanan | R_EN | 5 V langsung, bukan GPIO |
| BTS7960 kanan | L_EN | 5 V langsung, bukan GPIO |
| Encoder kanan | Phase A | 16 |
| Encoder kanan | Phase B | 17 |
| GY-511 | SDA | 21 |
| GY-511 | SCL | 22 (konflik; sensor wajib dilepas selama tes motor kanan) |

GPIO16 dan GPIO17 sesuai untuk ESP32-WROOM. Jika modul pada board bertuliskan
`ESP32-WROVER`, periksa kembali karena beberapa varian WROVER memakai pin
tersebut untuk PSRAM.

## 3. Konektor Motor dan Encoder 25GA370

Berdasarkan gambar pinout motor yang diberikan, urutan enam pin adalah:

| Pin motor | Warna pada gambar awal | Fungsi |
|---:|---|---|
| 1 | Merah | Motor power terminal (-) |
| 2 | Hijau | Encoder ground (-) |
| 3 | Kuning | Encoder A phase |
| 4 | Putih | Encoder B phase |
| 5 | Biru | Encoder VCC (+3,3/5 V, cek label unit) |
| 6 | Hitam | Motor power terminal (+) |

Warna kabel dapat berbeda antar-vendor. Prioritaskan nomor pin dan label pada
unit yang benar-benar dipakai, lalu verifikasi dengan datasheet atau multimeter.

### Perlindungan level encoder

GPIO ESP32 hanya aman pada logika sekitar 3,3 V dan **tidak 5 V tolerant**.
Gunakan salah satu skema berikut:

1. Jika encoder terbukti dapat bekerja pada 3,3 V, pasok pin 5 dari `3V3` dan
   hubungkan A/B langsung ke GPIO.
2. Jika encoder diwajibkan memakai 5 V, pasok pin 5 dari regulated `5V` dan
   turunkan kedua sinyal A/B menjadi 3,3 V menggunakan level shifter atau
   pembagi tegangan yang sesuai.

Jangan menyambungkan output A/B 5 V langsung ke ESP32. Ukur tegangan HIGH A/B
terhadap GND sebelum memasangnya ke GPIO jika spesifikasi encoder tidak jelas.

## 4. Wiring BTS7960 dan Motor Kiri

### Header kontrol BTS7960 kiri

| Pin BTS7960 kiri | Sambungkan ke | Fungsi |
|---|---|---|
| RPWM | ESP32 GPIO25 | PWM arah pertama |
| LPWM | ESP32 GPIO26 | PWM arah sebaliknya |
| R_EN | Regulated 5 V logic | Selalu enabled selama bench test |
| L_EN | Regulated 5 V logic | Selalu enabled selama bench test |
| VCC | Regulated 5 V logic | Bukan 12 V |
| GND | Common GND | Wajib tersambung dengan ESP32 |
| R_IS | Tidak dipasang | Current sense belum digunakan |
| L_IS | Tidak dipasang | Current sense belum digunakan |

### Terminal daya BTS7960 kiri

| Terminal | Sambungkan ke |
|---|---|
| B+ | Baterai +12 V setelah fuse dan emergency switch |
| B- | Baterai negatif/common ground |
| M+ | Pin 6 Motor+ kiri |
| M- | Pin 1 Motor- kiri |

### Encoder motor kiri

| Pin encoder kiri | Sambungkan ke |
|---|---|
| Pin 2, GND | Common GND |
| Pin 3, Phase A | GPIO32 melalui level shifter jika sinyal 5 V |
| Pin 4, Phase B | GPIO33 melalui level shifter jika sinyal 5 V |
| Pin 5, VCC | 3V3 jika didukung; jika tidak, regulated 5 V |

Ringkasan:

```text
ESP32 25 -> RPWM BTS kiri
ESP32 26 -> LPWM BTS kiri
Regulated 5 V -> R_EN BTS kiri
Regulated 5 V -> L_EN BTS kiri

Baterai +12 V -> fuse/switch -> B+ BTS kiri
Baterai negatif -------------> B- BTS kiri
BTS kiri M+ -----------------> Motor kiri pin 6
BTS kiri M- -----------------> Motor kiri pin 1

Motor kiri A -> level aman 3,3 V -> ESP32 GPIO32
Motor kiri B -> level aman 3,3 V -> ESP32 GPIO33
```

## 5. Wiring BTS7960 dan Motor Kanan

### Header kontrol BTS7960 kanan

| Pin BTS7960 kanan | Sambungkan ke | Fungsi |
|---|---|---|
| RPWM | ESP32 GPIO22 | PWM arah pertama; sama seperti sketch Arduino |
| LPWM | ESP32 GPIO23 | PWM arah sebaliknya; sama seperti sketch Arduino |
| R_EN | Regulated 5 V logic | Selalu enabled selama bench test |
| L_EN | Regulated 5 V logic | Selalu enabled selama bench test |
| VCC | Regulated 5 V logic | Bukan 12 V |
| GND | Common GND | Wajib tersambung dengan ESP32 |
| R_IS | Tidak dipasang | Current sense belum digunakan |
| L_IS | Tidak dipasang | Current sense belum digunakan |

### Terminal daya BTS7960 kanan

| Terminal | Sambungkan ke |
|---|---|
| B+ | Baterai +12 V setelah fuse dan emergency switch |
| B- | Baterai negatif/common ground |
| M+ | Pin 6 Motor+ kanan |
| M- | Pin 1 Motor- kanan |

### Encoder motor kanan

| Pin encoder kanan | Sambungkan ke |
|---|---|
| Pin 2, GND | Common GND |
| Pin 3, Phase A | GPIO16 melalui level shifter jika sinyal 5 V |
| Pin 4, Phase B | GPIO17 melalui level shifter jika sinyal 5 V |
| Pin 5, VCC | 3V3 jika didukung; jika tidak, regulated 5 V |

Ringkasan:

```text
ESP32 22 -> RPWM BTS kanan
ESP32 23 -> LPWM BTS kanan
Regulated 5 V -> R_EN BTS kanan
Regulated 5 V -> L_EN BTS kanan

Baterai +12 V -> fuse/switch -> B+ BTS kanan
Baterai negatif -------------> B- BTS kanan
BTS kanan M+ ----------------> Motor kanan pin 6
BTS kanan M- ----------------> Motor kanan pin 1

Motor kanan A -> level aman 3,3 V -> ESP32 GPIO16
Motor kanan B -> level aman 3,3 V -> ESP32 GPIO17
```

Motor kanan dipasang mirror terhadap motor kiri. Firmware saat ini memakai
`invert_motor=true` dan `invert_encoder=true` untuk sisi kanan agar command
positif dan RPM positif tetap berarti kendaraan bergerak maju.

Mapping GPIO22/23 ini bersifat sementara untuk menyamakan tes motor kanan
dengan sketch Arduino yang telah terbukti. GPIO22 juga merupakan SCL GY-511,
sehingga GY-511 wajib dilepas selama tes ini. Jangan menghubungkan SCL sensor
dan RPWM BTS ke GPIO22 secara bersamaan. Setelah power stage motor dinyatakan
lulus, salah satu fungsi harus dipindahkan ke GPIO lain sebelum integrasi IMU.

## 6. Wiring GY-511

| Pin GY-511 | Sambungkan ke | Catatan |
|---|---|---|
| VCC/VIN | ESP32 3V3 | Pilihan aman untuk level I2C ESP32 |
| GND | Common GND | Ground bersama |
| SDA | ESP32 GPIO21 | I2C data |
| SCL | ESP32 GPIO22 | I2C clock |
| DRDY/INT | Tidak dipasang | Belum digunakan firmware |

Driver mengharapkan accelerometer pada alamat `0x18` atau `0x19` dan
magnetometer pada `0x1E`. Pesan berikut berarti sensor atau wiring I2C belum
terdeteksi:

```text
LSM303 accelerometer not found at 0x18/0x19
```

Pasang GY-511 sejauh mungkin dari motor, BTS7960, kabel arus besar, magnet,
dan rangka besi. Medan magnet motor dapat mengganggu heading kompas.

Firmware bench test motor saat ini tidak membutuhkan GY-511. Sensor ini dapat
dilepas selama validasi awal motor dan encoder. Pada mapping diagnostic
GPIO22/23 saat ini, GY-511 **wajib dilepas** karena GPIO22 digunakan sebagai
RPWM motor kanan.

## 7. Distribusi Daya yang Benar

### Tahap bench test yang disarankan

```text
USB PC
  `-> konektor USB ESP32
        +-> ESP32 dan expansion board
        `-> VCC logic dua BTS7960, jika rail 5 V expansion sudah diverifikasi

Baterai +12 V
  `-> fuse utama
       `-> emergency switch
            +-> B+ BTS kiri
            `-> B+ BTS kanan

Baterai negatif
    +-> B- BTS kiri
    +-> B- BTS kanan
    `-> satu titik common GND logic
         +-> GND ESP32
         +-> GND BTS kiri dan kanan
         +-> GND kedua encoder
         `-> GND GY-511
```

Gunakan sambungan ground berbentuk star bila memungkinkan. Jauhkan kabel motor
dan baterai dari kabel encoder/I2C. Pilih fuse dan ukuran kabel berdasarkan arus
stall motor yang sebenarnya, bukan hanya rating pemasaran `43 A` BTS7960.

Untuk operasi tanpa USB, gunakan buck converter terpisah dari 12 V menjadi 5 V
yang stabil untuk ESP32 dan logic. Jangan memasukkan 12 V langsung ke expansion
board kecuali manual board secara eksplisit menyatakan input tersebut aman.
Jangan memberi daya dari USB dan supply 5 V eksternal secara bersamaan sebelum
jalur backfeed pada expansion board dipastikan aman.

## 8. Expansion Board dan Pencegahan Brownout

Jika ESP32 normal tanpa expansion tetapi menampilkan:

```text
E BOD: Brownout detector was triggered
```

setelah dipasang ke expansion, hentikan pengujian motor. Brownout berarti
tegangan ESP32 turun atau ada wiring/rail yang salah. Jangan menonaktifkan
brownout detector.

Lakukan isolasi berikut:

1. Cabut USB dan baterai 12 V.
2. Lepaskan seluruh periferal dari expansion.
3. Periksa ESP32 tidak terbalik atau bergeser satu pin.
4. Periksa pin bengkok dan jumper rail `3V3/5V` pada expansion.
5. Pasang ESP32 pada expansion kosong dan beri daya hanya dari USB ESP32.
6. Ukur `5V-GND` sekitar 5 V dan `3V3-GND` sekitar 3,3 V.
7. Tambahkan periferal satu per satu: GY-511, encoder kiri, encoder kanan,
   logic BTS kiri, kemudian logic BTS kanan.
8. Sambungkan baterai motor paling akhir setelah sistem logic stabil.

Jika expansion kosong tetap menyebabkan brownout, jangan lanjut. Kemungkinan
ada ketidakcocokan footprint, posisi board salah, short, jumper supply salah,
atau regulator expansion rusak.

## 9. Urutan Perakitan Aman

- [ ] Semua sumber daya dilepas.
- [ ] ESP32 terpasang dengan orientasi dan posisi pin yang benar.
- [ ] Tidak ada continuity/short langsung antara 3V3-GND dan 5V-GND.
- [ ] ESP32 boot normal pada expansion kosong menggunakan USB.
- [ ] Common ground dipasang.
- [ ] GY-511 diuji sendiri pada 3V3.
- [ ] Tegangan HIGH encoder dipastikan tidak melebihi 3,3 V pada GPIO.
- [ ] R_EN dan L_EN kedua BTS terhubung ke regulated 5 V, bukan GPIO ESP32.
- [ ] Logic BTS diuji tanpa baterai 12 V.
- [ ] Polaritas B+ dan B- diperiksa ulang.
- [ ] Fuse dan emergency switch terpasang.
- [ ] Kedua roda diangkat.
- [ ] Baterai 12 V disambungkan paling akhir.

## 10. Flash dan Bench Test Motor

Tutup serial monitor lama dengan `Ctrl+]`, lalu dari ESP-IDF Terminal:

```powershell
idf.py build
idf.py -p COM3 flash monitor
```

Firmware diagnostic saat ini hanya menginisialisasi satu driver: BTS7960 motor
kanan pada GPIO22/23. Motor kiri, GPIO25/26, service, PID, dan IMU tidak
digunakan. Motor ditahan STOP selama 5 detik setelah boot, kemudian menjalankan:

| Fase | RPWM GPIO22 | LPWM GPIO23 | Durasi |
|---|---:|---:|---:|
| `rpwm_gpio22_100pct` | 255 (100%) | 0 | 2 s |
| STOP | 0 | 0 | 3 s |
| `lpwm_gpio23_100pct` | 0 | 255 (100%) | 2 s |
| STOP | 0 | 0 | 3 s |

Urutan otomatis mengulang dan nomor `CYCLE` bertambah. Ini sengaja dibuat
setara dengan fungsi `setMotorSpeed()` pada sketch Arduino, tanpa dual motor
dan tanpa lapisan service, untuk mengisolasi driver dan power stage dahulu.

PWM menggunakan 1 kHz dan resolusi 8-bit. Input `+1.0` atau `-1.0` menghasilkan
duty maksimum `255/255` pada salah satu input BTS7960, sedangkan input arah
lain tetap `0`. Jika supply `B+`--`B-` adalah 12 V, terminal motor pada duty
100% seharusnya mendekati 12 V dengan polaritas sesuai arah, dikurangi voltage
drop driver dan kabel. Input normalized ini mengatur duty RPWM/LPWM; pin VCC
BTS hanya memasok rangkaian logika dan bukan input pengatur kecepatan.

> PERINGATAN: roda wajib terangkat. Jangan menahan poros/roda dan jangan
> menjalankan tes bila mekanik macet. Pada duty 100%, stall current dapat
> merusak motor, BTS7960, kabel, konektor, atau baterai. Siapkan pemutus daya
> dan cabut baterai untuk menghentikan loop setelah bukti diperoleh.

## 11. Kriteria Hasil yang Benar

| Fase | Gerakan fisik | Feedback yang diharapkan |
|---|---|---|
| RPWM GPIO22 100% | motor kanan berputar arah pertama | duty GPIO22=255 dan ticks berubah |
| LPWM GPIO23 100% | motor kanan berputar arah sebaliknya | duty GPIO23=255 dan ticks berubah berlawanan |

Contoh log:

```text
SINGLE-MOTOR TEST CYCLE=1 START
PHASE=rpwm_gpio22_100pct command=1.00 duration_ms=2000
PWM applied: command=1.00 rpwm_gpio=22 duty=255 lpwm_gpio=23 duty=0 max=255
PHASE=rpwm_gpio22_100pct DONE; motor STOP
PHASE=lpwm_gpio23_100pct command=-1.00 duration_ms=2000
PWM applied: command=-1.00 rpwm_gpio=22 duty=0 lpwm_gpio=23 duty=255 max=255
```

Untuk diagnosis ini `invert_motor=false`, agar command dan pin sama persis
dengan sketch Arduino. Arah fisik roda belum dipakai sebagai keputusan; yang
divalidasi lebih dahulu adalah kedua output arah dan kemampuan motor bergerak.

Log firmware sebelumnya membaca `ledc_get_duty()` langsung setelah
`ledc_update_duty()`. ESP-IDF menjelaskan bahwa nilai baru belum tentu terlihat
pada siklus PWM yang sama, sehingga log dapat mencetak `0` walaupun update baru
dijadwalkan. Driver tetap memakai API dasar `ledc_set_duty()` dan
`ledc_update_duty()` agar tidak membutuhkan fade service, lalu menunggu dua
periode PWM sebelum membaca duty untuk log diagnostik.

Jika log `PWM applied` menunjukkan duty `255` tetapi motor tetap diam, ukur
saat fase aktif: GPIO PWM aktif sekitar 3,3 V, GPIO arah lain 0 V, R_EN/L_EN/VCC
sekitar 5 V, B+--B- sekitar 12 V, dan M+--M- mendekati 12 V. GPIO benar tetapi
M+--M- tetap 0 V menunjuk ke enable, supply, wiring, atau modul BTS; M+--M-
benar tetapi motor diam menunjuk ke kabel motor/konektor/mekanik.

Jika arah fisik roda salah, koreksi `invert_motor` untuk sisi tersebut. Jika
arah fisik sudah benar tetapi tanda ticks/RPM salah, koreksi `invert_encoder`.
Jangan menukar banyak kabel dan flag sekaligus karena hasil diagnosis menjadi
tidak jelas.

## 12. Diagnosis Cepat

| Gejala | Kemungkinan utama | Pemeriksaan |
|---|---|---|
| Brownout sebelum `app_main()` | Supply drop, short, expansion salah posisi | Uji expansion kosong dari USB |
| Motor tidak bergerak | B+/B-, VCC logic, EN, PWM, atau common GND salah | Ukur 12 V di B+/B- dan cek tabel pin |
| Motor langsung bergerak saat boot | Wiring PWM/EN salah | Putus baterai dan cek RPWM/LPWM |
| Motor bergerak tetapi ticks nol | Supply/level/A/B encoder salah | Ukur A/B sambil roda diputar |
| Ticks berubah tetapi tandanya salah | Phase A/B atau inversi encoder | Koreksi `invert_encoder` |
| Kiri/kanan berlawanan saat maju | Mounting mirror belum dikompensasi | Validasi `invert_motor` kanan |
| ESP32 reset saat motor mulai | Noise atau drop supply logic | Pisahkan regulator logic, perbaiki ground dan kabel |
| IMU tidak ditemukan | SDA/SCL tertukar, VCC/GND salah, varian chip berbeda | Cek GPIO21/22 dan alamat I2C |
| Flash gagal `Access is denied` | COM3 masih dipakai monitor | Tutup monitor dengan `Ctrl+]` |

## 13. Checklist Bukti Approval

Catat hasil aktual agar wiring dan konfigurasi bisa disetujui sebelum tuning
PID dan menjalankan mobil di lantai:

```text
Tanggal:
Model expansion board:
Tegangan 5V-GND:
Tegangan 3V3-GND:
Supply motor:
Tegangan HIGH encoder A/B:
Ticks satu putaran roda kiri:
Ticks satu putaran roda kanan:
RPM no-load kiri/kanan:
Duty minimum motor kiri mulai bergerak:
Duty minimum motor kanan mulai bergerak:
Log duty GPIO kiri maju/mundur:
Log duty GPIO kanan maju/mundur:
Arah left_forward_100pct:
Arah left_reverse_100pct:
Arah right_forward_100pct:
Arah right_reverse_100pct:
Tegangan M+--M- kiri saat aktif:
Tegangan M+--M- kanan saat aktif:
Ada/tidak brownout:
Temperatur motor/BTS setelah test:
Keputusan invert_motor kiri/kanan:
Keputusan invert_encoder kiri/kanan:
Link log/foto/video bukti:
```

Jangan melanjutkan ke tuning PID atau kecepatan lintasan 0,8/1,3/1,8 m/s
sebelum safe stop, arah dua roda, ticks per revolution, RPM, dan kestabilan
power dinyatakan lulus.
