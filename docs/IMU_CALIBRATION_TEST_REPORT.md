# Laporan Pengujian Awal Kalibrasi IMU GY-511/LSM303DLHC

## 1. Identitas Pengujian

| Item | Nilai |
|---|---|
| Tanggal laporan | 2 November 2026 |
| Branch | `feat/develop-imu` |
| Target | ESP32 DevKit V1 |
| Sensor | GY-511 dengan LSM303DLHC |
| Framework build | ESP-IDF 5.5.2 |
| Jalur I2C | SDA GPIO17, SCL GPIO16 |
| Tegangan sensor | 3,3 V direkomendasikan; pemasangan aktual perlu dikonfirmasi |
| Frekuensi I2C | 100 kHz |
| Alamat accelerometer | `0x19` |
| Alamat magnetometer | `0x1E` |
| Periode output bench | 200 ms |
| Durasi kalibrasi | 30 detik |

## 2. Tujuan

Pengujian ini bertujuan untuk:

1. memastikan komunikasi ESP32 dengan accelerometer dan magnetometer berjalan;
2. memastikan firmware dapat melakukan kalibrasi minimum/maksimum magnetometer;
3. mengamati kestabilan heading saat sensor diam;
4. menyiapkan metode pengukuran error sudut terhadap busur derajat;
5. menentukan apakah IMU sudah siap masuk ke pengujian integrasi kendaraan.

## 3. Konfigurasi Hardware

Wiring yang ditetapkan untuk pengujian lanjutan:

```text
GY-511 SDA  -> ESP32 GPIO17
GY-511 SCL  -> ESP32 GPIO16
GY-511 VCC  -> 3,3 V
GY-511 GND  -> common GND
```

Sensor harus menggunakan logika I2C 3,3 V. Sistem motor yang membutuhkan 5 V
harus memakai rail daya terpisah dengan ground yang tetap disatukan. Motor dan
kabel arus besar perlu dijauhkan dari magnetometer selama pengujian awal.

## 4. Metode Firmware

Saat ESP32 dinyalakan, bench melakukan langkah berikut:

1. menginisialisasi accelerometer dan magnetometer;
2. mengambil sampel medan magnet setiap 50 ms selama 30 detik;
3. merekam nilai minimum dan maksimum setiap sumbu;
4. menghitung offset hard-iron dan skala X/Y;
5. menghitung heading menggunakan `atan2(Y, X)`;
6. menampilkan heading, percepatan, dan medan magnet setiap 200 ms.

Heading yang ditampilkan merupakan arah magnetik absolut pada rentang
`0..360` derajat. Nilai ini tidak dapat dibandingkan langsung dengan angka
busur tanpa menjadikan posisi awal sebagai referensi.

Untuk pengujian relatif digunakan:

```text
sudut_terukur = wrap180(heading_sekarang - heading_awal)
error         = sudut_terukur - sudut_busur
```

## 5. Hasil Build dan Komunikasi

| Pemeriksaan | Hasil |
|---|---|
| Konfigurasi target ESP32 | Lulus |
| Kompilasi firmware | Lulus |
| Pembuatan `minicar.bin` | Lulus |
| Inisialisasi sensor | Lulus berdasarkan keluaran data kontinu |
| Pembacaan accelerometer | Lulus |
| Pembacaan magnetometer | Lulus |
| Pembacaan heading | Lulus secara fungsional |
| Akurasi heading multi-sudut | Belum tervalidasi |

## 6. Data Pengujian Awal Saat Diam

Operator menempatkan penanda orientasi dengan Z di bawah, Y di kiri, dan X di
atas. Pada posisi busur yang disebut 90 derajat diperoleh 16 sampel heading.

Ringkasan data:

| Parameter | Hasil |
|---|---:|
| Jumlah sampel | 16 |
| Rata-rata heading | 76,394 derajat |
| Heading minimum | 75,0 derajat |
| Heading maksimum | 80,8 derajat |
| Rentang puncak-ke-puncak | 5,8 derajat |
| Simpangan baku populasi | 1,492 derajat |
| Rata-rata percepatan X | 9,780 m/s2 |

Contoh keluaran:

```text
heading=75.9 deg | accel x=+9.728 y=-0.314 z=+0.755 | mag x=+1.91 y=+12.36 z=+7.24
heading=75.1 deg | accel x=+9.836 y=-0.324 z=+0.853 | mag x=+2.00 y=+12.64 z=+7.04
heading=80.8 deg | accel x=+9.758 y=-0.373 z=+0.853 | mag x=+2.00 y=+12.64 z=+7.45
```

## 7. Analisis

Nilai percepatan X yang mendekati gravitasi menunjukkan bahwa sumbu X sensor
berada hampir vertikal. Sementara itu, implementasi heading saat ini memakai
bidang magnetometer X/Y dan mengasumsikan kedua sumbu tersebut berada pada
bidang horizontal. Karena asumsi pemasangan belum terpenuhi, selisih antara
angka busur 90 derajat dan heading rata-rata 76,394 derajat belum dapat disebut
sebagai error akurasi sebesar 13,606 derajat.

Selain itu, heading sensor adalah arah terhadap utara magnetik, sedangkan busur
mengukur perubahan sudut dari posisi awal. Sebagai contoh, perubahan dari
345,9 derajat ke 75,9 derajat merupakan putaran relatif 90 derajat setelah
normalisasi melewati batas 360 derajat.

Data awal membuktikan sensor dan jalur pembacaan bekerja, tetapi belum cukup
untuk menentukan akurasi belok kendaraan.

## 8. Prosedur Pengujian Ulang

1. Pasang sensor mendatar sehingga accelerometer diam menunjukkan X dan Y
   mendekati nol serta Z mendekati `+9,8` atau `-9,8 m/s2`.
2. Matikan motor dan jauhkan sensor dari material feromagnetik serta kabel arus
   besar.
3. Reset ESP32 untuk memulai kalibrasi baru.
4. Selama 30 detik, putar sensor atau kendaraan secara mendatar sebanyak dua
   sampai tiga putaran penuh tanpa memiringkannya.
5. Setelah kalibrasi, tempatkan busur pada 0 derajat dan catat rata-rata heading
   sebagai referensi awal.
6. Ambil minimal 10 sampel pada 0, 30, 45, 60, 90, 120, 135, 150, dan 180
   derajat.
7. Ulangi pengukuran searah dan berlawanan arah jarum jam sebanyak tiga kali.
8. Hitung error bertanda, mean absolute error, error maksimum, dan simpangan
   baku setiap titik.
9. Ulangi pengujian dengan subsistem motor diberi daya tetapi motor diam untuk
   mengukur gangguan magnetik dari instalasi kendaraan.

## 9. Status Kesiapan Integrasi

Status saat laporan dibuat: **siap untuk pengujian integrasi terbatas, belum
siap menjalankan seluruh kontrol belok dan motor secara bersamaan**.

Alasan:

- komunikasi dan pembacaan sensor sudah berjalan;
- build firmware bench berhasil;
- mapping SDA GPIO17 dan SCL GPIO16 sudah sesuai rencana integrasi;
- orientasi pemasangan belum sesuai asumsi perhitungan heading X/Y;
- belum tersedia baseline 0 derajat dan data multi-sudut;
- pengaruh motor, driver, arus besar, dan rangka kendaraan terhadap magnetometer
  belum diukur;
- pengujian fail-safe ketika komunikasi IMU terputus belum dicatat.

Integrasi penuh baru disarankan setelah pengujian ulang menghasilkan error yang
dapat diterima oleh kebutuhan kontrol belok dan hasilnya konsisten pada kedua
arah putaran.

## 10. Kesimpulan Sementara

Driver GY-511/LSM303DLHC berhasil melakukan komunikasi I2C, membaca sensor, dan
menghasilkan heading kontinu. Data awal menunjukkan pembacaan diam relatif
stabil, tetapi posisi pemasangan membuat sumbu X sejajar gravitasi sehingga
rumus heading X/Y belum diuji pada orientasi yang benar. Tidak dilakukan
penambahan offset berdasarkan satu titik pengukuran karena tindakan tersebut
tidak membuktikan linearitas atau akurasi pada sudut lain.

Tahap berikutnya adalah mengulang kalibrasi dengan sensor mendatar, mengambil
data relatif multi-sudut, lalu menjalankan integration check melalui
`HeadingService` sebelum kontrol motor dan perintah belok diaktifkan bersama.
