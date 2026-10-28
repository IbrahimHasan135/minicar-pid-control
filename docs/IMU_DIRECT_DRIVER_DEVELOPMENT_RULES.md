# Aturan Development IMU Langsung dari `main.c`

## 1. Tujuan

Dokumen ini mengatur fase development dan pengujian IMU secara mandiri.

Selama fase ini, entry point pengujian mengakses `IMUDriver` secara langsung tanpa melalui `HeadingService`. Tujuannya adalah memvalidasi komunikasi hardware, hasil pembacaan sensor, dan error handling driver sebelum driver dipakai kembali oleh service.

Pola sementara:

```text
main.c (IMU bench) -> IMUDriver -> IMU hardware
```

Pola arsitektur final tetap:

```text
Logic/Task -> HeadingService -> IMUDriver -> IMU hardware
```

Akses langsung dari `main.c` adalah pengecualian khusus bench development, bukan arsitektur produksi.

## 2. Batas Scope

Pada fase ini fokus perubahan hanya pada:

- `main/main.c` sebagai entry point bench IMU;
- `main/driver/imu/IMUDriver.hpp`;
- `main/driver/imu/IMUDriver.cpp`;
- konfigurasi hardware IMU yang memang diperlukan;
- `main/CMakeLists.txt` hanya sejauh diperlukan untuk memilih entry point bench dan dependency IMU.

Jangan mengubah behavior `HeadingService`, PID, motor, odometry, mission, telemetry, atau communication service untuk membuat bench IMU bekerja.

Jangan melakukan merge, push, rebase, atau perubahan struktur branch sebagai bagian dari development ini.

## 3. Aturan Entry Point

`main.c` harus tetap menjadi bench yang kecil dan mudah dibuang setelah validasi selesai. Tanggung jawabnya hanya:

1. membuat satu instance `IMUDriver`;
2. memanggil `init()` satu kali;
3. menghentikan pengujian dengan pesan error yang jelas jika init gagal;
4. memeriksa `isHealthy()`;
5. membaca `getYawRateDps()` secara periodik;
6. menampilkan hasil untuk validasi hardware;
7. memberi delay pada loop agar tidak melakukan busy-wait.

`main.c` tidak boleh:

- menghitung heading;
- menjalankan PID;
- membuat motion state machine;
- mengontrol motor;
- menambahkan API hardware IMU di luar driver;
- menduplikasi implementasi I2C/SPI milik driver;
- menjadi implementasi permanen aplikasi.

## 4. Catatan Bahasa dan Build

`IMUDriver` adalah class C++ di dalam namespace `minicar::driver`, sedangkan ekstensi `.c` secara default dikompilasi sebagai C. Karena itu `main.c` tidak dapat langsung membuat `IMUDriver` jika dikompilasi sebagai C biasa.

Untuk bench ini, saat implementasi dimulai, `main/CMakeLists.txt` harus mengompilasi `main.c` sebagai C++ dan memilihnya sebagai source entry point sementara. Jangan mengaktifkan `main.c` dan `app_main.cpp` bersamaan karena keduanya dapat mendefinisikan `app_main()`.

Walaupun nama file tetap `main.c`, isi bench harus mengikuti aturan C++ yang diperlukan untuk memakai `IMUDriver` secara langsung.

Perubahan build tersebut bersifat sementara dan harus mudah dikembalikan ke `app_main.cpp` setelah bench selesai.

## 5. Kontrak Driver yang Wajib Dipertahankan

Bench boleh mengakses driver secara langsung, tetapi driver wajib tetap kompatibel dengan kontrak `HeadingService` yang sudah ada:

```cpp
esp_err_t init();
float getYawRateDps() const;
bool isHealthy() const;
```

Tidak boleh mengganti nama, tipe return, parameter, namespace, atau makna ketiga API tersebut hanya untuk memudahkan bench.

### `init()`

- Mengonfigurasi bus dan device IMU.
- Mengembalikan `ESP_OK` hanya jika IMU siap digunakan.
- Mengembalikan `esp_err_t` yang sesuai jika konfigurasi, komunikasi, atau identifikasi device gagal.
- Setelah gagal, `isHealthy()` wajib tetap `false`.
- Tidak membuat task dan tidak menjalankan loop pembacaan permanen.

### `getYawRateDps()`

- Menghasilkan yaw rate sumbu Z dalam satuan degree per second.
- Nilai positif dan negatif harus konsisten untuk arah putaran yang sama pada setiap pembacaan.
- Tidak menghitung atau menyimpan heading terintegrasi.
- Tidak melakukan delay panjang atau blocking tanpa batas.
- Tidak mengubah satuan menjadi radian per second.

### `isHealthy()`

- Mengembalikan `true` hanya setelah init berhasil dan pembacaan IMU dinilai valid.
- Mengembalikan `false` jika device belum diinisialisasi atau komunikasi/pembacaan mengalami kegagalan yang membuat data tidak dapat dipercaya.
- Harus dapat dipanggil berulang kali oleh `HeadingService` tanpa efek samping yang merusak state driver.

## 6. Kesesuaian dengan `HeadingService`

Driver dianggap sesuai dengan service hanya jika alur berikut tetap bekerja tanpa perlakuan khusus dari service:

```text
HeadingService::init()
    -> IMUDriver::init()
    -> ESP_OK

HeadingService::updateHeading(dt_s)
    -> IMUDriver::isHealthy()
    -> IMUDriver::getYawRateDps()
    -> heading += yaw_rate_dps * dt_s
```

Implikasinya:

- driver hanya menyediakan yaw rate, bukan heading;
- service tetap menjadi pemilik integrasi heading dan heading PID;
- driver tidak boleh include atau memanggil `HeadingService`;
- bench tidak boleh memaksa perubahan service untuk menyesuaikan output yang salah;
- kalibrasi atau konversi data hardware yang diperlukan agar output menjadi `yaw_rate_dps` adalah tanggung jawab driver.

## 7. Aturan Timing dan Logging Bench

- Gunakan interval sampling yang tetap dan tercatat jelas.
- Gunakan mekanisme delay ESP-IDF/FreeRTOS, bukan loop kosong.
- Log minimal memuat status init, status health, dan `yaw_rate_dps`.
- Logging tidak boleh ditempatkan di dalam driver sebagai kebutuhan permanen bench.
- Error komunikasi harus terlihat; jangan menggantinya diam-diam dengan data seolah-olah valid.

## 8. Validasi Minimum

Driver belum dianggap siap sebelum pengujian langsung dari bench membuktikan:

- IMU terdeteksi dan `init()` menghasilkan `ESP_OK`;
- `isHealthy()` menjadi `true` setelah init berhasil;
- pembacaan saat board diam stabil di sekitar nol setelah penanganan bias yang sesuai;
- nilai yaw rate berubah saat board diputar;
- tanda positif/negatif konsisten terhadap arah putaran yang dipilih;
- nilai yang dilaporkan menggunakan degree per second;
- kegagalan koneksi menghasilkan error dan health menjadi `false`;
- loop dapat berjalan berulang tanpa crash, watchdog timeout, atau blocking berlebihan.

Setelah pengujian driver lulus, lakukan satu integration check melalui `HeadingService` untuk memastikan heading berubah konsisten dari yaw rate yang sama. Integration check dibahas dan dikerjakan setelah fase direct-driver bench selesai.

## 9. Definition of Done Fase Bench

Fase development langsung dari `main.c` selesai jika:

1. semua validasi minimum IMU lulus;
2. API `IMUDriver` tetap kompatibel dengan `HeadingService`;
3. tidak ada logic heading/PID yang bocor ke driver;
4. perubahan khusus bench teridentifikasi jelas sebagai sementara;
5. hasil pengujian hardware dicatat;
6. langkah berikutnya untuk integration test disepakati sebelum service atau arsitektur lain diubah.

## 10. Aturan Pegangan

```text
Direct access di main.c hanya untuk membuktikan driver.
IMUDriver menghasilkan yaw rate yang valid.
HeadingService tetap menghitung heading dan koreksi.
Bench sementara tidak mengubah kontrak arsitektur final.
```
