# Rancangan Komunikasi Wi-Fi dan MQTT

Status dokumen: **rancangan awal; belum ada implementasi**  
Branch: `feat/wifi-mqtt-com`  
Target project: ESP-IDF 5.4.4, ESP32 DevKit V1, C++, FreeRTOS

Dokumen ini menetapkan rancangan awal penambahan komunikasi Wi-Fi dan MQTT pada mini car. Perubahan kode, konfigurasi build, dan dependency MQTT belum dilakukan. Nilai kredensial, alamat broker, topik final, dan format payload masih perlu diputuskan sebelum implementasi.

## 1. Temuan pada repository

- Arsitektur yang berlaku adalah `Application → Logic/Task → Service → Driver → Hardware`. Logic memiliki runtime flow dan queue; service menyediakan capability; driver menangani peripheral.
- Jalur perintah yang direncanakan saat ini adalah `SerialTask → CommunicationService → SerialDriver → MotionCommandQueue`. Namun `SerialDriver::init()` masih mengembalikan `ESP_ERR_NOT_SUPPORTED`, dan serial task belum dijalankan dari `app_main()`.
- `CommunicationService` saat ini khusus membaca baris serial dan mengubah `STOP`, `MOVE <distance_m> <speed_mps>`, serta `TURN <angle_deg>` menjadi `MotionCommand`.
- `MotionCommandQueue` berkapasitas 12. `MotionCommandTask` memasukkan perintah ke `MotionControlContext`; context sudah punya `requestStop()` untuk permintaan stop.
- Telemetry saat ini mengalir melalui `TelemetryQueue → TelemetryTask → TelemetryService → ConsoleDriver`. Pesan telemetry berisi timestamp, level, tag, dan teks, dan dibuat periodik dari `MotionLoopTask`.
- File sumber komponen didaftarkan eksplisit di `main/CMakeLists.txt`.

## 2. Sasaran dan batas

Sasaran tahap komunikasi:

1. ESP32 bergabung ke Wi-Fi dalam mode station.
2. ESP32 terhubung dan tersambung kembali ke broker MQTT.
3. Perintah jarak jauh diterjemahkan menjadi model `MotionCommand` yang sudah ada.
4. Telemetry mini car dapat diterbitkan ke topic MQTT.
5. Komunikasi yang terputus tidak menghentikan atau menunda control loop 100 Hz.

Tahap ini tidak mengubah algoritma motion control, PID, odometry, hardware motor/IMU, atau menambah algoritma localization. Serial tetap menjadi transport terpisah dan tidak diganti diam-diam oleh MQTT.

## 3. Pembagian layer yang diusulkan

```text
Application / app_main
    membuat driver, service, queue, dan task; menginjeksikan dependency

Logic / Task
    WifiMqttTask: startup berurutan, status jaringan, retry, dan lifecycle runtime
    MotionCommandTask: tetap menjadi pemilik pemrosesan antrean perintah gerak
    TelemetryTask: tetap menjadi pemilik konsumsi TelemetryQueue

Service
    WifiService: capability koneksi station dan status Wi-Fi
    MqttCommunicationService: sesi/topik MQTT, validasi payload perintah,
                              konversi MotionCommand, dan serialisasi telemetry

Driver
    WifiDriver: init/config/connect/status Wi-Fi ESP-IDF
    MqttDriver: init/connect/subscribe/publish MQTT ESP-IDF

Hardware / ESP-IDF
    esp_wifi, esp_netif, event loop, TCP/IP, ESP-MQTT
```

Driver tidak membuat task aplikasi, tidak mengakses queue motion/telemetry, dan tidak memutuskan arti perintah. Service tidak membuat task atau memiliki queue aplikasi. Semua dependency dibuat di `app_main()` dan diberikan melalui constructor atau callback interface yang tidak mengekspos tipe FreeRTOS ke service.

### Alur startup dan koneksi

```text
app_main
  → buat WifiDriver dan MqttDriver
  → buat WifiService dan MqttCommunicationService
  → buat queue dan WifiMqttTask
  → WifiMqttTask memulai Wi-Fi
  → setelah IP tersedia, WifiMqttTask memulai koneksi broker
  → setelah MQTT tersambung, subscribe topic perintah dan publish status
```

`WifiMqttTask` memiliki state runtime seperti `WIFI_CONNECTING`, `WAITING_FOR_IP`, `MQTT_CONNECTING`, `ONLINE`, dan `RETRY_WAIT`. Penanganan retry bersifat non-blocking dan hanya memengaruhi jalur komunikasi. `MotionLoopTask` tetap berjalan ketika Wi-Fi atau broker tidak tersedia.

### Alur perintah masuk

```text
Broker MQTT
  → MqttDriver menerima event/payload
  → MqttCommunicationService memvalidasi JSON dan menghasilkan MotionCommand
  → Logic memasukkan perintah ke MotionCommandQueue
  → MotionCommandTask menyerahkan perintah ke MotionControlContext
  → MotionLoopTask menjalankan command melalui service yang ada
```

MQTT event callback tidak boleh melakukan parsing berat, menunggu ruang queue, menjalankan motor, atau mengakses `MotionControlContext` secara langsung. Penyerahan perintah ke logic menggunakan antarmuka callback/queue sink yang disediakan saat dependency injection dan pengiriman non-blocking. Perilaku ketika queue penuh harus dilaporkan sebagai command ditolak, bukan membuat task jaringan menunggu.

**STOP:** jalur STOP harus tetap dapat diterima saat antrean command penuh. Sebelum implementasi, tetapkan jalur prioritas yang memakai mekanisme stop yang sudah tersedia (`MotionControlContext::requestStop()` melalui pemilik logic) atau mekanisme prioritas setara. Jangan menempatkan STOP di belakang rangkaian MOVE/TURN biasa.

### Alur telemetry keluar

```text
MotionLoopTask
  → TelemetryQueue (kirim tanpa menunggu)
  → TelemetryTask
      → TelemetryService → ConsoleDriver (jalur saat ini)
      → MqttCommunicationService → MqttDriver → broker (jalur tambahan)
```

`TelemetryTask` dapat memanggil kedua service untuk satu item queue. Publikasi MQTT menggunakan QoS yang dipilih untuk telemetry dan tidak boleh menahan control loop. Saat broker terputus, telemetry boleh dilewati atau dibatasi dalam buffer berukuran tetap; jangan menambah antrean tak terbatas atau menahan semua gerak.

## 4. Kontrak MQTT awal untuk ditinjau

Nama berikut usulan awal, bukan konfigurasi final:

| Fungsi | Topic usulan | QoS | Retain |
|---|---|---:|---|
| Perintah ke mobil | `minicar/{device_id}/command` | 1 | Tidak |
| Telemetry dari mobil | `minicar/{device_id}/telemetry` | 0 | Tidak |
| Status koneksi/perangkat | `minicar/{device_id}/status` | 1 | Ya; pesan LWT untuk offline |

Perintah sebaiknya memakai JSON berversi dan `command_id` agar pengirim dapat mengaitkan hasil/penolakan serta mendeteksi pengiriman ulang QoS 1. Contoh rancangan:

```json
{"version":1,"command_id":"cmd-001","type":"MOVE","distance_m":1.0,"speed_mps":0.2}
{"version":1,"command_id":"cmd-002","type":"TURN","angle_deg":90}
{"version":1,"command_id":"cmd-003","type":"STOP"}
```

Field MOVE/TURN dipetakan ke nilai dan tipe `MotionCommand` yang telah ada. Validasi harus menolak JSON rusak, versi/tipe tidak dikenal, nilai non-finite, jarak nol, speed tidak positif atau melebihi `MAX_LINEAR_SPEED_MPS`, dan sudut di luar batas yang diterima motion logic. Perintah yang ditolak tidak boleh masuk queue. Rincian batas validasi perlu disamakan dengan validator `MotionLoopTask` agar aturan tidak berbeda.

Payload telemetry tahap pertama dapat membungkus field `TelemetryMessage` yang sudah ada (`timestamp_ms`, `level`, `tag`, `message`). Penambahan posisi `Pose2D` atau state lain dilakukan hanya jika ada kebutuhan konsumen yang jelas dan disepakati.

## 5. Wi-Fi, broker, dan keamanan

- Kredensial Wi-Fi, username/password broker, URI, port, dan device ID harus berasal dari konfigurasi terpisah, bukan literal di source. Pilih mekanisme `menuconfig`/`Kconfig`, provisioning, atau konfigurasi lokal sebelum coding.
- Gunakan MQTT melalui TLS (`mqtts://`) untuk broker di jaringan yang tidak sepenuhnya dipercaya; verifikasi CA sertifikat broker. Jangan menonaktifkan verifikasi sertifikat sebagai jalan pintas.
- Jangan cetak password, token, atau payload sensitif ke log.
- Batasi ukuran payload dan ukuran topic yang diterima. Tangani payload MQTT terfragmentasi sebelum parsing JSON; event MQTT dapat menyampaikan potongan data untuk pesan panjang.
- Status `online` baru dipublikasikan setelah MQTT benar-benar terhubung dan subscription berhasil. Atur Last Will untuk menandai `offline` bila koneksi terputus tak terduga.
- MQTT QoS 1 dapat menghasilkan pesan duplikat. Terapkan deduplikasi berbasis `command_id` atau dokumentasikan semantik idempotent sebelum perintah berulang diizinkan.

## 6. Dampak integrasi yang diharapkan

Saat rancangan disetujui dan implementasi dimulai, perubahan kemungkinan diperlukan pada:

- `main/driver/communication/` untuk driver Wi-Fi dan MQTT;
- `main/service/communication/` untuk service komunikasi MQTT dan validasi/serialisasi protocol;
- `main/logic/` untuk task lifecycle dan event jaringan;
- `main/config/` untuk konfigurasi Wi-Fi, broker, topik, dan ukuran buffer;
- `main/app_main.cpp` untuk dependency injection dan startup;
- `main/CMakeLists.txt` dan konfigurasi component untuk dependency Wi-Fi/MQTT;
- README/dokumen arsitektur dan tes parser/protokol.

File/file class final dapat berubah setelah rancangan ditinjau. Perubahan tersebut belum dibuat pada branch ini.

## 7. Pertanyaan yang harus diputuskan sebelum coding

1. Broker tujuan, URI/port, TLS/CA, username/password, dan apakah broker dapat diakses langsung oleh ESP32.
2. SSID/password Wi-Fi, cara provisioning, dan apakah koneksi hanya 2.4 GHz.
3. Device ID dan topic final, termasuk kebutuhan beberapa mobil.
4. Apakah format command JSON di atas diterima, serta apakah perlu response/ack topic untuk diterima, ditolak, selesai, dan gagal.
5. Kebijakan perintah saat motion sedang sibuk: tolak, antre, atau ganti command aktif; STOP tetap jalur prioritas.
6. Field telemetry yang dibutuhkan, interval publikasi, dan apakah ConsoleDriver tetap menerima setiap telemetry.
7. Batas payload, buffering saat offline, dan kebijakan drop/backoff.

## 8. Urutan implementasi setelah rancangan disetujui

1. Putuskan broker, credentials/provisioning, topics, payload schema, dan command-busy policy.
2. Tambahkan config model serta kontrak API driver Wi-Fi/MQTT; tetapkan error/status contract.
3. Implementasikan driver lalu service dengan dependency injection; driver hanya mengakses API ESP-IDF.
4. Implementasikan task state machine Wi-Fi/IP/MQTT/reconnect tanpa mengubah timing motion loop.
5. Integrasikan command sink ke antrean motion dan jalur STOP prioritas.
6. Tambahkan publikasi MQTT ke jalur TelemetryTask yang sudah ada dengan perilaku offline terbatas.
7. Tambahkan unit test parser/validator, build ESP-IDF, lalu verifikasi di broker dan perangkat nyata.

## Referensi teknis

- [ESP-IDF v5.4.4 Wi-Fi Driver guide](https://docs.espressif.com/projects/esp-idf/en/v5.4.4/esp32/api-guides/wifi.html) menjelaskan init Wi-Fi station, event, got-IP, dan pemulihan disconnect.
- [ESP-IDF v5.4.4 ESP-MQTT guide](https://docs.espressif.com/projects/esp-idf/en/v5.4.4/esp32/api-reference/protocols/mqtt.html) mendokumentasikan transport TCP/TLS/WS, subscribe/publish, autentikasi, QoS, dan event payload.
- Repository ini menargetkan ESP-IDF 5.4.4; panduan tersebut menyebut component MQTT sebagai `mqtt` pada `REQUIRES`/`PRIV_REQUIRES`. Verifikasi dependency aktual saat konfigurasi build dibuat.
