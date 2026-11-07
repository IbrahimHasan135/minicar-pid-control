# Catatan Masalah Kontrol Lurus Berbasis IMU

## Status

Pengujian kontrol gerak dihentikan sementara. Firmware dialihkan ke mode akuisisi
data navigasi statis: motor diam, penerimaan UART tetap aktif, dan data serial
diteruskan ke MQTT.

## Data pengujian terakhir

- Heading awal terkuantisasi pada `-2 derajat`.
- Penyimpangan awal kecil sempat kembali dari `-6 derajat` ke sekitar nol.
- Setelah melewati nol, heading terus bertambah hingga mendekati `+180 derajat`.
- Nilai kemudian berpindah dari `+174/+178` ke `-178/-180`. Ini merupakan
  wrap-around representasi sudut pada batas `-180..+180`, bukan heading yang
  tiba-tiba kembali stabil.
- Pola tersebut berulang beberapa kali. Artinya mobil berputar melewati satu
  putaran, bukan hanya mengalami penyimpangan kecil saat maju lurus.
- Besar perubahan per sampel juga meningkat. Koreksi bertingkat pada satu roda
  belum mampu meredam laju putar dan pada kondisi tertentu justru
  mempertahankan putaran.

## Masalah yang tercatat

1. Kontrol hanya memodifikasi target kecepatan roda kanan, sedangkan roda kiri
   tetap menjadi master.
2. Keputusan koreksi hanya menggunakan error heading saat ini. Laju perubahan
   heading (yaw rate) belum dipakai untuk mengetahui bahwa kendaraan sedang
   berputar cepat dan perlu direm sebelum mencapai target.
3. Respons PID kecepatan roda, filter encoder, slew-rate output, dan mekanika
   kendaraan menimbulkan keterlambatan antara perubahan target dan perubahan
   arah kendaraan.
4. Pergantian koreksi pada ambang sudut tidak menjamin putaran berhenti. Saat
   error masuk zona aman, kendaraan masih dapat memiliki momentum sudut besar.
5. Data terakhir belum cukup untuk memisahkan pengaruh target kecepatan,
   kecepatan aktual setiap roda, output PWM, dan yaw rate. Semua variabel itu
   perlu direkam bersamaan pada pengujian kontrol berikutnya.

## Keputusan sementara

- Jangan menjalankan mission maju atau belok.
- Jangan memulai task kontrol gerak.
- Paksa motor berhenti setelah inisialisasi.
- Fokus mengambil data serial RGBC/RF secara statis melalui MQTT.
- Pembahasan kontrol motor dilanjutkan setelah pengambilan data statis selesai.

## Jalur data statis yang aktif

- UART minicar: `UART2`, RX `GPIO32`, baud rate `115200`.
- Format input: JSON dengan `value1`, `value2`, `value3`, dan `value4`.
- Topic MQTT bersama: `serial-communication/rgbc-rf`.
- Periode publish: 5 detik.
- Sesi saat ini: 5 sampel data, kemudian satu pesan penanda selesai.

