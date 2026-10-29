# Analisis BTS7960: Motor Tidak Bergerak

Dokumen ini mencatat perbandingan antara sketch Arduino + potensiometer yang
berhasil menggerakkan satu motor dan firmware ESP-IDF di branch
`feat/motor-driver` yang belum menggerakkan motor.

Analisis ini belum mengubah kode. Tujuannya adalah memisahkan masalah power,
wiring, PWM open-loop, encoder, dan PID sebelum memilih perbaikan.

## Kesimpulan Utama

Ada tiga temuan utama:

1. **Kecepatan motor tidak diatur melalui VCC.** VCC BTS7960 adalah supply
   logic tetap. Pada sketch Arduino, potensiometer hanya menghasilkan nilai
   perintah melalui ADC; nilai itu kemudian diubah menjadi duty PWM pada
   `LPWM/RPWM`.
2. **Pin sketch Arduino berbeda dari pin firmware proyek.** Sketch memakai
   `LPWM=GPIO23` dan `RPWM=GPIO22`, sedangkan firmware proyek memakai pin lain.
   Jika wiring sketch dipakai tanpa mengubah config proyek, PWM dikirim ke
   kabel yang salah.
3. **Duty awal closed-loop proyek sangat kecil.** Target 30 RPM hanya
   menghasilkan sekitar 8,2% duty pada awal fase dan sekitar 10,2% setelah dua
   detik ketika motor masih diam. Target pivot 20 RPM hanya sekitar 5,5-6,8%.
   Nilai tersebut sangat mungkin belum melewati dead-zone/stiction motor,
   gearbox, dan drivetrain.

Dengan demikian, keberhasilan sketch potensiometer belum membuktikan PID saat
ini benar. Sketch tersebut membuktikan bahwa motor dan BTS7960 dapat bergerak
ketika diberi PWM open-loop yang cukup besar.

## 1. Fungsi Setiap Jalur BTS7960

| Jalur | Fungsi | Apakah mengatur kecepatan? |
|---|---|---|
| `B+ / B-` | Supply daya motor, misalnya 12 V | Menentukan tegangan maksimum yang tersedia, bukan command runtime |
| `VCC / GND` | Supply logic modul, umumnya 5 V | Tidak; harus stabil dan tidak dimodulasi |
| `R_EN / L_EN` | Mengaktifkan kedua sisi driver | Tidak; biasanya HIGH=enable dan LOW=disable |
| `RPWM / LPWM` | Input arah dan duty PWM | **Ya, inilah pengatur arah dan daya rata-rata motor** |
| `R_IS / L_IS` | Current sense/diagnostic pada modul yang mendukung | Tidak pada implementasi sekarang |

BTS7960 adalah half-bridge; modul IBT-2/BTS7960 memakai dua half-bridge untuk
membentuk H-bridge. Untuk satu arah, satu input PWM diberi duty dan input arah
lawan dibuat nol. Saat arah dibalik, urutannya dibalik.

## 2. Potensiometer Bukan Supply Kecepatan

Pada sketch Arduino:

```text
3,3 V ---- kaki luar potensiometer
             kaki tengah/wiper ---- GPIO33 ADC
GND  ---- kaki luar potensiometer
```

Potensiometer membuat pembagi tegangan antara 0 dan 3,3 V. GPIO33 hanya
**membaca** tegangan wiper:

```cpp
vrValue = analogRead(VR_PIN);
potValue = map(vrValue, 0, 4095, 0, 255);
```

Nilai ADC kemudian diterjemahkan menjadi duty PWM:

```cpp
ledcWrite(LPWM_PIN, left);
ledcWrite(RPWM_PIN, right);
```

Alur sebenarnya:

```text
posisi potensiometer
    -> tegangan ADC 0..3,3 V
    -> angka ADC 0..4095
    -> command 0..255
    -> duty PWM 0..100%
    -> LPWM atau RPWM BTS7960
    -> tegangan rata-rata motor berubah
    -> kecepatan motor berubah
```

Jadi bukan `VCC BTS7960` yang diubah oleh potensiometer. Supply logic BTS7960
tetap 5 V dan supply motor tetap 12 V. Potensiometer hanya menjadi input
manusia untuk menentukan duty PWM.

Nama `leftSpeed` dan `rightSpeed` pada sketch juga berpotensi membingungkan.
Keduanya bukan kecepatan roda kiri dan roda kanan. Dalam sketch satu motor itu,
variabel tersebut mewakili duty `LPWM` dan `RPWM`, yaitu dua arah untuk **satu
motor**.

## 3. Perbandingan Pengaturan Kecepatan

### Sketch Arduino

```text
potensiometer -> ADC -> mapping -> duty PWM langsung -> BTS7960
```

Karakteristik:

- open-loop;
- tidak membaca encoder;
- tidak mengetahui RPM aktual;
- duty dapat mencapai 100%;
- motor mudah mulai bergerak ketika potensiometer diputar cukup jauh;
- perubahan beban dapat mengubah RPM walaupun posisi potensiometer sama.

### Firmware ESP-IDF proyek

```text
main.c target RPM
    -> MotorControlService::setRpmTargets()
    -> target RPM dikonversi menjadi m/s
    -> PID membandingkan target m/s dengan feedback m/s
    -> output normalized -1,0..+1,0
    -> MotorEncoderDriver::setOutput()
    -> duty LEDC pada LPWM/RPWM
```

Karakteristik:

- closed-loop;
- target dari `main.c`, bukan potensiometer;
- encoder menghasilkan ticks dan RPM aktual;
- PID menentukan duty;
- seharusnya mampu mempertahankan RPM ketika beban berubah, setelah
  feedforward/dead-zone dan gain dituning.

API menerima target RPM, tetapi PID internal saat ini bekerja dalam m/s. Secara
matematis target dan feedback masih memakai unit internal yang sama, sehingga
perbandingannya konsisten. Namun gain `KP/KI/KD` harus dituning untuk skala m/s,
bukan diasumsikan sebagai gain untuk error RPM.

## 4. R_EN dan L_EN: 5 V Langsung vs GPIO

### R_EN/L_EN dihubungkan langsung ke 5 V

Keuntungan:

- sederhana untuk bench test satu motor;
- enable pasti HIGH selama supply logic tersedia;
- menghilangkan satu variabel saat membuktikan power stage;
- tidak memerlukan dua GPIO tambahan per BTS7960.

Konsekuensi:

- H-bridge selalu enabled;
- software tidak bisa men-disable power stage secara terpisah dari PWM;
- stop hanya bergantung pada `RPWM=0` dan `LPWM=0`;
- tidak dapat digunakan sebagai lapisan shutdown software tambahan.

### R_EN/L_EN dikendalikan GPIO

Alasan desain awal proyek memilih GPIO:

- memungkinkan driver tetap disabled saat boot;
- memungkinkan disable saat init gagal;
- memungkinkan shutdown saat fault/emergency software;
- memberi ruang untuk sequencing yang aman sebelum PWM diaktifkan;
- lebih cocok untuk driver reusable dan mode produk.

Namun implementasi sekarang belum lengkap memanfaatkan alasan tersebut.
`initPwm()` membuat R_EN/L_EN HIGH, tetapi `stop()` hanya menulis duty PWM nol
dan **tidak menurunkan R_EN/L_EN**. Setelah init berhasil, perilakunya hampir
sama dengan enable yang diikat ke 5 V, tetapi menggunakan empat GPIO tambahan
untuk dua BTS.

Selain itu, output GPIO ESP32 adalah 3,3 V sedangkan logic modul diberi VCC 5 V.
Banyak modul IBT-2 menerima sinyal 3,3 V dalam praktik, tetapi margin logika
bergantung pada rangkaian/buffer yang benar-benar dipasang pada modul. Hal ini
harus diverifikasi dari skematik modul atau pengukuran, bukan diasumsikan.

### Keputusan yang disarankan

Untuk **pembuktian awal power stage**, R_EN dan L_EN boleh diikat ke regulated
5 V agar sama dengan setup yang sudah terbukti. Dalam mode ini pin enable pada
config sebaiknya dinyatakan tidak dipakai agar dokumentasi dan firmware sama.

Untuk **versi produk**, GPIO enable tetap lebih baik jika:

- level HIGH 3,3 V terbukti diterima atau diberi buffer level yang benar;
- ada pull-down hardware agar default saat boot adalah disabled;
- `stop()`, init gagal, dan fault benar-benar menurunkan enable;
- emergency switch fisik tetap memutus supply motor karena software bukan
  satu-satunya mekanisme keselamatan.

Jangan pernah menyambungkan 5 V dari R_EN/L_EN ke GPIO ESP32 yang sedang
dikonfigurasi sebagai output.

## 5. Mismatch Pin yang Sangat Penting

Sketch Arduino satu motor memakai:

| Sinyal | GPIO sketch |
|---|---:|
| LPWM | 23 |
| RPWM | 22 |
| Potensiometer/ADC | 33 |
| R_EN | 5 V langsung |
| L_EN | 5 V langsung |

Firmware proyek memakai:

| Sisi | RPWM | LPWM | R_EN | L_EN | Encoder A | Encoder B |
|---|---:|---:|---:|---:|---:|---:|
| Kiri | 25 | 26 | 27 | 14 | 32 | 33 |
| Kanan | 18 | 19 | 23 | 13 | 16 | 17 |

GPIO22 pada arsitektur proyek dialokasikan sebagai SCL GY-511. GPIO23 adalah
R_EN motor kanan. GPIO33 adalah Encoder B motor kiri.

Artinya wiring sketch tidak bisa langsung dipakai bersama firmware proyek:

- PWM proyek tidak pernah dikirim ke GPIO22/23 sebagai pasangan motor;
- GPIO23 proyek dibuat HIGH sebagai `R_EN` kanan, bukan PWM motor sketch;
- potensiometer GPIO33 akan bertabrakan dengan Encoder B kiri;
- mencampur kedua mapping dapat menghasilkan motor diam atau perilaku yang
  tidak aman.

Harus dipilih satu mapping yang menjadi sumber kebenaran. Untuk dua motor dan
GY-511, mapping `MotorHardwareConfig.hpp` saat ini tidak memiliki konflik
internal. Wiring fisik wajib dipindahkan mengikuti mapping tersebut, atau
config firmware harus sengaja diubah mengikuti wiring baru.

## 6. Perhitungan Duty PID Saat Ini

Parameter saat ini:

```text
diameter roda = 0,065 m
KP = 0,8
KI = 0,1
KD = 0
periode kontrol = 0,05 s
```

Target 30 RPM dikonversi menjadi:

```text
keliling roda = pi x 0,065 = sekitar 0,2042 m
target speed  = 30 x 0,2042 / 60 = sekitar 0,1021 m/s
```

Ketika motor diam dan feedback 0:

```text
P awal       = 0,8 x 0,1021 = 0,0817
output awal  = sekitar 8,2% duty
output 2 s   = sekitar 10,2% duty, jika motor tetap diam
```

Untuk target pivot 20 RPM:

```text
target speed = sekitar 0,0681 m/s
output awal  = sekitar 5,5% duty
output 2 s   = sekitar 6,8% duty, jika motor tetap diam
```

Integral dibatasi pada `0,5`, sehingga kontribusi maksimum integral hanya:

```text
KI x integral_limit = 0,1 x 0,5 = 0,05 atau 5% duty
```

Motor DC dan gearbox memiliki dead-zone karena gesekan, beban mekanik, voltage
drop, serta karakteristik BTS7960. Duty 5-11% sering belum cukup untuk mulai
bergerak. Sketch potensiometer dapat memberikan duty jauh lebih besar hingga
100%, sehingga motor berhasil bergerak.

Ini adalah kandidat penyebab software paling kuat. Solusi produk biasanya
bukan sekadar menaikkan KP tanpa data, tetapi menggabungkan:

- hasil pengukuran minimum duty yang dapat mulai memutar tiap roda;
- feedforward berdasarkan target RPM;
- kompensasi static friction/minimum effective output;
- PID sebagai koreksi terhadap error;
- limit dan ramp output yang aman.

## 7. Perbedaan PWM 1 kHz dan 20 kHz

| Implementasi | Frekuensi | Resolusi |
|---|---:|---:|
| Sketch Arduino | 1 kHz | 8 bit, 0..255 |
| Firmware proyek | 20 kHz | 10 bit, 0..1023 |

Resolusi tidak menjadi masalah karena kedua skala diubah menjadi persentase
duty. Frekuensi 20 kHz dipilih agar di atas rentang suara, tetapi sketch yang
sudah terbukti memakai 1 kHz.

Frekuensi bukan tersangka pertama karena BTS7960 memang ditujukan untuk PWM,
tetapi saat bring-up sebaiknya samakan dahulu dengan setup 1 kHz yang terbukti.
Setelah power stage, arah, dan minimum duty diketahui, frekuensi dapat dinaikkan
dan efeknya diukur kembali.

## 8. Kekurangan Observability Firmware Sekarang

Log `main.c` menampilkan:

- target RPM;
- ticks;
- RPM aktual;
- m/s aktual.

Log belum menampilkan:

- output normalized PID;
- duty LEDC aktual;
- status R_EN/L_EN;
- apakah arah yang aktif adalah RPWM atau LPWM.

Akibatnya log target 30 RPM tidak membuktikan PWM benar-benar cukup besar atau
sampai pada pin BTS7960. Untuk diagnosis berikutnya, telemetry output/duty perlu
ditambahkan sebelum tuning PID.

## 9. Faktor Hardware yang Masih Harus Diperiksa

### Power dan brownout

Log sebelumnya menunjukkan `Brownout detector was triggered`. Jika log tidak
mencapai `PHASE=...`, motor belum pernah menerima command karena ESP32 reset
sebelum `app_main()` berjalan. Expansion board harus stabil lebih dahulu.

### Supply BTS7960

Ukur langsung:

```text
B+ terhadap B-      = sekitar 12 V
VCC terhadap GND    = sekitar 5 V
R_EN/L_EN enabled   = level HIGH yang valid
common GND          = tersambung
```

Jangan menyimpulkan LED indikator menyala berarti jalur motor dan common ground
sudah benar.

### Encoder

Nilai `1496 count/rev` masih estimasi. Nilai yang salah membuat perhitungan RPM
salah, tetapi tidak menjelaskan output awal yang tidak bergerak ketika feedback
masih nol. Encoder baru dikalibrasi setelah PWM open-loop berhasil.

## 10. Ranking Kemungkinan Penyebab

| Prioritas | Penyebab | Bukti/status |
|---:|---|---|
| 1 | Wiring masih mengikuti GPIO22/23 dari sketch | Mapping berbeda total dari config proyek; wajib diperiksa fisik |
| 2 | Duty PID terlalu kecil untuk melewati dead-zone | Terbukti dari perhitungan: hanya sekitar 5-11% |
| 3 | Expansion/power mengalami brownout | Pernah terlihat jelas pada log hardware |
| 4 | R_EN/L_EN 3,3 V tidak diterima stabil atau salah wiring | Perlu ukur level pada header modul |
| 5 | Firmware/log yang dijalankan bukan build terbaru | Perlu cocokkan hash commit dan fase log |
| 6 | Frekuensi 20 kHz memberi perilaku berbeda dari bench 1 kHz | Perlu A/B test setelah tiga poin pertama lulus |
| 7 | Estimasi encoder/count salah | Memengaruhi closed-loop setelah motor bergerak |

## 11. Urutan Pembuktian yang Disarankan

Jangan langsung tuning PID. Gunakan urutan berikut dengan roda terangkat dan
emergency power-off siap.

### Tahap A - Samakan hardware dengan setup yang sudah terbukti

1. Gunakan satu BTS7960 dan satu motor dahulu.
2. Tentukan mapping pin final; jangan campur GPIO sketch dengan config proyek.
3. Untuk isolasi awal, ikat R_EN dan L_EN ke regulated 5 V seperti setup yang
   sudah berhasil.
4. Gunakan VCC logic tetap 5 V, B+/B- 12 V, dan common ground.
5. Jangan pasang potensiometer pada GPIO33 bersamaan dengan Encoder B kiri.

### Tahap B - Uji driver open-loop tanpa PID

1. Motor STOP selama beberapa detik saat boot.
2. Berikan output langsung bertahap, misalnya 15%, 25%, lalu 35%, dengan durasi
   pendek.
3. Catat duty minimum saat motor mulai bergerak untuk arah maju dan mundur.
4. Ulangi untuk motor kedua.
5. Buktikan `RPWM/LPWM` dengan oscilloscope, logic analyzer, atau multimeter
   yang dapat membaca duty/frequency.

Tahap ini menjawab apakah GPIO, LEDC, BTS7960, supply, dan motor bekerja tanpa
dipengaruhi encoder atau PID.

### Tahap C - Uji encoder terpisah

1. Jalankan motor pada duty open-loop yang sudah terbukti.
2. Pastikan ticks berubah.
3. Pastikan tanda maju positif dan mundur negatif.
4. Ukur jumlah ticks untuk tepat satu putaran output shaft.
5. Perbarui count/rev berdasarkan hasil nyata.

### Tahap D - Aktifkan closed-loop RPM

1. Gunakan target RPM rendah dengan feedforward/minimum effective duty.
2. Log target RPM, measured RPM, error, normalized output, dan duty.
3. Tuning setiap roda secara terpisah.
4. Setelah stabil, uji dua roda maju/mundur dan pivot.

## 12. Keputusan Arsitektur yang Direkomendasikan

Untuk mempertahankan struktur proyek:

- `MotorEncoderDriver` tetap menerima output normalized `-1..+1` dan mengurus
  GPIO/PWM/encoder;
- `MotorControlService` menerima target RPM, membaca RPM aktual, menjalankan
  feedforward + PID, dan menghasilkan output normalized;
- `main.c` hanya mengatur urutan bench test;
- potensiometer, bila dipakai, adalah command source sementara di main/test,
  bukan bagian dari driver dan bukan pengganti VCC;
- mode open-loop harus dipakai sebagai alat bring-up terpisah sebelum mode PID.

Jika tujuan API adalah target dan feedback sama-sama RPM, service dapat
menjalankan PID langsung dalam RPM. Alternatifnya, service boleh tetap memakai
m/s secara internal, tetapi gain harus jelas diberi unit dan dituning untuk
m/s. Jangan mencampur gain yang diasumsikan berbasis RPM dengan error m/s.

## 13. Referensi Teknis

- Infineon BTS7960 datasheet:
  <https://www.infineon.com/assets/row/public/documents/10/57/infineon-bts7960-ds-en.pdf>
- Espressif ESP32 Series datasheet:
  <https://documentation.espressif.com/esp32_datasheet_en.html>
- Wiring sistem proyek:
  `docs/feature/motor-driver/COMPLETE_HARDWARE_WIRING_GUIDE.md`

Datasheet BTS7960 menjelaskan bahwa pin inhibit/enable LOW menempatkan device
pada kondisi sleep/disabled. Datasheet ESP32 menjadi acuan batas listrik GPIO;
input encoder atau sinyal eksternal 5 V tidak boleh langsung diasumsikan aman
untuk GPIO 3,3 V.

## 14. Data yang Dibutuhkan untuk Menentukan Fix Final

Catat hasil berikut pada pengujian berikutnya:

```text
GPIO fisik RPWM/LPWM motor kiri:
GPIO fisik RPWM/LPWM motor kanan:
R_EN/L_EN diikat 5 V atau GPIO:
Tegangan VCC-GND BTS:
Tegangan B+-B- BTS:
Apakah log mencapai PHASE=left_forward_low:
Ticks/RPM saat fase tersebut:
Duty minimum motor kiri mulai bergerak:
Duty minimum motor kanan mulai bergerak:
Frekuensi PWM yang diuji:
Versi/hash firmware pada boot:
Ada/tidak brownout/reset:
```

Setelah data tersebut tersedia, perubahan kode dapat dilakukan dengan dasar
pengukuran, bukan tebakan.
