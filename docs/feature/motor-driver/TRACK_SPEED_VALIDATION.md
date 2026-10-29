# Validasi Kecepatan Motor per Lintasan

Dokumen ini mengunci pembagian tanggung jawab dan urutan validasi untuk target
kecepatan mobile node:

| Profil | Target kendaraan | Target RPM roda dengan diameter 65 mm |
|---|---:|---:|
| V1 (slow) | 0,8 m/s | sekitar 235 RPM |
| V2 (medium) | 1,3 m/s | sekitar 382 RPM |
| V3 (fast) | 1,8 m/s | sekitar 529 RPM |

Rumus yang digunakan:

```text
wheel_rpm = speed_mps * 60 / (pi * wheel_diameter_m)
```

## Batas Hardware Saat Ini

Motor 25GA370 12 V 100 RPM dengan roda berdiameter 65 mm memiliki kecepatan
linear teoritis tanpa beban sekitar:

```text
100 RPM * pi * 0,065 m / 60 = 0,34 m/s
```

Kecepatan nyata di lantai biasanya lebih rendah karena beban, rugi mekanik,
tegangan baterai, dan slip. Karena itu V1, V2, dan V3 belum dapat dicapai dengan
kombinasi 100 RPM dan roda 65 mm. PID tidak dapat membuat motor melampaui batas
mekanis; output hanya akan saturasi pada duty maksimum.

Sebelum mengaktifkan ketiga profil, pilih salah satu perubahan hardware:

- motor/gearbox dengan output roda minimal sekitar 235, 382, dan 529 RPM;
- diameter roda atau rasio transmisi yang sesuai;
- revisi target kecepatan agar berada di bawah hasil ukur maksimum kendaraan.

## Kontrak Software yang Dipertahankan

```text
Main/Logic memberikan target m/s
    -> MotorControlService mengubah feedback RPM menjadi m/s dan menjalankan PID
        -> MotorEncoderDriver menerima output ternormalisasi -1,0 sampai +1,0
            -> BTS7960 menghasilkan PWM dan arah motor
```

Driver tidak menyimpan profil V1/V2/V3. Dengan kontrak ini, penggantian motor,
encoder, atau H-bridge hanya memerlukan kalibrasi konfigurasi/driver dan tidak
mengubah bahasa command pada logic.

## Urutan Uji dan Bukti Approval

Uji pertama wajib dilakukan dengan roda terangkat dan emergency power-off siap.
Jalankan hanya satu profil dalam satu lintasan/percobaan.

1. Verifikasi safe stop dua detik saat boot.
2. Jalankan target rendah yang sekarang tersedia di firmware bench (30 RPM).
3. Pastikan forward menghasilkan ticks dan RPM positif; reverse menghasilkan
   ticks dan RPM negatif.
4. Kalibrasikan `encoder_counts_per_output_rev` dengan satu putaran roda.
5. Ukur RPM maksimum pada 12 V, mula-mula roda terangkat lalu dengan beban.
6. Hitung kecepatan maksimum aktual dari RPM dan diameter roda.
7. Tuning PID hanya setelah tanda encoder dan kalibrasi benar.
8. Setelah hardware mampu, uji V1, V2, lalu V3 pada lintasan terpisah.

Simpan bukti setiap percobaan berikut:

| Data approval | Nilai yang dicatat |
|---|---|
| Profil | V1 / V2 / V3 |
| Target | m/s dan RPM roda |
| Feedback | RPM kiri/kanan dan m/s kiri/kanan |
| Encoder | ticks awal/akhir dan counts per revolution |
| Error steady-state | target dikurangi feedback |
| Kondisi | tegangan baterai, roda terangkat/di lantai, beban |
| Safety | STOP berhasil, temperatur motor dan BTS7960 aman |

Log firmware bench sudah menampilkan `left_ticks`, `right_ticks`, `left_rpm`,
`right_rpm`, `left_mps`, dan `right_mps`. Nilai nol masih normal jika encoder
belum bergerak; setelah hardware berjalan, nilai tersebut menjadi bukti utama
sebelum integrasi ke mission produk.
