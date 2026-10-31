# Driver Pin Mapping

Satu `MotorEncoderDriver` mengakses satu motor dan satu BTS7960. R_EN/L_EN
tidak dikontrol ESP32; keduanya dipasang langsung ke regulated 5 V.

| Motor | RPWM | LPWM | Encoder A | Encoder B | R_EN/L_EN |
|---|---:|---:|---:|---:|---|
| Kiri / Motor 1 | GPIO23 | GPIO22 | GPIO19 | GPIO18 | 5 V langsung |
| Kanan / Motor 2 | GPIO26 | GPIO25 | GPIO12 | GPIO13 | 5 V langsung |

## IMU (reserved)

IMU belum diimplementasikan, tetapi wiring yang disiapkan adalah:

| Signal | ESP32 |
|---|---:|
| SDA | GPIO17 |
| SCL | GPIO16 |

Mapping PWM mengikuti `docs/Driver/BTS7960_Dual_Motor_Diagnostic.ino`.
Encoder mengikuti `docs/Driver/sketch_oct29b.ino`.

ESP32 dan kedua BTS7960 wajib memiliki common GND. Sinyal encoder harus aman
3.3 V; gunakan level shifter jika encoder menghasilkan 5 V.

LEDC channel driver:

```text
Motor kiri  : channel 0/1
Motor kanan : channel 2/3
```
