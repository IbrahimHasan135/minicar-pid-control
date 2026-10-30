# Driver Pin Mapping

Satu `MotorEncoderDriver` mengakses satu motor dan satu BTS7960. R_EN/L_EN
tidak dikontrol ESP32; keduanya dipasang langsung ke regulated 5 V.

| Motor | RPWM | LPWM | Encoder A | Encoder B | R_EN/L_EN |
|---|---:|---:|---:|---:|---|
| Kiri / Motor 1 | GPIO22 | GPIO23 | GPIO16 | GPIO17 | 5 V langsung |
| Kanan / Motor 2 | GPIO18 | GPIO19 | GPIO32 | GPIO33 | 5 V langsung |

Mapping PWM mengikuti `docs/Driver/BTS7960_Dual_Motor_Diagnostic.ino`.
Encoder mengikuti `docs/Driver/sketch_oct29b.ino`.

ESP32 dan kedua BTS7960 wajib memiliki common GND. Sinyal encoder harus aman
3.3 V; gunakan level shifter jika encoder menghasilkan 5 V.

LEDC channel driver:

```text
Motor kiri  : channel 0/1
Motor kanan : channel 2/3
```
