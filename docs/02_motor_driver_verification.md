# Stage 2: Motor Driver & Encoder Verification

> Date: 2026-03-26 ~ 2026-04-01
> Hardware: Raspberry Pi 5 + Arduino Mega 2560 + TB6612FNG + MG310P20 (Hall encoder)

## Result

- Dual MG310P20 motors driving correctly via TB6612FNG
- Both encoders reading stably with consistent count rates
- Direction calibrated — right motor PWM inverted due to mirrored mounting
- Serial protocol verified: Pi Python script controls motors via USB serial

---

## Wiring

### TB6612FNG → Arduino Mega

| TB6612 | Arduino | TB6612 | Arduino |
|--------|---------|--------|---------|
| PWMA | 6 | PWMB | 9 |
| AIN1 | 4 | BIN1 | 7 |
| AIN2 | 5 | BIN2 | 8 |
| STBY | 10 | VCC | 5V |
| VM | 5V (temp) | GND | GND |

### MG310P20 Connector (pins 1–6, Hall encoder version)

| Pin | Function | Left motor | Right motor |
|-----|----------|------------|-------------|
| 1 | Motor – | TB6612 AO2 | TB6612 BO2 |
| 2 | Encoder VCC | Arduino 5V | Arduino 5V |
| 3 | Encoder A | Arduino **2** | Arduino **18** |
| 4 | Encoder B | Arduino **3** | Arduino **19** |
| 5 | Encoder GND | Arduino GND | Arduino GND |
| 6 | Motor + | TB6612 AO1 | TB6612 BO1 |

> All GND rails must be common: TB6612 GND, encoder GND, and power GND all tied to Arduino GND.

---

## Serial Protocol

```
Arduino → Pi  (every 10 ms):  E,<left_count>,<right_count>\n
Pi → Arduino  (on demand):    M,<left_pwm>,<right_pwm>\n   (range: –255 ~ 255)
```

Verified output (motors stopped):
```
E,882,3641
E,882,3641
```

---

## Firmware

| File | Purpose |
|------|---------|
| `step1_motor_test.ino` | Fixed PWM, verify motor spins |
| `step2_encoder_test.ino` | Single motor + encoder, print count |
| `step3_dual_motor.ino` | Dual motor + dual encoder, verify direction |
| `step4_serial_protocol.ino` | Full serial protocol — final Week 2 firmware |

---

## Issues & Fixes

**1. TB6612 STBY not wired → motors dead**
STBY floats LOW, disabling the chip. Fix: `digitalWrite(STBY, HIGH)` in `setup()`.

**2. B-side control pins not connected → right motor dead**
Forgot BIN1/BIN2/PWMB after verifying left motor. Fix: wire pins 7, 8, 9.

**3. Mirrored motors → robot spins in place**
Same PWM direction is physically opposite for left vs right. Fix: invert right motor PWM in firmware, no rewiring needed.

**4. Serial monitor line ending wrong → M command ignored**
Arduino waits for `\n` to parse commands. Fix: set line ending to **"Newline"** in Arduino IDE serial monitor.

---

## Lessons Learned

1. Always check enable pins first when a motor driver doesn't respond
2. Verify each side of a dual motor driver independently
3. Handle mirrored motor direction in firmware, not hardware
4. Confirm serial tool line-ending settings before debugging protocol issues
