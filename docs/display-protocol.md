# M365 display frame — 0x64 reply layout

Controller → ESP32 display, in response to a `0x64` poll. **14 bytes, fixed.**
Defined by `m365Answer` in `common_files/src/ninebot.h`, sent from
`app_uartcomm.c` in `task_app`'s 0x64 handler.

Current as of stage 5 (2026-09-06).

## Byte layout

| Offset | Field | Type | Value | Changed in stage 5 |
|---:|---|---|---|---|
| 0 | `start1` | `uint8_t` | `0x55` | |
| 1 | `start2` | `uint8_t` | `0xAA` | |
| 2 | `len` | `uint8_t` | `8` | |
| 3 | `addr` | `uint8_t` | `0x21` | |
| 4 | `cmd` | `uint8_t` | `0x64` | |
| 5 | `arg` | `uint8_t` | `0` | |
| 6 | `mode` | `uint8_t` | mode/status bit flags — see below | |
| 7 | `battery` | `uint8_t` | state of charge, 0–100 % | |
| 8 | `light` | `uint8_t` | headlight on/off | |
| **9** | **`esc_temp`** | **`int8_t`** | **ESC/FET temperature, whole °C, signed** | **was `beep` (always 0)** |
| 10 | `speed` | `uint8_t` | km/h, or mph when `mode & 0x40` | |
| **11** | **`power`** | **`int8_t`** | **electrical power in 20 W steps, signed** | **was `faultcode`** |
| 12–13 | `CheckSum[2]` | `uint8_t[2]` | little-endian, see below | |

### `mode` bit flags (offset 6)

| bit | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| value | 1 | 2 | 4 | 8 | 16 | 32 | 64 | 128 |
| meaning | Drive | Slow | Sport | Charge | Off | Lock | MPH | Temp |

Speed mode occupies the low three bits; `app_adc_speed_mode()` masks with `0xF8`
before setting them.

### New field semantics

**`esc_temp` (offset 9)** — `VescToSTM_get_temperature()`, i.e.
`NTC_GetAvTemp_C()`, which already returns whole degrees. Signed, so sub-zero
ambient reads correctly. Clamped to −128…127 only to survive a sensor fault.

```c
int8_t esc_temp;      /* degrees Celsius, direct */
```

**`power` (offset 11)** — electrical power, **20 W per count**, signed, negative
under regen:

```c
watts = power * 20;           /* range -2540 .. +2540 W */
```

20 W steps come from clamping to ±127. Computed as bus voltage × input current,
with the multiplication skipped below 1 V (see stage 5 report for why).

## Checksum

`addCRC((uint8_t*)&m365_to_display, m365_to_display.len + 6)` → `size = 14`.

```c
void addCRC(uint8_t * message, uint8_t size){
    unsigned long cksm = 0;
    for(int i = 2; i < size - 2; i++) cksm += message[i];
    cksm ^= 0xFFFF;
    message[size - 2] = (uint8_t)(cksm & 0xFF);
    message[size - 1] = (uint8_t)((cksm & 0xFF00) >> 8);
}
```

Sums **bytes 2 through 11 inclusive**, ones-complements to 16 bits, stores
little-endian at 12–13. Both repurposed bytes (9 and 11) are inside the summed
range, so they are covered exactly as `beep` and `faultcode` were. **No checksum
change was needed and none was made.**

## ESP32 side

Nothing breaks. The display currently reads `f[6]` mode, `f[7]` battery, `f[8]`
light and `f[10]` speed, and guards with `fullLen >= 14`. All four offsets, the
frame length and the checksum are unchanged, so existing firmware keeps working
as-is.

To pick up the new values, add two signed reads:

```c
int8_t esc_temp = (int8_t)f[9];    /* degrees C */
int8_t power_20w = (int8_t)f[11];  /* x20 for watts, negative = regen */
int16_t watts = (int16_t)power_20w * 20;
```

Both **must** be read as signed. Reading them as `uint8_t` will show negative
temperatures and all regen as large positive numbers.

## What was NOT changed

- Frame length (14) and `len` (8)
- The `0x65` handler — throttle and brake input
- Any g30p dashboard code
- The checksum function or its call
