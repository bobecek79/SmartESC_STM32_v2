# Changes against upstream

Consolidated from 15 development stages. Fork of
[Koxx3/SmartESC_STM32_v2](https://github.com/Koxx3/SmartESC_STM32_v2) `vesc_comp`,
retargeted for a hardware-modified Xiaomi M365 controller.

**Target:** m365 only. STM32F103C8Tx, 64 MHz, 128 K flash / 20 K RAM, PWM 16 kHz,
three-shunt low-side sensing, 15 pole pairs.

| | upstream baseline | this build | delta |
|---|---:|---:|---:|
| Flash | 82 016 B (62.6 %) | **83 136 B (63.4 %)** | +1 120 B |
| RAM | 20 416 B (99.7 %) | **17 432 B (85.1 %)** | −2 984 B |
| RAM free | 64 B | **3 048 B** | +2 984 B |

`mc_configuration` layout and `MCCONF_SIGNATURE` (2525666056) are unchanged
throughout, so stored VESC Tool detection results in flash page 127 stay valid.

---

## 1. Board constants

The hardware is modified; upstream's constants describe a stock board. These are
compiled in.

| Constant | File | Upstream | Here |
|---|---|---|---|
| `RSHUNT` | `product.h` | 0.00200 | **0.00100** — shunts doubled in parallel |
| `VOLTAGE_DIVIDER_GAIN` | `product.h` | 2808.6359 | **1537.0** — 1M/33k divider, meter-calibrated |
| `NOMINAL_BUS_VOLTAGE_V` | `power_stage_parameters.h` | 36 | **54** — 15S |
| `VBUS_PARTITIONING_FACTOR` | `power_stage_parameters.h` | 0.0535 | **0.0293** |
| `HW_LIM_VIN` | `product.h` | 6.0, 56.0 | **6.0, 75.0** |
| `MCCONF_L_MAX_VOLTAGE` | `mcconf_default.h` | 56.0 | **70.0** |
| `MCCONF_L_BATTERY_CUT_START/END` | `mcconf_default.h` | 34.0 / 30.0 | **45.0 / 40.0** |

`CURRENT_FACTOR_A` moves from 317.74 to **158.87 counts/A**. Current readings are
no longer halved and current limits are no longer doubled. A side effect worth
knowing: `HW_LIM_CURRENT_ABS` (100 A) previously sat at 97 % of ADC full scale and
was effectively unreachable; it now sits at 48 % and is a trip that can fire.

**A real overflow was fixed by the divider-gain change.** The bus reading is a
`uint16_t`. At the old gain, 63 V produced 73 007 counts — past 65 535 — wrapping
to an apparent 6.45 V, which is below the 8 V undervoltage threshold. A fully
charged 15S pack could not be represented at all, and would fault at full charge.

`HW_LIM_VIN`'s 56 V ceiling was the *stock* divider's measurement limit (99 % of
56.55 V full scale). With the 1M/33k divider the ADC ceiling is 105.19 V, so the
ceiling was raised to 75 V — 19 % over the 63 V pack, 25 V below the IRFB4110 and
bulk-cap 100 V rating, and 29 % below the ADC ceiling. 75 rather than 80 because
the bridge hard-switches and 15–25 V of switch-node overshoot is ordinary; at
80 V that lands past Vds(max).

`NOMINAL_CURRENT` and `ID_DEMAG` are deliberately left alone — both are overwritten
from `mc_conf` at runtime on every boot, including the CRC-failure path.

`OVERVOLTAGE_THRESHOLD_d` / `UNDERVOLTAGE_THRESHOLD_d` were found to be **dead
code** — each appears exactly once, at its own definition. The VESC layer replaced
the SDK's compile-time bus thresholds with runtime ones.

## 2. ADC sampling

The battery divider is 1M‖33k = **31.95 kΩ**. Upstream sampled it for 1.5 ADC
cycles, which budgets about 0.8 kΩ of source impedance — roughly 40× short.

`M1_VBUS_SAMPLING_TIME` and `M1_TEMP_SAMPLING_TIME` raised to **55.5 cycles**,
costing +1.31 µs on a task-rate conversion. 41.5 was rejected: it clears 31.95 kΩ
by under 10 % on a conservative settling model, too thin once two 1 % resistors
and leakage are allowed for.

**Injected ADC timing is unchanged** — `ADC_SAMPLETIME_1CYCLE_5`,
`ADC_SAMPLING_CYCLES = 1 + SAMPLING_CYCLE_CORRECTION`. The injected group is the
three current shunts, timed to the PWM zero vector; lengthening it would eat the
zero vector and break current sensing. Verified by preprocessor output that no
zero-vector timing macro moved.

`mc_config.c`'s `CurrRegConv.samplingTime` was `0`, sharing channel 0's SMPR
register with the NTC. It is inert today (`CURR_SENSOR_TYPE` is `VIRTUAL_SENSOR`
on M365) but would silently clobber the NTC's sampling time back to 1.5 cycles if
anyone ever flipped that. Set to `M1_TEMP_SAMPLING_TIME`.

## 3. Memory

RAM was at **99.7 % — 64 bytes of static headroom**. `ucHeap` was 70 % of all RAM.

- `ADC_SAMPLE_MAX_LEN` 1000 → **500**, halving the VESC Tool scope burst from
  7000 B to 3500 B. Scope still captures, at half the length.
- `top` now reports `xPortGetMinimumEverFreeHeapSize()` — the peak, not the
  current free, which is what matters when the largest consumer is transient.
- `HEAP_SIZE_KB` 14 → **11**, sized from a live measurement (`MinEverFree` 6168 B
  → peak used 8168 B; `ceil(8168 × 1.30 / 1024)` = 11).

Result: **3 048 B of free static RAM**, up from 64 B. Every `pvPortMalloc` site was
audited first; the largest burst not present in the measurement is 1 056 B
(`COMM_GET_MCCONF`, and motor detection), against 3 096 B of headroom.

> ⚠️ This sizing assumes a SCOPE_UVW capture never happens. A capture needs 3500 B,
> which exceeds the headroom. It fails the `pvPortMalloc` NULL check and trips the
> malloc-failed hook — 3 blinks and a safed power stage, not a silent failure. If
> you want the scope back, raise `HEAP_SIZE_KB` first.

## 4. RTOS fault detection

`configCHECK_FOR_STACK_OVERFLOW` 0 → **2**, `configUSE_MALLOC_FAILED_HOOK` 0 → **1**.

The flags alone would have done nothing: `cmsis_os2.c` ships `__WEAK` no-op
definitions of both hooks, so enabling the config links cleanly against those and
silently discards every fault. Strong definitions were added in `task_init.c`.

They never do a silent `while(1)`. They record a fault code and the offending
task's name, **stop the bridge first** (`VescToSTM_pwm_stop()` clears TIM1 MOE in
hardware *before* `__disable_irq()` — masking first would freeze the last commanded
duty on a live motor), then blink a distinguishable code forever: 2 for stack
overflow, 3 for malloc failure.

`tskLED`'s stack was raised 128 → **256 words**. Its measured high-water mark was
121 of 128 words, traced frame by frame to the `commands_printf` chain on the fault
path (`send_buffer` 136 B on the stack, then `_vsnprintf` 144 B, then `_ntoa_long`
88 B). 7 words of margin.

## 5. Control loop

### FOC current-PI gain scaling

The largest single control change. `conf_general_calc_apply_foc_cc_kp_ki_gain()`
produces VESC's gains in **SI units** (volts per amp), and they were loaded
straight into the ST current PI — which runs entirely in ADC counts, s16 current
in, s16 voltage out. The domain conversion was missing entirely.

```c
#define FOC_CC_GAIN_SCALE (65536.0f / ((float)NOMINAL_BUS_VOLTAGE_V * (float)CURRENT_FACTOR_A))
```

| | before | after |
|---|---|---|
| `hKpGain` | 167 | **1280** |
| `hKiGain` | 82 | **627** |
| current-loop bandwidth | ~153 rad/s | ~857 rad/s at 63 V |
| time constant | ~6.5 ms | **~1.2 ms** |

Kp and Ki were under by the same factor, so the loop was correctly damped — slow,
not marginal. Torque took 20–30 ms to settle where the detection routine designed
3–5 ms. Both gains are clamped to `INT16_MAX`; they are `int16_t`, and a wrapped
negative gain is positive feedback in the current loop.

### Battery current limiter

ST's Clarke/Park is amplitude invariant, so `P = 3/2·(Vq·Iq + Vd·Id)`. The limiter
divided by **32768** — a factor of 2 where 1.5 belongs. The estimate ran 4/3 high
and the limiter tripped at **0.75 × `l_in_current_max`**: a configured 35 A limited
at 26.25 A, matching ~25 A measured in a BMS log.

Replaced with `BATT_I_SCALE 43691` (= 65536/1.5) in all three places — the estimate
and the two back-solves, which invert the same relation and have to move together.
Written as a divide rather than `* 3 / 2` because `i_c·v_c` reaches 1.07e9 and
multiplying by 3 would overflow `int32_t`.

### Electrical angle re-sync on PWM re-engagement

With `cc_min_current > 0` the bridge freewheels while coasting. During that window
the high-frequency task is stopped, so **`hElAngle` is frozen for the entire
coast** — not one sector stale. Coast three seconds at 20 km/h and it is ~300
electrical revolutions out of date.

It does not recover on its own: above `SwitchSpeed` the hall driver only advances
the angle if the error is under ~66°, and otherwise leaves it alone.

`VescToSTM_pwm_start()` re-seeds `hElAngle` and `CompAngle` from the last hall edge
before switching on, bounding the error to one 60° sector. `CompAngle` moves with
it because `CompSpeed` is derived from their difference — leaving them a sector
apart would slam `CompSpeed` to its ±1000 clamp. Speed state is cleared only at a
true standstill.

Confirmed on the vehicle: raising `foc_hall_interp_erpm` above maximum ERPM forces
the same re-sync continuously, and the re-engagement punch disappears.

### Inverse-Park lead compensation

`REV_PARK_ANGLE_COMPENSATION_FACTOR` 0 → **1**. The sample-to-applied-voltage delay
is 1.5 PWM periods (currents sampled at counter overflow, new CCRs effective at the
next update event, centroid half a period later). The theoretical value is 1.5, but
the expression is `int16_t` and 1.5 would promote it to double — a soft-float
multiply inside the 16 kHz loop on an FPU-less Cortex-M3.

1 removes 3–5° of genuine lag at speed. The residual 1.5–2.7° costs under 0.11 % of
torque (`1 − cos θ`). 1 also errs safe: over-compensation leads the angle, injects
positive d-axis current and is mildly destabilising.

### Hall FIFO depth

`HALL_AVERAGING_FIFO_DEPTH` 12 → **6**, halving speed-loop phase lag (~75 ms → ~38 ms
at 5 km/h). The buffer is a separate constant fixed at 16, and every loop bound and
the averaging divisor are runtime fields, so nothing assumed 12. RAM is unchanged.

Cost: that averaging smooths hall-sensor mechanical misalignment. Fewer samples
means a noisier speed estimate at low rpm. First thing to put back up if low-speed
running turns rough.

## 6. Feed forward

Feed forward is enabled. `feed_forward_ctrl.o` is linked from ST's precompiled
`libmc-gcc_M3.lib` (two linker options; the rest of the `.cproject` diff is
reformatting, verified attribute by attribute).

ST ships no formulas for the constants, so they were derived from the algorithm and
validated against the MC Workbench rows. The s16 voltage domain — **32767 ≙ Vbus/2,
K = 65536** — is independently agreed by three code paths in this firmware: ST's
Workbench constants, `tune.c`'s flux-linkage detection, and the PI integrator
preload. `FF_VOLTAGE_SCALE` therefore stays 1.0.

**Enabling it naively caused an uncommanded-torque runaway on throttle release.**
Four structural integration faults were found, all confirmed by disassembling
`feed_forward_ctrl.o`:

| | fault | fix |
|---|---|---|
| (a) | Calling `FF_DataProcess` as well as the other two applies `Vqd_PI + ff + LPF(Vqd_PI)` — the current loop's proportional gain doubled | Don't call it. `--gc-sections` discards it |
| (b) | The PWM-off gate tested the **applied** Vq, which with FF on holds at back-EMF for as long as the wheel turns — so the bridge never freewheeled after release | Gate on `AvVolt_qd_pi`, the PI output with the FF contribution removed, low-passed identically |
| (c) | The min-duty deadband was applied to the PID output alone, deleting exactly the small error term FF is designed to leave behind | Moved after the FF add, where it tests the voltage actually commanded |
| (d) | `VescToSTM_pwm_start()` preloads the Iq integrator with `65536·λ·ω/Vbus` — **bit for bit the same quantity** `Vqdff.q` supplies. Twice the back-EMF the instant PWM restarts | With FF on the integrators start at zero; FF carries it, and tracks speed continuously where the preload latched one sample |

`FW_M1.AvVolt_qd` deliberately still tracks the applied voltage — the battery
current limiter and the Vd/Vq/duty reporting need the real value. The two signals
have different jobs; the fault was using one for both.

`Vqdff` saturates where back-EMF reaches Vbus/2 — about **41 km/h at 63 V**, 35 at
54 V, 29 at 45 V. That is inside the riding range but is not a fault: the modulator
runs out at the same point, so saturation and top speed arrive together, and above
it feed forward simply stops growing while the PI makes up the difference.

## 7. Thermal derate

The derate was mapped against **`fp.battery_cut_end` — a bus-voltage threshold in
ADC digits** — as its temperature end point. `fp.temp_cut_end` was written by
`VescToSTM_set_temp_cut()` and read nowhere in the firmware. `utils_map_int()` does
not clamp.

Two failures. Torque stepped to **zero** the instant `l_temp_fet_start` was crossed
instead of beginning a taper; and beyond that the multiplier went **negative** —
reversing commanded torque into braking at up to 1.77× the commanded current. An
uncommanded brake application while riding.

Pre-existing, present in every build including upstream.

Replaced with a clamped, sign-preserving taper against the correct end point.
Direction matters: NTC digits *fall* as the board heats, so the span is
`start − end`. `num` is bounded to `[0, span]`, so the multiplier stays in `[0, 1]`.
Scaling `iq` by a fraction rather than mapping it keeps the sign, so regen derates
by the same factor instead of inverting. `span <= 0` fails safe to zero torque.

> The FET thermistor on this board measures **~12.9 kΩ at 25 °C**, not the 10 k the
> firmware's linear model assumes, so reported temperature reads low —
> approximately 18 °C at a true 25 °C. `l_temp_fet_start` = 50 therefore trips at a
> true ~57 °C. Analysed but **not fixed**: correcting the conversion also requires
> new thresholds and a raised `HW_LIM_TEMP_FET`, and those must ship together.

## 8. Configuration handling

### Drive-mode current scale

`conf_general_setup_mc()` forces `lo_current_max_scale = 1.0` on every config load
or write. Nothing put the mode's scale back. Ride in SLOW, save any setting from
VESC Tool, and the current cap silently jumped from 0.5 to 1.0 while the display
still read SLOW.

A new `app_adc_apply_mode_scale()` is called at the `mc_conf = *mcconf` choke point
that every config write passes through, covering all six call sites. The change is
always a power *reduction*, never an increase.

A second path was missed by that fix and closed separately: after a **successful**
"Detect FOC Parameters", `*mcconf = *mcconf_old` copies a struct still carrying 1.0
over the corrected global, and the recovery `setup_mc()` call is guarded by
`result < 0`. So detection left the controller at full current scale in every drive
mode until the next mode press. The re-apply is now unconditional.

### Drive-mode speed limits

`MODE_SLOW_SPEED` and `MODE_DRIVE_SPEED` → `KMH_NO_LIMIT`. Any value other than 1337
computed a reduced `lo_max_erpm`; the modes now differ by current scale alone.

Note these are fields of `mc_configuration` and persist in flash, and they are not
part of the serialised VESC protocol — so the new compile-time defaults only apply
on a CRC-failure reset. The terminal commands `mode_speed [1-3] [kmh]` and
`mode_scale [1-3] [percent]` change them live (lost at power-off).

### `app.c` `APP_ADC` fallthrough

`case APP_ADC` had no `break` and ran on into `case APP_ADC_UART`. Harmless at boot
— the scheduler isn't running, so the kill blocks are skipped, and both init
functions guard on the same shared `port->task_handle`.

But with the scheduler up, **any** app-config write from VESC Tool would make the
fallthrough call `task_app_kill()` on the very task the line above had just kept
alive, and `task_cli_init()` would then claim the freed handle. The ESP32 display
task is destroyed and a VESC CLI task takes over the half-duplex UART. Throttle and
brake arrive in frame 0x65 via that task, so **they stop until the next power
cycle.** Reachable from an ordinary app-config write.

One `break` added. No RAM is recovered and none is claimed.

## 9. Telemetry and display

Two unused bytes in the 14-byte M365 display frame were repurposed:

| offset | was | now |
|---|---|---|
| 9 | `beep` (only ever written 0) | **`esc_temp`** — `int8_t` °C |
| 11 | `faultcode` (not read by the display) | **`power`** — `int8_t`, 20 W per count, ±2540 W, negative on regen |

The struct has **no packing directive** — it is 14 bytes only because every member
happens to be byte-sized. Seven `_Static_assert`s now pin the size and every payload
offset; the guard was verified to fire by compiling a deliberately broken copy.
Frame length, checksum range (bytes 2…11, which covers both new fields) and the
0x65 throttle/brake handler are unchanged, so existing display firmware keeps
working untouched. Both new bytes must be read as **signed**.

Byte layout: [`display-protocol.md`](display-protocol.md).

### Telemetry rate

`slow_update_cnt` was initialised to 0 with its only increment inside an unreachable
`else`, so the "slow" telemetry block ran on **every** loop pass — 10 ms rather than
the intended divider. Fixed in both writers (`app_uartcomm.c` and `task_cli.c`) to
give **100 ms**, half the display's 200 ms poll, so every poll still gets data at
most one refresh old. The originally specified 1-in-50 was rejected: at 500 ms it is
2.5× slower than the poll and the display would repeat stale readings.

### Pole count

`si_motor_poles` is used inconsistently across the tree. Six consumers were traced:
five treat it as pole **pairs**, and `VescCommand.c:826` halves it, treating it as a
pole count. The decisive evidence is `VescToSTM_speed_to_rpm()` and
`VescToSTM_speed_to_erpm()` — same input unit, differing by exactly one factor of
`si_motor_poles`, which is the definition of the pole-pair count.

The SDK hall path returns **electrical** 0.1 Hz despite the `MecSpeedUnit` name:
`bElToMecRatio` is never assigned for `HALL_M1`. The `/ 2.0` was removed, which was
halving the ERPM limits set from VESC Tool's speed-limit page.

> **VESC Tool still displays roughly 2× the firmware's speed.** It receives the raw
> field and interprets it as a pole count. That predates this change and is
> unaffected by it. Making them agree would mean changing the meaning of a persisted
> config field at all five sites.

## 10. Other fixes

- **`undriven_samples` read uninitialised** in
  `COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP`. Nothing ever writes it, so the test
  read stack garbage — and the branch it guards sets `linkage = linkage / 2.0`. The
  reported flux linkage was **halved at random**. Initialised to 0.0f.
- **Duty cycle reported stale.** `VescToSTM_get_duty_cycle_now()` read
  `FW_M1.AvVoltAmpl`, written only inside `FW_CalcCurrRef()` under
  `bDriveInput == INTERNAL`. Under throttle the field was frozen at whatever the
  last braking or ramp command left there. Recomputed from `AvVolt_qd`, which
  updates every FOC cycle regardless of mode.
- **Hall angle read once** at PWM re-engagement rather than twice. `MeasuredElAngle`
  is written by the TIM3 hall ISRs, which are unaffected by PWM state and can fire
  between two reads. It is not `volatile`, so GCC at `-Ofast` already emitted a
  single load — the binary is byte-identical. The emitted code was safe; the source
  was not.

---

## Known limitations

- **No motor temperature protection.** `l_temp_motor_start` and `l_temp_motor_end`
  are stored but read by nothing, and `VescToSTM_get_temperature2()` returns a
  hardcoded 0.0. This matters if you run field weakening, which is pure copper loss.
- **FET temperature reads low** — see §7. Analysed, not fixed.
- **`foc_fw_ramp_time` is dead.** VESC Tool sends it and it is stored, but nothing
  reads it, so field-weakening current engages as a step rather than a ramp.
- **Field weakening regulates modulation *down to* `foc_fw_duty_start`** rather than
  starting there — it is ST's regulator, not a threshold. Set `foc_fw_duty_start`
  just under the modulation actually reached without FW, or a chunk of the FW
  current is spent buying back voltage the loop gave away. Also: never set
  `foc_fw_current_max` ≥ `l_current_max` — `sqrt(Imax² − Ifw²)` goes negative,
  `MCM_Sqrt` returns 0, and **Iq is clamped to zero**, losing all torque at speed.
- **`g30p` does not link.** It was already 24 bytes over its 20 K RAM budget on the
  clean upstream clone, before any change here; the RTOS fault globals take it to
  48. It is not a target being built and cannot be used as a regression check.
- **Sensorless is not implemented.** ST's `sto_pll` / `sto_cordic` / `revup_ctrl`
  are available only as objects in `libmc-gcc_M3.lib`, and this repo's
  `SpeednPosFdbk_Handle_t` and `STO_PLL_Handle_t` have both been forked from ST's,
  so those objects are ABI-incompatible with these headers. Feasibility only.
- **`conf_general_setup_mc()` rewrites live control state with no critical
  section** — pre-existing. Each store is single-word and aligned, so on Cortex-M3
  no individual value can tear; only a mixed old/new set is observable, for one
  cycle. One related window was closed: `FF_Init()` blanked the feed-forward
  constants mid-write, so the computed values are now installed before it is called.
