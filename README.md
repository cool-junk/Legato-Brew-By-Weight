# Legato brew-by-weight screen prototype

This firmware combines the Waveshare ESP32-S3-Touch-AMOLED-1.75 display/touch setup with live Timemore DOT BLE readings and active-low control of the verified four-channel relay module.

## Screen

- Extra-large bold live weight digits rendered by a custom 72 px LVGL font derived from the installed Montserrat font, with a small `g` unit beside them. The green progress ring is read-only.
- Tap the top mode pill to cycle through Espresso, Americano, Beans, and the new Info page. Info shows firmware version/build time, active mode and brew state, target/live scale/battery status, relay pulse and GPIO assignments, safety limits, and uptime. From Info, tap the pill to return to Espresso. If a fault is latched, tapping the pill opens Info without clearing the fault; tap again to return to the previous mode. Beans mode is scale-only with an 18 g default target and no brew outputs.
- Target values are saved independently for all three modes across ESP32 reboots.
- Place the empty cup on the scale, then use the large combined control: while idle, a quick tap requests DOT tare and holding it for at least one second starts the selected Espresso or Americano cycle. During a cycle, tap the control to stop it. Tare is unavailable while brewing.
- The main control reads **TAP TO TARE / HOLD TO START** in Espresso and Americano modes, uses centered 32 px text, and spans most of the screen width. During a cycle it reads **TAP TO STOP**. Target adjustment buttons are below it.
- Espresso uses the Auto Brew relay; Americano water uses the Manual Brew relay. Reaching the scale target sends the corresponding second press automatically. Beans mode keeps the same control as a tare-only button.
- The separate **POWER** control stays at the top center and pulses the machine power button while no brew is active. The progress ring has a gap aligned from one edge of that button to the opposite edge. The sketch does not sense or display the machine's actual power state.
- The firmware version is `0.1.0`; the Info page shows this version and compile date/time, current mode/brew state/target, scale connection/live weight/battery, relay pulse duration and GPIO assignments, safety limits, and device uptime. Relay on time is the configured 250 ms press pulse.
- BLE status changes between searching, waiting for weight, and live.
- DOT battery percentage appears below the combined control and above the progress ring. It turns red below 20%. The sketch tries the standard BLE Battery Service when present, otherwise requests the DOT-specific report every five seconds until one arrives, then refreshes once per minute.

## Relay operation

- GPIO16 → relay IN1 (Manual Brew); GPIO17 → IN2 (Auto Brew); GPIO18 → IN3 (Power). The relay inputs are active-low; all three are set HIGH/off at startup. Use the tested 5 V supply and input ground reference for the module. Keep machine-side contacts isolated from ESP32 ground.
- Each press is a 250 ms pulse. The sketch never pulses two channels simultaneously. In Beans mode the combined control only tares and does not start or stop a brew output.
- Brew start requires fresh DOT data and a displayed weight no higher than 5 g, so place the cup and tare first. It stops at the saved target with a 0 g stop offset. A fresh-data timeout of 2.5 seconds or a 120-second maximum runtime sends one stop pulse and latches a fault; inspect the machine before acknowledging that fault on screen.
- The relay integration was reported working by the user. This latest combined-button UI revision has not been compiled or uploaded yet. The separate `Legato_Relay_Bench_Test` sketch remains available for relay-only checks.

## BLE

- Uses NimBLE-Arduino 2.x.
- Saves and verifies the DOT address and address type in ESP32 NVS after connecting. On later boots it scans for a live advertisement and prioritizes the saved DOT address; it connects using the current advertisement. Serial output reports the saved address and each connection result.
- Enables BLE bonding. Put the DOT into pairing mode for the first successful ESP32 connection so the scale and ESP32 can create a bond; later reconnects should use that stored bond without pairing mode. Keep Timemore/BeanConqueror disconnected during this first pairing attempt.
- Stores mode and per-mode target grams in ESP32 NVS, verifies each write by reading the values back, and logs the result to Serial.
- Tare uses the DOT BLE write command with opcode `0x0D` and no payload. The displayed weight changes when the scale reports its tared reading. Tare is available while connected.
- Reads a standard battery value if exposed, or the DOT battery command response otherwise. Short unrecognized BLE packets are printed to Serial for diagnosis; if no battery value is received, the display remains `BAT --%`.
- Serial Monitor at 115200 prints touch coordinates, the exact parsed grams values, and BLE connection/reconnect status.

## Display fixes in this revision

- Restores Waveshare's even-pixel LVGL area rounding for the CO5300 flush path.
- Uses partial render buffers and registers the full 44-row allocation so the QSPI rounder has extra headroom.
- Marks the perimeter progress arc read-only, preventing touches from dragging the ring.
- Updates LVGL weight/status objects only when new BLE data arrives or the status changes, avoiding needless redraws.

## Arduino IDE requirements

Use the known-working board setup: ESP32S3 Dev Module, ESP32 Arduino core 3.1.0, 16 MB flash, `app3M_fat9M_16MB` partition, OPI PSRAM, USB CDC On Boot enabled. Keep LVGL 8.4.0, GFX Library for Arduino 1.6.9, SensorLib 0.3.1, Waveshare Mylibrary `pin_config.h`, and install NimBLE-Arduino by h2zero (2.x).

Open `Legato_Brew_By_Weight.ino`, upload, then open Serial Monitor at 115200 baud. Supervise shots while validating target stop behavior and stale-scale/maximum-time fault handling.
