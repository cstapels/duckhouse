# Duck House Controller firmware (slice 4: light sleep)

ESP-IDF **v5.5.6**, original ESP32 (WROOM-32U DevKit). Written against the
*Duck House Controller Software Specification* and Hardware Specification Rev 1.8.

**Status: slice 4 of the firmware. It has not been compiled or run.** There is no ESP-IDF
toolchain in the environment it was written in, so expect a few compile errors
and API mismatches on the first build. Fix those, then work through the bring-up
steps in section 15 of the software spec.

## Build

```
. $IDF_PATH/export.sh
idf.py set-target esp32
idf.py build flash monitor
```

## What is in this slice

| Component | Spec | What it does |
| --- | --- | --- |
| `board` | 4, 6.5, 11 | All outputs LOW first. Reference-counted BTS7960 logic power (GPIO17): on before any move, 200 ms settle, off 5 s after the last move. Limit-switch supply (GPIO2) only while moving. |
| `led` | 10 | 2 flashes at startup, flashing during moves. Timer runs only while a pattern is active, so an idle LED never blocks light sleep. |
| `config` | 12 | NVS wrapper. Writes only on change. |
| `sensors` | 8 | ADC1 raw counts (battery, photo, pots) at 12 bit / 12 dB, no calibration. Travel-time pots map to 20-50 s. Tank level via touch pad 7. |
| `inputs` | 7 | Four buttons on rising-edge interrupts, 30 ms timer debounce, one event per press, both buttons of one object together are ignored. Limit switch reads. |
| `actuator` | 6 | Lift and door state machines: wait for logic power, soft-start PWM at 20 kHz, limit stops, timed lift-open (no Open limit), travel-timeout fault, reversal with 500 ms dead time, 500 ms stagger when both start together, saved position with no recovery move on boot. |
| `net` | 13 | Poll task on core 0 every 4 min (15 min after 5 failed cycles). Wi-Fi on only for the cycle: 3 attempts, 15 s each, 2 s then 5 s gaps. A button press cancels the cycle until the next period; a poll that comes due during a move is skipped. Reads the last 10 entries of the duck door channel and takes the newest non-empty field 1 (command) and field 5 (mode). Writes `update.json` with fields 1 and 5 = 0, RSSI, door state, light, lift state, battery count and the status message. Sets the clock from `created_at`; time zone `EST5EDT,M3.2.0,M11.1.0`. 3 LED flashes on connect. |
| `pump` | 9.3, 13 | Pump on GPIO23 from the pump channel 1160824: run length and interval (s), mode bits (sleep, pump enable, overnight off, level check, battery check), daily window decoded as wake = v mod 12 and sleep = floor(v / 12) + 13 in Boston time, battery set point (ADC count), tank set point (touch count). Hard limits 30 min run and 1 min interval. Start is blocked by the window, a battery reading below the set point, or a full tank (count at or below the set point); a running pump stops at run end, at the sleep hour, or when the tank fills, but never because of low battery. Settings are validated and saved to NVS as last-good. Activity goes into the status message ("pump 2 runs 120 s total, last 14:32"), plus "battery low". |
| `power` | 7 | Light-sleep manager. Stays awake while any of these is true: an actuator is active, the BTS7960 logic supply is still on, a button debounce is running, the pump is running, a poll cycle is running, or an LED pattern is active. After 5 s of that being false it sleeps until the next scheduled event (next poll, next pump evaluation, or a 60 s safety cap). Any of the four buttons wakes it. If pump channel mode bit 1 is 0 it never sleeps. |
| `main` | 2, 4 | Safe start-up, 10 ms control task with the task watchdog. |

## One deliberate difference from the spec

The spec has the button ISR start the debounce timer. Here the ISR only disables
that pin's interrupt and posts to a queue, and a small task starts the timer.
That avoids calling `esp_timer` from interrupt context. Behaviour is the same.

## Not built yet

1. Auto mode (acts once at each light change) and the Manual/Auto switch.
2. DS18B20 temperature on GPIO26.

## Pump notes

- The pump channel is read hourly between the wake and sleep hours, in the same
  Wi-Fi session as a door poll that is already due, plus once after the clock is
  first set following a boot. It takes the newest non-empty value of fields 1-6
  from the last 20 entries.
- With nothing stored the pump is disabled and sleep is enabled.
- **[ASSUMED]** After a boot the first run is one interval later, so a reboot
  loop cannot pump repeatedly. A blocked start is re-checked every 30 s.
- `pump.c` passed a syntax check against stub headers only.

## Light sleep notes

- **Why timers changed.** Light sleep stops the FreeRTOS tick but not `esp_timer`
  time. So nothing that waits a long time uses `vTaskDelay` any more: the 4 min
  poll and the pump are driven by one-shot `esp_timer`s that notify their task.
  The control task runs a 10 ms tick only while an actuator is active and
  otherwise blocks on its input queue.
- **Buttons.** The four inputs now use a level-high interrupt instead of
  rising edge, because the same level is the wake source and
  `gpio_wakeup_enable()` shares the pin's interrupt type. The ISR disables the
  pin's interrupt on the first hit and it is only re-armed after release, so one
  press is still one event.
- **Watchdog.** The idle-task watchdog check is turned off in
  `sdkconfig.defaults` (the esp_timer time jump after a sleep would trip it) and
  the control task is only subscribed to the task watchdog while it is active.
- **Pump re-checks.** Outside the window the pump re-checks at the next top of
  the hour, not every 30 s. A start blocked by battery or tank is re-checked
  every 30 s, which means a short wake each time.
- **Not measured.** Idle current has not been measured. Do this first: put a
  meter in series with the battery, leave sleep enabled (mode bit 1 = 1) and
  compare with the estimate in the spec's power budget.
- `power.c` and `pump.c` passed a syntax check against stub headers. Nothing
  has been compiled against ESP-IDF.

## Provisioning

Wi-Fi and ThingSpeak keys go into NVS, not the source. See `provision/README.md`.
Without them the poll cycle logs an error and does nothing.

## Slice 2 notes

- The write happens right after a command is *started*, so the door and lift
  states in that update still show the old state. The new state appears in the
  next cycle.
- Temperature (field 7) is not written yet; the DS18B20 is not built.
- The actuator API is now mutex-protected because the net task issues commands
  while the control task ticks.
- `ts.c` parsing of `created_at` was checked on a host against Python for both
  the `Z` and the `-05:00` forms. Nothing else has been compiled.

## Check on the first run

- **Touch driver:** `sensors.c` uses the legacy ESP32 touch API
  (`touch_pad_*`). Confirm it builds under v5.5 and that the count falls as the
  tank fills. The GPIO27 pulldown and capacitor must not be fitted.
- **Wi-Fi:** provision credentials, then watch the log for `connect attempt`,
  `reported, entry N, time valid 1` and 3 LED flashes. Unplug the router to see
  3 failed attempts and "wifi failed" in the next status message.
- **ThingSpeak:** write 1 to field 1 of the duck door channel and confirm the
  door opens within one cycle and field 1 reads 0 afterwards.
- **Sleep:** with sleep enabled the log should go quiet about 5 s after the last
  activity, a button press should wake it and start the move, and the poll should
  still fire about every 4 minutes (check the timestamps).
- **Boot:** every output stays LOW from reset until `board_init_safe()`.
- **Limits:** with the limit supply off, the limit inputs read LOW.
- **Dead time:** pressing a button during the 500 ms dead time is ignored.
