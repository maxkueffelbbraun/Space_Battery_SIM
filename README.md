# Space Battery SIM

ESP32-C6 test firmware (ESP-IDF via PlatformIO) for talking to a B.Braun Space battery pack (Charging Unit) over UART. A USB serial console lets you send protocol requests, read battery data and control the test PCB.

## Build and flash

```
pio run                  # build
pio run --target upload  # flash
pio device monitor --port COMxx --baud 115200
```

The console runs over the ESP32-C6 USB-Serial/JTAG port (115200 is only the console speed, not the battery UART).

## Battery UART

| Setting | Value |
|---|---|
| Port | UART1 |
| Baud rate | 19200 (compatible mode); the DIP switches SW1 select another rate at boot |
| Format | 8N1, no flow control |
| ESP TX / RX | GPIO1 / GPIO0 (net names are from the battery's view) |

Frames are 5 bytes: `[type][cmd][d0][d1][checksum]`, type `?` (0x3F) = read, `!` (0x21) = write, checksum = `0 - sum(first 4 bytes)`. Replies start with `!` and echo the command. Reads: `s c v t r 1 2`, writes: `d a r`. See `Dok/HAPBBNA-CP-1.40_Communication Protocol NiMH_LiON 1.PDF`.

## Console commands

Type `help` for the full list.

| Command | Purpose |
|---|---|
| `info` | Read version, capacity, remaining time, day counter, status flags, serial no. |
| `probe` | Find baud rate/parity with a version request |
| `rd <cmd> [d0] [d1]` | Read request, e.g. `rd v`, `rd c A5` |
| `wr <cmd> [d0] [d1]` | Write request, **changes battery state** |
| `status` | UART settings, link state, GPIO states, counters |
| `baud <n>`, `parity n\|e\|o`, `inv 0\|1` | UART settings |
| `scan [ms]`, `tx <hex..>`, `txt <text>` | Raw diagnostics |
| `route esp\|pc` | Route battery UART to the ESP or the isolated PC adapter |
| `pwr`, `load`, `q3`, `q4` `0\|1` | +12 V switch, current sink, low-side switches |
| `vbat` | Board ADC voltage measurement |
| `poll 0\|1` | Background `?s` request every 2 s |

The onboard RGB LED shows the link state: blue unknown, green OK, red no reply, orange echo only, magenta framing/parity errors. Flashes: cyan TX, yellow RX.

## Troubleshooting

- **No reply at all:** the Charging Unit sleeps in STOP mode with its UART disabled unless Vcc is at least 10 V or a discharge current flows. UART traffic does not wake it. Supply 10-12 V to the battery supply input first (`pwr 1` on the PCB).
- **`! break` and `00` bytes about once per second:** the battery received no valid telegram for 30 s and signals its charge state on TX. Send any valid request, or keep `poll 1` enabled (the poll interval is 2 s).
- **Serial no. `0xFFFFFFFF`:** the EEPROM has no serial number programmed.
- **`Voltage` in `info`:** this is the board ADC input, not the battery voltage, unless the PCB is connected.
- The battery TX line is open-drain; the firmware enables the internal pull-up on the ESP RX pin.

## Layout

- `src/main.c` - firmware
- `Dok/` - protocol specification, schematics, reference scripts and test tool
