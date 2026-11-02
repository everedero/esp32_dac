# esp32_dac

Audio on the ESP32 built-in DAC as a Zephyr `audio_codec` (same API/behaviour as
the STM32 DAC audio driver of zephyr PR #115001). Board: NodeMCU-32 /
ESP32-WROOM-32E → `esp32_devkitc/esp32/procpu`.

- `drivers/audio/dac_esp32_i2s.c` (`espressif,esp32-dac-audio`): I2S0 in built-in
  DAC mode paces the samples, its DMA walks a 2-descriptor ring (double buffer).
  16-bit signed mono PCM in, top 8 bits out. Rate ~5 kHz .. 2.5 MHz (PLL clock).
  I2S0 is taken, so no `CONFIG_ADC_ESP32_DMA` and keep `&i2s0` disabled.
- `dac_loopback/`: 200 Hz sine out, read back with the ADC, PASS/FAIL on UART.
- `speaker_demo/`: C-major scale + A4 (440 Hz tuner check) in a loop. Wiring: GPIO25 → 1 µF
  → amp IN (PAM8403/LM386), GND shared; the cap blocks the ~1.6 V DC idle.
  Volume knob: `SINE_AMPLITUDE`.

## Wiring

GPIO25 (DAC1) → 270 Ω → GPIO34 (ADC1_CH6).

## Setup, build, test

```bash
west config manifest.path esp32_dac && west update && west blobs fetch hal_espressif
source ~/zephyrproject/.venv/bin/activate
ZEPHYR_BASE=$PWD/zephyr west build -p -b esp32_devkitc/esp32/procpu esp32_dac/dac_loopback \
    -- -DPython3_EXECUTABLE=/home/eve/zephyrproject/.venv/bin/python3
west flash --esp-device /dev/ttyUSB0
python3 esp32_dac/scripts/uart_capture.py   # resets via RTS, exit 0 on PASS
```

Measured: `PASS p2p=1139mV rms=402mV crest=1.414 freq=199Hz mean=1603mV`
at 24 kHz, also fine at 44.1 kHz.

## Gotchas

- No `float` on hot paths you didn't check: GCC puts float literals in IROM and
  loads them with `lsi`, which faults (EXCCAUSE 3). The test uses `double`.
- The ESP32 ADC throws single-sample spikes; the test median-filters before p2p.
