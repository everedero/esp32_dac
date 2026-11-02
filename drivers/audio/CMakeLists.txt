# SPDX-License-Identifier: Apache-2.0

if(CONFIG_AUDIO_DAC_ESP32)
  zephyr_library()
  zephyr_library_sources(dac_esp32_i2s.c)
endif()
