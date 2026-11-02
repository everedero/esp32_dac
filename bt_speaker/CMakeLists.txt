# SPDX-License-Identifier: Apache-2.0
# Reuses Zephyr's a2dp_sink sample (GAP/SDP/A2DP + SBC decode); only the
# audio output (codec_play.c) is ours, targeting the ESP32 DAC codec.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bt_speaker)

set(SAMPLE ${ZEPHYR_BASE}/samples/bluetooth/classic/a2dp_sink)
target_sources(app PRIVATE ${SAMPLE}/src/main.c ${SAMPLE}/src/audio_buf.c src/codec_play.c)
target_include_directories(app PRIVATE ${SAMPLE}/src)
zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
