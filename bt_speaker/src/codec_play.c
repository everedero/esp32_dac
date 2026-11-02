/*
 * Copyright (c) 2026 Eve Redero
 * SPDX-License-Identifier: Apache-2.0
 *
 * codec_play.h backend for the Zephyr a2dp_sink sample: decoded SBC PCM
 * (16-bit, 1 or 2 channels) -> mono ring -> ESP32 DAC audio codec.
 * The sample's main loop thread runs codec_keep_play() as the producer;
 * the DAC tx-done ISR is the consumer.
 */

#include <zephyr/kernel.h>
#include <zephyr/audio/codec.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>
#include "audio_buf.h"
#include "codec_play.h"

LOG_MODULE_REGISTER(codec_play, LOG_LEVEL_INF);

#define BLOCK_SAMPLES 256U
#define BLOCK_BYTES   (BLOCK_SAMPLES * sizeof(int16_t))
#define CHUNK_FRAMES  480U   /* 10 ms @ 48 kHz, the largest audio_buf hands out */

static const struct device *codec_dev = DEVICE_DT_GET(DT_ALIAS(codec0));

/* ponytail: phone clock vs I2S PLL drift is absorbed by backpressure into
 * audio_buf (drops when full) and silence on underrun. Add sample slipping if
 * clicks show up every few seconds.
 */
RING_BUF_DECLARE(pcm_ring, 8192);   /* ~93 ms mono @ 44.1 kHz */

static int16_t tx_buf[BLOCK_SAMPLES];
static int16_t mono[CHUNK_FRAMES];
static uint32_t chunk_frames;
static uint8_t num_ch;
static volatile bool playing;

static void tx_done(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	uint32_t got = ring_buf_get(&pcm_ring, (uint8_t *)tx_buf, BLOCK_BYTES);

	if (got < BLOCK_BYTES) {
		memset((uint8_t *)tx_buf + got, 0, BLOCK_BYTES - got);
	}
	audio_codec_write(dev, (uint8_t *)tx_buf, BLOCK_BYTES);
}

int codec_play_init(void)
{
	if (!device_is_ready(codec_dev)) {
		LOG_ERR("codec device not ready");
		return -ENODEV;
	}
	return audio_codec_register_done_callback(codec_dev, tx_done, NULL, NULL, NULL);
}

void codec_play_configure(uint32_t sample_rate, uint8_t sample_width, uint8_t channels)
{
	struct audio_codec_cfg cfg = {
		.dai_type = AUDIO_DAI_TYPE_PCM,
		.dai_route = AUDIO_ROUTE_PLAYBACK,
		.dai_cfg.pcm = {
			.dir = AUDIO_DAI_DIR_TX,
			.pcm_width = AUDIO_PCM_WIDTH_16_BITS,
			.channels = 1,
			.block_size = BLOCK_BYTES,
			.samplerate = sample_rate,
		},
	};
	int ret;

	if (sample_width != 16 || channels == 0 || channels > 2) {
		LOG_ERR("unsupported PCM: %u bit, %u ch", sample_width, channels);
		return;
	}
	num_ch = channels;
	chunk_frames = sample_rate / 100;   /* audio_buf works in 10 ms chunks */

	ret = audio_codec_configure(codec_dev, &cfg);
	if (ret < 0) {
		LOG_ERR("audio_codec_configure failed: %d", ret);
		return;
	}
	LOG_INF("configured %u Hz, %u ch -> mono DAC", sample_rate, channels);
}

void codec_play_start(void)
{
	if (playing) {
		return;
	}
	ring_buf_reset(&pcm_ring);
	playing = true;
	audio_codec_start(codec_dev, AUDIO_DAI_DIR_TX);
}

void codec_play_stop(void)
{
	if (!playing) {
		return;
	}
	playing = false;
	audio_codec_stop(codec_dev, AUDIO_DAI_DIR_TX);
}

void codec_keep_play(void)
{
	while (true) {
		uint32_t len = chunk_frames * num_ch * sizeof(int16_t);
		uint8_t *data;

		if (!playing || ring_buf_space_get(&pcm_ring) < chunk_frames * sizeof(int16_t)) {
			k_msleep(2);
			continue;
		}

		audio_get_pcm_data(&data, len);
		if (data == NULL) {
			k_msleep(2);
			continue;
		}

		const int16_t *pcm = (const int16_t *)data;

		for (uint32_t i = 0; i < chunk_frames; i++) {
			mono[i] = (num_ch == 2) ? (int16_t)(((int32_t)pcm[2 * i] + pcm[2 * i + 1]) / 2)
						: pcm[i];
		}
		ring_buf_put(&pcm_ring, (uint8_t *)mono, chunk_frames * sizeof(int16_t));
		audio_media_sync(data, len);
	}
}
