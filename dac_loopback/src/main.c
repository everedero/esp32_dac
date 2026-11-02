/*
 * Copyright (c) 2026 Eve Redero
 * SPDX-License-Identifier: Apache-2.0
 *
 * DAC loopback self-test: sine on GPIO25 (ESP32 DAC1 via the audio codec
 * driver), captured back on GPIO34 (ADC1_CH6). Prints PASS/FAIL on UART.
 * Adapted from fah_dac/dac_loopback, metrics are in millivolts here.
 * double, not float: GCC puts float literals in IROM and loads them with
 * lsi, which faults on ESP32 (IROM only allows 32-bit integer loads).
 */

#include <zephyr/kernel.h>
#include <zephyr/audio/codec.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(dac_loopback, LOG_LEVEL_INF);

/* ── User-tunable parameters ─────────────────────────────────────────────── */

#define SINE_FREQ_HZ    200     /* generated frequency, Hz                    */
#define SINE_AMPLITUDE  48      /* peak amplitude in 8-bit DAC codes (0-127)  */
#define TOLERANCE_PCT   10      /* +/-% tolerance applied to every metric     */

/* Calibration knob: DAC full scale (code 256) in mV, ~VDD3P3_RTC.
 * 3200 measured on the NodeMCU-32 (mid-scale code 128 reads ~1605 mV).
 */
#define DAC_FULLSCALE_MV 3200

/* ── DAC / sine generator ────────────────────────────────────────────────── */

#define SAMPLE_RATE    24000U   /* PLL clock: >= ~19.6 kHz */
#define BLOCK_SAMPLES  256U
#define BLOCK_BYTES    (BLOCK_SAMPLES * sizeof(int16_t))
#define FREQ_STEP      ((uint32_t)(((uint64_t)SINE_FREQ_HZ << 32) / SAMPLE_RATE))

static const struct device *codec_dev = DEVICE_DT_GET(DT_ALIAS(codec0));

static int16_t audio_buf[BLOCK_SAMPLES];
static uint32_t phase_acc;
static int16_t sine_table[256];

static void build_sine_table(void)
{
	/* 8-bit DAC code amplitude -> 16-bit PCM (code << 8) */
	for (int i = 0; i < 256; i++) {
		sine_table[i] = (int16_t)(SINE_AMPLITUDE * 256 *
					  sin(2.0 * 3.14159265 * i / 256.0));
	}
}

static void tx_done(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	for (uint32_t i = 0; i < BLOCK_SAMPLES; i++) {
		audio_buf[i] = sine_table[phase_acc >> 24];
		phase_acc += FREQ_STEP;
	}
	audio_codec_write(dev, (uint8_t *)audio_buf, BLOCK_BYTES);
}

/* ── ADC loopback capture & self-test ────────────────────────────────────── */

#define CAPTURE_SAMPLES     1024
#define ADC_INTER_SAMPLE_US 100U

#define CODE_TO_MV(c) ((double)(c) * DAC_FULLSCALE_MV / 256.0)
#define EXP_P2P    CODE_TO_MV(2 * SINE_AMPLITUDE)
#define EXP_RMS    (CODE_TO_MV(SINE_AMPLITUDE) / 1.41421356)
#define EXP_CREST  1.41421356
#define EXP_MEAN   CODE_TO_MV(128)
#define EXP_FREQ   ((double)SINE_FREQ_HZ)
#define TOL        (TOLERANCE_PCT / 100.0)

static const struct adc_dt_spec adc_ch = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

static bool in_tol(double val, double exp)
{
	double margin = exp * TOL;

	return (val >= exp - margin) && (val <= exp + margin);
}

static void capture_and_check(void)
{
	static int32_t mv[CAPTURE_SAMPLES];
	int16_t raw;
	struct adc_sequence seq = {
		.buffer = &raw,
		.buffer_size = sizeof(raw),
	};

	adc_sequence_init_dt(&adc_ch, &seq);

	int64_t t0 = k_uptime_get();

	for (int i = 0; i < CAPTURE_SAMPLES; i++) {
		if (adc_read_dt(&adc_ch, &seq) < 0) {
			LOG_ERR("FAIL: adc_read[%d] error", i);
			return;
		}
		mv[i] = raw;
		adc_raw_to_millivolts_dt(&adc_ch, &mv[i]);
		k_busy_wait(ADC_INTER_SAMPLE_US);
	}

	int64_t t1 = k_uptime_get();

	/* 3-tap median: the ESP32 ADC throws single-sample spikes that wreck p2p */
	for (int i = 1, prev = mv[0]; i < CAPTURE_SAMPLES - 1; i++) {
		int32_t a = prev, b = mv[i], c = mv[i + 1];

		prev = b;
		mv[i] = MAX(MIN(a, b), MIN(MAX(a, b), c));
	}

	/* ── Amplitude metrics ── */
	int32_t s_min = INT32_MAX, s_max = INT32_MIN;
	int64_t sum = 0;

	for (int i = 0; i < CAPTURE_SAMPLES; i++) {
		s_min = MIN(s_min, mv[i]);
		s_max = MAX(s_max, mv[i]);
		sum += mv[i];
	}

	double mean = (double)sum / CAPTURE_SAMPLES;
	double p2p = (double)(s_max - s_min);
	double sum_sq = 0.0;

	for (int i = 0; i < CAPTURE_SAMPLES; i++) {
		double d = mv[i] - mean;

		sum_sq += d * d;
	}
	double rms = sqrt(sum_sq / CAPTURE_SAMPLES);
	double crest = (rms > 0.5) ? (p2p / 2.0) / rms : 0.0;

	/* ── Frequency: crossings with +/- p2p/4 hysteresis (ADC noise) ── */
	double hyst = p2p / 4.0;
	int crossings = 0;
	bool high = mv[0] > mean;

	for (int i = 1; i < CAPTURE_SAMPLES; i++) {
		if (high && (mv[i] < mean - hyst)) {
			high = false;
			crossings++;
		} else if (!high && (mv[i] > mean + hyst)) {
			high = true;
			crossings++;
		}
	}
	double capture_s = (double)(t1 - t0) / 1000.0;
	double est_freq = (double)crossings / (2.0 * capture_s);

	/* ── Tolerance checks ── */
	bool ok_p2p = in_tol(p2p, EXP_P2P);
	bool ok_rms = in_tol(rms, EXP_RMS);
	bool ok_crest = in_tol(crest, EXP_CREST);
	bool ok_mean = in_tol(mean, EXP_MEAN);
	bool ok_freq = in_tol(est_freq, EXP_FREQ);
	bool all_pass = ok_p2p && ok_rms && ok_crest && ok_mean && ok_freq;

	int crest_i = (int)(crest * 1000.0);

	if (all_pass) {
		LOG_INF("PASS  p2p=%dmV rms=%dmV crest=%d.%03d freq=%dHz mean=%dmV",
			(int)p2p, (int)rms, crest_i / 1000, crest_i % 1000,
			(int)est_freq, (int)mean);
	} else {
		LOG_ERR("FAIL  p2p=%dmV[%s] rms=%dmV[%s] crest=%d.%03d[%s] freq=%dHz[%s] "
			"mean=%dmV[%s]",
			(int)p2p, ok_p2p ? "ok" : "!", (int)rms, ok_rms ? "ok" : "!",
			crest_i / 1000, crest_i % 1000, ok_crest ? "ok" : "!",
			(int)est_freq, ok_freq ? "ok" : "!", (int)mean, ok_mean ? "ok" : "!");
		LOG_ERR("  expected: p2p=%dmV rms=%dmV crest=1.414 freq=%dHz mean=%dmV "
			"(tol +/-%d%%, %d samples in %d ms)",
			(int)EXP_P2P, (int)EXP_RMS, SINE_FREQ_HZ, (int)EXP_MEAN, TOLERANCE_PCT,
			CAPTURE_SAMPLES, (int)(t1 - t0));
	}
}

/* ── Entry point ─────────────────────────────────────────────────────────── */

int main(void)
{
	int ret;

	if (!device_is_ready(codec_dev)) {
		LOG_ERR("codec device not ready");
		return -ENODEV;
	}
	if (!adc_is_ready_dt(&adc_ch)) {
		LOG_ERR("ADC device not ready");
		return -ENODEV;
	}

	ret = adc_channel_setup_dt(&adc_ch);
	if (ret < 0) {
		LOG_ERR("adc_channel_setup failed: %d", ret);
		return ret;
	}

	build_sine_table();

	struct audio_codec_cfg codec_cfg = {
		.mclk_freq = 0,
		.dai_type = AUDIO_DAI_TYPE_PCM,
		.dai_route = AUDIO_ROUTE_PLAYBACK,
		.dai_cfg.pcm = {
			.dir = AUDIO_DAI_DIR_TX,
			.pcm_width = AUDIO_PCM_WIDTH_16_BITS,
			.channels = 1,
			.block_size = BLOCK_BYTES,
			.samplerate = SAMPLE_RATE,
		},
	};

	ret = audio_codec_configure(codec_dev, &codec_cfg);
	if (ret < 0) {
		LOG_ERR("audio_codec_configure failed: %d", ret);
		return ret;
	}

	ret = audio_codec_register_done_callback(codec_dev, tx_done, NULL, NULL, NULL);
	if (ret < 0) {
		LOG_ERR("audio_codec_register_done_callback failed: %d", ret);
		return ret;
	}

	ret = audio_codec_start(codec_dev, AUDIO_DAI_DIR_TX);
	if (ret < 0) {
		LOG_ERR("audio_codec_start failed: %d", ret);
		return ret;
	}

	LOG_INF("loopback self-test: %d Hz, amp=%d codes, tol=%d%%",
		SINE_FREQ_HZ, SINE_AMPLITUDE, TOLERANCE_PCT);

	while (true) {
		k_sleep(K_SECONDS(1));
		capture_and_check();
	}

	return 0;
}
