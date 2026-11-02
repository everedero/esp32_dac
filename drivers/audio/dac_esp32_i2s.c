/*
 * Copyright (c) 2026 Eve Redero
 * SPDX-License-Identifier: Apache-2.0
 *
 * ESP32 built-in DAC audio playback: I2S0 in built-in DAC ("LCD") mode clocks
 * the samples out and its DMA walks a two-descriptor ring (double buffer).
 * Same audio_codec behaviour as the STM32 DAC audio driver (PR #115001).
 */

#define DT_DRV_COMPAT espressif_esp32_dac_audio

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/audio/codec.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <soc.h>
#include <hal/dac_ll.h>
#include <hal/i2s_ll.h>
#include <soc/lldesc.h>

LOG_MODULE_REGISTER(esp32_dac_audio, CONFIG_AUDIO_CODEC_LOG_LEVEL);

#define ESP32_DAC_BYTES_PER_SAMPLE 2
#define ESP32_DAC_PCM_SIGN_BIT     0x8000U
#define ESP32_DAC_SILENCE          0x8000U /* mid-scale, high byte goes to the DAC */
#define ESP32_DAC_SLOT_BITS        16      /* DAC takes the high 8 bits of the slot */
#define ESP32_DAC_SCLK_HZ          I2S_LL_DEFAULT_CLK_FREQ /* PLL_D2, 160 MHz */
#define ESP32_DAC_MCLK_DIV_MIN     2
#define ESP32_DAC_MCLK_DIV_MAX     255
#define ESP32_DAC_BCK_DIV_MAX      63
#define ESP32_DAC_MCLK_FRAC_MAX    63      /* clkm_div_a is 6 bits */

#define ESP32_DAC_SAMPLES_PER_BLOCK                                                                \
	(CONFIG_AUDIO_DAC_ESP32_MAX_BLOCK_SIZE / ESP32_DAC_BYTES_PER_SAMPLE)

BUILD_ASSERT((CONFIG_AUDIO_DAC_ESP32_MAX_BLOCK_SIZE % 4) == 0,
	     "CONFIG_AUDIO_DAC_ESP32_MAX_BLOCK_SIZE must be a multiple of 4");

struct dac_esp32_cfg {
	i2s_dev_t *hw;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	int irq_source;
	int irq_priority;
	int irq_flags;
	const struct device *dac_dev;
	uint8_t dac_channel;
};

struct dac_esp32_data {
	struct audio_codec_cfg config;
	audio_codec_tx_done_callback_t tx_cb;
	void *tx_cb_user_data;
	intr_handle_t irq_handle;
	lldesc_t desc[2] __aligned(4);
	uint16_t dma_buf[2 * ESP32_DAC_SAMPLES_PER_BLOCK] __aligned(4);
	uint16_t *write_buf;
	bool configured;
	bool writable;
	bool started;
};

/*
 * mclk = sclk / (integer + numerator / denominator), WS = mclk / (2 * 16).
 * Returns the real sample rate, 0 if out of range.
 */
static uint32_t dac_esp32_set_clock(i2s_dev_t *hw, uint32_t rate)
{
	/* Low rates: grow the bck divider (6 bits) until the mclk divider fits */
	uint32_t bck_div = ESP32_DAC_SLOT_BITS;

	while ((bck_div < ESP32_DAC_BCK_DIV_MAX) &&
	       (ESP32_DAC_SCLK_HZ / ((uint64_t)rate * I2S_LL_AD_BCK_FACTOR * bck_div) >=
		ESP32_DAC_MCLK_DIV_MAX)) {
		bck_div++;
	}

	uint32_t mclk = rate * I2S_LL_AD_BCK_FACTOR * bck_div;
	uint32_t integer = ESP32_DAC_SCLK_HZ / mclk;
	uint32_t rem = ESP32_DAC_SCLK_HZ % mclk;
	uint32_t best_a = 1, best_b = 0;
	uint64_t best_err = rem;

	if ((integer < ESP32_DAC_MCLK_DIV_MIN) || (integer >= ESP32_DAC_MCLK_DIV_MAX)) {
		return 0;
	}

	/* ponytail: brute force over 63 denominators, it runs once per configure */
	for (uint32_t a = 2; (a <= ESP32_DAC_MCLK_FRAC_MAX) && (best_err != 0); a++) {
		uint32_t b = (uint32_t)(((uint64_t)rem * a + mclk / 2) / mclk);

		if (b >= a) {
			continue;
		}

		/* |rem/mclk - b/a| scaled by mclk * a */
		int64_t diff = (int64_t)rem * a - (int64_t)b * mclk;
		uint64_t err = (uint64_t)((diff < 0) ? -diff : diff) / a;

		if (err < best_err) {
			best_err = err;
			best_a = a;
			best_b = b;
		}
	}

	hal_utils_clk_div_t div = {
		.integer = integer,
		.denominator = best_a,
		.numerator = best_b,
	};

	i2s_ll_tx_clk_set_src(hw, I2S_CLK_SRC_DEFAULT);
	i2s_ll_tx_set_mclk(hw, &div);
	i2s_ll_tx_set_bck_div_num(hw, bck_div);

	uint64_t real_mclk_x_a = (uint64_t)ESP32_DAC_SCLK_HZ * best_a /
				 ((uint64_t)integer * best_a + best_b);

	return (uint32_t)(real_mclk_x_a / (I2S_LL_AD_BCK_FACTOR * bck_div));
}

static int dac_esp32_configure(const struct device *dev, struct audio_codec_cfg *cfg)
{
	const struct dac_esp32_cfg *dev_cfg = dev->config;
	struct dac_esp32_data *dev_data = dev->data;
	i2s_dev_t *hw = dev_cfg->hw;

	if (cfg == NULL) {
		LOG_ERR("configure: no config");
		return -EINVAL;
	}

	if (dev_data->started) {
		LOG_ERR("configure: output running");
		return -EBUSY;
	}

	if ((cfg->dai_type != AUDIO_DAI_TYPE_PCM) ||
	    (cfg->dai_cfg.pcm.pcm_width != AUDIO_PCM_WIDTH_16_BITS) ||
	    (cfg->dai_cfg.pcm.channels != 1)) {
		LOG_ERR("configure: need mono 16-bit PCM (dai %d, width %d, ch %" PRIu8 ")",
			cfg->dai_type, cfg->dai_cfg.pcm.pcm_width, cfg->dai_cfg.pcm.channels);
		return -ENOTSUP;
	}

	if ((cfg->dai_cfg.pcm.block_size == 0) || (cfg->dai_cfg.pcm.block_size % 4 != 0) ||
	    (cfg->dai_cfg.pcm.block_size > CONFIG_AUDIO_DAC_ESP32_MAX_BLOCK_SIZE)) {
		LOG_ERR("configure: bad block size %zu (max %d, multiple of 4)",
			cfg->dai_cfg.pcm.block_size, CONFIG_AUDIO_DAC_ESP32_MAX_BLOCK_SIZE);
		return -EINVAL;
	}

	uint32_t rate = cfg->dai_cfg.pcm.samplerate;
	uint32_t real_rate = (rate == 0U) ? 0U : dac_esp32_set_clock(hw, rate);

	if (real_rate == 0U) {
		LOG_ERR("configure: samplerate %" PRIu32 " out of range (~5 kHz .. 2.5 MHz)",
			rate);
		return -EINVAL;
	}

	/* Same register sequence as ESP-IDF esp_driver_dac/esp32/dac_dma.c */
	i2s_ll_enable_builtin_adc_dac(hw, true);
	i2s_ll_tx_reset(hw);
	i2s_ll_tx_set_slave_mod(hw, false);
	i2s_ll_tx_set_sample_bit(hw, ESP32_DAC_SLOT_BITS, ESP32_DAC_SLOT_BITS);
	i2s_ll_tx_enable_mono_mode(hw, true);
	i2s_ll_tx_select_std_slot(hw, I2S_STD_SLOT_BOTH, true);
	i2s_ll_tx_enable_msb_shift(hw, false);
	i2s_ll_tx_set_ws_width(hw, ESP32_DAC_SLOT_BITS);
	i2s_ll_tx_enable_msb_right(hw, false);
	i2s_ll_tx_enable_right_first(hw, true);
	i2s_ll_tx_force_enable_fifo_mod(hw, true);
	i2s_ll_dma_enable_owner_check(hw, false);

	dev_data->config = *cfg;
	dev_data->configured = true;

	LOG_INF("configured: %" PRIu32 " Hz (asked %" PRIu32 "), block size %zu", real_rate,
		rate, cfg->dai_cfg.pcm.block_size);

	return 0;
}

/* Hand one half of the ring to the user: point write_buf at it, call tx_cb */
static void dac_esp32_offer(const struct device *dev, size_t half)
{
	struct dac_esp32_data *dev_data = dev->data;

	dev_data->write_buf = &dev_data->dma_buf[half * ESP32_DAC_SAMPLES_PER_BLOCK];
	dev_data->writable = true;

	if (dev_data->tx_cb != NULL) {
		dev_data->tx_cb(dev, dev_data->tx_cb_user_data);
	}
}

static void dac_esp32_start_output(const struct device *dev)
{
	const struct dac_esp32_cfg *dev_cfg = dev->config;
	struct dac_esp32_data *dev_data = dev->data;
	i2s_dev_t *hw = dev_cfg->hw;
	size_t block = dev_data->config.dai_cfg.pcm.block_size;

	if (!dev_data->configured) {
		LOG_ERR("start: not configured");
		return;
	}

	if (dev_data->started) {
		LOG_DBG("start: already running");
		return;
	}

	for (size_t i = 0; i < ARRAY_SIZE(dev_data->dma_buf); i++) {
		dev_data->dma_buf[i] = ESP32_DAC_SILENCE;
	}

	/* Ring A -> B -> A; EOF on each descriptor raises out_eof */
	for (size_t i = 0; i < 2; i++) {
		lldesc_t *d = &dev_data->desc[i];

		d->size = block;
		d->length = block;
		d->offset = 0;
		d->sosf = 0;
		d->eof = 1;
		d->owner = 1;
		d->buf = (const uint8_t *)&dev_data->dma_buf[i * ESP32_DAC_SAMPLES_PER_BLOCK];
		d->empty = (uint32_t)&dev_data->desc[i ^ 1];
	}

	/* Let the user fill both halves before the DMA reads them */
	dev_data->started = true;
	if (dev_data->tx_cb == NULL) {
		LOG_WRN("start: no tx callback registered, output stays silent");
	}
	dac_esp32_offer(dev, 0);
	dac_esp32_offer(dev, 1);
	dev_data->writable = false;

	i2s_ll_tx_reset(hw);
	i2s_ll_tx_reset_dma(hw);
	i2s_ll_tx_reset_fifo(hw);
	i2s_ll_clear_intr_status(hw, I2S_LL_TX_EVENT_MASK);

	dac_ll_digi_enable_dma(true);

	i2s_ll_enable_dma(hw, true);
	i2s_ll_enable_intr(hw, I2S_LL_EVENT_TX_EOF, true);
	i2s_ll_dma_enable_eof_on_fifo_empty(hw, true);
	i2s_ll_tx_start_link(hw, (uint32_t)&dev_data->desc[0]);
	i2s_ll_tx_start(hw);

	esp_intr_enable(dev_data->irq_handle);
}

static void dac_esp32_stop_output(const struct device *dev)
{
	const struct dac_esp32_cfg *dev_cfg = dev->config;
	struct dac_esp32_data *dev_data = dev->data;
	i2s_dev_t *hw = dev_cfg->hw;

	if (!dev_data->started) {
		LOG_DBG("stop: already stopped");
		return;
	}

	esp_intr_disable(dev_data->irq_handle);
	i2s_ll_tx_stop(hw);
	i2s_ll_tx_stop_link(hw);
	i2s_ll_enable_intr(hw, I2S_LL_EVENT_TX_EOF, false);
	i2s_ll_enable_dma(hw, false);
	dac_ll_digi_enable_dma(false);

	dev_data->started = false;
	dev_data->writable = false;
}

static int dac_esp32_set_property(const struct device *dev, audio_property_t property,
				  audio_channel_t channel, audio_property_value_t val)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(val);

	LOG_WRN("set_property: property %d (ch %d) ignored, none supported", property, channel);

	return 0;
}

static int dac_esp32_apply_properties(const struct device *dev)
{
	ARG_UNUSED(dev);

	return -ENOTSUP;
}

static int dac_esp32_start(const struct device *dev, uint8_t dir)
{
	if (dir != AUDIO_DAI_DIR_TX) {
		LOG_ERR("start: dir %" PRIu8 " not supported, TX only", dir);
		return -ENOTSUP;
	}

	dac_esp32_start_output(dev);

	return 0;
}

static int dac_esp32_stop(const struct device *dev, uint8_t dir)
{
	if (dir != AUDIO_DAI_DIR_TX) {
		LOG_ERR("stop: dir %" PRIu8 " not supported, TX only", dir);
		return -ENOTSUP;
	}

	dac_esp32_stop_output(dev);

	return 0;
}

static int dac_esp32_write(const struct device *dev, uint8_t *data, size_t data_size)
{
	struct dac_esp32_data *dev_data = dev->data;

	if ((data == NULL) || (data_size == 0U) || (data_size % ESP32_DAC_BYTES_PER_SAMPLE != 0) ||
	    (data_size > dev_data->config.dai_cfg.pcm.block_size)) {
		LOG_ERR("write: bad data %p size %zu (block %zu)", (void *)data, data_size,
			dev_data->config.dai_cfg.pcm.block_size);
		return -EINVAL;
	}

	if (!dev_data->started) {
		LOG_ERR("write: output not started");
		return -EIO;
	}

	if (!dev_data->writable) {
		LOG_WRN_RATELIMIT("write: no free buffer, caller too early");
		return -EBUSY;
	}

	uint16_t *dst = dev_data->write_buf;
	size_t src_samples = data_size / ESP32_DAC_BYTES_PER_SAMPLE;
	size_t dst_samples = dev_data->config.dai_cfg.pcm.block_size / ESP32_DAC_BYTES_PER_SAMPLE;

	/* signed -> offset binary; the DAC only keeps the high byte */
	for (size_t i = 0; i < src_samples; i++) {
		dst[i] = (sys_get_le16(&data[2 * i]) ^ ESP32_DAC_PCM_SIGN_BIT) & 0xFF00U;
	}

	for (size_t i = src_samples; i < dst_samples; i++) {
		dst[i] = ESP32_DAC_SILENCE;
	}

	dev_data->writable = false;

	return 0;
}

static int dac_esp32_register_done_callback(const struct device *dev,
					    audio_codec_tx_done_callback_t tx_cb,
					    void *tx_cb_user_data,
					    audio_codec_rx_done_callback_t rx_cb,
					    void *rx_cb_user_data)
{
	ARG_UNUSED(rx_cb);
	ARG_UNUSED(rx_cb_user_data);

	struct dac_esp32_data *dev_data = dev->data;

	dev_data->tx_cb = tx_cb;
	dev_data->tx_cb_user_data = tx_cb_user_data;

	return 0;
}

static void dac_esp32_isr(void *arg)
{
	const struct device *dev = arg;
	const struct dac_esp32_cfg *dev_cfg = dev->config;
	struct dac_esp32_data *dev_data = dev->data;
	i2s_dev_t *hw = dev_cfg->hw;
	uint32_t status = i2s_ll_get_intr_status(hw);

	i2s_ll_clear_intr_status(hw, status);

	if (!(status & I2S_LL_EVENT_TX_EOF) || !dev_data->started) {
		return;
	}

	uint32_t done;

	i2s_ll_tx_get_eof_des_addr(hw, &done);

	if (dev_data->writable) {
		LOG_WRN_RATELIMIT("underrun: block not written in time, stale samples played");
	}

	/* The finished half is free while the DMA plays the other one */
	dac_esp32_offer(dev, (done == (uint32_t)&dev_data->desc[0]) ? 0 : 1);
}

static int dac_esp32_init(const struct device *dev)
{
	const struct dac_esp32_cfg *dev_cfg = dev->config;
	struct dac_esp32_data *dev_data = dev->data;
	int ret;

	if (!device_is_ready(dev_cfg->clock_dev) || !device_is_ready(dev_cfg->dac_dev)) {
		LOG_ERR("init: clock or DAC device not ready");
		return -ENODEV;
	}

	ret = clock_control_on(dev_cfg->clock_dev, dev_cfg->clock_subsys);
	if (ret < 0) {
		LOG_ERR("init: I2S clock on failed (%d)", ret);
		return ret;
	}
	i2s_ll_enable_core_clock(dev_cfg->hw, true);

	/* Powers the DAC pad and its SAR clock (Zephyr DAC driver) */
	const struct dac_channel_cfg ch_cfg = {
		.channel_id = dev_cfg->dac_channel,
		.resolution = 8,
	};

	ret = dac_channel_setup(dev_cfg->dac_dev, &ch_cfg);
	if (ret < 0) {
		LOG_ERR("init: DAC channel %u setup failed (%d)", dev_cfg->dac_channel, ret);
		return ret;
	}

	/* Otherwise ADC1 RTC sampling disturbs the DAC output */
	dac_ll_rtc_sync_by_adc(false);

	ret = esp_intr_alloc(dev_cfg->irq_source,
			     ESP_PRIO_TO_FLAGS(dev_cfg->irq_priority) |
				     ESP_INT_FLAGS_CHECK(dev_cfg->irq_flags) |
				     ESP_INTR_FLAG_INTRDISABLED,
			     dac_esp32_isr, (void *)dev, &dev_data->irq_handle);
	if (ret != 0) {
		LOG_ERR("init: interrupt alloc failed (%d)", ret);
		return -EIO;
	}

	return 0;
}

static DEVICE_API(audio_codec, dac_esp32_api) = {
	.configure = dac_esp32_configure,
	.start_output = dac_esp32_start_output,
	.stop_output = dac_esp32_stop_output,
	.set_property = dac_esp32_set_property,
	.apply_properties = dac_esp32_apply_properties,
	.start = dac_esp32_start,
	.stop = dac_esp32_stop,
	.write = dac_esp32_write,
	.register_done_callback = dac_esp32_register_done_callback,
};

#define I2S_NODE(inst) DT_INST_PHANDLE(inst, espressif_i2s)

#define ESP32_DAC_DEFINE(inst)                                                                     \
	BUILD_ASSERT(DT_PROP(I2S_NODE(inst), unit) == 0, "only i2s0 drives the built-in DAC");    \
	BUILD_ASSERT(!DT_NODE_HAS_STATUS_OKAY(I2S_NODE(inst)),                                     \
		     "i2s0 must be disabled: the DAC audio driver owns it");                       \
	static const struct dac_esp32_cfg dac_esp32_cfg_##inst = {                                 \
		.hw = I2S_LL_GET_HW(0),                                                            \
		.clock_dev = DEVICE_DT_GET(DT_CLOCKS_CTLR(I2S_NODE(inst))),                        \
		.clock_subsys = (clock_control_subsys_t)DT_CLOCKS_CELL(I2S_NODE(inst), offset),    \
		.irq_source = DT_IRQ_BY_NAME(I2S_NODE(inst), tx, irq),                             \
		.irq_priority = DT_IRQ_BY_NAME(I2S_NODE(inst), tx, priority),                      \
		.irq_flags = DT_IRQ_BY_NAME(I2S_NODE(inst), tx, flags),                            \
		.dac_dev = DEVICE_DT_GET(DT_INST_IO_CHANNELS_CTLR_BY_IDX(inst, 0)),                \
		.dac_channel = DT_INST_IO_CHANNELS_OUTPUT_BY_IDX(inst, 0),                         \
	};                                                                                         \
	static struct dac_esp32_data dac_esp32_data_##inst;                                        \
	DEVICE_DT_INST_DEFINE(inst, dac_esp32_init, NULL, &dac_esp32_data_##inst,                  \
			      &dac_esp32_cfg_##inst, POST_KERNEL,                                  \
			      CONFIG_AUDIO_CODEC_INIT_PRIORITY, &dac_esp32_api)

DT_INST_FOREACH_STATUS_OKAY(ESP32_DAC_DEFINE)
