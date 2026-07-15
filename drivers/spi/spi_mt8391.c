/*
 * Copyright (c) 2026 Proventus Nova
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * PIO/polling driver for the MediaTek IPM-generation SPI controller
 * (MT8391 / Genio 720; Linux reference drivers/spi/spi-mt65xx.c with
 * the mt8189 compatible data: ipm_design + enhance_timing + need_pad_sel).
 *
 * Chip select is expected to be a GPIO (cs-gpios): the controller's
 * internal CS output is not routed to the pad on this board, so
 * transfers longer than the 32-byte FIFO are streamed as back-to-back
 * hardware transactions while the GPIO holds CS asserted.
 */

#define DT_DRV_COMPAT mediatek_mt8391_spi

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(spi_mt8391, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"

#define SPI_CFG0_REG     0x0000
#define SPI_CFG1_REG     0x0004
#define SPI_TX_SRC_REG   0x0008
#define SPI_RX_DST_REG   0x000c
#define SPI_TX_DATA_REG  0x0010
#define SPI_RX_DATA_REG  0x0014
#define SPI_CMD_REG      0x0018
#define SPI_STATUS0_REG  0x001c
#define SPI_PAD_SEL_REG  0x0024
#define SPI_CFG2_REG     0x0028
#define SPI_CFG3_IPM_REG 0x0040

#define SPI_CFG1_CS_IDLE_OFFSET         0
#define SPI_CFG1_PACKET_LOOP_OFFSET     8
#define SPI_CFG1_PACKET_LENGTH_OFFSET   16
#define SPI_CFG1_PACKET_LOOP_MASK       0xff00
#define SPI_CFG1_IPM_PACKET_LENGTH_MASK GENMASK(31, 16)

#define SPI_CFG2_SCK_HIGH_OFFSET 0
#define SPI_CFG2_SCK_LOW_OFFSET  16

#define SPI_CMD_ACT         BIT(0)
#define SPI_CMD_RESUME      BIT(1)
#define SPI_CMD_RST         BIT(2)
#define SPI_CMD_PAUSE_EN    BIT(4)
#define SPI_CMD_DEASSERT    BIT(5)
#define SPI_CMD_SAMPLE_SEL  BIT(6)
#define SPI_CMD_CS_POL      BIT(7)
#define SPI_CMD_CPHA        BIT(8)
#define SPI_CMD_CPOL        BIT(9)
#define SPI_CMD_RX_DMA      BIT(10)
#define SPI_CMD_TX_DMA      BIT(11)
#define SPI_CMD_TXMSBF      BIT(12)
#define SPI_CMD_RXMSBF      BIT(13)
#define SPI_CMD_RX_ENDIAN   BIT(14)
#define SPI_CMD_TX_ENDIAN   BIT(15)
#define SPI_CMD_FINISH_IE   BIT(16)
#define SPI_CMD_PAUSE_IE    BIT(17)
#define SPI_CMD_IPM_NONIDLE BIT(19)

#define SPI_STATUS_FINISH BIT(0)
#define SPI_STATUS_PAUSE  BIT(1)

#define MTK_SPI_FIFO_SIZE 32U

/* Generous per-chunk completion budget: 32 bytes at the slowest
 * plausible SCK (~100 kHz) is under 3 ms; allow 100 ms before
 * declaring the controller wedged.
 */
#define MTK_SPI_POLL_TIMEOUT_US 100000

struct spi_mt8391_config {
	DEVICE_MMIO_ROM;
	const struct pinctrl_dev_config *pincfg;
	void (*irq_config_func)(const struct device *dev);
	uint32_t clock_frequency;
	uint32_t pad_select;
};

struct spi_mt8391_data {
	DEVICE_MMIO_RAM;
	struct spi_context ctx;
	struct k_sem done_sem;
	volatile bool awaiting;
};

/* Completion is signaled through the controller interrupt: the ISR is
 * the only reader of the read-to-clear status register, which closes
 * the races a polled model has when FINISH and PAUSE latch with skew
 * (an early-consumed latch makes the caller reprogram CFG1 while the
 * engine still runs, wedging it). Spurious or late latches hit an ISR
 * with no waiter and are cleared harmlessly.
 */
static void mt8391_spi_isr(const struct device *dev)
{
	struct spi_mt8391_data *data = dev->data;
	uint32_t status;

	status = sys_read32(DEVICE_MMIO_GET(dev) + SPI_STATUS0_REG);
	if ((status & (SPI_STATUS_FINISH | SPI_STATUS_PAUSE)) != 0U && data->awaiting) {
		data->awaiting = false;
		k_sem_give(&data->done_sem);
	}
}

static void mt8391_spi_reset(const struct device *dev)
{
	uint32_t reg;

	reg = sys_read32(DEVICE_MMIO_GET(dev) + SPI_CMD_REG);
	reg |= SPI_CMD_RST;
	sys_write32(reg, DEVICE_MMIO_GET(dev) + SPI_CMD_REG);
	reg &= ~SPI_CMD_RST;
	sys_write32(reg, DEVICE_MMIO_GET(dev) + SPI_CMD_REG);
}

static int mt8391_spi_configure(const struct device *dev, const struct spi_config *config)
{
	const struct spi_mt8391_config *cfg = dev->config;
	struct spi_mt8391_data *data = dev->data;
	uint32_t freq = config->frequency;
	uint32_t div, sck_time, reg;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (SPI_OP_MODE_GET(config->operation) != SPI_OP_MODE_MASTER) {
		LOG_ERR("Only controller mode is supported");
		return -ENOTSUP;
	}
	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		LOG_ERR("Only 8-bit words are supported");
		return -ENOTSUP;
	}
	if ((config->operation & SPI_TRANSFER_LSB) != 0U) {
		LOG_ERR("LSB-first is not supported");
		return -ENOTSUP;
	}
	if (freq == 0U) {
		return -EINVAL;
	}

	/* SCK divider (Linux mtk_spi_prepare_transfer): the SCK period is
	 * (sck_high + 1) + (sck_low + 1) kernel clock ticks.
	 */
	if (freq < (cfg->clock_frequency / 2U)) {
		div = DIV_ROUND_UP(cfg->clock_frequency, freq);
	} else {
		div = 1U;
	}
	sck_time = (div + 1U) / 2U;
	if (sck_time == 0U) {
		sck_time = 1U;
	}
	reg = ((sck_time - 1U) & 0xffffU) << SPI_CFG2_SCK_HIGH_OFFSET;
	reg |= ((sck_time - 1U) & 0xffffU) << SPI_CFG2_SCK_LOW_OFFSET;
	sys_write32(reg, DEVICE_MMIO_GET(dev) + SPI_CFG2_REG);

	/* Command register, written whole for a clean slate: full-duplex
	 * PIO, MSB-first bit order, little-endian FIFO byte order,
	 * completion status latching enabled. The controller's own CS
	 * timing and polarity are irrelevant (the CS pad is a GPIO).
	 */
	reg = SPI_CMD_IPM_NONIDLE | SPI_CMD_FINISH_IE | SPI_CMD_PAUSE_IE |
	      SPI_CMD_TXMSBF | SPI_CMD_RXMSBF;
	if ((config->operation & SPI_MODE_CPHA) != 0U) {
		reg |= SPI_CMD_CPHA;
	}
	if ((config->operation & SPI_MODE_CPOL) != 0U) {
		reg |= SPI_CMD_CPOL;
	}
	sys_write32(reg, DEVICE_MMIO_GET(dev) + SPI_CMD_REG);

	/* Plain full-duplex transfers: no command/address phases. */
	sys_write32(0, DEVICE_MMIO_GET(dev) + SPI_CFG3_IPM_REG);

	data->ctx.config = config;

	return 0;
}

static int mt8391_spi_wait_chunk(const struct device *dev)
{
	struct spi_mt8391_data *data = dev->data;

	if (k_sem_take(&data->done_sem, K_USEC(MTK_SPI_POLL_TIMEOUT_US)) != 0) {
		uint32_t status = sys_read32(DEVICE_MMIO_GET(dev) + SPI_STATUS0_REG);

		data->awaiting = false;
		LOG_ERR("Completion IRQ timed out (STATUS0 now 0x%02x)", status);
		return -ETIMEDOUT;
	}

	return 0;
}

static void mt8391_spi_setup_chunk(const struct device *dev, uint32_t len)
{
	uint32_t reg;

	reg = sys_read32(DEVICE_MMIO_GET(dev) + SPI_CFG1_REG);
	reg &= ~(SPI_CFG1_IPM_PACKET_LENGTH_MASK | SPI_CFG1_PACKET_LOOP_MASK);
	/* One packet of len bytes (fields hold value minus one). */
	reg |= (len - 1U) << SPI_CFG1_PACKET_LENGTH_OFFSET;
	sys_write32(reg, DEVICE_MMIO_GET(dev) + SPI_CFG1_REG);
}

/* Stream one chunk (<= FIFO size) full duplex through the FIFOs.
 * TX bytes without backing buffer content are sent as 0x00 filler;
 * RX bytes without a destination are discarded. NULL scatter segments
 * still advance the context so later segments stay aligned.
 */
static int mt8391_spi_chunk(const struct device *dev, uint32_t len)
{
	struct spi_mt8391_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t nwords = DIV_ROUND_UP(len, 4U);
	uint32_t word, i, b;
	uint32_t reg;
	int ret;

	mt8391_spi_setup_chunk(dev, len);

	/* Drain any stale completion status, then arm the ISR waiter
	 * before starting the engine.
	 */
	(void)sys_read32(DEVICE_MMIO_GET(dev) + SPI_STATUS0_REG);
	k_sem_reset(&data->done_sem);
	data->awaiting = true;

	for (i = 0; i < nwords; i++) {
		word = 0U;
		for (b = 0; b < 4U; b++) {
			uint32_t idx = (i * 4U) + b;

			if (idx >= len) {
				break;
			}
			if (spi_context_tx_on(ctx)) {
				if (ctx->tx_buf != NULL) {
					word |= (uint32_t)(*ctx->tx_buf) << (8U * b);
				}
				spi_context_update_tx(ctx, 1, 1);
			}
		}
		sys_write32(word, DEVICE_MMIO_GET(dev) + SPI_TX_DATA_REG);
	}

	reg = sys_read32(DEVICE_MMIO_GET(dev) + SPI_CMD_REG);
	reg |= SPI_CMD_ACT;
	sys_write32(reg, DEVICE_MMIO_GET(dev) + SPI_CMD_REG);

	ret = mt8391_spi_wait_chunk(dev);
	if (ret != 0) {
		LOG_ERR("Transfer chunk failed (len %u)", len);
		return ret;
	}

	for (i = 0; i < nwords; i++) {
		word = sys_read32(DEVICE_MMIO_GET(dev) + SPI_RX_DATA_REG);
		for (b = 0; b < 4U; b++) {
			uint32_t idx = (i * 4U) + b;

			if (idx >= len) {
				break;
			}
			if (spi_context_rx_on(ctx)) {
				if (ctx->rx_buf != NULL) {
					*ctx->rx_buf = (uint8_t)((word >> (8U * b)) & 0xffU);
				}
				spi_context_update_rx(ctx, 1, 1);
			}
		}
	}

	return 0;
}

static int mt8391_spi_transceive(const struct device *dev, const struct spi_config *config,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	struct spi_mt8391_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	uint32_t total_tx = 0, total_rx = 0, remaining, chunk;
	size_t i;
	int ret;

	spi_context_lock(ctx, false, NULL, NULL, config);

	ret = mt8391_spi_configure(dev, config);
	if (ret != 0) {
		goto out;
	}

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);

	if (tx_bufs != NULL) {
		for (i = 0; i < tx_bufs->count; i++) {
			total_tx += tx_bufs->buffers[i].len;
		}
	}
	if (rx_bufs != NULL) {
		for (i = 0; i < rx_bufs->count; i++) {
			total_rx += rx_bufs->buffers[i].len;
		}
	}
	remaining = MAX(total_tx, total_rx);
	if (remaining == 0U) {
		goto out;
	}

	spi_context_cs_control(ctx, true);

	while (remaining > 0U) {
		chunk = MIN(remaining, MTK_SPI_FIFO_SIZE);
		ret = mt8391_spi_chunk(dev, chunk);
		if (ret != 0) {
			/* Wedge recovery: reset the state machine and force a
			 * full reconfigure on the next transfer.
			 */
			mt8391_spi_reset(dev);
			data->ctx.config = NULL;
			break;
		}
		remaining -= chunk;
	}

	spi_context_cs_control(ctx, false);

out:
	spi_context_release(ctx, ret);

	return ret;
}

static int mt8391_spi_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_mt8391_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static DEVICE_API(spi, mt8391_spi_api) = {
	.transceive = mt8391_spi_transceive,
	.release = mt8391_spi_release,
};

static int mt8391_spi_init(const struct device *dev)
{
	const struct spi_mt8391_config *cfg = dev->config;
	struct spi_mt8391_data *data = dev->data;
	int ret;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		return ret;
	}

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		return ret;
	}

	k_sem_init(&data->done_sem, 0, 1);
	cfg->irq_config_func(dev);

	sys_write32(cfg->pad_select, DEVICE_MMIO_GET(dev) + SPI_PAD_SEL_REG);
	mt8391_spi_reset(dev);

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define SPI_MT8391_INIT(n)                                                                         \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void spi_mt8391_irq_config_##n(const struct device *dev)                            \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), mt8391_spi_isr,             \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
	static struct spi_mt8391_data spi_mt8391_data_##n = {                                      \
		SPI_CONTEXT_INIT_LOCK(spi_mt8391_data_##n, ctx),                                   \
		SPI_CONTEXT_INIT_SYNC(spi_mt8391_data_##n, ctx),                                   \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)                               \
	};                                                                                         \
	static const struct spi_mt8391_config spi_mt8391_config_##n = {                            \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.irq_config_func = spi_mt8391_irq_config_##n,                                      \
		.clock_frequency = DT_INST_PROP(n, clock_frequency),                               \
		.pad_select = DT_INST_PROP(n, mediatek_pad_select),                                \
	};                                                                                         \
	SPI_DEVICE_DT_INST_DEFINE(n, mt8391_spi_init, NULL, &spi_mt8391_data_##n,                  \
				  &spi_mt8391_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,   \
				  &mt8391_spi_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_MT8391_INIT)
