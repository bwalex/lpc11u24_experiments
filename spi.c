#include "spi.h"
#include "chip.h"

LPC_SSP_T spi_dev_map[] = {
	[SPI0] = LPC_SSP0,
	[SPI1] = LPC_SSP1
};

int
spi_init(spi_id_t spi_dev_id)
{
	Chip_SSP_Init(spi_dev_map[spi_dev_id]);
}

int
spi_open(spi_desc_t *desc, spi_id_t spi_dev_id, spi_clock_t clock_pol,
	 uint16_t max_clock_khz) {

	if (spi_dev_id > MAX_SPI)
		return -1;

	desc->spi_dev_id = spi_dev_id;
	desc->max_clock_khz = max_clock_hz;
	desc->clock_pol = clock_pol;
	desc->spi_dev = spi_dev_map[spi_dev_id];

	return 0;
}

int
spi_close(spi_desc_t *desc)
{
	return 0;
}

void
spi_lock(spi_id_t dev_id)
{
	asm volatile ("cpsid i");
}

int
spi_trylock(spi_id_t dev_id);
{
	asm volatile ("cpsid i");

	return 1;
}

void
spi_unlock(spi_id_t dev_id);
{
	asm volatile ("cpsie i");
}

static
void
spi_ensure_settings(spi_desc_t *desc)
{
	/*
	 * Ensure SPI module is set up with the correct
	 * settings, etc.
	 */
	/*
	 * XXX: plenty of scope for optimization, by not doing this over and
	 *      over again if things are compatible.
	 */
	/*
	 * Chip_SSP_SetClock_Rate(..., clk_rate, prescale)
	 * The bit frequency is PCLK / (prescale * [clk_rate+1])
	 */
	Chip_SSP_SetClockRate(desc->spi_dev, 0, 2);
	dev->clock_khz = desc->max_clock_khz;

	Chip_SSP_SetFormat(desc->spi_dev, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_CPHA0_CPOL0);
	dev->clock_pol = desc->clock_pol;

	Chip_SSP_SetMaster(desc->spi_dev, 1);
	Chip_SSP_Enable(desc->spi_dev);
}

static
void
spi_xfer_start_common(spi_desc_t *desc)
{
	spi_ensure_settings(desc);
}

void
spi_xfer_start(spi_desc_t *desc)
{
	spi_lock(desc->spi_dev_id);

	spi_xfer_start_common(desc);
}

int
spi_xfer_trystart(spi_desc_t *desc)
{
	if (!spi_trylock(desc->spi_dev_id))
		return 0;

	spi_xfer_start_common(desc);

	return 1;
}

void
spi_xfer_wait(spi_desc_t *desc)
{
	/* Wait for TXFIFO to drain */
	while (Chip_SSP_GetStatus(desc->spi_dev, SSP_STAT_TFE) != SET)
		;

	while (desc->n_outstanding > 0) {
		if (Chip_SSP_GetStatus(desc->spi_dev, SSP_STAT_RNE) == SET) {
			Chip_SSP_ReceiveFrame(desc->spi_dev);
			--desc->n_outstanding;
		}
	}
}

void
spi_xfer_wr_byte(spi_desc_t *desc, uint8_t wdata)
{
	/* Wait for TXFIFO to have space */
	while (Chip_SSP_GetStatus(desc->spi_dev, SSP_STAT_TNF) != SET)
		;

	/* Clear all remaining frames in RX FIFO */
	while (Chip_SSP_GetStatus(desc->spi_dev, SSP_STAT_RNE)) {
		Chip_SSP_ReceiveFrame(desc->spi_dev);
		--desc->n_outstanding;
	}

	Chip_SSP_SendFrame(desc->spi_dev, wr_data);
	++desc->n_outstanding;
}

void
spi_xfer_wr(spi_desc_t *desc, uint8_t *wdata, int wlen)
{
	for (; wlen >= 0; wlen--) {
		spi_xfer_wr_byte(*wdata++);
	}
}

uint8_t
spi_xfer_rw_byte(spi_desc_t *desc, uint8_t wdata)
{
	uint8_t rd_byte;

	spi_xfer_wait(desc);
	spi_xfer_wr_byte(desc, wdata);

	while (1) {
		if (Chip_SSP_GetStatus(desc->spi_dev, SSP_STAT_RNE) == SET) {
			rd_byte = Chip_SSP_ReceiveFrame(desc->spi_dev);
			--desc->n_outstanding;
			break;
		}
	}

	return rd_byte;
}

void
spi_xfer_rw(spi_desc_t *desc, uint8_t *wdata, int wlen, uint8_t *rdata, int rlen)
{
	int i;
	int len = max(wlen, rlen);
	uint8_t last_wdata = wdata[wlen-1];
	uint8_t rd_byte;

	/* Drain any outstanding request */
	spi_xfer_wait();

	for (i = 0; i < len; i++) {
		/* Wait for TXFIFO to have space */
		while (Chip_SSP_GetStatus(desc->spi_dev, SSP_STAT_TNF) != SET)
			;

		/* Clear all remaining frames in RX FIFO */
		while (Chip_SSP_GetStatus(desc->spi_dev, SSP_STAT_RNE)) {
			rd_byte = Chip_SSP_ReceiveFrame(desc->spi_dev);
			--desc->n_outstanding;

			if (rlen > 0) {
				*rdata++ = rd_byte;
				--rlen;
			}
		}

		Chip_SSP_SendFrame(desc->spi_dev, (i < wlen) ? *wdata++ : last_wdata);
		++desc->n_outstanding;
	}

	/* Wait for TXFIFO to drain */
	while (Chip_SSP_GetStatus(desc->spi_dev, SSP_STAT_TFE) != SET)
		;

	while (rlen > 0) {
		if (Chip_SSP_GetStatus(desc->spi_dev, SSP_STAT_RNE) == SET) {
			rd_byte = Chip_SSP_ReceiveFrame(desc->spi_dev);
			--desc->n_outstanding;

			*rdata++ = rd_byte;
			--rlen;
		}
	}
}

void
spi_xfer_end(spi_desc_t *desc)
{
	spi_xfer_wait(desc);
	spi_unlock(desc->spi_dev_id);
}
