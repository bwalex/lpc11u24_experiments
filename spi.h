#ifndef _SPI_H
#define _SPI_H

#define SPI0	0
#define SPI1	1
#define SPI2	2
#define SPI3	3

#define MAX_SPI	1

#include <stdint.h>
#include <unistd.h>

typedef uint8_t spi_id_t;

typedef enum {
	SPI_CPOL0_CPHA0,
	SPI_CPOL0_CPHA1,
	SPI_CPOL1_CPHA0,
	SPI_CPOL1_CPHA1
} spi_clock_t;

typedef struct spi_dev_info {
	uint16_t	clock_khz;
	spi_clock_t	clock_pol;
} spi_dev_info_t;


typedef struct spi_desc {
	spi_id_t	spi_dev_id;
	uint16_t	max_clock_khz;
	spi_clock_t	clock_pol;

	size_t		n_outstanding;

	LPC_SSP_T	*spi_dev;
} spi_desc_t;

#ifdef __cplusplus
extern "C" {
#endif

int spi_init(spi_id_t spi_dev_id);

int spi_open(spi_desc_t *desc, spi_id_t spi_dev_id, spi_clock_t clock_pol,
	     uint16_t max_clock_khz);
int spi_close(spi_desc_t *desc);
void spi_xfer_start(spi_desc_t *desc);
int spi_xfer_trystart(spi_desc_t *desc);

void spi_xfer_wait(spi_desc_t *desc);
void spi_xfer_wr_byte(spi_desc_t *desc, uint8_t wdata);
uint8_t spi_xfer_rw_byte(spi_desc_t *desc, uint8_t wdata);
void spi_xfer_wr(spi_desc_t *desc, uint8_t *wdata, int wlen);
void spi_xfer_rw(spi_desc_t *desc, uint8_t *wdata, int wlen, uint8_t *rdata, int rlen);

void spi_xfer_end(spi_desc_t *desc);

#ifdef __cplusplus
}
#endif

#endif
