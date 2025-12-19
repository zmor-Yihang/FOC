#ifndef __SPI_H__
#define __SPI_H__

#include "stm32g4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

void spi_init(void);

#endif /* __SPI_H__ */
