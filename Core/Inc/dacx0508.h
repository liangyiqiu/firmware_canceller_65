/**
 * @file    dacx0508.h
 * @author  Leon
 * @version 0.1
 * @date    2023.11.30
 * @brief   ti dacx0508 driver file
 */

#ifndef __DACX0508_H
#define __DACX0508_H

#define DACX0508_WRITE 0x00
#define DACX0508_READ 0x80

#define DACX0508_REG_DEVICE_ID 0x01
#define DACX0508_REG_SYNC 0x02
#define DACX0508_REG_CONFIG 0x03
#define DACX0508_REG_GAIN 0x04
#define DACX0508_REG_TRIGGER 0x05
#define DACX0508_REG_BRDCAST 0x06
#define DACX0508_REG_STATUS 0x07
#define DACX0508_REG_DAC0 0x08
#define DACX0508_REG_DAC1 0x09
#define DACX0508_REG_DAC2 0x0A
#define DACX0508_REG_DAC3 0x0B
#define DACX0508_REG_DAC4 0x0C
#define DACX0508_REG_DAC5 0x0D
#define DACX0508_REG_DAC6 0x0E
#define DACX0508_REG_DAC7 0x0F

#define DACX0508_REG_OFFSET 0x08

#define DACX0508_BRDCAST_ENABLE 0xFF00
#define DACX0508_BRDCAST_DISABLE 0x0000
#define DACX0508_SYNC_ENABLE 0x00FF
#define DACX0508_SYNC_DISABLE 0x0000

#define DACX0508_DAC6_PWDWN 0x0040
#define DACX0508_DAC7_PWDWN 0x0080

#define DACX0508_REFDIV_1 0x0000
#define DACX0508_REFDIV_2 0x0100
#define DACX0508_GAIN_1 0x0000
#define DACX0508_GAIN_2 0x00FF

#define DACX0508_LDAC_TRIG 0x0008
#define DACX0508_SOFT_RESET 0x000a

#include "stm32g4xx_hal.h"

extern void dacx0508_init(SPI_HandleTypeDef* spi_handle);
extern void dacx0508_setvalue(SPI_HandleTypeDef* spi_handle,uint8_t channel,uint16_t data);
extern uint16_t dacx0508_read_id(SPI_HandleTypeDef* spi_handle);
extern void dacx0508_reset(SPI_HandleTypeDef* spi_handle);

#endif
