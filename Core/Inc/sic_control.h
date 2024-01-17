/**
 * @file    sic_control.h
 * @author  Leon
 * @version 0.1
 * @date    2023.11.30
 * @brief   in band full duplex analog canceller control
 */

#ifndef __SIC_CONTROL_H
#define __SIC_CONTROL_H

#include "stm32g4xx_hal.h"
#include "dacx0508.h"

typedef struct 
{
    uint16_t att_data;
    uint16_t ps_data;
    uint16_t delay_data;
    DAC_HandleTypeDef*  att_dac_handle;
    uint32_t att_dac_channel;
    uint8_t att_slope;
    SPI_HandleTypeDef* ps_dac_spi_handle;
    uint8_t ps_dac_channel;
}tap_handle_type_def;


extern void sic_on(void);
extern void sic_off(void);
extern void sic_tap_on(void);
extern void sic_tap_off(void);
extern void sic_amp_on(void);
extern void sic_amp_off(void);
extern void sic_update_tap(tap_handle_type_def htap);

#endif
