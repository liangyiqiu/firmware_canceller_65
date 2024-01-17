/**
 * @file    sic_control.c
 * @author  Leon
 * @version 0.1
 * @date    2023.11.30
 * @brief   in band full duplex analog canceller control
 */


#include "sic_control.h"

void sic_on(void)
{
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_8,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_SET);
}

void sic_off(void)
{
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_8,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_RESET);
}

void sic_update_tap(tap_handle_type_def htap)
{
    HAL_DAC_SetValue(htap.att_dac_handle,htap.att_dac_channel,DAC_ALIGN_12B_R,htap.att_data);
    dacx0508_setvalue(htap.ps_dac_spi_handle,htap.ps_dac_channel,htap.ps_data<<4); //dac60508 12bit
}

void sic_tap_on(void)
{
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_8,GPIO_PIN_RESET);
}

void sic_tap_off(void)
{
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_8,GPIO_PIN_SET);
}

void sic_amp_on(void)
{
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_SET);
}

void sic_amp_off(void)
{
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_RESET);
}
