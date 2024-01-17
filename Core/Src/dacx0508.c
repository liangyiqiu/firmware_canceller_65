/**
 * @file    dacx0508.h
 * @author  Leon
 * @version 0.1
 * @date    2023.11.30
 * @brief   ti dacx0508 driver file
 */

#include "dacx0508.h"

void dacx0508_transmit(SPI_HandleTypeDef* spi_handle,uint8_t reg,uint16_t data)
{
    uint8_t spi_buffer[3]={0,0,0};
    spi_buffer[0]=DACX0508_WRITE|reg;
    spi_buffer[1]=(uint8_t)(data>>8);
    spi_buffer[2]=(uint8_t)data;
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi_handle,spi_buffer,3,1000);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
}

uint16_t dacx0508_receive(SPI_HandleTypeDef* spi_handle,uint8_t reg)
{
    uint8_t spi_buffer[3]={0,0,0};
    uint8_t rx_buffer[3]={0,0,0};

    spi_buffer[0]=DACX0508_READ|reg;

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(spi_handle,spi_buffer,rx_buffer,3,1000);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);

    return rx_buffer[1]<<8|rx_buffer[2];
}

void dacx0508_init(SPI_HandleTypeDef* spi_handle)
{   
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
    dacx0508_transmit(spi_handle,DACX0508_REG_SYNC,DACX0508_BRDCAST_DISABLE|DACX0508_SYNC_DISABLE);
    dacx0508_transmit(spi_handle,DACX0508_REG_CONFIG,DACX0508_DAC6_PWDWN|DACX0508_DAC7_PWDWN);
    dacx0508_transmit(spi_handle,DACX0508_REG_GAIN,DACX0508_REFDIV_1|DACX0508_GAIN_1);
}

void dacx0508_setvalue(SPI_HandleTypeDef* spi_handle,uint8_t channel,uint16_t data)
{
    dacx0508_transmit(spi_handle,DACX0508_REG_OFFSET+channel,data);
}

uint16_t dacx0508_read_id(SPI_HandleTypeDef* spi_handle)
{   
    return dacx0508_receive(spi_handle,DACX0508_REG_DEVICE_ID);
}

void dacx0508_reset(SPI_HandleTypeDef* spi_handle)
{
    dacx0508_transmit(spi_handle,DACX0508_REG_TRIGGER,DACX0508_SOFT_RESET);
}
