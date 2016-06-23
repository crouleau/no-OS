/***************************************************************************//**
 *   @file   Platform.c
 *   @brief  Implementation of Platform Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "stdint.h"
#include "util.h"
#include "platform.h"
#include <string.h>
#include "file_io.h"

/***************************************************************************//**
 * @brief usleep
*******************************************************************************/
static inline void usleep(unsigned long usleep)
{

}

/***************************************************************************//**
 * @brief spi_init
*******************************************************************************/
int32_t spi_init(uint32_t device_id,
				 uint8_t  clk_pha,
				 uint8_t  clk_pol)
{
    log_string("SPI_INIT_NOT_PROGRAMMED,,,\n");
	return device_id;
}

/***************************************************************************//**
 * @brief spi_read
*******************************************************************************/
int32_t spi_read(uint8_t *data, uint8_t bytes_number)
{
    log_string("SPI_READ_NOT_PROGRAMMED,,,\n");
	return 0;
}

/***************************************************************************//**
 * @brief spi_write_then_read
*******************************************************************************/
int spi_write_then_read(struct spi_device *spi, const uint8_t *txbuf, uint8_t n_tx, uint8_t *rxbuf, uint8_t n_rx)
{
    uint8_t i;
    uint8_t n;
    char str_buf[100];

    //CSV:
    /*
        header,     description,    register,       data
        spi,        read,           bytes[0-1],     num_read
        spi,        write,          bytes[0-1],     bytes[2-n_tx]
    */

    sprintf(str_buf,"spi,");
    n = strlen("spi,");
    if(n_rx>0)
    {
        sprintf(&str_buf[n],"read,");
        n += strlen("read,");
    }else{
        sprintf(&str_buf[n],"write,");
        n += strlen("write,");
    }

    sprintf(&str_buf[n],"%02x%02x,",txbuf[0],txbuf[1]);
    n += 5;

    if(n_rx>0){
        sprintf(&str_buf[n],"%d\n",n_rx);
    }else{
        for(i=2; i<n_tx; i++)
        {
            sprintf(&str_buf[(i-2)*2+n], "%02x", txbuf[i]);
        }
        n += (i-2)*2;
        sprintf(&str_buf[n],"\n");
    }

    log_string(str_buf);

    /*dev_spi(&spi->dev,"tx %d: ",n_tx);
    for(i=0; i<n_tx; i++)
    {
        dev_spi(&spi->dev,"%02x",txbuf[i]);
    }
    if(n_rx>0){
        dev_spi(&spi->dev," -- rx %d bytes ",n_rx);
    }
    dev_spi(&spi->dev,"\r\n");*/

	return SUCCESS;
}

/***************************************************************************//**
 * @brief gpio_init
*******************************************************************************/
void gpio_init(uint32_t device_id)
{
    log_string("NO_GPIO_INIT,,,\n");

}

/***************************************************************************//**
 * @brief gpio_direction
*******************************************************************************/
void gpio_direction(const char *pin_text, uint8_t pin, uint8_t direction)
{
    char str_buf[100];
    uint8_t i;
    uint8_t n;

    //CSV:
    /*
        header,     description,    register,   data
        gpio_dir,   pin_text_descr  pin_num,    direction
    */
    sprintf(str_buf,"gpio_dir,");
    n = strlen("gpio_dir,");
    sprintf(&str_buf[n],pin_text);
    n +=  strlen(pin_text);
    sprintf(&str_buf[n],",%d,%d\n",pin,direction);

    log_string(str_buf);
}

/***************************************************************************//**
 * @brief gpio_is_valid
*******************************************************************************/
bool gpio_is_valid(int number)
{
    log_string("GPIO_IS_VALID_NOT_PROGRAMMED,,,\n");
	return 0;
}

/***************************************************************************//**
 * @brief gpio_data
*******************************************************************************/
void gpio_data(uint8_t pin, uint8_t data)
{
    log_string("GPIO_DATA_NOT_PROGRAMMED,,,\n");
}

/***************************************************************************//**
 * @brief gpio_set_value
*******************************************************************************/
void gpio_set_value(unsigned gpio, int value)
{
    log_string("GPIO_SET_VALUE_NOT_PROGRAMMED,,,\n");
}

/***************************************************************************//**
 * @brief udelay
*******************************************************************************/
void udelay(unsigned long usecs)
{
    log_string("udelay_not_programmed,,,\n");
}

/***************************************************************************//**
 * @brief mdelay
*******************************************************************************/
void mdelay(unsigned long msecs)
{
    char str_buf[50];
    sprintf(str_buf,"msec_delay,,,%d\n",msecs);
    log_string(str_buf);
}

/***************************************************************************//**
 * @brief msleep_interruptible
*******************************************************************************/
unsigned long msleep_interruptible(unsigned int msecs)
{
    log_string("msleep_interrupible_NOT_PROGRAMMED,,,\n");
	return 0;
}

/***************************************************************************//**
 * @brief axiadc_init
*******************************************************************************/
void axiadc_init(struct ad9361_rf_phy *phy)
{

}

/***************************************************************************//**
 * @brief axiadc_post_setup
*******************************************************************************/
int axiadc_post_setup(struct ad9361_rf_phy *phy)
{
    log_string("NEED TO DELETE axiadc_post_setup!,,,\n");
	return 0;
}

/***************************************************************************//**
 * @brief axiadc_read
*******************************************************************************/
unsigned int axiadc_read(struct axiadc_state *st, unsigned long reg)
{
    log_string("NEED TO DELETE axiadc_read!,,,\n");
	return 0;
}

/***************************************************************************//**
 * @brief axiadc_write
*******************************************************************************/
void axiadc_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
    log_string("NEED TO DELETE axiadc_write!,,,\n");
}

/***************************************************************************//**
* @brief axiadc_set_pnsel
*******************************************************************************/
int axiadc_set_pnsel(struct axiadc_state *st, int channel, enum adc_pn_sel sel)
{
    log_string("NEED TO DELETE axiadc_set_pnsel!,,,\n");
	return 0;
}

/***************************************************************************//**
 * @brief axiadc_idelay_set
*******************************************************************************/
void axiadc_idelay_set(struct axiadc_state *st,
				unsigned lane, unsigned val)
{
    log_string("NEED TO DELETE axiadc_idelay_set!,,,\n");
}
