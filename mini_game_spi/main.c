
/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


//add for scheduler

#include "app_scheduler.h"
#include "nordic_common.h"

//add for button
#include <stdbool.h>
#include <stdint.h>
#include "bsp.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "nrf_error.h"

#define ST7586_SPI_INSTANCE  0										/**< SPI instance index. */
static const nrf_drv_spi_t st7586_spi = NRF_DRV_SPI_INSTANCE(ST7586_SPI_INSTANCE);  	/**< SPI instance. */
static volatile bool st7586_spi_xfer_done = false;  						/**< Flag used to indicate that SPI instance completed the transfer. */

#define ST_COMMAND			0
#define ST_DATA			1

#define RATIO_SPI0_LCD_SCK          		4
#define RATIO_SPI0_LCD_A0		    	28
#define RATIO_SPI0_LCD_MOSI		    	29
#define RATIO_SPI0_LCD_BSTB		    	30
#define RATIO_SPI0_LCD_CS			31

#define LCD_INIT_DELAY(t) nrf_delay_ms(t)
int x = 0;
int y = 0;
static unsigned char rx_data;


#define BUTTON_PREV_ID           0                           /**< Button used to switch the state. */
#define BUTTON_NEXT_ID           1                           /**< Button used to switch the state. */

static bsp_indication_t actual_state =  BSP_INDICATE_FIRST;         /**< Currently indicated state. */

static const char * indications_list[] = BSP_INDICATIONS_LIST;


//start button code


void st7586_write(const uint8_t category, const uint8_t data)
{
	int err_code;
    nrf_gpio_pin_write(RATIO_SPI0_LCD_A0, category);
    st7586_spi_xfer_done = false;

    err_code = nrf_drv_spi_transfer(&st7586_spi, &data, 1, &rx_data, 0);
    APP_ERROR_CHECK(err_code);
    while (!st7586_spi_xfer_done) {
    	__WFE();
    }
    nrf_delay_us(10);
}


void clear_gallag(){

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x-1); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+5); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y-1); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+17);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=6;i++){
		for(int j = 0; j<=18 ; j++){
			st7586_write(ST_DATA,  0x00);
		}
	}
}


void plane(){	

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+2); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+2); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+3);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=3 ; j++){
			st7586_write(ST_DATA,  0x03);
		}
	}


	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+3); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+3); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+3);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=3 ; j++){
			st7586_write(ST_DATA,  0xe0);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+2); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+2); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+4); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+7);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=4 ; j++){
			st7586_write(ST_DATA,  0x1f);
		}
	}


	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+3); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+3); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+4); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+7);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=4 ; j++){
			st7586_write(ST_DATA,  0xfc);
		}
	}


	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+0); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+0); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+7); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+11);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=5 ; j++){
			st7586_write(ST_DATA,  0xfc);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+5); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+5); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+7); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+11);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=5 ; j++){
			st7586_write(ST_DATA,  0x1f);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+1); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+1); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+11); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+11);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=0 ; j++){
			st7586_write(ST_DATA,  0x1f);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+1); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+1); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+10); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+10);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=0 ; j++){
			st7586_write(ST_DATA,  0x03);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+2); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+2); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+8); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+9);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=2 ; j++){
			st7586_write(ST_DATA,  0xfc);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+2); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+2); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+10); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+11);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=2 ; j++){
			st7586_write(ST_DATA,  0xe0);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+3); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+3); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+8); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+9);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=2 ; j++){
			st7586_write(ST_DATA,  0x1f);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+3); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+3); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+10); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+11);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=2 ; j++){
			st7586_write(ST_DATA,  0x03);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+4); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+4); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+11); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+11);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=0 ; j++){
			st7586_write(ST_DATA,  0xfc);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+4); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+4); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+10); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+10);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=0 ; j++){
			st7586_write(ST_DATA,  0xe0);
		}
	}
	
	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+5); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+12); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+14);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=5;i++){
		for(int j = 0; j<=2 ; j++){
			st7586_write(ST_DATA,  0xff);
		}
	}


	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+14); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+17);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=3 ; j++){
			st7586_write(ST_DATA,  0xff);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+16); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+17);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=1 ; j++){
			st7586_write(ST_DATA,  0xfc);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+5); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+5); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+14); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+17);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=3 ; j++){
			st7586_write(ST_DATA,  0xff);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+5); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+5); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+16); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+17);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=1 ; j++){
			st7586_write(ST_DATA,  0x1f);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+1); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+1); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+15); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+15);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=0 ; j++){
			st7586_write(ST_DATA,  0x03);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+1); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+1); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+16); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+17);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=1 ; j++){
			st7586_write(ST_DATA,  0x1f);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+2); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+2); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+16); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+17);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=1 ; j++){
			st7586_write(ST_DATA,  0xfc);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+4); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+4); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+15); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+15);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=0 ; j++){
			st7586_write(ST_DATA,  0xe0);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+4); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+4); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+16); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+17);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=1 ; j++){
			st7586_write(ST_DATA,  0xfc);
		}
	}

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+3); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+3); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+16); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+17);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=0;i++){
		for(int j = 0; j<=1 ; j++){
			st7586_write(ST_DATA,  0x1f);
		}
	}


	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+2); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, x+3); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+15); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, y+15);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<=1;i++){
		for(int j = 0; j<=0 ; j++){
			st7586_write(ST_DATA,  0xff);
		}
	}
}
void led0_invert(void * p_event_data, uint16_t event_size){
	bsp_board_led_invert(0);
	x++;
	clear_gallag();
	plane();
}

void led1_invert(void * p_event_data, uint16_t event_size){
	bsp_board_led_invert(1);
	y++;
	clear_gallag();
	plane();
}

/**@brief Function for handling bsp events.
 */
void bsp_evt_handler(bsp_event_t evt)
{
    uint32_t err_code;
    switch (evt)
    {
        case BSP_EVENT_KEY_0:

	     app_sched_event_put (&evt, sizeof(evt),led0_invert);
		
            break;

        case BSP_EVENT_KEY_1:

	     app_sched_event_put (&evt, sizeof(evt),led1_invert);
            break;

        default:
            return; // no implementation needed
    }
    err_code = bsp_indication_set(actual_state);
    NRF_LOG_INFO("%s", (uint32_t)indications_list[actual_state]);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing low frequency clock.
 */
void clock_initialization()
{
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }
}


/**@brief Function for initializing bsp module.
 */
void bsp_configuration()
{
    uint32_t err_code;

    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_evt_handler);
    APP_ERROR_CHECK(err_code);
}

//finish button code

void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
	st7586_spi_xfer_done = true;
}

static inline void st7586_pinout_setup()
{
    // spi setup
	int err_code;
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.ss_pin   = RATIO_SPI0_LCD_CS;
    	spi_config.miso_pin = NRF_DRV_SPI_PIN_NOT_USED;
    	spi_config.mosi_pin = RATIO_SPI0_LCD_MOSI;
    	spi_config.sck_pin  = RATIO_SPI0_LCD_SCK;
    	spi_config.frequency = NRF_SPI_FREQ_1M;
    	spi_config.mode = NRF_DRV_SPI_MODE_3;
	err_code = nrf_drv_spi_init(&st7586_spi, &spi_config, spi_event_handler, NULL);
	APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_set(RATIO_SPI0_LCD_A0);
    nrf_gpio_cfg_output(RATIO_SPI0_LCD_A0);

    nrf_gpio_pin_clear(RATIO_SPI0_LCD_BSTB);
    nrf_gpio_cfg_output(RATIO_SPI0_LCD_BSTB);
}

void init(){
	
    nrf_gpio_pin_write(RATIO_SPI0_LCD_BSTB, 0);
	LCD_INIT_DELAY(10);
	nrf_gpio_pin_write(RATIO_SPI0_LCD_BSTB, 1);


	LCD_INIT_DELAY(120);


	st7586_write(ST_COMMAND,  0xD7); 
	st7586_write(ST_DATA, 0x9F); 
	st7586_write(ST_COMMAND,  0xE0); 
	st7586_write(ST_DATA, 0x00); 
	LCD_INIT_DELAY(10);
	st7586_write(ST_COMMAND,  0xE3); 
	LCD_INIT_DELAY(20);
	st7586_write(ST_COMMAND,  0xE1); 
	st7586_write(ST_COMMAND,  0x11); 
	st7586_write(ST_COMMAND,  0x28); 
	LCD_INIT_DELAY(50);
	st7586_write(ST_COMMAND,  0xC0);
	st7586_write(ST_DATA, 0x53);
	st7586_write(ST_DATA, 0x01);
	st7586_write(ST_COMMAND,  0xC3);
	st7586_write(ST_DATA, 0x02);
	st7586_write(ST_COMMAND,  0xC4);
	st7586_write(ST_DATA, 0x06);	

	st7586_write(ST_COMMAND,  0xD0); 
	st7586_write(ST_DATA, 0x1D); 
	st7586_write(ST_COMMAND,  0xB5); 
	st7586_write(ST_DATA, 0x00); 

	st7586_write(ST_COMMAND,  0x39); 
	st7586_write(ST_COMMAND,  0x3A); 
	st7586_write(ST_DATA, 0x02); 
	st7586_write(ST_COMMAND,  0x36); 
	st7586_write(ST_DATA, 0x00); 

	st7586_write(ST_COMMAND,  0xB0); 
	st7586_write(ST_DATA, 0x9F); 
	st7586_write(ST_COMMAND,  0xB4); 
	st7586_write(ST_DATA, 0xA0); 

	st7586_write(ST_COMMAND,  0x30); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x77); 
	st7586_write(ST_COMMAND,  0x20);
	st7586_write(ST_COMMAND,  0x2A); 
	

	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x7F);
	st7586_write(ST_COMMAND,  0x2B); 
	
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x9F);

	//Clear_DDRAM();


	st7586_write(ST_COMMAND,  0x2A); 	
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x4F); 
	st7586_write(ST_COMMAND,  0x2B); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x78);

	//Disp_Image();

	st7586_write(ST_COMMAND,  0x29);		    // disp on
}


void clear_noise(){

	st7586_write(ST_COMMAND,  0x2A); 	        //set column
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x7f); 

	st7586_write(ST_COMMAND,  0x2B); 	        //set row
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x00); 
	st7586_write(ST_DATA, 0x9f);

	st7586_write(ST_COMMAND,  0x2c);


	for(int i = 0; i<0x0000007f*3;i++){
		for(int j = 0; j<0x0000009f ; j++){
			st7586_write(ST_DATA,  0x00);
		}
	}
}



void led_invert(){
	for (int i = 0; i < 4; i++){
            bsp_board_led_invert(i);
            nrf_delay_ms(500);
        }
}

int main(void)
{
    	bsp_board_leds_init();

    	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    	NRF_LOG_DEFAULT_BACKENDS_INIT();
    
    	st7586_pinout_setup();

    	NRF_LOG_INFO("SPI example.");

	//Reset_ms(10); 
	
	//code start

	init();
	clear_noise();
	plane();

	st7586_write(ST_COMMAND,  0x29);		    //disp on

	
  	clock_initialization();
  	uint32_t err_code = app_timer_init();
    	APP_ERROR_CHECK(err_code);
    	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

	APP_SCHED_INIT(sizeof(bsp_event_t),10*sizeof(bsp_event_t));
    	bsp_configuration();
	

    while (1)
    {
	app_sched_execute();
        NRF_LOG_FLUSH();
        __WFE();


    }
}