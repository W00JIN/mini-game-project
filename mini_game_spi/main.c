/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *	list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *	Semiconductor ASA integrated circuit in a product or a software update for
 *	such product, must reproduce the above copyright notice, this list of
 *	conditions and the following disclaimer in the documentation and/or other
 *	materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *	contributors may be used to endorse or promote products derived from this
 *	software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *	Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *	engineered, decompiled, modified and/or disassembled.
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
/**
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

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

#define ST7586_SPI_INSTANCE  0									/**< SPI instance index. */
static const nrf_drv_spi_t st7586_spi = NRF_DRV_SPI_INSTANCE(ST7586_SPI_INSTANCE);  	/**< SPI instance. */
static volatile bool st7586_spi_xfer_done = false;						/**< Flag used to indicate that SPI instance completed the transfer. */

#define ST_COMMAND			0
#define ST_DATA			1

#define RATIO_SPI0_LCD_SCK				4
#define RATIO_SPI0_LCD_A0				28
#define RATIO_SPI0_LCD_MOSI				29
#define RATIO_SPI0_LCD_BSTB				30
#define RATIO_SPI0_LCD_CS				31

#define LCD_INIT_DELAY(t) nrf_delay_ms(t)
#define TIME	10



// gallag

int x = 18;
int y = 120;
int hp_gallag = 5;
int game_num = 2;			//game num : 1 == gallag / 2 == tatris
int ble_available = 0;		//if you want use ble, change to 1 else 0

int x_enemy1 = 15;
int y_enemy1 = 40;
int x_bullet_enemy[10];
int y_bullet_enemy[10];
int bullet_des_enemy[10];
int hp_enemy1 = 3;
bool move_enemy1 = true;		//left,right
bool fire_enemy[10] = {false,};
int enemy_bullet_num = 1;

int x_bullet = 0;
int y_bullet = 200;

bool fire = false; //variable that became true when bullet flying.

unsigned long long frame = 0;

int x_kau = 0;
int y_kau = 0;
int hp_k = 6;
int hp_a = 6;
int hp_u = 6;




//tatris 

bool block_location[10][14] = {0};
int x_block=19;
int y_block=18+30; //18 + 140 is end of screen

int current_block = 2;
int current_color;



 
static unsigned char rx_data;
nrf_drv_spi_evt_t const * p;
bsp_event_t const * a;

#define BUTTON_PREV_ID		   0						   	/**< Button used to switch the state. */
#define BUTTON_NEXT_ID		   1						   	/**< Button used to switch the state. */

static bsp_indication_t actual_state =  BSP_INDICATE_FIRST;		 		/**< Currently indicated state. */

static const char * indications_list[] = BSP_INDICATIONS_LIST;


//start button code


#define APP_FEATURE_NOT_SUPPORTED	   BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2	/**< Reply when unsupported features are requested. */

#define ADVERTISING_LED			BSP_BOARD_LED_0			/**< Is on when device is advertising. */
#define CONNECTED_LED			BSP_BOARD_LED_1			/**< Is on when device has connected. */
#define LEDBUTTON_LED			BSP_BOARD_LED_2			/**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON			BSP_BUTTON_0				/**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME				"Mini Game Project"			/**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO		3					/**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG		1					/**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL			64					/**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS	  BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED	/**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL			MSEC_TO_UNITS(100, UNIT_1_25_MS)	/**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL			MSEC_TO_UNITS(200, UNIT_1_25_MS)	/**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY			0					/**< Slave latency. */
#define CONN_SUP_TIMEOUT			MSEC_TO_UNITS(4000, UNIT_10_MS)	/**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  	APP_TIMER_TICKS(20000)		/**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY	APP_TIMER_TICKS(5000)		/**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT	3					/**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY		APP_TIMER_TICKS(50)			/**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF				0xDEADBEEF				/**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


BLE_LBS_DEF(m_lbs);									/**< LED Button Service instance. */
NRF_BLE_GATT_DEF(m_gatt);								/**< GATT module instance. */
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;			/**< Handle of the current connection. */


void st7586_write(const uint8_t category, const uint8_t data);
void set_location(uint8_t cs, uint8_t csize, uint8_t rs, uint8_t rsize, uint8_t dot_num_and_color);
void set_location2(uint8_t cs, uint8_t csize, uint8_t rs, uint8_t rsize);
void bullet();
void clear_gallag();
void plane();
void enemy1();
void is_enemy1_alive();
void shoot_enemy();
void bullet_enemy(int num);
bool got_shot(int x_, int y_);
void end_game();
void kau();
void is_k_alive();
void clear_k();
void is_a_alive();
void is_u_alive();
void gallag_background();
void tetris_background();
void clear_block1();
void clear_block2();
void block1(uint8_t dot_111, uint8_t dot_110, uint8_t dot_100, uint8_t dot_001, uint8_t dot_011);
void block2(uint8_t dot_111, uint8_t dot_110, uint8_t dot_100, uint8_t dot_001, uint8_t dot_011);
void move_gallag_left(void * p_event_data, uint16_t event_size);
void move_gallag_right(void * p_event_data, uint16_t event_size);
void shoot_bullet(void * p_event_data, uint16_t event_size);
void move_block_left(void * p_event_data, uint16_t event_size);
void move_block_right(void * p_event_data, uint16_t event_size);
void spin_block(void * p_event_data, uint16_t event_size);
void accelerate_block_velocity(void * p_event_data, uint16_t event_size);
void drop_block(void * p_event_data, uint16_t event_size);
int current_block_fixed();
void new_block_down(void * p_event_data, uint16_t event_size);
void bsp_evt_handler(bsp_event_t evt);
void clock_initialization();
void bsp_configuration();
void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context);
static inline void st7586_pinout_setup();
void init();
void clear_noise();
void led_invert();
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);
static void leds_init(void);
static void timers_init(void);
static void gap_params_init(void);
static void gatt_init(void);
static void advertising_init(void);
void start_gallag(void * p_event_data, uint16_t event_size);
static void game_choice_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state);
static void services_init(void);
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt);
static void conn_params_error_handler(uint32_t nrf_error);
static void conn_params_init(void);
static void advertising_start(void);
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
static void ble_stack_init(void);
static void buttons_init(void);
static void log_init(void);
static void power_manage(void);

int main(void)
{
	// Initialize.
	leds_init();
	timers_init();
	log_init();
	buttons_init();
	ble_stack_init();
	gap_params_init();
	gatt_init();
	services_init();
	advertising_init();
	conn_params_init();
	
	advertising_start();
	st7586_pinout_setup();
	init();
	clear_noise();
	st7586_write(ST_COMMAND,  0x38);
   
	if(game_num == 1)
	{
		gallag_background();
		//kau();
		plane();
	}else if(game_num == 2)
	{
		tetris_background();
		switch(current_block)
		{
			case 1:
				block1(0xff,0xfc,0xe0,0x03,0x1f);
				break;
			case 2:
				block2(0xff,0xfc,0xe0,0x03,0x1f);
				break;
			default:
				break;
		}

	}

	st7586_write(ST_COMMAND,  0x29);					//실제 블루투스 연결시 삭제

	APP_SCHED_INIT(sizeof(bsp_event_t),3*sizeof(bsp_event_t));
	
	hp_a = 0;	hp_k = 0;	hp_u = 0;  //for fast debug

	while (1)
	{
		app_sched_execute(); 
		NRF_LOG_FLUSH();
		__WFE();
		
		if(game_num == 1)   //Condition for operating Gallag
		{
			if(fire)
			{
				led_invert();
				bullet();
			}
			if(hp_k) is_k_alive();
			if(hp_a) is_a_alive();
			if(hp_u) is_u_alive();
			if(!hp_k && !hp_a && !hp_u)
			{
				if(frame % (TIME*1) == 0)
				{
					for(int i = 0 ;i < enemy_bullet_num; i++)
					{
						if(fire_enemy[i])
						{
							bullet_enemy(i);
						}
					}
				}
				if(frame % (TIME*20) == 0)  //excuted about every 0.2sec. 
				{
					if(hp_enemy1 >= 0)  //if enemy1 alive
					{
						is_enemy1_alive();
					}
				}
				if(frame % (TIME*50000)) //executed about every 10sec
				{
					if((hp_enemy1 >= 0)&&(enemy_bullet_num < 2))   //enemy1 shoot
					{
						shoot_enemy();
					}
				}
			}
		}else if(game_num ==2)   //Condition for operating Tetris
		{
			if(current_block_fixed()) app_sched_event_put(NULL,0,new_block_down);
			//if(blocks_make_lines) app_sched_event_put(NULL,0,remove_lines);
			//if(blocks_to_ceilling) app_sched_event_put(NULL,0,game_over);
		}
		
		if (NRF_LOG_PROCESS() == false && ble_available)
		{
			power_manage();
		}
		frame += 1;
	}
}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *		  how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num	Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
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
void set_location(uint8_t cs, uint8_t csize, uint8_t rs, uint8_t rsize, uint8_t dot_num_and_color)
{
	st7586_write(ST_COMMAND,  0x2A); 			//set column
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, cs);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, cs + csize);
	
	st7586_write(ST_COMMAND,  0x2B); 			//set row
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, rs);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, rs + rsize);
	
	st7586_write(ST_COMMAND,  0x2c);
	
	for(int i = 0; i<=csize;i++){
		for(int j = 0; j<=rsize ; j++){
			st7586_write(ST_DATA, dot_num_and_color);
		}
	}
}

void set_location2(uint8_t cs, uint8_t csize, uint8_t rs, uint8_t rsize)
{
	st7586_write(ST_COMMAND,  0x2A); 			//set column
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, cs);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, cs + csize);
	
	st7586_write(ST_COMMAND,  0x2B); 			//set row
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, rs);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, rs + rsize);
	
	st7586_write(ST_COMMAND,  0x2c);
}

//using set_location
void bullet()
{
	set_location(x_bullet+2,0,y_bullet-10,10,0xff);
	set_location(x_bullet+2,0,y_bullet-10,10,0x00);
	
	y_bullet -= 1;
	if(y_bullet < 0)
		fire = false;
	//
	if((x_enemy1 <= x_bullet)&&(x_bullet <= (x_enemy1+3))&&((y_enemy1-9) < y_bullet)&&(y_bullet < y_enemy1))
	{
		hp_enemy1--;
		if(hp_enemy1 <= 0)
		{
			set_location(0,0,0,3,0xff);	 //for debug
			set_location(x_enemy1,3,y_enemy1,9,0x00);
		}
		fire = false;
	}
	
}
//using set_location
void clear_gallag(){
	set_location(x-2, 9, 116, 25, 0x00);
}

//using set_location
void plane()
{	
	clear_gallag();
	
	set_location(x+2, 0, y, 3, 0x03);
	set_location(x+3, 0, y, 3, 0xe0);
	set_location(x+2, 0, y+4, 3, 0x1f);
	set_location(x+3, 0, y+4, 3, 0xfc);
	set_location(x, 0, y+7, 4, 0xfc);
	set_location(x+5, 0, y+7, 4, 0x1f);
	set_location(x+1, 0, y+11, 0, 0x1f);
	set_location(x+1, 0, y+10, 0, 0x03);
	set_location(x+2, 0, y+8, 1, 0xfc);
	set_location(x+2, 0, y+10, 1, 0xe0);
	set_location(x+3, 0, y+8, 1, 0x1f);
	set_location(x+3, 0, y+10, 1, 0x03);
	set_location(x+4, 0, y+11, 0, 0xfc);
	set_location(x+4, 0, y+10, 0, 0xe0);
	set_location(x, 5, y+12, 2, 0xff);
	set_location(x, 0, y+14, 3, 0xff);
	set_location(x, 0, y+16, 1, 0xfc);
	set_location(x+5, 0, y+14, 3, 0xff);
	set_location(x+5, 0, y+16, 1, 0x1f);
	set_location(x+1, 0, y+15, 0, 0x03);
	set_location(x+1, 0, y+16, 1, 0x1f);
	set_location(x+2, 0, y+16, 1, 0xfc);
	set_location(x+4, 0, y+15, 0, 0xe0);
	set_location(x+4, 0, y+16, 1, 0xfc);
	set_location(x+3, 0, y+16, 1, 0x1f);
	set_location(x+2, 1, y+15, 0, 0xff);
}

void enemy1()
{
	set_location(x_enemy1,3,y_enemy1,9,0xff);
}
void is_enemy1_alive()
{
	set_location(x_enemy1, 3, y_enemy1, 9, 0x00); //erase previous enemy1
	if(move_enemy1)
		x_enemy1 += 1;						
	else
		x_enemy1 -= 1;
	enemy1();
	if( x_enemy1 > 40)
		move_enemy1 = false;
	if( x_enemy1 < 1)
		move_enemy1 = true;
}
void shoot_enemy()
{
	/*
	bullet_des_enemy[enemy_bullet_num] = (y-y_enemy1)/(x-x_enemy1);
	x_bullet_enemy[enemy_bullet_num] = x_enemy1;
	y_bullet_enemy[enemy_bullet_num] = y_enemy1+5;
	fire_enemy[enemy_bullet_num] = true;
	enemy_bullet_num++;
*/
	
	bullet_des_enemy[0] = (y-y_enemy1)/(x-x_enemy1);
	x_bullet_enemy[0] = x_enemy1;
	y_bullet_enemy[0] = y_enemy1+5;
	fire_enemy[0] = true;
//	enemy_bullet_num++;
}
void bullet_enemy(int num)
{

	set_location(x_bullet_enemy[num],0,y_bullet_enemy[num],2,0x00);  //erase previous bullet_enemy_1
	y_bullet_enemy[num]++;
	if((y_bullet_enemy[num] % bullet_des_enemy[num]) == 0)
		x_bullet_enemy[num]++;
	set_location2(x_bullet_enemy[num],0,y_bullet_enemy[num],2);
	st7586_write(ST_DATA,0x1c);
	st7586_write(ST_DATA,0xff);
	st7586_write(ST_DATA,0x1c);
	if(got_shot(x_bullet_enemy[num], y_bullet_enemy[num]) || (y_bullet_enemy[num] >= 125))
	{
		fire_enemy[num] = false;
	}
		
}

bool got_shot(int x_, int y_)
{
	if((x+1 < x_) && (x_ < x+5) && (y+4 < y_) && (y_ < y+10))
	{
		hp_gallag--;
		if(hp_gallag <= 0)
		{
			end_game();
		}
		return true;
	}
	return false;

}

void end_game()
{
	//should be implimented
	set_location(10,20,30,100,0xff);
}

void kau()
{
	int a,b;
	a = x_kau;
	b = y_kau;
	hp_a = 6;
	hp_k = 6;
	hp_u = 6;
	//K
	set_location(a, 3, b, 70, 0xff);
	for(int i = 0; i < 45; i++)
	{
		set_location((a+10) - (i/4), 2, b+i, 9, 0xff);
	}
	b += 22;
	for(int i = 0; i < 40; i++)
	{
		set_location(a + (i/4), 2, b+i, 9, 0xff);
	}
	//A
	b = 0;
	a += 14;
	for(int i = 0; i < 70; i++)
	{
		set_location(a+5-(i/10), 2, b+i,1,0xff);
	}

	for(int i = 0; i < 70; i++)
	{
		set_location(a+5+(i/10), 2, b+i,1,0xff);
	}

	set_location(a+2, 6, b + 40, 7, 0xff);

	//U
	a += 15;
	set_location(a,2,b,65,0xff);
	set_location(39,2,b,65,0xff);
	set_location(a+2,8,b+60,8,0xff);

}

void is_k_alive()
{
	if((y_bullet < 79) && (x_bullet < 13))
	{
		hp_k--;
		fire = false;
		y_bullet = 200;
		if(hp_k == 0)	
			set_location(0,12,0,70,0x00);

	}
}

void is_a_alive()
{
	if((y_bullet < 79)&& (13<x_bullet) && (x_bullet < 25))
	{
		hp_a--;
		fire = false;
		y_bullet = 200;
		if(hp_a == 0)
			set_location(13,14,0,70,0x00);
	}
}

void is_u_alive()
{
	if((y_bullet < 79)&& (25<x_bullet))
	{
		hp_u--;
		fire = false;
		y_bullet = 200;
		if(hp_u == 0)
			set_location(24,17,0,70,0x00);
	}
}

void gallag_background()
{
	for(int i = 0; i < 5; i++)
	{
		set_location2(3+i*4,0,145+i,2);
		st7586_write(ST_DATA,0x1c);
		st7586_write(ST_DATA,0xff);
		st7586_write(ST_DATA,0x1c);
	}

	for(int i = 0; i < 5; i++)
	{
		set_location2(39-i*4,0,150+i,2);
		st7586_write(ST_DATA,0x1c);
		st7586_write(ST_DATA,0xff);
		st7586_write(ST_DATA,0x1c);
	}
}

void tetris_background()
{
	set_location(0,0x7f,0x00,0x9f,0xff);
	
	set_location(3,0,19,12,0x00);
	set_location(3,0,32,127,0x92);
	set_location(4,0,19,12,0x00);
	set_location(4,0,32,127,0x49);
	set_location(5,0,19,140,0x00);
	set_location(37,0,19,140,0x00);
	set_location(38,0,19,12,0x00);
	set_location(38,0,32,127,0x49);
	set_location(39,0,19,12,0x00);
	set_location(39,0,32,127,0x52);
	
	set_location(6,0,32,127,0x11);
	
	set_location(7,29,32,127,0x49);
	set_location(37,0,32,127,0x80);
	
	set_location(40,0,32,127,0x9f);
	
	
	set_location(0,0x7f,159,0,0xff);
	
	set_location(2,0,31,0,0xfc);
	set_location(6,0,31,0,0x1f);
	set_location(36,0,31,0,0xfc);
	set_location(40,0,31,0,0x1f);
	
	set_location(2,0,30,0,0xe0);
	set_location(6,0,30,0,0x03);
	set_location(36,0,30,0,0xe0);
	set_location(40,0,30,0,0x03);
	
	set_location(2,0,29,0,0x00);
	set_location(6,0,29,0,0x00);
	set_location(36,0,29,0,0x00);
	set_location(40,0,29,0,0x00);

	set_location(1,0,28,0,0xfc);
	set_location(2,0,28,0,0x00);
	set_location(6,0,28,0,0x00);
	set_location(7,0,28,0,0x1f);
	set_location(35,0,28,0,0xfc);
	set_location(36,0,28,0,0x00);
	set_location(40,0,28,0,0x00);
	set_location(41,0,28,0,0x1f);
	
	set_location(1,0,27,0,0xe0);
	set_location(2,0,27,0,0x00);
	set_location(6,0,27,0,0x00);
	set_location(7,0,27,0,0x03);
	set_location(35,0,27,0,0xe0);
	set_location(36,0,27,0,0x00);
	set_location(40,0,27,0,0x00);
	set_location(41,0,27,0,0x03);
	
	set_location(1,0,23,3,0x00);
	set_location(2,0,23,3,0x00);
	set_location(6,0,23,3,0x00);
	set_location(7,0,23,3,0x00);
	set_location(35,0,23,3,0x00);
	set_location(36,0,23,3,0x00);
	set_location(40,0,23,3,0x00);
	set_location(41,0,23,3,0x00);
	
	set_location(1,0,22,0,0xe0);
	set_location(2,0,22,0,0x00);
	set_location(6,0,22,0,0x00);
	set_location(7,0,22,0,0x03);
	set_location(35,0,22,0,0xe0);
	set_location(36,0,22,0,0x00);
	set_location(40,0,22,0,0x00);
	set_location(41,0,22,0,0x03);
	
	set_location(1,0,21,0,0xfc);
	set_location(2,0,21,0,0x00);
	set_location(6,0,21,0,0x00);
	set_location(7,0,21,0,0x1f);
	set_location(35,0,21,0,0xfc);
	set_location(36,0,21,0,0x00);
	set_location(40,0,21,0,0x00);
	set_location(41,0,21,0,0x1f);
	
	set_location(2,0,20,0,0x00);
	set_location(6,0,20,0,0x00);
	set_location(36,0,20,0,0x00);
	set_location(40,0,20,0,0x00);
	
	set_location(2,0,19,0,0xfc);
	set_location(6,0,19,0,0x1f);
	set_location(36,0,19,0,0xfc);
	set_location(40,0,19,0,0x1f);
	
	set_location(3,0,18,0,0x00);
	set_location(4,0,18,0,0x00);
	set_location(5,0,18,0,0x00);
	set_location(37,0,18,0,0x00);
	set_location(38,0,18,0,0x00);
	set_location(39,0,18,0,0x00);
	
	
	set_location(3,0,17,0,0xfc);
	set_location(4,0,17,0,0x00);
	set_location(5,0,17,0,0x1f);
	set_location(37,0,17,0,0xfc);
	set_location(38,0,17,0,0x00);
	set_location(39,0,17,0,0x1f);
	
	set_location(4,0,16,0,0x00);
	set_location(38,0,16,0,0x00);
	
	set_location(4,0,15,0,0xe3);
	set_location(38,0,15,0,0xe3);
	
}
void clear_block1()
{
	int block_height = 7;
	//Enter by block pixel
	set_location(x_block, 5, y_block - (block_height*2+2), block_height*2+2, 0x49);
}
void block1(uint8_t dot_111, uint8_t dot_110, uint8_t dot_100, uint8_t dot_001, uint8_t dot_011)
{
	
	int block_height = 7;
	//Enter by block pixel
	set_location(x_block, 1, y_block - (block_height*2+2), block_height, dot_111);
	set_location(x_block+2, 0, y_block - (block_height*2+2), block_height, dot_110);
	
	set_location(x_block+3, 1, y_block - (block_height*2+2),block_height, dot_111);
	set_location(x_block+5, 0, y_block - (block_height*2+2),block_height, dot_110);
	
	set_location(x_block, 1, y_block - (block_height), block_height, dot_111);
	set_location(x_block+2, 0, y_block - (block_height), block_height, dot_110);
	
	set_location(x_block+3, 1, y_block - (block_height), block_height, dot_111);
	set_location(x_block+5, 0, y_block - (block_height), block_height, dot_110);
}
void clear_block2()
{
	int block_height = 7;
	//Enter by block pixel
	set_location(x_block, 1, y_block - (block_height*2+2), block_height, 0x49);
	set_location(x_block+2, 0, y_block - (block_height*2+2), block_height, 0x49);
	
	set_location(x_block+3, 1, y_block+2, block_height, 0x49);
	set_location(x_block+5, 0, y_block+2, block_height, 0x49);
	
	set_location(x_block, 1, y_block - (block_height), block_height, 0x49);
	set_location(x_block+2, 0, y_block - (block_height), block_height, 0x49);
	
	set_location(x_block+3, 1, y_block - (block_height), block_height, 0x49);
	set_location(x_block+5, 0, y_block - (block_height), block_height, 0x49);

}
void block2(uint8_t dot_111, uint8_t dot_110, uint8_t dot_100, uint8_t dot_001, uint8_t dot_011)
{
	
	int block_height = 7;
	//Enter by block pixel
	set_location(x_block, 1, y_block - (block_height*2+2), block_height, dot_111);
	set_location(x_block+2, 0, y_block - (block_height*2+2), block_height, dot_110);
	
	set_location(x_block+3, 1, y_block+2, block_height, dot_111);
	set_location(x_block+5, 0, y_block+2, block_height, dot_110);
	
	set_location(x_block, 1, y_block - (block_height), block_height, dot_111);
	set_location(x_block+2, 0, y_block - (block_height), block_height, dot_110);
	
	set_location(x_block+3, 1, y_block - (block_height), block_height, dot_111);
	set_location(x_block+5, 0, y_block - (block_height), block_height, dot_110);
}

/**@brief Function for btn event to send scheduler
 */
void move_gallag_left(void * p_event_data, uint16_t event_size)
{
	bsp_board_led_invert(0);
	if(x>0) x-=2;
	plane();
}
void move_gallag_right(void * p_event_data, uint16_t event_size)
{
	bsp_board_led_invert(0);
	if(x<43) x+=2;
	plane();
}
void shoot_bullet(void * p_event_data, uint16_t event_size)
{
	bsp_board_led_invert(1);
	x_bullet = x;
	y_bullet = y-3;
	fire = true;
}
void move_block_left(void * p_event_data, uint16_t event_size)
{
	int x_num = ((x_block - 1)/3)-2;
	int y_num = (156 - y_block + 8)/9;

	bsp_board_led_invert(0);

	switch(current_block)
	{
		case 1:
			if(x_block > 7 && block_location[x_num - 1][y_num] == false && block_location[x_num-1][y_num + 1] == false && block_location[x_num-1][y_num-1] == false)
			{
				clear_block1();
				x_block-=3;
				block1(0xff,0xfc,0xe0,0x03,0x1f);
			}
			break;

		case 2:
			if(x_block > 7 && block_location[x_num - 1][y_num-1] == false && block_location[x_num-1][y_num ] == false && block_location[x_num][y_num-2] == false)
			{
				clear_block2();
				x_block-=3;
				block2(0xff,0xfc,0xe0,0x03,0x1f);
			}
			break;
		default:
			break;
	}

}
void move_block_right(void * p_event_data, uint16_t event_size)
{
	int x_num = ((x_block - 1)/3)-2;
	int y_num = (156 - y_block + 8)/9;

	bsp_board_led_invert(0);
	
	switch(current_block)
	{
		case 1:
			if(x_block < 30 && block_location[x_num +2][y_num] == false && block_location[x_num +2][y_num + 1] == false && block_location[x_num +2][y_num-1] == false)
			{
				clear_block1();
				x_block+=3;
				block1(0xff,0xfc,0xe0,0x03,0x1f);
			}
			break;

		case 2:
			if(x_block < 30 && block_location[x_num +2][y_num-1] == false && block_location[x_num +2][y_num - 2] == false && block_location[x_num +2][y_num] == false)
			{
				clear_block2();
				x_block+=3;
				block2(0xff,0xfc,0xe0,0x03,0x1f);
			}
			break;
		default:
			break;
	}
}

void spin_block(void * p_event_data, uint16_t event_size)
{
	
}
void accelerate_block_velocity(void * p_event_data, uint16_t event_size)
{

	int x_num = ((x_block - 1)/3)-2;
	int y_num = (156 - y_block + 8)/9;

	switch(current_block)
	{
		case 1:
			clear_block1();
			break;
		case 2:
			clear_block2();
			break;
		default:
			break;
	}

	if( (block_location[x_num][y_num-1] == false || block_location[x_num + 1][y_num-1] == false) && y_num < 149)
	{
		y_block+=3;
	}
	switch(current_block)
	{
		case 1:
			block1(0xff,0xfc,0xe0,0x03,0x1f);
			break;
		case 2:
			block2(0xff,0xfc,0xe0,0x03,0x1f);
			break;
		default:
			break;
	}
}
void drop_block(void * p_event_data, uint16_t event_size)
{
	switch(current_block)
	{
		case 1:
			clear_block1();
			break;
		case 2:
			clear_block2();
			break;
		default:
			break;
	}
	y_block++;
	switch(current_block)
	{
		case 1:
			block1(0xff,0xfc,0xe0,0x03,0x1f);
			break;
		case 2:
			block2(0xff,0xfc,0xe0,0x03,0x1f);
			break;
		default:
			break;
	}
	nrf_delay_ms(100);

}
int current_block_fixed()
{
	app_sched_event_put (NULL, 0 ,drop_block);
	int x_num = ((x_block - 1)/3)-2;
	int y_num = (156 - y_block + 8)/9;

	if(current_block == 1)
	{
		if(y_num == 0 || block_location[x_num][y_num-1] == true || block_location[x_num + 1][y_num-1] == true)
		{
			block_location[x_num][y_num] = true;
			block_location[x_num][y_num + 1] = true;
			block_location[x_num + 1][y_num] = true;
			block_location[x_num + 1][y_num + 1] = true;
	
			return 1;	//fix current block and go to new_block_down
		}

		else return 0;	//repeat current_block_fixed untill current block cant drop anymore
	}

	if(current_block == 2)
	{
		if(y_block ==147 || block_location[x_num][y_num-1] == true || block_location[x_num + 1][y_num-2] == true)
		{
			block_location[x_num][y_num] = true;
			block_location[x_num][y_num + 1] = true;
			block_location[x_num + 1][y_num] = true;
			block_location[x_num + 1][y_num - 1] = true;
	
			return 1;	//fix current block and go to new_block_down
		}

		else return 0;	//repeat current_block_fixed untill current block cant drop anymore
	}
	return 0;
}

void new_block_down(void * p_event_data, uint16_t event_size)
{
	x_block = 19;
	y_block = 18+30;
	
	
	switch(current_block)
	{
		case 1:
			block1(0xff,0xfc,0xe0,0x03,0x1f);
			break;
		case 2:
			block2(0xff,0xfc,0xe0,0x03,0x1f);
			break;
		default:
			break;
	}

}
/**@brief Function for handling bsp events.
 */
void bsp_evt_handler(bsp_event_t evt)
{
	uint32_t err_code;
	if(!ble_available)
	{
		switch (evt)
		{
			case BSP_EVENT_KEY_0:
				if(game_num == 1)
					app_sched_event_put (&evt, sizeof(evt),move_gallag_left);
				if(game_num == 2)
					app_sched_event_put (&evt, sizeof(evt),move_block_left);
				break;
				
			case BSP_EVENT_KEY_1:
				if(game_num == 1)
					app_sched_event_put (&evt, sizeof(evt),move_gallag_right);
				if(game_num == 2)
					app_sched_event_put (&evt, sizeof(evt),move_block_right);
				break;

			case BSP_EVENT_KEY_2:
				if(game_num == 1)
					app_sched_event_put (&evt, sizeof(evt),shoot_bullet);
				if(game_num == 2)
					app_sched_event_put (&evt, sizeof(evt),spin_block);
				break;

			case BSP_EVENT_KEY_3:
				if(game_num == 1)
					app_sched_event_put (&evt, sizeof(evt),shoot_bullet);
				if(game_num == 2)
					app_sched_event_put (&evt, sizeof(evt),accelerate_block_velocity);
				break;

			default:
				return; // no implementation needed
		}
	}
	err_code = bsp_indication_set(actual_state);
	NRF_LOG_INFO("%s", (uint32_t)indications_list[actual_state]);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing low frequency clock.
 */
void clock_initialization()
{
	NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
	NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_LFCLKSTART	= 1;

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
void init()
{
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
}
void clear_noise()
{
	set_location(0,0x7f,0x00,0x9f,0x00);
}
void led_invert()
{
	for (int i = 0; i < 4; i++){
			bsp_board_led_invert(i);
			//nrf_delay_ms(500);
		}
}
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}
/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
	bsp_board_leds_init();
}
/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
	// Initialize timer module, making it use the scheduler
	ret_code_t err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *		  device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
	ret_code_t   err_code;
	ble_gap_conn_params_t   gap_conn_params;
	ble_gap_conn_sec_mode_t   sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency	 = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
	ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *		  Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
	ret_code_t	err_code;
	ble_advdata_t advdata;
	ble_advdata_t srdata;

	ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};

	// Build and set advertising data
	memset(&advdata, 0, sizeof(advdata));

	advdata.name_type		  = BLE_ADVDATA_FULL_NAME;
	advdata.include_appearance = true;
	advdata.flags			  = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


	memset(&srdata, 0, sizeof(srdata));
	srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
	srdata.uuids_complete.p_uuids  = adv_uuids;

	err_code = ble_advdata_set(&advdata, &srdata);
	APP_ERROR_CHECK(err_code);
}

void start_gallag(void * p_event_data, uint16_t event_size)
{
	plane();
	st7586_write(ST_COMMAND,  0x29);			//disp on
}

void start_tatris(void * p_event_data, uint16_t event_size)
{
	tetris_background();

	switch(current_block)
	{
		case 1:
			block1(0xff,0xfc,0xe0,0x03,0x1f);
			break;
		case 2:
			block2(0xff,0xfc,0xe0,0x03,0x1f);
			break;
		default:
			break;
	}
}


/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs	 Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void game_choice_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t data)
{
	if(ble_available)   //For init first time
	{
	}
	ble_available = 0;
	
	switch(data)
	{
		case 1:
			game_num = 1;
			bsp_board_led_on(LEDBUTTON_LED);
			app_sched_event_put ( NULL,0, start_gallag);
			break;

		case 2:
			game_num = 2;
			bsp_board_led_on(LEDBUTTON_LED);
			app_sched_event_put ( NULL,0, start_tatris);
			break;

		default:
			break;
	}
	NRF_LOG_INFO("Received BLE data!");
}
/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
	ret_code_t	 err_code;
	ble_lbs_init_t init;

	init.led_write_handler = game_choice_handler;

	err_code = ble_lbs_init(&m_lbs, &init);
	APP_ERROR_CHECK(err_code);
}
/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *		  are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *	   setting the disconnect_on_fail config parameter, but instead we use the event
 *	   handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
	ret_code_t err_code;

	if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
	{
		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
		APP_ERROR_CHECK(err_code);
	}
}
/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}
/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
	ret_code_t			 err_code;
	ble_conn_params_init_t cp_init;

	memset(&cp_init, 0, sizeof(cp_init));

	cp_init.p_conn_params			= NULL;
	cp_init.first_conn_params_update_delay	= FIRST_CONN_PARAMS_UPDATE_DELAY;
	cp_init.next_conn_params_update_delay	= NEXT_CONN_PARAMS_UPDATE_DELAY;
	cp_init.max_conn_params_update_count	= MAX_CONN_PARAMS_UPDATE_COUNT;
	cp_init.start_on_notify_cccd_handle	= BLE_GATT_HANDLE_INVALID;
	cp_init.disconnect_on_fail			= false;
	cp_init.evt_handler				= on_conn_params_evt;
	cp_init.error_handler			= conn_params_error_handler;

	err_code = ble_conn_params_init(&cp_init);
	APP_ERROR_CHECK(err_code);
}
/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
	ret_code_t		   err_code;
	ble_gap_adv_params_t adv_params;

	// Start advertising
	memset(&adv_params, 0, sizeof(adv_params));

	adv_params.type		= BLE_GAP_ADV_TYPE_ADV_IND;
	adv_params.p_peer_addr	= NULL;
	adv_params.fp			= BLE_GAP_ADV_FP_ANY;
	adv_params.interval		= APP_ADV_INTERVAL;
	adv_params.timeout		= APP_ADV_TIMEOUT_IN_SECONDS;

	err_code = sd_ble_gap_adv_start(&adv_params, APP_BLE_CONN_CFG_TAG);
	APP_ERROR_CHECK(err_code);
	bsp_board_led_on(ADVERTISING_LED);
}
/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
	ret_code_t err_code;

	switch (p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
			NRF_LOG_INFO("Connected");
			bsp_board_led_on(CONNECTED_LED);
			bsp_board_led_off(ADVERTISING_LED);
			m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

			err_code = app_button_enable();
			APP_ERROR_CHECK(err_code);
			break;

		case BLE_GAP_EVT_DISCONNECTED:
			NRF_LOG_INFO("Disconnected");
			bsp_board_led_off(CONNECTED_LED);
			m_conn_handle = BLE_CONN_HANDLE_INVALID;
			err_code = app_button_disable();
			APP_ERROR_CHECK(err_code);
			advertising_start();
			break;

		case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
			// Pairing not supported
			err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
			APP_ERROR_CHECK(err_code);
			break;

#ifndef S140
		case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
		{
			NRF_LOG_DEBUG("PHY update request.");
			ble_gap_phys_t const phys =
			{
				.rx_phys = BLE_GAP_PHY_AUTO,
				.tx_phys = BLE_GAP_PHY_AUTO,
			};
			err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
			APP_ERROR_CHECK(err_code);
		} break;
#endif

		case BLE_GATTS_EVT_SYS_ATTR_MISSING:
			// No system attributes have been stored.
			err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
			APP_ERROR_CHECK(err_code);
			break;

		case BLE_GATTC_EVT_TIMEOUT:
			// Disconnect on GATT Client timeout event.
			NRF_LOG_DEBUG("GATT Client Timeout.");
			err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			APP_ERROR_CHECK(err_code);
			break;

		case BLE_GATTS_EVT_TIMEOUT:
			// Disconnect on GATT Server timeout event.
			NRF_LOG_DEBUG("GATT Server Timeout.");
			err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			APP_ERROR_CHECK(err_code);
			break;

		case BLE_EVT_USER_MEM_REQUEST:
			err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
			APP_ERROR_CHECK(err_code);
			break;

		case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
		{
			ble_gatts_evt_rw_authorize_request_t  req;
			ble_gatts_rw_authorize_reply_params_t auth_reply;

			req = p_ble_evt->evt.gatts_evt.params.authorize_request;

			if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
			{
				if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)	 ||
					(req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
					(req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
				{
					if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
					{
						auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
					}
					else
					{
						auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
					}
					auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
					err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle, &auth_reply);
					APP_ERROR_CHECK(err_code);
				}
			}
		} break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

		default:
			// No implementation needed.
			break;
	}
}
/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
	ret_code_t err_code;

	err_code = nrf_sdh_enable_request();
	APP_ERROR_CHECK(err_code);

	// Configure the BLE stack using the default settings.
	// Fetch the start address of the application RAM.
	uint32_t ram_start = 0;
	err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
	APP_ERROR_CHECK(err_code);

	// Enable BLE stack.
	err_code = nrf_sdh_ble_enable(&ram_start);
	APP_ERROR_CHECK(err_code);

	// Register a handler for BLE events.
	NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}
/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no		The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */

/*
//button event for ble
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
	ret_code_t err_code;
	switch (pin_no)
	{
		case LEDBUTTON_BUTTON:
			NRF_LOG_INFO("Send button state change.");
			err_code = ble_lbs_on_button_change(m_conn_handle, &m_lbs, button_action);
			if (err_code != NRF_SUCCESS &&
				err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
				err_code != NRF_ERROR_INVALID_STATE &&
				err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
			{
				APP_ERROR_CHECK(err_code);
			}
			break;
		default:
			APP_ERROR_HANDLER(pin_no);
			break;
	}
}
*/

/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
	uint32_t err_code;
	
	err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_evt_handler);
	APP_ERROR_CHECK(err_code);
}
static void log_init(void)
{
	ret_code_t err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_DEFAULT_BACKENDS_INIT();
}
/**@brief Function for the Power Manager.
 */
static void power_manage(void)
{
	ret_code_t err_code = sd_app_evt_wait();
	APP_ERROR_CHECK(err_code);
}