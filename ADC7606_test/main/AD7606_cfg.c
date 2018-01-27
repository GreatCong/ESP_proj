/**
 * AD7606 config using esp32 
 */
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_err.h"


#include "main.h"

/*********************************************************************/
//spi handle
spi_device_handle_t AD_SPI_handle;
bool inter_state = false;
//The semaphore indicating the slave is ready to receive stuff.
// static xQueueHandle rdySem;
spi_transaction_t spi_t;
uint8_t rx_data[8]={0,0,0,0,0,0,0,0};
xQueueHandle AD_evt_queue = NULL;

static void IRAM_ATTR gpio_handshake_isr_handler(void* arg);
/*********************************************************************/

/**
 * [Init_pwm description]
 * note:use ledc module to generate PWM
 */
void Init_pwm(void){
	ledc_timer_config_t ledc_timer = {
		.bit_num = LEDC_TIMER_10_BIT,//range is 0--1024-1
		.freq_hz = 10000,//10kHZ
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.timer_num = LEDC_TIMER_0
	};
	ledc_timer_config(&ledc_timer);

	ledc_channel_config_t ledc_channel = {
		.channel = LEDC_CHANNEL_0,
		// .duty = 512,//50%
		 .duty = 0,//50%
		.gpio_num = AD_CONVEST,
		// .intr_type = LEDC_INTR_FADE_END,
		.intr_type = LEDC_INTR_DISABLE,//disable ledx interrupt
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.timer_sel = LEDC_TIMER_0
	};

	ledc_channel_config(&ledc_channel);
	// ledc_fade_func_install(0);//OR intr_alloc_flags with ESP_INTR_FLAG_IRAM because the fade isr is in IRAM
}

/**
 * [Start_AD description]
 */
void Start_AD(void){
	ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 512);//50% duty
	ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

/**
 * [Stop_AD description]
 */
void Stop_AD(void){
	ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);//50% duty
	ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

/**
 * [Init_gpio description]
 */
void Init_gpio(void){
   //GPIO config for the handshake line.
    gpio_config_t io_conf={
        .intr_type=GPIO_PIN_INTR_NEGEDGE,
        .mode=GPIO_MODE_INPUT,
        .pull_up_en=0,
        .pin_bit_mask=(1ULL << AD_BUSY)
    };

    //Set up handshake line interrupt.
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_set_intr_type(AD_BUSY, GPIO_PIN_INTR_NEGEDGE);//positive edge interrupt
    gpio_isr_handler_add(AD_BUSY, gpio_handshake_isr_handler, NULL);

    //GPIO config AD_OS*
    gpio_config_t io_conf_OS;
    //disable interrupt
    io_conf_OS.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf_OS.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf_OS.pin_bit_mask = AD_OS_PIN_SEL;//AD_OS0 AD_OS1 AD_OS2
    //disable pull-down mode
    io_conf_OS.pull_down_en = 0;
    //disable pull-up mode
    io_conf_OS.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf_OS);    
}

/**
 * [AD7606_SetOSRate description]
 * @param rate [description]
 */
void AD7606_SetOSRate(AD7606_OS_Rate rate){
	switch(rate){
		case AD_OS_NO: {OS2(0);OS1(0);OS0(0);break;}
		case AD_OS_X2: {OS2(0);OS1(0);OS0(1);break;}
		case AD_OS_X4: {OS2(0);OS1(1);OS0(0);break;}
		case AD_OS_X8: {OS2(0);OS1(1);OS0(1);break;}
		case AD_OS_X16: {OS2(1);OS1(0);OS0(0);break;}
		case AD_OS_X32: {OS2(1);OS1(0);OS0(1);break;}
		case AD_OS_X64: {OS2(1);OS1(1);OS0(0);break;}
		default: {OS2(0);OS1(0);OS0(0);break;}
	}
}

/**
 * [AD7606_Init description]
 */
void AD7606_Init(void){

	AD7606_SetOSRate(AD_OS_NO);
}

/*
This ISR is called when the handshake line goes high.
*/
static void IRAM_ATTR gpio_handshake_isr_handler(void* arg)
{

    //Give the semaphore.
     // BaseType_t mustYield=false;
     // xSemaphoreGiveFromISR(rdySem, &mustYield);
     // if (mustYield) {
     //    //spi_device_transmit(AD_SPI_handle, &spi_t);
        spi_t.tx_buffer = rx_data;
	    spi_t.length = 8*8;
		//uint8_t rx_data = 0;
		spi_t.rx_buffer = rx_data;
        if(spi_device_queue_trans(AD_SPI_handle, &spi_t, 5)!=ESP_OK){//keep 5 tick
        	inter_state = true;
        }
        AD_custom_event_t ad_data;
        for (int i = 0; i < 8; ++i)
        {
        	/* code */
        	ad_data.data[i] = rx_data[i];
        }
        xQueueSendFromISR(AD_evt_queue, &ad_data, NULL);
     // 	portYIELD_FROM_ISR();
     // }
    //printf("inter ok\n");//not used in interrupt for "printf",will abort
     //inter_state = true;
}

/**
 * [Init_spi description]
 *  note: master SPI
 */
void Init_spi(void){
	esp_err_t ret;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=AD_MOSI,
        .miso_io_num=AD_MISO,
        .sclk_io_num=AD_SCLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };

    //Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=20*1000*1000,               //Clock out at 10 MHz
        .mode=2,                                //SPI mode 0
        .spics_io_num=AD_CS,               //CS pin
        //.cs_ena_posttrans=3,        //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size=3*4
        //.pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };


    //Initialize the SPI bus and add the device we want to send stuff to.
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);//DMA1
    assert(ret==ESP_OK);
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &AD_SPI_handle);
    assert(ret==ESP_OK);
}
