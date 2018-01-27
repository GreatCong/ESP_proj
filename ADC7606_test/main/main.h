#ifndef __MAIN_H__
#define __MAIN_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/spi_master.h"

#define AD_CONVEST (2)
/*
Pins in use. The SPI Master can use the GPIO mux, so feel free to change these if needed.
*/
#define AD_BUSY 33
#define AD_MOSI 12//Master in
#define AD_MISO 13//Master out #board connect miso
#define AD_SCLK 15
#define AD_CS 14

#define AD_OS0 25
#define AD_OS1 26
#define AD_OS2 27
#define AD_OS_PIN_SEL  ((1ULL<<AD_OS0) | (1ULL<<AD_OS1) | (1ULL<<AD_OS2))

#define OS2(State) gpio_set_level(AD_OS2, State)
#define OS1(State) gpio_set_level(AD_OS1, State)
#define OS0(State) gpio_set_level(AD_OS0, State)

typedef enum{
	AD_OS_NO = 0U,
	AD_OS_X2 = 1U,
	AD_OS_X4 = 2U,
	AD_OS_X8 = 3U,
	AD_OS_X16 = 4U,
	AD_OS_X32 = 5U,
	AD_OS_X64 = 6U
}AD7606_OS_Rate;


typedef struct custom_event{
	uint8_t data[8];
}AD_custom_event_t;

extern spi_device_handle_t AD_SPI_handle;
extern bool inter_state;
//The semaphore indicating the slave is ready to receive stuff.
// static xQueueHandle rdySem;
extern spi_transaction_t spi_t;
extern xQueueHandle AD_evt_queue;

void Init_pwm(void);
void Start_AD(void);
void Stop_AD(void);
void Init_gpio(void);
void AD7606_SetOSRate(AD7606_OS_Rate rate);
void AD7606_Init(void);
void Init_spi(void);

#endif
