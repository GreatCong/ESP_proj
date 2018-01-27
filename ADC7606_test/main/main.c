#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"

#include "main.h"

/**
 * [spi_test_task description]
 * @param arg [description]
 */
static void spi_test_task(void * arg){
	AD_custom_event_t ad_data_rev;
    while(true) {
        if(xQueueReceive(AD_evt_queue, (void *)&ad_data_rev, portMAX_DELAY)) {
            if(ad_data_rev.data[0]!=0){
				for (int i = 0; i < 4; ++i)
				{
					/* code */
					printf("%d ", ad_data_rev.data[2*i]*255+ad_data_rev.data[2*i+1]);
                    // putchar(1);
				}
				printf("\n");
				// printf("%d,%d,%d\n",rx_data[0],rx_data[1],rx_data[2]);
		    }//if
        }//if queue

       if(inter_state){
		  printf("inter OK\n");
		  inter_state = false;
		}
    }//while
}

/**
 * [app_main description]
 */
void app_main(void){
    // esp_err_t ret;
    // spi_transaction_t t;
    memset(&spi_t, 0, sizeof(spi_t));

    //Init here
	Init_gpio();
	Init_pwm();
	Init_spi();
	AD7606_Init();

	Start_AD();
	AD_evt_queue = xQueueCreate(2048, sizeof(AD_custom_event_t));
	//Stop_AD();
    //
	    //Create the semaphore.
     // rdySem=xSemaphoreCreateBinary();
    // //Assume the slave is ready for the first transmission: if the slave started up before us, we will not detect 
    // //positive edge on the handshake line.
     // xSemaphoreGive(rdySem);
    // uint8_t rx_data[8]={0,0,0,0,0,0,0,0};

	// while(true){
	// 	spi_t.tx_buffer = rx_data;
	// 	spi_t.length = 8*8;
	// 	//uint8_t rx_data = 0;
	// 	spi_t.rx_buffer = rx_data;
	//     //Wait for slave to be ready for next byte before sending
 //         //xSemaphoreTake(rdySem, 10);//portMAX_DELAY); //Wait until slave is ready
	// 	// ret=spi_device_transmit(AD_SPI_handle, &spi_t);
	// 	// vTaskDelay(3000/portTICK_PERIOD_MS);//portTICK_PERIOD_MS=1
	// }
    xTaskCreate(&spi_test_task,"spi_test_task",2048,NULL,5,NULL);
    // while(true){

    // }
	//Never reached.
    // ret=spi_bus_remove_device(AD_SPI_handle);
    // assert(ret==ESP_OK);
}

