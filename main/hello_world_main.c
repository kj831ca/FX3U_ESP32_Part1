/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "PLCModBus/PLCModBus.h"

static const char *TAG = "PLC_Example";
static PLCUartSettings uartSettings = {UART_NUM_2,GPIO_NUM_17,GPIO_NUM_16};

static PLCMReg plcM0 = {.stationNumber=1, .address = 0}; 
static PLCMReg plcM7 = {.stationNumber=1, .address= 7};
//Demonstrate reading and control the PLC from ESP32..
static void plc_reading_task(void *pvParameter)
{
    InitPLCMReg(&plcM0);
    InitPLCMReg(&plcM7);

    TickType_t prevTick =  xTaskGetTickCount();
    bool lastX03State = true;
    while(1)
    {
        ReadPLCMReg(&plcM0);
        if(plcM0.state)
        {
            SetPLCMReg(&plcM0,0);
        } else 
        {
            SetPLCMReg(&plcM0,1);
        }        
        if(ReadPLCMReg(&plcM7))
        {
            if(plcM7.state != lastX03State)
            {
                lastX03State = plcM7.state;
                ESP_LOGI(TAG,"X03 = %s",plcM7.state ? "ON" : "OFF");
            }
        }
        vTaskDelayUntil(&prevTick,500/portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    InitPLCTask(&uartSettings);
    xTaskCreate(plc_reading_task, "PLC_Main_Task",2048,NULL,4,NULL);

}
