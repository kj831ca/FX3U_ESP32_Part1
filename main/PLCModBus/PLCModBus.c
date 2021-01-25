#include <stdio.h>
#include "PLCModBus.h"
#include "esp_log.h"
#include "driver/uart.h"
//#define DEBUG_PRINT

const uint8_t gabyCRCLo[256] = 
                {   0x00,0xc0,0xc1,0x01,0xc3,0x03,0x02,0xc2,0xc6,0x06,
                    0x07,0xc7,0x05,0xc5,0xc4,0x04,0xcc,0x0c,0x0d,0xcd,
                    0x0f,0xcf,0xce,0x0e,0x0a,0xca,0xcb,0x0b,0xc9,0x09,
                    0x08,0xc8,0xd8,0x18,0x19,0xd9,0x1b,0xdb,0xda,0x1a,
                    0x1e,0xde,0xdf,0x1f,0xdd,0x1d,0x1c,0xdc,0x14,0xd4,
                    0xd5,0x15,0xd7,0x17,0x16,0xd6,0xd2,0x12,0x13,0xd3,
                    0x11,0xd1,0xd0,0x10,0xf0,0x30,0x31,0xf1,0x33,0xf3,
                    0xf2,0x32,0x36,0xf6,0xf7,0x37,0xf5,0x35,0x34,0xf4,
                    0x3c,0xfc,0xfd,0x3d,0xff,0x3f,0x3e,0xfe,0xfa,0x3a,
                    0x3b,0xfb,0x39,0xf9,0xf8,0x38,0x28,0xe8,0xe9,0x29,
                    0xeb,0x2b,0x2a,0xea,0xee,0x2e,0x2f,0xef,0x2d,0xed,
                    0xec,0x2c,0xe4,0x24,0x25,0xe5,0x27,0xe7,0xe6,0x26,
                    0x22,0xe2,0xe3,0x23,0xe1,0x21,0x20,0xe0,0xa0,0x60,
                    0x61,0xa1,0x63,0xa3,0xa2,0x62,0x66,0xa6,0xa7,0x67,
                    0xa5,0x65,0x64,0xa4,0x6c,0xac,0xad,0x6d,0xaf,0x6f,
                    0x6e,0xae,0xaa,0x6a,0x6b,0xab,0x69,0xa9,0xa8,0x68,
                    0x78,0xb8,0xb9,0x79,0xbb,0x7b,0x7a,0xba,0xbe,0x7e,
                    0x7f,0xbf,0x7d,0xbd,0xbc,0x7c,0xb4,0x74,0x75,0xb5,
                    0x77,0xb7,0xb6,0x76,0x72,0xb2,0xb3,0x73,0xb1,0x71,
                    0x70,0xb0,0x50,0x90,0x91,0x51,0x93,0x53,0x52,0x92,
                    0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,0x9c,0x5c,
                    0x5d,0x9d,0x5f,0x9f,0x9e,0x5e,0x5a,0x9a,0x9b,0x5b,
                    0x99,0x59,0x58,0x98,0x88,0x48,0x49,0x89,0x4b,0x8b,
                    0x8a,0x4a,0x4e,0x8e,0x8f,0x4f,0x8d,0x4d,0x4c,0x8c,
                    0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,
                    0x43,0x83,0x41,0x81,0x80,0x40 };
const uint8_t  gabyCRCHi[256] =
                {   0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,
                    0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,
                    0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,0x01,0xc0,
                    0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,
                    0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x00,0xc1,
                    0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,
                    0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x00,0xc1,
                    0x81,0x40,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,
                    0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,
                    0x80,0x41,0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,
                    0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,
                    0x81,0x40,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,
                    0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,
                    0x80,0x41,0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,
                    0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,
                    0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,
                    0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,
                    0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,
                    0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,0x01,0xc0,
                    0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,
                    0x01,0xc0,0x80,0x41,0x00,0xc1,0x81,0x40,0x01,0xc0,
                    0x80,0x41,0x00,0xc1,0x81,0x40,0x00,0xc1,0x81,0x40,
                    0x01,0xc0,0x80,0x41,0x01,0xc0,0x80,0x41,0x00,0xc1,
                    0x81,0x40,0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,
                    0x00,0xc1,0x81,0x40,0x01,0xc0,0x80,0x41,0x01,0xc0,
                    0x80,0x41,0x00,0xc1,0x81,0x40 };

static QueueHandle_t xPLCQueue;

void ModBusOneByteCRC(ModBusFrame *modBusCmd, uint8_t bData)
{
    uint8_t byIdx;
    byIdx = (uint8_t)(modBusCmd->crc_hi ^ bData);
    modBusCmd->crc_hi = (uint8_t)(modBusCmd->crc_lo ^ gabyCRCHi[byIdx]);
    modBusCmd->crc_lo = gabyCRCLo[byIdx];    
}

void ResetCRC(ModBusFrame *modBusCmd)
{
    modBusCmd->crc_hi = 0xff;
    modBusCmd->crc_lo = 0xff;
}

void CalculateCRC(ModBusFrame *modBusCmd)
{
    uint8_t byteTmp;
    uint8_t index ;
    //Reset the crc first
    ResetCRC(modBusCmd);

    //byteTmp = (uint8_t)(modBusCmd->address >> 8) ;
    //ModBusOneByteCRC(modBusCmd,byteTmp);

    byteTmp = (uint8_t)(modBusCmd->address & 0xFF);
    ModBusOneByteCRC(modBusCmd,byteTmp);

    ModBusOneByteCRC(modBusCmd,modBusCmd->command);
    for(index = 0; index < modBusCmd->data_length; index++)
    {
        ModBusOneByteCRC(modBusCmd,modBusCmd->data[index]);
    }
}
void FormatReadMRegCommand(ModBusFrame *modBus,uint16_t station_number,uint16_t mRegNumber)
{
    modBus->address = station_number ;
    modBus->command = READ_M;
    modBus->data[0] = HIGH_BYTE(mRegNumber);
    modBus->data[1] = LOW_BYTE(mRegNumber);
    modBus->data[2] = 0;
    modBus->data[3] = 8;
    modBus->data_length = 4;
    CalculateCRC(modBus);
}

void FormatSetMRegCommand(ModBusFrame *modBus,uint16_t station_number,uint16_t mRegNumber, uint8_t state)
{
    #ifdef DEBUG_PRINT
    ESP_LOGI("PLC","Formatting SM Command");
    #endif
    modBus->address = station_number ;
    modBus->command = SET_M;
    modBus->data[0] = HIGH_BYTE(mRegNumber);
    modBus->data[1] = LOW_BYTE(mRegNumber);
    if(state != 0)
        modBus->data[2] = 0xFF ;
    else 
        modBus->data[2] = 0;
    modBus->data[3] = 0;
    modBus->data_length = 4;
    CalculateCRC(modBus);
}

void FormatReadDRegCommand(ModBusFrame *modBus,uint16_t station_number,uint16_t dRegNumber)
{
    modBus->address = station_number ;
    modBus->command = READ_D;
    modBus->data[0] = HIGH_BYTE(dRegNumber);
    modBus->data[1] = LOW_BYTE(dRegNumber);
    modBus->data[2] = 0;
    modBus->data[3] = 1;
    modBus->data_length = 4;
    modBus->frameState = FrameIdle;
    CalculateCRC(modBus);
}

void FormatReadD32RegCommand(ModBusFrame *modBus,uint16_t station_number,uint16_t dRegNumber)
{
    modBus->address = station_number ;
    modBus->command = READ_D;
    modBus->data[0] = HIGH_BYTE(dRegNumber);
    modBus->data[1] = LOW_BYTE(dRegNumber);
    modBus->data[2] = 0;
    modBus->data[3] = 2;
    modBus->data_length = 4;
    modBus->frameState = FrameIdle;
    CalculateCRC(modBus);
}

void FormatSetDRegCommand(ModBusFrame *modBus,uint16_t station_number,uint16_t dRegNumber, uint16_t dRegValue)
{
    modBus->address = station_number ;
    modBus->command = SET_D;
    modBus->data[0] = HIGH_BYTE(dRegNumber);
    modBus->data[1] = LOW_BYTE(dRegNumber);
    modBus->data[2] = HIGH_BYTE(dRegValue);
    modBus->data[3] = LOW_BYTE(dRegValue);
    modBus->data_length = 4;
    modBus->frameState = FrameIdle;
    CalculateCRC(modBus);
}

int SendPLCFrame(uart_port_t uart_num,ModBusFrame * modBus )
{
    char sendBuffer[10];
    int i = 0;

    sendBuffer[i++] = modBus->address;
    sendBuffer[i++] = modBus->command;
    for(int j=0;j<4;j++)
    {
        sendBuffer[i++]= modBus->data[j];
    }
    sendBuffer[i++] = modBus->crc_hi;
    sendBuffer[i++] = modBus->crc_lo;

    #ifdef DEBUG_PRINT
    DebugPrintBytes("SEND",sendBuffer,i);
    #endif
    modBus->frameState = FrameTX;
    return uart_write_bytes(uart_num,sendBuffer, i);
}

void InitPLCUart(PLCUartSettings *setting)
{
    uart_config_t uart_config = {
        .baud_rate = PLC_BAUDRATE, 
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(setting->uart_num, 1024, 0, 0, NULL, 0);
    uart_param_config(setting->uart_num, &uart_config);
    uart_set_pin(setting->uart_num, setting->tx_io_num, setting->rx_io_num, 
                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void vPLCTask(void *pvParameter)
{
    PLCUartSettings *plc = (PLCUartSettings *)pvParameter;
    ModBusFrame *modBusFrame;
    int len;
    char buffer[100];
    while(1)
    {
        if(uxQueueMessagesWaiting(xPLCQueue)>0)
        {
            if(xQueueReceive(xPLCQueue,&modBusFrame,(TickType_t)10) == pdPASS)
            {
                uart_flush(plc->uart_num);
                SendPLCFrame(plc->uart_num, modBusFrame);
                vTaskDelay(PKG_TIMEOUT/portTICK_PERIOD_MS);
                uart_get_buffered_data_len(plc->uart_num, (size_t*)&len);
                if(len > 5)
                {
                    len = uart_read_bytes(plc->uart_num,buffer,len,10);
                    #ifdef DEBUG_PRINT
                    DebugPrintBytes("RECEIVE",buffer,len);
                    #endif
                    HandleRecieveByte(modBusFrame,buffer,len);
                } else 
                {
                    modBusFrame->frameState = FrameTimeout;
                }
            }
        }
        else
        {
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
        
    }
}
void DebugPrintBytes(const char* tag,char* buffer, int length)
{
    if(tag != NULL)
    {
        printf("%s: ",tag);
    }
    for(int i=0;i<length;i++)
    {
        printf(" %02x",buffer[i]);
    }
    putchar('\n');
}
void DebugPrint(ModBusFrame *modBus)
{
    char sendBuffer[10];
    
    int i = 0;

    sendBuffer[i++] = modBus->address;
    sendBuffer[i++] = modBus->command;
    for(int j=0;j<modBus->data_length;j++)
    {
        sendBuffer[i++]= modBus->data[j];
    }
    sendBuffer[i++] = modBus->crc_hi;
    sendBuffer[i++] = modBus->crc_lo;

    for(int j=0; j<i;j++)
    {
        printf(" %02x",sendBuffer[j]);
    }
    putchar('\n');

}

QueueHandle_t InitPLCTask(PLCUartSettings *plcSetting)
{
    xPLCQueue = xQueueCreate(20,sizeof(ModBusFrame *));
    InitPLCUart(plcSetting);
    xTaskCreate(vPLCTask,"PLCTask",2048,(void *)plcSetting,6,NULL);
    return xPLCQueue;
}

int ParsingPLCCommand(char *cmdStr, PLCCommand *cmd)
{
    char buff[3];
    int stNumber, regNumber;
    int32_t val;
    int length = sscanf(cmdStr,"%2s%d,%d,%d",buff,&stNumber,&regNumber,&val);
    if(length >= 3)
    {
        if(strcmp(buff,"SM")== 0)
        {
            #ifdef DEBUG_PRINT
            ESP_LOGI("PLC","Send SM Command");
            #endif
            cmd->command = SET_M ;
            cmd->station_number = stNumber ;
            cmd->reg_number = regNumber ;
            cmd->value = val;
            return 1;
        } else if(strcmp(buff,"RM")==0)
        {
            #ifdef DEBUG_PRINT
            ESP_LOGI("PLC","Send RM Command");
            #endif
            cmd->command = READ_M;
            cmd->station_number = stNumber ;
            cmd->reg_number = regNumber ;
            return 1;
        } else if(strcmp(buff,"SD")==0)
        {
            #ifdef DEBUG_PRINT
            ESP_LOGI("PLC","Send SD Command");
            #endif
            cmd->command = SET_D;
            cmd->station_number = stNumber ;
            cmd->reg_number = regNumber ;
            cmd->value = val;
            return 1;
        } else if(strcmp(buff,"RD")==0)
        {
            #ifdef DEBUG_PRINT
            ESP_LOGI("PLC","Send RD Command");
            #endif
            cmd->command = READ_D;
            cmd->station_number = stNumber ;
            cmd->reg_number = regNumber ;
            cmd->value = val;
            return 1;
        } else if(strcmp(buff,"RW")==0)
        {
             #ifdef DEBUG_PRINT
            ESP_LOGI("PLC","Send RD 32 bitsCommand");
            #endif
            cmd->command = READ_D32;
            cmd->station_number = stNumber ;
            cmd->reg_number = regNumber ;
            cmd->value = val;
            return 1;
        } else if(strcmp(buff,"SW")==0)
        {
            cmd->command = WRITE_D32;
            cmd->station_number = stNumber ;
            cmd->reg_number = regNumber ;
            cmd->value = val;
            return 1;
        }
    }
    return 0;
}

void HandlePLCCommand(PLCCommand *cmd,ModBusFrame *frame)
{
    static ModBusFrame tmpFrame;
    ModBusFrame *pF = &tmpFrame;
    uint16_t tempD;
    
    switch(cmd->command)
    {
        case SET_M:
            FormatSetMRegCommand(frame,cmd->station_number,cmd->reg_number,cmd->value);
            xQueueSend(xPLCQueue,&frame,10);
        break;
        case SET_D:
            FormatSetDRegCommand(frame,cmd->station_number,cmd->reg_number,cmd->value);
            xQueueSend(xPLCQueue,&frame,10);
        break;
        case READ_D:
            FormatReadDRegCommand(frame,cmd->station_number,cmd->reg_number);
            xQueueSend(xPLCQueue,&frame,10);
        break;
        case READ_M:
            FormatReadMRegCommand(frame,cmd->station_number,cmd->reg_number);
            xQueueSend(xPLCQueue,&frame,10);
        break;
        case READ_D32:
            FormatReadD32RegCommand(frame,cmd->station_number,cmd->reg_number);
            xQueueSend(xPLCQueue,&frame,10);
        break;
        
        case WRITE_D32:

            tempD = (uint16_t)(cmd->value & 0xffff);
            FormatSetDRegCommand(&tmpFrame,cmd->station_number,cmd->reg_number,tempD); //Write lower D first.
            xQueueSend(xPLCQueue,&pF,10);

            tempD = (uint16_t)(cmd->value >> 16);
            FormatSetDRegCommand(frame,cmd->station_number,cmd->reg_number+1,tempD);
            xQueueSend(xPLCQueue,&frame,10);
        break;
    }
}

uint8_t HandleRecieveByte(ModBusFrame *modBus,char *data, uint8_t length)
{
    static ModBusFrame retFrame ;
    if(length < 5)
    {
        return 0;
    } 
    else //Check CRC matching
    {
        //Copy the original sending fram
        retFrame.address = modBus->address ;
        retFrame.command = modBus->command;
        retFrame.data[0] = modBus->data[0];
        retFrame.data[1] = modBus->data[1];

        ResetCRC(&retFrame);        
        
        for(int i=0;i<length-2;i++)
        {
            ModBusOneByteCRC(&retFrame,data[i]);
        }
            
        if(retFrame.crc_hi != data[length-2] || retFrame.crc_lo != data[length-1])
        {
            //CRC Error here 
            ESP_LOGE("RECEIVE FRAME","CRC ERROR");
            return 0;
        }
            
        if(data[1] != retFrame.command)
        {
            ESP_LOGE("RECEIVE FRAME","Command mismatch");    
            return 0;
        }
        switch(data[1])
        {
            case SET_M:
            case SET_D:
                retFrame.data[0] = data[2];
                retFrame.data[1] = data[3];
                retFrame.data[2] = data[4];
                retFrame.data[3] = data[5];
                retFrame.data_length = 4;
            break;
            case READ_M:
                retFrame.data_length = 3;
                retFrame.data[2] = data[3];
            break;
            case READ_D:
                retFrame.data_length = data[2]+2;
                retFrame.data[2] = data[3];
                retFrame.data[3] = data[4];
                if(data[2] == 4) //We got reading from 32 bits read.
                {
                    retFrame.data[4] = data[5];
                    retFrame.data[5] = data[6];
                }

            break;

        }
        //Copy the frame back to the original frame.
        for(int i=0;i<retFrame.data_length;i++)
        {
            modBus->data[i] = retFrame.data[i];
        }
        modBus->frameState = FrameRX;
        //Notify callback function.
        if(modBus->read_cb != NULL)
        {
            modBus->read_cb(&retFrame);
        }
        #ifdef DEBUG_PRINT
        DebugPrint(&retFrame);
        #endif
        return 1;
    }
}
bool EnqueueFrame(ModBusFrame *modBus)
{
    if(xPLCQueue == NULL) return false;
    modBus->frameState = FrameInQueue;
    return xQueueSend(xPLCQueue,&modBus,10) == pdPASS;
}

void InitPLCMReg(PLCMReg *mReg)
{
    FormatReadMRegCommand(&mReg->readCommand,mReg->stationNumber,mReg->address);
    FormatSetMRegCommand(&mReg->writeCommand,mReg->stationNumber,mReg->address,mReg->state);
}

bool SetPLCMReg(PLCMReg *mReg, uint8_t state)
{
    ModBusFrame *pf = &mReg->writeCommand;
    if(state != 0)
        mReg->writeCommand.data[2] = 0xFF ;
    else 
        mReg->writeCommand.data[2] = 0;
    mReg->writeCommand.data[3] = 0;
    mReg->writeCommand.data_length = 4;
    CalculateCRC(&mReg->writeCommand);
    mReg->writeCommand.frameState = FrameInQueue;
    return EnqueueFrame(pf);
}
bool ReadPLCMRegAsync(PLCMReg *mReg)
{
    ModBusFrame *pf = &mReg->readCommand;
    FormatReadMRegCommand(&mReg->readCommand,mReg->stationNumber,mReg->address);
    return EnqueueFrame(pf);
}
uint8_t ReadPLCMReg(PLCMReg *mReg)
{
    ModBusFrame *pf = &mReg->readCommand;
    FormatReadMRegCommand(&mReg->readCommand,mReg->stationNumber,mReg->address);
    bool isPolling = true;
    int loopCount = 0;
    int txDelayCount = 0;
    if(xPLCQueue == NULL) return 0;
    if(EnqueueFrame(pf))
    {
        while(isPolling)
        {
            if(pf->frameState == FrameTX)
            {
                
                vTaskDelay(10/portTICK_PERIOD_MS);
                if(pf->frameState == FrameRX)
                {
                    mReg->state = (pf->data[2] & 0x01) != 0;
                    return 1;
                } else 
                {
                    txDelayCount++;
                    if(txDelayCount>7)
                    {
                        ESP_LOGE("RD_MREG","Package Timeout error");
                        return 0;
                    }
                }
            } else
            {
                vTaskDelay(10/portTICK_PERIOD_MS);
                loopCount++;
                if(loopCount > 100)
                {
                    ESP_LOGE("RD_MREG","PLC No repsonse error");
                    return 0; //Timeout error 
                }
            } 

        }
    } else 
    {
        return 0;
    }
    return 0;
}

void InitPLCDMReg(PLCDReg *dReg)
{
    FormatReadDRegCommand(&dReg->readCommand,dReg->stationNumber,dReg->address);
    FormatSetDRegCommand(&dReg->readCommand,dReg->stationNumber,dReg->address,dReg->value);
}

bool SetPLCDReg(PLCDReg *dReg, uint16_t value)
{
    ModBusFrame *pf = &dReg->writeCommand;
    FormatSetDRegCommand(&dReg->writeCommand,dReg->stationNumber,dReg->address,value);
    return EnqueueFrame(pf);
}

bool ReadPLCDRegAsync(PLCDReg *dReg)
{
    ModBusFrame *pf = &dReg->readCommand;
    FormatReadMRegCommand(&dReg->readCommand,dReg->stationNumber,dReg->address);
    return EnqueueFrame(pf);   
}

uint8_t ReadPLCDReg(PLCDReg *dReg)
{
    ModBusFrame *pf = &dReg->readCommand;
    FormatReadDRegCommand(&dReg->readCommand,dReg->stationNumber,dReg->address);
    bool isPolling = true;
    int loopCount = 0;
    int txDelayCount = 0;
    if(xPLCQueue == NULL) return 0;
    if(EnqueueFrame(pf))
    {
        while(isPolling)
        {
            if(pf->frameState == FrameTX)
            {
                vTaskDelay(10/portTICK_PERIOD_MS);
                if(pf->frameState == FrameRX)
                {
                    dReg->value = (uint16_t)(pf->data[2] << 8) | (pf->data[3]);
                    return 1;
                } else 
                {
                    txDelayCount++;
                    if(txDelayCount>7)
                    {
                        ESP_LOGE("RD_DREG","Package Timeout error");
                        return 0;
                    }
                }
            } else
            {
                vTaskDelay(10/portTICK_PERIOD_MS);
                loopCount++;
                if(loopCount > 100)
                {
                    ESP_LOGE("RD_MREG","PLC No repsonse error");
                    return 0; //Timeout error 
                }
            } 

        }
    } else 
    {
        ESP_LOGE("RD_MREG","Unable to Queue command");
        return 0;
    }
    return 0;
}

void InitPLCD32Reg(PLCD32Reg *d32Reg)
{
    FormatReadD32RegCommand(&d32Reg->readCommand,d32Reg->stationNumber,d32Reg->address);
    FormatSetDRegCommand(&d32Reg->writeLowDCommand,d32Reg->stationNumber,d32Reg->address,0);
    FormatSetDRegCommand(&d32Reg->writeHiDCommand,d32Reg->stationNumber,d32Reg->address+1,0);
}

bool SetPLCD32Reg(PLCD32Reg *d32Reg, uint32_t value)
{
    bool sendLowOK, sendHiOK;
    uint16_t val = value & 0xffff;
    FormatSetDRegCommand(&d32Reg->writeLowDCommand,d32Reg->stationNumber,d32Reg->address,val);
    
    val = value >> 16 ;
    FormatSetDRegCommand(&d32Reg->writeHiDCommand,d32Reg->stationNumber,d32Reg->address+1,val);

    sendLowOK = EnqueueFrame(&d32Reg->writeLowDCommand);
    sendHiOK = EnqueueFrame(&d32Reg->writeHiDCommand);

    return (sendLowOK && sendHiOK);
}

bool ReadPLCD32RegAsync(PLCD32Reg *d32Reg)
{
    FormatReadD32RegCommand(&d32Reg->readCommand,d32Reg->stationNumber,d32Reg->address);
    return EnqueueFrame(&d32Reg->readCommand);
}

uint8_t ReadPLCD32Reg(PLCD32Reg *dReg)
{
    ModBusFrame *pf = &dReg->readCommand;
    FormatReadD32RegCommand(&dReg->readCommand,dReg->stationNumber,dReg->address);
    bool isPolling = true;
    int loopCount = 0;
    int txDelayCount = 0;
    if(xPLCQueue == NULL) return 0;
    if(EnqueueFrame(pf))
    {
        while(isPolling)
        {
            if(pf->frameState == FrameTX)
            {
                vTaskDelay(10/portTICK_PERIOD_MS);
                if(pf->frameState == FrameRX)
                {
                    dReg->value = (pf->data[2] << 8) | (pf->data[3]);
                    dReg->value += ((pf->data[4] << 8) | (pf->data[5])) << 16 ;
                    return 1;
                } else 
                {
                    txDelayCount++;
                    if(txDelayCount>7)
                    {
                        ESP_LOGE("RD_DREG","Package Timeout error");
                        return 0;
                    }
                }
            } else
            {
                vTaskDelay(10/portTICK_PERIOD_MS);
                loopCount++;
                if(loopCount > 100)
                {
                    ESP_LOGE("RD_MREG","PLC No repsonse error");
                    return 0; //Timeout error 
                }
            } 

        }
    } else 
    {
        ESP_LOGE("RD_MREG","Unable to Queue command");
        return 0;
    }
    return 0;
}