/*H**********************************************************************
* FILENAME : PLCModBus.h 
*
* DESCRIPTION :
*       Allow ESP32 to communicate with Lollette FX3U PLC via RS485 using ModBus RTU protocol
*
* PUBLIC FUNCTIONS :
*       void InitPLCUart(PLCUartSettings *setting);
*       void FormatReadMRegCommand(ModBusFrame *modBus,uint16_t station_number,uint16_t mRegNumber);
*       void FormatSetMRegCommand(ModBusFrame *modBus,uint16_t station_number,uint16_t mRegNumber, uint8_t state);
*       void FormatReadDRegCommand(ModBusFrame *modBus,uint16_t station_number,uint16_t dRegNumber);
*       void FormatSetDRegCommand(ModBusFrame *modBus,uint16_t station_number,uint16_t dRegNumber, uint16_t dRegValue);
*       void FormatReadD32RegCommand(ModBusFrame *modBus,uint16_t station_number,uint16_t dRegNumber);
*       int ParsingPLCCommand(char *cmdStr, PLCCommand *cmd);       
*       void InitPLCMReg(PLCMReg *mReg);
*       bool SetPLCMReg(PLCMReg *mReg, uint8_t state);
*       uint8_t ReadPLCMReg(PLCMReg *mReg); //Blocking function call to get the M Reg state, not a threadsafe !!!
*       void InitPLCDMReg(PLCDReg *dReg);
*       bool SetPLCDReg(PLCDReg *dReg, uint16_t value);
*       uint8_t ReadPLCDReg(PLCDReg *dReg); //Blocking function call to read D Reg value, not a thradsafe !!!
*
* NOTES :
* AUTHOR :    Kris Jearakul        START DATE :    25 Oct 2020
*
* CHANGES :
*
* Copyright (C) 2020  Somewhere in Morgan Hill, CA 
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>. 
*
*H*/
#ifndef PLC_MODBUS_DOT_H
#define PLC_MODBUS_DOT_H
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif


#define HIGH_BYTE(b) (uint8_t)(b >> 8)
#define LOW_BYTE(b) (uint8_t)(b & 0xff)
#define PLC_BAUDRATE 38400
#define PKG_TIMEOUT 25   //If you are using 19200 baudrate change to atlease 50 !!!.

typedef void(*plc_read_cb)(void *arg);

typedef enum 
{
    FrameIdle,
    FrameInQueue,
    FrameTX,
    FrameRX,
    FrameTimeout
}FrameState;
typedef struct 
{
    uint8_t address;
    uint8_t command;
    uint8_t data[6]; //Support 32 bit DREG read
    uint8_t data_length;
    uint8_t crc_hi;
    uint8_t crc_lo;
    FrameState frameState;
    plc_read_cb read_cb;
} ModBusFrame;

typedef struct 
{
    uart_port_t  uart_num;
    int tx_io_num;
    int rx_io_num;
}PLCUartSettings;


typedef enum 
{
    READ_D = 3,
    SET_D = 6,
    READ_M = 1,
    SET_M = 5,
    READ_D32 = 10, //Virtual command to read 2 DREG as 32 bits
    WRITE_D32 = 20
    
}ModBusRTUCommand;

typedef struct 
{
    uint8_t command;
    uint8_t station_number;
    uint16_t reg_number;
    int32_t value;
} PLCCommand;

typedef struct 
{
    uint16_t stationNumber;
    uint16_t address ;
    bool state;
    ModBusFrame readCommand;
    ModBusFrame writeCommand;
}PLCMReg;

typedef struct 
{    
    uint16_t stationNumber;
    uint16_t address ;
    uint16_t value;
    ModBusFrame readCommand;
    ModBusFrame writeCommand;
}PLCDReg;

typedef struct 
{
    uint16_t stationNumber;
    uint16_t address ;
    uint32_t value;
    ModBusFrame readCommand;
    ModBusFrame writeLowDCommand;
    ModBusFrame writeHiDCommand;
}PLCD32Reg;




void ModBusOneByteCRC(ModBusFrame *modBusCmd, uint8_t bData);
void ResetCRC(ModBusFrame *modBusCmd);
void CalculateCRC(ModBusFrame *modBusCmd);
void FormatReadMRegCommand(ModBusFrame *modBus,uint16_t station_number,uint16_t mRegNumber);
void FormatSetMRegCommand(ModBusFrame *modBus,uint16_t station_number,uint16_t mRegNumber, uint8_t state);
void FormatReadDRegCommand(ModBusFrame *modBus,uint16_t station_number,uint16_t dRegNumber);
void FormatSetDRegCommand(ModBusFrame *modBus,uint16_t station_number,uint16_t dRegNumber, uint16_t dRegValue);
void FormatReadD32RegCommand(ModBusFrame *modBus,uint16_t station_number,uint16_t dRegNumber);
int SendPLCFrame(uart_port_t uart_num,ModBusFrame * modBus );
void InitPLCUart(PLCUartSettings *setting);
void DebugPrint(ModBusFrame *modBus);
QueueHandle_t InitPLCTask(PLCUartSettings *plcSetting);
int ParsingPLCCommand(char *cmdStr, PLCCommand *cmd);
void HandlePLCCommand(PLCCommand *cmd,ModBusFrame *frame);
uint8_t HandleRecieveByte(ModBusFrame *modBus,char *data, uint8_t length);
void DebugPrintBytes(const char* tag,char* buffer, int length);

bool EnqueueFrame(ModBusFrame *modBus);

void InitPLCMReg(PLCMReg *mReg);
bool SetPLCMReg(PLCMReg *mReg, uint8_t state);
inline void MRegReadCallBack(PLCMReg *mReg,plc_read_cb cb) {mReg->readCommand.read_cb = cb;}
inline void MRegWriteCallBack(PLCMReg *mReg,plc_read_cb cb) {mReg->writeCommand.read_cb = cb;} 
bool ReadPLCMRegAsync(PLCMReg *mReg);
uint8_t ReadPLCMReg(PLCMReg *mReg); //Blocking function call to get the M Reg state, may be not a threadsafe !!!

void InitPLCDMReg(PLCDReg *dReg);
bool SetPLCDReg(PLCDReg *dReg, uint16_t value);
inline void DRegReadCallBack(PLCDReg *dReg,plc_read_cb cb) {dReg->readCommand.read_cb = cb;}
inline void DRegWriteCallBack(PLCDReg *dReg,plc_read_cb cb) {dReg->writeCommand.read_cb = cb;} 
bool ReadPLCDRegAsync(PLCDReg *dReg);
uint8_t ReadPLCDReg(PLCDReg *dReg); //Blocking function call to read D Reg value, may be not a thradsafe !!!

void InitPLCD32Reg(PLCD32Reg *d32Reg);
bool SetPLCD32Reg(PLCD32Reg *d32Reg, uint32_t value);
inline void D32RegReadCallBack(PLCD32Reg *d32Reg,plc_read_cb cb) {d32Reg->readCommand.read_cb = cb;}
bool ReadPLCD32RegAsync(PLCD32Reg *d32Reg);
uint8_t ReadPLCD32Reg(PLCD32Reg *d32Reg); //Blocking function call to read D Reg value, may be not a thradsafe !!!


#ifdef __cplusplus
}
#endif
#endif