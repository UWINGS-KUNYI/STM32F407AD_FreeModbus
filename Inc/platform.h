#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#include <stdint.h>
#include "tftp_server.h"
#include "mb.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define FLASH_ADDR_CONFIG       ADDR_FLASH_SECTOR_3
#define FLASH_ADDR_FIRMWARE     ADDR_FLASH_SECTOR_6

#define FLASH_ADDR_SAVE_ADDR    0x000C8000        //800k
#define FLASH_DATA_ADDR         0x00100000        //1M
#define FLASH_ADDR_OUT_STATA    0x00200000        //2M
#define FLASH_ADDR_END          0x01000000        //16MB

typedef struct
{
	uint8_t Modbus_Addr;
	uint8_t IP[4];
	uint8_t Net_Mask[4];
	uint8_t MACAddr[6];
	uint32_t RS485_BaudRate;
	uint8_t RS485_WordLength;
	uint8_t RS485_STOPBITS;
	uint8_t RS485_Parity;
	uint8_t APP_Update;        //有新的固件需要更新 ==1时有固件更新
	uint32_t APP_Size;
	uint32_t APP_Version;      //APP版本
	uint8_t In_Chandle_9;      //0->开关量， 1->电压， 2->电流, 3->电阻
	uint8_t In_Chandle_10;
	uint8_t In_Chandle_11;
	uint8_t In_Chandle_12;
	uint8_t In_Chandle_13;
	uint8_t In_Chandle_14;
	uint8_t In_Chandle_15;
	uint8_t In_Chandle_16;

	uint8_t Sysinfo_errror;    //无效系统信息       ==0x55时系统信息有效
} SYSINFO_TypeDef;

typedef struct
{
	uint8_t mainId;      //主设备号
	uint8_t type;        //通道类型
	uint16_t addr;
	uint16_t value;

	uint8_t hour;
	uint8_t minute;
	uint8_t second;

} TimeTable_TypeDef;

void RS485_Send(uint8_t * send_buf, uint16_t size);
void RS485_Recive(uint8_t ** recive_buf, uint8_t *size);

void Soft_Reset(void);

void PLATFORM_Init(void );
void UpdateAD(void);
//tftp
void* tftp_open(const char* fname, const char* mode, u8_t write);
void tftp_close(void* handle);
int tftp_read(void* handle, void* buf, int bytes);
int tftp_write(void* handle, struct pbuf* p);
//timetable
void TimeTable_Analysis(void );
void Chandle_TimeTable(uint8_t hour, uint8_t min, uint8_t second);
//lcd
void Sys_Analysis(void);
//clear
void ClearSysInfo(void );
//wwdg
void WWDG_Refresh(void);
#ifdef __cplusplus
}
#endif

#endif /* __PLATFORM_H__ */
