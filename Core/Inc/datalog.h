/*
* Datalogger memory management
*/

#ifndef DATALOG_H
#define DATALOG_H
#include "libs.h"
typedef unsigned int uint;
typedef unsigned char uchar;
typedef unsigned long ulong;
typedef signed char schar;
typedef unsigned short ushort;
typedef unsigned int BOOL;

typedef unsigned long long u64;
typedef signed long long s64;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef u8 BYTE;
typedef u16 WORD;
typedef u32 DWORD;
typedef u64 ULONGLONG;

typedef u8 UINT8;
typedef u16 UINT16;
typedef u32 UINT32;
typedef u64 UINT64;

typedef s8 INT8;
typedef s16 INT16;
typedef s32 INT32;
typedef s64 INT64;

typedef u8 UCHAR;
typedef u16 USHORT;
typedef u32 ULONG;

typedef float FLOAT32;
typedef double FLOAT64;

#define TRUE 0x01
#define FALSE 0x00

#define DATALOG_RECORD_TYPE_HEADER 0
#define DATALOG_RECORD_TYPE_WAVEFORM 1
#define DATALOG_RECORD_TYPE_DATALOG 2
#define DATALOG_RECORD_TYPE_GPSDATA 3

#define DATALOG_HEADER_FLAGS_DIGITALFILTER  0x00000001
#define DATALOG_HEADER_FLAGS_NOGPS  0x00000002
#define DATALOG_HEADER_FLAGS_EXTENDEDHIZ  0x00000004
#define DATALOG_HEADER_FLAGS_ONFIRST  0x00000008

#define FIRMWARE_APP_MAXSIZE    0x00077E00 /* Application maximum size (512)*/
#define FIRMWARE_FLASH_END      0x0007FFFF /* Last available address in flash (512K) */
/* 512KB Blank memory reserved for future use */
#define RESERVED_BLANK_BLOCK (512*1024)
/* First available address in flash */
#define DATALOG_NORMAL_FLASH_START (FIRMWARE_FLASH_END+RESERVED_BLANK_BLOCK+1)
/* 1024KB reserved for normal speed sampling data */
#define DATALOG_NORMAL_FLASH_BYTES (1024*1024)
/* 2048KB */
#define DATALOG_NORMAL_FLASH_END (DATALOG_NORMAL_FLASH_START+DATALOG_NORMAL_FLASH_BYTES)

#define DATALOG_DATAPOINT_SIZE 17
#define DATALOG_GPSPOINT_SIZE 22
#define DATALOG_HEADER_CHECK_SIZE 256

#pragma pack(1)

typedef struct
{
  UINT8 type;
  UINT32 signature;
  UINT8 fileType;
  UINT16 firmware;
  UINT8 recordVersion;
  UINT32 stationSeries;
  UINT32 chainage;
  UINT64 fileStartDate;
  UINT16 offDuration;
  UINT16 cycleDuration;
  UINT16 wavePrintInterval;
  UINT8 meterRange;
  UINT16 onOffset;
  UINT16 offOffset;
  INT32 latitude;
  INT32 longitude;
  UINT16 altitude;
  char notes[256];
  UINT32 serialNum;
  UINT32 flags;
  UINT32 utcOffset;
  UINT16 bmmFirmware;
  UINT8 gpsType;
  char pad[195];
} T_DATALOG_HEADER;

typedef struct
{
  UINT8 type;
  UINT64 timeMilliseconds;
  INT32 acValue;
  INT32 dcValue;

} T_DATALOG_DATAPOINT;

typedef struct
{
  UINT8 type;
  UINT64 timeMilliseconds;
  INT32 latitude;
  INT32 longitude;
  UINT16 altitude;
  UINT8 satsVisible;
  UINT8 satsSolution;
  UINT8 solutionValid;
} T_DATALOG_GPSPOINT;

#pragma pack()

BOOL datalog_Initialize( void );
BOOL datalog_ResetMemory(void);
BOOL datalog_IsFull(void);
BOOL datalog_IsEmpty(void);
BOOL datalog_ClearMemoryStart(void);
BOOL datalog_ClearMemoryIteration(void);
BOOL datalog_ClearMemoryBusy(UINT8 *percentDone, BOOL *failure);
void datalog_GetStorageInfo(UINT32 *bytesUsed, UINT32 *bytesTotal);
BOOL datalog_CheckFixNextWriteLocationIsBlank(UINT8 recordType);
BOOL datalog_CreateNewHeader(void);
BOOL datalog_AddDataPointRecord(T_DATALOG_DATAPOINT *dataPoint);
BOOL datalog_AddGpsPointRecord(T_DATALOG_GPSPOINT gpsPoint);
void datalog_Iteration(void);
void datalog_QueueNormalSample(UINT64 time, float acValue, float dcValue, BOOL on);
void datalog_QueueFastSample(UINT64 time, float acValue, float dcValue);
void datalog_OutputGPSRecord(void);
void datalog_EnableDebugLogging(BOOL enable);

#endif
