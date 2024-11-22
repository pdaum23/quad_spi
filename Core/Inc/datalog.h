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
typedef enum
{
  E_DATALOG_TIMESTAMP, //0
  E_DATALOG_GPS, //1
  E_DATALOG_READINGDC, //2
  E_DATALOG_READINGAC, //3
  E_DATALOG_TEMPERATURE, //4
  E_DATALOG_BATTERYVOLTS, //5
  E_DATALOG_NOTES, //6
  E_DATALOG_READINGDCFAST, //7
  E_DATALOG_GPSACCURACY, //8
  E_DATALOG_INTREADINGDC, //9
  E_DATALOG_INTREADINGAC, //10
} T_DATALOG_RECORDTYPES;

#pragma pack(1)
#define DATALOG_TIME_REASON_OFFSET 0
#define DATALOG_TIME_REASON_INIT  1
#define DATALOG_TIME_REASON_CABLEINSERTED 2
#define DATALOG_TIME_REASON_CABLEREMOVED 3
#define DATALOG_TIME_REASON_GPS 4
#define DATALOG_TIME_REASON_LOWBAT 5
#define DATALOG_TIME_REASON_NOTES 6
#define DATALOG_TIME_REASON_MEMFULL 7
#define DATALOG_TIME_REASON_WAITSTART 8

#define DATALOG_OPTIONS_RANGELOW 0x00
#define DATALOG_OPTIONS_RANGEMED 0x01
#define DATALOG_OPTIONS_RANGEHI 0x02
typedef struct
{
  UINT8 recordType; /* D7-D5=record specific flags, D4-D0=record type(0)  */
  UINT32 seconds; /* Seconds from Jan 1, 1970, unix time format */
  UINT8 reason; /* Reason why timestamp record was inserted */
  UINT8 optionsVerHigh; /* Pertinent config options, D1-D0=range, D2,D3=spare, D4-D7=version high nibble*/
  UINT8 verLow; /* Low version bytes */
} T_DATALOG_TIME_RECORD;

typedef struct
{
  UINT8 recordType; /* D7-D5=record specific flags, D4-D0=record type(1)  */
  UINT8 latitude[3]; /* 3-byte fixed point encoded latitude */
  UINT8 longitude[3]; /* 3-byte fixed point encoded longitude */
  UINT8 satCount; /* Number of satellites in fix */
} T_DATALOG_GPS_RECORD;

#define DATALOG_GPSACCURACY_FLAG_EHPE_LIM_ENABLED 0x20
#define DATALOG_GPSACCURACY_FLAG_EHPE_LIM_TIMEOUT 0x40
typedef struct
{
  UINT8 recordType; /* D7-D5=record specific flags, D4-D0=record type(8)  */
  UINT16 hdop; /* fixed point 0.1m resolution */
  UINT16 ehpe; /* fixed point 0.1m resolution */
  UINT16 ehpeLimit; /* fixed point 0.1m resolution */
  UINT8 spare; /* future use */
} T_DATALOG_GPSACCURACY_RECORD;


#define DATALOG_READING_FLAG_ON 0x80
#define DATALOG_READING_FLAG_FAHRENHEIT 0x40
#define DATALOG_READING_FLAG_AMPS 0x40
#define DATALOG_READING_FLAG_TIMELOCK 0x20
typedef struct
{
  UINT8 recordType; /* D7-D5=record specific flags, D4-D0=record type(2-5,7)  */
  UINT8 timeOffsetMs[3]; /* 3-byte time offset from last timestamp record */
  FLOAT32 value; /* reading value */
} T_DATALOG_READING_RECORD;

#define DATALOG_NOTES_FLAG_START 0x80
#define DATALOG_NOTES_FLAG_END 0x40
typedef struct
{
  UINT8 recordType; /* D7-D5=record specific flags, D4-D0=record type(6)  */
  UINT8 notes[7];
} T_DATALOG_NOTES_RECORD;

#pragma pack()

BOOL datalog_Initialize( void );
BOOL datalog_AddBaseTimeStampRecord(UINT32 secondsJan1970, UINT8 reason);
BOOL datalog_AddMeasurementDCRecord(FLOAT32 value, UINT64 timeStamp, BOOL onPoint, BOOL intTracking, BOOL fastSample);
BOOL datalog_AddMeasurementACRecord(FLOAT32 value, UINT64 timeStamp, BOOL onPoint, BOOL intTracking);
BOOL datalog_AddTemperatureRecord(FLOAT32 value, UINT64 timeStamp);
BOOL datalog_AddBatteryVoltsRecord(FLOAT32 value, UINT64 timeStamp);
BOOL datalog_AddGPSRecord(FLOAT32 latitude, FLOAT32 longitude, UINT8 satCount);
BOOL datalog_AddGPSAccuracyRecord(UINT16 hdop, UINT16 ehpe, UINT16 ehpeLimit, BOOL ehpeLimEnabled, BOOL ehpeLimTimeoutExpired);
BOOL datalog_AddNotesRecord(void);
BOOL datalog_ResetMemory(void);
BOOL datalog_IsFull(void);
BOOL datalog_IsEmpty(void);
BOOL datalog_ClearMemoryStart(void);
BOOL datalog_ClearMemoryIteration(void);
BOOL datalog_ClearMemoryBusy(UINT8 *percentDone, BOOL *failure);
void datalog_GetStorageInfo(UINT32 *bytesUsed, UINT32 *bytesTotal);
BOOL datalog_CheckFixNextWriteLocationIsBlank(void);

#endif
