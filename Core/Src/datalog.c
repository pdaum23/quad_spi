/*
* Datalogger memory management
*/

#include "datalog.h"
#include "w25q_mem.h"

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

volatile UINT32 datalog_nextWriteLocation;
UINT32 datalog_flashSize;

BOOL datalog_clearMemBusy;
UINT32 datalog_clearBlockNumber;
BOOL datalog_clearMemFailure;

UINT32 datalog_lastTimeStampSecond;

void datalog_GPSThreeByte(uchar *p, float value);

BOOL datalog_Initialize( void )
{
  UINT32 address, blockAddr;
  uint32_t flashId;
  UINT64 tempu64;
  UINT16 bufferOffset;
  UINT8 buffer[1024];
  UINT8 retryCount;

  datalog_clearMemBusy=FALSE;
  datalog_lastTimeStampSecond=0;


#define SPI_FLASH_WINBOND32MB_DEVID 0x00EF4016
#define SPI_FLASH_WINBOND64MB_DEVID 0x00EF4017
#define SPI_FLASH_WINBOND128MB_DEVID 0x00EF4018
#define SPI_FLASH_WINBOND512MB_DEVID 0x00EF7020

  W25Q_WakeUP();
  flashId = 0;
  //Read in the flash ID and figure out the flash size
  W25Q_ReadJEDECID(&flashId);
  switch(flashId)
  {
  case SPI_FLASH_WINBOND32MB_DEVID:
    datalog_flashSize=(4UL*1024UL*1024UL);
    break;

  case SPI_FLASH_WINBOND64MB_DEVID:
    datalog_flashSize=(8UL*1024UL*1024UL);
    break;

  case SPI_FLASH_WINBOND128MB_DEVID:
    datalog_flashSize=(16UL*1024UL*1024UL);
    break;

  case SPI_FLASH_WINBOND512MB_DEVID:
    datalog_flashSize=(64UL*1024UL*1024UL);
    break;
  default:
    //EventLog(CLASS_ERROR,PRI_SYSTEM,"Unknown dataflash mfg/device ID: 0x%08X",flashId);
    //fault_Set(FAULT_DATAFLASH);
    datalog_nextWriteLocation=0;
    datalog_flashSize=0;
    return FALSE;
    break;
  }

  //Limit detected flash size if extended memory flag is not set in MFG options
  /* n/a to BMM1
  if (!(param_mfgdata.optionFlags&OPTIONFLAGS_EXTMEM))
  {
    if (datalog_flashSize>(4UL*1024UL*1024UL))
      datalog_flashSize=(4UL*1024UL*1024UL);
  }
  */
  retryCount=0;
retryFindStart:
  //SPI_FLASH_ClockRateFast(TRUE);
  tempu64=0xFFFFFFFFFFFFFFFF;
  //Now locate the first unwritten location in memory
  for (address=0; address<datalog_flashSize; address+=1024)
  {
    bufferOffset = 0;
    for (int i = 0; i < 4; i++)
    {
      W25Q_ReadRaw((buffer+bufferOffset), 256, (address+bufferOffset));
      bufferOffset += MEM_PAGE_SIZE;
    }
    //SPI_FLASH_BufferRead(buffer, address, 1024);
    for (blockAddr=0; blockAddr<1024; blockAddr+=8)
    {
      if (!memcmp(buffer+blockAddr,&tempu64,8))
      {
        HAL_Delay(200); //Wait in case flash was busy
        bufferOffset = 0;
        for (int i = 0; i < 4; i++)
        {
          W25Q_ReadRaw((buffer+bufferOffset), 256, (address+bufferOffset));
          bufferOffset += MEM_PAGE_SIZE;
        }
        //SPI_FLASH_BufferRead(buffer, address, 1024); //Read it in a second time just to be sure
        if (!memcmp(buffer+blockAddr,&tempu64,8)) goto found; //Second time shows blank as well
        //EventLog(CLASS_ERROR,PRI_SYSTEM,"Mismatch on second read of flash blank area.");
        if (retryCount++<2) goto retryFindStart; //No match to first read, so try again from the beginning
        //EventLog(CLASS_ERROR,PRI_SYSTEM,"Failed to identify flash blank area consistently.");
        //We're toast as we weren't able to start the flash bank area after multiple retries.  Set an error and show no flash present
        //fault_Set(FAULT_DATAFLASH_FINDSTART);
        address=0;
        blockAddr=0;
        datalog_flashSize=0;
      }
    }
  }
  blockAddr=0;
found:
  datalog_nextWriteLocation=address+blockAddr;
  //EventLog(CLASS_INFO,PRI_SYSTEM,"Dataflash mfg/device ID: 0x%08X, memory size: %lu, write location: %lu",flashId,datalog_flashSize,datalog_nextWriteLocation);
  W25Q_Sleep();
  // phil SPI_FLASH_ClockRateFast(FALSE);
  return TRUE;
}


BOOL datalog_AddBaseTimeStampRecord(UINT32 secondsJan1970, UINT8 reason)
{
  T_DATALOG_TIME_RECORD data;
  BOOL passed;
  //BOOL writeNotes;

  //EventLog(CLASS_INFO,PRI_SYSTEM,"Timestamp: %lu",secondsJan1970);

  datalog_CheckFixNextWriteLocationIsBlank();

  if (datalog_nextWriteLocation>=datalog_flashSize) return FALSE;

  //Last record in memory will be a time stamp with a full indication
  if ((datalog_nextWriteLocation+sizeof(T_DATALOG_TIME_RECORD))>=datalog_flashSize)
  {
    reason=DATALOG_TIME_REASON_MEMFULL;
  }

  //If memory was empty, then write out notes after initial time stamp
  //writeNotes=(!datalog_nextWriteLocation)?TRUE:FALSE;

  datalog_lastTimeStampSecond=secondsJan1970;

  memset(&data,0,sizeof(data));
  data.recordType=E_DATALOG_TIMESTAMP;
  data.seconds=secondsJan1970;
  data.reason=reason;
  // phil data.optionsVerHigh=((APPVERSION>>4)&0xF0)|(param_data.rangeSelect&0x03);
  // phil data.verLow=APPVERSION&0xFF;

  W25Q_WakeUP();
  if (W25Q_ReadRaw((UINT8*)&data, sizeof(data), datalog_nextWriteLocation) == W25Q_OK)
  {
    passed = TRUE;
  }
  else
  {
    passed = FALSE;
  }
  //passed=SPI_FLASH_PageWrite((UINT8*)&data, datalog_nextWriteLocation, sizeof(data));
  W25Q_Sleep();

  datalog_nextWriteLocation+=sizeof(data);

  //if (writeNotes)
  //{
  //  datalog_AddNotesRecord();
  //}

  return passed;
}

BOOL datalog_AddGPSRecord(FLOAT32 latitude, FLOAT32 longitude, UINT8 satCount)
{
  T_DATALOG_GPS_RECORD data;
  BOOL passed;

  //EventLog(CLASS_INFO,PRI_SYSTEM,"GPS: %.6f,%.6f,%hhu",latitude,longitude,satCount);

  datalog_CheckFixNextWriteLocationIsBlank();

  if (datalog_nextWriteLocation>=datalog_flashSize) return FALSE;

  //Last record in memory will be a time stamp with a full indication
  if ((datalog_nextWriteLocation+sizeof(T_DATALOG_GPS_RECORD))>=datalog_flashSize)
  {
    // phil datalog_AddBaseTimeStampRecord(time(NULL), DATALOG_TIME_REASON_MEMFULL);
    return FALSE;
  }

  //Always time stampt before a GPS record
  // phil datalog_AddBaseTimeStampRecord(rtc_Read(),DATALOG_TIME_REASON_GPS);

  memset(&data,0,sizeof(data));
  data.recordType=E_DATALOG_GPS;
  datalog_GPSThreeByte(data.latitude, latitude);
  datalog_GPSThreeByte(data.longitude, longitude);
  data.satCount=satCount;

  W25Q_WakeUP();
  //passed=SPI_FLASH_PageWrite((UINT8*)&data, datalog_nextWriteLocation, sizeof(data));
  if (W25Q_ReadRaw((UINT8*)&data, sizeof(data), datalog_nextWriteLocation) == W25Q_OK)
  {
    passed = TRUE;
  }
  else
  {
    passed = FALSE;
  }
  W25Q_Sleep();

  datalog_nextWriteLocation+=sizeof(data);

  return passed;
}

BOOL datalog_AddGPSAccuracyRecord(UINT16 hdop, UINT16 ehpe, UINT16 ehpeLimit, BOOL ehpeLimEnabled, BOOL ehpeLimTimeoutExpired)
{
  T_DATALOG_GPSACCURACY_RECORD data;
  BOOL passed;

  //EventLog(CLASS_INFO,PRI_SYSTEM,"GPS Accuracy: %.1f,%.1f,%1f,%lu,%lu",hdop/10.0,ehpe/10.0,ehpeLimit/10.0,ehpeLimEnabled?1UL:0UL,ehpeLimTimeoutExpired?1UL:0UL);

  datalog_CheckFixNextWriteLocationIsBlank();

  if (datalog_nextWriteLocation>=datalog_flashSize) return FALSE;

  //Last record in memory will be a time stamp with a full indication
  if ((datalog_nextWriteLocation+sizeof(T_DATALOG_GPSACCURACY_RECORD))>=datalog_flashSize)
  {
    // phil datalog_AddBaseTimeStampRecord(time(NULL), DATALOG_TIME_REASON_MEMFULL);
    return FALSE;
  }

  //Always time stamp before a GPS record
  // phil datalog_AddBaseTimeStampRecord(rtc_Read(),DATALOG_TIME_REASON_GPS);

  memset(&data,0,sizeof(data));
  data.recordType=E_DATALOG_GPSACCURACY;
  if (ehpeLimEnabled) data.recordType|=DATALOG_GPSACCURACY_FLAG_EHPE_LIM_ENABLED;
  if (ehpeLimTimeoutExpired) data.recordType|=DATALOG_GPSACCURACY_FLAG_EHPE_LIM_TIMEOUT;
  data.hdop=hdop;
  data.ehpe=ehpe;
  data.ehpeLimit=ehpeLimit;
  data.spare=0;

  W25Q_Sleep();
  if (W25Q_ReadRaw((UINT8*)&data, sizeof(data), datalog_nextWriteLocation) == W25Q_OK)
  {
    passed = TRUE;
  }
  else
  {
    passed = FALSE;
  }
  W25Q_WakeUP();

  datalog_nextWriteLocation+=sizeof(data);

  return passed;
}

BOOL datalog_AddMeasurementDCRecord(FLOAT32 value, UINT64 timeStamp, BOOL onPoint, BOOL intTracking, BOOL fastSample)
{
  T_DATALOG_READING_RECORD data;
  BOOL passed;
  UINT32 timeOffset;

  //EventLog(CLASS_INFO,PRI_SYSTEM,"DC reading: %.6f,%llu,%lu,%lu,%lu",value,timeStamp,onPoint,intTracking,fastSample);

  datalog_CheckFixNextWriteLocationIsBlank();

  if (datalog_nextWriteLocation>=datalog_flashSize) return FALSE;

  //Last record in memory will be a time stamp with a full indication
  if ((datalog_nextWriteLocation+sizeof(T_DATALOG_READING_RECORD))>=datalog_flashSize)
  {
    // phil datalog_AddBaseTimeStampRecord(time(NULL), DATALOG_TIME_REASON_MEMFULL);
    return FALSE;
  }

  //If our timestamp does not fit in the offset, write a new seconds time stamp
  if ((timeStamp-(UINT64)datalog_lastTimeStampSecond*1000)>16777214)
  {
    if (!datalog_AddBaseTimeStampRecord(timeStamp/1000,DATALOG_TIME_REASON_OFFSET))
    {
      return FALSE;
    }
  }

  timeOffset=timeStamp-(UINT64)datalog_lastTimeStampSecond*1000;

  memset(&data,0,sizeof(data));
  data.recordType=fastSample?E_DATALOG_READINGDCFAST:(intTracking?E_DATALOG_INTREADINGDC:E_DATALOG_READINGDC);
  if (onPoint) data.recordType|=DATALOG_READING_FLAG_ON;
  // phil if (timeLock) data.recordType|=DATALOG_READING_FLAG_TIMELOCK;
  memcpy(data.timeOffsetMs,&timeOffset,3);
  // phil if(param_data.configFlags&CONFIGFLAGS_DC_AMPS)
  {
    data.recordType|=DATALOG_READING_FLAG_AMPS;
  }
  data.value=value;

  W25Q_WakeUP();
  if (W25Q_ReadRaw((UINT8*)&data, sizeof(data), datalog_nextWriteLocation) == W25Q_OK)
  {
    passed = TRUE;
  }
  else
  {
    passed = FALSE;
  }
  W25Q_Sleep();

  datalog_nextWriteLocation+=sizeof(data);

  return passed;
}

BOOL datalog_AddMeasurementACRecord(FLOAT32 value, UINT64 timeStamp, BOOL onPoint, BOOL intTracking)
{
  T_DATALOG_READING_RECORD data;
  BOOL passed;
  UINT32 timeOffset;

  //EventLog(CLASS_INFO,PRI_SYSTEM,"AC reading: %.6f,%llu,%lu,%lu",value,timeStamp,onPoint,intTracking);

  datalog_CheckFixNextWriteLocationIsBlank();

  //Last record in memory will be a time stamp with a full indication
  if ((datalog_nextWriteLocation+sizeof(T_DATALOG_READING_RECORD))>=datalog_flashSize)
  {
    // phil datalog_AddBaseTimeStampRecord(time(NULL), DATALOG_TIME_REASON_MEMFULL);
    return FALSE;
  }

  if (datalog_nextWriteLocation>=datalog_flashSize) return FALSE;

  //If our timestamp does not fit in the offset, write a new seconds time stamp
  if ((timeStamp-(UINT64)datalog_lastTimeStampSecond*1000)>16777214)
  {
    if (!datalog_AddBaseTimeStampRecord(timeStamp/1000,DATALOG_TIME_REASON_OFFSET))
    {
      return FALSE;
    }
  }

  timeOffset=timeStamp-(UINT64)datalog_lastTimeStampSecond*1000;

  memset(&data,0,sizeof(data));
  data.recordType=intTracking?E_DATALOG_INTREADINGAC:E_DATALOG_READINGAC;
  if (onPoint) data.recordType|=DATALOG_READING_FLAG_ON;
  // phil if (timeLock) data.recordType|=DATALOG_READING_FLAG_TIMELOCK;
  memcpy(data.timeOffsetMs,&timeOffset,3);
  // phil if(param_data.configFlags&CONFIGFLAGS_AC_AMPS)
  {
    data.recordType|=DATALOG_READING_FLAG_AMPS;
  }
  data.value=value;

  W25Q_WakeUP();
  if (W25Q_ReadRaw((UINT8*)&data, sizeof(data), datalog_nextWriteLocation) == W25Q_OK)
  {
    passed = TRUE;
  }
  else
  {
    passed = FALSE;
  }
  W25Q_Sleep();

  datalog_nextWriteLocation+=sizeof(data);

  return passed;
}

BOOL datalog_AddTemperatureRecord(FLOAT32 value, UINT64 timeStamp)
{
  T_DATALOG_READING_RECORD data;
  BOOL passed;
  UINT32 timeOffset;

  //EventLog(CLASS_INFO,PRI_SYSTEM,"Temp reading: %.6f,%llu",value,timeStamp);

  datalog_CheckFixNextWriteLocationIsBlank();

  if (datalog_nextWriteLocation>=datalog_flashSize) return FALSE;

  //Last record in memory will be a time stamp with a full indication
  if ((datalog_nextWriteLocation+sizeof(T_DATALOG_READING_RECORD))>=datalog_flashSize)
  {
    // phil datalog_AddBaseTimeStampRecord(time(NULL), DATALOG_TIME_REASON_MEMFULL);
    return FALSE;
  }

  //If our timestamp does not fit in the offset, write a new seconds time stamp
  if ((timeStamp-(UINT64)datalog_lastTimeStampSecond*1000)>16777214)
  {
    if (!datalog_AddBaseTimeStampRecord(timeStamp/1000,DATALOG_TIME_REASON_OFFSET))
    {
      return FALSE;
    }
  }

  timeOffset=timeStamp-((UINT64)datalog_lastTimeStampSecond)*1000;

  memset(&data,0,sizeof(data));
  data.recordType=E_DATALOG_TEMPERATURE;
  // phil if (timeLock) data.recordType|=DATALOG_READING_FLAG_TIMELOCK;
  memcpy(data.timeOffsetMs,&timeOffset,3);
  // phil if(param_data.configFlags&CONFIGFLAGS_TEMPERATURE_FAHRENHEIT)
  {
    value=value*9.0/5.0+32.0;
    data.recordType|=DATALOG_READING_FLAG_FAHRENHEIT;
  }
  data.value=value;

  W25Q_WakeUP();
  if (W25Q_ReadRaw((UINT8*)&data, sizeof(data), datalog_nextWriteLocation) == W25Q_OK)
  {
    passed = TRUE;
  }
  else
  {
    passed = FALSE;
  }
  W25Q_Sleep();

  datalog_nextWriteLocation+=sizeof(data);

  return passed;
}

BOOL datalog_AddBatteryVoltsRecord(FLOAT32 value, UINT64 timeStamp)
{
  T_DATALOG_READING_RECORD data;
  BOOL passed;
  UINT32 timeOffset;

  //EventLog(CLASS_INFO,PRI_SYSTEM,"Battery volts reading: %.6f,%llu",value,timeStamp);

  datalog_CheckFixNextWriteLocationIsBlank();

  if (datalog_nextWriteLocation>=datalog_flashSize) return FALSE;

  //Last record in memory will be a time stamp with a full indication
  if ((datalog_nextWriteLocation+sizeof(T_DATALOG_READING_RECORD))>=datalog_flashSize)
  {
    // phil datalog_AddBaseTimeStampRecord(time(NULL), DATALOG_TIME_REASON_MEMFULL);
    return FALSE;
  }

  //If our timestamp does not fit in the offset, write a new seconds time stamp
  if ((timeStamp-(UINT64)datalog_lastTimeStampSecond*1000)>16777214)
  {
    if (!datalog_AddBaseTimeStampRecord(timeStamp/1000,DATALOG_TIME_REASON_OFFSET))
    {
      return FALSE;
    }
  }

  timeOffset=timeStamp-((UINT64)datalog_lastTimeStampSecond)*1000;

  memset(&data,0,sizeof(data));
  data.recordType=E_DATALOG_BATTERYVOLTS;
  memcpy(data.timeOffsetMs,&timeOffset,3);
  data.value=value;

  W25Q_WakeUP();
  if (W25Q_ReadRaw((UINT8*)&data, sizeof(data), datalog_nextWriteLocation) == W25Q_OK)
  {
    passed = TRUE;
  }
  else
  {
    passed = FALSE;
  }
  W25Q_Sleep();

  datalog_nextWriteLocation+=sizeof(data);

  return passed;
}

BOOL datalog_AddNotesRecord(void)
{
  T_DATALOG_NOTES_RECORD data;
  BOOL passed;
  int i, j, records;
  UINT8 *memPtr;
  BOOL end;

  //Make sure we have zero terminator somewhere in the notes
  passed=FALSE;
  // phil for (i=0; i<PARAM_NOTES_SIZE; i++) if (!param_data.notes[i]) {passed=TRUE; break;};
  if (!passed)
  {
    //EventLog(CLASS_INFO,PRI_SYSTEM,"Invalid notes in parameters--not zero terminated");
    return FALSE;
  }

  //EventLog(CLASS_INFO,PRI_SYSTEM,"Notes: %s",param_data.notes);

  //Always time stamp before a notes record group
  // phil datalog_AddBaseTimeStampRecord(rtc_Read(),DATALOG_TIME_REASON_NOTES);

  if (datalog_nextWriteLocation>=datalog_flashSize) return FALSE;
  /* phil
  memPtr=param_data.notes;
  records=PARAM_NOTES_SIZE/(sizeof(T_DATALOG_NOTES_RECORD)-1);
  if (PARAM_NOTES_SIZE%(sizeof(T_DATALOG_NOTES_RECORD)-1)) records++; //Last record not full
  */
  for (i=0; i<records; i++)
  {
    datalog_CheckFixNextWriteLocationIsBlank();

    //Last record in memory will be a time stamp with a full indication
    if ((datalog_nextWriteLocation+sizeof(T_DATALOG_NOTES_RECORD))>=datalog_flashSize)
    {
      // phil datalog_AddBaseTimeStampRecord(time(NULL), DATALOG_TIME_REASON_MEMFULL);
      return FALSE;
    }

    data.recordType=E_DATALOG_NOTES;
    if (!i) data.recordType|=DATALOG_NOTES_FLAG_START;
    memcpy(data.notes,memPtr,sizeof(T_DATALOG_NOTES_RECORD)-1);

    //If we have found the zero termination value, don't store any remaining records.
    end=FALSE;
    for (j=0; j<sizeof(T_DATALOG_NOTES_RECORD)-1; j++)
      if (!data.notes[j]) end=TRUE;

    if ((i==(records-1))||end) data.recordType|=DATALOG_NOTES_FLAG_END;

    memPtr+=7;

    W25Q_WakeUP();
    if (W25Q_ReadRaw((UINT8*)&data, sizeof(data), datalog_nextWriteLocation) == W25Q_OK)
    {
      passed = TRUE;
    }
    else
    {
      passed = FALSE;
    }
    W25Q_Sleep();

    datalog_nextWriteLocation+=sizeof(data);

    if (end) break;
  }
  return passed;
}

BOOL datalog_IsFull(void)
{
  if (datalog_nextWriteLocation>=datalog_flashSize) return TRUE;
  return FALSE;
}

BOOL datalog_IsEmpty(void)
{
  if (datalog_nextWriteLocation==0) return TRUE;
  return FALSE;
}


BOOL datalog_ClearMemoryStart(void)
{
  W25Q_WakeUP();
  datalog_clearMemFailure=FALSE;
  datalog_clearBlockNumber=0;
  datalog_clearMemBusy=TRUE;
  return TRUE;
}

BOOL datalog_ClearMemoryIteration(void)
{
  if (datalog_clearMemBusy)
  {
    if (datalog_clearBlockNumber<BLOCK_COUNT)
    {
      //EventLog(CLASS_INFO,PRI_SYSTEM,"Erasing DF 64K block at address 0x%08X",datalog_clearMemAddress);
      W25Q_WakeUP();
      if (W25Q_EraseBlock(datalog_clearBlockNumber, 64) != W25Q_OK)
      {
        datalog_clearMemFailure=TRUE;
      }
      //if (!SPI_FLASH_SectorErase(datalog_clearMemAddress, SPI_FLASH_SECTOR_64K)) datalog_clearMemFailure=TRUE;
      W25Q_Sleep();
      if (!datalog_clearBlockNumber)
      {
        datalog_nextWriteLocation=0; //Reset write pointer on first sector erase
        datalog_AddNotesRecord(); //Save a base time stamp and notes at the beginning
      }
      datalog_clearBlockNumber++;
      return TRUE;
    }
    if (!datalog_Initialize())
      {
        datalog_clearMemFailure=TRUE;
      }
  }
  return FALSE;
}

BOOL datalog_ClearMemoryBusy(UINT8 *percentDone, BOOL *failure)
{
  UINT8 percent;
  UINT32 num, den;
  float ratio;
  num = datalog_clearBlockNumber*1024*MEM_BLOCK_SIZE;
  den = datalog_flashSize+1;
  ratio = ((float)num/(float)den);
  percent = ratio*100;
  if (failure) *failure=datalog_clearMemFailure;
  if (datalog_clearMemBusy)
  {
    if (percentDone) *percentDone=percent;
    return TRUE;
  }
  if (percentDone) *percentDone=100;
  return FALSE;
}

void datalog_GetStorageInfo(UINT32 *bytesUsed, UINT32 *bytesTotal)
{
  if (bytesUsed) *bytesUsed=datalog_nextWriteLocation;
  if (bytesTotal) *bytesTotal=datalog_flashSize;
}

void datalog_GPSThreeByte(uchar *p, float value)
// Returns the value * 32768 in three bytes of p[].
{
  int n;

  n=(int)(32768.0*value);
  memcpy(p,&n,3);
}

BOOL datalog_CheckFixNextWriteLocationIsBlank(void)
{ //Checks the next write location in dataflash to ensure it is blank.  If not,
  //the 8-byte dataslot is written with zeros and the next write location is
  //tested until a blank location is found.
  volatile BOOL passed;
  UINT8 buffer[8];
  UINT8 bufferFF[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  UINT8 buffer00[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

  W25Q_WakeUP();

  while (datalog_nextWriteLocation<datalog_flashSize)
  {

    //SPI_FLASH_BufferRead(buffer, datalog_nextWriteLocation, 8);
    W25Q_ReadRaw(buffer, 8, datalog_nextWriteLocation);
    if (!memcmp(buffer,bufferFF,8))
    {
      W25Q_Sleep();
      return TRUE;
    }
    //EventLog(CLASS_ERROR,PRI_SYSTEM,"Dataflash not blank when it should be at address 0x%08X",datalog_nextWriteLocation);
    //fault_Set(FAULT_DATAFLASH_FF); //Memory was not blank when it should have been
    //passed=SPI_FLASH_PageWrite((UINT8*)&buffer00, datalog_nextWriteLocation, 8); //Blank out location
    W25Q_WakeUP();
    if (W25Q_ReadRaw((UINT8*)&buffer00, 8, datalog_nextWriteLocation) == W25Q_OK)
    {
      passed = TRUE;
    }
    else
    {
      passed = FALSE;
    }
    W25Q_Sleep();
    datalog_nextWriteLocation+=8; //Check next slot
  }

  W25Q_Sleep();
  return FALSE; //Out of memory
}
