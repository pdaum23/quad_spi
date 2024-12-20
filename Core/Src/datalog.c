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

UINT32 datalog_currentFileNumber;
UINT32 datalog_datapointCount;
UINT32 datalog_samplesLostCount;
UINT8 datalog_sampleLost;

#define DATALOG_BMMQUEUE_SIZE 100
T_DATALOG_DATAPOINT datalog_bmmqueue[DATALOG_BMMQUEUE_SIZE];
UINT8 datalog_bmmqueueOnTag[DATALOG_BMMQUEUE_SIZE];
UINT8 datalog_bmmqueuePut;
UINT8 datalog_bmmqueueGet;
UINT8 datalog_bmmqueueCount;

#define DATALOG_FASTQUEUE_SIZE 50
T_DATALOG_DATAPOINT datalog_fastqueue[DATALOG_FASTQUEUE_SIZE];
UINT8 datalog_fastqueuePut;
UINT8 datalog_fastqueueGet;
UINT8 datalog_fastqueueCount;

T_DATALOG_GPSPOINT datalog_gpspoint;
BOOL datalog_gpsRecordOutput;

BOOL datalog_enabledlogging;
BOOL datalog_waitForOffSample;

BOOL datalog_Initialize( void )
{
  UINT32 address;
  uint32_t flashId;
  UINT8 buffer[256];
  UINT8 buffer256FF[256]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

  datalog_clearMemBusy=FALSE;
  datalog_lastTimeStampSecond=0;
  datalog_bmmqueuePut=0;
  datalog_bmmqueueGet=0;
  datalog_bmmqueueCount=0;
  datalog_fastqueuePut=0;
  datalog_fastqueueGet=0;
  datalog_fastqueueCount=0;
  datalog_currentFileNumber=0;
  datalog_gpsRecordOutput=FALSE;
  datalog_enabledlogging=FALSE;


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
  //SPI_FLASH_ClockRateFast(TRUE);
  //Now locate the first unwritten header location of 256 bytes in memory
  for (address=0; address<datalog_flashSize; address+=256)
  {
    for (int i = 0; i < 4; i++)
    {
      W25Q_ReadRaw(buffer, 256, address);
      if (!memcmp(buffer,&buffer256FF,256)) // All FFs?
      {
        break;
      }
    }
  }
  datalog_nextWriteLocation=address;
  //EventLog(CLASS_INFO,PRI_SYSTEM,"Dataflash mfg/device ID: 0x%08X, memory size: %lu, write location: %lu",flashId,datalog_flashSize,datalog_nextWriteLocation);
  W25Q_Sleep();
  // phil SPI_FLASH_ClockRateFast(FALSE);
  return TRUE;
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
        //datalog_AddNotesRecord(); //Save a base time stamp and notes at the beginning
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


BOOL datalog_CheckFixNextWriteLocationIsBlank(UINT8 recordType)
{ //Checks the next write location in dataflash to ensure it is blank.  If not,
  //the header or dataslot is written with zeros and the next write location is
  //tested until a blank location is found.
  BOOL onPageBoundary;
  UINT16 bytesToNextPage;

  UINT8 buffer[256];
  UINT8 buffer17FF[17]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  UINT8 buffer22FF[22]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                      0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  UINT8 buffer256FF[256]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                       0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  UINT8 buffer1700[17]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  UINT8 buffer2200[22]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                        0x00,0x00,0x00,0x00,0x00,0x00};
  UINT8 buffer25600[256]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};


  W25Q_WakeUP();
  onPageBoundary = FALSE;
  while (datalog_nextWriteLocation<datalog_flashSize)
  {
    if (recordType == DATALOG_RECORD_TYPE_HEADER)
    { // Move to next page boundary then start checking.
      if (onPageBoundary == FALSE)
      {
        bytesToNextPage = datalog_nextWriteLocation % MEM_PAGE_SIZE;
        datalog_nextWriteLocation += bytesToNextPage;
        onPageBoundary = TRUE;
      }
      W25Q_ReadRawSpi(buffer, 256, datalog_nextWriteLocation);
      if (!memcmp(buffer,buffer256FF,256))
      {
        W25Q_Sleep();
        return TRUE; //Page is blank can put header here.
      }
      else
      {
        W25Q_ProgramRaw(buffer25600, 256, datalog_nextWriteLocation);
        datalog_nextWriteLocation+=DATALOG_HEADER_CHECK_SIZE; // Check next page.
      }
    }
    else if (recordType == DATALOG_RECORD_TYPE_DATALOG)
    {
      bytesToNextPage = datalog_nextWriteLocation % MEM_PAGE_SIZE;
      if (bytesToNextPage < DATALOG_DATAPOINT_SIZE) // Crossing page boundary
      {
        W25Q_ReadRawSpi(buffer, bytesToNextPage, datalog_nextWriteLocation);
        W25Q_ReadRawSpi((buffer+bytesToNextPage), (DATALOG_DATAPOINT_SIZE-bytesToNextPage), (datalog_nextWriteLocation+bytesToNextPage));
        if (!memcmp(buffer,buffer17FF,DATALOG_DATAPOINT_SIZE))
        {
          W25Q_Sleep();
          return TRUE; //DATALOG_DATAPOINT_SIZE bytes blank can put data point here.
        }
        else
        {
          W25Q_ProgramRaw(buffer1700, bytesToNextPage, datalog_nextWriteLocation);
          W25Q_ProgramRaw(buffer1700, (DATALOG_DATAPOINT_SIZE-bytesToNextPage), (datalog_nextWriteLocation+bytesToNextPage));
          datalog_nextWriteLocation += DATALOG_DATAPOINT_SIZE; //Check next slot
        }
      }
      else
      {
        W25Q_ReadRawSpi(buffer, DATALOG_DATAPOINT_SIZE, datalog_nextWriteLocation);
        if (!memcmp(buffer,buffer17FF,DATALOG_DATAPOINT_SIZE))
        {
          W25Q_Sleep();
          return TRUE; //DATALOG_DATAPOINT_SIZE bytes blank can put data point here.
        }
        else
        {
          W25Q_ProgramRaw(buffer1700, DATALOG_DATAPOINT_SIZE, datalog_nextWriteLocation);
          datalog_nextWriteLocation += DATALOG_DATAPOINT_SIZE; //Check next slot
        }
      }
    }
    else if (recordType == DATALOG_RECORD_TYPE_GPSDATA)
    {
      bytesToNextPage = datalog_nextWriteLocation % MEM_PAGE_SIZE;
      if (bytesToNextPage < DATALOG_GPSPOINT_SIZE) // Crossing page boundary
      {
        W25Q_ReadRawSpi(buffer, bytesToNextPage, datalog_nextWriteLocation);
        W25Q_ReadRawSpi((buffer+bytesToNextPage), (DATALOG_GPSPOINT_SIZE-bytesToNextPage), (datalog_nextWriteLocation+bytesToNextPage));
        if (!memcmp(buffer,buffer22FF,DATALOG_GPSPOINT_SIZE))
        {
          W25Q_Sleep();
          return TRUE; //DATALOG_GPSPOINT_SIZE bytes blank can put GPS data here.
        }
        else
        {
          W25Q_ProgramRaw(buffer2200, bytesToNextPage, datalog_nextWriteLocation);
          W25Q_ProgramRaw(buffer2200, (DATALOG_GPSPOINT_SIZE-bytesToNextPage), (datalog_nextWriteLocation+bytesToNextPage));
          datalog_nextWriteLocation += DATALOG_GPSPOINT_SIZE; //Check next slot
        }
      }
      else
      {
        W25Q_ReadRawSpi(buffer, DATALOG_GPSPOINT_SIZE, datalog_nextWriteLocation);
        if (!memcmp(buffer,buffer22FF,DATALOG_GPSPOINT_SIZE))
        {
          W25Q_Sleep();
          return TRUE; //DATALOG_GPSPOINT_SIZE bytes blank can put GPS data here.
        }
        else
        {
          W25Q_ProgramRaw(buffer2200, DATALOG_GPSPOINT_SIZE, datalog_nextWriteLocation);
          datalog_nextWriteLocation += DATALOG_GPSPOINT_SIZE; //Check next slot
        }
      }
    }
  }
  W25Q_Sleep();
  return FALSE; //Out of memory
}

BOOL datalog_CreateNewHeader(void)
{
  T_DATALOG_HEADER dataHeader;

  datalog_bmmqueuePut=0;
  datalog_bmmqueueGet=0;
  datalog_bmmqueueCount=0;
  datalog_fastqueuePut=0;
  datalog_fastqueueGet=0;
  datalog_fastqueueCount=0;

  //Write out the file headers
  dataHeader.type=DATALOG_RECORD_TYPE_HEADER;
  dataHeader.signature=0xF0615D82; // Change this is SPI
  dataHeader.fileType=DATALOG_RECORD_TYPE_DATALOG;
  dataHeader.firmware=0;
  dataHeader.recordVersion=0;
  dataHeader.stationSeries=0;
  dataHeader.chainage=0;
  dataHeader.fileStartDate=0; //timer_GetMillisecondTime();
  dataHeader.offDuration=0;
  dataHeader.cycleDuration=0;
  dataHeader.wavePrintInterval=0;
  dataHeader.meterRange=0;
  dataHeader.latitude=0; //gpsData.latitude;
  dataHeader.longitude=0; //gpsData.longitude;
  dataHeader.altitude=0; //(UINT32)(gpsData.altitude/100);
  dataHeader.onOffset=0;
  dataHeader.offOffset=0;
  memset(dataHeader.notes,0,256);
  dataHeader.serialNum=0;
  dataHeader.flags=0;
  dataHeader.utcOffset=0;
  dataHeader.bmmFirmware=0;
  dataHeader.gpsType=0;

  datalog_CheckFixNextWriteLocationIsBlank(DATALOG_RECORD_TYPE_HEADER);

  W25Q_WakeUP();
  if (W25Q_ProgramRaw((UINT8 *)&dataHeader, 256, datalog_nextWriteLocation) != W25Q_OK)
  {
    W25Q_Sleep();
    return FALSE;
  }
  datalog_nextWriteLocation+=256;
  if (W25Q_ProgramRaw((UINT8 *)(&dataHeader+256), 256, datalog_nextWriteLocation) != W25Q_OK)
  {
    W25Q_Sleep();
    return FALSE;
  }
  datalog_nextWriteLocation+=256;
  W25Q_Sleep();
  return TRUE;
}

BOOL datalog_AddDataPointRecord(T_DATALOG_DATAPOINT *dataPoint)
{
  UINT16 bytesToNextPage;
  datalog_CheckFixNextWriteLocationIsBlank(DATALOG_RECORD_TYPE_DATALOG);
  W25Q_WakeUP();
  bytesToNextPage = datalog_nextWriteLocation % MEM_PAGE_SIZE;
  if (bytesToNextPage < DATALOG_DATAPOINT_SIZE) // Crossing page boundary
  {
    if (W25Q_ProgramRaw((UINT8 *) &dataPoint, bytesToNextPage, datalog_nextWriteLocation) != W25Q_OK)
    {
      W25Q_Sleep();
      return FALSE;
    }
    if (W25Q_ProgramRaw((UINT8 *) (&dataPoint+bytesToNextPage), (DATALOG_DATAPOINT_SIZE-bytesToNextPage), (datalog_nextWriteLocation+bytesToNextPage)) != W25Q_OK)
    {
      W25Q_Sleep();
      return FALSE;
    }
  }
  else
  {
    if (W25Q_ProgramRaw((UINT8 *) &dataPoint, DATALOG_DATAPOINT_SIZE, datalog_nextWriteLocation) != W25Q_OK)
    {
      W25Q_Sleep();
      return FALSE;
    }
  }
  datalog_nextWriteLocation += DATALOG_DATAPOINT_SIZE;
  W25Q_Sleep();
  return TRUE;
}

BOOL datalog_AddGpsPointRecord(T_DATALOG_GPSPOINT dataPoint)
{
  UINT16 bytesToNextPage;
  datalog_CheckFixNextWriteLocationIsBlank(DATALOG_RECORD_TYPE_GPSDATA);
  W25Q_WakeUP();
  bytesToNextPage = datalog_nextWriteLocation % MEM_PAGE_SIZE;
  if (bytesToNextPage < DATALOG_GPSPOINT_SIZE) // Crossing page boundary
  {
    if (W25Q_ProgramRaw((UINT8 *) &dataPoint, bytesToNextPage, datalog_nextWriteLocation) != W25Q_OK)
    {
      W25Q_Sleep();
      return FALSE;
    }
    if (W25Q_ProgramRaw((UINT8 *) (&dataPoint+bytesToNextPage), (DATALOG_GPSPOINT_SIZE-bytesToNextPage), (datalog_nextWriteLocation+bytesToNextPage)) != W25Q_OK)
    {
      W25Q_Sleep();
      return FALSE;
    }
  }
  else
  {
    if (W25Q_ProgramRaw((UINT8 *) &dataPoint, DATALOG_GPSPOINT_SIZE, datalog_nextWriteLocation) != W25Q_OK)
    {
      W25Q_Sleep();
      return FALSE;
    }
  }
  datalog_nextWriteLocation += DATALOG_GPSPOINT_SIZE;
  W25Q_Sleep();
  return TRUE;
}

void datalog_QueueNormalSample(UINT64 time, float acValue, float dcValue, BOOL on)
{ //This is called in an interrupt context
  if (datalog_bmmqueueCount<DATALOG_BMMQUEUE_SIZE)
  {
    datalog_bmmqueue[datalog_bmmqueuePut].type=DATALOG_RECORD_TYPE_DATALOG;
    datalog_bmmqueue[datalog_bmmqueuePut].timeMilliseconds=time;
    datalog_bmmqueue[datalog_bmmqueuePut].acValue=(INT32)(acValue*1000);
    datalog_bmmqueue[datalog_bmmqueuePut].dcValue=(INT32)(dcValue*1000);
    datalog_bmmqueueOnTag[datalog_bmmqueuePut++]=on;
    if (datalog_bmmqueuePut>=DATALOG_BMMQUEUE_SIZE) datalog_bmmqueuePut=0;
    datalog_bmmqueueCount++;
  }
  else
  {
    datalog_samplesLostCount++;
    datalog_sampleLost=1;
  }
  //printf("Normal, dc=%.3f, ac=%.3f\r\n",dcValue,acValue);
}

void datalog_QueueFastSample(UINT64 time, float acValue, float dcValue)
{ //This is called in an interrupt context
  if (datalog_fastqueueCount<DATALOG_FASTQUEUE_SIZE)
  {
    datalog_fastqueue[datalog_fastqueuePut].type=DATALOG_RECORD_TYPE_WAVEFORM;
    datalog_fastqueue[datalog_fastqueuePut].timeMilliseconds=time;
    datalog_fastqueue[datalog_fastqueuePut].acValue=(INT32)(acValue*1000);
    datalog_fastqueue[datalog_fastqueuePut++].dcValue=(INT32)(dcValue*1000);
    if (datalog_fastqueuePut>=DATALOG_FASTQUEUE_SIZE) datalog_fastqueuePut=0;
    datalog_fastqueueCount++;
  }
  //printf("Fast, dc=%.3f, ac=%.3f\r\n",dcValue,acValue);
}

void datalog_OutputGPSRecord(void)
{
  if (1) // phil test for GPS time valid (Gpsprot_GetGpsType()!=GPS_TYPE_NOGPS)
  {
    datalog_gpspoint.type=DATALOG_RECORD_TYPE_GPSDATA;
    datalog_gpspoint.timeMilliseconds= 0; //timer_GetMillisecondTime();;
    datalog_gpspoint.latitude=0; //gpsData.latitude;
    datalog_gpspoint.longitude=0; //gpsData.longitude;
    datalog_gpspoint.altitude=0; //(UINT16)(gpsData.altitude/100);
    datalog_gpspoint.satsVisible=0; //(gpsData.satsVisible<gpsData.sats)?gpsData.sats:gpsData.satsVisible;
    datalog_gpspoint.satsSolution=0; //gpsData.sats;
    datalog_gpspoint.solutionValid=0; //gpsData.solValid;
    datalog_gpsRecordOutput=TRUE;
  }
}

void datalog_Iteration(void)
{ //This routine writes out queued samples
  BOOL sendOneDataPoint;
  int count;
  UINT32 lostCount;

  if (datalog_sampleLost)
  {
    datalog_sampleLost=0;
    lostCount=datalog_samplesLostCount;
    //printf("Data sample lost(%lu)\r\n",lostCount);
  }

  if (datalog_fastqueueCount||datalog_bmmqueueCount||datalog_gpsRecordOutput)
  {
    if (datalog_gpsRecordOutput)
    {
      datalog_gpsRecordOutput=FALSE;
      if (datalog_AddGpsPointRecord(datalog_gpspoint) == TRUE)
      {
        //if (datalog_enabledlogging) printf("GPS, timer=%llu, lat=%li, long=%li, alt=%u\r\n",datalog_gpspoint.timeMilliseconds,datalog_gpspoint.latitude, datalog_gpspoint.longitude, datalog_gpspoint.altitude);
        datalog_datapointCount++;
      }
      else
      {
        //printf("Write GPS point failed!\r\n");
      }
    }

    while (datalog_bmmqueueCount)
    {
      if (!datalog_bmmqueueOnTag[datalog_bmmqueueGet]&&datalog_waitForOffSample)
      {
        datalog_waitForOffSample=FALSE; //The sample is an off already
      }
      if (datalog_waitForOffSample) //Are we waiting still waiting for an off
      {
        //printf("Skipping sample write for synchronization.\r\n");
      }
      else
      {
        if (datalog_AddDataPointRecord((&datalog_bmmqueue[datalog_bmmqueueGet])) == TRUE)
        {

          if (datalog_bmmqueueGet>=DATALOG_BMMQUEUE_SIZE)
          {
            datalog_bmmqueueGet=0;
          }
          //if (datalog_enabledlogging)printf("Normal, timer=%llu, dc=%li, ac=%li, on=%u\r\n",datalog_bmmqueue[datalog_bmmqueueGet].timeMilliseconds,datalog_bmmqueue[datalog_bmmqueueGet].dcValue,datalog_bmmqueue[datalog_bmmqueueGet].acValue,datalog_bmmqueueOnTag[datalog_bmmqueueGet]);
        }
        else
        {
          //printf("Write data point failed!\r\n");
        }
        datalog_bmmqueueGet++;
        if (datalog_bmmqueueGet>=DATALOG_BMMQUEUE_SIZE)
        {
          datalog_bmmqueueGet=0;
        }
        datalog_bmmqueueCount--;
        datalog_datapointCount++;
      }
    }
    while (datalog_fastqueueCount)
    {
      if (datalog_AddDataPointRecord((&datalog_fastqueue[datalog_fastqueueGet])) == TRUE)
      {

        if (datalog_fastqueueGet>=DATALOG_FASTQUEUE_SIZE)
        {
          datalog_fastqueueGet=0;
        }
        //if (datalog_enabledlogging)printf("Normal, timer=%llu, dc=%li, ac=%li, on=%u\r\n",datalog_fastqueue[datalog_fastqueueGet].timeMilliseconds,datalog_fastqueue[datalog_fastqueueGet].dcValue,datalog_fastqueue[datalog_fastqueueGet].acValue,datalog_fastqueueOnTag[datalog_fastqueueGet]);
      }
      else
      {
        //printf("Write data point failed!\r\n");
      }
      datalog_fastqueueGet++;
      if (datalog_fastqueueGet>=DATALOG_FASTQUEUE_SIZE)
      {
        datalog_fastqueueGet=0;
      }
      datalog_fastqueueCount--;
      datalog_datapointCount++;
    }
  }
}
