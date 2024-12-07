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

#define DATALOG_SDLQUEUE_SIZE 100
T_DATALOG_DATAPOINT datalog_sdlqueue[DATALOG_SDLQUEUE_SIZE];
UINT8 datalog_sdlqueueOnTag[DATALOG_SDLQUEUE_SIZE];
UINT8 datalog_sdlqueuePut;
UINT8 datalog_sdlqueueGet;
UINT8 datalog_sdlqueueCount;

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
  UINT32 address, blockAddr;
  uint32_t flashId;
  UINT64 tempu64;
  UINT16 bufferOffset;
  UINT8 buffer[1024];
  UINT8 retryCount;
  UINT8 emptyPage[256];

  datalog_clearMemBusy=FALSE;
  datalog_lastTimeStampSecond=0;
  datalog_sdlqueuePut=0;
  datalog_sdlqueueGet=0;
  datalog_sdlqueueCount=0;
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
  memset(emptyPage, 256, 0xFF);
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
    for (blockAddr=0; blockAddr<1024; blockAddr+=256)
    {
      if (!memcmp(buffer+blockAddr,&emptyPage,256))
      {
        HAL_Delay(200); //Wait in case flash was busy
        bufferOffset = 0;
        for (int i = 0; i < 4; i++)
        {
          W25Q_ReadRaw((buffer+bufferOffset), 256, (address+bufferOffset));
          bufferOffset += MEM_PAGE_SIZE;
        }
        //SPI_FLASH_BufferRead(buffer, address, 1024); //Read it in a second time just to be sure
        if (!memcmp(buffer+blockAddr,&tempu64,256)) goto found; //Second time shows blank as well
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
#define DATALOG_RECORD_TYPE_HEADER 0
#define DATALOG_RECORD_TYPE_WAVEFORM 1
#define DATALOG_RECORD_TYPE_DATALOG 2
#define DATALOG_RECORD_TYPE_GPSDATA 3
BOOL datalog_CheckFixNextWriteLocationIsBlank(UINT8 recordType)
{ //Checks the next write location in dataflash to ensure it is blank.  If not,
  //the 8-byte dataslot is written with zeros and the next write location is
  //tested until a blank location is found.
  volatile BOOL passed;
  UINT8 buffer[8];
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
  UINT8 buffer00[17]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

  W25Q_WakeUP();

  while (datalog_nextWriteLocation<datalog_flashSize)
  {

    //SPI_FLASH_BufferRead(buffer, datalog_nextWriteLocation, 8);
    W25Q_ReadRawSpi(buffer, 8, datalog_nextWriteLocation);
    if (!memcmp(buffer,buffer17FF,8))
    {
      W25Q_Sleep();
      return TRUE;
    }
    //EventLog(CLASS_ERROR,PRI_SYSTEM,"Dataflash not blank when it should be at address 0x%08X",datalog_nextWriteLocation);
    //fault_Set(FAULT_DATAFLASH_FF); //Memory was not blank when it should have been
    //passed=SPI_FLASH_PageWrite((UINT8*)&buffer00, datalog_nextWriteLocation, 8); //Blank out location
    W25Q_WakeUP();
    if (W25Q_ReadRawSpi((UINT8*)&buffer00, 8, datalog_nextWriteLocation) == W25Q_OK)
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
