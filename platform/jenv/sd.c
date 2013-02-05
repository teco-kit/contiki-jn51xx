/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for Petit FatFs (C)ChaN, 2009      */
/*-----------------------------------------------------------------------*/

#include "pff.h"
#include "sd.h"
#include "AppHardwareApi.h"

#define LOWER_CS() do { vAHI_SpiWaitBusy(); vAHI_SpiSelect(1<<1); vAHI_SpiWaitBusy(); } while(0)
#define RAISE_CS() do { vAHI_SpiWaitBusy(); vAHI_SpiSelect(0); vAHI_SpiWaitBusy(); } while(0)

/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

#define DEBUG 0

#include <stdio.h>
#if DEBUG != 0
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifndef MIN
#define MIN(a, b)_((a) < (b) ? (a) : (b))
#endif /* MIN */

#define SDCARD_SLAVE E_AHI_SPIM_SLAVE_ENBLE_0

/* SD commands */
#define GO_IDLE_STATE     0
#define SEND_OP_COND      1
#define SWITCH_FUNC       6
#define SEND_IF_COND      8
#define SEND_CSD          9
#define SEND_CID          10
#define STOP_TRANSMISSION 12
#define SEND_STATUS       13
#define READ_SINGLE_BLOCK 17
#define WRITE_BLOCK       24
#define READ_OCR          58
#define SPI_IDLE          0xff

/* SD response lengths. */
#define R1 1
#define R2 2
#define R3 5
#define R7 5

#define START_BLOCK_TOKEN 0xfe

/* Status codes returned after writing a block. */
#define DATA_ACCEPTED 2
#define DATA_CRC_ERROR 5
#define DATA_WRITE_ERROR 6

#define SD_TRANSACTION_ATTEMPTS		512
#define SD_READ_RESPONSE_ATTEMPTS	8
#define SD_READ_BLOCK_ATTEMPTS		2

#define SD_DEFAULT_BLOCK_SIZE			512

static uint16_t rw_block_size,
                block_size;

static int
send_command(uint8_t cmd, uint32_t argument)
{
  uint8_t req[6], i;

  req[0] = 0x40 | cmd;
  req[1] = argument >> 24;
  req[2] = argument >> 16;
  req[3] = argument >> 8;
  req[4] = argument;
  /* The CRC hard-wired to 0x95 is only needed for the initial 
     GO_IDLE_STATE command. */
  req[5] = 0x95;

  vAHI_SpiWaitBusy(); vAHI_SpiStartTransfer8(SPI_IDLE);
  for (i=0; i<sizeof(req); i++) {
    vAHI_SpiWaitBusy(); vAHI_SpiStartTransfer8(req[i]);
  }
  vAHI_SpiWaitBusy(); vAHI_SpiStartTransfer8(SPI_IDLE);

  PRINTF("send command: %d %d %d %d %d %d\n", req[0]-0x40,req[1],req[2],req[3],req[4],req[5]);

  return 0;
}
/*---------------------------------------------------------------------------*/
static uint8_t *
get_response(int length)
{
  int i;
  int x;
  static uint8_t r[R7];

  for(i = 0; i < SD_READ_RESPONSE_ATTEMPTS; i++) {
    vAHI_SpiStartTransfer8(0xff);
    vAHI_SpiWaitBusy();
    x = u8AHI_SpiReadTransfer8();

    if((x & 0x80) == 0) {
      /* A get_response byte is indicated by the MSB being 0. */
      r[0] = x;
      break;
    }
  }

  if(i == SD_READ_RESPONSE_ATTEMPTS) {
    PRINTF("get_response: timeout");
    return NULL;
  }

  for(i = 1; i < length; i++) {
    vAHI_SpiStartTransfer8(0xff);
    vAHI_SpiWaitBusy();
    r[i] = u8AHI_SpiReadTransfer8();
  }

  PRINTF("get_response %d", r[0]);

  return r;
}
/*---------------------------------------------------------------------------*/
static unsigned char *
transaction(int command, unsigned long argument,
  int response_type, unsigned attempts)
{
  unsigned i;
  unsigned char *r;

  r = NULL;
  for(i = 0; i < attempts; i++) {
    LOWER_CS();
    send_command(command, argument);
    r = get_response(response_type);
    RAISE_CS();
    if(r != NULL) {
      break;
    }
  }

  return r;
}

static enum { IDLE=0x00,TRANSMIT  } write_state = IDLE;
DRESULT
disk_readp2(
  int read_cmd,
  BYTE *dest,
  DWORD sect,
  DWORD offset,
  WORD count)
{
  unsigned char *r = NULL;
  int retval = RES_ERROR, i, token;

  if (write_state==TRANSMIT) {
    PRINTF("sd: read failed, write transaction in progress\n");
    return RES_NOTRDY;
  }

  PRINTF("disk_readp(%d,0x%x,%d,%d,%d)==",read_cmd, dest,sect,offset,count);

  if (read_cmd==READ_SINGLE_BLOCK && offset+count > rw_block_size) {
    PRINTF("attempting to read block (%d) bigger than rw_block_size (%d)!!"
       ,offset+count,rw_block_size);
    return 0;
  }

  /* try and send the read command */
  for(i = 0; i < SD_TRANSACTION_ATTEMPTS; i++) {
    LOWER_CS();
    send_command(read_cmd, sect*rw_block_size);
    r = get_response(R1);
    if(r != NULL && r[0]==0) {
      break;
    }
    RAISE_CS();
  }

  if (r==NULL || r[0]!=0) {
    PRINTF("error during read");
    RAISE_CS();
    return RES_ERROR;
  }

  /* We received an R1 response with no errors.
     Get a token from the card now. */
  for(i = 0; i < SD_TRANSACTION_ATTEMPTS; i++) {
    vAHI_SpiWaitBusy();
    vAHI_SpiStartTransfer8(0xff);
    vAHI_SpiWaitBusy();
    token = u8AHI_SpiReadTransfer8();
    if(token == START_BLOCK_TOKEN || (token > 0 && token <= 8))
      break;
  }

  PRINTF("returned read token: %d (%d attemtps)\n", token, i);

  if(token == START_BLOCK_TOKEN) {
    /* consume the offset */
    for(i=0; i<offset; i++) {
      vAHI_SpiStartTransfer8(0x00);
      vAHI_SpiWaitBusy();
      u8AHI_SpiReadTransfer8();
    }

    /* A start block token has been received. Read the block now. */
    for(i=0; i<count; i++) {
      vAHI_SpiStartTransfer8(0x00);
      vAHI_SpiWaitBusy();
      dest[i] = u8AHI_SpiReadTransfer8();
    }

    /* consume what is left of the block 
     * XXX: might wanna send STOP_COND here instead of consuming whole block */
    if (read_cmd==READ_SINGLE_BLOCK)
      for (i=0; i<(rw_block_size-offset-count); i++) {
        vAHI_SpiStartTransfer8(0x00);
        vAHI_SpiWaitBusy();
      }

    /* Consume CRC. TODO: Validate the block. */
    vAHI_SpiStartTransfer8(0x00); vAHI_SpiWaitBusy();
    vAHI_SpiStartTransfer8(0x00); vAHI_SpiWaitBusy();

    retval = RES_OK;
  } else if(token > 0 && token <= 8) {
    /* The card returned a data error token. */
    retval = RES_ERROR;
    if (retval & (1<<0))
      PRINTF("General Error\n");
    else if (retval & (1<<1))
      PRINTF("Internal card controller error\n");
    else if (retval & (1<<2))
      PRINTF("Card ECC error\n");
    else if (retval & (1<<3))
      PRINTF("Out of Range\n");
  } else {
    /* The card never returned a token after our read attempts. */
    retval = RES_NOTRDY;
    PRINTF("no returned read token\n");
  }

  RAISE_CS();
  return retval;
}


static int
read_register(int register_cmd, char *buf, int register_size)
{
  return disk_readp2(register_cmd, buf, 0, 0, register_size);
}


/*-----------------------------------------------------------------------*/
/* Read Partial Sector                                                   */
/*-----------------------------------------------------------------------*/

DRESULT
disk_readp(BYTE *dest, DWORD src, WORD offset, WORD count)
{
  return disk_readp2(READ_SINGLE_BLOCK, dest, src, offset, count);
}


/*-----------------------------------------------------------------------*/
/* Write Partial Sector                                                  */
/*-----------------------------------------------------------------------*/

DRESULT disk_writep (
  const BYTE* buff,   /* Pointer to the data to be written, NULL:Initiate/Finalize write operation */
  DWORD sc            /* Sector number (LBA) or Number of bytes to send */
)
{
  DRESULT res;
  uint8_t* r, data_response, status_code;
  uint16_t i;
  static uint16_t d;

  PRINTF("disk_writep(buff=0x%x, sc=%d)\n", buff, sc);

  if (!buff) {
    if (sc) { // initialize write process
      for(i = 0; i < SD_TRANSACTION_ATTEMPTS; i++) {
        if (write_state != IDLE) {
          PRINTF("sd: initialize write, write transaction already in progress\n");
          return RES_NOTRDY;
        }
        LOWER_CS();
        send_command(WRITE_BLOCK, sc*rw_block_size);
        r = get_response(R1);
        if(r != NULL && r[0] == 0) {
          PRINTF("write request OK\n");
          break;
        }
        RAISE_CS();
      }

      if (r==0) {
        PRINTF("no response to write request\n");
        return RES_NOTRDY;
      }

      /* re-select card and wait to transmit data */
      LOWER_CS();

      vAHI_SpiWaitBusy();
      vAHI_SpiStartTransfer8(START_BLOCK_TOKEN);
      vAHI_SpiWaitBusy();

      write_state = TRANSMIT;
      d=0;

      return RES_OK;
    } else {  // finalize write process
      if (write_state != TRANSMIT) {
        PRINTF("sd: finalize error, write transaction not started\n");
        return RES_NOTRDY;
      }

      PRINTF("finalizing write request after %d bytes",d);

      for(i=0;i<SD_TRANSACTION_ATTEMPTS*10; i++) {
        vAHI_SpiStartTransfer8(SPI_IDLE);
        vAHI_SpiWaitBusy();
        data_response = u8AHI_SpiReadTransfer8();
        if((data_response & 0x11) == 1) {
          /* Data response token received. */
          status_code = (data_response >> 1) & 0x7;
          if(status_code == DATA_ACCEPTED) {
            PRINTF(" OK (%d tries)\n", i);
            res = RES_OK;
            break;
          } else {
            PRINTF(" ERROR (%d tries)\n", i);
            res = RES_ERROR;
            break;
          }
        }
      }

      /* wait until card is idle */
      while (u8AHI_SpiReadTransfer8()!=SPI_IDLE) {
        vAHI_SpiStartTransfer8(SPI_IDLE);
        vAHI_SpiWaitBusy();
      }

      if (i==SD_TRANSACTION_ATTEMPTS)
        PRINTF(" ERROR (timeout %d tries)\n", i);

      RAISE_CS();
      write_state = IDLE;
    }
  } else { // Send data to the disk
    if (write_state!=TRANSMIT) {
      PRINTF("sd: write, transaction not started\n");
      return RES_NOTRDY;
    }

    for (i=0;i<sc; i++)
    {
      vAHI_SpiWaitBusy();
      vAHI_SpiStartTransfer8(buff[i]);
    }

    d+=sc;
    return RES_OK;
  }

  return res;
}


DSTATUS
disk_initialize (void)
{
  uint8_t *r=NULL, reg[16], read_bl_len;
  uint16_t i;

  /* init with slow SPI speed */
  vAHI_SpiConfigure(2,   // number of spi slave select line
                    E_AHI_SPIM_MSB_FIRST,
                    0,0, // polarity and phase
                    400,   // clock divisor, 8Mhz clock
                    E_AHI_SPIM_INT_DISABLE,
                    E_AHI_SPIM_AUTOSLAVE_DSABL);

  LOWER_CS();
  for (i=0;i<100; i++)
  { // initialize card by sending some clocks
    vAHI_SpiStartTransfer8(0xff);
    vAHI_SpiWaitBusy();
  }
  RAISE_CS();

  r = transaction(GO_IDLE_STATE, 0, R1, SD_TRANSACTION_ATTEMPTS);
  if(r != NULL) {
    PRINTF("Go-idle result: %d\n", r[0]);
  } else {
    PRINTF("Failed to get go-idle response\n");
    return STA_NOINIT;
  }

  LOWER_CS();
  for(i = 0; i < SD_TRANSACTION_ATTEMPTS; i++) {
    send_command(SEND_OP_COND, 0);
    r = get_response(R1);
    if(r != NULL && !(r[0] & 1)) {
      break;
    }
  }
  RAISE_CS();

  if(r != NULL) {
    PRINTF("OP cond: %d (%d iterations)\n", r[0], i);
  } else {
    PRINTF("Failed to get OP cond get_response\n");
  }

  for(i = 0; i < SD_TRANSACTION_ATTEMPTS; i++) {
    LOWER_CS();
    send_command(READ_OCR, 0);
    r = get_response(R3);
    RAISE_CS();
    if(r != NULL) {
      break;
    }
  }

  if(r != NULL) {
    PRINTF("OCR: %d %d %d %d %d\n", r[0], r[1], r[2], r[3], r[4]);
  }

  if(read_register(SEND_CSD, reg, sizeof(reg)) < 0) {
    PRINTF("Failed to get block size of SD card\n");
    return STA_NOINIT;
  }

  read_bl_len = reg[5] & 0x0f;
  block_size = 1 << read_bl_len;
  rw_block_size = (block_size > SD_DEFAULT_BLOCK_SIZE) ?
                   SD_DEFAULT_BLOCK_SIZE : block_size;

  PRINTF("Found block size %d\n", block_size);

  /* XXX Arbitrary wait time here. Need to investigate why this is needed. */
  clock_delay(CLOCK_SECOND/200);

  /* continue at high speed */
  vAHI_SpiConfigure(2,   // number of spi slave select line
                    E_AHI_SPIM_MSB_FIRST,
                    0,0, // polarity and phase
                    1,   // clock divisor, 8Mhz clock
                    E_AHI_SPIM_INT_DISABLE,
                    E_AHI_SPIM_AUTOSLAVE_DSABL);


  return 0;
}
