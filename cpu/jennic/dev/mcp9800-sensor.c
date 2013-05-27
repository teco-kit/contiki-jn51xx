/*
 * Copyright (c) 2009-2010
 * Telecooperation Office (TecO), Universitaet Karlsruhe (TH), Germany.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. Neither the name of the Universitaet Karlsruhe (TH) nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author(s): Philipp Scholl <scholl@teco.edu>
 *
 * TODO: needs ro be ported to new i2c
 */

#include "dev/temperature-sensor.h"
#include "dev/i2c.h"
#include <AppHardwareApi.h>
#include <stdbool.h>

#define MCP_ADDR  0x90
#define TEMP_REG  0x00
#define CONF_REG  0x01

#define SHUTDOWN  0x01

#ifdef JENNIC_CONF_MCP9800_PWRPIN
# define POWER_ON() vAHI_DioSetDirection(0x00, JENNIC_CONF_MCP9800_PWRPIN);
#else
# define POWER_ON()
#endif

static uint8_t  res        = MCP_RES_9BIT;
static uint16_t res_factor = 1000/2;
static uint16_t conf       = (CONF_REG<<8);

/* shorthand for i2c function */
// equivalent to the I2CW used in other sensors, should only be used when switching on and off
#define mcp(r,n,w,m) i2cb(MCP_ADDR,n,m,w,r)


static void mcp_cb(bool status)
{
    i2c(&mcp_txn);
    sensors_changed(&temperature_sensor);
}


static i2c_t mcp_txn = {
   .addr  = MCP_ADDR,
   .cb    = mcp_cb,
   .wrlen = 1,
   .rdlen = 2,
   .buf   = {0, 0}
};


static int
value(int type)
  /* convert value into milli-celsius */
{
  uint16_t val = *((uint16_t*)&mcp_txn.buf);

  //uint8_t c = TEMP_REG;
  //mcp(&c, sizeof(c), (uint8_t*) &val, sizeof(val));
  
  return  (val&(1<<15) ? -1 : 1)  *      /* sign */
         ((((val&0x7f00)>>8)*1000) +     /* decimal value */
          ((val&0xf0)>>4) * res_factor); /* mantissa */
}

static int
configure(int type, int value)
{
  switch(type) {
  case SENSORS_HW_INIT:
    POWER_ON();
    return 1;

  case SENSORS_ACTIVE:
    if (value) {
      POWER_ON();

      /* configuration will be be written by resolution configure */
      conf = (CONF_REG<<8);
      configure(MCP_CONFIGURE_RES, MCP_RES_12BIT);
      mcp_cb(true);  
    } else {
      /* mcp pin will not be switched off to avoid side effects on the tsl2550
       * sensor which is connected on the same pin */
      conf = (CONF_REG<<8) | SHUTDOWN | (res&MCP_RES_12BIT);
      mcp(&conf, sizeof(conf), NULL, 0);
    }
    return true;

  case MCP_CONFIGURE_RES:
    switch(value) {
    case MCP_RES_9BIT:
      res_factor = 1000/2;
      break;
    case MCP_RES_10BIT:
      res_factor = 1000/4;
      break;
    case MCP_RES_11BIT:
      res_factor = 1000/8;
      break;
    case MCP_RES_12BIT:
      res_factor = 1000/16;
      break;
    default:
      res = MCP_RES_12BIT;
      res_factor = 1000/16;
      break;
    }

    res  = value & MCP_RES_12BIT;
    conf = (CONF_REG<<8) | (conf&SHUTDOWN) | res;
    mcp(&conf, sizeof(conf), NULL, 0);
    return 1;
  }

  return 0;
}

static int*
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
    return 0;
  case SENSORS_READY:
    return 0;
  }

  return NULL;
}

SENSORS_SENSOR(temperature_sensor, TEMPERATURE_SENSOR,
               value, configure, status);
