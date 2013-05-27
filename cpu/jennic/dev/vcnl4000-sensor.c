/*
 * Copyright (c) 2011
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
 * Driver for Vishay VCNL4000.
 */
#include "lib/sensors.h"
#include "dev/lightlevel-sensor.h"
#include "dev/proximity-sensor.h"
#include "dev/i2c.h"
#include <stdbool.h>
#include <AppHardwareApi.h>

//#ifndef JENNIC_CONF_VCNL2550_CONVERT
//# define CONVERT_LUX 1
//#else
//# define CONVERT_LUX JENNIC_CONF_VCNL2550_CONVERT
//#endif

#define VCNL_ADDR        0x26
#define VCNL_CMD_REG     0x80
#define VCNL_PRD_ID      0x81
#define VCNL_IR_CURRENT  0x83
#define VCNL_AMB_PAR     0x84
#define VCNL_AMB_RES     0x85
#define VCNL_PROX_RES    0x87
#define VCNL_PROX_FREQ   0x89
#define VCNL_PROX_MODU   0x8A

#define MEASURE_AMBIENT   0x10
#define MEASURE_PROXIMITY 0x08
#define AMBIENT_DR        0x40
#define PROXIMITY_DR      0x20

#define VCNL_3M125        0
#define VCNL_1M5625       1
#define VCNL_781K25       2
#define VCNL_390K625      3

static uint8_t rreg(uint8_t r)
{
  uint8_t buf[] = {r,0};
  i2cb(VCNL_ADDR,1,1,buf);
  return buf[1];
}

static bool wreg(uint8_t r, uint8_t c)
{
  return I2CW(VCNL_ADDR,r,c);
}

static int*
lstatus(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
    return (int*) (rreg(VCNL_AMB_PAR) & (1<<7));
  case SENSORS_READY:
    return (int*) (rreg(VCNL_CMD_REG) & (1<<6));
  }

  return NULL;
}

static u16_t ambient_light, proximity;
static u8_t  active = 0;

static struct pt vcnlpt;

static
PT_THREAD(vcnlptcb(bool status))
{
  static i2c_t t = {.cb=vcnlptcb,
                    .addr=VCNL_ADDR,
                    .buf={0,0,0} };
  static uint8_t res=0;

  /* 1. start a measurement of ambient light and proximity
   * 2. wait until measurement is there
   * 3. read measurement
   * 4. goto 1 */
  PT_BEGIN(&vcnlpt);

  while (active)
  {
    t.rdlen  = 0;
    t.wrlen  = 2;
    t.buf[0] = VCNL_CMD_REG;
    t.buf[1] = active;

    i2c(&t); PT_YIELD(&vcnlpt);

    t.rdlen  = 1;
    t.wrlen  = 1;
    t.buf[0] = VCNL_CMD_REG;
    do {
      t.buf[1] = 0;
      i2c(&t);
      PT_YIELD(&vcnlpt);
    } while (!(t.buf[1] & (1<<5)) &&
             !(t.buf[1] & (1<<6)));

    res = t.buf[1];

    /* one of the measurments is complete,
     * store the result and ask for new measurment. */
  
    if (res & (1<<5))
    {
      t.rdlen  = 2;
      t.wrlen  = 1;
      t.buf[0] = VCNL_PROX_RES;
      i2c(&t);
      PT_YIELD(&vcnlpt);
      proximity = (t.buf[1]<<8)|t.buf[2];
      sensors_changed(&proximity_sensor);
    }
  
  
  
    // if (t.buf[1] & (1<<6)) /* Ambient light */
    if (res & (1<<6)) /* Ambient light */
    {
      t.rdlen  = 2;
      t.wrlen  = 1;
      t.buf[0] = VCNL_AMB_RES;
      i2c(&t);
      PT_YIELD(&vcnlpt);
      ambient_light = (t.buf[1]<<8)|t.buf[2];
      sensors_changed(&lightlevel_sensor);
    }
    //else if (t.buf[1] & (1<<5))
  }

  PT_END(&vcnlpt);
}

static int
lvalue(int type)
{
  switch(type) {
  case LIGHT_VALUE_VISIBLE_CENTILUX:
    /* from http://www.vishay.com/docs/83395/vcnl4000_demo_kit.pdf
     *
     * response to different light sources is different average scaling factor
     * is 4 counts per lux to convert counts to LUX and not to loose precision
     * convert it to centi (10**-2) */
    return ambient_light * 100/4;
  }

  return 0;
}

static int
lconfigure(int type, int value)
{
  switch(type) {
  case SENSORS_HW_INIT:
  case SENSORS_ACTIVE:
    if (value) /* 32 samples average, continous mode */
    {
      if (!wreg(VCNL_AMB_PAR, 0x0d))
        return false;

      if (active)
        active |= (1<<4);
      else {
        active |= (1<<4);
        vcnlptcb(true); /* make the first call to start the cycle */
      }

      return true;
    }
    else
    {
      active &= ~(1<<4);
      return true;
    }
  }

  return 0;
}

static int*
pstatus(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
    return (int*) (rreg(VCNL_IR_CURRENT)); // == zero if not
  case SENSORS_READY:
    return (int*) (rreg(VCNL_CMD_REG) & PROXIMITY_DR);
  }

  return NULL;
}

static int
pvalue(int type)
{
  switch(type) {
  case PROXIMITY_VALUE:
    return proximity;
  }

  return 0;
}

static int
pconfigure(int type, int value)
{
  switch(type) {
  case SENSORS_HW_INIT:
  case SENSORS_ACTIVE:
    if (value)
    {
      clock_delay(250); // needed, to make sure device is already awake

      if (!wreg(VCNL_IR_CURRENT, 10)) // x*10mA
        return false;

      wreg(VCNL_PROX_FREQ, VCNL_390K625);

      /* dead an delay time as documented in Vishay Datasheet pg. 9 */
      wreg(VCNL_PROX_MODU, 0x81);

      if (active)
        active |= (1<<3);
      else {
        active |= (1<<3);
        vcnlptcb(true); /* make the first call to start the cycle */
      }

      return true;
    }
    else
    {
      active &= ~(1<<3);
      wreg(VCNL_IR_CURRENT, 0); // off?!?
      return true;
    }
  }

  return 0;
}

SENSORS_SENSOR(lightlevel_sensor, LIGHTLEVEL_SENSOR,
               lvalue,lconfigure,lstatus);
SENSORS_SENSOR(proximity_sensor, PROXIMITY_SENSOR,
               pvalue,pconfigure,pstatus);
