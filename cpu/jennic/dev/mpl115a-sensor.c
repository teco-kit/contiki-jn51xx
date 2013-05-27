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
 * Driver for MPL115a2.
 */
#include "lib/sensors.h"
#include "dev/pressure-sensor.h"
#include "dev/temperature-sensor.h"
#include "dev/i2c.h"
#include <stdbool.h>
#include <AppHardwareApi.h>

#define MPL_ADDR        0xC0
#define MPL_PRESSURE    0x00
#define MPL_TEMPERATURE 0x02
#define MPL_COEFFICIENT 0x04
#define MPL_START_PRESS 0x10
#define MPL_START_TEMP  0x11
#define MPL_START_BOTH  0x12

#define TEMPERATURE     0x01
#define PRESSURE        0x02

static struct {
  int16_t sia0,sib1,sib2,sic12,sic11,sic22;
} __attribute((__packed__)) coeff;

static struct {
  int16_t uiPadc,uiTadc;
} __attribute((__packed__)) p;

static u16_t temperature;
static u8_t  active = 0x00;

static struct pt mplpt;

static
PT_THREAD(mplptcb(bool status))
{
  static struct ctimer timer;
  static i2c_t t = {.cb=mplptcb,
                    .addr=MPL_ADDR,
                    .buf={0,0,0,0,0} };

  /* 1. start a measurement of ambient light and proximity
   * 2. wait until measurement is there
   * 3. read measurement
   * 4. goto 1 */
  PT_BEGIN(&mplpt);

  /* measurement cycle is at least one ms, we take 5ms here
   * to not overload the system */
  ctimer_set(&timer, CLOCK_SECOND/20, mplptcb, NULL);
  //ctimer_set(&timer, CLOCK_SECOND/200, mplptcb, NULL);
  ctimer_stop(&timer);

  while (active)
  {
    t.rdlen  = 0;
    t.wrlen  = 2;
    if ((active&PRESSURE) && (active&TEMPERATURE))
      t.buf[0] = MPL_START_BOTH;
    else if (active&PRESSURE)
      t.buf[0] = MPL_START_PRESS;
    else if (active&TEMPERATURE);
      t.buf[0] = MPL_START_TEMP;
    t.buf[1]  = 0x01;

    i2c(&t); PT_YIELD(&mplpt);
    ctimer_restart(&timer); PT_YIELD(&mplpt);

    t.wrlen  = 1;
    if (active&TEMPERATURE)
    {
      t.rdlen  = 2;
      t.buf[0] = MPL_TEMPERATURE;
      i2c(&t); PT_YIELD(&mplpt);
      temperature = (t.buf[1]<<8)|t.buf[2];
      sensors_changed(&temperature_sensor);
    }
    if (active&PRESSURE)
    {
      t.rdlen  = 4;
      t.buf[0] = MPL_PRESSURE;
      i2c(&t); PT_YIELD(&mplpt);
      p.uiPadc = (t.buf[1]<<8)|t.buf[2];
      p.uiTadc = (t.buf[3]<<8)|t.buf[4];
      sensors_changed(&pressure_sensor);
    }
  }

  PT_END(&mplpt);
}

static int*
tstatus(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return active & TEMPERATURE;
  }

  return NULL;
}

static int
tvalue(int type)
{
  switch(type) {
  case TEMPERATURE_VALUE_MILLICELSIUS:
    /* original formula with floats:
     *  25 + (rreg16(MPL_TEMPERATURE) - 498)/-5.35 */
    return 25000 + ((((temperature>>6) - 498)*1000000 / -5350));
  }

  return 0;
}

static int
tconfigure(int type, int value)
{
  if (!value)
    return true;

  switch(type) {
  case SENSORS_HW_INIT:
  case SENSORS_ACTIVE:
    if (!active) {
      /* load compensation coeefficients */
      u8_t buf[1+sizeof(coeff)] = {MPL_COEFFICIENT};
      if (!i2cb(MPL_ADDR,1,sizeof(buf),buf))
        return false;
      memcpy(&coeff, &buf[1], sizeof(coeff));

      active |= TEMPERATURE;
      mplptcb(true);
    } else
      active |= TEMPERATURE;

    return true;
  }

  return 0;
}

static int*
pstatus(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return active & PRESSURE;
  }

  return NULL;
}

static int
pvalue(int type)
{
  int32_t siPcomp,
          lt1, lt2, lt3, si_c11x1, si_a11, si_c12x2,
          si_a1, si_c22x2, si_a2, si_a1x1, si_y1, si_a2x2;

  /* conversion code is based on
   * https://github.com/misenso/MPL115A2-Arduino-Library/blob/master/MPL115A2.cpp
   */
  switch(type) {
  case PRESSURE_VALUE_PASCAL:
    /* coeefficient 9 equation compensation */
    p.uiPadc = p.uiPadc >> 6;
    p.uiTadc = p.uiTadc >> 6;

    // Step 1 c11x1 = c11 * Padc
    lt1 = (signed long) coeff.sic11;
    lt2 = (signed long) p.uiPadc;
    lt3 = lt1*lt2;
    si_c11x1 = (signed long) lt3;

    // Step 2 a11 = b1 + c11x1
    lt1 = ((signed long)coeff.sib1)<<14;
    lt2 = (signed long) si_c11x1;
    lt3 = lt1 + lt2;
    si_a11 = (signed long)(lt3>>14);

    // Step 3 c12x2 = c12 * Tadc
    lt1 = (signed long) coeff.sic12;
    lt2 = (signed long) p.uiTadc;
    lt3 = lt1*lt2;
    si_c12x2 = (signed long)lt3;

    // Step 4 a1 = a11 + c12x2
    lt1 = ((signed long)si_a11<<11);
    lt2 = (signed long)si_c12x2;
    lt3 = lt1 + lt2;
    si_a1 = (signed long) lt3>>11;

    // Step 5 c22x2 = c22*Tadc
    lt1 = (signed long)coeff.sic22;
    lt2 = (signed long)p.uiTadc;
    lt3 = lt1 * lt2;
    si_c22x2 = (signed long)(lt3);

    // Step 6 a2 = b2 + c22x2
    lt1 = ((signed long)coeff.sib2<<15);
    lt2 = ((signed long)si_c22x2>1);
    lt3 = lt1+lt2;
    si_a2 = ((signed long)lt3>>16);

    // Step 7 a1x1 = a1 * Padc
    lt1 = (signed long)si_a1;
    lt2 = (signed long)p.uiPadc;
    lt3 = lt1*lt2;
    si_a1x1 = (signed long)(lt3);

    // Step 8 y1 = a0 + a1x1
    lt1 = ((signed long)coeff.sia0<<10);
    lt2 = (signed long)si_a1x1;
    lt3 = lt1+lt2;
    si_y1 = ((signed long)lt3>>10);

    // Step 9 a2x2 = a2 * Tadc
    lt1 = (signed long)si_a2;
    lt2 = (signed long)p.uiTadc;
    lt3 = lt1*lt2;
    si_a2x2 = (signed long)(lt3);

    // Step 10 pComp = y1 + a2x2
    lt1 = ((signed long)si_y1<<10);
    lt2 = (signed long)si_a2x2;
    lt3 = lt1+lt2;

    // Fixed point result with rounding
    siPcomp = ((signed long)lt3>>13);

    // decPcomp is defined as a floating point number
    // Conversion to decimal value from 1023 ADC count value
    // ADC counts are 0 to 1023, pressure is 50 to 115kPa respectively
    return (((65000/1023)*siPcomp)+50000);
  }

  return 0;
}

static int
pconfigure(int type, int value)
{
  if (!value)
    return true;

  switch(type) {
  case SENSORS_HW_INIT:
  case SENSORS_ACTIVE:
    if (!active) {
      /* load compensation coeefficients */
      u8_t buf[1+sizeof(coeff)] = {MPL_COEFFICIENT}, i;
      if (!i2cb(MPL_ADDR,1,sizeof(coeff),buf))
        return false;
      memcpy(&coeff, &buf[1], sizeof(coeff));

      active |= PRESSURE;
      mplptcb(true);
    } else
      active |= PRESSURE;

    return true;
  }

  return 0;
}

SENSORS_SENSOR(temperature_sensor, TEMPERATURE_SENSOR,
               tvalue,tconfigure,tstatus);
SENSORS_SENSOR(pressure_sensor, PRESSURE_SENSOR,
               pvalue,pconfigure,pstatus);
