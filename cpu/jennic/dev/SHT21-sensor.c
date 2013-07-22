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
 * Author(s): DAWUD GORDON <gordon@teco.edu>
 *
 * Driver for SHT21.
 */
#include "lib/sensors.h"
//#include "dev/pressure-sensor.hxxx"
#include "dev/SHT21-sensor.h"
#include "dev/temperature-sensor.h"
#include "dev/i2c.h"
#include <stdbool.h>
#include <AppHardwareApi.h>
#include <math.h>

#define SHT21_ADDR        0x80

//was: read pressure MSB 0x00
//#define SHT21_HUMIDITY    0x80

//read temp MSB

#define SHT21_START_HUMID 0xF5
#define SHT21_START_TEMP  0xF3
#define SHT21_SOFT_RESET 0xFE

#define SHT21_TEMP_OFFSET -3000.0

#define TEMPERATURE     0x01
#define HUMIDITY        0x02

static struct {
  int16_t sia0,sib1,sib2,sic12,sic11,sic22;
} __attribute((__packed__)) coeff;

static struct {
  int16_t uiPadc,uiTadc;
} __attribute((__packed__)) p;

static u16_t humidity_value;
static u16_t humidity_comp;
static u16_t temperature_value;
static u16_t temperature_comp;

static u8_t  active = 0x00;

static struct pt mplpt;

static
PT_THREAD(mplptcb(bool status))
{
  static struct ctimer timer, timer_period;
  static i2c_t t = {.cb=mplptcb,
                    .addr=SHT21_ADDR,
                    .buf={0,0,0,0,0} };
bool i;
 

  /* 1. start a measurement of ambient light and proximity
   * 2. wait until measurement is there
   * 3. read measurement
   * 4. goto 1 */
  PT_BEGIN(&mplpt);

  /* measurement cycle is at least one ms, we take 5ms here
   * to not overload the system */
  ctimer_set(&timer, CLOCK_SECOND/200, mplptcb, NULL);
  ctimer_set(&timer_period, CLOCK_SECOND, mplptcb, NULL);
  ctimer_stop(&timer);
  ctimer_stop(&timer_period);

/*
* http://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/Humidity/Sensirion_Humidity_SHT21_Datasheet_V3.pdf
*/
  

  while (active)
  {

    //ctimer_restart(&timer); 
    //PT_YIELD(&mplpt);

    t.wrlen  = 0;
    if (active&TEMPERATURE)
    {
	//write one byte with the read sensor command
	t.rdlen  = 0;
    	t.wrlen  = 1;
      	t.buf[0] = SHT21_START_TEMP;
    	i2c(&t); 
   	PT_YIELD(&mplpt); // wait for i2c command return interrupt
    	ctimer_restart(&timer); //wait for device to finish measureing
    	PT_YIELD(&mplpt); //wait till timer expires

	t.rdlen  = 3;
	t.wrlen  = 0;
	do {
		i2c(&t);
		PT_YIELD(&mplpt);
		
	} while (!status);
	//14 bit value, last 2 LSB bytes are status and need to be zeroed
      	temperature_value = t.buf[0]<<8 | t.buf[1];
	//printf("checksum1: %x\n", t.buf[2]);
	temperature_value = temperature_value & 0xfffc;
      	sensors_changed(&sht21_temperature_sensor);
    }
    if (active&HUMIDITY )
    {
	t.rdlen  = 0;
    	t.wrlen  = 1;
      	t.buf[0] = SHT21_START_HUMID;
    	i2c(&t); 
   	PT_YIELD(&mplpt);
    	ctimer_restart(&timer); 
    	PT_YIELD(&mplpt);

	t.rdlen  = 3;
	t.wrlen  = 0;
	do {
		i2c(&t);
		PT_YIELD(&mplpt);
		
	} while (!status);

	//12 bit value, last two need to be zeroed
	humidity_value = t.buf[0]<<8 | t.buf[1];
	humidity_value = humidity_value & 0xfffc;
      	sensors_changed(&sht21_humidity_sensor);
    }
	ctimer_restart(&timer_period); 
    	PT_YIELD(&mplpt);
    //PT_YIELD(&mplpt);
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
  case SHT21_TEMPERATURE_VALUE_MILLICELSIUS:
    /* original formula with floats:
     *  -46.85 + 175.72 * val * 2^16*/
	temperature_comp = ((-46.850 + 175.720 * (float)temperature_value / 65536.0 ) *1000.0);
        return (int)(temperature_comp + SHT21_TEMP_OFFSET);
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
    return active & HUMIDITY ;
  }

  return NULL;
}

static int
pvalue(int type)
{

  /* conversion code is based on
   * http://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/Humidity/Sensirion_Humidity_SHT21_Datasheet_V3.pdf
   */
  switch(type) {
  case SHT21_HUMIDITY_VALUE_PERCENT:
  	humidity_comp = (-6 + 125 * humidity_value / 65536);
	return (int)(humidity_comp*exp(-4283.78*(SHT21_TEMP_OFFSET)/(243.12+temperature_comp)/(243.12+temperature_comp+SHT21_TEMP_OFFSET)));
  }

  return 0;

}

static int
pconfigure(int type, int value)
{
static i2c_t t = {.cb=mplptcb,
                    .addr=SHT21_ADDR,
                    .buf={0,0,0,0,0} };
bool i;
  if (!value)
    return true;

  switch(type) {
  case SENSORS_HW_INIT:
  case SENSORS_ACTIVE:
    if (!active) {

	printf("init_sensor");
	t.rdlen  = 0;
	t.wrlen  = 1;
	t.buf[0] = SHT21_SOFT_RESET;
	i2c(&t); 

      active |= HUMIDITY;
      mplptcb(true);
    } else
      active |= HUMIDITY;

    return true;
  }

  return 0;
}

SENSORS_SENSOR(sht21_temperature_sensor, SHT21_TEMPERATURE_SENSOR,
               tvalue,tconfigure,tstatus);
SENSORS_SENSOR(sht21_humidity_sensor, SHT21_HUMIDITY_SENSOR,
               pvalue,pconfigure,pstatus);
