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
 * Driver for external env board temperature probe.
 */

#include "dev/irq.h"
#include "dev/ext_temp_sensor.h"
#include <AppHardwareApi.h>
#include <stdbool.h>

const struct sensors_sensor ext_temp_sensor;
static volatile int _value = 0;

#define ADC_MAX       4095
#define VREF		      1.20
#define VCC           3.0 //TODO: measure VCC
#define GAIN		      12.3
#define NUM_SAMPLES   16

//returns only 10bit!!
static uint16_t adc_poll(){
      vAHI_AdcStartSample();
      // wait until complete
      while(!bAHI_AdcPoll());
      return u16AHI_AdcRead();
}

#define abs_diff(a,b) ((a>b)?(a-b):(b-a))

static uint16_t
adc_majority(uint16_t epsilon)
{
  //assert(num<(1<<(16-10))&&"no overflow of uint16_t");
  static struct {
  	uint16_t mean;
	uint8_t count;
	} c[NUM_SAMPLES];

  uint16_t adcOut = 0;

  //adcOut = u16AHI_AdcRead();
  //printf("ADC val:%d\n",adcOut);
  //vADC = adcOut * VREF / ADC_MAX;
  int i,j,n=0,max=0; 

  for(i=0; i < NUM_SAMPLES; i++){
    adcOut = adc_poll();

    for(j=0; j < n ; j++){

	if(abs_diff(c[j].mean,adcOut)<epsilon) {

		c[j].mean*=c[j].count;

		c[j].mean+=adcOut;

		c[j].count+=1;
		c[j].mean/=c[j].count;

		if(c[j].count>c[max].count) max=j;
		break;
	}

    }
    if(j==n){c[j].mean=adcOut;c[j].count=1;n++;}

  }
  
  // mean over NUM_SAMPLES to reduce noise
  return c[max].mean;
}

static uint16_t
adc_mean()
{
  //assert(num<(1<<(16-10))&&"no overflow of uint16_t");
  uint16_t adcOut = 0;
  //adcOut = u16AHI_AdcRead();
  //printf("ADC val:%d\n",adcOut);
  //vADC = adcOut * VREF / ADC_MAX;
  int i;
  for(i=0; i < NUM_SAMPLES; i++){
    adcOut += adc_poll();
  }
  
  // mean over NUM_SAMPLES to reduce noise
  return adcOut / NUM_SAMPLES;
}

#define is_even(x) (!(x%2))

static uint16_t adc_median(){
	static uint16_t d[NUM_SAMPLES];
        int i, j;
        for (i = 0; i < NUM_SAMPLES; i++) {
		uint16_t tmp=adc_poll();

                for (j = i; j >= 1 && tmp < d[j-1]; j--)
                        d[j] = d[j-1];
                d[j] = tmp;
        }

	return is_even(NUM_SAMPLES)?((d[NUM_SAMPLES/2]+d[NUM_SAMPLES/2+1])/2):d[NUM_SAMPLES/2];
}

static void irq(irq_t s)
{
  _value = 0xff;
  sensors_changed(&ext_temp_sensor);
}

static uint16_t getTemp()
{
  float vADC, vTemp, rTemp;
  uint32_t adcOut=0;

  //adcOut=adc_majority(4); 
  adcOut=adc_median(); 
  //adcOut=adc_mean(); 
  
  //printf("ADC val:%d\n",adcOut);

  vADC = adcOut * VREF / ADC_MAX;

  // voltage at V+ of OpAmp
  vTemp = (vADC / GAIN) + 0.25023;
  // resistance of PT1000 element
  rTemp = 10000 / ((VCC/vTemp) - 1);
  
  // temperature in millicesius
  return (int16_t) (1000 * (rTemp - 1000) / 3.85);
  
  //_value = (u16AHI_AdcRead() * TEMP_FACTOR) + TEMP_MIN;
  //_value = u16AHI_AdcRead();
}

static int
configure(int type, int value)
{
  /*static irq_handle_t handle = {.callback=irq, .irqsrc=IRQ_ADC1, 
                                .adc_input_range=ADC_INPUT_RANGE_1};*/

  switch(type) {
  case SENSORS_HW_INIT:
    
    // A1
    vAHI_DioSetDirection(0, E_AHI_DIO20_INT);
    // A0
    vAHI_DioSetDirection(0, E_AHI_DIO19_INT);
    // enable mux
    vAHI_DioSetDirection(0, E_AHI_DIO17_INT);
    
    //vAHI_DioSetOutput(uint32 u32On, uint32 u32Off);
    //vAHI_DioSetOutput(E_AHI_DIO17_INT | E_AHI_DIO20_INT | E_AHI_DIO19_INT, 0);
    vAHI_DioSetOutput(0, E_AHI_DIO17_INT | E_AHI_DIO20_INT | E_AHI_DIO19_INT);

    vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE, E_AHI_AP_INT_DISABLE,
                   E_AHI_AP_SAMPLE_8, E_AHI_AP_CLOCKDIV_500KHZ,
                   E_AHI_AP_INTREF);

    while(!bAHI_APRegulatorEnabled())
        ; /* wait until adc powered up */
      
    vAHI_AdcEnable(E_AHI_ADC_SINGLE_SHOT,
                     ADC_INPUT_RANGE_1,
                     IRQ_ADC1);
  
    _value = 0;
    return 1;
  case SENSORS_HW_EXT1:
    //vAHI_DioSetOutput(uint32 u32On, uint32 u32Off);
    vAHI_DioSetOutput(0, E_AHI_DIO17_INT); // disable mux
    vAHI_DioSetOutput(0,  E_AHI_DIO20_INT | E_AHI_DIO19_INT); // enable mux channel 1
    vAHI_DioSetOutput(E_AHI_DIO17_INT, 0); // enable mux
    
    _value = 0;
    return 1;

  case SENSORS_HW_EXT2: 
    //vAHI_DioSetOutput(uint32 u32On, uint32 u32Off);
    vAHI_DioSetOutput(0, E_AHI_DIO17_INT); // disable mux
    vAHI_DioSetOutput(E_AHI_DIO19_INT , E_AHI_DIO20_INT); // enable mux channel 2
    vAHI_DioSetOutput(E_AHI_DIO17_INT, 0); // enable mux
    
    _value = 0;
    return 1;
    
  case SENSORS_HW_EXT3:
    //vAHI_DioSetOutput(uint32 u32On, uint32 u32Off);
    vAHI_DioSetOutput(0, E_AHI_DIO17_INT); // disable mux
    vAHI_DioSetOutput(E_AHI_DIO20_INT , E_AHI_DIO19_INT); // enable mux channel 3
    vAHI_DioSetOutput(E_AHI_DIO17_INT, 0); // enable mux
    
    _value = 0;
    return 1;

  case SENSORS_HW_EXT_REF:  
    //vAHI_DioSetOutput(uint32 u32On, uint32 u32Off);
    vAHI_DioSetOutput(0, E_AHI_DIO17_INT); // disable mux
    vAHI_DioSetOutput(E_AHI_DIO19_INT | E_AHI_DIO20_INT, 0); // enable mux channel 4 (refernce)
    vAHI_DioSetOutput(E_AHI_DIO17_INT, 0); // enable mux
    
    _value = 0;
    return 1;

  case SENSORS_HW_OFF:
    vAHI_DioSetOutput(0, E_AHI_DIO17_INT);
    return 1;
    
  case SENSORS_ACTIVE:
/*    if (value) irq_add(&handle);
    else       irq_remove(&handle);*/
    return 1;
  }

  return 0;
}

static void *
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    break;
  }

  return NULL;
}

static int
value(int type)
{
  //_value=0;
  //irq( (irq_t)NULL);
  switch(type) {
    case EXT_TEMPERATURE_VALUE_MILLICELSIUS:
      return getTemp();
     default:
	return ~0;
    }
}

SENSORS_SENSOR(ext_temp_sensor, EXT_TEMP_SENSOR,
               value, configure, status);
