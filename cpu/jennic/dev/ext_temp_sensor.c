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

#define ADC_MAX     	4095
#define VREF		1.20
#define GAIN		3.85

static void adc_poll(){
      vAHI_AdcStartSample();
      // wait until complete
      while(!bAHI_AdcPoll());

}

static void
irq(irq_t s)
{
	float vADC, vTemp, rTemp;

 	//uint16_t adcOut;
	//adcOut = u16AHI_AdcRead();
	//printf("ADC val:%d\n",adcOut);
	//vADC = adcOut * VREF / ADC_MAX;
    adc_poll();
	vADC = u16AHI_AdcRead() * VREF / ADC_MAX;

	vTemp = (vADC / 12.3) + 0.25023;
	rTemp = 10000 / ((3/vTemp) - 1);
	_value = (int16_t) (1000 * (rTemp - 1000) / 3.85);
	printf("ADC val:%d\n",_value);
  //_value = (u16AHI_AdcRead() * TEMP_FACTOR) + TEMP_MIN;
  //_value = u16AHI_AdcRead();
  sensors_changed(&ext_temp_sensor);
}

static int
configure(int type, int value)
{
  static irq_handle_t handle = {.callback=irq, .irqsrc=IRQ_ADC1, 
                                .adc_input_range=ADC_INPUT_RANGE_1};

  switch(type) {
  case SENSORS_HW_INIT:
    
    // A1
    vAHI_DioSetDirection(0, E_AHI_DIO20_INT);
    // A0
    vAHI_DioSetDirection(0, E_AHI_DIO19_INT);
    // enable mux
    vAHI_DioSetDirection(0, E_AHI_DIO17_INT);
    //vAHI_DioSetOutput(uint32 u32On, uint32 u32Off);
    vAHI_DioSetOutput(E_AHI_DIO17_INT | E_AHI_DIO20_INT | E_AHI_DIO19_INT, 0);
  
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
    // enable admux channel 1
    vAHI_DioSetOutput(E_AHI_DIO17_INT,  E_AHI_DIO20_INT | E_AHI_DIO19_INT);
    
    _value = 0;
    return 1;

  case SENSORS_HW_EXT2:
    
    //vAHI_DioSetOutput(uint32 u32On, uint32 u32Off);
    // enable admux channel 2
    vAHI_DioSetOutput(0, E_AHI_DIO17_INT);
    vAHI_DioSetOutput(E_AHI_DIO17_INT | E_AHI_DIO19_INT , E_AHI_DIO20_INT);
    
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
  irq( (irq_t)NULL);
  switch(type) {
    case EXT_TEMPERATURE_VALUE_MILLICELSIUS:
      return _value;
    }
}

SENSORS_SENSOR(ext_temp_sensor, EXT_TEMP_SENSOR,
               value, configure, status);
