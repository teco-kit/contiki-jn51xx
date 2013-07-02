/*
 * Copyright (c) 2009
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
 */

#include "i2c.h"
#include <AppHardwareApi.h>
#include <stdbool.h>
#include <lib/list.h>
#include <string.h>

//***** from adc sampling (irq.c)
#include "contiki.h"
#include "dev/irq.h"
#include "sensors.h"
#define ADC_MAX     	4095
#define VREF		1.20
#define GAIN		3.85
//*****
//
//
#define I2C_10KHZ_SLOW_MODE (319)
#define I2C_100KHZ_SLOW_MODE (31)
#define I2C_400KHZ_FAST_MODE (7)

#define DEBUG 0
#define ADC_HACK 0

LIST(transactions);

PROCESS_NAME(i2c_process);
PROCESS(i2c_process, "i2c bus process");

#define FAIL   false
#define SUCCESS true

static bool volatile i2c_status = FAIL;
static volatile i2c_t *transaction;

static void
i2c_irq(u32_t src, u32_t mask)
{
  i2c_status = !bAHI_SiPollRxNack();
  process_poll(&i2c_process);
}

PROCESS_THREAD(i2c_process, ev, data)
{
  static size_t i;
#if (DEBUG==1)
  static u32_t i2c_ticks;
#endif
#if (ADC_HACK==1)
  static int dirty_i =0;
  // nice and slow adc
  vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE, E_AHI_AP_INT_DISABLE,
                   E_AHI_AP_SAMPLE_2, E_AHI_AP_CLOCKDIV_2MHZ,
                   E_AHI_AP_INTREF);

  // make adc slower..
  //*(uint32 *)0x02001f00 |= 0xf000; // enable prescaler

  while(!bAHI_APRegulatorEnabled())
    ; /* wait until adc powered up */
#endif
  PROCESS_BEGIN();

  vAHI_SiRegisterCallback(i2c_irq);
  vAHI_SiConfigure(true,true,I2C_400KHZ_FAST_MODE);

  while ((transaction=list_pop(transactions)))
  {
#if (DEBUG==1)
    printf("i2c: to 0x%x start, %d written, %d read\n",
           transaction->addr, transaction->wrlen, transaction->rdlen);
    i2c_ticks = u32AHI_TickTimerRead();
#endif

    if (transaction->wrlen) {
      /* send slave address, start condition */
      vAHI_SiWriteData8(transaction->addr);
      vAHI_SiSetCmdReg(E_AHI_SI_START_BIT,     E_AHI_SI_NO_STOP_BIT,
                       E_AHI_SI_NO_SLAVE_READ, E_AHI_SI_SLAVE_WRITE,
                       E_AHI_SI_SEND_ACK,      E_AHI_SI_NO_IRQ_ACK);
      PROCESS_YIELD_UNTIL(ev==PROCESS_EVENT_POLL);
      if (i2c_status==FAIL) { /* we only test for ACK on the first byte! */
        if (transaction->cb) transaction->cb(false);
#if (DEBUG==1)
        printf("i2c: to 0x%x failed, %d written, %d read in %d ticks\n",
               transaction->addr, transaction->wrlen, transaction->rdlen,
               u32AHI_TickTimerRead() - i2c_ticks);
#endif
        continue;
      }
    }

    /* send wr data. If repeated_start and there is something to read, send no
     * stop condition, else send a stop condition on the last byte. */
    for (i=0; i<transaction->wrlen; i++) {
      vAHI_SiWriteData8(transaction->buf[i]);
      vAHI_SiSetCmdReg(E_AHI_SI_NO_START_BIT,
                       (transaction->wrlen!=0) ? E_AHI_SI_NO_STOP_BIT : transaction->rdlen,
                       E_AHI_SI_NO_SLAVE_READ, E_AHI_SI_SLAVE_WRITE,
                       E_AHI_SI_SEND_ACK,      E_AHI_SI_NO_IRQ_ACK);
      PROCESS_YIELD_UNTIL(ev==PROCESS_EVENT_POLL);
      if (i2c_status==FAIL) { /* we only test for ACK on the first byte! */
        if (transaction->cb) transaction->cb(false);
#if (DEBUG==1)
        printf("i2c: to 0x%x failed, %d written, %d read in %d ticks\n",
               transaction->addr, transaction->wrlen, transaction->rdlen,
               (uint32_t) u32AHI_TickTimerRead() - i2c_ticks);
#endif
        continue;
      }
    }

    if (transaction->rdlen) {
      /* send slave addr, start condition */
      vAHI_SiWriteData8(transaction->addr|1);
      vAHI_SiSetCmdReg(E_AHI_SI_START_BIT,     E_AHI_SI_NO_STOP_BIT,
                       E_AHI_SI_NO_SLAVE_READ, E_AHI_SI_SLAVE_WRITE,
                       E_AHI_SI_SEND_ACK,      E_AHI_SI_NO_IRQ_ACK);
      PROCESS_YIELD_UNTIL(ev==PROCESS_EVENT_POLL);
    }

    /* read data, send stop condition on last byte, send nak on last byte when
     * end_of_transmission is set.*/
    for (i=0; i<transaction->rdlen; i++) {
      vAHI_SiSetCmdReg(E_AHI_SI_NO_START_BIT, i==transaction->rdlen-1,
                       E_AHI_SI_SLAVE_READ,   E_AHI_SI_NO_SLAVE_WRITE,
                       (i==transaction->rdlen-1) ? E_AHI_SI_SEND_NACK : E_AHI_SI_SEND_ACK,
                       E_AHI_SI_NO_IRQ_ACK);
      PROCESS_YIELD_UNTIL(ev==PROCESS_EVENT_POLL);
      transaction->buf[transaction->wrlen+i] = u8AHI_SiReadData8();
    }

#if (DEBUG==1)
    printf("i2c: to 0x%x completed, %d written, %d read in %d ticks (rd:",
           transaction->addr, transaction->wrlen, transaction->rdlen,
           (u32_t) u32AHI_TickTimerRead() - i2c_ticks);
    for (i=0; i<transaction->rdlen+transaction->wrlen; i++)
      printf("0x%x,", transaction->buf[i]);
    printf(")\n");
#endif

#if (ADC_HACK==1)
    if(dirty_i==2000){
      vAHI_AdcEnable(E_AHI_ADC_SINGLE_SHOT,
                     ADC_INPUT_RANGE_1,
                     IRQ_ADC1);
      vAHI_AdcStartSample();
      // wait until complete
      while(!bAHI_AdcPoll());
      
      float vADC, vTemp, rTemp;

      //uint16_t adcOut;
      //adcOut = u16AHI_AdcRead();
      //printf("ADC val:%d\n",adcOut);
      //vADC = adcOut * VREF / ADC_MAX;

      vADC = u16AHI_AdcRead() * VREF / ADC_MAX;

      vTemp = (vADC / 12.3) + 0.25023;
      rTemp = 10000 / ((3/vTemp) - 1);
      int _value = (int16_t) (1000 * (rTemp - 1000) / 3.85);
      //_value = (u16AHI_AdcRead() * TEMP_FACTOR) + TEMP_MIN;
      //_value = u16AHI_AdcRead();
      //sensors_changed(&ext_temp_sensor);
      printf("ext_temp_i2c: %d", _value);
      printf("\n");
      dirty_i=0;
    }
    dirty_i++;

#endif


    if (transaction->cb) transaction->cb(i2c_status==SUCCESS);
  }
  vAHI_SiConfigure(false,false,0);

  PROCESS_END();
}


void
i2c(i2c_t *t)
{
  if (!process_is_running(&i2c_process))
    list_init(transactions);

  list_add(transactions, t);

  if (!process_is_running(&i2c_process))
    process_start(&i2c_process, NULL);
}

static bool pending = false;
static void internal_cb(bool status)
{
  pending = false;
}

bool
i2cb(u8_t addr, u8_t wrlen, u8_t rdlen, u8_t buf[])
{
  static i2c_t t = {.buf = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};

  t.addr    = addr;
  t.wrlen   = wrlen;
  t.rdlen   = rdlen;
  t.cb      = internal_cb;
  memcpy(t.buf, buf, wrlen);

  i2c(&t);

  pending = true;
  while (pending) {
    if (i2c_process.needspoll) {
      process_post_synch(&i2c_process, PROCESS_EVENT_POLL, NULL);
      i2c_process.needspoll = false;
    }
  }

  memcpy(buf+wrlen,t.buf+wrlen,rdlen);

  return i2c_status;
}

bool
i2cs(u8_t addr, char *wrbuf)
{
  return i2cb(addr, strlen(wrbuf), 0, wrbuf);
}
