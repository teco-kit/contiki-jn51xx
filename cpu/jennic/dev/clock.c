/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This module keeps track of the local time.
 */

#include <sys/clock.h>
#include <sys/rtimer.h>
#include <stdbool.h>
#include <AppHardwareApi.h>
#include "hrclock.h"
#include "gdb2.h"

/* cpu clock of JN5139 is 16 mHz */
#define TICKS_TO_USEC  (16)
#define TICK_TIMER_MAX (0x0fffffff)

/* enable/disable interrupts */
#define ENABLE_INTERRUPTS();  {  register uint32 ru32CtrlReg; asm volatile ("l.mfspr %0, r0, 17;" :"=r"(ru32CtrlReg) : ); ru32CtrlReg |= 6; asm volatile ("l.mtspr r0, %0, 17;" : :"r"(ru32CtrlReg)); } 

#define DISABLE_INTERRUPTS(); { register uint32 ru32CtrlReg; asm volatile ("l.mfspr %0, r0, 17;" :"=r"(ru32CtrlReg) : ); ru32CtrlReg &= 0xfffffff9; asm volatile ("l.mtspr r0, %0, 17;" : :"r"(ru32CtrlReg)); }

static bool ticking = false;
static uint32_t tick_timer_interval;
static hrclock_t ticks = 0;
static uint32_t last_tick = 0;

void 
tick_timer_int(uint32 u32Device, uint32 u32ItemBitmap)
{
  ticks += tick_timer_interval - last_tick;
  last_tick = 0;
  rtimer_run_next();
}

void
clock_init()
{
  tick_timer_interval = TICK_TIMER_MAX;
  vAHI_TickTimerInterval(tick_timer_interval);
  vAHI_TickTimerWrite(0);
  vAHI_TickTimerIntEnable(1);
  vAHI_TickTimerRegisterCallback(&tick_timer_int);
  vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_RESTART);
  ticking = true;
}

clock_time_t
clock_time(void)
  /* returns the current time in milli-seconds */
{
  if(!ticking) clock_init();
  return clock_hrtime()/(1000);
}

clock_time_t
clock_seconds(void)
{
  if(!ticking) clock_init();
  return clock_hrtime()/(1000*1000ULL);
}

void
clock_delay(unsigned int i)
{
  clock_time_t start;
  if(!ticking) clock_init();
  start = clock_time();
  while(clock_time()-start<i)
    ;
}

void 
rtimer_arch_init()
{
}


rtimer_clock_t 
rtimer_arch_now()
{
  if(!ticking) clock_init();
#ifdef JENNIC_CONF_TIMESYNC 
  return clock_synced_hrtime()/(1000);
#else
  return clock_hrtime()/(1000);
#endif
}

void
rtimer_arch_schedule(rtimer_clock_t t)
{
  uint32_t temp_tick = u32AHI_TickTimerRead();
#ifdef JENNIC_CONF_TIMESYNC 
  hrclock_t usecs_required = (t * 1000) - clock_synced_hrtime();
#else
  hrclock_t usecs_required = (t * 1000) - clock_hrtime();
#endif
  uint32_t ticks_required = usecs_required * TICKS_TO_USEC;
  tick_timer_interval = temp_tick + ticks_required;
  vAHI_TickTimerInterval(tick_timer_interval);
}

/* return time in micro-seconds after this functions has completed. */
hrclock_t
clock_hrtime()
{
  DISABLE_INTERRUPTS();
  uint32_t temp_tick = u32AHI_TickTimerRead();
  if (temp_tick > last_tick) {
    ticks += (temp_tick - last_tick); 
    last_tick = temp_tick;
  } 
  ENABLE_INTERRUPTS();
  return ticks/TICKS_TO_USEC;
}


#ifdef JENNIC_CONF_TIMESYNC

#include "uip.h"
#include "ieee802.h"

static uint8_t synced = 0;
static hrclock_t offset = 0;
uint8_t clock_synced()
{
  return synced;
}

hrclock_t clock_synced_hrtime()
{
  return clock_hrtime() + offset;
}

void clock_synchronize()
{
  synced = 1;
  memcpy(&offset, UIP_ICMP6_TIMESTAMP, sizeof(hrclock_t));
  offset = offset + 1800 - ieee_get_last_timestamp();   //offset = master_clock - recved_clock + time_delay
  //printf("new offset\r\n");
}

#endif

