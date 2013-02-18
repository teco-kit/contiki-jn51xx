/*
 * Copyright (c) 2008
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

#include "dev/leds.h"
#include <AppHardwareApi.h>
#include "contiki-conf.h"

static unsigned int leds = 0x00;

// green led dio2
// blue led dio3
// red led dio6
// inner led dio5
// outer led dio1

#define LED_MASK ((1<<6) | (1<<3) | (1<<2) | (1<<1) | (1<<5))

/*---------------------------------------------------------------------------*/
void
leds_arch_init(void)
{
  leds_arch_set(0);
}

/*---------------------------------------------------------------------------*/
unsigned char
leds_arch_get(void)
{
  return leds;
}
/*---------------------------------------------------------------------------*/
void
leds_arch_set(unsigned char c)
{
  if (c == LEDS_ALL)
    leds = (1<<3) | (1<<2);
  else
    leds = (1<<5) | (1<<1) | (1<<6);

  if (c&LEDS_BLUE)  leds |= (1<<3); else leds &= ~(1<<3);
  if (c&LEDS_GREEN) leds |= (1<<2); else leds &= ~(1<<2);

  if (!(c&LEDS_RED))   leds |= (1<<6); else leds &= ~(1<<6);

  if (!(c&LEDS_OUTER)) leds |= (1<<1); else leds &= ~(1<<1);
  if (!(c&LEDS_INNER)) leds |= (1<<5); else leds &= ~(1<<5);

  vAHI_DioSetDirection((~leds)&LED_MASK,leds&LED_MASK);
}
