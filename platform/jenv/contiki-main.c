#include "contiki.h"
#include "contiki-net.h"
#include "bootloader.h"
#include "gdb2.h"
#include "jts.h"
#include "pff.h"
#include "net/ieee802.h"
#include "dev/leds.h"
#include "dev/lightlevel-sensor.h"
#include "dev/proximity-sensor.h"
#include "dev/l3g4200d-sensor.h"
#include "dev/temperature-sensor.h"
#include "dev/pressure-sensor.h"
#include "dev/button-sensor.h"
#include "dev/mag-sensor.h"
#include "dev/acc-sensor.h"
#include "AppHardwareApi.h"

PROCINIT(&etimer_process, &tcpip_process, &jennic_bootloader_process, &sensors_process);
SENSORS(&lightlevel_sensor, &l3g4200d_sensor, &mag_sensor, &acc_sensor, &proximity_sensor, &temperature_sensor, &pressure_sensor, &button_sensor);

void
init_net(void)
{
  uip_ipaddr_t ipaddr;
  uip_ip6addr(&ipaddr, 0xfe80, 0, 0, 0, 0, 0, 0, 0);

  /* load mac address */
  memcpy(uip_lladdr.addr, ieee_get_mac(), sizeof(uip_lladdr.addr));

#if UIP_CONF_ROUTER
  uip_ds6_prefix_add(&ipaddr, UIP_DEFAULT_PREFIX_LEN, 0, 0, 0, 0);
#else /* UIP_CONF_ROUTER */
  uip_ds6_prefix_add(&ipaddr, UIP_DEFAULT_PREFIX_LEN, 0);
#endif /* UIP_CONF_ROUTER */
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);

  printf("Tentative link-local IPv6 address ");
  {
    int i, a;
    for(a = 0; a < UIP_DS6_ADDR_NB; a++) {
      if (uip_ds6_if.addr_list[a].isused) {
        for(i = 0; i < 7; ++i) {
          printf("%02x%02x:",
                 uip_ds6_if.addr_list[a].ipaddr.u8[i * 2],
                 uip_ds6_if.addr_list[a].ipaddr.u8[i * 2 + 1]);
        }
        printf("%02x%02x\n",
               uip_ds6_if.addr_list[a].ipaddr.u8[14],
               uip_ds6_if.addr_list[a].ipaddr.u8[15]);
      }
    }
  }

  netstack_init();
}

void AppColdStart(void)
{
  static FATFS fs;
  static uint8_t i=0;

  init_hardware_baud(1000000);

  /* initialize the sd-card first and wait for power supply to stabilize */
  clock_delay(CLOCK_SECOND/5);
  switch(pf_mount(&fs)){
  case FR_OK:
    break;
  case FR_NO_FILESYSTEM:
    printf("pf_mount() failed: no filesystem\n");
    break;
  case FR_DISK_ERR:
  case FR_NOT_READY:
    printf("pf_mount() failed: no card found\n");
    break;
  default:
    printf("pf_mount() failed\n");
    break;
  }

  /* enable watchdog on JN5148, there is none on JN5139 */
  #ifdef __BA2__
    watchdog_start();
  #endif

  /* start the rest */
  process_init();
  init_net();

  /* start the main processes */
  procinit_init();
  autostart_start(autostart_processes);
  jts_init();

  leds_on(LEDS_ALL);

  /* default main loop */
  while(1)
  {
    int n = process_run();
    etimer_request_poll();

#ifdef __BA2__
    watchdog_periodic();
    //printf("events\n");
    //if (n==0) /* no pending events */
    {
      //printf("went to sleep\n");
      //vAHI_CpuDoze(); //weird problem with tcp and this func
    }
#endif
  }
}

void AppWarmStart(void)
{
  AppColdStart();
}

void uip_log(char *msg)
{
  printf(msg);
}
