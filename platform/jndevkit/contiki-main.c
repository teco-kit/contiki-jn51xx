#include "contiki.h"
#include "contiki-net.h"
#include "bootloader.h"
#include "gdb2.h"
#include "jts.h"
#include "net/ieee802.h"
#include "dev/lightlevel-sensor.h"
#include "dev/button-sensor.h"

PROCINIT(&etimer_process, &tcpip_process, &jennic_bootloader_process,&sensors_process);
SENSORS(&lightlevel_sensor, &button_sensor);

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
  /* default startup */
  init_hardware();
  process_init();
  init_net();

  /* start the main processes */
  procinit_init();
  autostart_start(autostart_processes);
  jts_init();

  /* enable watchdog on JN5148, there is none on JN5139 */
#ifdef __BA2__
  watchdog_start();
#endif

  /* default main loop */
  while(1)
  {
    process_run();
    etimer_request_poll();

#ifdef __BA2__
    watchdog_periodic();
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
