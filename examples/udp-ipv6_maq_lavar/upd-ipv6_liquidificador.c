/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/rpl/rpl.h"
#include "dev/leds.h"

#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "dev/watchdog.h"
#include "random.h"
#include "button-sensor.h"
#include "board-peripherals.h"

#include "ti-lib.h"

#include <stdio.h>
#include <stdint.h>

#include "board.h"
#include "dev/adc-sensor.h"
#include "lib/sensors.h"
#include "lpm.h"

#include <string.h>

#define FUNC_MODE_REQUEST (0x79)
#define FUNC_MODE_RESPONSE (0x7B)
#define FUNC_SET_MODE (0x7A)

#define LED_STATE (0x7C)


#define LED_BIT_1 IOID_12
#define LED_BIT_2 IOID_27
#define LED_BIT_3 IOID_22
#define LED_BIT_4 IOID_15
#define PORTA_PWM IOID_21
// IOID_23 USADA PELO ADC

#define CONN_PORT (8802)

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[UIP_LLH_LEN + UIP_IPH_LEN])

#define MAX_PAYLOAD_LEN 120

static struct uip_udp_conn *server_conn;
static struct etimer et_tempo;
static struct etimer et_timer_ad;

static int programa_escolhido = 0;
static int programa_atual = 0;

uint8_t ledCounter=0;

/*---------------------------------------------------------------------------*/
uint8_t pwm_request_max_pm(void) {
    return LPM_MODE_DEEP_SLEEP;
}

void sleep_enter(void){
    leds_on(LEDS_RED);
}

void sleep_leave(void){
    leds_off(LEDS_RED);
}

LPM_MODULE(pwmdrive_module, pwm_request_max_pm, sleep_enter, sleep_leave, LPM_DOMAIN_PERIPH);

int16_t pwminit(int32_t freq){
    uint32_t load = 0;

    ti_lib_ioc_pin_type_gpio_output(PORTA_PWM);
    leds_off(LEDS_RED);

    /* Enable GPT0 clocks under active, sleep, deep sleep */
    ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_peripheral_sleep_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_peripheral_deep_sleep_enable(PRCM_PERIPH_TIMER0);
    ti_lib_prcm_load_set();
    while (!ti_lib_prcm_load_get());

    /* Register with LPM. This will keep the PERIPH PD powered on
    * during deep sleep, allowing the pwm to keep working while the chip is
    * being power-cycled */
    lpm_register_module(&pwmdrive_module);

    /* Drive the I/O ID with GPT0 / Timer A */
    ti_lib_ioc_port_configure_set(PORTA_PWM, IOC_PORT_MCU_PORT_EVENT0, IOC_STD_OUTPUT);

    /* GPT0 / Timer A: PWM, Interrupt Enable */
    ti_lib_timer_configure(
            GPT0_BASE,
            TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);

    /* Stop the timers */
    ti_lib_timer_disable(GPT0_BASE, TIMER_A);
    ti_lib_timer_disable(GPT0_BASE, TIMER_B);

    if (freq > 0){

        load = (GET_MCU_CLOCK / freq);
        ti_lib_timer_load_set(GPT0_BASE, TIMER_A, load);
        ti_lib_timer_match_set(GPT0_BASE, TIMER_A, load-1);

        /* Start */
        ti_lib_timer_enable(GPT0_BASE, TIMER_A);
    }

    return load;
}

/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
    char buf[MAX_PAYLOAD_LEN];
    char* msg = (char*)uip_appdata;
    int i;

    if(uip_newdata()) {
        leds_toggle(LEDS_RED);
        ((char *)uip_appdata)[uip_datalen()] = 0;
        PRINTF("Server received: '%s' from ", msg);
        PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
        PRINTF("\n");

        //uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
        //PRINTF("Responding with message: ");
        //sprintf(buf, "Hello from the server! (%d)", ++seq_id);
        //PRINTF("%s\n", buf);


        switch (msg[0]) {

            case FUNC_MODE_REQUEST: {
                PRINTF("Solicitação do Estado do Liquidificador\n");

                //Monta um FUNC_MODE_RESPONSE e envia para o nó solicitante
                uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
                server_conn->rport = UIP_UDP_BUF->destport;
                buf[0] = FUNC_MODE_RESPONSE;
                buf[1] = programa_atual;
                uip_udp_packet_send(server_conn, buf, 2);
                PRINTF("Enviando FUNC_MODE_RESPONSE para [");
                PRINT6ADDR(&server_conn->ripaddr);
                PRINTF("]:%u\n", UIP_HTONS(server_conn->rport));
                /* Restore server connection to allow data from any node */
                uip_create_unspecified(&server_conn->ripaddr);
                server_conn->rport = 0;
                break;
            }
            case FUNC_SET_MODE: {
                PRINTF("Setar modo de funcionamento \n");
                PRINTF("Modo: %d", &msg[1]);
                programa_escolhido = msg[1];
                break;
            }

            default: {
                PRINTF("Comando Invalido: ");
                for(i=0;i<uip_datalen();i++) {
                    PRINTF("0x%02X ",msg[i]);
                }
                PRINTF("\n");
                break;
            }


        /*
        *case LED_TOGGLE_REQUEST: {
        *    PRINTF("LED_TOGGLE_REQUEST\n");
        *    //Monta um LED_SET_STATE e envia para o nó solicitante
        *    uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
        *    server_conn->rport = UIP_UDP_BUF->destport;
        *    buf[0] = LED_SET_STATE;
        *    buf[1] = (ledCounter++)&0x03;
        *    uip_udp_packet_send(server_conn, buf, 2);
        *    PRINTF("Enviando LED_SET_STATE para [");
        *    PRINT6ADDR(&server_conn->ripaddr);
        *    PRINTF("]:%u\n", UIP_HTONS(server_conn->rport));
        *   Restore server connection to allow data from any node
        *    uip_create_unspecified(&server_conn->ripaddr);
        *    server_conn->rport = 0;
        *    break;
        *}
        */

        }
    }

    uip_udp_packet_send(server_conn, buf, strlen(buf));
    /* Restore server connection to allow data from any node */
    memset(&server_conn->ripaddr, 0, sizeof(server_conn->ripaddr));
    return;
}

/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Server IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
    }
  }
}
/*---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/
PROCESS(LIQUI_process, "Liquidificador process");
PROCESS(udp_server_process, "UDP server process");

AUTOSTART_PROCESSES(&resolv_process, &udp_server_process, &LIQUI_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(LIQUI_process, ev, data)
{
  PROCESS_BEGIN();

  etimer_set(&et_timer_ad, 1*CLOCK_SECOND);

  static struct sensors_sensor *sensor;
  static int valor = 0;
  static int nivel = 0;
  static int estadoCicloPrograma3 = 1;
  static int ciclosPrograma3 = 0;

  static int16_t current_duty = 0;
  static int16_t loadvalue;
  static int16_t ticks;

  // configure IO controller
  IOCPinTypeGpioOutput(LED_BIT_1);
  IOCPinTypeGpioOutput(LED_BIT_2);
  IOCPinTypeGpioOutput(LED_BIT_3);
  IOCPinTypeGpioOutput(LED_BIT_4);

  // configure AD sensor
//  sensor = sensors_find(ADC_SENSOR);

//  nivel = (3.30 * 1000000)/5;

  // init PWM
  loadvalue = pwminit(5000);

  while(1) {
        PROCESS_WAIT_EVENT();
        if (ev == PROCESS_EVENT_TIMER){

/*            //read ADC sensor
        SENSORS_ACTIVATE(*sensor);
        sensor->configure(ADC_SENSOR_SET_CHANNEL,ADC_COMPB_IN_AUXIO7);
        valor = sensor->value(ADC_SENSOR_VALUE);

        printf("sensor: %d\n", valor);

        SENSORS_DEACTIVATE(*sensor);
*/
        // reset do timer
        etimer_reset(&et_timer_ad);


        // aguarda receber pela rede o programa escolhido
 //       programa_escolhido = 0;

        // check step program (0, 1, 2 or 3)
        // 0 - desligado
        // 1 - Maquina ligada
        // 2 - girar devagar
        // 3 - girar e parar
        // 4 - girar bem rapido
/*            if (valor < nivel) {
            programa_escolhido = 0;
        } else if (valor < (nivel * 2)) {
            programa_escolhido = 1;
        } else if (valor < (nivel * 3)) {
            programa_escolhido = 2;
        } else if (valor < (nivel * 4)) {
            programa_escolhido = 3;
        } else {
            programa_escolhido = 4;
        }*/
        printf("programa_atual: %d, programa_escolhido: %d\n", programa_atual, programa_escolhido);

        // check if program has changed
        if (programa_escolhido != programa_atual || programa_atual == 3){

            switch (programa_escolhido) {

                case 0:
                    // set LEDs
                    GPIO_writeDio(LED_BIT_1, 0);
                    GPIO_writeDio(LED_BIT_2, 0);
                    GPIO_writeDio(LED_BIT_3, 0);
                    GPIO_writeDio(LED_BIT_4, 0);

                    // set motor speed
                    current_duty = 1;
                    ticks = (current_duty * loadvalue) / 100;
                    ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);
                    printf("current duty: %d\n", current_duty);

                    break;

                case 1:
                    // set LEDs
                    GPIO_writeDio(LED_BIT_1, 1);
                    GPIO_writeDio(LED_BIT_2, 0);
                    GPIO_writeDio(LED_BIT_3, 0);
                    GPIO_writeDio(LED_BIT_4, 0);

                    // set motor speed
                    current_duty = 1;
                    ticks = (current_duty * loadvalue) / 100;
                    ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);
                    printf("current duty: %d\n", current_duty);

                    break;

                case 2:
                    // set LEDs
                    GPIO_writeDio(LED_BIT_1, 0);
                    GPIO_writeDio(LED_BIT_2, 1);
                    GPIO_writeDio(LED_BIT_3, 0);
                    GPIO_writeDio(LED_BIT_4, 0);

                    // set motor speed
                    current_duty = 65;
                    ticks = (current_duty * loadvalue) / 100;
                    ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);
                    printf("current duty: %d\n", current_duty);

                    break;

                case 3:
                    // set LEDs
                    GPIO_writeDio(LED_BIT_1, 0);
                    GPIO_writeDio(LED_BIT_2, 0);
                    GPIO_writeDio(LED_BIT_3, 1);
                    GPIO_writeDio(LED_BIT_4, 0);

                    if(estadoCicloPrograma3 == 1) {
                        if(ciclosPrograma3 <= 1) {
                            // set motor speed
                            current_duty = 60;
                            ticks = (current_duty * loadvalue) / 100;
                            ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);
                            printf("current duty (lig): %d\n", current_duty);
                        } else {
                            estadoCicloPrograma3 = 0;
                            ciclosPrograma3 = 0;
                        }
                    } else {
                        if(ciclosPrograma3 <= 1) {
                            // set motor speed
                            current_duty = 100;
                            ticks = (current_duty * loadvalue) / 100;
                            ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);
                            printf("current duty(des): %d\n", current_duty);
                        } else {
                            estadoCicloPrograma3 = 1;
                            ciclosPrograma3 = 0;
                        }
                    }

                    ciclosPrograma3++;

                    break;

                case 4:
                    // set LEDs
                    GPIO_writeDio(LED_BIT_1, 0);
                    GPIO_writeDio(LED_BIT_2, 0);
                    GPIO_writeDio(LED_BIT_3, 0);
                    GPIO_writeDio(LED_BIT_4, 1);

                    // set motor speed
                    current_duty = 100;
                    ticks = (current_duty * loadvalue) / 100;
                    ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);
                    printf("current duty: %d\n", current_duty);

                    break;
            }

            programa_atual = programa_escolhido;

            }
        }
  }

  PROCESS_END();
}


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
#if UIP_CONF_ROUTER
  uip_ipaddr_t ipaddr;
  rpl_dag_t *dag;
#endif /* UIP_CONF_ROUTER */

  PROCESS_BEGIN();
  PRINTF("UDP server started\n");

#if RESOLV_CONF_SUPPORTS_MDNS
  resolv_set_hostname("contiki-udp-server");
  PRINTF("Setting hostname to contiki-udp-server\n");
#endif

#if UIP_CONF_ROUTER
  uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
#endif /* UIP_CONF_ROUTER */

  print_local_addresses();

#if 0 //UIP_CONF_ROUTER
  dag = rpl_set_root(RPL_DEFAULT_INSTANCE,
                     &uip_ds6_get_global(ADDR_PREFERRED)->ipaddr);
  if(dag != NULL) {
    uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
    rpl_set_prefix(dag, &ipaddr, 64);
    PRINTF("Created a new RPL dag with ID: ");
    PRINT6ADDR(&dag->dag_id);
    PRINTF("\n");
  }
#endif

  server_conn = udp_new(NULL, UIP_HTONS(CONN_PORT), NULL);
  udp_bind(server_conn, UIP_HTONS(CONN_PORT));

  while(1) {
    PROCESS_YIELD();
    PRINTF("Evento\n");
    if(ev == tcpip_event) {
      printf ("evento tcp udp\n");
      tcpip_handler();
    }
  }

  PROCESS_END();
}
