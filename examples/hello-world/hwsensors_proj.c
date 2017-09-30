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
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "dev/leds.h"
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

static struct etimer et_tempo;
static struct etimer et_timer_ad;



#define LED_BIT_1 IOID_12
#define LED_BIT_2 IOID_27
#define LED_BIT_3 IOID_22
#define LED_BIT_4 IOID_15
#define PORTA_PWM IOID_21
// IOID_23 USADA PELO ADC

uint8_t pwm_request_max_pm(void)
{
    return LPM_MODE_DEEP_SLEEP;
}

void sleep_enter(void)
{
    leds_on(LEDS_RED);
}

void sleep_leave(void)
{
    leds_off(LEDS_RED);
}

LPM_MODULE(pwmdrive_module, pwm_request_max_pm,
           sleep_enter, sleep_leave, LPM_DOMAIN_PERIPH);

int16_t pwminit(int32_t freq)
{
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
PROCESS(LAVAR_process, "Lavar process");
PROCESS(PWM_process, "pwm process");
PROCESS(gpio_process, "gpio process");
PROCESS(read_button_process, "gpio process");

AUTOSTART_PROCESSES(&LAVAR_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(LAVAR_process, ev, data)
{
  PROCESS_BEGIN();

  etimer_set(&et_timer_ad, 1*CLOCK_SECOND);

  static struct sensors_sensor *sensor;
  static int valor = 0;
  static int nivel = 0;
  static int programa_escolhido = 0;
  static int programa_atual = 0;
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
  sensor = sensors_find(ADC_SENSOR);

  nivel = (3.30 * 1000000)/5;

  // init PWM
  loadvalue = pwminit(5000);


  while(1) {
        PROCESS_WAIT_EVENT();
        if (ev == PROCESS_EVENT_TIMER){

            //read ADC sensor
            SENSORS_ACTIVATE(*sensor);
            sensor->configure(ADC_SENSOR_SET_CHANNEL,ADC_COMPB_IN_AUXIO7);
            valor = sensor->value(ADC_SENSOR_VALUE);

            printf("sensor: %d\n", valor);

            SENSORS_DEACTIVATE(*sensor);
            etimer_reset(&et_timer_ad);

            // check step program (0, 1, 2 or 3)
            // 0 - desligado
            // 1 - Maquina ligada
            // 2 - girar devagar
            // 3 - girar e parar
            // 4 - girar bem rapido
            if (valor < nivel) {
                programa_escolhido = 0;
            } else if (valor < (nivel * 2)) {
                programa_escolhido = 1;
            } else if (valor < (nivel * 3)) {
                programa_escolhido = 2;
            } else if (valor < (nivel * 4)) {
                programa_escolhido = 3;
            } else {
                programa_escolhido = 4;
            }
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
                        current_duty = 100;
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
                        current_duty = 100;
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
                        current_duty = 70;
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
                            if(ciclosPrograma3 <= 6) {
                                // set motor speed
                                current_duty = 40;
                                ticks = (current_duty * loadvalue) / 100;
                                ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);
                                printf("current duty (lig): %d\n", current_duty);
                            } else {
                                estadoCicloPrograma3 = 0;
                                ciclosPrograma3 = 0;
                            }
                        } else {
                            if(ciclosPrograma3 <= 6) {
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
                        current_duty = 1;
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
PROCESS_THREAD(PWM_process, ev, data)
{
  PROCESS_BEGIN();

  static int16_t current_duty = 0;
  static int16_t loadvalue;
  static int16_t ticks;

  loadvalue = pwminit(5000);

  printf("iniciando PWM\n");

  current_duty = 50;
  ticks = (current_duty * loadvalue) / 100;
  ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);
  printf("current duty: %d\n", current_duty);

  while(1) {

      PROCESS_WAIT_EVENT();

      if(ev == sensors_event){
          if(data == &button_left_sensor){
              printf("Left Button! -10 PWM\n");
              if (current_duty == 10) {
                  current_duty = 1;
              }
              else if (current_duty > 10) {
                  current_duty -= 10;
              }
          }
          else if(data == &button_right_sensor){
              printf("Right Button! +10 PWM\n");
              if (current_duty == 1) {
                  current_duty = 10;
              }
              else if (current_duty <= 90) {
                  current_duty += 10;
              }
          }

          ticks = (current_duty * loadvalue) / 100;
          ti_lib_timer_match_set(GPT0_BASE, TIMER_A, loadvalue - ticks);

          printf("current duty: %d\n", current_duty);

      }
  }


  PROCESS_END();
}


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(gpio_process, ev, data)
{
  PROCESS_BEGIN();

  etimer_set(&et_tempo, 1*CLOCK_SECOND);

  static int contador = 0;
  static int bit_0 = 0, bit_1 = 0;

  // configure IO controller
  IOCPinTypeGpioOutput(LED_BIT_1);
  IOCPinTypeGpioOutput(LED_BIT_2);

  while(1) {
        PROCESS_WAIT_EVENT();
        if (ev == PROCESS_EVENT_TIMER){
            //printf("Passou 1 sec\n");

            contador++;

            bit_0 = contador & 0b01;
            bit_1 = (contador & 0b10)>>1;

            //printf("contador: %d, bit_1: %d, bit_0: %d\n", contador, bit_1, bit_0);

            //piscar os leds
            GPIO_writeDio(LED_BIT_1, bit_0);
            GPIO_writeDio(LED_BIT_2, bit_1);

            etimer_reset(&et_tempo);
        }
  }


  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(read_button_process, ev, data)
{
    PROCESS_BEGIN();
    printf("Read Button Demo\n");

    while(1){
        PROCESS_YIELD();

  /*      if(ev == sensors_event){
            if(data == &button_left_sensor){
                printf("Left Button!\n");
                leds_toggle(LEDS_RED);
            }
            else if(data == &button_right_sensor){
                leds_toggle(LEDS_GREEN);
                printf("Right Button!\n");
            }

        }

        */
        PROCESS_END();
    }
    return 0;
}
