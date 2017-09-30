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
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include <stdio.h> /* For printf() */
#include "dev/leds.h"

#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "random.h"
#include "button-sensor.h"
#include "board-peripherals.h"

#include "ti-lib.h"

#define LED_PING_EVENT 44
#define LED_PONG_EVENT 45

static struct etimer et_hello;
static struct etimer et_blink;
static struct etimer et_proc3;



/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
PROCESS(blink_process, "Blink process");
PROCESS(proc3_process, "Proc3 process");
PROCESS(pong_process, "Pong process");
PROCESS(read_button_process, "Buttons process");

AUTOSTART_PROCESSES(&hello_world_process, &blink_process, &proc3_process, &pong_process, &read_button_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(pong_process, ev, data)
{
  PROCESS_BEGIN();



  while(1) {
        PROCESS_WAIT_EVENT();
        if (ev == LED_PING_EVENT){
            printf("Recebeu um ping do processo: %s\n", ((struct process*)data)->name);

            process_post((struct process*)data, LED_PONG_EVENT, ((struct process*)data)->name);

        }
  }


  PROCESS_END();
}

PROCESS_THREAD(read_button_process, ev, data)
{
    PROCESS_BEGIN();
    printf("Read Button Demo\n");

    struct process *nome;


    while(1){
        PROCESS_YIELD();

        if(ev == sensors_event){
            if(data == &button_left_sensor){
                printf("Left Button!\n");
                leds_toggle(LEDS_RED);
                nome->name = "Esquerdo";

                process_post(&pong_process, LED_PING_EVENT, (void*)(&nome));
            }
            else if(data == &button_right_sensor){
                leds_toggle(LEDS_GREEN);
                printf("Right Button!\n");
                nome->name = "Direito";
                process_post(&pong_process, LED_PING_EVENT, (void*)(&nome));
            }
        }  else if (ev == LED_PONG_EVENT) {

           printf("Recebeu pong.\n");
       }


        PROCESS_END();
    }
    return 0;
}
/*---------------------------------------------------------------------------*/


PROCESS_THREAD(proc3_process, ev, data)
{
  PROCESS_BEGIN();

  etimer_set(&et_proc3, 10*CLOCK_SECOND);

  while(1) {
        PROCESS_WAIT_EVENT();
        if (ev == PROCESS_EVENT_TIMER){
            printf("Passou 10 sec\n");

            process_post(&pong_process, LED_PING_EVENT, (void*)(&proc3_process));

            etimer_reset(&et_proc3);
        } else if (ev == LED_PONG_EVENT) {

            printf("Recebeu pong: %s\n", (&proc3_process)->name);
        }
  }


  PROCESS_END();
}
/*---------------------------------------------------------------------------*/


PROCESS_THREAD(blink_process, ev, data)
{
  PROCESS_BEGIN();

  etimer_set(&et_blink, 2*CLOCK_SECOND);

  while(1) {
        PROCESS_WAIT_EVENT();
        if (ev == PROCESS_EVENT_TIMER){
            leds_toggle(LEDS_GREEN);
            printf("Piscou led\n");

            process_post(&pong_process, LED_PING_EVENT, (void*)(&blink_process));

            etimer_reset(&et_blink);
        } else if (ev == LED_PONG_EVENT) {

            printf("Recebeu pong: %s\n", (&blink_process)->name);
        }
  }


  PROCESS_END();
}
/*---------------------------------------------------------------------------*/


PROCESS_THREAD(hello_world_process, ev, data)
{
  PROCESS_BEGIN();

  etimer_set(&et_hello, 3*CLOCK_SECOND);

  while(1) {
        PROCESS_WAIT_EVENT();
        if (ev == PROCESS_EVENT_TIMER){
            printf("Hello world!\n");

            process_post(&pong_process, LED_PING_EVENT, (void*)(&hello_world_process));

            etimer_reset(&et_hello);
        } else if (ev == LED_PONG_EVENT) {

            printf("Recebeu pong: %s\n", (&hello_world_process)->name);
        }
  }


  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
