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
#include "random.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"

static struct etimer et;

enum COR_ATIVA {VERDE, VERMELHO};

int geraNumero(void){

    random_init((unsigned)clock_time());

    return ( 1 + (random_rand() % 2) );
}

/*---------------------------------------------------------------------------*/
PROCESS(lab_3_process, "LAB 3 process 1");

AUTOSTART_PROCESSES(&lab_3_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(lab_3_process,ev,data)
{
  PROCESS_BEGIN();

  SENSORS_ACTIVATE(button_sensor);

  static int CorLigada = 0;
  static int i = 1;
  static int pontos = 0;

    do{
        PROCESS_YIELD();

        if(ev == sensors_event) {
            if(data==&button_left_sensor || data==&button_right_sensor){

                printf("Botao pressionado. Iniciando rodada %d\n", i);

                if (geraNumero() == 1){
                    CorLigada = VERMELHO;
                    leds_on(LEDS_RED);
                } else {
                    CorLigada = VERDE;
                    leds_on(LEDS_GREEN);
                }

                // TIMER COM EVENTO
                etimer_set(&et, CLOCK_SECOND * 3);

                while(1) {

                    PROCESS_WAIT_EVENT_UNTIL(ev==PROCESS_EVENT_TIMER || ev == sensors_event);

                    if(ev == PROCESS_EVENT_TIMER){
                        printf("ESTOROU O TEMPO\n");
                        pontos--;
                    } else {

                        if(data == &button_right_sensor && CorLigada == VERDE){
                           printf("Botao 2 pressionado e cor esta verde = ACERTOU.\n");
                           pontos++;
                        } else if(data == &button_left_sensor && CorLigada == VERMELHO){
                            printf("Botao 1 pressionado e cor esta vermelho = ACERTOU.\n");
                            pontos++;
                        } else {
                            printf("ERROU feio.\n");
                            pontos--;
                        }

                    }
                    printf("Pontos ate o momento: %d\n", pontos);
                    etimer_reset(&et);
                    leds_off(LEDS_GREEN);
                    leds_off(LEDS_RED);
                    i++;
                    break;
                }
            }
        }
    }while(i<11);

    printf("\nRESULTADO FINAL: %d\n", pontos);


   SENSORS_DEACTIVATE(button_sensor);

PROCESS_END();
}
