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
#include "sys/etimer.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include "funcao.h"

static struct etimer et;

enum COR_ATIVA {VERDE, VERMELHO};
#define NUM_CORES 2

/*int geraNumero(void){

    random_init((unsigned)clock_time());

    return ( 1 + (random_rand() % 2) );
}*/


/*---------------------------------------------------------------------------*/
PROCESS(lab_4_process, "LAB 4 process 1");

AUTOSTART_PROCESSES(&lab_4_process);
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(lab_4_process,ev,data)
{
  PROCESS_BEGIN();

  SENSORS_ACTIVATE(button_sensor);

  static int CorLigada = 0;
  static int i = 0;
  static int rodada = 1;
  static int reiniciar = 0;
  static int vencedor = 0;
  static int memoria[NUM_CORES];
  static int pontuacao = 0;

    do{
        PROCESS_YIELD();

        if(ev == sensors_event) {
            if(data==&button_left_sensor || data==&button_right_sensor){

                printf("Reiniciando as cores\n");

                reiniciar = 0;
                pontuacao = 0;

                for (i=0; i<NUM_CORES; i++){

                    if (geraNumero() == 1){
                        CorLigada = VERMELHO;
                    } else {
                        CorLigada = VERDE;
                    }

                    memoria[i] = CorLigada;
                }

                printf("Memoria preenchida com %d\n", i);

                do{

                    printf("Rodada %d\n", rodada);

                    for(i=0; i<rodada;i++) {

                         if(memoria[i] == VERDE){
                             leds_on(LEDS_GREEN);
                          } else if(memoria[i] == VERMELHO){
                              leds_on(LEDS_RED);
                          }

                          etimer_set(&et, CLOCK_SECOND);
                          PROCESS_WAIT_EVENT_UNTIL(ev==PROCESS_EVENT_TIMER);

                          etimer_restart(&et);
                          leds_off(LEDS_GREEN);
                          leds_off(LEDS_RED);

                          PROCESS_WAIT_EVENT_UNTIL(ev==PROCESS_EVENT_TIMER);

                    }

                    for(i=0; i<rodada;i++) {

                        PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event);

                        if(data == &button_right_sensor && memoria[i] == VERDE){
                           printf("Botao 2 pressionado e cor MEMORIA esta verde = ACERTOU.\n");
                           pontuacao++;
                        } else if(data == &button_left_sensor && memoria[i] == VERMELHO){
                            printf("Botao 1 pressionado e cor MEMORIA esta vermelho = ACERTOU.\n");
                            pontuacao++;
                        } else {
                            printf("ERROU feio - Vai Reiniciar.\n");
                            leds_on(LEDS_RED);
                            leds_on(LEDS_GREEN);
                            reiniciar = 1;

                            printf("Pontuacao = %d\n", pontuacao);
                            printf("Sequencia = ");
                            for(i=0; i<rodada;i++) {
                                  if(memoria[i] == VERDE){
                                      printf("VERDE, ");
                                  } else if(memoria[i] == VERMELHO){
                                      printf("VERMELHO, ");
                                  }
                            }

                            break;
                        }
                    }
                    if(reiniciar) {
                        rodada = 1;
                    } else if(rodada==NUM_CORES){
                        vencedor = 1;
                    } else {
                        rodada++;
                    }

                }while(!reiniciar && !vencedor);
            }
        }
    }while(!vencedor);


    for(i=0; i<3;i++) {

         leds_on(LEDS_GREEN);
         leds_on(LEDS_RED);


          etimer_set(&et, CLOCK_SECOND/2);

          PROCESS_WAIT_EVENT_UNTIL(ev==PROCESS_EVENT_TIMER);

          etimer_reset(&et);
          leds_off(LEDS_GREEN);
          leds_off(LEDS_RED);

          PROCESS_WAIT_EVENT_UNTIL(ev==PROCESS_EVENT_TIMER);

    }

    printf("\n*********PARABENS VOCE VENCEU**************\n");


   SENSORS_DEACTIVATE(button_sensor);

PROCESS_END();
}
