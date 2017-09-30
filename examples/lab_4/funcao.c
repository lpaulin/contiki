/*
 * funcao.c
 *
 *  Created on: Aug 19, 2017
 *      Author: mint
 */
#include "funcao.h"
#include "random.h"
#include "clock.h"

int geraNumero(void){

    random_init((unsigned)clock_time());

    return ( 1 + (random_rand() % 2) );
}

