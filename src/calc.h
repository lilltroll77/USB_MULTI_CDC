/*
 * calc.h
 *
 *  Created on: 17 nov 2018
 *      Author: micke
 */

#include "cdc_handlers.h"

#define FS (5e8/26/64)

void calcPI_fixedpoint(chanend c , struct PI_section_t &PI , int ch);
void calcEQ_fixedpoint(chanend c , struct EQ_section_t &EQ , int ch , int sec);
