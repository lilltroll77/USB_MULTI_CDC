/*
 * dsp.xc
 *
 *  Created on: 27 okt 2018
 *      Author: micke
 */
#include <stdio.h>
#include <string.h>
#include "dsp.h"
#include <math.h>

unsafe void DSP(streaming chanend c_from_GUI , chanend c_from_CDC){ //Emulates dsp working cores by updating different signals
    timer tmr1 , tmr2;
    unsigned t1 , t2;
    tmr1:> t1;
    tmr2:> t2;
//    int i=0;
//    int j=0;
    struct DSPmem_t dsp_memory[2];
    struct DSPmem_t* unsafe mem;
    memset( dsp_memory , 0 , sizeof(dsp_memory));
    int ctrl=0;
    int block=0;
    unsigned dsp_counter=0;

#define SIN_LEN 8192
    int sin_tbl[(SIN_LEN*4)/3];
    for(int i=0; i < sizeof(sin_tbl)/(sizeof(int)) ; i++)
        sin_tbl[i] = 8000*sin(2.0 * M_PI/SIN_LEN * (double)i);

    while(1){
        select{
        case tmr1 when timerafter(t1 + 333):>t1: //300 kHz
                mem = &dsp_memory[block];
                //DSP code below
                dsp_counter = (dsp_counter+1)&(SIN_LEN-1);
                mem->fast.IA = sin_tbl[dsp_counter];
                mem->fast.IC = sin_tbl[dsp_counter+(SIN_LEN/3)];;
                mem->fast.QE =  dsp_counter;
                mem->fast.angle  = (dsp_counter+100) & (SIN_LEN-1);

                /*mem.fast.U = (mem.fast.IA>>3) * (mem.fast.IC>>2);
                mem.fast.angle  = mem.fast.QE+10 & (SIN_LEN-1);

                mem.fast.Flux = j;
                mem.fast.Torque = -j;
                if( i == 0){ //37.5 kHz
                    mem.mid.pos++;
                    mem.mid.vel--;
                }
                i = (i+1) & 0x7;*/
                if(ctrl)
                    c_from_GUI <: mem;
                block = !block;
                break;

   /*      case tmr2 when timerafter(t2 + 1e7):>t2:  //10Hz
                 mem->slow.temp = (mem->slow.temp+1)&0x3FF;
                break;
                */
         case c_from_CDC :> ctrl:
             dsp_counter=0;
             //printint(ctrl);
                break;
        }
    }
}
