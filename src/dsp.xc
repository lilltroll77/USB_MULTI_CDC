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
#include "xclib.h"
#include "cdc_handlers.h"

struct EQ_t{
    int B0;
    int B1;
    int B2;
    int A1;
    int A2;
};

struct regulator_t{
    int I;
    int P;
    struct EQ_t EQ[2];
};

unsafe void DSP(streaming chanend c_from_GUI , chanend c_from_CDC){ //Emulates dsp working cores by updating different signals
    timer tmr1 , tmr2;
    unsigned t1 , t2;

//    int i=0;
//    int j=0;
    struct DSPmem_t dsp_memory[2];
    struct DSPmem_t* unsafe mem;
    memset( dsp_memory , 0 , sizeof(dsp_memory));
    int ctrl=0;
    int block=0;
    unsigned dsp_counter=0;
    struct regulator_t reg[2]={0};

#define SIN_LEN 8192
    int sin_tbl[(SIN_LEN*4)/3];
    for(int i=0; i < sizeof(sin_tbl)/(sizeof(int)) ; i++)
        sin_tbl[i] = 8000*sin(2.0 * M_PI/SIN_LEN * (double)i);

    int rnd1 , rnd2;
    unsigned poly=0xEDB88320;
    tmr1:> t1;
    t1=rnd1;
    tmr2:> t2;
    t2=rnd2;
    while(1){
        select{
        case tmr1 when timerafter(t1 + 333):>t1: //300 kHz
                mem = &dsp_memory[block];
                //DSP code below
                crc32((rnd1 , unsigned) ,t1, poly);
                crc32((rnd2 , unsigned) ,t1, poly);

                mem->fast.IA = sin_tbl[dsp_counter] + (rnd1>>20);

                mem->fast.IC = sin_tbl[dsp_counter+(SIN_LEN/3)]+ (rnd2>>20);
                mem->fast.QE =  dsp_counter;
                mem->fast.angle  = (dsp_counter+100) & (SIN_LEN-1);

                mem->fast.Flux =   ((int)bitrev(rnd1)>>20);
                mem->fast.Torque = ((int)bitrev(rnd2)>>20) + 4096;


                dsp_counter = (dsp_counter+1)&(SIN_LEN-1);
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
        case c_from_CDC :> int cmd:
            switch(cmd){
            case streamOUT:
                c_from_CDC :> ctrl;
                dsp_counter=0;
                break;
            case PIsection:
                slave{
                    int ch;
                    c_from_CDC :> ch;
                    c_from_CDC :> reg[ch].P;
                    c_from_CDC :> reg[ch].I;
                }
                break;
            case EQsection:
                slave{
                    int ch , sec;
                    c_from_CDC :> ch;
                    c_from_CDC :> sec;
                    struct EQ_t* eq = &reg[ch].EQ[sec];
                    c_from_CDC :> eq->B0;
                    c_from_CDC :> eq->B1;
                    c_from_CDC :> eq->B2;
                    c_from_CDC :> eq->A1;
                    c_from_CDC :> eq->A2;
#if(0)
                     printf("Ch:%d Sec:%d B0=%d, B1=%d, B2=%d, A1=%d, A2=%d\n" , ch, sec ,
                         eq->B0, eq->B1, eq->B2, eq->A1, eq->A2);
#endif
                }
                break;
            }
         break;
        }
    }
}
