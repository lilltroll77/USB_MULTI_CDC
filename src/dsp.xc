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
#include "xs1.h"
#include "cdc_handlers.h"



struct state_t{
    s64 y1;
    s64 y2;
    int x1;
    int x2;
    unsigned error;

};


struct data64_t{
    unsigned lo;
    int hi;

};

static inline
void macs96(struct data64_t &y , int &a , int &hi , unsigned &mi , unsigned &lo){
    int s5 , shift=31;
    int asgn = a>>shift;
   {s5 , lo} = lmul( y.lo , a  , 0 , lo);
   {s5 , mi} = lmul( y.lo , asgn  , s5 , mi );
   {s5 , mi} = mac(  y.hi , a  , s5 , mi );
   {s5 , hi} = lmul( y.hi , asgn  , s5 , hi );
   {s5 , hi} = mac(  y.lo , asgn  , s5 , hi );
   {s5 , hi} = mac(  y.hi>>shift , a , s5 , hi );
}



static inline
int soc(int x , struct EQ_t &eq , struct state_t &s ){
    int hi;
    unsigned mid , lo;
    {hi , mid}= macs(eq.B2 , s.x2 , 0 , 0);
    s.x2 = s.x1;
    {hi , mid}= macs(eq.B1 , s.x1 , hi , mid);
    s.x1 = x;
    {hi , mid}= macs(eq.B0 , x , hi , mid);

    macs96( (s.y2 , struct data64_t) , eq.A2 , hi ,mid , s.error);
    s.y2 = s.y1;
    macs96( (s.y1 , struct data64_t) , eq.A1 , hi ,mid , lo);

    int yhi;
    unsigned ymid;
    asm("lextract %0,%1,%2,%3,32": "=r"(yhi) :"r"(hi) ,"r"(mid),"r"(eq.shift));
    asm("lextract %0,%1,%2,%3,32": "=r"(ymid):"r"(mid),"r"(lo) ,"r"(eq.shift));
    s.y1 = ((s64)yhi<<32) | ymid;
    s.error = (lo & 0x3FFFFFFF); // Feedback the trucation error to next iteration
    return yhi;
}


static inline
int PI(int x, int p , int i , int &hi , unsigned &lo){
    //antiwindup ??
    int h; unsigned l;
    {h , l}  = macs( x , p , 0 , 0 );
    asm("lextract %0,%1,%2,%3,32": "=r"(h):"r"(h),"r"(l) ,"r"(22));

    {hi , lo} = macs( h , i , hi , lo);
    //asm("ladd %0,%1,%2,%3,%4" : "=r"(h), "=r"(l) :  "r"(hi), "r"(lo) , "r"(0));
    return h+hi;
}

void MLSgen(streaming chanend c){
    unsigned lfsr=1 ,zero=0;
    unsigned checksum=1;
 /*   while(1){
        crc32(checksum, ~0,   0xEDB88320);
        c<: sext(checksum,16);
    }*/
    while(1){
        int lsb;
        if(zero){
            lsb=0;
            zero=0;
        }else{
            lsb = lfsr & 1;   /* Get LSB (i.e., the output bit). */
            lfsr >>= 1;                /* Shift register */
            if (lsb)                   /* If the output bit is 1, apply toggle mask. */
                lfsr ^= 0b100000010000000000;
            if(lfsr==1)
                zero=1;
        }
        int val;
        if(lsb)
            val=(1<<17);
        else
            val=-(1<<17);
        c<: val;
        c<: val>>2;
        c<: -val>>2;
    }
}

unsafe void DSP(streaming chanend c_from_GUI , streaming chanend c_from_CDC  , streaming chanend c_from_MLS){ //Emulates dsp working cores by updating different signals
    timer tmr1 , tmr2;
    unsigned t1 , t2;

//    int i=0;
//    int j=0;
    struct DSPmem_t dsp_memory[2];
    struct DSPmem_t* unsafe mem;
    struct state_t state[4];
    struct data64_t Int[2];
    memset( dsp_memory , 0 , sizeof(dsp_memory));
    memset( state , 0 , sizeof(state));
    memset( Int   , 0 , sizeof(Int));
    int ctrl=0;
    int block=0;
    unsigned dsp_counter=0;
    struct regulator_t reg[2];
    memset( reg , 0 , sizeof(reg));
    for(int i=0; i<2 ; i++){
        for(int j=0; j<2 ; j++){
            reg[i].EQ[j].shift = 30;
            reg[i].EQ[j].B0 = 1<<reg[i].EQ[j].shift;
            reg[i].P = 1<<24;
        }
        reg[i].P = 1<<22;
    }

#define SIN_LEN 8192
    int sin_tbl[(SIN_LEN*4)/3];
    for(int i=0; i < sizeof(sin_tbl)/(sizeof(int)) ; i++)
        sin_tbl[i] = 200*sin(2.0 * M_PI/SIN_LEN * (double)i);

    tmr1:> t1;
    tmr2:> t2;

    c_from_CDC <:(int* unsafe) reg;
    printint((int)reg);
    int rnd;

    while(1){
        select{
        case tmr1 when timerafter(t1 + 333):>t1: //300 kHz
                //https://en.wikipedia.org/wiki/Linear-feedback_shift_register#Some_polynomials_for_maximal_LFSRs

                mem = &dsp_memory[block];
                //DSP code below

                c_from_MLS :> rnd;

                int s0 = -mem->fast.IA;
                s0 = PI( s0 , reg[0].P , reg[0].I , Int[0].hi , Int[0].lo);
                s0 = soc(s0 , reg[0].EQ[0] , state[0] );
                mem->fast.IA = soc(s0, reg[0].EQ[1] , state[1] ) + rnd; // inject disturbance

                int s1 = -mem->fast.IC;
                s1 = PI(s1 , reg[1].P , reg[1].I , Int[1].hi , Int[1].lo);
                s1 = soc(s1 , reg[1].EQ[0] , state[2] );
                mem->fast.IC = soc(s1, reg[1].EQ[1] , state[3] ) - rnd;

                mem->fast.QE =  dsp_counter;
                mem->fast.angle  = (dsp_counter+100) & (SIN_LEN-1);

                c_from_MLS :> mem->fast.Flux ;
                c_from_MLS :> mem->fast.Torque;

                dsp_counter = (dsp_counter+1)&(SIN_LEN-1);

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
                    int* unsafe ptr;
                    c_from_CDC :> ptr;
                    //printf("PI %d\n" , ptr);
                    c_from_CDC :> ptr[1];
                    c_from_CDC :> ptr[0];
                    //printf("P=%d I=%d\n" , ptr[1] , ptr[0]);

                break;
            case EQsection:
                    int* unsafe ptr;
                    c_from_CDC :> ptr;
                    //printf("EQ %d\n" , ptr);
                    c_from_CDC :> ptr[0];
                    c_from_CDC :> ptr[1];
                    c_from_CDC :> ptr[2];
                    c_from_CDC :> ptr[3];
                    c_from_CDC :> ptr[4];
                    c_from_CDC :> ptr[5];
#if(0)
                     printf("B0=%d, B1=%d, B2=%d, A1=%d, A2=%d\n" ,
                             ptr[0] , ptr[1], ptr[2], ptr[3], ptr[4]);
#endif

                break;
            }
         break;
        }
    }
}
