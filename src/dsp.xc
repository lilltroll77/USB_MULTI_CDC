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

#define SIN_LEN 8192
#define CT 0



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
    if(eq.shift <0)
        return x;
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

//https://en.wikipedia.org/wiki/Linear-feedback_shift_register#Some_polynomials_for_maximal_LFSRs



void signalgenerator(streaming chanend c , chanend c_ctrl){
    unsigned lfsr=1 ,zero=0;
    unsigned checksum=1;
    int sin_tbl[(SIN_LEN*4)/3];
    const double amp = pow(2,16);
    const double scale = (2.0 * M_PI/SIN_LEN);
    unsigned sample_sent=0;
    enum signal_e type=MLS18;
    timer tmr;
    unsigned t;
    int val;
    while(1){
        select{
        default:
            switch(type){
            case OFF:
                val=0;
                break;
            case MLS18:
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
                val = lsb ? (1<<17) : -(1<<17);
                //printintln(val);
                break;
            case RND:
                crc32(checksum, ~0,   0xEDB88320);
                val = sext(checksum, 18);
                break;
            case SINE:
            case OCTAVE:
                val = sin_tbl[sample_sent&(SIN_LEN-1)];
                 break;
            default:
                break;
            }
            c <: val;
            c <: sample_sent;
            sample_sent = (sample_sent +1 )& (FFT_LEN-1);

            break;
            case c_ctrl:> type:
                c <: 0;
                c <: 0;
                switch(type){
                case OFF:
                    break;
                case SINE:
                    for(int i=0; i < sizeof(sin_tbl)/(sizeof(int)) ; i++)
                        sin_tbl[i] = amp*sin(scale * (double)i);
                    break;
                case OCTAVE:
                    for(int i=0; i < sizeof(sin_tbl)/(sizeof(int)) ; i++){
                        double x=sin_tbl[i] = (amp/4)*sin(scale * (double)(i));
                        for(int oct=1 ; oct < 4 ; oct ++)
                             x += (amp/4)*sin(scale * (double)(i<<oct));
                        sin_tbl[i]=x;
                    }
                    break;

                default:
                    break;
                }
                sample_sent=0;
                break;
        }
    }
}


unsafe void DSP(streaming chanend c_from_GUI , streaming chanend c_from_CDC  , streaming chanend c_from_MLS , struct fuse_t &fuse){ //Emulates dsp working cores by updating different signals
    timer tmr1;
    unsigned t1,t2;

//    int i=0;
//    int j=0;
    struct DSPmem_t shared_mem[2];
    struct DSPmem_t* unsafe mem;
    struct state_t state[4];
    struct data64_t Int[2];
    memset( &shared_mem , 0 , sizeof(shared_mem));
    memset( state , 0 , sizeof(state));
    memset( Int   , 0 , sizeof(Int));
    int ctrl=0;
    int block=0;
    struct regulator_t reg[2];
    memset( reg , 0 , sizeof(reg));
    for(int i=0; i<2 ; i++){
        for(int j=0; j<2 ; j++)
            reg[i].EQ[j].shift = -1; //Disable all filters

    }


    tmr1:> t1;
    c_from_CDC <:(int* unsafe) reg;
    c_from_CDC <:(int* unsafe) state;
    //printint((int)reg);
    int rnd;
    int signalsource=0;
    int siggen=1;
    /* This loop priorities speed optimization over code clarity */
    while(1){
        select{
         //Priority: always perform at least one dsp-loop cycle between each transaction from CDC
        default:
#pragma unsafe arrays
               mem = &shared_mem[block];
            select{
            case c_from_MLS :> rnd: // Dont stop DSP loop if signalgen is late or sleeping
                c_from_MLS :> mem->fast.FFTtrig;
                mem->fast.QE =  mem->fast.FFTtrig&(SIN_LEN-1);
                break;
            }


                tmr1 :> t2;
                mem->fast.CPUload = t2-t1;
                tmr1 when timerafter(t1 + 333):>t1; //300 kHz

                //DSP code below
                //printint(shared_mem.fuse.state);


                if(fuse.state){

                 if(signalsource){
                    int s0 = -mem->fast.IA + rnd; //Negative feedback
                    s0 = PI( s0 , reg[0].P , reg[0].I , Int[0].hi , Int[0].lo);
                    s0 = soc(s0 , reg[0].EQ[0] , state[0] );
                    mem->fast.IA = soc(s0, reg[0].EQ[1] , state[1] ); // inject disturbance

                    int s1 = -mem->fast.IC+ rnd; //Negative feedback
                    s1 = PI(s1 , reg[1].P , reg[1].I , Int[1].hi , Int[1].lo);
                    s1 = soc(s1 , reg[1].EQ[0] , state[2] );
                    mem->fast.IC = soc(s1, reg[1].EQ[1] , state[3] );  // inject disturbance
                 }else{
                     int s0 = -mem->fast.IA ; //Negative feedback
                     s0 = PI( s0 , reg[0].P , reg[0].I , Int[0].hi , Int[0].lo);
                     s0 = soc(s0 , reg[0].EQ[0] , state[0] );
                     mem->fast.IA = soc(s0, reg[0].EQ[1] , state[1] )+ rnd; // inject disturbance

                     int s1 = -mem->fast.IC; //Negative feedback
                     s1 = PI(s1 , reg[1].P , reg[1].I , Int[1].hi , Int[1].lo);
                     s1 = soc(s1 , reg[1].EQ[0] , state[2] );
                     mem->fast.IC = soc(s1, reg[1].EQ[1] , state[3] )- rnd;  // inject disturbance

                 }
                }else{
                    mem->fast.IA=0;
                    mem->fast.IC=0;
                }


                mem->fast.angle  = 0;
                mem->fast.Flux = rnd;
                mem->fast.Torque = -rnd;

                if(ctrl){
                   c_from_GUI <: mem;
                   block = !block;
                }

                break;
        case c_from_CDC :> int cmd:
            switch(cmd){
            case streamOUT:
                c_from_CDC :> ctrl;
                break;
            case PIsection:
                    struct regulator_t* unsafe reg_ptr;
                    c_from_CDC :> reg_ptr;
                    c_from_CDC :> reg_ptr->P;
                    c_from_CDC :> reg_ptr->I;
                    //printf("P=%d I=%d\n" , reg_ptr->P , reg_ptr->I);

                break;
            case EQsection:
                     struct EQ_t* unsafe eq;
                    c_from_CDC :> eq;
                    //printf("EQ %d\n" , ptr);
                    c_from_CDC :> eq->B0;
                    c_from_CDC :> eq->B1;
                    c_from_CDC :> eq->B2;
                    c_from_CDC :> eq->A1;
                    c_from_CDC :> eq->A2;
                    c_from_CDC :> eq->shift;
                    // Verified that above compiles to imediate stw instructions
#if(0)
                     printf("B0=%d, B1=%d, B2=%d, A1=%d, A2=%d Shift=%d\n" ,
                             eq->B0 , eq->B1, eq->B2, eq->A1, eq->A2 , shift);
#endif

                break;
            case resetPI:
                int ch;
                c_from_CDC :> ch;
                (Int[ch] , long long )=0; //reset first 64 bits
                //printf("Reset ch:%d\n", ch);
                break;
            case resetEQsec:
                s64* unsafe ptr;
                c_from_CDC :> ptr;
                ptr[0]=0; //y1
                ptr[1]=0; //y2
                ptr[2]=0; //x1,x2
                ((int*)ptr)[6]=0; //error
                break;
            case FuseCurrent:
                c_from_CDC :> fuse.current;
                //printintln(fuse.current);
                break;
            case FuseStatus:
                c_from_CDC :>  fuse.state;
                //printstr("Fuse:");
                //printintln(fuse.state);
                break;
            case SignalSource:
                c_from_CDC :> signalsource;
                break;
            }
         break;
        }
    }
}
