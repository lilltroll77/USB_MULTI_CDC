/*
 * gui_server.xc
 *
 *  Created on: 27 okt 2018
 *      Author: micke
 */

#include "structs.h"
#include "stdio.h"
#include "stdlib.h"
#include "xclib.h"
//#include "print.h"

unsafe void gui_server(streaming chanend c_from_RX , streaming chanend c_from_dsp , struct fuse_t* unsafe fuse){
    enum pos_e i=Slow;
    unsigned slow=0;
    unsigned CPUload=0;
    struct DSPmem_t* unsafe mem;
#pragma unsafe arrays
    //printintln(fuse->current);
    //printintln(fuse->state);
    fuse->current= 32<<14; //32 [A] default
    fuse->state = 1;
    unsigned blockNumber=0;
    timer tmr;
    unsigned time;
    tmr:>time;
    while(1){
        select{
        case tmr when timerafter(time + 2E6) :> time:
        if(CPUload>0)
            CPUload--;
        break;
        case c_from_dsp :> mem:
        struct hispeed_t* unsafe fast =&mem->fast;
        // set_core_fast_mode_on();
/*1*/   c_from_RX <:fast->FFTtrig;
        c_from_RX <: fast->IA;
        c_from_RX <: fast->IC;
        c_from_RX <: fast->QE;
        c_from_RX <: fast->Torque;
/*6*/   c_from_RX <: fast->Flux;
        c_from_RX <: fast->angle;
        c_from_RX <: fuse->state;
        if(fast->CPUload > CPUload)
            CPUload = fast->CPUload;
/*9*/   c_from_RX <:fast->CPUload;


        int absIA = abs(fast->IA);
        int absIB = abs(fast->IA + fast->IC);
        int absIC = abs(fast->IC);
        int Imax;
        if(absIA > absIB)
            Imax = absIA;
        else
            Imax = absIB;
        if(absIC > Imax)
            Imax = absIC;
        int Ifuse;
        //Force compiler to update Ifuse here in time!
        asm("ldw %0 , %1[%2]" : "=r"(Ifuse) : "r"(fuse) , "r"(0));
        if(Imax > Ifuse){
            fuse->state=0;

        }

        break;

        }
    }
}
