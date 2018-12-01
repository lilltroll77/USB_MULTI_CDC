/*
 * transfer_block.xc
 *
 *  Created on: 27 okt 2018
 *      Author: micke
 */
//static inline
#include <stdio.h>
#include "structs.h"


/// RX and TX function must match each other regarding channel transfers!


//Make it possible to debug att any opt level by breaking it into a separate file
//static inline
#pragma unsafe arrays
static inline
unsafe void RX_block(streaming chanend c ,  struct USBmem_t* unsafe mem , unsigned &mid , unsigned &fast){
    //Readble version without pointer maniac
    enum pos_e i;
    //printf("RX:i=%d mid=%d fast=%d\n" , i , mid , fast );
    soutct(c , 5);
    c :> i;
    switch(i){
    case Slow:
        int slow;
        c :> slow; //pos
      //  c :> (mem->slow , int[])[slow];
      //  slow = (slow+1)% sizeof(mem->slow)/4;
        break;
    case Pos:
        c :> mem->mid.pos[mid];
        break;
    case Vel:
        c :> mem->mid.vel[mid];
        break;
    case Perror:
        c :> mem->mid.perror[mid];
        break;
        // add cases here!
    default:
        break;
    };
    mid = (mid+1)&(PKG_SIZE/32-1);

     c :> mem->fast.IA[fast];
     c :> mem->fast.IC[fast];
     c :> mem->fast.QE[fast];
     c :> mem->fast.Torque[fast];
     c :> mem->fast.Flux[fast];
     c :> mem->fast.U[fast];
     c :> mem->fast.angle[fast];

     fast++;
}
#pragma unsafe arrays
static inline
void TX_block(streaming chanend c ,  struct DSPmem_t* unsafe mem , enum pos_e &i , unsigned &slow){
    //Readble version without pointer maniac
    //printf("TX:i=%d slow=%d\n" , i , slow );
    c <: i;
    unsafe{
        switch(i){
        case Slow:
            c <: slow; //pos
            //c <: (mem->slow , int[])[slow];
            //slow = (slow+1)& sizeof(mem->slow)-1;
            break;
        case Pos:
            //c <: mem->mid.pos;
            break;
        case Vel:
            //c <: mem->mid.vel;
            break;
        case Perror:
            //c <: mem->mid.perror;
            break;
        default:
            break;
        };
        i = (i+1) & sizeof(struct midspeed_t)/4;
        c <: mem->fast.IA;
        c <: mem->fast.IC;
        c <: mem->fast.QE;
        c <: mem->fast.Torque;
        c <: mem->fast.Flux;
        c <: mem->fast.U;
        c <: mem->fast.angle;
    }
}

