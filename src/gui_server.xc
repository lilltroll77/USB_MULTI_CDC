/*
 * gui_server.xc
 *
 *  Created on: 27 okt 2018
 *      Author: micke
 */

#include "structs.h"
#include "stdio.h"
#include "xclib.h"

unsafe void gui_server(streaming chanend c_from_RX , streaming chanend c_from_dsp){
    enum pos_e i=Slow;
    unsigned slow=0;

    struct DSPmem_t* unsafe mem;
    #pragma unsafe arrays
    while(1){
        c_from_dsp :> mem;
        struct hispeed_t* unsafe fast =&mem->fast;
       // set_core_fast_mode_on();
        soutct(c_from_RX , 5);
        c_from_RX <: fast->IA;
        c_from_RX <: fast->IC;
        c_from_RX <: fast->QE;
        c_from_RX <: fast->Torque;
        c_from_RX <: fast->Flux;
        c_from_RX <: fast->angle;
  /*      c_from_RX <: *fast; fast++;
        c_from_RX <: *fast; fast++;
        c_from_RX <: *fast; fast++;
        c_from_RX <: *fast; fast++;
        c_from_RX <: *fast; fast++;
        c_from_RX <: i;
        unsafe{
            switch(i){
            case Slow:
                c_from_RX <: slow; //pos
                c_from_RX <: (mem->slow , int[])[slow];
                slow = (slow+1)& sizeof(mem->slow)-1;
                break;
            case Pos:
                c_from_RX <: mem->mid.pos;
                break;
            case Vel:
                c_from_RX <: mem->mid.vel;
                break;
            case Perror:
                c_from_RX <: mem->mid.perror;
                break;
            default:
                break;
            };
            set_core_fast_mode_off();
            i = (i+1) & sizeof(struct midspeed_t)/4;
*/
        //}
    }
}
