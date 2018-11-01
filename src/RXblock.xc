/*
 * RXblock.xc
 *
 *  Created on: 28 okt 2018
 *      Author: micke
 */
#include "structs.h"

#pragma unsafe arrays
unsafe void RX_block(streaming chanend c_from_gui , streaming chanend c_from_CDC){
    //Readble version without full pointer maniac
    //set_core_fast_mode_off();
    unsigned mid , i , blockNumber;
    int block=0;
    struct USBmem_t USBmem[2]={0};
    USBmem[0].checknumber = 3141592543;
    USBmem[1].checknumber = 3141592543;
    //printf("RX:i=%d mid=%d fast=%d\n" , i , mid , fast );
    char ct;
    struct hispeed_vector_t* unsafe fast;
    while(1){
        select{
        case sinct_byref(c_from_CDC , ct):
            mid=0;
            i=0;
            blockNumber=0;
            fast = &USBmem[block].fast;
        break;
        case sinct_byref(c_from_gui , ct):
            c_from_gui :> fast->IA[i]; //printintln(mem->fast.IA[fast]);
            c_from_gui :> fast->IC[i];
            c_from_gui :> fast->QE[i];
            c_from_gui :> fast->Torque[i];
            c_from_gui :> fast->Flux[i];
            c_from_gui :> fast->angle[i];
   /*         c_from_gui :> int _;
            c_from_gui :> int _;
            c_from_gui :> int _;
            c_from_gui :> int _;
            c_from_gui :> int _;

            c_from_gui :> i;
            //printchar('R');
            switch(i){
            case Slow:
                int slow;
                c_from_gui :> slow; //pos
                c_from_gui :> (mem->slow , int[])[slow];
                slow = (slow+1)% sizeof(mem->slow)/4;
                break;
            case Pos:
                c_from_gui :> mem->mid.pos[mid];
                break;
            case Vel:
                c_from_gui :> mem->mid.vel[mid];
                break;
            case Perror:
                c_from_gui :> mem->mid.perror[mid];
                break;
                // add cases here!
            default:
                break;
            };
            mid = (mid+1)&(PKG_SIZE/32-1);

*/
            i++;
            if(i==128){
                USBmem[block].index = blockNumber++;
                c_from_CDC <:(int* unsafe) &USBmem[block]; // send pointer to CDC core
                i=0;
                block = !block;
                fast = &USBmem[block].fast;
                break;
            }
            break;
        }
    }
}
