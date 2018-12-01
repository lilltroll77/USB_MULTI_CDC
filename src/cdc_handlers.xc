/*
 * cdc_handlers.xc
 *
 *  Created on: 27 okt 2018
 *      Author: micke
 */

#include <stdio.h>
#include "print.h"
#include <math.h>
#include "structs.h"
#include "cdc_handlers.h"
#include "calc.h"
#include "dsp.h"





//cdc data is queued on a FIFO
unsafe void cdc_handler1(client interface cdc_if cdc , streaming chanend c_from_dsp , streaming chanend c_from_RX, chanend c_temp  , chanend c_ep_in[],XUD_buffers_t * unsafe buff){
    XUD_Result_t result;
    struct master_setting_t settings;
    int buffer_writing2host=InEPready;
    char* unsafe writePtr;
    struct USBmem_t* unsafe USBmem;
    struct regulator_t* unsafe reg;
    struct state_t* unsafe dsp_state;
    c_from_RX :> USBmem;
    c_from_dsp :> reg;
    c_from_dsp :> dsp_state;
    //printf("CDCin %d" , reg);
    //printintln(val);
    while(1){
        select{
        case c_from_RX :> writePtr:
           if( buffer_writing2host == InEPready){
                XUD_SetReady_In(buff->tx.ep[1] ,(char*) writePtr , PKG_SIZE );
#if(0)
           if(first){
              struct USBmem_t* unsafe mem=(struct USBmem_t* unsafe)writePtr;
                for(int i=0;i<4;i++)
                    printintln( mem->fast.IA[i]);
                first--;
                }
#endif
                writePtr +=PKG_SIZE;
                buffer_writing2host = 1;
            }
            else
                buffer_writing2host = BufferReadyToWrite; // Redundat ??
        break;
        case c_temp:> USBmem[0].temp:
        USBmem[1].temp = USBmem[0].temp;
        //printf("%f\n",USBmem[0].temp);
        break;
        case cdc.data_available():
            while(buff->rx.queue_len1 > 0){
#if(DEBUG == 1)
                printf("CDC commad:%d , queue depth=%d ,read-pos = %d\n" , rx->header.command , buff->rx.queue_len1 , buff->rx.read1 - buff->rx.fifo1);
                printf("CDC IN: %f\n" , rx->data.a);
#endif
                switch(*buff->rx.read1++){
                 case streamOUT:
                    int state = *buff->rx.read1++;
                    soutct(c_from_RX , 5); // Reset all states in RX
                    if(state)
                        printf("stream out: ON\n");
                    else
                        printf("stream out: OFF\n");
                        c_from_dsp <: streamOUT;
                        c_from_dsp <: state; //Tell dsp core to start/stop sending sync signals

                break;
                 case PIsection:{
                     int ch = buff->rx.read1[0];
                     void* unsafe ptr = &reg[ch];
                     //printf("CDC PI %d\n" , reg);
                    // struct PI_t* pi = &reg[ch];
                     {
                         c_from_dsp  <: PIsection;
                         c_from_dsp  <: ptr;
                         c_from_dsp  <: buff->rx.read1[3];
                         c_from_dsp  <: buff->rx.read1[4];
                     }
                     buff->rx.read1 += 5;
                 }
                     break;
                 case EQsection:{
                     //see struct USB_EQsection_t in QT usbbulk.h
                     int i;
                     int ch =  buff->rx.read1[0];
                     int sec = buff->rx.read1[1];
                     int section = 2*ch +sec;
                     int active = buff->rx.read1[3];
                     void* unsafe ptr = &reg[ch].EQ[sec];
                     //printf("CDC EQ %d\n" , reg);
                    c_from_dsp <: EQsection;
                     c_from_dsp <:  ptr;
#pragma loop unroll
                     {

                         for(i=8 ; i< 11 ; i++)
                             c_from_dsp <: buff->rx.read1[i];  //B coeffs
                         for(i ; i< 13 ; i++)
                             c_from_dsp <: -buff->rx.read1[i]; //Invert A coeffs
                     }
                     if(active)
                         c_from_dsp <: buff->rx.read1[i]; //shift
                     else{
                         c_from_dsp <: -1; //Bypass section...
                         c_from_dsp <: resetEQsec;
                         c_from_dsp <: &dsp_state[section];//... and reset states

                     }
                     buff->rx.read1 +=i+1;
                     }
                     break;
                 case resetPI:
                     c_from_dsp <:resetPI;
                     c_from_dsp <:buff->rx.read1[0];
                     buff->rx.read1++;
                     break;
                 case resetEQ:
                     unsafe{
                         for(int i=0; i<4 ; i++){
                             c_from_dsp <: resetEQsec;
                             c_from_dsp <: &dsp_state[i];
                         }
                     }
                     break;
                 case FuseCurrent:
                     const float gain = 16384.0f; // for testing
                     int current = gain*(buff->rx.read1[0] , float);
                     buff->rx.read1++;
                     c_from_dsp <: FuseCurrent;
                     c_from_dsp <: current;

                     break;
                 case FuseStatus:
                     int state = buff->rx.read1[0];
                     buff->rx.read1++;
                     if(state){
                         //Reset all states
                         unsafe{
                             for(int i=0; i<4 ; i++){
                                 c_from_dsp <: resetEQsec;
                                 c_from_dsp <: &dsp_state[i];

                             }
                             c_from_dsp <:resetPI;
                             c_from_dsp <:0;
                             c_from_dsp <:resetPI;
                             c_from_dsp <:1;
                         }
                     }
                     c_from_dsp <: FuseStatus;
                     c_from_dsp <: state;

                     break;
                default:
                    printf("Unknown command %d %d %d\n" , buff->rx.read1[0] , buff->rx.read1[1],buff->rx.read1[2]);
                    break;
                }


#if(DEBUG)
                    printf("CDC: Reset RX write pos\n");
#endif


                if(buff->rx.read1 >= buff->rx.fifoLen1){  //Passed last written place in FIFO
                    buff->rx.read1 = buff->rx.fifo1;      //reset to start of FIFO1

#if(DEBUG)
                    printf("CDC: Reset read RX pos\n");
#endif
                    }

                buff->rx.queue_len1--;
            }
            cdc.queue_empty();
            break;
            case XUD_SetData_Select(c_ep_in[1],  buff->tx.ep[1] , result):
                if(result == XUD_RES_RST){
                    result = XUD_ResetEndpoint(buff->tx.ep[1] , null);
#if(DEBUG_RESET)
                    printf("CDC: !! RESET!! in SETDATA result = %d\n" , result);
#endif               //XUD_SetReady_In(   buff->tx.ep[1] , (char*) buff->tx.fifo1 ,0);
                break;
                }
                if(result == XUD_RES_ERR){
                    printf("CDC: !! RES_ERROR!! in SETDATA");
                    break;
                }
                if(buffer_writing2host >= BufferReadyToWrite){
                    XUD_SetReady_In(buff->tx.ep[1] ,(char*) writePtr , PKG_SIZE );
                    writePtr +=PKG_SIZE;
                    buffer_writing2host++;
                    if(buffer_writing2host >= 8)
                        buffer_writing2host = InEPready;
                    break;
                }
                buffer_writing2host = InEPready;
              break;
#if N_CDC>0
            case XUD_SetData_Select(c_ep_in[3],  buff->tx.ep[3] , result):
#endif
                    break;
        }//select
    }
}

