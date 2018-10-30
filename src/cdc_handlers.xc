/*
 * cdc_handlers.xc
 *
 *  Created on: 27 okt 2018
 *      Author: micke
 */

#include <stdio.h>
#include "structs.h"
#include "cdc_handlers.h"



unsafe void cdc_handler1(client interface cdc_if cdc , chanend c_from_dsp , streaming chanend c_from_RX  , chanend c_ep_in[], XUD_buffers_t * unsafe buff){
    XUD_Result_t result;
    rx_t* rx = (rx_t*) buff->rx.read1;
    int buffer_writing2host=InEPready;
    int first;
    unsigned blockNumber;
    char* unsafe writePtr;
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
        case cdc.data_available():
            while(buff->rx.queue_len1 > 0){
#if(DEBUG == 1)
                printf("CDC commad:%d , queue depth=%d ,read-pos = %d\n" , rx->header.command , buff->rx.queue_len1 , buff->rx.read1 - buff->rx.fifo1);
                printf("CDC IN: %f\n" , rx->data.a);
#endif
                switch(rx->header.command){
                 case streamOUT:
                    first=4;
                    soutct(c_from_RX , 5); // Reset all states in RX
                    if(rx->data.stream)
                        printf("stream out: ON\n");
                    else
                        printf("stream out: OFF\n");
                    c_from_dsp <: rx->data.stream; //Tell dsp core to start/stop sending sync signals

                break;
                default:
                    printf("Unknown command: %d\n" , rx->header.command);
                    break;
                }
                buff->tx.write1++;
                //RESETTING TX Write ptr ?
                if(buff->tx.write1 == buff->tx.fifo2){  //reached fifo2 ?
                    buff->tx.write1 = buff->tx.fifo1;  //reset to start of FIFO1
#if(DEBUG)
                    printf("CDC: Reset RX write pos\n");
#endif
                }

                buff->rx.read1++; // update RX read pos with size of rx size
                //RESETTING RX Read ptr
                //printf("%d > %d\n",buff->rx.read1 , buff->rx.fifoLen1);
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

