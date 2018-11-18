/*
 * cdc_handlers.xc
 *
 *  Created on: 27 okt 2018
 *      Author: micke
 */

#include <stdio.h>
#include <math.h>
#include "structs.h"
#include "cdc_handlers.h"
#include "calc.h"




//cdc data is queued on a FIFO
unsafe void cdc_handler1(client interface cdc_if cdc , chanend c_from_dsp , streaming chanend c_from_RX, chanend c_temp  , chanend c_ep_in[],XUD_buffers_t * unsafe buff){
    XUD_Result_t result;
    struct master_setting_t settings;
    int buffer_writing2host=InEPready;
    int first;
    unsigned blockNumber;
    char* unsafe writePtr;
    struct USBmem_t* unsafe USBmem;
    c_from_RX :> USBmem;
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
                     struct USB_PIsection_t* usbData = (struct USB_PIsection_t*) buff->rx.read1;
                     int ch = usbData->channel;
//                   printf("Ch:%d:Fc=%.1f Gain=%.1f\n" , ch, usbData->section.Fc , usbData->section.Gain);
                     struct PI_section_t* PI =  &settings.channel[ch].PI;
                     PI->Fc =  usbData->section.Fc;
                     PI->Gain = usbData->section.Gain;
                     calcPI_fixedpoint(c_from_dsp , *PI , ch);
                     buff->rx.read1 += sizeof(usbData)/(sizeof(int));
                 }
                     break;
                 case EQsection:{
                     int i;
                     c_from_dsp <: EQsection;
                     master{
                         c_from_dsp <:  buff->rx.read1[0];
                         c_from_dsp <:  buff->rx.read1[1];
                         for(i=8 ; i< 13 ; i++)
                             c_from_dsp <: buff->rx.read1[i];
                     }
                     buff->rx.read1 +=i;
                     }
                     break;
                default:
                    printf("Unknown command\n");
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

