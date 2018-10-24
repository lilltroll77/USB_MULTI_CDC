/*
 * USB_CDC.xc
 *
 *  Created on: 15 okt 2018
 *      Author: micke
 */

#include "usb.h"
#include "xud.h"
#include <platform.h>
#include <stdio.h>
#include <stdlib.h>
#include <xclib.h>
#include "string.h"
#include "structs.h"
#include "descriptors.h"

#define DEBUG 0
#define DEBUG_RESET 0


typedef unsigned long long u64;
typedef unsigned long long s64;

extern XUD_Result_t ControlInterfaceClassRequests(XUD_ep ep_out, XUD_ep ep_in, USB_SetupPacket_t sp);

#if(BLOCKSIZE >128)
#error "Maximum block size in bulk mode is 512 bytes in USB 2.0"
#endif


interface cdc_if{
   [[notification]] slave void data_available(void);
   [[clears_notification]] void queue_empty(void);
   [[guarded]] void writeblock(int* ptr);
};


unsafe void resetPointers(XUD_buffers_t &buffer){
    // init all read and write pointers to the beginning of the corresponding FIFO
    buffer.rx.read1 =  buffer.rx.fifo1;
    buffer.rx.write1 = buffer.rx.fifo1;
    buffer.rx.fifoLen1 = buffer.rx.fifo2;

    buffer.tx.read1 =  buffer.tx.fifo1;
    buffer.tx.write1 = buffer.tx.fifo1;
}
//Only work for CDC class
unsafe unsigned char* unsafe writeIAD(unsigned char* unsafe ptr , unsigned char intf){
    memcpy(ptr , IAD_CDC , sizeof(IAD_CDC));
    ptr[2]=intf; //bFirstInterface
    return ptr+sizeof(IAD_CDC);
}

unsafe unsigned char* unsafe writeCDC(unsigned char* unsafe ptr , struct descriptor_t cdc){
    memcpy(ptr , CDC , sizeof(CDC));
    ptr[2]= cdc.intf.notification;
    ptr[21]=cdc.intf.notification;
    ptr[22]=cdc.intf.data;
    ptr[30]=cdc.EP.notification;
    ptr[37]=cdc.intf.data;
    ptr[45]=cdc.EP.rx;
    ptr[52]=cdc.EP.tx | 0x80;
    return ptr+sizeof(CDC);

}


/* Endpoint 0 handling both std USB requests and CDC class specific requests */
unsafe void Endpoints(server interface cdc_if cdc[] , chanend chan_ep_out[] , XUD_buffers_t &buffer)
{

    struct descriptor_t cdc_desc;

    unsafe{
        unsigned char* unsafe ptr = &cfgDesc[cfgDescHeadSize];
#if(N_CDC>0)
        // memcpy(ptr , IAD_BULK , sizeof(IAD_BULK));
        // ptr += sizeof(IAD_BULK);
         devDesc[4] = 0xEF;  /* 4  bDeviceClass - USB Class for IAD*/
         devDesc[5] = 0x02;  /* 5  bDeviceSubClass*/
         devDesc[6] = 0x01;
#endif
        memcpy( ptr , cfgBulkDesc , sizeof(cfgBulkDesc)); //Interface 0
        ptr += sizeof(cfgBulkDesc);

        const int offset=1;
        for(int i=0; i<N_CDC ; i++){
            int i2=2*i;
            cdc_desc.intf.notification=  i2+  offset; //Interface 1,3,5
            cdc_desc.intf.data =         i2+1+offset; //Interface 2,4,6
            cdc_desc.EP.notification =   i2+1+offset; //EP 2,4,6,8
            cdc_desc.EP.rx =             i+1+ offset; //EP 2,3,4,5,6
            cdc_desc.EP.tx =             i2+2+offset; //EP 3,5,7,9
            ptr = writeIAD(ptr , cdc_desc.intf.notification); // ( 1,3,5,7) //First interface
            ptr = writeCDC(ptr , cdc_desc);
        }
    }

    USB_SetupPacket_t sp;
    XUD_BusSpeed_t usbBusSpeed;
    buffer.reset_N =1;

    unsigned char sbuffer[120]={0};
    unsigned length;
    XUD_Result_t result;
    XUD_SetReady_Out(buffer.rx.ep[0] , sbuffer);

    XUD_SetReady_Out(buffer.rx.ep[1], (char*) buffer.rx.fifo1);
#if(N_CDC>0)
    XUD_SetReady_Out(buffer.rx.ep[2], (char*) buffer.rx.fifo2);
#endif
//XUD_SetReady_Out(buffer.rx.ep[2], (buffer.rx.fifo2 , unsigned char []));

    buffer.rx.pkg_maxSize1=64;


    //safememcpy(&devDesc[0xEE] , MSOS20PlatformCapabilityDescriptor , sizeof(MSOS20PlatformCapabilityDescriptor));
    //devDesc[0] = sizeof(devDesc);

    while(1){
        select{
          // XUD_GetSetupData (chanend c, XUD_ep ep, REFERENCE_PARAM(unsigned, length), REFERENCE_PARAM(XUD_Result_t, result));
        case XUD_GetSetupData_Select( chan_ep_out[0] , buffer.rx.ep[0] , length , result):
                USB_ParseSetupPacket(sbuffer, sp); // Data in sbuffer
        if(result== XUD_RES_OKAY){
              printf("SetupData OK\n");
              /* Set result to ERR, we expect it to get set to OKAY if a request is handled */
              result = XUD_RES_ERR;
              /* Stick bmRequest type back together for an easier parse... */
#if N_CDC>0
              unsigned bmRequestType = (sp.bmRequestType.Direction<<7) |
                      (sp.bmRequestType.Type<<5) |
                      (sp.bmRequestType.Recipient);
              if ((bmRequestType == USB_BMREQ_H2D_STANDARD_DEV) &&
                      (sp.bRequest == USB_SET_ADDRESS))
              {  /* Host has set device address, value contained in sp.wValue*/
                 printf("set device address:%d\n",sp.wValue);
              }
              /* Inspect for CDC Communications Class interface num */
              if(sp.wIndex == 0){

                  switch(bmRequestType)
                  {
                  /* Direction: Device-to-host and Host-to-device
                   * Type: Class
                   * Recipient: Interface
                   */
                  case USB_BMREQ_H2D_CLASS_INT:
                  case USB_BMREQ_D2H_CLASS_INT:

                      /* Returns  XUD_RES_OKAY if handled,
                       *          XUD_RES_ERR if not handled,
                       *          XUD_RES_RST for bus reset */
                      result = ControlInterfaceClassRequests(buffer.rx.ep[0], buffer.tx.ep[0], sp);
                      printf("ControlInterfaceClassRequests=%d" , result);
                  break;
                  default:
                      printf("WARNING: Unknown bmRequestType:%d , MASK:0x%x bRequest:%d\n" , bmRequestType , USB_BMREQ_D2H_CLASS_INT ,sp.bRequest);
                      break;
                  }
              }//if
              else
                printf("wTindex=0x%x\n",sp.wIndex);

#endif
              //break;
        }
        if(result == XUD_RES_ERR){
             printf("USB_StandardRequests for enumeration\n");
             result = USB_StandardRequests(buffer.rx.ep[0], buffer.tx.ep[0], devDesc,
                     sizeof(devDesc), cfgDesc, sizeof(cfgDesc),
                     null, 0,
                     null, 0,
                     stringDescriptors, sizeof(stringDescriptors)/sizeof(stringDescriptors[0]),
                     sp, usbBusSpeed);
        }
        if(result==XUD_RES_RST)
            usbBusSpeed = XUD_ResetEndpoint(buffer.rx.ep[0], buffer.tx.ep[0]);

        memset(sbuffer , 0 , sizeof(sbuffer));
        XUD_SetReady_Out(buffer.rx.ep[0] , sbuffer);

        //switch
        //XUD_SetReady_Out(buffer.rx.ep[0] , sbuffer);

        break;
// Get DATA
          case XUD_GetData_Select(chan_ep_out[1],  buffer.rx.ep[1] , length ,result):
                        length /=sizeof(int);
#if(DEBUG == 1)
        printf("EP: GetData, len=%d\n" , length);
#endif
        if(result == XUD_RES_RST){
            result = XUD_ResetEndpoint(buffer.rx.ep[1] , null);
            // //block Set data reset
            resetPointers(buffer);
            XUD_SetReady_Out(buffer.rx.ep[1], (char*) buffer.rx.write1);
#if(DEBUG_RESET)
            printf("EP: !! RESET!! in GETDATA, res=%d" , result);
#endif
            break;
        }
        if(result == XUD_RES_ERR)
            printf("EP: !! error!! in GETDATA");
        else if(result == XUD_RES_OKAY){
            //buffer.tx.reset=0; // release block
            buffer.rx.queue_len1++;
            cdc[0].data_available();
            //char* write = (char*)buffer.rx.write1;
            //*write+= length;                // move write pointer !! length is in bytes !!
            buffer.rx.write1 +=length/sizeof(buffer.rx.write1);

          //reseting RX write
            if( buffer.rx.write1 >= buffer.rx.fifo2 - buffer.rx.pkg_maxSize1){ //does it risk to start writing into fifo2?
             buffer.rx.fifoLen1 = buffer.rx.write1; //update actual FIFO len that has data
             buffer.rx.write1 = buffer.rx.fifo1;
             printf("CDC: Reset RX write pos\n");
         }
#if(DEBUG == 1)
           printf("EP: WritePos=%d\n" , buffer.rx.write1);
#endif
        }
        // add to queue
         XUD_SetReady_Out(buffer.rx.ep[1] , (char*) buffer.rx.write1 );
        break;
        case cdc[0].writeblock(int* data):
           XUD_SetReady_In(  buffer.tx.ep[1] ,(char*) data , BLOCKSIZE*sizeof(int));
           // printf("Write block\n");
                break;

        case cdc->queue_empty():
         break;
// DEFAULT
        default:
            if( buffer.tx.read1 != buffer.tx.write1){
                XUD_SetReady_In(  buffer.tx.ep[1] , (char*) buffer.tx.read1 , 4);
#if(DEBUG == 1)
                printf("TX package qeueud: %f , %f , read=%d write=%d\n" , (*buffer.tx.read1,float), (float) *buffer.tx.write1 , buffer.tx.read1 - buffer.tx.fifo1 , buffer.tx.write1- buffer.tx.fifo1);
#endif
                buffer.tx.read1++;
                //RESETTING TX Read ptr
                if(buffer.tx.read1 == buffer.tx.fifo2){  //pointer has reached next buffer
                    buffer.tx.read1 = buffer.tx.fifo1;    // reset to beginning of buffer
#if(DEBUG)
                    printf("CDC: Reset TX read pos\n");
#endif
                }
            }
                break;


        }
    }
}



//XUD_SetReady_In(  ep_in[1],  (buffers.rx0 , unsigned char []) , length);
//        rx_ptr += length;
//        if( rx_ptr >= (sizeof( buffer.rx0)-512 ))
//            XUD_SetReady_Out(ep_out[1],  &(buffers.rx0 , unsigned char [])[rx_ptr]);

enum message_e{A , B , streamOUT};

interface gui_if{
    [[guarded]] float setA(float a);
    [[guarded]] float setB(float b);
    [[guarded]] void writeBlockToHost(int data[]);
};

union rx_u{
    float a;
    float b;
    int stream;
};

struct header_t{
    int command;
};

typedef struct{
    struct header_t header;
    union rx_u data;
}rx_t;

unsafe void cdc_handler1(client interface cdc_if cdc , client interface gui_if gui , chanend c_ep_in[], XUD_buffers_t * unsafe buff){
    XUD_Result_t result;
    rx_t* rx = (rx_t*) buff->rx.read1;
    float prod;
    int stream_active=0;
    int block[2][BLOCKSIZE];
    int block_i=0;

    while(1){
        select{
        case cdc.data_available():
            while(buff->rx.queue_len1 > 0){
#if(DEBUG == 1)
                printf("CDC commad:%d , queue depth=%d ,read-pos = %d\n" , rx->header.command , buff->rx.queue_len1 , buff->rx.read1 - buff->rx.fifo1);
                printf("CDC IN: %f\n" , rx->data.a);
#endif
                switch(rx->header.command){
                case A:
                    prod = gui.setA(rx->data.a);
                    *buff->tx.write1 = (prod , int);
                    #if(DEBUG == 1)
                    printf("CDC A: prod = %f | new readpos=%d\n" , prod ,buff->rx.read1- buff->rx.fifo1);
#endif
                    break;
                case B:
                     prod = gui.setB(rx->data.b);
                     *buff->tx.write1 = (prod , int);
                     #if(DEBUG == 1)
                     printf("CDC B: prod = %f\n" , prod );
#endif
                     break;
                case streamOUT:
                    stream_active = rx->data.stream;
                    if(stream_active){ // write first block
                     gui.writeBlockToHost( block[block_i]);
                     cdc.writeblock( block[block_i]);
                     block_i =! block_i;
                     gui.writeBlockToHost( block[block_i]); // prepare next block
                    }

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
                if(stream_active){
                    cdc.writeblock( block[block_i]);
                    block_i = !block_i;
                    gui.writeBlockToHost( block[block_i]);
                }
             break;
#if N_CDC>0
            case XUD_SetData_Select(c_ep_in[3],  buff->tx.ep[3] , result):
#endif
                    break;
        }//select
    }
}

void usb_server(client interface gui_if gui){
    chan c_ep_out[XUD_EP_COUNT_OUT], c_ep_in[XUD_EP_COUNT_IN];
    interface cdc_if cdc[1];

        XUD_buffers_t buffer={0};
        unsafe{
            resetPointers(buffer);
        par{
            xud(c_ep_out, XUD_EP_COUNT_OUT, c_ep_in, XUD_EP_COUNT_IN , null ,  XUD_SPEED_HS, XUD_PWR_SELF);
            {
                //Init all endpoints in XUD
                int etype;
                for(int i=0 ; i <XUD_EP_COUNT_OUT; i++){
                    switch(i){
                    case 0:
                        etype= XUD_EPTYPE_CTL | XUD_STATUS_ENABLE;
                        break;
                    case 1:
                        etype = XUD_EPTYPE_BUL | XUD_STATUS_ENABLE;
                        break;
                    default:
                        etype = XUD_EPTYPE_BUL;
                        break;

                    }
                    buffer.rx.ep[i] = XUD_InitEp(c_ep_out[i] , etype);
                }// for

                for(int i=0 ; i <XUD_EP_COUNT_IN; i++){
                    switch(i){
                    case 0:
                        etype= XUD_EPTYPE_CTL | XUD_STATUS_ENABLE;
                        break;
                    case 1:
                        etype= XUD_EPTYPE_BUL | XUD_STATUS_ENABLE;
                        break;
                    case 2:
                        etype= XUD_EPTYPE_INT; //Only used for setup !?
                        break;
                    case 3:
                        etype= XUD_EPTYPE_BUL;
                        break;
                    default:
                        etype = i&1 ? XUD_EPTYPE_BUL : XUD_EPTYPE_INT;
                        break;
                    }
                    buffer.tx.ep[i]  = XUD_InitEp(c_ep_in[i]  , etype );
                } // for

                XUD_buffers_t* unsafe buffer_ptr = &buffer;
                par{
                    Endpoints(cdc , c_ep_out ,  buffer);
                    cdc_handler1(cdc[0] , gui , c_ep_in , buffer_ptr);
                }

            }//guards
        }
    }
}




void gui_server(server interface gui_if gui){
    double a=0,b=0;
    int test_block[BLOCKSIZE];
    for(int i=0; i<BLOCKSIZE ; i++)
        test_block[i]=i;
    while(1){
        select{
            case gui.setA(float a_in) -> float prod:
                a = a_in;
                prod = a*b;
                //printf("GUI Ain: %g*%g=%g\n" , a , b , prod);
                break;
            case gui.setB(float b_in) -> float prod:
                b=b_in;
                prod = a*b;
                //printf("GUI Bin: %g*%g=%g\n" , a , b , prod);
                break;
            case gui.writeBlockToHost(int data[]):
                memcpy(data , test_block , sizeof(test_block));
            break;
        }
    }
}

int main(){
    interface gui_if gui;
    par{
        on tile[0]:gui_server( gui );
        on tile[1]:usb_server( gui );
    }
    return 0;
}
