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
#include <xclib.h>
#include "string.h"
#define DEBUG 0
#define DEBUG_RESET 0


#define XUD_EP_COUNT_OUT 2
#define XUD_EP_COUNT_IN 2


typedef unsigned long long u64;
typedef unsigned long long s64;

#define FIFO1_LEN 1024
#define FIFO2_LEN 64
#define BLOCKSIZE 16

//recieve data from host
struct XUDbufferRx_t{
    int* unsafe write1;
    int* unsafe write2;
    int* unsafe read1;
    int* unsafe read2;
    int* unsafe fifoLen1;
    int* unsafe fifoLen2;
    unsigned pkg_maxSize1;
    unsigned pkg_maxSize2;
    unsigned queue_len1;
    unsigned queue_len2;
    XUD_ep ep[XUD_EP_COUNT_OUT];
    int fifo1[FIFO1_LEN];
    int fifo2[FIFO2_LEN];
};


//transmir data to host
struct XUDbufferTx_t{
    int* unsafe write1;
    int* unsafe write2;
    int* unsafe read1;
    int* unsafe read2;
    unsigned ready1;
    unsigned ready2;
    unsigned dataWaiting1;
    unsigned dataWaiting2;
    unsigned queue_len1;
    unsigned queue_len2;
    XUD_ep ep[XUD_EP_COUNT_IN];
    int fifo1[FIFO1_LEN];
    int fifo2[FIFO2_LEN];
};

typedef struct{
    struct XUDbufferTx_t tx;
    struct XUDbufferRx_t rx;
    int reset_N;
}XUD_buffers_t;

#define BCD_DEVICE              0x1000
#define VENDOR_ID               0x20B1
#define PRODUCT_ID              0x00CC
#define MANUFACTURER_STR_INDEX  0x0001
#define PRODUCT_STR_INDEX       0x0002

/* Vendor specific class defines */
#define VENDOR_SPECIFIC_CLASS    0xff
#define VENDOR_SPECIFIC_SUBCLASS 0x00
#define VENDOR_SPECIFIC_PROTOCOL 0x00




static unsigned char MSOS20PlatformCapabilityDescriptor[] =
{
    //
    // Microsoft OS 2.0 Platform Capability Descriptor Header
    //
    0x1C,                    // bLength - 28 bytes
    0x10,                    // bDescriptorType - 16
    0x05,                    // bDevCapability – 5 for Platform Capability
    0x00,                    // bReserved - 0
    0xDF, 0x60, 0xDD, 0xD8,  // MS_OS_20_Platform_Capability_ID -
    0x89, 0x45, 0xC7, 0x4C,  // {D8DD60DF-4589-4CC7-9CD2-659D9E648A9F}
    0x9C, 0xD2, 0x65, 0x9D,  //
    0x9E, 0x64, 0x8A, 0x9F,  //

    //
    // Descriptor Information Set for Windows 8.1 or later
    //
    0x00, 0x00, 0x03, 0x06,  // dwWindowsVersion – 0x06030000 for Windows Blue
    0x48, 0x00,              // wLength – size of MS OS 2.0 descriptor set
    0x01,                    // bMS_VendorCode
    0x00,                    // bAltEnumCmd – 0 Does not support alternate enum
};



/* Device Descriptor */
static unsigned char devDesc[] =
{
    0x12,                     /* 0  bLength */
    USB_DESCTYPE_DEVICE,      /* 1  bdescriptorType */
    0x00,                     /* 2  bcdUSB */
    0x02,                     /* 3  bcdUSB */
    VENDOR_SPECIFIC_CLASS,    /* 4  bDeviceClass */
    VENDOR_SPECIFIC_SUBCLASS, /* 5  bDeviceSubClass */
    VENDOR_SPECIFIC_PROTOCOL, /* 6  bDeviceProtocol */
    0x40,                     /* 7  bMaxPacketSize */
    (VENDOR_ID & 0xFF),       /* 8  idVendor */
    (VENDOR_ID >> 8),         /* 9  idVendor */
    (PRODUCT_ID & 0xFF),      /* 10 idProduct */
    (PRODUCT_ID >> 8),        /* 11 idProduct */
    (BCD_DEVICE & 0xFF),      /* 12 bcdDevice */
    (BCD_DEVICE >> 8),        /* 13 bcdDevice */
    MANUFACTURER_STR_INDEX,   /* 14 iManufacturer */
    PRODUCT_STR_INDEX,        /* 15 iProduct */
    0x00,                     /* 16 iSerialNumber */
    0x01                      /* 17 bNumConfigurations */
};

/* Configuration Descriptor */
static unsigned char cfgDesc[] =
{
    0x09,                     /* 0  bLength */
    0x02,                     /* 1  bDescriptortype */
    0x20, 0x00,               /* 2  wTotalLength */
    0x01,                     /* 4  bNumInterfaces */
    0x01,                     /* 5  bConfigurationValue */
    0x00,                     /* 6  iConfiguration */
    0x80,                     /* 7  bmAttributes */
    0xFA,                     /* 8  bMaxPower */

    0x09,                     /* 0  bLength */
    0x04,                     /* 1  bDescriptorType */
    0x00,                     /* 2  bInterfacecNumber */
    0x00,                     /* 3  bAlternateSetting */
    0x02,                     /* 4: bNumEndpoints */
    0xFF,                     /* 5: bInterfaceClass */
    0xFF,                     /* 6: bInterfaceSubClass */
    0xFF,                     /* 7: bInterfaceProtocol*/
    0x03,                     /* 8  iInterface */

    0x07,                     /* 0  bLength */
    0x05,                     /* 1  bDescriptorType */
    0x01,                     /* 2  bEndpointAddress */
    0x02,                     /* 3  bmAttributes */
    0x00,                     /* 4  wMaxPacketSize */
    0x02,                     /* 5  wMaxPacketSize */
    0x01,                     /* 6  bInterval */

    0x07,                     /* 0  bLength */
    0x05,                     /* 1  bDescriptorType */
    0x81,                     /* 2  bEndpointAddress */
    0x02,                     /* 3  bmAttributes */
    0x00,                     /* 4  wMaxPacketSize */
    0x02,                     /* 5  wMaxPacketSize */
    0x01                      /* 6  bInterval */
};

/* Set language string to US English */
#define STR_USENG 0x0409

/* String table */
unsafe
{
static char * unsafe stringDescriptors[] =
{
    "\x09\x04",                             // Language ID string (US English)
    "XMOS",                                 // iManufacturer
    "XMOS BLDC motor driver",     // iProduct
    "Custom Interface",                     // iInterface
    "Config",                               // iConfiguration
};
}



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

/* Endpoint 0 handling both std USB requests and CDC class specific requests */
unsafe void Endpoints(server interface cdc_if cdc[] , chanend chan_ep_out[] , XUD_buffers_t &buffer)
{

    USB_SetupPacket_t sp;
    XUD_BusSpeed_t usbBusSpeed;
    buffer.reset_N =1;

    unsigned char sbuffer[120];
    unsigned length;
    XUD_Result_t result;
    XUD_SetReady_Out(buffer.rx.ep[0] , sbuffer);

    XUD_SetReady_Out(buffer.rx.ep[1], (char*) buffer.rx.fifo1);
    //XUD_SetReady_Out(buffer.rx.ep[2], (buffer.rx.fifo2 , unsigned char []));

    buffer.rx.pkg_maxSize1=64;


    //safememcpy(&devDesc[0xEE] , MSOS20PlatformCapabilityDescriptor , sizeof(MSOS20PlatformCapabilityDescriptor));
    //devDesc[0] = sizeof(devDesc);

    while(1){
        select{
          // XUD_GetSetupData (chanend c, XUD_ep ep, REFERENCE_PARAM(unsigned, length), REFERENCE_PARAM(XUD_Result_t, result));
        case XUD_GetSetupData_Select( chan_ep_out[0] , buffer.rx.ep[0] , length , result):
            if (result == XUD_RES_OKAY) {
                //printstrln("XUD_RES_OKAY");
                USB_ParseSetupPacket(sbuffer, sp);
                  result = USB_StandardRequests(buffer.rx.ep[0], buffer.tx.ep[0], devDesc,
                          sizeof(devDesc), cfgDesc, sizeof(cfgDesc),
                          null, 0,
                          null, 0,
                          stringDescriptors, sizeof(stringDescriptors)/sizeof(stringDescriptors[0]),
                          sp, usbBusSpeed);
         }

         /* USB bus reset detected, reset EP and get new bus speed */
             if(result == XUD_RES_RST)
                {
                    usbBusSpeed = XUD_ResetEndpoint(buffer.rx.ep[0], buffer.tx.ep[0]);
#if(DEBUG_RESET)
                    printstrln("XUD_RES_RST");
#endif
                }
             if(result == XUD_RES_ERR)
                 printstr("XUD_RES_ERROR");

             XUD_SetReady_Out(buffer.rx.ep[0] , sbuffer);
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
            printf("Write block\n");
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

enum message_e{A , B , streamOUT1MB};

interface gui_if{
    [[guarded]] float setA(float a);
    [[guarded]] float setB(float b);
    [[guarded]] void writeBlockToHost(int data[]);
};

union rx_u{
    float a;
    float b;
};

struct header_t{
    int command;
};

typedef struct{
    struct header_t header;
    union rx_u data;
}rx_t;

unsafe void cdc_handler1(client interface cdc_if cdc , client interface gui_if gui , chanend c_ep_in, XUD_buffers_t * unsafe buff){
    XUD_Result_t result;
    rx_t* rx = (rx_t*) buff->rx.read1;
    float prod;

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
                case streamOUT1MB:
                    printf("streamOUT1MB\n");
                    for(int i=0; i<(1024/sizeof(int)/BLOCKSIZE) ; i++){
                        gui.writeBlockToHost( block[block_i]);
                        cdc.writeblock(block[block_i]);
                        block_i =! block_i;
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
            case XUD_SetData_Select(c_ep_in,  buff->tx.ep[1] , result):
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
                //buff->tx.queue_len1--;
                //buff->reset_N =1;
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
                buffer.rx.ep[0]= XUD_InitEp(c_ep_out[0], XUD_EPTYPE_CTL | XUD_STATUS_ENABLE);
                for(int i=1 ; i <XUD_EP_COUNT_OUT; i++)
                    buffer.rx.ep[i] = XUD_InitEp(c_ep_out[i] , XUD_EPTYPE_BUL | XUD_STATUS_ENABLE);

                buffer.tx.ep[0]  = XUD_InitEp(c_ep_in[0] , XUD_EPTYPE_CTL | XUD_STATUS_ENABLE);
                for(int i=1 ; i <XUD_EP_COUNT_IN; i++)
                    buffer.tx.ep[i]  = XUD_InitEp(c_ep_in[i]  , XUD_EPTYPE_BUL | XUD_STATUS_ENABLE);

                XUD_buffers_t* unsafe buffer_ptr = &buffer;
                par{
                    Endpoints(cdc , c_ep_out ,  buffer);
                    cdc_handler1(cdc[0] , gui , c_ep_in[1] , buffer_ptr);
                }

            }
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
