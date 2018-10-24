/*
 * structs.h
 *
 *  Created on: 23 okt 2018
 *      Author: micke
 */
#include "usb.h"

#ifndef STRUCTS_H_
#define STRUCTS_H_
#define FIFO1_LEN 1024
#define FIFO2_LEN 64
#define BLOCKSIZE 128
#define N_CDC 1 // Needs clean -> build after change
#define XUD_EP_COUNT_OUT ( 2 + N_CDC)
#define XUD_EP_COUNT_IN  (2+ 2*N_CDC)



//recieve data from host -OUT
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


//transmit data to host -IN
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

struct type_t{
    unsigned char notification;
    unsigned char rx;
    unsigned char tx;
    unsigned char data;
};


struct descriptor_t{
    struct type_t intf;
    struct type_t EP;
};
/*
typedef struct
{
    struct USB_Descriptor_Configuration_Header_t configHeader;
    char HFD[5];
    char ACM[4];
    char UNION[4];
    char Management[5];
    struct USB_Descriptor_Endpoint_t epNot;
    struct USB_Descriptor_Interface_t If;
    USB_Descriptor_Endpoint_t epOut;
    USB_Descriptor_Endpoint_t epIn[1];
}cfgCDC_t;
*/
#endif /* STRUCTS_H_ */
