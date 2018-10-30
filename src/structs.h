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
#define BLOCKSIZE 64
#define N_CDC 0 // Needs clean -> build after change
#define XUD_EP_COUNT_OUT ( 2 + N_CDC)
#define XUD_EP_COUNT_IN  (2+ 2*N_CDC)

#define PKG_SIZE 512 /*In bytes*/

union rx_u{
    int stream;
};

struct header_t{
    int command;
};

typedef struct{
    struct header_t header;
    union rx_u data;
}rx_t;



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




//sizeof(struct DSPmem_t)
//sizeof(struct DSPmem_t)

enum pos_e{Slow , Pos , Vel , Perror , Reserved1 , Reserved2 , Reserved3 , Reserved4};

struct lowspeed_t{
    int temp;
    int reserved1; // Replace with real signal
    int reserved2;
    int reserved3;
    int reserved4;
    int reserved5;
    int reserved6;
    int reserved7;
    int reserved8;
    int reserved9;
    int reserved10;
    int reserved11;
    int reserved12;
    int reserved13;
};



struct midspeed_t{
    int pos;
    int vel;
    int perror;
    int reserved1;
    int reserved2;
    int reserved3;
    int reserved4;
};

struct midspeed_vecotr_t{
    int pos[PKG_SIZE/32];
    int vel[PKG_SIZE/32];
    int perror[PKG_SIZE/32];
    int reserved1[PKG_SIZE/32];
    int reserved2[PKG_SIZE/32];
    int reserved3[PKG_SIZE/32];
    int reserved4[PKG_SIZE/32];
};

struct hispeed_t{
    int IA;
    int IC;
    int QE;
    int Torque;
    int Flux;
    int U;
    int angle;
};

struct hispeed_vector_t{
    int IA[PKG_SIZE/4];
    int IC[PKG_SIZE/4];
    int QE[PKG_SIZE/4];
    int Torque[PKG_SIZE/4];
    int Flux[PKG_SIZE/4];
    int U[PKG_SIZE/4];
    int angle[PKG_SIZE/4];
};

struct USBmem_t{
    unsigned checknumber;
    unsigned index;
    struct lowspeed_t slow;
    struct midspeed_vecotr_t mid;
    struct hispeed_vector_t fast;
};

struct DSPmem_t{
    struct lowspeed_t slow;
    struct midspeed_t mid;
    struct hispeed_t fast;
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
