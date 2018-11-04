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
#include "cdc_handlers.h"
#include "gui_server.h"

#define DEBUG 0
#define DEBUG_RESET 0


typedef unsigned long long u64;
typedef unsigned long long s64;



#if(BLOCKSIZE >128)
#error "Maximum block size in bulk mode is 512 bytes in USB 2.0"
#endif








/* Endpoint 0 handling both std USB requests and CDC class specific requests */




//XUD_SetReady_In(  ep_in[1],  (buffers.rx0 , unsigned char []) , length);
//        rx_ptr += length;
//        if( rx_ptr >= (sizeof( buffer.rx0)-512 ))
//            XUD_SetReady_Out(ep_out[1],  &(buffers.rx0 , unsigned char [])[rx_ptr]);











