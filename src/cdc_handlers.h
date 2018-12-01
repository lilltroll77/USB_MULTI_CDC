/*
 * cdc_handlers.h
 *
 *  Created on: 27 okt 2018
 *      Author: micke
 */

#ifndef CDC_HANDLERS_H_
#define CDC_HANDLERS_H_

#include "structs.h"

interface cdc_if{
   [[notification]] slave void data_available(void);
   [[clears_notification]] void queue_empty(void);
};




enum INstatus_e{InEPready=-2 , BufferWritten=-1 , BufferReadyToWrite=0};
enum message_e{streamOUT, PIsection , EQsection , resetPI , resetEQsec , resetEQ , FuseCurrent , FuseStatus};

unsafe void cdc_handler1(client interface cdc_if cdc , streaming chanend c_from_dsp , streaming chanend c_from_RX , chanend c_temp , chanend c_ep_in[], XUD_buffers_t * unsafe buff);



#endif /* CDC_HANDLERS_H_ */
