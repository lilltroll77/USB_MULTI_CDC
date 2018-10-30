/*
 * cdc_handlers.h
 *
 *  Created on: 27 okt 2018
 *      Author: micke
 */


#ifndef CDC_HANDLERS_H_
#define CDC_HANDLERS_H_

interface cdc_if{
   [[notification]] slave void data_available(void);
   [[clears_notification]] void queue_empty(void);
};
enum INstatus_e{InEPready=-2 , BufferWritten=-1 , BufferReadyToWrite=0};
enum message_e{streamOUT};

unsafe void cdc_handler1(client interface cdc_if cdc , chanend c_from_dsp , streaming chanend c_from_RX  , chanend c_ep_in[], XUD_buffers_t * unsafe buff);



#endif /* CDC_HANDLERS_H_ */
