/*
 * usb_server.h
 *
 *  Created on: 27 okt 2018
 *      Author: micke
 */


#ifndef USB_SERVER_H_
#define USB_SERVER_H_



void usb_server(streaming chanend c_dsp2CDC , streaming chanend sc_GUI2RX, chanend c_temp ,  chanend c_sigGen2CDC);
unsafe void resetPointers(XUD_buffers_t &buffer);
unsafe void RX_block(streaming chanend c_from_gui , streaming chanend c_from_CDC);

#endif /* USB_SERVER_H_ */
