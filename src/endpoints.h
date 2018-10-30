/*
 * endpoints.h
 *
 *  Created on: 27 okt 2018
 *      Author: micke
 */


#ifndef ENDPOINTS_H_
#define ENDPOINTS_H_

#include "structs.h"
#include "cdc_handlers.h"


unsafe void Endpoints(server interface cdc_if cdc[] , chanend chan_ep_out[] , XUD_buffers_t &buffer);


#endif /* ENDPOINTS_H_ */
