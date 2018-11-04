/*
 * transfer_block.h
 *
 *  Created on: 28 okt 2018
 *      Author: micke
 */


#ifndef TRANSFER_BLOCK_H_
#define TRANSFER_BLOCK_H_

unsafe void RX_block(streaming chanend c ,  struct USBmem_t* unsafe mem , unsigned &mid , unsigned &fast);
void TX_block(streaming chanend c ,  struct DSPmem_t* unsafe mem , enum pos_e &i , unsigned &slow);

#endif /* TRANSFER_BLOCK_H_ */
