/*
 * top.xc
 *
 *  Created on: 27 okt 2018
 *      Author: micke
 */
#include <platform.h>
#include "structs.h"
#include "gui_server.h"
#include "dsp.h"
#include "usb_server.h"

int main(){
    streaming chan sc_dsp2GUI , sc_GUI2RX;
    chan c_dsp2CDC;
        par{

            on tile[0]: unsafe{
            par{
                DSP(sc_dsp2GUI , c_dsp2CDC);
                gui_server(sc_GUI2RX , sc_dsp2GUI );
            }}
            on tile[1]:usb_server( c_dsp2CDC , sc_GUI2RX );

    }
    return 0;
}
