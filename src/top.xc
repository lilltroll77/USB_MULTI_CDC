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
#include <math.h>

void Thermometer(chanend c_temp){
    float temp;
    timer tmr;
    float scale = powf(2 , -29);
    unsigned t;
    tmr:>t;
    while(1){
        temp = 25.0 + (float)t*scale;
        c_temp <: temp;
        tmr when timerafter(t+2e7):>t;
    }
}

int main(){
    streaming chan sc_dsp2GUI , sc_GUI2RX , c_dsp2CDC , c_dsp2LMS;
    chan c_temp;
        par{

            on tile[0]: unsafe{
            par{
                MLSgen(c_dsp2LMS);
                DSP(sc_dsp2GUI , c_dsp2CDC , c_dsp2LMS);
                gui_server(sc_GUI2RX , sc_dsp2GUI );
                Thermometer(c_temp);
            }}
            on tile[1]:usb_server( c_dsp2CDC , sc_GUI2RX ,c_temp);

    }
    return 0;
}
