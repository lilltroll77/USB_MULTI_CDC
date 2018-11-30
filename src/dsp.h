/*
 * dsp.h
 *
 *  Created on: 27 okt 2018
 *      Author: micke
 */


#ifndef DSP_H_
#define DSP_H_

#include "structs.h"
typedef long long int s64;

struct state_t{
    s64 y1;
    s64 y2;
    int x1;
    int x2;
    unsigned error;

};

struct EQ_t{
    int B0;
    int B1;
    int B2;
    int A1;
    int A2;
    int shift;
};

struct regulator_t{
    int I;
    int P;
    struct EQ_t EQ[2];
};



void MLSgen(streaming chanend c);
unsafe void DSP(streaming chanend c_from_GUI , streaming chanend c_from_CDC , streaming chanend c_from_MLS);


#endif /* DSP_H_ */
