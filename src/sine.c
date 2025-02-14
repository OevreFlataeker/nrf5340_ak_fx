#include "sine.h"
#include "math.h"

int16_t sine[48000] = {};

void createsine() {
    // Creates a sine table with N/4 precalculated values of a 1Hz sine
    #define PI 3.141592
    double t = 0;
    int idx = 0;
    double val = 0.0;
    while (idx<6000) {         
         val = sin(2*PI*t);
         sine[idx]= val;
         t+=1.0/48000.0;
         idx++;
    }
}
