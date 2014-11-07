#ifndef _ADC_H_
#define _ADC_H_

#include "stdbool.h"

extern volatile bool ADCDone;	/* ADC Done flag */
extern volatile unsigned int ADCValue;	/* Measured ADC Value */

void Single_Measure(unsigned int chan);
void Single_Measure_REF(unsigned int chan, unsigned int ref);

#endif /* _ADC_H_ */
