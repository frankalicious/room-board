#include <msp430.h>
#include "stdbool.h"
#include "adc.h"

/* http://www.msp430launchpad.com/2010/09/simple-adc-example-on-launchpad.html */

__interrupt void ADC10_ISR (void);

volatile bool ADCDone;	/* ADC Done flag */
volatile unsigned int ADCValue;	/* Measured ADC Value */

__interrupt void ADC10_ISR (void)
{
  ADCValue = ADC10MEM;	/* Saves measured value. */
  ADCDone = true; /* Sets flag for main loop. */
#if 0
  LPM0_EXIT;      /* TODO Test if this is working */
#endif
}

/**
 * Reads ADC 'chan' once using AVCC as the reference.
 **/
void Single_Measure(unsigned int chan)
{
  ADC10CTL0 &= ~ENC;	/* Disable ADC */
  ADC10CTL0 = ADC10SHT_3 + ADC10ON + ADC10IE;	/* 16 clock ticks, ADC On, enable ADC interrupt */
  ADC10CTL1 = ADC10SSEL_3 + chan;	/* Set 'chan', SMCLK */
  ADC10CTL0 |= ENC + ADC10SC; /* Enable and start conversion */
}
 
/**
 * Reads ADC 'chan' once using an internal reference, 'ref' determines if the
 * 2.5V or 1.5V reference is used.
 **/
void Single_Measure_REF(unsigned int chan, unsigned int ref)
{
  ADC10CTL0 &= ~ENC;	/* Disable ADC */
  ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ref + ADC10IE;	/* Use reference, */
/* 16 clock ticks, internal reference on */
/* ADC On, enable ADC interrupt, Internal = 'ref' */
  ADC10CTL1 = ADC10SSEL_3 + chan;	/* Set 'chan', SMCLK */
  __delay_cycles (128);	/* Delay to allow Ref to settle */
  ADC10CTL0 |= ENC + ADC10SC; /* Enable and start conversion */
}
 
