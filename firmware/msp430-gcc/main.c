#include <msp430.h>
#include "uart/uart.h"
#include "adc/adc.h"
#include "stdbool.h"

#define     LED BIT3
#define     LED_DIR P2DIR
#define     LED_OUT P2OUT
#define     LED_OFF (LED_OUT &= ~LED)
#define     LED_ON (LED_OUT |= LED)

void init_led(void);
void uart_rx_isr(unsigned char c);

void init_led(void)
{
  LED_DIR |= LED;
  LED_OFF;
}

void uart_rx_isr(unsigned char c) {
  uart_putc(c);
  /* P1OUT ^= BIT0;		// toggle P1.0 (red led) */
}

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                  /* Stop WDT */
  BCSCTL1 = CALBC1_1MHZ; /* Set range */
  DCOCTL = CALDCO_1MHZ; /* SMCLK = DCO = 1MHz */
  uart_init();

  /* register ISR called when data was received */
  uart_set_rx_isr_ptr(uart_rx_isr);

  /* init_spi(); */
  /* init_nrf24l01 */
  /* init_adc();                   /\* not needed? *\/ */
  init_led();
  __bis_SR_register(GIE);
  uart_puts((char *)"\n\r***************\n\r");
  uart_puts((char *)"room board\n\r");
  uart_puts((char *)"***************\n\r\n\r");
  ADCDone = false;
  Single_Measure(INCH_0);
#if 0
  Single_Measure_REF(INCH_10, 0);	/* Reads the temperature sensor once */
  Single_Measure_REF(INCH_11, REF2_5V);	/* Reads VCC once (VCC/2 internally) */
#endif
  while(1)
  {
    if (ADCDone)
    {
      ADCDone = false;
      Single_Measure(INCH_0);
#if 0
      __bis_SR_register(GIE);   /* check if working */
      LPM0;   /* check if working */
#endif
    }
  }
}
