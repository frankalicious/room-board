#include <msp430.h>
#include <stdlib.h>
#include "uart/uart.h"
#include "adc/adc.h"
#include "stdbool.h"
#include "DHT11_LIB/DHT11_LIB.h"

#define     LED BIT3
#define     LED_DIR P2DIR
#define     LED_OUT P2OUT
#define     LED_OFF (LED_OUT &= ~LED)
#define     LED_ON (LED_OUT |= LED)
#define     LED_TOGGLE (LED_OUT ^= LED)

unsigned char volatile TOUT;			/* REQUIRED for library */
unsigned char volatile SECOND_TIMER=0;

unsigned char RH_byte1;
unsigned char RH_byte2;
unsigned char T_byte1;
unsigned char T_byte2;
unsigned char checksum;

unsigned char Packet[5];

void init_led(void);
void uart_rx_isr(unsigned char c);
void delay_ms(unsigned int ms);
void get_dht(void);
void init_dht (void);
void print_dht (void);

__interrupt void CCR0_ISR(void);

/* TODO: fix makefile. In the actual state only c files in the main folder can be compiled */

#pragma vector = TIMER0_A0_VECTOR
__interrupt void CCR0_ISR(void){
  SECOND_TIMER++;
  TOUT=1;
  /* TOG (P1OUT,0x01); */
  CLR (TACCTL0, CCIFG);
}

void get_dht(void)
{
  /* Must wait 1 second initially and between all reads */
  if(SECOND_TIMER >= 5){		/* 5 @ CCR0 = 50000 & div 4 */
    /* Manual way to gather all data without array */
    /*
      start_Signal();
      if(check_Response()){
      RH_byte1 = read_Byte();
      RH_byte2 = read_Byte();
      T_byte1 = read_Byte();
      T_byte2 = read_Byte();
      checksum = read_Byte();
      }
    */

    /* Simple way to gather all data with one instruction */
    read_Packet(Packet);
    RH_byte1 =	Packet[0];
    RH_byte2 =	Packet[1];
    T_byte1 =	Packet[2];
    T_byte2 =	Packet[3];
    checksum =	Packet[4];

    /* if (check_Checksum(Packet)) */
    /* SET (P1OUT, 0x40); */

    SET (TACTL, TACLR);
    SET (TA0CTL, 0x10);
    TACCR0 = 50000;		/* Initialize the timer to count at 5Hz */
    SECOND_TIMER = 0;	/* Clear counter */
  }
}

void init_dht (void)
{
  TACCR0 = 50000;				/* Initialize the timer to count at 5Hz */
  TACCTL0 = CCIE;				/* Enable interrupt */
  TA0CTL = TASSEL_2 + ID_2 + MC_1 + TACLR;	/* SMCLK, div 4, up mode, */
}

void print_dht (void)
{
  char buffer[10];
/*
  itoa(RH_byte1,buffer,10);
  uart_puts(buffer);
  uart_puts(" ");
  itoa(RH_byte2,buffer,10);
  uart_puts(buffer);
  uart_puts(" ");
  itoa(T_byte1,buffer,10);
  uart_puts(buffer);
  uart_puts(" ");
  itoa(T_byte2,buffer,10);
  uart_puts(buffer);
  uart_puts(" ");
  itoa(checksum,buffer,10);
  uart_puts(buffer);
  uart_puts(" ");
  uart_puts("\r\n");
*/
  uart_puts("Humidity: ");
  itoa(RH_byte1,buffer,10);
  uart_puts(buffer);
  uart_puts("\tTemperature: ");
  itoa(T_byte1,buffer,10);
  uart_puts(buffer);
  uart_puts("\tChecksum: ");
  itoa(checksum,buffer,10);
  uart_puts(buffer);
  uart_puts("\r\n");
}

void delay_ms(unsigned int ms){
  while(ms--){
    __delay_cycles(1000);
  }
}

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
  init_dht();
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
    char buffer[10];
    /* unsigned volt; */
    get_dht();
    print_dht();

    delay_ms(1000);
    /* volt = read_voltage(); */
    /* itoa(volt,buffer,10); */
    /* uart_puts("Brightness: "); */
    /* uart_puts(buffer); */
    /* uart_puts("\r\n"); */
    if (ADCDone)
    {
      LED_TOGGLE;
      itoa(ADCValue,buffer,10);
      uart_puts("Brightness: ");
      uart_puts(buffer);
      uart_puts("\r\n");
      ADCDone = false;
      Single_Measure(INCH_0);
#if 0
      __bis_SR_register(GIE);   /* check if working */
      LPM0;   /* check if working */
#endif
    }
  }
}
