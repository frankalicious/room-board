#include <msp430.h>
#include <stdlib.h>
#include <string.h>
#include "uart/uart.h"
#include "adc/adc.h"
#include "stdbool.h"
#include "DHT11_LIB/DHT11_LIB.h"
#include "msprf24/msprf24.h"
#include "msprf24/nrf_userconfig.h"

#define     LED BIT3
#define     LED_DIR P2DIR
#define     LED_OUT P2OUT
#define     LED_OFF (LED_OUT &= ~LED)
#define     LED_ON (LED_OUT |= LED)
#define     LED_TOGGLE (LED_OUT ^= LED)

/* #define NODE "010" /\* perf board version *\/ */
/* #define NODE "011" /\* launchpad version *\/ */
#define NODE "012" /* osh park version */

#define SEPARATOR ";"

unsigned char volatile TOUT;			/* REQUIRED for library */
unsigned char volatile SECOND_TIMER=0;

unsigned char RH_byte1;
unsigned char RH_byte2;
unsigned char T_byte1;
unsigned char T_byte2;
unsigned char checksum;

unsigned char Packet[5];

volatile unsigned int user;

void init_led(void);
void uart_rx_isr(unsigned char c);
void delay_ms(unsigned int ms);
void get_dht(void);
void init_dht (void);
void print_dht (void);
void init_nrf24 (void);
void check_nrf24 (void);
void send_nrf24 (void);
void create_string(char* payload, int brightness, int sensorValue, uint8_t temperature);
void print_string(char* payload, int brightness, int sensorValue, uint8_t temperature);

__interrupt void CCR0_ISR(void);

#pragma vector = TIMER0_A0_VECTOR
__interrupt void CCR0_ISR(void){
  SECOND_TIMER++;
  TOUT=1;
  /* TOG (P1OUT,0x01); */
  CLR (TACCTL0, CCIFG);
}
void create_string(char* payload, int brightness, int sensorValue, uint8_t temperature)
{
  char buffer[4];
  int index;
  /* todo: zeroize not needed if strcpy is used for first char? */
  /* strcpy(payload,"N"); */
  for ( index= 0; index < 21; index++)
    payload[index] = 0;
  payload[0] = 'N';
  strcat(payload,NODE);
  strcat(payload,SEPARATOR);
  strcat(payload,"B");
  if (brightness <10)
    strcat(payload, "00");
  else if (brightness <100)
    strcat(payload, "0");
  itoa(brightness,buffer,10);
  strcat(payload, buffer);
  strcat(payload,SEPARATOR);
  strcat(payload,"M");
  strcat(payload,"00");
  itoa(sensorValue,buffer,10);
  strcat(payload, buffer);
  strcat(payload,SEPARATOR);
  strcat(payload,"T");
  if (temperature <10)
    strcat(payload, "00");
  else if (temperature <100)
    strcat(payload, "0");
  itoa(temperature,buffer,10);
  strcat(payload, buffer);
  payload[20] = 0;
}

void print_string(char* payload, int brightness, int sensorValue, uint8_t temperature)
{
  char buffer[10];
  char* act_char = &payload[0];
  uart_puts("Sending packet: ");
  while (*act_char)
  {
    itoa(*act_char,buffer,10);
    uart_puts(" ");
    act_char++;
  }
  uart_puts(buffer);
  uart_puts("\r\n");
  uart_puts("sensor value: ");
  itoa(sensorValue,buffer,10);
  uart_puts(buffer);
  uart_puts("\r\n");
  uart_puts("brightness: ");
  itoa(brightness,buffer,10);
  uart_puts(buffer);
  uart_puts("\r\n");
  uart_puts("temperature: ");
  itoa(temperature,buffer,10);
  uart_puts(buffer);
  uart_puts("\r\n");
}

void init_nrf24(void)
{
  uint8_t addr[5];

  /* Initial values for nRF24L01+ library config variables */
  /* rf_crc = RF24_EN_CRC | RF24_CRCO; /\* CRC enabled, 16-bit *\/ */
  rf_crc = RF24_EN_CRC; /* CRC enabled, 8-bit */
  rf_addr_width      = 5;
  rf_speed_power     = RF24_SPEED_1MBPS | RF24_POWER_MAX;
  /* rf_channel         = 119; */
  rf_channel         = 0;
  /* rf_channel         = 23; */
  msprf24_init();
  /* msprf24_set_pipe_packetsize(0, 32); */
  msprf24_set_pipe_packetsize(0, 0); /* dynamic */
  msprf24_open_pipe(0, 1);  /* Open pipe#0 with Enhanced ShockBurst */
  /* msprf24_open_pipe(0, 0); */

  /* Set our RX address */
  addr[0] = 0xDE;
  addr[1] = 0xAD;
  addr[2] = 0xBE;
  addr[3] = 0xEF;
  /* addr[4] = 0x00; */
  addr[4] = 0x01;
  /* addr[4] = 0x23; */
  w_tx_addr(addr);
  w_rx_addr(0, addr);  /* Pipe 0 receives auto-ack's, autoacks are sent back to the TX addr so the PTX node */
  /* needs to listen to the TX addr on pipe#0 to receive them. */
  msprf24_standby();
  user = msprf24_current_state();
#if 0
  /* Receive mode */
  if (!(RF24_QUEUE_RXEMPTY & msprf24_queue_state())) {
    flush_rx();
  }
  msprf24_activate_rx();
#endif
}

void check_nrf24(void)
{
  uint8_t buf[32];

  if (rf_irq & RF24_IRQ_FLAGGED) {
    msprf24_get_irq_reason();
  }
  if (rf_irq & RF24_IRQ_RX || msprf24_rx_pending()) {
    r_rx_payload(32, buf);
    msprf24_irq_clear(RF24_IRQ_RX);
    user = buf[0];
    uart_puts("nrf24 received!\r\n");
  } else {
    user = 0xFF;
  }
}

void send_nrf24 (void)
{
  char buf[32];
  char buffer[10];
  uint8_t sensor = 1;
  user = msprf24_current_state();
  itoa(user,buffer,10);
  uart_puts("current state: ");
  uart_puts(buffer);
  uart_puts("\r\n");

  create_string(&buf[0], ADCValue, sensor, T_byte1);
  print_string(&buf[0], ADCValue, sensor, T_byte1);
  w_tx_payload(32, (uint8_t*)buf);
  msprf24_activate_tx();
  /* msprf24_irq_clear(rf_irq); */
  user = msprf24_get_last_retransmits();
  itoa(user,buffer,10);
  uart_puts("last retransmit: ");
  uart_puts(buffer);
  uart_puts("\r\n");
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
  init_nrf24();
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
    check_nrf24();
    send_nrf24();
  }
}
