#include <Enrf24.h>
#include <nRF24L01.h>
#include <string.h>
#include <SPI.h>

// node 10, brightness 100, movement detected
// N010;B100;M001

// node 10, brightness 5, no movement detected
// N010;B005;M000

#define NODE "010"
#define SPEPARATOR ";"

// msp430g2452, pir sensor, nrf24l01
Enrf24 radio(P2_0, P2_1, P2_2);  // P2.0=CE, P2.1=CSN, P2.2=IRQ
const uint8_t txaddr[] = { 
  0xDE, 0xAD, 0xBE, 0xEF, 0x01 };

#define PIRPIN P1_4

#define ANODE P2_3
#define CATHODE P2_4
//usage led
//led on: ANODE +, CATHODE -
//led off: ANODE -, CATHODE don't care

//usage brightness:
//load led: ANODE -, CATHODE +

#define LIGHT_DARK_BOUNDARY 100

//http://arduino.cc/en/Tutorial/Smoothing
const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average


void dump_radio_status_to_serialport(uint8_t);

void setup() {
  Serial.begin(9600);

  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(1); // MSB-first

  radio.begin();  // Defaults 1Mbps, channel 0, max TX power
  dump_radio_status_to_serialport(radio.radioState());

  radio.setTXaddress((void*)txaddr);

  pinMode(PIRPIN, INPUT_PULLUP);

  pinMode(ANODE, OUTPUT);
  pinMode(CATHODE, OUTPUT);
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;
}

void loop() {
  static int lastValue=0;
  static int lastBrightness=0;
  static int brightness=0;
  unsigned int brightnessChanged=0;
  //Serial.print("brightness = ");  
  //Serial.println(measure_brightness());  
  //brightness = measure_brightness();

  if (lastValue == 0)//only measure brightness if light turned off
    brightness = smooth_brightness();
  if (brightness < (lastBrightness - 10) || 
      brightness > (lastBrightness+10))
  {
    brightnessChanged = 1;
  }

  int sensorValue = digitalRead(PIRPIN);
  //if (sensorValue != lastValue)
  // if (sensorValue != lastValue || brightness != lastBrightness)
  if (sensorValue != lastValue || brightnessChanged)
  {
    /*
      String payload;
     payload = "N";
     payload += NODE;
     payload += SPEPARATOR;
     payload += "B";
     if (brightness <10)
     payload += "00";
     else if (brightness <100)
     payload += "0";
     payload += brightness;
     payload += SPEPARATOR;
     payload += "M";
     payload += "00";
     payload += sensorValue;
     */
    char payload[20];
    char buffer[4];
    // todo: zeroize not needed if strcpy is used for first char?
    // strcpy(payload,"N");
    for (int index = 0; index < 20; index++)
      payload[index] = 0;
    payload[0] = 'N';
    strcat(payload,NODE);
    strcat(payload,SPEPARATOR);
    strcat(payload,"B");
    if (brightness <10)
      strcat(payload, "00");
    else if (brightness <100)
      strcat(payload, "0");
    itoa(brightness,buffer,10);
    strcat(payload, buffer);
    strcat(payload,SPEPARATOR);
    strcat(payload,"M");
    strcat(payload,"00");
    itoa(sensorValue,buffer,10);
    strcat(payload, buffer);
    payload[15] = 0;
    Serial.print("Sending packet: ");
    Serial.println(payload);
    Serial.print("sensor value: ");
    Serial.println(sensorValue);
    Serial.print("brightness: ");
    Serial.println(brightness);

    //turn off independant of brightness
    //turn on only if not too bright
    //    if (!sensorValue || brightness > LIGHT_DARK_BOUNDARY)
    //    {
    radio.print(payload);
    radio.flush();  // Force transmit (don't wait for any more data)
    //dump_radio_status_to_serialport(radio.radioState());  // Should report IDLE
    //    }
    delay(1000); 
  }
  lastValue = sensorValue;
  lastBrightness = brightness;
}

void dump_radio_status_to_serialport(uint8_t status)
{
  Serial.print("Enrf24 radio transceiver status: ");
  switch (status) {
  case ENRF24_STATE_NOTPRESENT:
    Serial.println("NO TRANSCEIVER PRESENT");
    break;

  case ENRF24_STATE_DEEPSLEEP:
    Serial.println("DEEP SLEEP <1uA power consumption");
    break;

  case ENRF24_STATE_IDLE:
    Serial.println("IDLE module powered up w/ oscillators running");
    break;

  case ENRF24_STATE_PTX:
    Serial.println("Actively Transmitting");
    break;

  case ENRF24_STATE_PRX:
    Serial.println("Receive Mode");
    break;

  default:
    Serial.println("UNKNOWN STATUS CODE");
  }
}

uint8_t measure_brightness()
{
  uint8_t brightness = 0;
  //load led
  pinMode(CATHODE, OUTPUT);
  digitalWrite(CATHODE, HIGH);
  pinMode(ANODE, OUTPUT);
  digitalWrite(ANODE, LOW);
  delayMicroseconds(10);

  //measure time till cathode low
  pinMode(CATHODE, INPUT);
  while ( digitalRead(CATHODE) && brightness != 0xff)
  {
    brightness++;
    //delayMicroseconds(100);
    delay(1);
  }
  return brightness;  
}

uint8_t smooth_brightness()
{
  // subtract the last reading:
  total= total - readings[index];        
  // read from the sensor:  
  readings[index] = measure_brightness();
  // add the reading to the total:
  total= total + readings[index];      
  // advance to the next position in the array:  
  index = index + 1;                    

  // if we're at the end of the array...
  if (index >= numReadings)              
    // ...wrap around to the beginning:
    index = 0;                          

  // calculate the average:
  average = total / numReadings;        
  // send it to the computer as ASCII digits
  return (uint8_t) average;
}


