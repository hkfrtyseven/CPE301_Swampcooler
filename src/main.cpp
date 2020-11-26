#include <Arduino.h>

#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void setup();
void loop();
void adc_init();
unsigned int adc_read(unsigned char adc_channel_num);


volatile unsigned char * my_ADMUX = (unsigned char *) 0x7C;   // ADC Registers
volatile unsigned char * my_ADCSRB = (unsigned char *) 0x7B;
volatile unsigned char * my_ADCSRA = (unsigned char *) 0x7A;
volatile unsigned int * my_ADC_DATA = (unsigned int *) 0x78;

volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 

int adc_id = 0;
volatile unsigned int historyValue;
const int threshold = 200;
char printBuffer[128];

ISR(TIMER0_COMPA_vect)
{
  // int currentValue = analogRead(adc_id);
  int currentValue = adc_read(adc_id);
  // int historyValue = adc_read(adc_id);
  historyValue = currentValue;
  if(currentValue < threshold)
  {
    *port_b |= 0b00100000;
    // PORTB |= 0x20; // 0b00100000 Turn on red LED
    *port_b &= 0b10111111;
    // PORTB &= 0xBF; // 0b10111111 Turn off green LED
  }
  else
  {
    *port_b &= 0b11011111;
    // PORTB &= 0xDF; // 0b11011111 Turn off red LED
    *port_b |= 0b01000000;
    // PORTB |= 0x40; // 0b01000000 Turn on green LED
  }
}

void setup() {
  // put your setup code here, to run once:
  *ddr_b |= 0b01100000;
  // adc_init(); // Initialize andaloc-to-digital conversion
  Serial.begin(9600);
  
  OCR0A = 255;
  TCCR0A = (1 << WGM01); // Compare (CTC) mode, internal clk, no prescaler
  TCCR0B = 0x01;
  TIMSK0 = (1 << OCIE0A); // enable timer0, compare match int
  sei();                  // enable interrupts
}

void loop() {
  // put your main code here, to run repeatedly:

}

unsigned int adc_read(unsigned char adc_channel_num)
{
  uint8_t low, high;

	if (adc_channel_num >= 54) adc_channel_num -= 54; // allow for channel or pin numbers

	// MUX5 bit determines whether data is read from pins 0-7 or 8-15
	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((adc_channel_num >> 3) & 0x01) << MUX5);

	ADMUX = (1 << 6) | (adc_channel_num & 0x07);

#if defined(ADCSRA) && defined(ADCL)
	// begin the conversion
	sbi(ADCSRA, ADSC);

	// ADSC is cleared when the conversion finishes
	while (bit_is_set(ADCSRA, ADSC));
  
	low  = ADCL;
	high = ADCH;
#else
	// no data, return 0
	low  = 0;
	high = 0;
#endif

	return (high << 8) | low;
}