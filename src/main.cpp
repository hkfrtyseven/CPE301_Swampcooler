#include <Arduino.h>
#include <DHTxx.h>
#include <LiquidCrystal.h>
#include <TimeLib.h>

#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void setup();
void loop();
void adc_init();
unsigned int adc_read(unsigned char adc_channel_num);
void DelayTimer(long int DelayValue);
void DHT11();
void convertToF();
time_t requestSync();
void processSyncMessage();
void printDigits(int digits);
void logTime(bool b);
void displayClimate();
void displayError();

dht DHT;
LiquidCrystal lcd(6,5,4,3,8,2);
#define DHT11_PIN 7        // Pin that DHT sensor is connected to
#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message 

volatile unsigned char * my_ADMUX = (unsigned char *) 0x7C;   // ADC Registers
volatile unsigned char * my_ADCSRB = (unsigned char *) 0x7B;
volatile unsigned char * my_ADCSRA = (unsigned char *) 0x7A;
volatile unsigned int * my_ADC_DATA = (unsigned int *) 0x78;

volatile unsigned char* port_b = (unsigned char*) 0x25; // Port Registers
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* ddr_h = (unsigned char*) 0x101;
volatile unsigned char* port_h = (unsigned char*) 0x102;
volatile unsigned char* pin_k  = (unsigned char*) 0x106;
volatile unsigned char* ddr_k  = (unsigned char*) 0x107; 
volatile unsigned char* port_k = (unsigned char*) 0x108;
volatile unsigned char* port_l = (unsigned char*) 0x10B;
volatile unsigned char* pin_l = (unsigned char*) 0x109;
volatile unsigned char* ddr_l = (unsigned char*) 0x10A;

int watersensor_id = 0;
volatile unsigned int historyValue;
const int waterThreshold = 200;
const int tempThreshold = 63;
// char printBuffer[128];
int Temp = 0;
int Humidity = 0;
// int secondOfError = -1;
// int secondOfRecovery = -1;

// Status booleans
bool standby = false;
bool waterok = false;
bool tempabovelevel = false;
bool isFan = false;
bool newError = false;
bool newRecovery = false;

ISR(TIMER3_COMPA_vect)
{
  int currentValue = adc_read(watersensor_id); // Check water level
  historyValue = currentValue;

  if((currentValue < waterThreshold) && (waterok == true)) { newError = true; }
  if(!(currentValue < waterThreshold) && (waterok == false)) { newRecovery = true; Serial.println("New Recovery!"); }
  if(currentValue < waterThreshold) { waterok = false;}
  else { waterok = true; }
  
  if(Temp > tempThreshold) { tempabovelevel = true; }
  else { tempabovelevel = false; }
}

void setup() {
  // put your setup code here, to run once:
  *ddr_b |= 0b01111000;
  *ddr_h |= 0b01000000;
  *ddr_k &= 0b01111111;
  *port_k |= 0b10000000;
  // set fan pins to output
  *ddr_l |= 0b00101000;
  // set fans to LOW
  *port_l &= 0b11010111;

  Serial.begin(9600);
  setSyncProvider(requestSync);  //set function to call when time sync required
  lcd.begin(16, 2);
  lcd.print("Reading");
  lcd.setCursor(0,2);
  lcd.print("Climate");

  // OCR0A = 255;
  // TCCR0A = (1 << WGM01); // Compare (CTC) mode, internal clk, no prescaler
  // TCCR0B = 0x01;
  // TIMSK0 = (1 << OCIE0A); // enable timer0, compare match int
  // sei();                  // enable interrupts
  
  OCR3A = 255;
  TCCR3A = (1 << WGM01);  // Compare (CTC) mode, internal clk, no prescaler
  TCCR3B = 0x03;
  TIMSK3 = (1 << OCIE0A); // enable timer0, compare match int
  sei();                  // enable interrupts
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()) { processSyncMessage(); }
  if(!(*pin_k & 0b10000000))
  {
    for (volatile unsigned int i = 0; i < 50; i++); // Wait a moment and re-check to ensure input was detected and not noise
      if(!(*pin_k & 0b10000000))
      {
        standby = !standby;
        if(standby)
          {
            cli();
          }
        else
          {
            *port_b &= 0b11101111;
            sei();
          }
        while(!(*pin_k & 0b10000000));
      }
  }

  if(standby)
  {
    *port_b |= 0b00010000; // Turn on yellow LED
    *port_b &= 0b10011111; // Turn off others
    *port_h &= 0b10111111;
    if(isFan) { logTime(!isFan);
                *port_l &= 0b11110111; }  // Turn off fan motor (PL3 to LOW)
    isFan = false;
  //  Turn off motor (PL3 to LOW)
  //  *port_l &= 0b11110111;
  //  'Close' vent
  }
  else
  {
    if(!waterok)
    {
    *port_b |= 0b00100000; // Turn on red LED
    *port_b &= 0b10111111; // Turn off green LED
    *port_h &= 0b10111111; // Turn off blue LED
    if(isFan) { logTime(!isFan);
                *port_l &= 0b11110111; } // If fan was on, send a log saying it was disabled
                                         // Turn off fan motor (PL3 to LOW)
    isFan = false;
    displayError();
    // 'Close' vent    
    }
    else if(waterok)
    {
      *port_b &= 0b11011111; // Turn off red LED
      if(newRecovery)
      {
        if(DHT.temperature > 0)
        {
          Temp = (DHT.temperature * 9/5) + 32;
          Humidity = DHT.humidity;
        }
        
          lcd.clear();
          lcd.print("Temp: ");
          lcd.print(Temp);
          lcd.print(" F");
          lcd.setCursor(0,2);
          lcd.print("Humidity: ");
          lcd.print(Humidity);
          lcd.print("%");
          newRecovery = false;
      }

      displayClimate();
      if(!tempabovelevel){ *port_b |= 0b01000000; } // Turn on green LED

      if(tempabovelevel)
      {
        *port_b &= 0b10111111; // Turn off green LED
        *port_h |= 0b01000000; // Turn on blue LED
        if(!isFan) { logTime(true); 
                      *port_l |= 0b00001000; } // If the fan was not on, send a log saying it was enabled
                                               // Turn on fan (PL3 to HIGH)
        isFan = true;
        //put vent in 'active' position
      }
      else
      {
        *port_h &= 0b10111111; // Turn off blue LED
        if(isFan) { logTime(!isFan); 
                    *port_l &= 0b11110111; } // If the fan was on, send a log saying it was disabled
                                             // Turn the fan off (PL3 to LOW)
        isFan =  false;
        // Put the vent in 'disabled' or 'inactive' state
      }
    }
  }
}

void displayClimate()
{
  DHT.read11(DHT11_PIN);
  if(newRecovery)
  {
      if(DHT.temperature > 0)
      {
        Temp = (DHT.temperature * 9/5) + 32;
        Humidity = DHT.humidity;
      }

    lcd.clear();
    lcd.print("Temp: ");
    lcd.print(Temp);
    lcd.print(" F");
    lcd.setCursor(0,2);
    lcd.print("Humidity: ");
    lcd.print(Humidity);
    lcd.print("%");
    newRecovery = false;
  } else if((DHT.temperature > 0) && waterok)
  {
    Temp = (DHT.temperature * 9/5) + 32;
    Humidity = DHT.humidity;
    lcd.clear();
    lcd.print("Temp: ");
    lcd.print(Temp);
    lcd.print(" F");
    lcd.setCursor(0,2);
    lcd.print("Humidity: ");
    lcd.print(Humidity);
    lcd.print("%");
  } else if((Temp == 0) && waterok)
  {
    lcd.clear();
    lcd.print("Reading");
    lcd.setCursor(0,2);
    lcd.print("Climate");
  }
}

void displayError()
{
  if(newError)
  {
    lcd.clear();
    lcd.print("Error!");
    lcd.setCursor(0,2);
    lcd.print("Reservior Empty!");
    newError = false;
  }
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

void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

time_t requestSync()
{
  Serial.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}

void processSyncMessage() 
{
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
       setTime(pctime); // Sync Arduino clock to the time received on the serial port
     }
  }
}

void logTime(bool b)
{
  if(b) { Serial.print("Fan Enabled\t- "); }
  else { Serial.print("Fan Disabled\t- "); }

  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(month());
  Serial.print("/");
  Serial.print(day());
  Serial.print("/");
  Serial.print(year()); 
  Serial.println(); 
}