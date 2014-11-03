#include <RFBeeSendRev.h>
#include <EEPROM.h>

#define START1      0x53
#define START2      0x01
#define END1        0x2f
#define END2        0x45
#define DTALEN      7
#define ADC_CH1     1
#define ADC_CH4     4
#define ADC_CH5     5
#define CMD_PERIOD  300//300ms

uint16_t r_val,x_val,y_val;
unsigned char dtaSend[10] = {START1, START2, 0, 0, 0, 0, 0, 0, END1, END2};

void setup()
{
  adc_init();
  RFBEE.init();
  powerInit();
}

void loop()
{
  r_val = ReadADC(ADC_CH1);  //get rotation value
  x_val = ReadADC(ADC_CH4);  //get x-axis value
  y_val = ReadADC(ADC_CH5);  //get y-axis value
  dtaSend[2] = highByte(r_val);
  dtaSend[3] = lowByte(r_val);
  dtaSend[4] = highByte(x_val);
  dtaSend[5] = lowByte(x_val);
  dtaSend[6] = highByte(y_val);
  dtaSend[7] = lowByte(y_val);
  RFBEE.sendDta(10, dtaSend);
  delay(CMD_PERIOD);
}
 
void adc_init()
{
  r_val = 0;
  x_val = 0;
  y_val = 0;
  ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));    //16Mhz/128 = 125Khz the ADC reference clock
  ADMUX |= (1<<REFS0);                //Voltage reference from Avcc (5v)
  ADCSRA |= (1<<ADEN);                //Turn on ADC
  ADCSRA |= (1<<ADSC);                //Do an initial conversi
}

uint16_t ReadADC(uint8_t channel)
{
   ADMUX &= 0xF0;                   //Clear the older channel that was read
   ADMUX |= channel;                //Defines the new ADC channel to be read
   ADCSRA |= (1<<ADSC);             //Starts a new conversion
   while(ADCSRA & (1<<ADSC));       //Wait until the conversion is done
   return ADCW;                     //Returns the ADC value of t
}

void powerInit()
{
  pinMode(16, OUTPUT);//power ctrl 1
  pinMode(17, OUTPUT);//power ctrl 2
  digitalWrite(16, LOW);//set to low
  digitalWrite(17, LOW);//set to low
  r_val = x_val = y_val = 0;
  while((abs(r_val-512) > 30) || (abs(x_val-512) > 30) || (abs(y_val-512) > 30)){
    r_val = ReadADC(1);
    x_val = ReadADC(4);
    y_val = ReadADC(5);   
    delay(100); 
  }  
}
