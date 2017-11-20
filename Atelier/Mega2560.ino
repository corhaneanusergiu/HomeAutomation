// So this is a relatively simple,cheap peripheral using a ÂŁ1.50 NANO board from China. 
// It is supported by my ESP8266 software but as it is an I2c slave you could run it
// from anything able to handle I2c. For example I have found that some of the NanoPi units 
// are not too keen on even a bright LED on their IO pins and from an operating
// system like Linux, getting PWM on several pins is just not on... so - plug in this device 
// (default ID 9) with pullups (always needed for I2c) and you can gain PWM, ADC 
// and general IO for very little money. 
//
//
// So as a guide you could use 3,5,6 and 9, 10 and 11 for PWM (unless you use these pins for general IO)
// you can use 2, 4, 7, 8, 12 and 13 as input/output (I tried using 0 and 1 - no go 0 flashes 
// on power up - 1 has pullup - best just avoid for GPIO 0 - use for serial IO).
// remember 13 probably has a LED attached on the board so best used for output.
// You could use A0 (14), A1 (15), A2 (16) and A3 (17) as analog in - possibly 
// A6 (20) and A7 (21) if available. Set to 1.1v full scale.
// A4 and A5 are used for the I2c where A4 is SDA and A5 is SCL.
// On the blog at http://tech.scargill.net you'll see several examples of using I2c from various boards.
//
// Late addition - servos - any of the pins 2-13 can be a servo. command is 11 - so device, command, pin, value
// Send value 255 to disconnect a servo and note if ANY pin is set up as a servo you lose PWM options on pins 9 and 10.
// Just disconnect all individually to get the PWM back (normally all disconnected at power up). 
// Values 0-180 but this varies with different servos. Mine buzzed at 0 !! See Arduino Servo library
//
// A simple i2c SLAVE - default device number 9 - reads instructions from
// master and either sets outputs or returns inputs accordingly.
//
// Not yet used but check out the enable interrupt library here
// https://github.com/GreyGnome/EnableInterrupt
// could be used for edge-triggered interrupts on any pin for pulse counting
//
// There is also now a soft fade option for PWM, a tone generator and Dallas temperatur chip support for up to 2 chips.
// Here I use a simplified version of my DS18B20 code from years back. This starts the conversion at the END
// of the code - so the first value is rubbish - read the blog as this is hidden - and there are no delays. On the assumption of one chip
// per pin, no need for search either!
//
//
// PROMINI - NANO = PINS
//-----------------------------------------------------------------------------------------------------------------------------------------
// ANALOG   -     A0        - A7 (8 PINS)
// PWM      -     D3        - D5,   D6,   D9 - D11 (6 PINS)
// DIGITAL  -     D0  (RX)  - D1 (TX),    D2 - D13 (14 PINS)
// I2C      -     D4  (SDA) - D5  (SCL)
// SPI      -     D10 (SS)  - D13 (SCK)  - D11 (MOSI)  - D12 (MISO)
// SERIAL   -     D1  (TX)  - D0  (RX)
// LED      -     D13
// AREF     -     1.1V
//-----------------------------------------------------------------------------------------------------------------------------------------
//
//
// MEGA 2560 = PINS
//-----------------------------------------------------------------------------------------------------------------------------------------
// ANALOG   -     A0        - A15 (16 PINS)
// PWM      -     D2        - D13,   D20,   D38 - D40 (15 PINS)
// DIGITAL  -     D14       - D53 (40 PINS)
// I2C      -     D20 (SDA) - D21 (SCL)
// SPI      -     D19 (SS)  - D20 (SCK)  - D21 (MOSI)  - D22 (MISO)
// SERIAL   - 0 - D1  (TX)  - D0  (RX)
//            1 - D18 (TX)  - D19 (RX)
//            2 - D16 (TX)  - D17 (RX)
//            3 - D14 (TX)  - D15 (RX)
//
//
//-----------------------------------------------------------------------------------------------------------------------------------------
//
//
// COMPONENTE
//-----------------------------------------------------------------------------------------------------------------------------------------
// DENUMIREA                  -  NR IO  -   TENSIUNEA   -  PROTOCOL  -                  IO                   -
//------------------------------------------------------------------------------------------------------------
// SURSA                      -         -  240VCA/12VCC -            -                                       -
// BATERIE                    -         -     12V/7A    -            -                                       -
// ARDUINO MEGA 2560          -         -       5V      -    I2C     -                                       -
// ESP8266 2.4GHz             -         -      3.3 V    -    I2C     -                                       -
// ARDUINO PRO MINI (TENSIUNI)-         -       5V      -    I2C     -                                       -
// RFM69HW 868MHz             -         -      3.3V     -    SPI     -                                       -
// REED - NC (2 Doors 3 WIN)  -         -      GND      -            -                                       -
// PIR - 12V (3 BUC)          -         -       12v     -            -                                       -
// SIRENA                     -         -    12V/1.3A   -            -                                       -
// SIRENA - RELEU             -         -       5V      -            -                                       -
// LCD 20X4                   -         -       5V      -     I2C    -                                       -
// TASTATURA                  -         -               -            -                                       -
// BUZZER                     -         -       5V      -            -                                       -
// LED-uri                    -         -       5V      -            -                                       -
// RFID                       -         -      3.3V     -     SPI    -                                       -
// ARDUINO PRO MINI (RFID)    -         -       5V      -     I2C    -                                       -
// DS18B20+                   -         -               -   1 WIRE   -                                       -
// VOLTMETRU                  -         -       12V     -            -                                       -
// VOLTMETRU                  -         -        5V     -            -                                       -
//
//


#include <EEPROM.h>
#include <Wire.h>
#include <Servo.h> /// note that if you use ANY servo, you lose PWM on pins 9 and 10.
#include <avr/pgmspace.h>
#include <OneWire.h>
#define MAXPORTS 21

#define SET_OUTPUT  1
#define READ_INPUT  2
#define READ_INPUT_PULLUP 3
#define SET_PWM     4
#define READ_ANALOG 5
#define SET_ADDRESS 6
#define PORTSET 7
#define PORTOUT 8
#define PORTIN 9
#define SEROUT 10
#define SERVO 11   /// value 255 disconnects.... - normally use 0-180
#define FADE 12
#define TONE 13
#define NOTONE 14
#define DALLAS1 15
#define DALLAS2 16

#define STRUCTBASE 0

byte busy=0;
struct STORAGE{
byte chsm;
byte device;
byte t1;
byte t2;
};

int tr1=255;
int tr2=255;

STORAGE stored;

byte ports[MAXPORTS];
byte params[128];
byte retparams[3];
byte paramp;

long mymillis;


const PROGMEM  uint8_t ledTable[256] = // Nano is so pathetically short of RAM I have to do this!
{
  0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4,
  4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 13, 13, 14, 14, 15, 15, 15, 16, 16, 17, 17, 18,
  18, 19, 19, 20, 20, 21, 22, 22, 23, 23, 24, 25, 25, 26, 26, 27, 28, 28, 29, 30, 30, 31, 32, 33, 33, 34, 35, 36, 36, 37, 38, 39, 40, 40, 41,
  42, 43, 44, 45, 46, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 67, 68, 69, 70, 71, 72, 73, 75, 76, 77,
  78, 80, 81, 82, 83, 85, 86, 87, 89, 90, 91, 93, 94, 95, 97, 98, 99, 101, 102, 104, 105, 107, 108, 110, 111, 113, 114, 116, 117, 119, 121,
  122, 124, 125, 127, 129, 130, 132, 134, 135, 137, 139, 141, 142, 144, 146, 148, 150, 151, 153, 155, 157, 159, 161, 163, 165, 166, 168, 170,
  172, 174, 176, 178, 180, 182, 184, 186, 189, 191, 193, 195, 197, 199, 201, 204, 206, 208, 210, 212, 215, 217, 219, 221, 224, 226, 228, 231,
  233, 235, 238, 240, 243, 245, 248, 250, 253, 255
};

byte fade[12][3];
Servo myservos[14]; // just for ease - so use any pin from 3 to 13... bit of waste but so what.

// Here's the Dallas code - end user need to spot negative values...see https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf
int16_t dallas (int x)
{
  OneWire ds(x);
  byte i;
  byte data[2];
  int16_t result;
      ds.reset();
      ds.write(0xCC);
      ds.write(0xBE);
      for (i=0;i<2; i++) data[i]=ds.read();
      result=(data[1]<<8)|data[0];
 
      ds.reset();
      ds.write(0xCC);
      ds.write(0x44,1);
      return result;
}


void setup(void) {
  // NO serial if using using 0-7 as port expansion (I'm not)
  // If you want serial - set the speed in the setup routine, if not, comment out
  int a;
  uint16_t time = millis();
  byte eeprom1,eeprom2;
  analogReference(INTERNAL);  // 1.1v
  Serial.begin(115200);
 // get info out of EEPROM
 EEPROM.get(STRUCTBASE,stored);
 
 // first check if EEPROM info is valid?
 if (stored.chsm!=0x3d)
   {
    stored.chsm=0x3d;
    stored.device=9;
    stored.t1=255;
    stored.t2=155;
    EEPROM.put(STRUCTBASE,stored);
   }

  for (a=0;a<MAXPORTS;a++) ports[a]=0; // all inputs
  Wire.begin(stored.device);           // join i2c bus with address #9 by default
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent); 
  
  paramp=0;
  Serial.begin(115200);
  mymillis=0;
  for (a=0;a<12;a++){ fade[a][0]=0; fade[a][2]=0; }
  for (a=0;a<128;a++) params[a]=0;

  delay(100);
  
  if (stored.t1!=255) tr1=dallas(stored.t1);
  if (stored.t2!=255) tr2=dallas(stored.t2);  

  tr1=85*16;
  tr2=85*16;

}

void loop() {

if (mymillis<millis())
    {
      mymillis=millis()+10;
      for (int a=0; a<12; a++)
        {
          if (fade[a][0])
            {
              if (fade[a][1]<fade[a][2]) { if (++fade[a][1]==fade[a][2]) fade[a][0]=0; analogWrite(a,pgm_read_word_near(ledTable+fade[a][1])); }
              if (fade[a][1]>fade[a][2]) { if (--fade[a][1]==fade[a][2]) fade[a][0]=0; analogWrite(a,pgm_read_word_near(ledTable+fade[a][1])); }
            }
        }
    }  

}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
      retparams[2]=busy;
      Wire.write(retparams,3); 
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void receiveEvent(int count) {
busy=1;
int a;
int tcount;
tcount=count;
paramp=0;
for (a=0;a<6;a++) params[a]=0; 
  // Nothing time consuming or visual debugging in here if a RETURN VALUE is expected or the routine to send a byte back could be missed.
  while ((tcount--)&&(paramp<128))
   {
    params[paramp++]=Wire.read(); 
   }
  switch (params[0])
    {
    case SET_OUTPUT:
          if (ports[params[1]]!=1) { ports[params[1]]=1; pinMode(params[1],OUTPUT); } 
          digitalWrite(params[1],params[2]? HIGH : LOW); 
          break;
    case READ_INPUT:
          if (ports[params[1]]!=2) { ports[params[1]]=2; pinMode(params[1],INPUT); } 
          retparams[0]=0; retparams[1]=digitalRead(params[1]); 
          break;
    case READ_INPUT_PULLUP:
          if (ports[params[1]]!=3) { ports[params[1]]=3; pinMode(params[1],INPUT_PULLUP); } 
          retparams[0]=0; retparams[1]=digitalRead(params[1]); 
          break;          
    case SET_PWM:
          if (ports[params[1]]!=4) { ports[params[1]]=4; pinMode(params[1],OUTPUT); } 
          analogWrite(params[1],params[2]); 
          break;
    case READ_ANALOG:
          if (ports[params[1]]!=2) { ports[params[1]]=2; pinMode(params[1],INPUT); } 
          uint16_t anback; anback=analogRead(params[1]); retparams[0]=anback>>8; retparams[1]=anback&255;
          break;    
    case SET_ADDRESS:
          stored.device=params[1]; EEPROM.put(STRUCTBASE,stored);
          // update address - will take effect on next powerup of the device as you 
          // can only call "begin" once
          break;
    case SEROUT: char *m;
                 m=(char *)&params[1];
                 Serial.print(m);
                 break;
    case SERVO : if (ports[params[1]]!=5) { ports[params[1]]=5; myservos[params[1]].attach(params[1]); }  
                 if (params[2]==255) { myservos[params[1]].detach(); ports[params[1]]=0; break; }
                 myservos[params[1]].write(params[2]);
                 break; 
    case FADE:
          if (ports[params[1]]!=4) { ports[params[1]]=4; pinMode(params[1],OUTPUT);  } 
          fade[params[1]][0]=1; fade[params[1]][2]=params[2];
          break;  

    case TONE:  // can't do PWM on pins 2 and 11 while doing this... only one pin at a time...use NOTONE when finished
          if ((params[4]|params[5])==0) tone(params[1],(params[2]<<8)+params[3]); else tone(params[1],(params[2]<<8)+params[3],(params[4]<<8)+params[5]); 
          ports[params[1]]=0;
          break;
             
    case NOTONE:  // can't do PWM on pins 3 and 11 while doing TONE...
          noTone(params[1]); ports[params[1]]=0; 
          break;

    case DALLAS1:
          tr1=dallas(params[1]); 
          if (params[1]!=stored.t1) { stored.t1=params[1];  EEPROM.put(STRUCTBASE,stored); } // no delay hence first value crap
          retparams[1]=tr1&255; retparams[0]=tr1>>8; 
          break;

    case DALLAS2:
          tr2=dallas(params[1]); 
          if (params[1]!=stored.t2) { stored.t2=params[1]; EEPROM.put(STRUCTBASE,stored); }   // no delay hence first value crap
          retparams[1]=tr2&255; retparams[0]=tr2>>8;
          break;  
                                 
   default: break;  
    }
    busy=0;
}