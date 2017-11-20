// Corhaneanu Sergiu
//
// v1.0 - 20.11.2017
//
//------ INFO -----
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
//
// MEGA 2560 = PINS
//-----------------------------------------------------------------------------------------------------------------------------------------
// ANALOG   -     A0        - A15 (16 PINS) (D54 - D69)
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

//###### ALARM FUNCTION ######
//============================

#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <Time.h>
#include <Timezone.h>
#include <WSWire.h>
//#include <PN532_I2C.h>
//#include <PN532.h>
#include "pitches.h"
#include "types.h"
#include "mpr121.h"
#include <dht.h>
#include <SoftwareSerial.h>
#include "SIM900.h"
#include "sms.h"
#include "call.h"
#include <avr/wdt.h>


//------ PINS ARDUINO MEGA 2560 ------

//#define 0 // RX0
//#define 1 // TX0
//#define 2 // PWM - INT4
//#define 3 // PWM - INT5
//#define 4 // PWM
//#define 5 // PWM
#define BUZZER 6 // PWM
#define LED_RGB_R 7 // PWM - RGB - (blue - PROGRAMING; green - DISARMED; red - ARMED; red blink - ALARMED)
#define LED_RGB_G 8 // PWM - RGB - (blue - PROGRAMING; green - DISARMED; red - ARMED; red blink - ALARMED)
#define LED_RGB_B 9 // PWM - RGB - (blue - PROGRAMING; green - DISARMED; red - ARMED; red blink - ALARMED)
#define LED_5V 10 // PWM - red
#define LED_12V 11 // PWM - red
#define LED_240V 12 // PWM - red
#define LED_ESP 13 // PWM - transmisiuni cu ESP8266
#define BUTON_RESET 14 // resetare la valori initiale (anulare inregistrare senzori si tag-uri)
#define BUTON_ARMARE 15 // varianta push with led
#define BUTON_ARMARE_CS 16 // varianta buton capacitiv
#define CONTACT_CUTIE_ALARMA 17 // microcontact deschidere cutie alarma si cutie baterie
#define CONTACT_CUTIE_BATERIE 18 // INT3
//#define  19 // INT2
#define SDA 20 // SDA - INT1
#define SCL 21 // SCL - INT0
#define SENZOR_01 22 // reed yala
#define SENZOR_02 23 // reed usa 1
#define SENZOR_03 24 // reed usa 2
#define SENZOR_04 25 // reed geam 1
#define SENZOR_05 26 // reed geam 2
#define SENZOR_06 27 // reed geam 3
#define SENZOR_07 28 // reed beci
#define SENZOR_08 29 // cablu taiat 
#define SENZOR_09 30 // divers
#define SENZOR_10 31 // fum
#define SENZOR_11 32 // flacara
#define SENZOR_12 33 // CO2, CO
#define SIRENA_EXT 34 // releu
#define SIRENA_INT 35 // releu
#define TASTATURA_01 36
#define TASTATURA_02 37
#define TASTATURA_03 38
#define TASTATURA_04 39
#define TASTATURA_05 40
#define TASTATURA_06 41
#define TASTATURA_07 42
#define TASTATURA_08 43
#define VENTILATOR_ALARMA 44 // PWM
#define VENTILATOR_BATERIE 45 // PWM
#define ILUMINAT_URGENTA_INT 46 // PWM - leduri in cutii 12V
#define ILUMINAT_URGENTA_EXT 47 // releu iluminat exterior 12V
#define YALA 48 // releu yala usa principala
#define RFID_RST 49 //
#define RFID_MISO 50 // MISO
#define RFID_MOSI 51 // MOSI
#define RFID_SCK 52 // SCK - PWM
#define RFID_SDA 53 // SS
#define TENSIUNE_240V 54 // prezenta tensiune de la iesire releu 240VCA (ON/OFF)
#define TENSIUNE_12V 55 // A0
#define TENSIUNE_5V 56 // A1
#define TEMP_EXT 57 // A2 - DS18B20 sonda exterior cutie
#define TEMP_INT 58 // A3 - DS18B20 senzor interior cutie alarma
#define TEMP_BAT 59 // A4 - DS18B20 senzor interior cutie baterie
#define LDR 60 // A5
//#define 61 // A6
//#define 62 // A7
//#define 63 // A8
//#define 64 // A9
//#define 65 // A10
//#define 66 // A11
//#define 67 // A12
//#define 68 // A13
//#define 69 // A14
//
//

//------ FUNCTION VARIABLES ------
// Enable or disable system modules
#define TEMP_ENABLED 1 //Enable temperature measurement
#define CS_ENABLED 1 //Enable touch buttons controller
#define LCD_ENABLED 1 //Enable LCD output
#define FIRE_ENABLED 0 //Enable Fire detection
#define NFC_ENABLED 0 //Enable NFC detection
#define GSM_ENABLED 0 //Enable GSM functions
#define SND_ENABLED 1 //Enable sound
#define WCH_ENABLED 0 //Enable watchdog
#define REBOOT_CHK 1 //Security mode. If some component fails, after 5 reboot the system enters a safe mode to avoid the siren ring at each reboot (they may be endless!)
#define REBOOT_RST 0 //Reset security mode. If the system  is stuck, recompile with this option to 1 to reset it. When fixed, recompile with 0

//------ SENSOR SETTINGS ------
#define NR_SENZORI 12
#define REED 1
#define PIR 2
#define INCENDIU 3
#define DIVERS 4

// DE MODIICAT !!!!
// Declare sensors array
#define sensor_number 12 // number of sensors
sensor sensors[] = 
{
  //fields: pin, state, pir, reed, alarmed_timestamp, name, enabled
  //set state, alarmed_timestamp to 0 and enabled to false
  //first sensor is the DOOR sensor.
	{ 49, HIGH, 0, 1, 0, "Door", true },
	{ 6, HIGH, 1, 0, 0, "Volumetric 1", true },
//  {4,0,1,1,0,("Volumetric 2"),false},
	{ 13, HIGH, 0, 1, 0, "Sensor 1", true },
	{ 47, HIGH, 0, 1, 0, "Sensor 2", true },
	{ 43, HIGH, 0, 1, 0, "Sensor 3", true },
	{ 39, HIGH, 0, 1, 0, "sensor 4", true },
	{ 35, HIGH, 0, 1, 0, "Sensor 5", true },
	{ 31, HIGH, 0, 1, 0, "Sensor 6", true },
};


//------ ALARM STATE ------
#define PROGRAMMING 0
#define DISARMED 1
#define ARMED 2
#define ALARM 3
#define FIRE 4

#define alarm_limit 5 //stop after 5 alarms

// DE REFACUT CODUL PENTRU MASTER AND SLAVE CITITE IN PRAGRAMMING STATE !!!!!!
//------ RFID ------ 
//Declare RFIDs
#define TOKEN_NUMBER 0
static token tokens[] =
{
  {{ 111, 111, 111, 111 }, "User 1"},  // RFID #1
  {{ 111, 111, 111, 111 }, "User 2"},  // RFID #2
  {{ 111, 111, 111, 111 }, "User 3"},  // RFID #3
  {{ 111, 111, 111, 111 }, "User 4"},  // RFID #4
  {{ 111, 111, 111, 111 }, "User 5"},  // RFID #5
  {{ 111, 111, 111, 111 }, "User 6"},  // RFID #6
  {{ 111, 111, 111, 111 }, "User 7"},  // RFID #7
};

// DE VERIFICAT DOAR PENTRU RFID !!!!!
//------ NFC VARIABLES ------
PN532_I2C pn532i2c(Wire);
PN532 nfc(pn532i2c);
boolean nfc_read;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
char last_UID[30];

//------ LCD VARIABLES ------
#define LCD_I2C_ADDR    0x27  // Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);
int lcd_message_timeout = 10000; // 10 seconds
PGM_P lcd_message;
char lcd_message_str[30];
char lcd_welcome_message[20]; 
int lcd_status;
char tmp_char;

// DE REFACUT CODUL PENTRU DS18B20 !!!!!!
//------ DS18B20 VARIABLES ------
#define dht_refresh_rate 30000
dht DHT;
unsigned long dht_ts;

// DE MODIFICAT CODUL DOAR PENTRU ARMARE/DEZARMARE !!!!!!
//------ CAPACITIVE SENSORS VARIABLES ------
#define irqPin 17 // !!! ALT PIN
#define touch_timeout 1000
boolean touchStates[12], touchStates_prev[12]; //to keep track of the previous touch states
unsigned long touch_ts = 0;
char ledStatus = 0; // !!! LED MODIFICARE
byte LSB, MSB;
uint16_t touched;
bool led1, led2, led1_prev, led2_prev;
unsigned long led2_ts;

// DE VERIFICAT SI RESTRUCTURAT !!!!!
//------OPTION VARIABLES-------
//These are just default settings, they may be configured in the options webpage
int ENABLE_BACKLIGHT_CONTROL = 1;
int ENABLE_SEND_MAIL = 0;
int ENABLE_DOMOTIC_CONTROL = 1;
int ENABLE_IPCAM_CONTROL = 1;
int ENABLE_PUSH = 1;
int enable_intelligent_mode=1, enable_sensor_reactivation=0;
unsigned long override_intelligent_ts;
unsigned long alarm_timeout = 1000; //set waiting time before turning off the siren once the sensor alarm is off
unsigned long grace_period = 10000; //alarm grace period
unsigned long lcd_bk_period = 8000; //backlight duration
unsigned long siren_start_timeout = 5000; //avoid duplicate alarm start/stop request from webserver
unsigned long alarm_standby_timeout = 300; //time before siren starts again while the alarm signal is alarmed
int vol_from, vol_to; //set pause for volumetric

// DE VERIFICAT SI RESTRUCTURAT !!!!!
//------GENERAL VARIABLES-------
#define reboot_count 99
#define prev_stat_address 100

int prev_day, prev_hour, alarm_count=0;
boolean enable_alarm = false, enable_volumetric = true, enable_perimetral = true, alarm_armed = false, alarm = false;
unsigned long nfc_ts = millis(), lcd_ts = millis(), lcd_message_ts = millis(), lcd_bk_ts = millis(), siren_start_ts = millis(), reset_sensors_ts = millis(), alarm_standby_timeout_ts = millis(), alarm_delay_ts = millis();
int prev_sec = 0;
bool eth_enabled = 1;

static int nfc_period = 1000; //nfc read frequency

unsigned long grace_period_ts = millis();
bool menu_enabled = false;
int menu_option = 0;

long int alarm_timeout_ts;
bool alarm_siren_started = false;
bool alarm_standby = false;
bool force_alarm = false;
bool check_sensors_before_activation = false;

char tmp[30];
int tmp_int;
unsigned long tmp_ulong;
unsigned long delay_ts;



//============================
//###### ALARM FUNCTION ######



//###### MEGA PERIPHERIAL ######
//==============================
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
//==============================
//###### MEGA PERIPHERIAL ######




void setup(void) {

//###### MEGA PERIPHERIAL ######
//==============================
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

  //==============================
  //###### MEGA PERIPHERIAL ######


  //###### ALARM FUNCTION ######
  //============================


  // DE RESTRUCTURAT !!!!!!!!!!!!!!!!!!!!!!!
  wch_disable();
  //Initialize siren
  pinMode(alarmPin, OUTPUT);
  digitalWrite(alarmPin, LOW);//disable the alarm siren!

  Serial.begin(9600);
  
  //SAFE MODE CHECK
  if (REBOOT_RST) EEPROM.write(reboot_count, 0);
  
  if (REBOOT_CHK)
  {
    int reboot=EEPROM.read(reboot_count);
    if (EEPROM.read(reboot_count) > 5) 
    {
      Serial.println("SAFE MODE ENABLED, system rebooted more than 5 times");
      while(true);
    }
    reboot=reboot+1;
    EEPROM.write(reboot_count, reboot);
  }  

	SerialPrint_P(PSTR("Garo Anti-Theft Alarm 1.0 BOOTING"), 1);

	Wire.begin();

	SerialPrint_P(PSTR("Loading Options"), 1);
	loadOptions();

	//Initialize LCD
	SerialPrint_P(PSTR("Initializing LCD"), 1);
	if (LCD_ENABLED)
	{
        initialize_lcd();
		lcd_output_string(PSTR("AntiTheft Alarm"));
	}

	//Initialize sensors
	SerialPrint_P(PSTR("Initializing Sensors"), 1);
	for (int i = 0; i < sensor_number; i++) pinMode(sensors[i].pin, INPUT);

	//Initialize CS
	if (CS_ENABLED)
	{
		SerialPrint_P(PSTR("Initializing Capacitive Sensors"), 1);
		pinMode(irqPin, INPUT);
		digitalWrite(irqPin, HIGH); //enable pullup resistor

		mpr121_setup();
	}
	//Initialize network
  SerialPrint_P(PSTR("Initialize network"), 1);
  SerialPrint_P(PSTR("Getting DHCP"), 1);
  delay(1000);
  //if (ETH_W5100) W5100.select(53); //Uncomment this if the SS pin is different from 10

	if (Ethernet.begin(mac) == 0)
	{
		SerialPrint_P(PSTR("Failed to configure Ethernet using DHCP"));
    lcd.print("Eth failed");
    eth_enabled = false;      
	}
	else
	{
		eth_enabled = true;
		SerialPrint_P(PSTR("Ethernet initialized"),1);		
		webserver.begin();	
		lcd.setCursor(0, 1);
    lcd.print(IPAddress(Ethernet.localIP()));
    if (SERIAL_OUTPUT) Serial.println(IPAddress(Ethernet.localIP()));
	}
  	
	//Initialize NFC
	if (NFC_ENABLED) initialize_nfc();

	//Initialize GSM
	if (GSM_ENABLED) initialize_gsm();

	//Try to sync to NTP. 10 tentatives as workaround
	NtpCheck();

	if (enable_intelligent_mode) checkIntelligent();
		
	sendMessage(PSTR("Antitheft alarm has just started"));
	
	sound(0);
	SerialPrint_P(PSTR("ANTITHEFT ALARM SUCCESSFULLY BOOTED"), 1);

	log(PSTR("Start"));

	//read prev state in case of accidental reboot
	if (EEPROM.read(prev_stat_address) == 1)
	{
		if (EEPROM.read(prev_stat_address + 1) == 1) enable_perimetral = true;
		if (EEPROM.read(prev_stat_address + 2) == 1) enable_volumetric = true;
		alarm_start(true);
	}
	wch_enable();
}


  //============================
  //###### ALARM FUNCTION ######


}

void loop() {

//###### MEGA PERIPHERIAL ######
//==============================
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
void requestEvent() {    // executata la trimitere Event la ESP8266 dupa intrebare
      retparams[2]=busy;
      Wire.write(retparams,3); 
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void receiveEvent(int count) {    // executata la receptionare Event de la ESP8266
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
    case SET_OUTPUT: // seteaza OUTPUT
          if (ports[params[1]]!=1) { ports[params[1]]=1; pinMode(params[1],OUTPUT); } 
          digitalWrite(params[1],params[2]? HIGH : LOW); 
          break;
    case READ_INPUT: // citeste INPUT
          if (ports[params[1]]!=2) { ports[params[1]]=2; pinMode(params[1],INPUT); } 
          retparams[0]=0; retparams[1]=digitalRead(params[1]); 
          break;
    case READ_INPUT_PULLUP: // citeste INPUT cu PULLUP
          if (ports[params[1]]!=3) { ports[params[1]]=3; pinMode(params[1],INPUT_PULLUP); } 
          retparams[0]=0; retparams[1]=digitalRead(params[1]); 
          break;          
    case SET_PWM: // seteaza PWM
          if (ports[params[1]]!=4) { ports[params[1]]=4; pinMode(params[1],OUTPUT); } 
          analogWrite(params[1],params[2]); 
          break;
    case READ_ANALOG: // citeste intrarea ANALOG
          if (ports[params[1]]!=2) { ports[params[1]]=2; pinMode(params[1],INPUT); } 
          uint16_t anback; anback=analogRead(params[1]); retparams[0]=anback>>8; retparams[1]=anback&255;
          break;    
    case SET_ADDRESS: // seteaza adresa I2C - initial "9"
          stored.device=params[1]; EEPROM.put(STRUCTBASE,stored);
          // update address - will take effect on next powerup of the device as you 
          // can only call "begin" once
          break;
    case SEROUT: char *m; // SERIAL OUT
                 m=(char *)&params[1];
                 Serial.print(m);
                 break;
    case SERVO : if (ports[params[1]]!=5) { ports[params[1]]=5; myservos[params[1]].attach(params[1]); }  // executie SERVO MOTOR
                 if (params[2]==255) { myservos[params[1]].detach(); ports[params[1]]=0; break; }
                 myservos[params[1]].write(params[2]);
                 break; 
    case FADE: // FADE pentru LED
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

    case DALLAS1: // senzor temperatura 1
          tr1=dallas(params[1]); 
          if (params[1]!=stored.t1) { stored.t1=params[1];  EEPROM.put(STRUCTBASE,stored); } // no delay hence first value crap
          retparams[1]=tr1&255; retparams[0]=tr1>>8; 
          break;

    case DALLAS2: // senzor temperatura 2
          tr2=dallas(params[1]); 
          if (params[1]!=stored.t2) { stored.t2=params[1]; EEPROM.put(STRUCTBASE,stored); }   // no delay hence first value crap
          retparams[1]=tr2&255; retparams[0]=tr2>>8;
          break;  
                                 
   default: break;  
    }
    busy=0;

    //==============================
    //###### MEGA PERIPHERIAL ######


    //###### ALARM FUNCTION ######
    //============================

// DE RESTRUCTURAT !!!!!!!!!!!!!!!!!!!!!
wch_reset();

    if (!ETH_W5100) Ethernet.maintain(); //Fix for some UIPEthernet crashes

    wch_disable();
    if (GSM_ENABLED && gsmstarted && ((unsigned long)(millis() - gsm_ts) > gsm_refresh_rate))
    {
    	//Read if there are messages on SIM card and print them.
    	SerialPrint_P(PSTR("Checking for new SMS"), 1);
    	checkSMS();
    	gsm_ts = millis();
    }
    wch_enable();
    wch_reset();
    
    if (DHT_ENABLED && (unsigned long)(millis() - dht_ts) > dht_refresh_rate) updateDHT();
    
    //check NTP every day    
    if (day() != prev_day) NtpCheck();
    prev_day = day();
    
    if (hour() != prev_hour && year()==1970) NtpCheck();    
    
    //intelligent mode
    if (enable_intelligent_mode && ((unsigned long)(millis() - override_intelligent_ts) > 3600000) && hour()!=prev_hour && !enable_alarm) checkIntelligent();      
    prev_hour = hour();

	//manage leds and capacitive sensors
	if (CS_ENABLED)
	{
		//manage leds
		if (enable_alarm)
		{
			if ((unsigned long)(millis() - led2_ts) > 1000) led2 = true;
			if ((unsigned long)(millis() - led2_ts) > 1500)
			{
				led2 = false;
				led2_ts = millis();
			}
		}
		else led2 = false;

		if (led1_prev != led1 || led2_prev != led2)
		{
			set_register(GPIO_CLEAR, 0xFF);       // clear all leds
			delay(10);
			ledStatus = 0;
			if (led2) bitWrite(ledStatus, 0, 1);
                        if (led1) { bitWrite(ledStatus, 1, 1); bitWrite(ledStatus, 2, 1);}
			set_register(GPIO_SET, ledStatus);  // set LED
			delay(10);
		}

		led1_prev = led1;
		led2_prev = led2;

		//manage Capacitive Sensors
		readTouchInputs();
	}

	//Manage LCD menu
	if ((unsigned long)(millis() - touch_ts) > touch_timeout && touchStates[0] && !touchStates_prev[0])
	{
		touchStates[0] = 0;
		lcd_backlight_on();
		sound(5);
		touch_ts = millis();
		if (menu_enabled)
		{
			menu_option++;
			if (menu_option > 2) menu_option = 0;
		}
		else {
			menu_option = 0;
			menu_enabled = true;
		}
	}

	if ((unsigned long)(millis() - touch_ts) > touch_timeout && touchStates[1] && !touchStates_prev[1])
	{
		touchStates[1] = 0;
		lcd_backlight_on();
		touch_ts = millis();

		if (menu_enabled)
		{
			menu_enabled = false;
			switch (menu_option)
			{
			case 0://enable volum and peri
				if (!enable_alarm)
				{
					enable_volumetric = true;
					enable_perimetral = true;
					output_lcd(PSTR("All sensors OK     "));
					sound(6);
				}
				else
				{
					output_lcd(PSTR("Not allowed     "));
					sound(3);
				}
                override_intelligent_ts=millis();//override intelligent mode for 1 hour
				break;
			case 1://enable only peri
				if (!enable_alarm)
				{
					enable_volumetric = false;
					enable_perimetral = true;
					output_lcd(PSTR("Peri only OK     "));
					sound(6);
				}
				else
				{
					output_lcd(PSTR("Not allowed      "));
					sound(3);
				}
                override_intelligent_ts=millis();//override intelligent mode for 1 hour
				break;
			default://exit
				sound(6); break;
			}
		}
		else sound(6);
	}


	//search for rfid/nfc
	if (NFC_ENABLED)
	{
		if ((unsigned long)(millis() - nfc_ts) > nfc_period)
		{
			nfc_read = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength, 50);
                        wch_reset();
			nfc_period = 1000;

			if (nfc_read) {

				SerialPrint_P(PSTR("Found a card!"));
				SerialPrint_P(PSTR("UID Value: "));
				for (uint8_t i = 0; i < uidLength; i++) SerialPrint(uid[i]); SerialPrint("-");

				char user[80]="Unknown";
				int nfc_index = -1;
				nfc_index = nfc_check(uid);

				if (nfc_index>-1)
				{
					SerialPrint_P(PSTR("NFC Accepted"), 1);

					if (!enable_alarm) alarm_start(false);
					else
					{
						alarm_stop(false);
						sprintf(lcd_welcome_message, "Ciao %s      \0", tokens[nfc_index].uid_name);
						output_lcd_str(lcd_welcome_message);
						refresh_lcd();
                                                sound(2);
					}

					if (nfc_index != -1) sprintf(user, "%s", tokens[nfc_index].uid_name);
					
					nfc_period = 5000; //don't search for 5 seconds

					log(URLEncode(user));

					if (enable_alarm) sprintf(user, "%s has enabled the alarm, peri %s, vol %s", user, (enable_perimetral ? "ON" : "OFF"), (enable_volumetric ? "ON" : "OFF"));
					else sprintf(user, "%s has disabled the alarm", user);
					pushNotification(user);
					

				}
				else
				{
					sendMessage(PSTR("Someone try to enter with unrecognized RFID or NFC"));
					SerialPrint_P(PSTR("NFC not accepted"), 1);
					output_lcd(PSTR("NFC not accepted"));
                                        refresh_lcd();
                                        sound(3);

					log(URLEncode(user));

					//delay(300);
				}
			}

			nfc_ts = millis();
		}
	}


    checkSensors();
        
	alarm = false; //reset alarm
	if (siren_start_ts != 0 && ((unsigned long)(millis() - siren_start_ts)) > siren_start_timeout) alarm = true; //else, alarm after siren timeout

	if (siren_start_ts != 0 && ((unsigned long)(millis() - siren_start_ts)) > siren_start_timeout) alarm = true; //else, alarm after siren timeout

	if (((unsigned long)(millis() - reset_sensors_ts)) > 3600000) //after 1 hour with no alarm, reset sensors and alarm counter
	{
		for (int i = 0; i < sensor_number; i++) sensors[1].alarmed_timestamp = 0;
		alarm_count = 0; 
	}

	//start the alarm at the end of grace period
	if (enable_alarm)
	{
		if (!alarm_armed)
		{
			if (millis() - grace_period_ts>2000)
			{
				tmp_ulong = (unsigned long)(grace_period - (millis() - grace_period_ts));
				if (tmp_ulong > 1000)
				{
					tmp_int = (int)(tmp_ulong / (unsigned long)1000);
					sprintf(tmp, "Start in %d sec", tmp_int);
					output_lcd_str(tmp);
				}
				else
				{
					alarm_armed = true;
					for (int i = 0; i < sensor_number; i++) sensors[i].alarmed = false;
					output_lcd(PSTR("Alarm started     "));
					sound(4);
				}
			}
		}
	}
	else alarm_armed = false;
	
	//After the siren has started, wait for a certain amount of time before starting again
	if ((unsigned long)(millis() - alarm_standby_timeout_ts) > alarm_standby_timeout) alarm_standby = false;
	
	//manage "force alarm"
	if (force_alarm) digitalWrite(alarmPin, HIGH);
	else
	{
		//If the system is alarmed, start the siren, send mail...
		if (alarm_armed && !alarm_standby)
		{
			//check if the siren has to be started
			if (alarm && !alarm_siren_started && alarm_count < alarm_limit)
			{
				digitalWrite(alarmPin, HIGH);
				sound(6);			
                                log("Alarm");	
				SerialPrint_P(PSTR("ALARM!!"));

				char *buffer;
				buffer = (char*)malloc(200);
				bool found = false;
				sprintf(buffer, "%s", "ALARM! Sensors alarmed:\n");
				
				for (int i = 0; i < sensor_number; i++)
				{
					if (sensors[i].alarmed)
					{
						if (!found) sprintf(buffer, "%s %s", buffer, sensors[i].name);
						else sprintf(buffer, "%s, %s", buffer, sensors[i].name);
						found = true;
					}
				}
				
				sendMessage(buffer);
				
				if (GSM_ENABLED)
				{
          wch_disable();
                                        
					for (int i = 0; i < phones_num; i++) sms.SendSMS(phones[i], buffer);

					calling_number = 0;
					call_started = true;
					gsm_call_ts = millis();
					call.Call(phones[0]);
					SerialPrint_P(PSTR("CALLING..."));

	        wch_enable();
				}

				free(buffer);

				alarm_count++;
				alarm_siren_started = true;
				alarm_timeout_ts = millis();
			}

			//Stop siren after alarm timeout
			if (alarm_siren_started)
			{
				if ((unsigned long)(millis() - alarm_timeout_ts) > alarm_timeout)
				{
					digitalWrite(alarmPin, LOW);
					SerialPrint_P(PSTR("ALARM STOP"));
					alarm_siren_started = false;
					alarm_standby_timeout_ts = millis();
					alarm_standby = true;
				}
			}
		}
		else digitalWrite(alarmPin, LOW);
	}

	if (call_started && ((unsigned long)(millis() - gsm_call_ts) > call_hangup_time))
	{
		wch_disable();
    call.HangUp();
		SerialPrint_P(PSTR("HANGING UP CALL"));
		if (!enable_alarm) call_started = false;
		else if (calling_number < phones_num - 1)
		{
			SerialPrint_P(PSTR("CALLING NEXT NUMBER"));
			calling_number++;
			gsm_call_ts = millis();
			call.Call(phones[calling_number]);
		}
		else call_started = false;
    wch_enable();
	}

	if (eth_enabled) process_web(webserver);
	refresh_lcd();
}


    //============================
    //###### ALARM FUNCTION ######

}

//------ FUNCTIONS ------

//###### ALARM FUNCTION ######
//============================

// DE RESTRUCTURAT !!!!!!!!!!!!!!!!!!!!!!!
void loadOptions() // incarcare optiuni din memoria EEPROM
{
	int i = 0;

	alarm_timeout = ((unsigned long)EEPROM.read(i)) * 1000 * 60; i++;
	grace_period = ((unsigned long)EEPROM.read(i)) * 1000; i++; //
	lcd_bk_period = ((unsigned long)EEPROM.read(i)) * 1000; i++;
	siren_start_timeout = ((unsigned long)EEPROM.read(i)) * 1000; i++;
	alarm_standby_timeout = ((unsigned long)EEPROM.read(i)) * 1000 * 60; i++;
	ENABLE_BACKLIGHT_CONTROL = EEPROM.read(i); i++;
	ENABLE_SEND_MAIL = EEPROM.read(i); i++;
	ENABLE_DOMOTIC_CONTROL = EEPROM.read(i); i++;
	vol_from = ((int)EEPROM.read(i)); i++;
	vol_to = ((int)EEPROM.read(i)); i++;
	enable_intelligent_mode = EEPROM.read(i); i++;
	enable_sensor_reactivation = EEPROM.read(i); i++;
	ENABLE_IPCAM_CONTROL = EEPROM.read(i); i++;
	ENABLE_PUSH = EEPROM.read(i); i++;
}

void checkIntelligent() // verificare stare inteligent (automat) (ora 05:00-22:00)
{
	if (hour()>5 && hour()<22) { enable_volumetric = true; enable_perimetral = true; }
	else { enable_volumetric = false; enable_perimetral = true; }
}

byte uptime_d() // de la pornire - zile
{
  unsigned long milli = millis();
  unsigned long secs=milli/1000, mins=secs/60;
  unsigned int hours=mins/60, days=hours/24;
  milli-=secs*1000;
  secs-=mins*60;
  mins-=hours*60;
  hours-=days*24;
  return (byte)days;
}

byte uptime_h() // de la pornire - ore
{
	unsigned long milli = millis();
	unsigned long secs = milli / 1000, mins = secs / 60;
	unsigned int hours = mins / 60, days = hours / 24;
	milli -= secs * 1000;
	secs -= mins * 60;
	mins -= hours * 60;
	hours -= days * 24;
	return (byte)hours;
}

byte uptime_m() // de la pornire - minute
{
	unsigned long milli = millis();
	unsigned long secs = milli / 1000, mins = secs / 60;
	unsigned int hours = mins / 60, days = hours / 24;
	milli -= secs * 1000;
	secs -= mins * 60;
	mins -= hours * 60;
	hours -= days * 24;
	return (byte)mins;
}

byte uptime_s() // de la pornire - secunde
{
  unsigned long milli = millis();
  unsigned long secs = milli / 1000, mins = secs / 60;
  unsigned int hours = mins / 60, days = hours / 24;
  milli -= secs * 1000;
  secs -= mins * 60;
  mins -= hours * 60;
  hours -= days * 24;
  return (byte)secs;
}



//============================
//###### ALARM FUNCTION ######