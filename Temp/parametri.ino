//-----FUNCTION VARIABLES-----
// Enable or disable system modules
#define SERIAL_OUTPUT 1 //Enable serial debug output
#define DHT_ENABLED 1 //Enable temperature and humidity measurement
#define CS_ENABLED 1 //Enable touch buttons controller
#define LCD_ENABLED 1 //Enable LCD outpu
#define NFC_ENABLED 1 //Enable NFC detection
#define GSM_ENABLED 0 //Enable GSM functions
#define SND_ENABLED 1 //Enable sound
#define WCH_ENABLED 0 //Enable watchdog
#define REBOOT_CHK 1 //Security mode. If some component fails, after 5 reboot the system enters a safe mode to avoid the siren ring at each reboot (they may be endless!)
#define REBOOT_RST 0 //Reset security mode. If the system  is stuck, recompile with this option to 1 to reset it. When fixed, recompile with 0
#define VOLTAGE_MSR 0 //Enable measure voltage and battery protection
#define SIREN_SND 1 //Enable siren or mode program
#define ESP8266_MOD 0 //Enable ESP6288 2.4GHz module I2C
#define RFM69_MOD 0 //Enable RFM69HW 868MHz module


//----------------------------------
// Files for sistem

#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <Time.h>
#include <Timezone.h>
#include <WSWire.h>
#include <PN532_I2C.h>
#include <PN532.h>
#include "pitches.h"
#include "types.h"
#include <EEPROM.h>
#include "mpr121.h"
#include <dht.h>
#include <SoftwareSerial.h>
#include "SIM900.h"
#include "sms.h"
#include "call.h"
#include <avr/wdt.h>
// de verificat fisierele


//-----ALARM VARIABLES-----

//Declare RFIDs
// !!! prima data de trecut codurile in clar
// !!! la prima citire de facut unul admin apoi cu el de executat pe celelalte
#define token_number 7
static token tokens[] =
{
  {{ 111, 111, 111, 111 }, "User 1"},  // RFID #1 ADMIN
  {{ 111, 111, 111, 111 }, "User 2"},  // RFID #2
  {{ 111, 111, 111, 111 }, "User 3"},  // RFID #3
  {{ 111, 111, 111, 111 }, "User 4"},  // RFID #4
  {{ 111, 111, 111, 111 }, "User 5"},  // RFID #5
  {{ 111, 111, 111, 111 }, "User 6"},  // RFID #6
  {{ 111, 111, 111, 111 }, "User 7"},  // RFID #7
};



//Declare sensors array
// !!! prima data 0 senzori urmand sa se adauge prin activare din tastatura
// !!! de modificat setarile exemplu
#define sensor_number 8 // number of sensors
sensor sensors[] = 
{
  //fields: pin, state, volumetric, perimetral, alarmed_timestamp, name, enabled
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


#define alarm_limit 5 //stop after 5 alarms




//------LCD VARIABLES---------
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
int lcd_message_timeout = 5000;
PGM_P lcd_message;
char lcd_message_str[30];
char lcd_welcome_message[20]; 
int lcd_status;
char tmp_char;

//------SENSORS VARIABLES---------
#define door_sensor 0   // set the sensor number of the house door
#define alarmPin  8     // the number of the alarm pin
#define soundPin  2     // the number of the alarm pin
#define DHT22_PIN 3		// humidity and temperature sensor pin

//------DHT VARIABLES---------
#define dht_refresh_rate 30000
dht DHT;
unsigned long dht_ts;

//------CAPACITIVE SENSORS VARIABLES-------
#define irqPin 17
#define touch_timeout 1000
boolean touchStates[12], touchStates_prev[12]; //to keep track of the previous touch states
unsigned long touch_ts = 0;
char ledStatus = 0;
byte LSB, MSB;
uint16_t touched;
bool led1, led2, led1_prev, led2_prev;
unsigned long led2_ts;

//------NFC VARIABLES-------
PN532_I2C pn532i2c(Wire);
PN532 nfc(pn532i2c);
boolean nfc_read;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
char last_UID[30];



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



//----------------------------------

// parametri de functionare

// stari de functionare
#define STATE_DEZARMAT   1
#define STATE_ARMAT      2
#define STATE_ALARMA     3
#define STATE_PROGRAMARE 4

// parametri initiali
#define 




//---------------------------------
//---------------------------------
//---------------------------------
void setup() {
	wch_disable();
  //Initialize siren
  pinMode(alarmPin, OUTPUT);
  digitalWrite(alarmPin, LOW);//disable the alarm siren!

  Serial.begin(9600); // 115200 de verificat
  
  //SAFE MODE CHECK
  if (REBOOT_RST) EEPROM.write(reboot_count, 0);
  
  if (REBOOT_CHK)
  {
    int reboot=EEPROM.read(reboot_co unt);
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
	
	//Initialize NFC
	if (NFC_ENABLED) initialize_nfc();

	//Initialize GSM
	//if (GSM_ENABLED) initialize_gsm();

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


//---------------------------------
//---------------------------------
//---------------------------------

