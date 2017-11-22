// Corhaneanu Sergiu
// versiunea minima - pentru initializare
// v1.0 - 22.11.2017
//
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

//###### ALARM FUNCTION ######
//============================

#include <Time.h>
#include <WSWire.h>
#include "pitches.h"
#include "types.h"
#include <SoftwareSerial.h>
#include <avr/wdt.h>


//------ PINS ARDUINO MEGA 2560 ------

//#define 0 // RX0
//#define 1 // TX0
//#define 2 // PWM - INT4
//#define 3 // PWM - INT5
#define SIRENA_EXT 4 // PWM - releu
//#define SIRENA_INT 5 // PWM - releu
#define BUZZER 6 // PWM
#define LED_RGB_R 7 // PWM - RGB - (blue - PROGRAMING; green - DISARMED; red - ARMED; red blink - ALARMED)
#define LED_RGB_G 8 // PWM - RGB - (blue - PROGRAMING; green - DISARMED; red - ARMED; red blink - ALARMED)
#define LED_RGB_B 9 // PWM - RGB - (blue - PROGRAMING; green - DISARMED; red - ARMED; red blink - ALARMED)
//#define LED_5V 10 // PWM - red
//#define LED_12V 11 // PWM - red
//#define LED_240V 12 // PWM - red
//#define LED_ESP 13 // PWM - transmisiuni cu ESP8266
//#define BUTON_RESET 14 // resetare la valori initiale (anulare inregistrare senzori si tag-uri)
#define BUTON_ARMARE 15 // varianta push with led
//#define BUTON_ARMARE_CS 16 // varianta buton capacitiv
//#define CONTACT_CUTIE_ALARMA 17 // microcontact deschidere cutie alarma si cutie baterie
//#define CONTACT_CUTIE_BATERIE 18 // INT3
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
#define SENZOR_08 29 // pir sala mare 
#define SENZOR_09 30 // pir sala mica
#define SENZOR_10 31 // diverse
#define SENZOR_11 32 // diverse (cablu taiat)
#define SENZOR_12 33 // fum
#define SENZOR_13 34 // flacara
#define SENZOR_14 35 // CO2, CO, gaze
//#define TASTATURA_01 36
//#define TASTATURA_02 37
//#define TASTATURA_03 38
//#define TASTATURA_04 39
//#define TASTATURA_05 40
//#define TASTATURA_06 41
//#define TASTATURA_07 42
//#define TASTATURA_08 43
//#define VENTILATOR_ALARMA 44 // PWM
//#define VENTILATOR_BATERIE 45 // PWM
//#define ILUMINAT_URGENTA_INT 46 // PWM - leduri in cutii 12V
//#define ILUMINAT_URGENTA_EXT 47 // releu iluminat exterior 12V
//#define YALA 48 // releu yala usa principala
//#define RFID_RST 49 //
//#define RFID_MISO 50 // MISO
//#define RFID_MOSI 51 // MOSI
//#define RFID_SCK 52 // SCK - PWM
//#define RFID_SDA 53 // SS
//#define TENSIUNE_240V 54 // prezenta tensiune de la iesire releu 240VCA (ON/OFF)
//#define TENSIUNE_12V 55 // A0
//#define TENSIUNE_5V 56 // A1
//#define TEMP_EXT 57 // A2 - DS18B20 sonda exterior cutie
//#define TEMP_INT 58 // A3 - DS18B20 senzor interior cutie alarma
//#define TEMP_BAT 59 // A4 - DS18B20 senzor interior cutie baterie
//#define LDR 60 // A5
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
#define TEMP_ENABLED 0 //Enable temperature measurement
#define CS_ENABLED 0 //Enable touch buttons controller
#define LCD_ENABLED 0 //Enable LCD output
#define FIRE_ENABLED 0 //Enable Fire detection
#define GAS_ENABLE 0 //Enable Gaze detection
#define NFC_ENABLED 0 //Enable NFC detection
#define SND_ENABLED 1 //Enable sound
#define WCH_ENABLED 0 //Enable watchdog
#define REBOOT_CHK 1 //Security mode. If some component fails, after 5 reboot the system enters a safe mode to avoid the siren ring at each reboot (they may be endless!)
#define REBOOT_RST 0 //Reset security mode. If the system  is stuck, recompile with this option to 1 to reset it. When fixed, recompile with 0

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
int lcd_message_timeout = 8000; // 10 seconds
PGM_P lcd_message;
char lcd_message_str[30];
char lcd_welcome_message[20]; 
int lcd_status;
char tmp_char;



//------ SENSOR SETTINGS ------
#define NR_SENZORI 14
#define SENZOR 0 // senzor neactivat
#define REED 1
#define PIR 2
#define INCENDIU 3
#define DIVERS 4

senzor senzori[] {
    // campuri: pin, instalat, stare, tip_senzor, nume, activat, alarmed_timestamp
    // setare initiala: instalat - false, stare - high, tip_senzor - senzor, activat - false, alarmed_timestamp - 0
    //
    {22, false, HIGH, 0, "Senzor 1", false, 0},
    {23, false, HIGH, 0, "Senzor 2", false, 0},
    {24, false, HIGH, 0, "Senzor 3", false, 0},
    {25, false, HIGH, 0, "Senzor 4", false, 0},
    {26, false, HIGH, 0, "Senzor 5", false, 0},
    {27, false, HIGH, 0, "Senzor 6", false, 0},
    {28, false, HIGH, 0, "Senzor 7", false, 0},
    {29, false, HIGH, 0, "Senzor 8", false, 0},
    {30, false, HIGH, 0, "Senzor 9", false, 0},
    {31, false, HIGH, 0, "Senzor 10", false, 0},
    {32, false, HIGH, 0, "Senzor 11", false, 0},
    {33, false, HIGH, 0, "Senzor 12", false, 0},
    {34, false, HIGH, 0, "Senzor 13", false, 0},
    {35, false, HIGH, 0, "Senzor 14", false, 0},
};


//------ ALARM STATE ------
#define PROGRAMMING 0
#define DISARMED 1
#define ARMED 2
#define ALARM 3
#define FIRE 4
#define GAS 5

//------ VARIABILE OPTIUNI-------
//Acestea sunt doar setarile initiale, pot fi reconfigurate in program
int ENABLE_BACKLIGHT_CONTROL = 1;

int enable_intelligent_mode = 1;
int enable_sensor_reactivation = 0;
unsigned long override_intelligent_ts;
unsigned long alarm_timeout = 1000; //set waiting time before turning off the siren once the sensor alarm is off
unsigned long grace_period = 10000; //alarm grace period
unsigned long lcd_bk_period = 10000; //backlight duration
unsigned long siren_start_timeout = 5000; //avoid duplicate alarm start/stop request from webserver
unsigned long alarm_standby_timeout = 300; //time before siren starts again while the alarm signal is alarmed
int vol_from, vol_to; //set pause for volumetric


//------VARIABILE GENERALE-------
#define reboot_count 99
#define prev_stat_address 100


int alarm_count = 0;
bool enable_alarm = false;
bool enable_volumetric = true;
bool enable_perimetral = true;
bool alarm_armed = false;
bool alarm = false;
bool alarm_siren_started = false;
bool alarm_standby = false;
bool force_alarm = false;
bool check_sensors_before_activation = false;
long int alarm_timeout_ts;
unsigned long siren_start_ts = millis();
unsigned long reset_sensors_ts = millis();
unsigned long alarm_standby_timeout_ts = millis();
unsigned long alarm_delay_ts = millis();
unsigned long grace_period_ts = millis();
int prev_sec = 0;

unsigned long lcd_ts = millis();
unsigned long lcd_message_ts = millis();
unsigned long lcd_bk_ts = millis();
bool menu_enabled = false;
int menu_option = 0;

char tmp[30];
int tmp_int;
unsigned long tmp_ulong;
unsigned long delay_ts;
