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
#include <Timezone.h>
#include <WSWire.h>
#include "pitches.h"
#include "types.h"
#include <dht.h>
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

//###### ALARM FUNCTION ######
//============================

#include <Time.h>
#include <WSWire.h>
#include "pitches.h"
#include "types.h"
#include <SoftwareSerial.h>
#include <avr/wdt.h>

