//==================================
// 15.11.2017
// 
//==================================

// stari de functionare
#define STARE_PROGRAMARE 0
#define STARE_DEZARMAT   1
#define STARE_ARMAT      2
#define STARE_ALARMA     3

// module sistem active
#define SERIAL_OUTPUT 1 //Enable serial debug output
#define DHT_ENABLED 0 //Enable temperature and humidity measurement and sending info to an EMONCMS server
#define CS_ENABLED 1 //Enable touch buttons controller
#define LCD_ENABLED 1 //Enable LCD output
#define NFC_ENABLED 1 //Enable NFC detection
#define GSM_ENABLED 0 //Enable GSM functions
#define SND_ENABLED 1 //Enable sound
#define WCH_ENABLED 0 //Enable watchdog
#define REBOOT_CHK 1 //Security mode. If some component fails, after 5 reboot the system enters a safe mode to avoid the siren ring at each reboot (they may be endless!)
#define REBOOT_RST 0 //Reset security mode. If the system  is stuck, recompile with this option to 1 to reset it. When fixed, recompile with 0

// de verificat fisierele necesare
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <Time.h>
#include <Timezone.h>
#include <WSWire.h>
#include "pitches.h"
#include "types.h"
#include <EEPROM.h>
#include <dht.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>


// pini arduino mega
// PINI ANALOG - PINI DIGITALI
//  A0 - 54 |  A1 - 55 |  A2 - 56 |  A3 - 57 
//  A4 - 58 |  A5 - 59 |  A6 - 60 |  A7 - 61
//  A8 - 62 |  A9 - 63 | A10 - 64 | A11 - 65 
// A12 - 66 | A13 - 67 | A14 - 68 | A15 - 69
//
//
//#define 0 // RX0
//#define 1 // TX0
//#define 2 // PWM - INT4
//#define 3 // PWM - INT5
//#define 4 // PWM
//#define 5 // PWM
//#define 6 // PWM
//#define 7 // PWM
//#define 8 // PWM
//#define 9 // PWM
//#define 10 // PWM
//#define 11 // PWM
//#define 12 // PWM
//#define 13 // PWM
//#define 14
//#define 15
//#define 16
//#define 17
//#define 18 // INT3
//#define 19 // INT2
//#define 20 // SDA - INT1
//#define 21 // SCL - INT0
//#define 22
//#define 23
//#define 24
//#define 25
//#define 26
//#define 27
//#define 28
//#define 29
//#define 30
//#define 31
//#define 32
//#define 33
//#define 34
//#define 35
//#define 36
//#define 37
//#define 38
//#define 39
//#define 40
//#define 41
//#define 42
//#define 43
//#define 44 // PWM
//#define 45 // PWM
//#define 46 // PWM
//#define 47
//#define 48
//#define 49
//#define 50 // MISO
//#define 51 // MOSI
//#define 52 // SCK - PWM
//#define 53 // SS
//#define 54
//#define 55 // A0
//#define 56 // A1
//#define 57 // A2
//#define 58 // A3
//#define 59 // A4
//#define 60 // A5
//#define 61 // A6
//#define 62 // A7
//#define 63 // A8
//#define 64 // A9
//#define 65 // A10
//#define 66 // A11
//#define 67 // A12
//#define 68 // A13
//#define 69 // A14

#define BUZZER 12
#define RELEU_YALA 14
#define SIRENA_INTERNA 15 // releul sirenei (tranzistor, mosfet)
#define SIRENA_EXTERNA 16 // releul sirenei (tranzistor, mosfet)
#define BUTON_PROGRAMARE 35  // buton programare (sau direct din program)
#define BUTON_RESET 36 // reset la setari initiale
#define BUTON_ARMARE 37
#define LED_240V // rgb 1 sau bicolor RED
#define LED_12V // rgb 1 sau bicolor BLUE
#define LED_ARMAT_OFF //rgb 2 BLUE
#define LED_ARMAT_ON // rgb 2 GREEN
#define LED_ARMAT_ALARM // rgb 2 RED
#define TEMP_INT // temperatura interior cutie
#define TEMP_EXT // temperatura exterior cutie
#define VENTILATOR // cutie
#define INTRERUPATOR_CUTIE D38 
#define TENSIUNE_240VAC 39// prezenta tensiune
#define TENSIUNE_12VCC A0// valoare tensiune necesara pentru urmarirea incarcarii si descarcarii
#define TENSIUNE_5VCC A1// valoare tensiune
#define CONTACT_CUTIE 40 // microcontact usa cutie
#define SENZOR_01 22 // reed yala
#define SENZOR_02 23 // reed usa 1
#define SENZOR_03 24 // reed usa 2
#define SENZOR_04 25 // reed geam 1
#define SENZOR_05 26 // reed geam 2
#define SENZOR_06 27 // reed geam 3
#define SENZOR_07 28 // pir 1
#define SENZOR_08 29 // pir 2
#define SENZOR_09 30 // pir 3
#define SENZOR_10 31 // divers
#define SENZOR_11 32 // divers pir smuls
#define SENZOR_12 33 // divers cablu taiat
//#define SENZOR_FUM
//#define SENZOR_CO2
//#define SENZOR_CO
//#define RELEU_ILUMINAT_URGENTA
//#define BUTON_ILUMINAT_URGENTA 
#define TASTATURA_01 42
#define TASTATURA_02 43
#define TASTATURA_03 44
#define TASTATURA_04 45
#define TASTATURA_05 46
#define TASTATURA_06 47
#define TASTATURA_07 48
#define TASTATURA_08 49


// parametri initiali

// SETARI STARE
int stare_prev
int stare
int stare_next

// SETARI TASTATURA

// SETARI LCD 20X4 I2C

// SETARI RFID


