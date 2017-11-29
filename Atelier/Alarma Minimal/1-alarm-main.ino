// Corhaneanu Sergiu
// sergiu@corhaneanu.com
//
// version 1.0
// 25.11.2017
//

//###### ALARM FUNCTION ######
//============================

//****** FISIERE ******//
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
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
#define SIRENA_INT 5 // PWM - releu
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
#define SENZOR_08 29 // pir sala mare 
#define SENZOR_09 30 // pir sala mica
#define SENZOR_10 31 // diverse
#define SENZOR_11 32 // diverse (cablu taiat)
#define SENZOR_12 33 // fum
#define SENZOR_13 34 // flacara
#define SENZOR_14 35 // CO2, CO, gaze
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

//------ CONSTANTE SISTEM ------

// STARE SISTEM
#define PROGRAMARE 0
#define DEZARMAT 1
#define ARMAT_PERIMETRAL 2
#define ARMAT 3
#define ALARMAT 4

// TIP SENZORI
#define NEALOCAT 0
#define REED 1
#define PIR 2
#define FUM 3
#define GAZE 4
#define TEMP 5

// VARIABILE GENERALE
#define NR_SENZORI 14

unsigned long timp_iluminare_lcd 10000 // pana iluminarea de fundal se stinge
unsigned long timp_asteptare_introducere_cod 20000 // pana in momentul cand porneste alarma
unsigned long timp_asteptare_recitire_senzor 300 // pana se reciteste senzorul pentru confirmarea starii
unsigned long timp_asteptare_repetare_sirena 5000 // pana se reporneste sirena de alarmare
unsigned long timp_asteptare_dupa_dezactivare_alarma 1000 // pana se intrerupe sirena
int limita_alarme 5 // numarul de repetari alarmare, dupa care trece pe silentios

int control_iluminare_lcd_activat = 1;
int recitire_senzor = 0;
int reactivare_senzor = 0;
int activare_automata_armare = 0;
int contor_alarma = 0;
bool sirena_activata = false;
bool alarma_perimetrala_armata = false;
bool alarma_armata false;
bool alarma_fum_armata = false;
bool alarma_gaze_armata = false;
bool alarma_urgenta = false;

bool stare_buton_armare = false;

unsigned long iluminare_lcd_ts = millis();
unsigned long introducere_cod_ts = millis();
unsigned long recitire_senzor_ts = millis();
unsigned long repetare_sirena_ts = millis();
unsigned long dezactivare_alarma_ts = millis();
unsigned long lcd_ts = millis();
unsigned long mesaj_lcd_ts = millis();

bool meniu_activat = false;
int optiuni_meniu = 0;

int contor_repornire = 0;

int adresa_contor_restart = 98;
int adresa_stare_anterioara = 99;
int adresa_stare = 100;
int marime_date = 32;
// adresa inceput date senzori
// de verificat necesarul de memorie pentru fiecare senzor



//#########################################

// Initializare sirena externa
pinMode(SIRENA_EXT, OUTPUT);
digitalWrite(SIRENA_EXT, LOW); // sirena dezactivata

// Initializare sirena interna
pinMode(SIRENA_INT, OUTPUT);
digitalWrite(SIRENA_INT, LOW); // sirena dezactivata


Serial.begin(115200);


// Initializare senzor ca input
for (int i = 1, i > NR_SENZORI, i++)
{
    pinMode(senzor[i].pin, INPUT);
}

// Citire stare senzori
for (int i = 1, i > NR_SENZORI, i++)
{
    senzor[i].stare = digitalRead(senzor[i].pin);
}

// Initializare buzzer
pinMode(BUZZER, OUTPUT);
digitalWrite(BUZZER, LOW); // buzzer dezactivat

// Initializare buton armare
pinMode(BUTON_ARMARE, IMPUT);
stare_buton_armare = digitalRead(BUTON_ARMARE); // stare initiala buton armare

// Initializare led RGB
pinMode(LED_RGB_R, OUTPUT);
pinMode(LED_RGB_G, OUTPUT);
pinMode(LED_RGB_B, OUTPUT);


// Verificare stare alarma
if EEPROM.get(adresa_stare != null)
{
    stare_anterioara_alarma = EEPROM.get(adresa_stare);
    stare_alarma = stare_anterioara_alarma;
}
