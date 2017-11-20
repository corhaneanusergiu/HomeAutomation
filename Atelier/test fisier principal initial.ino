//==================================
// 15.11.2017
// 
//==================================

// stari de functionare
#define STARE_PROGRAMARE   0
#define STARE_PRE_DEZARMAT 1
#define STARE_DEZARMAT     2
#define STARE_PRE_ARMAT    3
#define STARE_ARMAT        4
#define STARE_PRE_ALARMA   5
#define STARE_ALARMA       6

// module sistem active
#define SERIAL_OUTPUT 1 //Enable serial debug output
#define DHT_ENABLED   0 //Enable temperature and humidity measurement and sending info to an EMONCMS server
#define CS_ENABLED    1 //Enable touch buttons controller
#define LCD_ENABLED   1 //Enable LCD output
#define NFC_ENABLED   1 //Enable NFC detection
#define GSM_ENABLED   0 //Enable GSM functions
#define SND_ENABLED   1 //Enable sound
#define WCH_ENABLED   0 //Enable watchdog
#define REBOOT_CHK    1 //Security mode. If some component fails, after 5 reboot the system enters a safe mode to avoid the siren ring at each reboot (they may be endless!)
#define REBOOT_RST    0 //Reset security mode. If the system  is stuck, recompile with this option to 1 to reset it. When fixed, recompile with 0

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


//#define                     0 // RX0
//#define                     1 // TX0
//#define                     2 // PWM - INT4
//#define                     3 // PWM - INT5
//#define                     4 // PWM
//#define                     5 // PWM
//#define                     6 // PWM
//#define                     7 // PWM
//#define                     8 // PWM
//#define                     9 // PWM
#define VENTILATOR           10 // PWM - CUTIE
#define LED_12V              11 // PWM - ROSU
#define LED_240V             12 // PWM - ROSU
#define BUZZER               13 // PWM
#define CONTACT_CUTIE        14 // microcontact usa cutie
#define SIRENA_INTERNA       15 // releul sirenei (tranzistor, mosfet)
#define SIRENA_EXTERNA       16 // releul sirenei (tranzistor, mosfet)
#define RELEU_YALA           17 //
#define BUTON_PROGRAMARE     18 // intrare in stare programare
#define BUTTON_RESET         19 // resetare la valorile initiale
#define SDA                  20 // SDA
#define SCL                  21 // SCL
#define SENZOR_01            22 // reed yala
#define SENZOR_02            23 // reed usa 1 (principala)
#define SENZOR_03            24 // reed usa 2 (secundara - acces beci)
#define SENZOR_04            25 // reed geam 1 (2 mici camera mare)
#define SENZOR_05            26 // reed geam 2 (camera mare)
#define SENZOR_06            27 // reed geam 3 (camera mica)
#define SENZOR_07            28 // pir 1 (2 in camera mare)
#define SENZOR_08            29 // pir 2 (camera mica)
#define SENZOR_09            30 // rezerva 1
#define SENZOR_10            31 // rezerva 2
#define SENZOR_11            32 // cablu taiat
#define SENZOR_12            33 // fum
#define SENZOR_13            34 // CO2
#define SENZOR_14            35 // temperatura foc
#define TASTATURA_01         36 //
#define TASTATURA_02         37 //
#define TASTATURA_03         38 //
#define TASTATURA_04         39 //
#define TASTATURA_05         40 //
#define TASTATURA_06         41 //
#define TASTATURA_07         42 //
#define TASTATURA_08         43 //
#define LED_ARMAT_ON         44 // PWM - RGB VERDE
#define LED_ARMAT_OFF        45 // PWM - RGB ALBASTRU
#define LED_ARMAT_ALARM      46 // PWM - RGB ROSU
#define BUTON_ARMARE         47 // buton (senzitiv sau intrerupator - loc ascuns)
#define BUTON_ILUMINAT       48 // iluminat cu banda interior cutie
#define RELEU_ILUMINAT       49 //
#define MISO                 50 // MISO
#define MOSI                 51 // MOSI
#define SCK                  52 // SCK - PWM
#define SS                   53 // SS
#define TENSIUNE_240VAC      54 // prezenta tensiune
#define TENSIUNE_12VCC       55 // A0 - valoare tensiune, necesara pentru urmarirea incarcarii si descarcarii
#define TENSIUNE_5VCC        56 // A1 - valoare tensiune
#define TEMP_INT             57 // A2 - temperatura interior cutie (DS18B20)
#define TEMP_EXT             58 // A3 - temperatura exterior cutie (DS18B20)
//#define                    59 // A4
//#define                    60 // A5
//#define                    61 // A6
//#define                    62 // A7
//#define                    63 // A8
//#define                    64 // A9
//#define                    65 // A10
//#define                    66 // A11
//#define                    67 // A12
//#define                    68 // A13
//#define                    69 // A14

//------ SENZORI ------
// Declarare caracteristici senzori
int numar_senzori = 0;

// Tipuri de senzori
#define NESETAT 0
#define REED    1
#define PIR     2
#define TEMP    3
#define FUM     4
#define CABLU   5
#define DIVERS  6

// caracteristici pentru starea dezarmat:
//    - pin - pin de legatura 22-35, initial 22
//    - state - HIGH sau LOW pentru starea nealarmat, initial LOW
//    - tip senzor - din lista anterioara, initial NESETAT
//    - nume - initial ""
//    - activ - true sau false, initial false
//    - alarmed_timestamp (ultima alarmare), initial 0
senzor[] = {pin, state, tip_senzor, nume, activ, alarmed_timestamp};

senzor_initial[] = {22, LOW, 0, "", false, 0};


#define limita_alarme 5 // se opreste dupa 5 alarme (pentru caz de eroare)

// Zona senzori REED
#define reed_usa    1
#define reed_geam   2
#define reed_yala   3
#define reed_beci   4
#define reed_pod    5
#define reed_alarma 6
#define reed_divers 7

int numar_senzori_reed = 0;
int numar_senzori_pir = 0;
int numar_senzori_temp - 0;
int numar_senzori_fum = 0;
int numar_senzori_cablu = 0;
int numar_senzori_divers = 0;

//------ VARIABILE LCD 20x4 ------
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

//------ VARIABILE SENZORI TEMPERATURA ------


//------ VARIABILE SENZORI CAPACITIVI ------??????????????????
#define touch_timeout 1000
boolean touchStates[12], touchStates_prev[12]; //to keep track of the previous touch states
unsigned long touch_ts = 0;
char ledStatus = 0;
byte LSB, MSB;
uint16_t touched;
bool led1, led2, led1_prev, led2_prev;
unsigned long led2_ts;


//------ VARIABILE RFID ------
PN532_I2C pn532i2c(Wire);
PN532 nfc(pn532i2c);
boolean nfc_read;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
char last_UID[30];

//------ VARIABILE OPTIUNI -------
// Acestea sunt doar setari initiale
int LUMINA_LCD_ACTIVAT = 1;

int mod_inteligent_activat = 0;
int reactivare_senzor_activat = 0;
unsigned long sprascrie_timestamp_mod_inteligent;
unsigned long timp_continuare_alarma = 1000; //seteaza timpul de asteptare inainte de a opri sirena, odata ce senzorul a fost anulat
unsigned long timp_introducere_parola = 20000; //perioada inainte de alarmare (pentru introducere cod)
unsigned long timp_lumina_lcd = 10000; //durata iluminat LCD
unsigned long timp_asteptare_pornire_sirena = 5000; //avoid duplicate alarm start/stop request from webserver
unsigned long timp_asteptare_alarma = 300; //timpul de asteptare a pornirii sirenei, cand starea este ALARMA
int pir_de_la, pir_pana_la; //setare pauza detectie pir

//------ SETARI GENERALE -------
#define contor_reboot 99
#define adresa_stare_anterioara 100

int ziua_anterioara;
int ora_anterioara;
int contor_alarma = 0;
boolean alarma_activata = false;
boolean pir_activ = true;
boolean perimetral_armat = false;
boolean alarma_armata = false
boolean alarma = false;
unsigned long rfid_timestamp = millis();
unsigned long lcd_timestamp = millis();
unsigned long mesaj_lcd_timestamp = millis();
unsigned long lumina_lcd_backlite_timestamp = millis();
unsigned long pornire_sirena_timestamp = millis();
unsigned long resetare_senzori_timestamp = millis();
unsigned long timp_asteptare_alarma_timestamp = millis();
unsigned long timp_asteptare_pornire_sirena_timestamp = millis();
int prev_sec = 0;

static int frecventa_citire_rfid = 1000; //nfc read frequency

unsigned long timp_introducere_parola_timestamp = millis();
bool menu_activ = false;
int menu_optiuni = 0;

long int alarm_timeout_ts;
bool sirena_alarma_pornita = false;
bool alarma_in_asteptare = false;
bool fortare_alarma = false;
bool verifica_senzori_inaintea_activarii = false;

char tmp[30];
int tmp_int;
unsigned long tmp_ulong;
unsigned long delay_ts;


