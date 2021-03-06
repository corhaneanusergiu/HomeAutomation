// Corhaneanu Sergiu
//
// v1.0 - 29.11.2017
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
// include file

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
#include <string>
#include "lcd.ino" // de verificat

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
//

// Module sistem - activare
int MODUL_SERIAL = 1;
int MODUL_TASTATURA = 0;
int MODUL_LCD = 1;
int MODUL_RFID = 0;
int MODUL_TOUCH = 0;
int MODUL_TEMP = 0;
int MODUL_FUM = 0;
int MODUL_GAZE = 0;

// Stare sistem
#define PROGRAMARE 0
#define DEZARMAT 1
#define PERIMETRAL 2
#define ARMAT 3
#define ALARMA 4
#define FOC 5
#define GAZE 6
#define DIVERSE 7

// Senzori
// Tipuri de senzori
#define SENZOR_GEN 0
#define REED 1
#define PIR 2
#define FUM 3
#define FOC 4
#define TEMP 5

// date senzori
int nr_senzori = 14;

// structura senzori
senzori senzor[] = 
{
    //campuri: nr, pin, adresa, tip, activ, stare, nume, alarmat, alarmat_ts
    //seteaza stare, alarmat_timestamp la 0 si enabled to false
	{ 1, 22, 100, 0, false, HIGH, "SENZOR_01", true, 0 },
	{ 2, 23, 200, 0, false, HIGH, "SENZOR_02", true, 0 },
    { 3, 24, 300, 0, false, HIGH, "SENZOR_03", true, 0 },
	{ 4, 25, 400, 0, false, HIGH, "SENZOR_04", true, 0 },
	{ 5, 26, 500, 0, false, HIGH, "SENZOR_05", true, 0 },
	{ 6, 27, 600, 0, false, HIGH, "SENZOR_06", true, 0 },
	{ 7, 28, 700, 0, false, HIGH, "SENZOR_07", true, 0 },
	{ 8, 29, 800, 0, false, HIGH, "SENZOR_08", true, 0 },
	{ 9, 30, 900, 0, false, HIGH, "SENZOR_09", true, 0 },
	{ 10, 31, 1000, 0, false, HIGH, "SENZOR_10", true, 0 },
	{ 11, 32, 1100, 0, false, HIGH, "SENZOR_11", true, 0 },
	{ 12, 33, 1200, 0, false, HIGH, "SENZOR_12", true, 0 },
	{ 13, 34, 1300, 0, false, HIGH, "SENZOR_13", true, 0 },
	{ 14, 35, 1400, 0, false, HIGH, "SENZOR_14", true, 0 },
};

//------LCD VARIABLES---------
#define LCD_I2C_ADDR 0x27  // Defineste adresa I2C PCF8574A LCD 20x4
#define BACKLIGHT_PIN 3
#define En_pin 2
#define Rw_pin 1
#define Rs_pin 0
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);
int lcd_message_timeout = 5000;
PGM_P lcd_message;
char lcd_message_str[30];
char lcd_welcome_message[20]; 
int lcd_status;
char tmp_char;

//------VARIABILE GENERALE-------

// Adrese din EEPROM
#define EE_CONTOR_REPORNIRE 90
#define EE_STARE_ANTERIOARA_ALARMA 50
#define EE_STARE_ALARMA 51

// Setarile initiale, 
//de configurat pentru modificare din tastatura
int nr_reporniri_sistem = 0;
int nr_max_reporniri_sistem = 5;

char stare_alarma_nume[10];
int stare_alarma = 0;
bool alarma_silentioasa = false;

unsigned long led_rgb_r_ts = 0;
unsigned long led_rgb_g_ts = 0;
unsigned long led_rgb_b_ts = 0;
unsigned long led_240v_ts = 0;
unsigned long led_12v_ts = 0;
unsigned long led_5v_ts = 0;
unsigned long led_esp_ts = 0;
int timp_led_puls_on = 300;
int timp_led_puls_off = 300;

int activat_control_lumina_lcd = 1;
unsigned long perioada_iluminat_lcd = 10000; //durata iluminat lcd

bool senzor_activ = false;
bool reactivare_senzor = false;
bool recitire_senzor = false;

unsigned long timp_asteptare_cod = 30000; //perioada de asteptare alarmare pentru introducere cod
unsigned long timp_asteptare_pornire_sirena = 1000; //previne dublarea cererii de pornire/oprire a alarmei din retea
unsigned long timp_asteptare_oprire_sirena = 1000; //seteaza timpul de asteptare inainte de inchiderea sirenei odata ce alarma sunt dezactivata
unsigned long timp_asteptare_repornire_sirena = 300; //timpul pana la repornirea sirenei in starea alarma
unsigned long timp_asteptare_recitire_senzor = 500; //perioada dupa care se face recitirea senzorului pentru confirmarea starii

int contor_alarma = 0;
bool alarma_programare = false; 
bool alarma_perimetru_armata = false;
bool alarma_armata = false;
bool alarma_activata = false;
bool alarmare = false;
bool sirena_alarma_pornita = false;
unsigned long sirena_alarma_ts = 0;
bool asteptare_alarma = false;
bool fortare_alarma = false;
bool verificare_senzori_inainte_de_activare = false;
bool verificare_senzori_dupa_activare = true;
bool verificare_senzori_dupa_alarmare = true;


//============================
//###### ALARM FUNCTION ######



//###### MEGA PERIPHERIAL ######
//==============================

#define MAXPORTS 68

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
#define ARMING 17
#define DISARMING 18
#define ALARM 19
#define PROGRAMMING 20
#define PERIMETRAL 21

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
    if (stored.chsm!=0x3d) {
        
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
    for (a=0;a<12;a++) {
        fade[a][0]=0;
        fade[a][2]=0;
    }
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
    
    
    //Initializare sirena externa
    digitalWrite(SIRENA_EXT, LOW); //dezactivare iesire sirena externa!
    pinMode(SIRENA_EXT, OUTPUT); 

    //Initializare sirena interna
    digitalWrite(SIRENA_INT, LOW); //dezactivare iesire sirena interna!
    pinMode(SIRENA_INT, OUTPUT); 

    // Initializare serial
    if (MODUL_SERIAL == 1)
    {
        Serial.begin(115200);    
    }
        
    SerialPrint_P(PSTR("Alarma Atelier 1.0 BOOTARE"), 1); // Alarma Atelier 1.0 Initializare
    
	Wire.begin();

	//Initializare senzori
	SerialPrint_P(PSTR("Initializare Senzori"), 1);
	for (int i = 0; i < NR_SENZORI; i++) pinMode(senzori[i].pin, INPUT);
    
	SerialPrint_P(PSTR("ALARMA PORNITA"), 1);
    
	log(PSTR("Start"));
    
	// verificare starea anterioara in caz de repornire accidentala
    SerialPrint_P(PSTR("Verific starea anterioara in EEPROM !"), 1);
    if (EEPROM.read(adresa_stare) != null)
    {
        stare_alarma = EEPROM.read(adresa_stare);
        
        SerialPrint_P(PSTR("Stare anterioara in EEPROM: %s", stare_alarma), 1);
    }
    else
    {
        stare_alarma = PROGRAMARE;
    }
    
    SerialPrint_P(PSTR("Verificat starea anterioara in EEPROM"), 1);


    // Initializare leduri

    pinMode(LED_240V, OUTPUT); 
    digitalWrite(LED_240V, HIGH);

    pinMode(LED_12V, OUTPUT); 
    digitalWrite(LED_12V, HIGH);
    
    pinMode(LED_5V, OUTPUT); 
    digitalWrite(LED_5V, HIGH);
    
    pinMode(LED_RGB_R, OUTPUT); 
    digitalWrite(LED_RGB_R, HIGH);
    
    pinMode(LED_RGB_G, OUTPUT); 
    digitalWrite(LED_RGB_G, HIGH);
    
    pinMode(LED_RGB_B, OUTPUT); 
    digitalWrite(LED_RGB_B, HIGH);
    
    // Initializare LCD
    // Initializare RFID
    // Initializare TOUCH

}
wch_enable();


//============================
//###### ALARM FUNCTION ######


void loop()
{
    
    //###### MEGA PERIPHERIAL ######
    //==============================
    if (mymillis<millis())
    {
        mymillis=millis()+10;
        for (int a=0; a<12; a++)
        {
            if (fade[a][0])
            {
                if (fade[a][1]<fade[a][2])
                {
                    if (++fade[a][1]==fade[a][2]) fade[a][0]=0;
                    analogWrite(a,pgm_read_word_near(ledTable+fade[a][1]));
                }
                if (fade[a][1]>fade[a][2])
                {
                    if (--fade[a][1]==fade[a][2]) fade[a][0]=0;
                    analogWrite(a,pgm_read_word_near(ledTable+fade[a][1]));
                }
            }
        }
    }
    //==============================
    //###### MEGA PERIPHERIAL ######
    
    //###### ALARM FUNCTION ######
    //============================

    

    //============================
    //###### ALARM FUNCTION ######

}


//=====================
//=====================
//#####################
//###    FUNCTII    ###
//#####################
//=====================
//=====================


//###### MEGA PERIPHERIAL ######
//==============================

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{   // executata la trimitere Event la ESP8266 dupa intrebare
    retparams[2]=busy;
    Wire.write(retparams,3); 
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void receiveEvent(int count)
{    // executata la receptionare Event de la ESP8266
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
        if (ports[params[1]]!=1)
        {
            ports[params[1]]=1; pinMode(params[1],OUTPUT);
        } 
        digitalWrite(params[1],params[2]? HIGH : LOW); 
        break;
        case READ_INPUT: // citeste INPUT
        if (ports[params[1]]!=2)
        {
            ports[params[1]]=2;
            pinMode(params[1],INPUT);
        } 
        retparams[0]=0;
        retparams[1]=digitalRead(params[1]); 
        break;
        case READ_INPUT_PULLUP: // citeste INPUT cu PULLUP
        if (ports[params[1]]!=3)
        {
            ports[params[1]]=3;
            pinMode(params[1],INPUT_PULLUP);
        } 
        retparams[0]=0;
        retparams[1]=digitalRead(params[1]); 
        break;          
        case SET_PWM: // seteaza PWM
        if (ports[params[1]]!=4)
        {
            ports[params[1]]=4;
            pinMode(params[1],OUTPUT);
        } 
        analogWrite(params[1],params[2]); 
        break;
        case READ_ANALOG: // citeste intrarea ANALOG
        if (ports[params[1]]!=2)
        {
            ports[params[1]]=2;
            pinMode(params[1],INPUT);
        } 
        uint16_t anback;
        anback=analogRead(params[1]);
        retparams[0]=anback>>8;
        retparams[1]=anback&255;
        break;    
        case SET_ADDRESS: // seteaza adresa I2C - initial "9"
        stored.device=params[1];
        EEPROM.put(STRUCTBASE,stored);
        // update address - will take effect on next powerup of the device as you 
        // can only call "begin" once
        break;
        case SEROUT: 
        char *m; // SERIAL OUT
        m=(char *)&params[1];
        Serial.print(m);
        break;
        case SERVO : 
        if (ports[params[1]]!=5)
        {
            ports[params[1]]=5;
            myservos[params[1]].attach(params[1]);
        }  // executie SERVO MOTOR
        if (params[2]==255)
        {
            myservos[params[1]].detach();
            ports[params[1]]=0; break;
        }
        myservos[params[1]].write(params[2]);
        break; 
        case FADE: // FADE pentru LED
        if (ports[params[1]]!=4)
        {
            ports[params[1]]=4;
            pinMode(params[1],OUTPUT);
        } 
        fade[params[1]][0]=1;
        fade[params[1]][2]=params[2];
        break;  
        
        case TONE:  // can't do PWM on pins 2 and 11 while doing this... only one pin at a time...use NOTONE when finished
        if ((params[4]|params[5])==0) tone(params[1],(params[2]<<8)+params[3]);
        else tone(params[1],(params[2]<<8)+params[3],(params[4]<<8)+params[5]); 
        ports[params[1]]=0;
        break; 
        case NOTONE:  // can't do PWM on pins 3 and 11 while doing TONE...
        noTone(params[1]); ports[params[1]]=0; 
        break;
        case DALLAS1: // senzor temperatura 1
        tr1=dallas(params[1]); 
        if (params[1]!=stored.t1)
        {
            stored.t1=params[1];
            EEPROM.put(STRUCTBASE,stored);
        } // no delay hence first value crap
        retparams[1]=tr1&255;
        retparams[0]=tr1>>8; 
        break;
        case DALLAS2: // senzor temperatura 2
        tr2=dallas(params[1]); 
        if (params[1]!=stored.t2)
        {
            stored.t2=params[1];
            EEPROM.put(STRUCTBASE,stored);
        }   // no delay hence first value crap
        retparams[1]=tr2&255;
        retparams[0]=tr2>>8;
        break;
        case ARMING: // armare sistem
        alarm_armed = true;
        break;
        case DISARMING:
        alarm_armed = false;
        break;
        case ALARM:
        alarm = true
        break;
        case PROGRAMMING:
        break;
        
        default: break;  
    }
    busy=0;
}
//==============================
//###### MEGA PERIPHERIAL ######


//###### ALARM FUNCTION ######
//============================



//============================
//###### ALARM FUNCTION ######