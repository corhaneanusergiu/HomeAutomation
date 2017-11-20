// varianta initiala minimala
//
// 15.11.2017
// 
//------
//
// Componente:
// -------------------------
// Sursa 240VCA/12VCC 
// Baterie 12VCC/7A
// Sursa 12VCC
// Sursa 5VCC
// Arduino Mega 2560
// Sirena 130 DB / 12V/1,3A
// Sirena interna
// Releu Sirena
// REED (9 BUC)
// PIR (2 BUC)
// Switch ARMAT / DEZARMat
// RGB LED
// BUZZER
// --------------------------
//

// stari de functionare
#define STARE_PROGRAMARE 0
#define STARE_DEZARMAT   1
#define STARE_ARMAT      2
#define STARE_ALARMA     3


// pini
#define SENZOR // reed, pir, 1 intrare
#define BUTON // arm / dizarm
#define CAPACITIV // senzor buton
#define SIRENA_EXT // sirena exterioara
#define SIRENA_INT // sirena interioara
#define LED_ALARMA
#define BUZZER
#define INTRERUPATOR // arm / dizarm


#define PIR 1
#define REED 2
#define FUM 3
#define TEMPERATURA 4

int stare_alarma = 0;
int stare_anterioara_alarma = 0;
int led = 0;
int buton = 0;
int sirena_ext = 0;
int sirena_int = 0;
int buzzer = 0;
int capacitiv = 0;
int nr_senzori = 0;

int enable_sensor_reactivation=0;
unsigned long alarm_timeout = 1000; //set waiting time before turning off the siren once the sensor alarm is off
unsigned long grace_period = 10000; //alarm grace period
unsigned long lcd_bk_period = 8000; //backlight duration
unsigned long siren_start_timeout = 5000; //avoid duplicate alarm start/stop request from webserver
unsigned long alarm_standby_timeout = 300; //time before siren starts again while the alarm signal is alarmed

//------ VARIABILE GENERALE -------
#define reboot_count 99
#define adresa_stare_anterioara_alarma 100 // adresa din eeprom a starii anterioare

boolean permite_alarm = false;
boolean permite_pir = true;
boolean permite_reed = true;
boolean alarm_armed = false;
boolean alarm = false;

unsigned long nfc_ts = millis();
unsigned long lcd_ts = millis();
unsigned long lcd_message_ts = millis();
unsigned long lcd_bk_ts = millis();
unsigned long siren_start_ts = millis();
unsigned long reset_sensors_ts = millis();
unsigned long alarm_standby_timeout_ts = millis();
unsigned long alarm_delay_ts = millis();
unsigned long grace_period_ts = millis();

int prev_sec = 0;

static int nfc_period = 1000; //nfc read frequency


bool menu_activat = false;
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




void setup(){
    
    pinMode(SENZOR, INPUT);
    pinMode(INTRERUPATOR, INPUT);
    pinMode(BUTON, INPUT);
    pinMode(CAPACITIV, INPUT);
    pinMode(SIRENA_EXT, OUTPUT);
    pinMode(SIRENA_INT, OUTPUT);
    pinMode(LED_ALARMA, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    
}

void loop(){
    
    // verificare stare Initiala
    if (digitalRead(INTRERUPATOR) ==  HIGH){
        stare = 2; // armat
        if (digitalRead(SENZOR) == HIGH) {
            t = milis();
            stare = 3; // alarma
            digitalWrite(SIRENA_EXT, HIGH); // sirena exterior activata
            digitalWrite(SIRENA_INT, HIGH); // sirena interior activata
            digitalWrite(LED_ALARMA, HIGH); // led alarma aprins
            if (milis() - t >= 180000){
                digitalWrite(SIRENA_EXT, LOW);
                digitalWrite(SIRENA_INT, LOW);
                digitalWrite(LED_ALARMA, LOW);
            }
        }
    }
}


// FUNCTII - DE FACUT
//-------------------
void verificareSenzor() 
void ledBlink()
void alarmaActivata()
void alarmaDezactivata()
void activareSirenaExterior()
void activareSirenaInterior()
void stareProgramare()
void stareDezarmat()
void stareArmat()
void stareAlarma()
void lcd()



// Functii