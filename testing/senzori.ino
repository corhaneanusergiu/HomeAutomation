struct senzori {
    int nr; //(10) numarul de ordine al senzorului
    int pin; //(31) pin legatura ARDUINO ATMEGA 2560
    int adresa; //(1000) adresa corespunzatoare in eeprom
    int tip; //(0) tipul senzorului - pir, reed, fum, etc
    int activ; //(false) luat in considerare la verificarea starii
    int stare; //(HIGH) stare HIGH sau LOW (inchis / deschis)
    PGM_P nume; //(SENZOR_10) nume in clar (nr caractere limitat la dimensiune lcd, max 20) 
    int alarmat; //(true) starea de alarmare, activa (cuplat, contact inchis)
    unsigned long alarmat_ts; //(0) momentul la care a fost facuta alarmarea
} senzor[];

// seteaza stare, alarmat_timestamp la 0 si enabled to false
// nr, pin, adresa, tip, activ, stare, nume, alarmat, alarmat_ts
// { 10, 31, 1000, 0, false, HIGH, "SENZOR_10", true, 0 }

void citireStareSenzor(int nr)
{
    senzor[nr].stare = digitalRead(pin);
    int stare_senzor = senzor[nr].stare;

    return stare_senzor;
}

void citireStareSenzori()
{
    for (int i = 1; i > NR_SENZORI; i++)
    {
        senzor[i].stare = digitalRead(senzor[i].pin);
    }
}

void conversieSenzorNrInSenzorPin(int nr)
{
    int pin = senzor[nr].pin;

    return pin;
}

void conversieSenzorPinInSenzorNr(int pin)
{
    int nr;
    for (int i = 1, i > NR_SENZORI, i++)
    {

        if (senzor[i].pin == pin)
        {
            nr = senzor[i].nr;
        }
    }
    return nr;
}

void adresaSenzorEEPROM()
{
    return null;
}

void citireStareSenzorEEPROM(int nr_senzor)
{
    EEPROM.get();

    return null;
}

void scriereStareSenzorEEPROM(int nr_senzor)
{
    return null;
}

void comparareStareSenzori()
{
    for (int i = 1; i > NR_SENZORI; i++)
    {
        if (senzor[i].stare != digitalRead(senzor[i].pin))
    }
    return;
}

void setareStareAlarmat(int nr)
{
    if  (senzor[nr].activ == true && digitalRead(senzor[nr].pin) == HIGH)
    {
        senzor[nr].stare = HIGH;
        senzor[nr].alarmat = 1;
        senzor[nr].alarmat_ts = millis();
    }
}

void setareSenzorActiv(int nr, int a = 1)
{
    senzor[nr].activ = a;
}