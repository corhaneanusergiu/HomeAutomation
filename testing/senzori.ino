struct senzori {
    int nr; // numarul de ordine al senzorului
    int pin; // pin legatura ARDUINO ATMEGA 2560
    int adresa; // adresa corespunzatoare in eeprom
    int tip; // tipul senzorului - pir, reed, fum, etc
    int activ; // luat in considerare la verificarea starii
    int stare; // stare HIGH sau LOW (inchis / deschis)
    PGM_P nume; // nume in clar (nr caractere limitat la dimensiune lcd, max 20) 
    int alarmat; // starea de alarmare, activa (cuplat, contact inchis)
    unsigned long int alarmat_ts; // momentul la care a fost facuta alarmarea
} senzor[];

// nr, pin, tip, activ,  stare, name, alarmat, alarmat_ts

void citireStareSenzor(int nr)
{
    senzor[nr].stare = digitalRead(pin);
    int stare_senzor = senzor[nr].stare;

    return stare_senzor;
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