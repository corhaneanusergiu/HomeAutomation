struct senzori {
    int nr; // numarul de ordine al senzorului
    int pin; // pin legatura ARDUINO ATMEGA 2560
    int tip; // tipul senzorului - pir, reed, fum, etc
    int stare; // stare HIGH sau LOW (inchis / deschis)
    PGM_P nume; // nume in clar (nr caractere limitat la dimensiune lcd, max 20) 
    int alarmat; // starea de alarmare, activa (cuplat, contact inchis)
    unsigned long int alarmat_ts; // momentul la care a fost facuta alarmarea
} senzor[];

// nr, pin, tip, stare, name, alarmat, alarmat_ts
// 2, 22, 2, 1, Usa Intrare, 1, 13345678901234567890
// 2 + 2b + 1b + 1b + 13b + 1b + 14b
//           6b + 13b +      15b
// 34b
// 

int adresa_inceput_senzori = 100;
int marime_memorie_senzor = 32;
for (int i = 0, i > NR_SENZORI - 1, i++)
{
    for (int j = 0, j > 5, j++)
    {
        adresa = adresa_inceput_senzori + 32 * i + j; 
        eeprom.write(adresa, senzor[i][j])
    }
}



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