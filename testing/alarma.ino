void activare_alarma(bool alarma_silentioasa)
{
    // Senzori recitire?
    // Sirena
    activareSirena();
    // LED RGB - albastru pulsatoriu
    // Serial
    // LCD
    // Buzzer
    // Tastatura

    break;
    }
              
}

void dezactivare_alarma(bool silent)
{

}

void activareSirena()
{
    int sir = digitalRead(SIRENA_EXT); //verificare stare sirena
    if (!sirena_alarma_pornita && sir == 0)
    {
        digitalWrite(SIRENA_EXT, HIGH);
        siren_start_ts = millis();
    }
}

void dezactivareSirena()
{
    if (!sirena_alarma_pornita)
}