void activare_alarma(bool alarma_silentioasa)
{
    // Senzori recitire?
    if (verificare_senzori_inainte_de_activare)
    {
        citireStareSenzori();
    }
    // Sirena
    activareSirena();
    // LED RGB - rosu continuu
    // Serial
    // LCD
    // Buzzer
    // Tastatura

    break;              
}

void dezactivare_alarma(bool silent)
{
    // Senzori recitire?
    // Sirena
    dezactivareSirena();
    // LED RGB - verde
    // Serial
    // LCD
    // Buzzer
    // Tastatura

    break; 
}

void activareSirena()
{
    int sir = digitalRead(SIRENA_EXT); //verificare stare pin sirena
    if (!sirena_alarma_pornita && sir == LOW)
    {
        digitalWrite(SIRENA_EXT, HIGH);
        sirena_alarma_pornita = true;
        sirena_alarma_ts = millis();
    }
}

void dezactivareSirena()
{
    int sir = digitalRead(SIRENA_EXT); //verificare stare pin sirena
    if (sirena_alarma_pornita && sir == HIGH)
    {
        digitalWrite(SIRENA_EXT, LOW);
        sirena_alarma_pornita = false;
        sirena_alarma_ts = 0;
    }
}