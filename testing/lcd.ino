void lcdAprins()
{
    return;
}

void lcdStins()
{
    return;
}

void lcdInitializare()
{
    lcd.setWire(Wire);
    lcd.begin(20, 4);
    lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
    lcdAprins();
    lcd.home();  
    return;
}

void lcdStergereTotala()
{
    return;
}

void lcdStergereLinie()
{
    return;
}

void lcdScriereLinie()
{
    return;
}

