
bool checkInterrupt(void) {
	return digitalRead(irqPin);
}

uint8_t two[2];
void set_register(unsigned char r, unsigned char v) {
	Wire.beginTransmission(0x5A);
	Wire.write(r);
	Wire.write(v);
	Wire.endTransmission();
}

//print a PSTR string on the Serial port
void SerialPrint_P(PGM_P str, boolean println) {
	if (!SERIAL_OUTPUT) return;
	for (uint8_t c; (c = pgm_read_byte(str)); str++) Serial.write(c);
	if (println) Serial.write('\n');
}

void PSTRtoSTR(char *buffer, PGM_P pstr)
{
	//buffer = (char *)malloc(strlen_P(pstr) + 1);
	strcpy_P(buffer, pstr);
	//free(buffer);
}

//print a PSTR string on the Serial port
void SerialPrint_P(PGM_P str) {
	SerialPrint_P(str, true);
}

void SerialPrintln(char* str)
{
	if (!SERIAL_OUTPUT) return;
	Serial.println(str);
}

void SerialPrint(char* str)
{
	if (!SERIAL_OUTPUT) return;
	Serial.print(str);
}

void SerialPrintln(uint32_t str)
{
	if (!SERIAL_OUTPUT) return;
	Serial.println(str);
}

void SerialPrint(uint32_t str)
{
	if (!SERIAL_OUTPUT) return;
	Serial.print(str);
}

int freeRam()
{
	extern int __heap_start, *__brkval;
	int v;
	return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

int decimal(double x)
{
	int temp = x * 10;
	int temp2 = (int)x * 10;
	return temp - temp2;
}

//###### WCH ######
void wch_enable()
{
  if (WCH_ENABLED) wdt_enable(WDTO_8S);
}

void wch_disable()
{
  if (WCH_ENABLED) wdt_disable();
}


void wch_reset()
{
  if (WCH_ENABLED) wdt_reset();
}


//###### LED ######
void ledPornit(int pin)
{
	if (digitalRead(pin) == LOW) digitalWrite(pin, HIGH);
}

void ledOprit(int pin)
{
	if (digitalRead(pin) == HIGH) digitalWrite(pin, LOW);
}

void ledPuls(int pin, unsigned long &led_ts, int timp_on = 300, int timp_off = 300)
{// valoarea ts se transfera prin referinta (&led_ts) pentru modificare la baza

	unsigned long curent_millis = millis();
	int stare_led = digitalRead(pin);

    if((stare_led == HIGH) && (curent_millis - led_ts >= timp_on))
    {
        stare_led = LOW;
        led_ts = curent_millis;
        digitalWrite(pin, stare_led);
    }
    else if ((stare_led == LOW) && (curent_millis - led_ts >= timp_off))
    {
        stare_led = HIGH;
		led_ts = curent_millis;
		digitalWrite(pin, stare_led);
    }
}

void ledColor(void)
{
	return;
}