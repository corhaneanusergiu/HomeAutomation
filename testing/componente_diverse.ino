
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
	digitalWrite(pin, HIGH);
}

void ledOprit(int pin)
{
	digitalWrite(pin, LOW);
}

void ledPuls(int pin)
{
	unsigned long timp = millis();
	int led_on = led_off = 200;
	
	digitalWrite(pin, HIGH);
}

