void pornireAlarma(bool silentios){
    alarma = true;
    perioada_gratie_ts = millis();
    contor_alarma = 0;
    sirena_pornita = false;

    refresh_lcd();

    in (!silentios) sound(1);

    //salvare stare curenta pentru caz de restart accidental
    EEPROM.write(adresa_stare_anterioara, )
}

void alarm_start(bool silent, bool domotic)
{
	if (check_sensors_before_activation && enable_perimetral)
	{
		output_lcd(PSTR("Windows open"));		
		sound(3);
		//return;
	}
	enable_alarm = true;
	grace_period_ts = millis();
	alarm_count = 0;
	alarm_siren_started = false;
	
    refresh_lcd();
        
	if (enable_volumetric) enableIpCam();

	if (domotic)
	{
		if (hour() > 6 && hour() < 22) domotic_command("*1*0*0##"); //turning off all lights
		domotic_command("*2*2*0##");//close all the windows
	}
	if (!silent) sound(1);

	//save current state to restore it after accidental reboot
	EEPROM.write(prev_stat_address, 1);
	EEPROM.write(prev_stat_address + 1, enable_perimetral);
	EEPROM.write(prev_stat_address + 2, enable_volumetric);
}

void alarm_stop(bool silent)
{
	alarm_stop(silent, true);
}

void alarm_stop(bool silent, bool domotic)
{
	enable_alarm = false;
	alarm_count = 0;
	alarm_siren_started = false;

        refresh_lcd();
	
	disableIpCam();

	if (domotic)
	{
		if (hour() > 6 && hour() < 22) domotic_command("*2*1*0##"); //open windows
		else domotic_command("*1*1*12##"); //turning on tv's light
	}

	//save current state to restore it after accidental reboot
	EEPROM.write(prev_stat_address, 0);
}
