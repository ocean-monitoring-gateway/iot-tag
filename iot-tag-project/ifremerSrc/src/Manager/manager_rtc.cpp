/*
 * manager_rtc.cpp
 *
 *  Created on: 18 févr. 2020
 *      Author: jfezande
 */

#include "Arduino.h"
#include "RTC.h"

#include "manager_rtc.h"


const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

void manager_rtc_setDefaultTime(void)   // Function sets the RTC to the FW build date-time...
{
	char Build_mo[3];
	uint8_t hours[4] = {12,12,12,12}, minutes[4] = {0,0,0,0}, seconds[4] = {0,0,0,0}, year = 1, month = 1, day = 1;

	String build_mo = "";

	Build_mo[0] = build_date[0];    // Convert month string to integer
	Build_mo[1] = build_date[1];
	Build_mo[2] = build_date[2];
	for(uint8_t i=0; i<3; i++)
	{
		build_mo += Build_mo[i];
	}
	if(build_mo == "Jan")
	{
		month = 1;
	} else if(build_mo == "Feb")
	{
		month = 2;
	} else if(build_mo == "Mar")
	{
		month = 3;
	} else if(build_mo == "Apr")
	{
		month = 4;
	} else if(build_mo == "May")
	{
		month = 5;
	} else if(build_mo == "Jun")
	{
		month = 6;
	} else if(build_mo == "Jul")
	{
		month = 7;
	} else if(build_mo == "Aug")
	{
		month = 8;
	} else if(build_mo == "Sep")
	{
		month = 9;
	} else if(build_mo == "Oct")
	{
		month = 10;
	} else if(build_mo == "Nov")
	{
		month = 11;
	} else if(build_mo == "Dec")
	{
		month = 12;
	} else
	{
		month = 1;                                                                                       // Default to January if something goes wrong...
	}
	if(build_date[4] != 32)                                                                            // If the first digit of the date string is not a space
	{
		day   = (build_date[4] - 48)*10 + build_date[5]  - 48;                                           // Convert ASCII strings to integers; ASCII "0" = 48
	} else
	{
		day   = build_date[5]  - 48;
	}
	year    = (build_date[9] - 48)*10 + build_date[10] - 48;
	hours[0]   = (build_time[0] - 48)*10 + build_time[1]  - 48;
	minutes[0] = (build_time[3] - 48)*10 + build_time[4]  - 48;
	seconds[0] = (build_time[6] - 48)*10 + build_time[7]  - 48;
	RTC.setDay(day);                                                                                   // Set the date/time
	RTC.setMonth(month);
	RTC.setYear(year);
	RTC.setHours(hours[0]);
	RTC.setMinutes(minutes[0]);
	RTC.setSeconds(seconds[0]);
}
