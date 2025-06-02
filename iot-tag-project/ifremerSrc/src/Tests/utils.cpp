/* 09/01/2020 Pierre Gogendeau

*/

#include "../_board.h"

#include "utils.h"

void int32_t_float_to_bytes(float temp, uint8_t * dest)
{
  int32_t tempOut = temp * 10000000;
  //Serial.print(" tempOu fonction float to byte : "); Serial.println(tempOut);
  dest[0] = (tempOut & 0xFF000000) >> 24;
  dest[1] = (tempOut & 0x00FF0000) >> 16;
  dest[2] = (tempOut & 0x0000FF00) >> 8;
  dest[3] = (tempOut & 0x000000FF);
}

/**
    @brief  setRTC
    @param  None
    @retval None
*/

void printRTC(uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint8_t &year, uint8_t &month, uint8_t &day, uint32_t &subSeconds, uint32_t &milliseconds)
{
  Serial.println("RTC:");
  RTC.getDate(day, month, year);
  RTC.getTime(hours, minutes, seconds, subSeconds);
  milliseconds = ((subSeconds >> 17) * 1000 + 16384) / 32768;
  Serial.print("RTC Time = ");
  if (hours < 10)   {
    Serial.print("0");
    Serial.print(hours);
  } else Serial.print(hours);
  Serial.print(":");
  if (minutes < 10) {
    Serial.print("0");
    Serial.print(minutes);
  } else Serial.print(minutes);
  Serial.print(":");
  if (seconds < 10) {
    Serial.print("0");
    Serial.print(seconds);
  } else Serial.print(seconds);
  Serial.print(".");
  if (milliseconds <= 9) {
    Serial.print("0");
  }
  if (milliseconds <= 99) {
    Serial.print("0");
  }
  Serial.print(milliseconds);
  Serial.println(" ");

}

/**
    @brief  setRTC
    @param  None
    @retval None
*/

void setRTC(uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint8_t &year, uint8_t &month, uint8_t &day)      // Function sets the RTC to the FW build date-time...
{
  const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
  const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

  char Build_mo[3];
  String build_mo = "";

  Build_mo[0] = build_date[0];                                                                       // Convert month string to integer
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];
  for (uint8_t i = 0; i < 3; i++)
  {
    build_mo += Build_mo[i];
  }
  if (build_mo == "Jan")
  {
    month = 1;
  } else if (build_mo == "Feb")
  {
    month = 2;
  } else if (build_mo == "Mar")
  {
    month = 3;
  } else if (build_mo == "Apr")
  {
    month = 4;
  } else if (build_mo == "May")
  {
    month = 5;
  } else if (build_mo == "Jun")
  {
    month = 6;
  } else if (build_mo == "Jul")
  {
    month = 7;
  } else if (build_mo == "Aug")
  {
    month = 8;
  } else if (build_mo == "Sep")
  {
    month = 9;
  } else if (build_mo == "Oct")
  {
    month = 10;
  } else if (build_mo == "Nov")
  {
    month = 11;
  } else if (build_mo == "Dec")
  {
    month = 12;
  } else
  {
    month = 1;                                                                                       // Default to January if something goes wrong...
  }
  if (build_date[4] != 32)                                                                           // If the first digit of the date string is not a space
  {
    day   = (build_date[4] - 48) * 10 + build_date[5]  - 48;                                         // Convert ASCII strings to integers; ASCII "0" = 48
  } else
  {
    day   = build_date[5]  - 48;
  }
  year    = (build_date[9] - 48) * 10 + build_date[10] - 48;
  hours   = (build_time[0] - 48) * 10 + build_time[1]  - 48;
  minutes = (build_time[3] - 48) * 10 + build_time[4]  - 48;
  seconds = (build_time[6] - 48) * 10 + build_time[7]  - 48;
  RTC.setDay(day);                                                                                   // Set the date/time
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);
}

/**
    @brief  Blink led using delay of 300ms
    @param  None
    @retval None
*/

void blink_led(int nb_blink) {
  int i = 0;
  for (i = 0; i < nb_blink; i++) {
    digitalWrite(LED, LOW); // turn on led during device initialization
    delay(300);
    digitalWrite(LED, HIGH); // turn on led during device initialization
    delay(300);
  }
  digitalWrite(LED, LOW); // turn on led during device initialization
}

 /*void getRTC (uint8_t index)
  {
    RTC.getDate(day, month, year);
    RTC.getTime(hours[index], minutes[index], seconds[index], subSeconds[index]);
  }*/

  
