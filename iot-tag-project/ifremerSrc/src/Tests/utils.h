/* 09/01/2020 Pierre Gogendeau  

*/


#ifndef UTILS_H_
#define UTILS_H_


#include <STM32L0.h>
#include "RTC.h"

void int32_t_float_to_bytes(float temp, uint8_t * dest);
void printRTC(uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint8_t &year, uint8_t &month, uint8_t &day, uint32_t &subSeconds, uint32_t &milliseconds);
void setRTC(uint8_t &hours, uint8_t &minutes, uint8_t &seconds, uint8_t &year, uint8_t &month, uint8_t &day);
void getRTC (uint8_t index);
void blink_led(int nb_blink) ;


#endif
