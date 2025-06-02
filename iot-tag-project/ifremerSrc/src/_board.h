/*
 * _board.h
 *
 *  Created on: 18 févr. 2020
 *      Author: jfezande
 *
 *      Add all board definitons
 */

#ifndef SRC__BOARD_H_
#define SRC__BOARD_H_

#include "Arduino.h"
#include "I2CDev.h"
#include "SPIFlash.h"
#include "LSM303AGR_M.h"
#include "LSM303AGR_A.h"


#define LED  10 // blue led
// SPI Flash definitions
#define csPin 25 // SPI Flash chip select pin


#define sensorPin  A3 // Analog pin for surface detection
#define switchPin  9

//LSM303AGR accel definitions
#define LSM303AGR_A_intPin1 A4  // interrupt1 pin definitions, data ready
#define LSM303AGR_A_intPin2 3   // interrupt2 pin definitions, significant motion
//LIS2MDL magnetometer definitions
#define LSM303AGR_M_intPin  A2 // interrupt for magnetometer data ready


//GNSS definition
// MAX M8Q GNSS configuration
#define GNSS_en      5     // enable for GNSS 3.0 V LDO
#define pps          4     // 1 Hz fix pulse
#define GNSS_backup A0     // RTC backup for MAX M8Q




#define safeWhile(condition,timeout,code)	do{int16_t whileCount = timeout;while((whileCount-- >0) && (condition)){code;}}while(0);

/*extern I2Cdev		i2c_0;
extern SPIFlash	SPIFlash;


extern LSM303AGR_M LSM303AGR_M;
extern LSM303AGR_A LSM303AGR_A;
*/


#endif /* SRC__BOARD_H_ */
