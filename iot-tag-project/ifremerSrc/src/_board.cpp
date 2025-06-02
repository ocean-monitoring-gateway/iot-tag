/*
 * _board.cpp
 *
 *  Created on: 28 févr. 2020
 *      Author: jfezande
 */


#include "Arduino.h"
#include "I2CDev.h"
#include "SPIFlash.h"
#include "LSM303AGR_M.h"
#include "LSM303AGR_A.h"

#include "_board.h"


I2Cdev		i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus
SPIFlash	SPIFlash(csPin);


LSM303AGR_M LSM303AGR_M(&i2c_0);
LSM303AGR_A LSM303AGR_A(&i2c_0); // instantiate LSM303AGR accel class

