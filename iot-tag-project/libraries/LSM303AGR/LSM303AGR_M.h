/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The LSM303AGR_M is a low power magnetometer, here used as 3 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#ifndef LSM303AGR_M_h
#define LSM303AGR_M_h

#include "Arduino.h"
#include <Wire.h>
#include "I2CDev.h"

//Register map for LSM303AGR_M'
// http://www.st.com/content/ccc/resource/technical/document/datasheet/group3/29/13/d1/e0/9a/4d/4f/30/DM00395193/files/DM00395193.pdf/jcr:content/translations/en.DM00395193.pdf

#define LSM303AGR_M_OFFSET_X_REG_L        0x45
#define LSM303AGR_M_OFFSET_X_REG_H        0x46
#define LSM303AGR_M_OFFSET_Y_REG_L        0x47
#define LSM303AGR_M_OFFSET_Y_REG_H        0x48
#define LSM303AGR_M_OFFSET_Z_REG_L        0x49
#define LSM303AGR_M_OFFSET_Z_REG_H        0x4A
#define LSM303AGR_M_WHO_AM_I              0x4F  // should be 0x40
#define LSM303AGR_M_CFG_REG_A             0x60
#define LSM303AGR_M_CFG_REG_B             0x61
#define LSM303AGR_M_CFG_REG_C             0x62
#define LSM303AGR_M_INT_CTRL_REG          0x63
#define LSM303AGR_M_INT_SOURCE_REG        0x64
#define LSM303AGR_M_INT_THS_L_REG         0x65
#define LSM303AGR_M_INT_THS_H_REG         0x66
#define LSM303AGR_M_STATUS_REG            0x67
#define LSM303AGR_M_OUTX_L_REG            0x68
#define LSM303AGR_M_OUTX_H_REG            0x69
#define LSM303AGR_M_OUTY_L_REG            0x6A
#define LSM303AGR_M_OUTY_H_REG            0x6B
#define LSM303AGR_M_OUTZ_L_REG            0x6C
#define LSM303AGR_M_OUTZ_H_REG            0x6D

#define LSM303AGR_M_ADDRESS               0x1E

#define MODR_10Hz   0x00
#define MODR_20Hz   0x01
#define MODR_50Hz   0x02
#define MODR_100Hz  0x03


class LSM303AGR_M
{
  public:
  LSM303AGR_M(I2Cdev* i2c_bus);
  uint8_t getChipID();
  void init(uint8_t MODR);
  void offsetBias(float * dest1, float * dest2);
  void reset();
  void selfTest();
  uint8_t status();
  void readData(int16_t * destination);
  private:
  float _mRes;
  I2Cdev* _i2c_bus;
};

#endif
