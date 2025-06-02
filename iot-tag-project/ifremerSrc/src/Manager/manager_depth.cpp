/*
 * manager_depth.cpp
 *
 *  Created on: 23 janv. 2020
 *      Author: jfezande
 */


#include "../_appDirectives.h"

#include "MS5837.h"
#include "MS5803.h"
#include "I2CDev.h"

#include "manager_depth.h"

#define OSR	ADC_256


extern I2Cdev             i2c_0;   // Instantiate the I2Cdev object and point to the desired I2C bus


uint16_t Pcal30BA[8];         // calibration constants from MS5837 PROM registers
uint16_t Pcal1BA[8];         // calibration constants from MS5803 PROM registers
MS5837 MS5837(&i2c_0);
MS5803 MS5803(&i2c_0);

const char* const sensorTypeStr[pressureSensor_size] = {"1BA", "30BA"};

int32_t offset30BA_mPa = 0;


void _calcTemp(uint32_t D2, uint16_t* Pcal, int32_t *pdT, int32_t *pTemp){
	 //int32_t dT = D2 - (((int32_t)Pcal[5])<<8);
	int32_t dT = Pcal[5];
	dT = dT <<8;
	dT = -dT;
	dT += D2;

	int64_t temp = dT;
	temp *= Pcal[6];
    temp = (temp>>23) + 2000;

	*pdT = dT;
	*pTemp = (int32_t)temp;

}
	
	
	

void _calcPressure(uint32_t D1, uint16_t* Pcal, int32_t dT, int64_t *pPressure){

    //int64_t OFF = ( ((int64_t)Pcal[2])<<16) * (( ((int64_t)Pcal[4])*dT)>>7);
    int64_t OFF = Pcal[2];
	OFF = OFF<<16;
	
	int64_t OFF2 = Pcal[4];
	OFF2 *= dT;
	OFF2 = OFF2>>7;

	OFF += OFF2;
	
	//int64_t SENS = ( ((int64_t)Pcal[1]) <<15) + ((((int64_t)Pcal[3])*dT)>>8);
	int64_t SENS = Pcal[1];
	SENS = SENS << 15;
	int64_t SENS1 = Pcal[3];
	SENS1 *= dT;
	SENS1 = SENS1>>8;
	SENS += SENS1;
    
	SENS *= D1;
    SENS = SENS>>21;

    SENS -= OFF;

	*pPressure = SENS;
}


pressureSensor_result_t _readPressure_mPa_30BA(int32_t *pPressure, int32_t *pTemp){
	
	uint32_t D1, D2;
	
	D1 = MS5837.DataRead(ADC_D1, OSR);  // get raw pressure value
	D2 = MS5837.DataRead(ADC_D2, OSR);  // get raw temperature value
	
	if((D1 == 0) || (D2 == 0)) return psr_ERROR;

	int32_t temp, dT;
	int64_t pressure;
	
	_calcTemp(D2, Pcal30BA, &dT, &temp);
	_calcPressure(D1, Pcal30BA, dT, &pressure);
	
	//convert to mPa
	pressure *=10*1000; 
	pressure = pressure >> 13;
	
	//Serial.print("30BA : ");Serial.print((int32_t)pressure);Serial.println(" mPa");

	*pPressure = (int32_t)pressure;
	*pTemp = temp;
	return psr_OK;
}

pressureSensor_result_t _readPressure_mPa_1BA(int32_t *pPressure, int32_t *pTemp){
	
	uint32_t D1, D2;
	
	D1 = MS5803.DataRead(ADC_D1, OSR);  // get raw pressure value
	D2 = MS5803.DataRead(ADC_D2, OSR);  // get raw temperature value

	if((D1 == 0) || (D2 == 0)) return psr_ERROR;

	int32_t temp, dT;
	int64_t pressure;
	

	_calcTemp(D2, Pcal1BA, &dT, &temp);
	_calcPressure(D1, Pcal1BA, dT, &pressure);
	
	//convert to mPa
	pressure *=1000; 
	pressure = pressure >> 15;

	Serial.print("1BA : ");Serial.print((int32_t)pressure);Serial.println(" mPa");

	//limit of pressure read
	if(pressure > LIMIT_OF_1BA_SENSOR_mPA){
		*pPressure = 0;
		*pTemp = 0;
		LOGLN("overlimit");
		return psr_OVERLIMIT;
	}



	*pPressure = (int32_t)pressure;
	*pTemp = temp;
	return psr_OK;
//	return (int32_t)pressure;
}


bool manager_depth_init(void){

	I2C_BUS.begin();                                      // Set master mode, default on SDA/SCL for STM32L4
	delay(1000);
	I2C_BUS.setClock(400000);                             // I2C frequency at 400 kHz
	delay(1000);

	//#warning pkoi ??
	//i2c_0.I2Cscan();

	//reset sensor
	MS5837.Reset();
	MS5803.Reset();
	delay(200);
	MS5837.PromRead(Pcal30BA);
	delay(10);
	MS5803.PromRead(Pcal1BA);
	unsigned char refCRC = Pcal30BA[0] >> 12;

	unsigned char nCRC = MS5837.checkCRC(Pcal30BA);  //calculate checksum to ensure integrity of MS5837 calibration data

	if(nCRC != refCRC)
		Serial.println("Error");
	
	return true;
}



pressureSensor_result_t manager_depth_readPressure_mPa_1BA(int32_t *pValue){
	int32_t temp;
	return _readPressure_mPa_1BA(pValue,&temp);
}

pressureSensor_result_t manager_depth_readPressure_mPa_30BA(int32_t *pValue){
	int32_t temp;
	return _readPressure_mPa_30BA(pValue,&temp);
}

pressureSensor_result_t manager_depth_readPressure_mPa_Temp_1BA(int32_t *pPressure, int32_t *pTemp){

	return _readPressure_mPa_1BA(pPressure,pTemp);
}

pressureSensor_result_t manager_depth_readPressure_mPa_Temp_30BA(int32_t *pPressure, int32_t *pTemp){

	return _readPressure_mPa_30BA(pPressure,pTemp);
}


//return true if mesure is OK and in range
//
pressureSensor_result_t manager_depth_get_mm(pressureSensor_range_t range, int32_t *pValue){
	int32_t temp;

	return manager_depth_get_mm_temp(range, pValue, &temp);

}

int32_t manager_depth_convert_mPa2mm(int32_t value, int32_t surfacePressure){
	//From pressure in Pa
	// atmospheric pressure = 1013hPa = 101300 Pa
	//seawater density is 1029kg/m3
	//gravity is 9.80665m/s�

	//depth is : (Pa - atmoPressure) / (density * gravity )
	const int32_t denGrav_saltedWater = (int32_t)(9.80665 * 1029);

	// �quation lin�aire de calcul la variation dans l'air
	// 	a =-5588586 b =  566263523658885	(shift 26 bits )
	//const int64_t denGrav_air_a = -5588586LL;
	//const int64_t denGrav_air_b = 566263523658885LL;

	//voir document
	const int32_t airDiv = 12;

	int32_t pressure = value;
	//if(pressure < (101325*1000)){ //in air
	if(pressure < surfacePressure){ //in air

		//attention l'algo n'est valable que proche de la surface

		pressure = value - surfacePressure;
		pressure /= 12;

	}
	else{//in water
		pressure -= surfacePressure; //remove
		pressure /= denGrav_saltedWater;
	}


	return pressure;

}


pressureSensor_result_t manager_depth_get_mm_temp(pressureSensor_range_t range, int32_t *pPressure, int32_t *pTemp){

	//From pressure in Pa
	// atmospheric pressure = 1013hPa = 101300 Pa
	//seawater density is 1029kg/m3
	//gravity is 9.80665m/s�

	//depth is : (Pa - atmoPressure) / (density * gravity )
	const int32_t denGrav_saltedWater = (int32_t)(9.80665 * 1029);
	
	// �quation lin�aire de calcul la variation dans l'air
	// 	a =-5588586 b =  566263523658885	(shift 26 bits )
	const int64_t denGrav_air_a = -5588586LL;
	const int64_t denGrav_air_b = 566263523658885LL;
	

	int32_t pressure = 0;
	pressureSensor_result_t res = psr_ERROR;

	switch(range){
	case pressureSensor_1BA:
		res = _readPressure_mPa_1BA(&pressure,pTemp);
		break;
	case pressureSensor_30BA:
		res = _readPressure_mPa_30BA(&pressure,pTemp);
		pressure += offset30BA_mPa;
		break;
	}

	
	if(res != psr_OK) return res;

	if(pressure < (101325*1000)){ //in air
		Serial.println("Air");
		int64_t pressure64 = pressure;
		pressure64 *= denGrav_air_a;
		pressure64 += denGrav_air_b;
		
		pressure64 = pressure64>>26;
		pressure = -((int32_t)pressure64);
		
	}
	else{//in water
		Serial.println("Water");
		pressure -= (101325*1000); //remove
		pressure /= denGrav_saltedWater;
	
	}
	
	*pPressure = pressure;

	Serial.print("Sensor ");Serial.print(sensorTypeStr[range]);
	Serial.print(" : ");Serial.print(pressure);Serial.print(" mm, ");
	Serial.print(*pTemp);Serial.println(" �C");
	return res;
}


void manager_depth_calibrate30BA(void){
	int32_t pressure1BA, pressure30BA = 0, temp;
	pressureSensor_result_t res;

	res = _readPressure_mPa_1BA(&pressure1BA, &temp);
	if(res == psr_ERROR){
		return;
	}

	res = _readPressure_mPa_30BA(&pressure30BA, &temp);
	if(res == psr_ERROR){
		return;
	}

	offset30BA_mPa = pressure1BA - pressure30BA;
	LOG("Offset 30BA : ");LOG(offset30BA_mPa);LOGLN(" mPa");
}

int32_t manager_depth_getOffset30BA(void){
	return offset30BA_mPa;
}

