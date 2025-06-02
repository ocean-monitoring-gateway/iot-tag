/*
 * test_accelero.cpp
 *
 *  Created on: 15 avr. 2020
 *      Author: jnguyen
 */

#include "LoRaWAN.h"

#include "../_appDirectives.h"
#include "../LoraApplication/ifremerLoraMsg.h"

#include "../Manager/_manager.h"
#include "../_board.h"
#include "SPIFlash.h"
#include "LSM303AGR_M.h"
#include "LSM303AGR_A.h"
#include "test_accelero.h"

//---------------------------- DATALOG PARAMETERS --------------------------------------
#define pressure_sensor_sampling_time_test 100
//--------------------------------------------------------------------------------------

// SPI Flash: 64 MBit (8 MByte) SPI Flash 32,768, 256-byte pages
//page 0 and 1 reserved, accelero values start at page 2)
uint16_t  max_last_page_test = 0x7FFF;  // lower it if you want to reserve some pages for other use...
uint32_t  force_last_page_test;         // force last usable page number (max 32767 or 0x7FFF) for debug (no stop button), min = 3 (2 config + 1 values). <3 => disable
uint16_t  page_number_test;
uint16_t  last_page_test;
uint8_t   flash_Page_test[256];          // array to hold the data for flash page write
uint8_t   index_page_number_test=0;         // compteur de triplets dans une page (max 42 = 252 octets)
//uint16_t  i;
uint8_t index_first_param_test = 2;

TimerMillis pressure_sensor_timer_test;  // instantiate high-frequency timer

extern I2Cdev             i2c_0;
extern SPIFlash SPIFlash;

uint32_t UID_test[3] = {0, 0, 0};
char buffer_test[32];

//bool SerialDebug = true;

uint8_t hours_test[4] = {12,12,12,12}, minutes_test[4] = {0,0,0,0}, seconds_test[4] = {0,0,0,0}, year_test = 1, month_test = 1, day_test = 1;
uint32_t subSeconds_test[4], milliseconds_test;

// battery voltage monitor definitions
float VDDA_test, VBAT_test, VBUS_test, STM32L0Temp_test;

// Cricket pin assignments
#define my_VBAT_en_test  2 // enable VBat read
#define my_VBAT_test    A1 // VBat analog read pin

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G
      AODR_1Hz, AODR_10Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_400Hz
*/
uint8_t A_scale_test = AFS_2G, A_ODR_test = AODR_100Hz; // assuming normal mode operation

float a_Res_test;
float param_accel_bias_test[3] = {0.0f, 0.0f, 0.0f}; // offset biases copy for fonction call
int16_t accel_data_test[3], acc_temp_data_test;  // Stores the 10-bit signed accel output if normal mode
float acc_temperature_test;             // Stores the real internal gyro temperature in degrees Celsius
float x_accel_test, y_accel_test, z_accel_test;                   // variables to hold latest accel data values

//LSM303AGR_A LSM303AGR_A(&i2c_0); // instantiate LSM303AGR accel class
extern LSM303AGR_A LSM303AGR_A;

volatile bool newLSM303AGR_A_Data_test = false; // used for data ready interrupt handling
volatile bool newLSM303AGR_A_Activity_test  = false; // used for activity interrupt handling

/* Specify sensor parameters (sample rate is twice the bandwidth)
   choices are: MODR_10Hz, MOIDR_20Hz, MODR_50 Hz and MODR_100Hz
 */
uint8_t M_ODR_test = MODR_10Hz;

float m_Res_test = 0.0015f;            // mag
float param_mag_bias_test[3] = {0.0f, 0.0f, 0.0f}, param_mag_scale_test[3]  = {0.0f, 0.0f, 0.0f}; // Bias corrections for magnetometer
int16_t mag_data_test[4];              // Stores the 16-bit signed sensor output
float mag_temperature_test;            // Stores the real internal chip temperature in degrees Celsius
float x_mag_test, y_mag_test, z_mag_test;                // variables to hold latest mag data values
uint8_t LSM303AGR_M_Status_test;

volatile bool newLSM303AGR_M_Data_test = false; // LIS2MDL magnetometer interrupt flag

//LSM303AGR_M LSM303AGR_M(&i2c_0); // instantiate LSM303AGR_M class
extern LSM303AGR_M LSM303AGR_M;

volatile bool newPRESSURE_Data_test = false;

dataFlash_test_t data_test;
dataParamFlash_test_t param_data_test;

/**
    @brief  Main function of test_jellyfish_bis
    @param  None
    @retval None
 */

void test_accelero() {

	delay(4000);
	Serial.begin(115200);
	delay(4000);

	uint8_t nb_byte_write_test = sizeof(data_test);
	uint8_t nb_sector_page_test = 254/nb_byte_write_test;

	Serial.print("Nb secteur par page : ");
	Serial.println(nb_sector_page_test);

	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);  // start with blue led on (since active LOW)

	pinMode(my_VBAT_en_test, OUTPUT);
	digitalWrite(my_VBAT_en_test, LOW); // start with battery voltage monirot off
	pinMode(my_VBAT_test, INPUT);
	analogReadResolution(12);
	// set up Accelero
	pinMode(LSM303AGR_M_intPin_test, INPUT);    // set up interrupt pins
	pinMode(LSM303AGR_A_intPin1_test, INPUT);
	pinMode(LSM303AGR_A_intPin2_test, INPUT);

	// Set the RTC time to firmware build time
	manager_rtc_setDefaultTime();

	Serial.println("Serial enabled!");

	STM32L0.getUID(UID_test);
	Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID_test[0], HEX); Serial.print(UID_test[1], HEX); Serial.println(UID_test[2], HEX);

	VDDA_test = STM32L0.getVDDA();
	VBUS_test = STM32L0.getVBUS();
	digitalWrite(my_VBAT_en_test, HIGH);
	VBAT_test = 1.27f * VDDA_test * analogRead(my_VBAT_test) / 4096.0f;
	digitalWrite(my_VBAT_en_test, LOW);
	STM32L0Temp_test = STM32L0.getTemperature();

	// Internal STM32L0 functions
	Serial.print("VDDA = "); Serial.print(VDDA_test, 2); Serial.println(" V");
	Serial.print("VBAT = "); Serial.print(VBAT_test, 2); Serial.println(" V");
	if(VBUS_test ==  1)  Serial.println("USB Connected!");
	Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp_test, 2);
	Serial.println(" ");

	//SPI FLASH
	pinMode(csPin, OUTPUT); // set SPI chip select as L082 output
	digitalWrite(csPin, HIGH);

	// check SPI Flash ID
	SPIFlash.init();      // start SPI (include SPI.begin)
	SPIFlash.powerUp();   // MX25R6435FZAI defaults to power down state
	SPIFlash.getChipID(); // Verify SPI flash communication

	delay(15000);
/*
	//dump SpiFlash si usb connecté
	if(VBUS_test ==  1)
	{
		// Read max page, aRes and biases in SpiFlash page 0
		SPIFlash.flash_read_pages(flash_Page_test, 0, 1);

		last_page_test = (flash_Page_test[0] << 8) | flash_Page_test[1];
		if (last_page_test > max_last_page_test) //numero invalide ou flash vide (tout à 1), ou zone réservée
		{
			last_page_test = max_last_page_test;
		}

		data_param_flash_t *pParam_flash_read;

		pParam_flash_read = (data_param_flash_t*)(&flash_Page_test[index_first_param_test]);

		//printAscale();
		//printAODR();
		Serial.print("aRes "); Serial.println(pParam_flash_read->aRes, HEX);
		Serial.println("accel biases (mg)"); Serial.println(1000.0f * pParam_flash_read->accelBias[0]); Serial.println(1000.0f * pParam_flash_read->accelBias[1]); Serial.println(1000.0f * pParam_flash_read->accelBias[2]);
		Serial.println("mag biases "); Serial.println(1000.0f * pParam_flash_read->magBias[0]); Serial.println(1000.0f * pParam_flash_read->magBias[1]); Serial.println(1000.0f * pParam_flash_read->magBias[2]);
		Serial.println("mag scale "); Serial.println(1000.0f * pParam_flash_read->magScale[0]); Serial.println(1000.0f * pParam_flash_read->magScale[1]); Serial.println(1000.0f * pParam_flash_read->magScale[2]);


		// Read number of used pages in SpiFlash page 1
		SPIFlash.flash_read_pages(flash_Page_test, 1, 1);

		page_number_test = (flash_Page_test[0] << 8) | flash_Page_test[1];
		if ((page_number_test != 0xFFFF) && (page_number_test <= max_last_page_test)) //numero valide
		{
			last_page_test = page_number_test;
		}


		//Debug until button to stop recording
		force_last_page_test = ((uint32_t)logFreq * (uint32_t)logTime * 60) / nb_sector_page + index_first_param_test;
		if (force_last_page_test > 2)
		{
			if (force_last_page_test > 0x7FFF)
			{
				force_last_page_test = 0x7FFF;
			}
			last_page_test = (uint16_t)force_last_page_test;
			Serial.print("Last page force "); Serial.println(last_page_test); Serial.println("");
		}

		Serial.println("SpiFLASH dump start in 5s...\r\n");
		delay(5000);

		for (page_number_test = index_first_param_test; page_number_test < last_page_test; page_number_test++)
		{
			digitalWrite(LED, LOW); delay(1); digitalWrite(LED, HIGH);
			//Serial.print("Read Page 0x"); Serial.println(page_number_test, HEX);  //debug
			SPIFlash.flash_read_pages(flash_Page_test, page_number_test, 1);

			for (index_page_number_test = 0; index_page_number_test < nb_sector_page; index_page_number_test++)
			{

				data_flash_t *pData_flash_read;

				pData_flash_read = (data_flash_t*)(&flash_Page_test[index_page_number_test * 16]);

				/*x_accel_test = (float)pData_flash_read->accel_data_test[0] * pParam_flash_read->aRes - pParam_flash_read->accelBias[0];
				y_accel_test = (float)pData_flash_read->accel_data_test[1] * pParam_flash_read->aRes - pParam_flash_read->accelBias[1];
				z_accel_test = (float)pData_flash_read->accel_data_test[2] * pParam_flash_read->aRes - pParam_flash_read->accelBias[2];

				x_mag_test = (float)pData_flash_read->mag_data_test[0] * pParam_flash_read->mRes - pParam_flash_read->magBias[0]; // get actual G value
				x_mag_test = (float)pData_flash_read->mag_data_test[1] * pParam_flash_read->mRes - pParam_flash_read->magBias[1];
				x_mag_test = (float)pData_flash_read->mag_data_test[2] * pParam_flash_read->mRes - pParam_flash_read->magBias[2];
				x_mag_test *= pParam_flash_read->magScale[0];
				y_mag_test *= pParam_flash_read->magScale[1];
				z_mag_test *= pParam_flash_read->magScale[2];*/

/*
				Serial.print(pData_flash_read->accel_data_test[0]); Serial.print(";");
				Serial.print(pData_flash_read->accel_data_test[0]); Serial.print(";");
				Serial.print(pData_flash_read->accel_data_test[0]);
				Serial.print(" ; ");
				Serial.print(pData_flash_read->mag_data_test[0]); Serial.print(";");
				Serial.print(pData_flash_read->mag_data_test[1]); Serial.print(";");
				Serial.print(pdata_testflash_read->mag_data_test[2]);
				Serial.print(" ; ");
				Serial.println(pData_flash_read->depth_mm);
				Serial.print("\r\n");

			}
		}

		//Serial.print("Total used pages = ");Serial.print(page_number_test); Serial.println("\r\n");
		Serial.print("\r\nTotal Measurements : "); Serial.print(((uint32_t)page_number_test - 2) * 21);

		Serial.println("\r\nEnd of SpiFLASH dump");

		Serial.print("\r\nSpiFlash erase in 10s...");
		delay(10000);
		Serial.print("Erasing...");
		digitalWrite(LED, LOW); //led on
	} // fin dump si usb connecté
	else
	{
		last_page_test = max_last_page_test;

		//Debug until button to stop recording
		force_last_page_test = ((uint32_t)logFreq * (uint32_t)logTime * 60) / nb_sector_page + index_first_param_test;;
		if (force_last_page_test > 2)
		{
			if (force_last_page_test > 0x7FFF)
			{
				force_last_page_test = 0x7FFF;
			}
			last_page_test = (uint16_t)force_last_page_test;
			Serial.print("Last page force "); Serial.println(last_page_test); Serial.println("\r\n");
		}
	}

	if (VBUS_test == 1){

		eraseFlash(last_page_test);

	}

	page_number_test = index_first_param_test;      // reset first values page for main loop
	index_page_number_test = 0;  // reset first value number for main loop
*/
	I2C_BUS.begin();                                      // Set master mode, default on SDA/SCL for STM32L4
	delay(1000);
	I2C_BUS.setClock(400000);                             // I2C frequency at 400 kHz
	delay(1000);

	//Serial.println("Scan for I2C devices:");
	//i2c_0.I2Cscan();                                      // should detect MS5803 at 0x76, MS5837 at 0x77, VEML6030 at 0x10, LIS2MDL at 0x1E, and LSM303AGR at 0x19

	//Init LSM303AGR
	//initLSM303AGR();
	// Read the LSM303AGR Chip ID register, this is a good test of communication
	Serial.println("LSM303AGR accel/gyro...");
	byte c = LSM303AGR_A.getChipID();  // Read CHIP_ID register for LSM303AGR
	Serial.print("LSM303AGR "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x33, HEX);

	// Read the LSM303AGR_M Chip ID register, this is a good test of communication
	Serial.println("LSM303AGR_M mag...");
	byte d = LSM303AGR_M.getChipID();  // Read CHIP_ID register for LSM303AGR
	Serial.print("LSM303AGR_M "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x40, HEX);
	Serial.println(" ");
	delay(1000);

	param_data_test.A_ODR_test = AODR_100Hz;
	param_data_test.A_scale_test = AFS_2G;
	param_data_test.a_Res_test = LSM303AGR_A.getAres(param_data_test.A_scale_test); // get sensor resolution, only need to do this once

	blink_led(3);
	delay(5000);
	LSM303AGR_A.selfTest();
	LSM303AGR_A.reset();
	LSM303AGR_A.init(param_data_test.A_scale_test, param_data_test.A_ODR_test);

	LSM303AGR_A.offsetBias(param_accel_bias_test);

	param_data_test.accel_bias_test[0] = param_accel_bias_test[0];
	param_data_test.accel_bias_test[1] = param_accel_bias_test[1];
	param_data_test.accel_bias_test[2] = param_accel_bias_test[2];

	Serial.println("accel biases (mg)");
	Serial.println(1000.0f * param_data_test.accel_bias_test[0]);
	Serial.println(1000.0f * param_data_test.accel_bias_test[1]);
	Serial.println(1000.0f * param_data_test.accel_bias_test[2]);

	param_data_test.m_Res_test = 0.0015f;  // fixed sensitivity and full scale (+/- 49.152 Gauss);

	blink_led(5);
	delay(5000);
	LSM303AGR_M.selfTest();
	LSM303AGR_M.reset(); // software reset LIS2MDL to default registers
	LSM303AGR_M.init(param_data_test.M_ODR_test);

	LSM303AGR_M.offsetBias(param_mag_bias_test, param_mag_scale_test);

	param_data_test.mag_bias_test[0] = param_mag_bias_test[0];
    param_data_test.mag_bias_test[1] = param_mag_bias_test[1];
    param_data_test.mag_bias_test[2] = param_mag_bias_test[2];
    param_data_test.mag_scale_test[0] = param_mag_scale_test[0];
    param_data_test.mag_scale_test[1] = param_mag_scale_test[1];
    param_data_test.mag_scale_test[2] = param_mag_scale_test[2];

	Serial.println("mag biases (mG)"); Serial.println(1000.0f * param_data_test.mag_bias_test[0]); Serial.println(1000.0f * param_data_test.mag_bias_test[1]); Serial.println(1000.0f * param_data_test.mag_bias_test[2]);
	Serial.println("mag scale (mG)"); Serial.println(1000.0f * param_data_test.mag_scale_test[0]); Serial.println(1000.0f * param_data_test.mag_scale_test[1]); Serial.println(1000.0f * param_data_test.mag_scale_test[2]);
/*
	// Write default number of used pages, aRes and biases in SpiFlash page 0
	flash_Page_test[0]  = (max_last_page_test >> 8) & 0x00FF;  // MSB page max
	flash_Page_test[1]  = max_last_page_test & 0x00FF;         // LSB page max

	memcpy(&flash_Page_test[index_first_param_test], &param_data_test, sizeof(param_data_test));


	SPIFlash.flash_Page_test_program(flash_Page_test, 0); // write page zero
	//Serial.println("Page 0 written.");

	/*else
			{
				if(c != 0x33) Serial.println(" LSM303AGR_A not functioning!"); // otherwise there is a problem somewhere
				if(d != 0x40) Serial.println("LSM303AGR_M not functioning!");
				while(1){};
			}
	 */
/*
	digitalWrite(LED, HIGH); // turn off led when configuration successfully completed

	attachInterrupt(LSM303AGR_A_intPin2_, myinthandler_2, RISING);  // define double click interrupt for intPin2 output of LSM303AGR
	newLSM303AGR_A_Activity_test = false;
    int32_t depth_m = 0;
	manager_depth_init();
	manager_depth_get_mm(pressureSensor_1BA, &depth_m);
	data_test.depth_mm = depth_m;
	//Serial.println("Depth : ");
	//Serial.println(data_test.depth_mm);

	//get_RTC_test(0); //timestamp

	//Serial.println("Waiting tap");
*/

    attachInterrupt(LSM303AGR_A_intPin1_test, myinthandler_1_test, RISING);  // define data ready interrupt for intPin1 output of LSM303AGR
    attachInterrupt(LSM303AGR_A_intPin2_test, myinthandler_2_test, RISING);  // define no-motion activity interrupt for intPin2 output of LSM303AGR
    attachInterrupt(LSM303AGR_M_intPin_test , myinthandler_3_test, RISING);  // define data ready interrupt for intPin  output of LIS2MDL

    LSM303AGR_A.readAccData(accel_data_test); // INT1 cleared on any read
    LSM303AGR_M.readData(mag_data_test);  // read data register to clear interrupt before main loop

	while (1) {

		if (newLSM303AGR_A_Activity_test == true)
		{
			Serial.println("No motion activity detected!");
			newLSM303AGR_A_Activity_test = false;
		}

		if(newPRESSURE_Data_test == true){
			Serial.println("ici");
			newPRESSURE_Data_test = false;
			manager_depth_get_mm(pressureSensor_1BA,&data_test.depth_mm_1_test);
			manager_depth_get_mm(pressureSensor_30BA,&data_test.depth_mm_30_test);
			Serial.println("ici sortie");
		}


		if(newLSM303AGR_A_Data_test == true) // on interrupt, read data
		{
			digitalWrite(LED, HIGH);    // turn off led
			newLSM303AGR_A_Data_test = false;     // reset newData flag
			LSM303AGR_A.readAccData(data_test.accel_data_test); // INT1 cleared on any read
			// Now we'll calculate the accleration value into actual g's
			x_accel_test = (float)data_test.accel_data_test[0]*param_data_test.a_Res_test - param_data_test.accel_bias_test[0];  // get actual g value, this depends on scale being set
			y_accel_test = (float)data_test.accel_data_test[1]*param_data_test.a_Res_test - param_data_test.accel_bias_test[1];
			z_accel_test = (float)data_test.accel_data_test[2]*param_data_test.a_Res_test - param_data_test.accel_bias_test[2];

			//Serial.println(accel_data_test[0], HEX);
			//Serial.println(accel_data_test[1], HEX);
			//Serial.println(accel_data_test[2], HEX);
			Serial.print("ax = ");Serial.print((int)1000*x_accel_test); Serial.print(";");
			Serial.print("ay = ");Serial.print((int)1000*y_accel_test); Serial.print(";");
			Serial.print("az = ");Serial.println((int)1000*z_accel_test);
/*			acc_temp_data_test = LSM303AGR_A.readAccTempData();
			acc_temperature_test = ((float) acc_temp_data_test) + 25.0f; // Accel chip temperature in degrees Centigrade
			Serial.print("Accel temperature is ");  Serial.print(acc_temperature_test, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
*/
			//Serial.print(" Value_number="); Serial.println(index_page_number_test);  //debug
		    //Serial.print("\r\n");
		}

			if (newLSM303AGR_M_Data_test == true) {
				newLSM303AGR_M_Data_test = false;     // reset newData flag*/
				LSM303AGR_M_Status_test = LSM303AGR_M.status();

				if (LSM303AGR_M_Status_test & 0x08) // if all axes have new data ready
				{
					LSM303AGR_M.readData(data_test.mag_data_test);
					// Now we'll calculate the accleration value into actual G's
					x_mag_test = (float)data_test.mag_data_test[0] * param_data_test.m_Res_test - param_data_test.mag_bias_test[0]; // get actual G value
					y_mag_test = (float)data_test.mag_data_test[1] * param_data_test.m_Res_test - param_data_test.mag_bias_test[1];
					z_mag_test = (float)data_test.mag_data_test[2] * param_data_test.m_Res_test - param_data_test.mag_bias_test[2];
					x_mag_test *= param_data_test.mag_scale_test[0];
					y_mag_test *= param_data_test.mag_scale_test[1];
					z_mag_test *= param_data_test.mag_scale_test[2];
					Serial.print("mx = ");Serial.print((int)1000*x_mag_test); Serial.print(";");
					Serial.print("my = ");Serial.print((int)1000*y_mag_test); Serial.print(";");
					Serial.print("mz = ");Serial.println((int)1000*z_mag_test);
				}
			}

			// 32,768 256-byte pages in a 8 MByte flash
			// 42 valeurs accelero -> 252 octets -> ~ 1 page
			//store RAW accel_data_test (3 x 2 octets)
			//mise en forme + bias au dump

/*
			memcpy(&flash_Page_test[index_page_number_test*16], &data_, sizeof data_);


			if ( index_page_number_test < nb_sector_page && page_number_test < last_page_test)   // on remplit la page
			{
				index_page_number_test++;
			}
			else if ( index_page_number_test == nb_sector_page && page_number_test < last_page_test) // on Ã©crit la page
			{
				digitalWrite(LED, LOW); //led on
				SPIFlash.flash_Page_test_program(flash_Page_test, page_number_test);  // Temps d'Ã©criture mesurÃ© : 1 ms
				//Serial.print("***Wrote flash page: "); Serial.println(page_number_test);
				index_page_number_test = 0;
				page_number_test++;
				digitalWrite(LED, HIGH); //led off
			}
			else  // fin de log
			{
				//get_RTC_test(1); //timestamp

				detachInterrupt(LSM303AGR_A_intPin1_);  // detach interrupt to stay in stop mode
				detachInterrupt(LSM303AGR_A_intPin2_);  // detach interrupt to stay in stop mode
				detachInterrupt(LSM303AGR_M_intPin_);  // detach interrupt to stay in stop mode

				pressure_sensor_timer_test.stop();
				//LSM303AGR.reset();
				//LIS2MDL.reset();

				Serial.print("DataLog started : ");
				//print_RTC_test(0); //timestamp
				Serial.print("DataLog stopped : ");
				//print_RTC_test(1); //timestamp

				// record number of written pages in page 1
				flash_Page_test[0]  = (page_number_test >> 8) & 0x00FF;  // MSB derniÃ¨re page
				flash_Page_test[1]  = page_number_test & 0x00FF;         // LSB derniÃ¨re page

				SPIFlash.flash_Page_test_program(flash_Page_test, 1); // write page 1
				//Serial.println("Page 1 written.");
				//Serial.print("Total used pages = ");Serial.print(page_number_test); Serial.println("\r\n");
				Serial.print("\r\nTotal Measurements : "); Serial.print(((uint32_t)page_number_test - index_first_param_test) * nb_sector_page); Serial.println("\r\n");

				//Serial.print("Start Log: ");
				//print_RTC_test(0); //timestamp
				//Serial.print("End Log:   ");
				//print_RTC_test(1); //timestamp

				digitalWrite(LED, HIGH); // turn off led


			}
			Serial.print("index_page_number_test : ");
			Serial.println(index_page_number_test);

			/*SPIFlash.powerDown();  // Put SPI flash into power down mode
		SPI.end();             // End SPI peripheral to save power in STOP mode
		STM32L0.stop();        // Enter STOP mode and wait for an interrupt*/
		}
	}

void callback_pressureTimer_test() {
	newPRESSURE_Data_test = true;
}

/**
    @brief  interrupt handler for the accelerometer interruption 1
    @param  None
    @retval None
 */

void myinthandler_1_test()
{
	newLSM303AGR_A_Data_test = true;
	STM32L0.wakeup();
}

/**
    @brief  interrupt handler for the accelerometer interruption 2
    @param  None
    @retval None
 */

void myinthandler_2_test()
{

	newLSM303AGR_A_Activity_test = true;
	STM32L0.wakeup();
}

/**
    @brief  interrupt handler for the magnetometer interruption 1
    @param  None
    @retval None
 */

void myinthandler_3_test()
{
	newLSM303AGR_M_Data_test = true;
	STM32L0.wakeup();
}

void get_RTC_test (uint8_t index)
{
	RTC.getDate(day_test, month_test, year_test);
	RTC.getTime(hours_test[index], minutes_test[index], seconds_test[index], subSeconds_test[index]);
}

void print_RTC_test (uint8_t index)
{
	Serial.print(day_test); Serial.print(":"); Serial.print(month_test); Serial.print(":20"); Serial.print(year_test);
	Serial.print(" ");

	milliseconds_test = ((subSeconds_test[index] >> 17) * 1000 + 16384) / 32768;

	if (hours_test[index] < 10)
	{
		Serial.print("0");Serial.print(hours_test[index]);
	}
	else
		Serial.print(hours_test[index]);

	Serial.print(":");
	if (minutes_test[index] < 10)
	{
		Serial.print("0"); Serial.print(minutes_test[index]);
	}
	else
		Serial.print(minutes_test[index]);

	Serial.print(":");
	if (seconds_test[index] < 10)
	{
		Serial.print("0"); Serial.print(seconds_test[index]);
	}
	else
		Serial.print(seconds_test[index]);

	Serial.print(".");
	if (milliseconds_test <= 9)
	{
		Serial.print("0");
	}
	if (milliseconds_test <= 99)
	{
		Serial.print("0");
	}
	Serial.print(milliseconds_test);
	Serial.println(" ");
}
