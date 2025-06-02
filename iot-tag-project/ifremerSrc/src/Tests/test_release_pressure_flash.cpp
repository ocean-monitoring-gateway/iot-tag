/*
 * test_release_pressure_flash.cpp
 *
 *  Created on: 7 juil. 2020
 *      Author: jnguyen
 */


#include "LoRaWAN.h"

#include "../_appDirectives.h"
#include "../LoraApplication/ifremerLoraMsg.h"

#include "../Manager/_manager.h"
#include "../_board.h"
#include "../Tools/shell.h"
#include "SPIFlash.h"
//#include "VEML6030.h"
#include "test_release_pressure_flash.h"

//---------------------------- DATALOG PARAMETERS --------------------------------------
#define pressure_sensor_sampling_time_r 100
//--------------------------------------------------------------------------------------

// SPI Flash: 64 MBit (8 MByte) SPI Flash 32,768, 256-byte pages
//page 0 and 1 reserved, accelero values start at page 2
uint16_t  max_last_page_r = 0x7FFF;  // lower it if you want to reserve some pages for other use...
uint16_t  page_number_r = 0;
uint16_t  last_page_r;
uint8_t   flash_Page_r[256];          // array to hold the data for flash page write
uint8_t   index_page_number_r = 0;         // compteur de triplets dans une page (max 42 = 252 octets)

TimerMillis pressure_sensor_timer_r;  // instantiate high-frequency timer

extern I2Cdev             i2c_0;
extern SPIFlash SPIFlash;

uint32_t UID_r[3] = {0, 0, 0};

const char        *build_date_r = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time_r = __TIME__;   // 8 characters HH:MM:SS
uint8_t hours_r = 12, minutes_r = 0, seconds_r = 0, year_r = 1, month_r = 1, day_r = 1;
uint32_t subSeconds_r, milliseconds_r;

// battery voltage monitor definitions
float VDDA_r, VBAT_r, VBUS_r, STM32L0Temp_r;

// Cricket pin assignments
#define my_VBAT_r_en  2 // enable VBat read
#define my_VBAT_r    A1 // VBat analog read pin

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G
      AODR_1Hz, AODR_10Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_400Hz
*/
uint8_t A_scale_r = AFS_2G, A_ODR_r = AODR_100Hz; // assuming normal mode operation

float a_Res_r;
float param_accel_bias_r[3] = {0.0f, 0.0f, 0.0f}; // offset biases copy for fonction call
int16_t accel_data_r[3];  // Stores the 10-bit signed accel output if normal mode
float acc_temperature_r;             // Stores the real internal gyro temperature in degrees Celsius
float x_accel_r, y_accel_r, z_accel_r;                   // variables to hold latest accel data values

//LSM303AGR_A LSM303AGR_A(&i2c_0); // instantiate LSM303AGR accel class
extern LSM303AGR_A LSM303AGR_A;

volatile bool newLSM303AGR_A_Data_r = false; // used for data ready interrupt handling
volatile bool newLSM303AGR_A_Activity_r  = false; // used for activity interrupt handling

volatile bool newPRESSURE_Data_r = false;

dataFlash_r_t data_r;

uint8_t neg_data_r = 0;


/**
    @brief  Main function of test_release_pressure_flash
    @param  None
    @retval None
 */

void test_release_pressure_flash() {

	delay(30000);
	Serial.begin(115200);
	delay(4000);

	uint8_t nb_byte_write_r = sizeof(data_r) + 7;
	uint8_t nb_sector_page_r = 254/nb_byte_write_r;

	Serial.print("Nb byte write : ");
	Serial.println(nb_byte_write_r);
	Serial.print("Nb secteur par page : ");
	Serial.println(nb_sector_page_r);

	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);  // start with blue led on (since active LOW)

	pinMode(my_VBAT_r_en, OUTPUT);
	digitalWrite(my_VBAT_r_en, LOW); // start with battery voltage monirot off
	pinMode(my_VBAT_r, INPUT);
	analogReadResolution(12);

	pinMode(LSM303AGR_A_intPin1_r, INPUT);
	pinMode(LSM303AGR_A_intPin2_r, INPUT);

	// Set the RTC time to firmware build time
//	manager_rtc_setDefaultTime();

	char Build_mo_r[3];
	//uint8_t hours = 12, minutes = 0, seconds = 0, year = 1, month = 1, day = 1;

	String build_mo_r = "";

	Build_mo_r[0] = build_date_r[0];    // Convert month string to integer
	Build_mo_r[1] = build_date_r[1];
	Build_mo_r[2] = build_date_r[2];
	for(uint8_t i=0; i<3; i++)
	{
		build_mo_r += Build_mo_r[i];
	}
	if(build_mo_r == "Jan")
	{
		month_r = 1;
	} else if(build_mo_r == "Feb")
	{
		month_r = 2;
	} else if(build_mo_r == "Mar")
	{
		month_r = 3;
	} else if(build_mo_r == "Apr")
	{
		month_r = 4;
	} else if(build_mo_r == "May")
	{
		month_r = 5;
	} else if(build_mo_r == "Jun")
	{
		month_r = 6;
	} else if(build_mo_r == "Jul")
	{
		month_r = 7;
	} else if(build_mo_r == "Aug")
	{
		month_r = 8;
	} else if(build_mo_r == "Sep")
	{
		month_r = 9;
	} else if(build_mo_r == "Oct")
	{
		month_r = 10;
	} else if(build_mo_r == "Nov")
	{
		month_r = 11;
	} else if(build_mo_r == "Dec")
	{
		month_r = 12;
	} else
	{
		month_r = 1;                                                                                       // Default to January if something goes wrong...
	}
	if(build_date_r[4] != 32)                                                                            // If the first digit of the date string is not a space
	{
		day_r   = (build_date_r[4] - 48)*10 + build_date_r[5]  - 48;                                           // Convert ASCII strings to integers; ASCII "0" = 48
	} else
	{
		day_r   = build_date_r[5]  - 48;
	}
	year_r    = (build_date_r[9] - 48)*10 + build_date_r[10] - 48;
	hours_r   = (build_time_r[0] - 48)*10 + build_time_r[1]  - 48;
	minutes_r = (build_time_r[3] - 48)*10 + build_time_r[4]  - 48;
	seconds_r = (build_time_r[6] - 48)*10 + build_time_r[7]  - 48;
	RTC.setDay(day_r);                                                                                   // Set the date/time
	RTC.setMonth(month_r);
	RTC.setYear(year_r);
	RTC.setHours(hours_r);
	RTC.setMinutes(minutes_r);
	RTC.setSeconds(seconds_r);


	Serial.println("Serial enabled!");
	Serial.println(day_r);
	Serial.println(month_r);
	Serial.println(year_r);
	Serial.println(hours_r);
	Serial.println(minutes_r);
	Serial.println(seconds_r);

	STM32L0.getUID(UID_r);
	Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID_r[0], HEX); Serial.print(UID_r[1], HEX); Serial.println(UID_r[2], HEX);

	VDDA_r = STM32L0.getVDDA();
	VBUS_r = STM32L0.getVBUS();
	digitalWrite(my_VBAT_r_en, HIGH);
	VBAT_r = 1.27f * VDDA_r * analogRead(my_VBAT_r) / 4096.0f;
	digitalWrite(my_VBAT_r_en, LOW);
	STM32L0Temp_r = STM32L0.getTemperature();

	// Internal STM32L0 functions
	Serial.print("VDDA = "); Serial.print(VDDA_r, 2); Serial.println(" V");
	Serial.print("VBAT = "); Serial.print(VBAT_r, 2); Serial.println(" V");
	if(VBUS_r ==  1)  Serial.println("USB Connected!");
	Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp_r, 2);
	Serial.println(" ");

	//SPI FLASH
	pinMode(csPin, OUTPUT); // set SPI chip select as L082 output
	digitalWrite(csPin, HIGH);

	// check SPI Flash ID
	SPIFlash.init();      // start SPI (include SPI.begin)
	SPIFlash.powerUp();   // MX25R6435FZAI defaults to power down state
	SPIFlash.getChipID(); // Verify SPI flash communication
	SPIFlash.flash_chip_erase(1);

	delay(15000);

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
/*	Serial.println("LSM303AGR_M mag...");
	byte d = LSM303AGR_M.getChipID();  // Read CHIP_ID register for LSM303AGR
	Serial.print("LSM303AGR_M "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x40, HEX);
	Serial.println(" ");
*/	delay(1000);

/*	param_data_r.A_ODR_r = AODR_100Hz;
	param_data_r.A_scale_r = AFS_2G;
	param_data_r.a_Res_r = LSM303AGR_A.getAres(param_data_r.A_scale_r); // get sensor resolution, only need to do this once
*/
	blink_led(3);
	delay(5000);
//	LSM303AGR_A.selfTest();
	LSM303AGR_A.reset();
	LSM303AGR_A.init(A_scale_r, A_ODR_r);

	LSM303AGR_A.offsetBias(param_accel_bias_r);

/*	param_data_r.accel_bias[0] = param_accel_bias_r[0];
	param_data_r.accel_bias[1] = param_accel_bias_r[1];
	param_data_r.accel_bias[2] = param_accel_bias_r[2];

	Serial.println("Accel biases (mg)");
	Serial.println(1000.0f * param_data_r.accel_bias[0]);
	Serial.println(1000.0f * param_data_r.accel_bias[1]);
	Serial.println(1000.0f * param_data_r.accel_bias[2]);
	Serial.println(" ");
*/
    attachInterrupt(LSM303AGR_A_intPin1_r, myinthandler_1_r, RISING);  // define data ready interrupt for intPin1 output of LSM303AGR
	attachInterrupt(LSM303AGR_A_intPin2_r, myinthandler_2_r, RISING);  // define no-motion activity interrupt for intPin2 output of LSM303AGR

    LSM303AGR_A.readAccData(accel_data_r); // INT1 cleared on any read
//    LSM303AGR_M.readData(mag_data);  // read data register to clear interrupt before main loop


    // PRESSURE
    int32_t depth_1 = 0;
    int32_t depth_30 = 0;
    manager_depth_init();
    manager_depth_get_mm(pressureSensor_1BA, &depth_1);
    manager_depth_get_mm(pressureSensor_30BA, &depth_30);
    manager_depth_readPressure_mPa_1BA(&depth_1);
    manager_depth_readPressure_mPa_30BA(&depth_30);
    data_r.depth_mm_1_r = depth_1;
    data_r.depth_mm_30_r = depth_30;
    pressure_sensor_timer_r.start(callback_pressureTimer_r, 0,  pressure_sensor_sampling_time_r);
    // END PRESSURE

//	Serial.println("Wait double tap");
/*
	while (newLSM303AGR_A_Activity_r == false);
	digitalWrite(LED_BUILTIN, LOW);
*/

	while (1) {
/*
		if (newLSM303AGR_A_Activity_r == true)
		{
			Serial.println("\r\nStarting DataLog...\r\n");
			//Serial.println("No motion activity detected!");
			newLSM303AGR_A_Activity_r = false;
			Serial.println("Double tap");
		}
*/
		// RTC
		RTC.getDate(day_r, month_r, year_r);
		RTC.getTime(hours_r, minutes_r, seconds_r, subSeconds_r);

		milliseconds_r = ((subSeconds_r >> 17) * 1000 + 16384) / 32768;

		Serial.print("RTC Time = ");
		if (hours_r < 10)
		{
			Serial.print("0");Serial.print(hours_r);
		}
		else
			Serial.print(hours_r);

		Serial.print(":");
		if (minutes_r < 10)
		{
			Serial.print("0"); Serial.print(minutes_r);
		}
		else
			Serial.print(minutes_r);

		Serial.print(":");
		if (seconds_r < 10)
		{
			Serial.print("0"); Serial.print(seconds_r);
		}
		else
			Serial.print(seconds_r);

		Serial.print(".");
		if (milliseconds_r <= 9)
		{
			Serial.print("0");
		}
		if (milliseconds_r <= 99)
		{
			Serial.print("0");
		}
		Serial.print(milliseconds_r);
		Serial.println(" ");

		Serial.print("RTC Date = ");
		Serial.print(day_r); Serial.print(":"); Serial.print(month_r); Serial.print(":20"); Serial.print(year_r);
		Serial.println(" ");
		// END RTC

		if(newPRESSURE_Data_r == true){
			newPRESSURE_Data_r = false;
			manager_depth_get_mm_temp(pressureSensor_1BA,&data_r.depth_mm_1_r,&data_r.temp_1_r);
			manager_depth_get_mm_temp(pressureSensor_30BA,&data_r.depth_mm_30_r,&data_r.temp_30_r);
			Serial.println(" ");
		}

			// 32,768 256-byte pages in a 8 MByte flash
			// 42 valeurs accelero -> 252 octets -> ~ 1 page
			//store RAW accel_data_r (3 x 2 octets)
			//mise en forme + bias au dump


		    // Send some data to the SPI flash
		    if (index_page_number_r < nb_sector_page_r && page_number_r < max_last_page_r) { // 32,768 256-byte pages in a 8 MByte flash
		    	Serial.println("entre ici");

		    	flash_Page_r[index_page_number_r * 23 + 0]  = day_r;
		    	flash_Page_r[index_page_number_r * 23 + 1]  = month_r;
		    	flash_Page_r[index_page_number_r * 23 + 2]  = year_r;
		    	flash_Page_r[index_page_number_r * 23 + 3]  = hours_r;
		    	flash_Page_r[index_page_number_r * 23 + 4]  = minutes_r;
		    	flash_Page_r[index_page_number_r * 23 + 5]  = seconds_r;


		    	if (data_r.depth_mm_1_r > 0) {
		    		flash_Page_r[index_page_number_r * 23 + 6]  = (data_r.depth_mm_1_r & 0xFF000000) >> 24;
		    		flash_Page_r[index_page_number_r * 23 + 7]  = (data_r.depth_mm_1_r & 0x00FF0000) >> 16;
			        flash_Page_r[index_page_number_r * 23 + 8]  = (data_r.depth_mm_1_r & 0x0000FF00) >> 8;
			        flash_Page_r[index_page_number_r * 23 + 9]  = (data_r.depth_mm_1_r & 0x000000FF);
		    		Serial.print("flash_Page_r depth_mm_1_r_MSB_1 : ");
		    		Serial.println(flash_Page_r[index_page_number_r * 23 + 6]);
		    		Serial.print("flash_Page_r depth_mm_1_r_MSB_2 : ");
		    		Serial.println(flash_Page_r[index_page_number_r * 23 + 7]);
		    		Serial.print("flash_Page_r depth_mm_1_r_LSB_1 : ");
		    		Serial.println(flash_Page_r[index_page_number_r * 23 + 8]);
		    		Serial.print("flash_Page_r depth_mm_1_r_LSB_2 : ");
		    		Serial.println(flash_Page_r[index_page_number_r * 23 + 9]);
		    		Serial.println(" ");
		    	}
		    	else {
		    		int32_t depth_mm_1_temp = - data_r.depth_mm_1_r;
		    		int32_t depth_mm_1_temp_MSB_1 = (depth_mm_1_temp & 0xFF000000);
		    		int32_t depth_mm_1_temp_MSB_2 = (depth_mm_1_temp & 0x00FF0000);
		    		int32_t depth_mm_1_temp_LSB_1 = (depth_mm_1_temp & 0x0000FF00);
		    		int32_t depth_mm_1_temp_LSB_2 = (depth_mm_1_temp & 0x000000FF);
			    	Serial.print("depth_mm_1_temp_MSB_1 : ");
			    	Serial.println(depth_mm_1_temp_MSB_1);
			    	Serial.print("depth_mm_1_temp_MSB_2 : ");
			    	Serial.println(depth_mm_1_temp_MSB_2);
			    	Serial.print("depth_mm_1_temp_LSB_1 : ");
			    	Serial.println(depth_mm_1_temp_LSB_1);
			    	Serial.print("depth_mm_1_temp_LSB_2 : ");
			    	Serial.println(depth_mm_1_temp_LSB_2);
			    	flash_Page_r[index_page_number_r * 23 + 6]  = depth_mm_1_temp_MSB_1 >> 24;
			    	flash_Page_r[index_page_number_r * 23 + 7]  = depth_mm_1_temp_MSB_2 >> 16;
			    	flash_Page_r[index_page_number_r * 23 + 8]  = depth_mm_1_temp_LSB_1 >> 8;
			    	flash_Page_r[index_page_number_r * 23 + 9]  = depth_mm_1_temp_LSB_2;
			    	neg_data_r = neg_data_r + 1;
		    		Serial.print("flash_Page_r depth_mm_1_MSB_1 : ");
		    		Serial.println(flash_Page_r[index_page_number_r * 23 + 6]);
		    		Serial.print("flash_Page_r depth_mm_1_MSB_2 : ");
		    		Serial.println(flash_Page_r[index_page_number_r * 23 + 7]);
		    		Serial.print("flash_Page_r depth_mm_1_LSB_1 : ");
		    		Serial.println(flash_Page_r[index_page_number_r * 23 + 8]);
		    		Serial.print("flash_Page_r depth_mm_1_LSB_2 : ");
		    		Serial.println(flash_Page_r[index_page_number_r * 23 + 9]);
		    		Serial.println(" ");
		     	}

		    	if (data_r.depth_mm_30_r > 0) {
		    		flash_Page_r[index_page_number_r * 23 + 10]  = (data_r.depth_mm_30_r & 0xFF000000) >> 24;
		    		flash_Page_r[index_page_number_r * 23 + 11]  = (data_r.depth_mm_30_r & 0x00FF0000) >> 16;
			        flash_Page_r[index_page_number_r * 23 + 12]  = (data_r.depth_mm_30_r & 0x0000FF00) >> 8;
			        flash_Page_r[index_page_number_r * 23 + 13]  = (data_r.depth_mm_30_r & 0x000000FF);
		    		Serial.print("flash_Page_r depth_mm_30_r_MSB_1 : ");
		    		Serial.println(flash_Page_r[index_page_number_r * 23 + 10]);
		    		Serial.print("flash_Page_r depth_mm_30_r_MSB_2 : ");
		    		Serial.println(flash_Page_r[index_page_number_r * 23 + 11]);
		    		Serial.print("flash_Page_r depth_mm_30_r_LSB_1 : ");
		    		Serial.println(flash_Page_r[index_page_number_r * 23 + 12]);
		    		Serial.print("flash_Page_r depth_mm_30_r_LSB_2 : ");
		    		Serial.println(flash_Page_r[index_page_number_r * 23 + 13]);
		    		Serial.println(" ");
		    	}
		    	else {
		    		int32_t depth_mm_30_temp = - data_r.depth_mm_30_r;
		    		int32_t depth_mm_30_temp_MSB_1 = (depth_mm_30_temp & 0xFF000000);
		    		int32_t depth_mm_30_temp_MSB_2 = (depth_mm_30_temp & 0x00FF0000);
		    		int32_t depth_mm_30_temp_LSB_1 = (depth_mm_30_temp & 0x0000FF00);
		    		int32_t depth_mm_30_temp_LSB_2 = (depth_mm_30_temp & 0x000000FF);
			    	Serial.print("depth_mm_30_r_temp_MSB_1 : ");
			    	Serial.println(depth_mm_30_temp_MSB_1);
			    	Serial.print("depth_mm_30_temp_MSB_2 : ");
			    	Serial.println(depth_mm_30_temp_MSB_2);
			    	Serial.print("depth_mm_30_temp_LSB_1 : ");
			    	Serial.println(depth_mm_30_temp_LSB_1);
			    	Serial.print("depth_mm_30_temp_LSB_2 : ");
			    	Serial.println(depth_mm_30_temp_LSB_2);
			    	flash_Page_r[index_page_number_r * 23 + 10]  = depth_mm_30_temp_MSB_1 >> 24;
			    	flash_Page_r[index_page_number_r * 23 + 11]  = depth_mm_30_temp_MSB_2 >> 16;
			    	flash_Page_r[index_page_number_r * 23 + 12]  = depth_mm_30_temp_LSB_1 >> 8;
			    	flash_Page_r[index_page_number_r * 23 + 13]  = depth_mm_30_temp_LSB_2;
			    	neg_data_r = neg_data_r + 2;
		    		Serial.print("flash_Page_r depth_mm_30_MSB_1 : ");
		    		Serial.println(flash_Page_r[index_page_number_r * 23 + 10]);
		    		Serial.print("flash_Page_r depth_mm_30_MSB_2 : ");
		    		Serial.println(flash_Page_r[index_page_number_r * 23 + 11]);
		    		Serial.print("flash_Page_r depth_mm_30_LSB_1 : ");
		    		Serial.println(flash_Page_r[index_page_number_r * 23 + 12]);
		    		Serial.print("flash_Page_r depth_mm_30_LSB_2 : ");
		    		Serial.println(flash_Page_r[index_page_number_r * 23 + 13]);
		    		Serial.println(" ");
		     	}

		    	flash_Page_r[index_page_number_r * 23 + 14]  = (data_r.temp_1_r & 0xFF000000) >> 24;
		    	flash_Page_r[index_page_number_r * 23 + 15]  = (data_r.temp_1_r & 0x00FF0000) >> 16;
		    	flash_Page_r[index_page_number_r * 23 + 16]  = (data_r.temp_1_r & 0x0000FF00) >> 8;
		    	flash_Page_r[index_page_number_r * 23 + 17]  = (data_r.temp_1_r & 0x000000FF);
		    	Serial.print("flash_Page_r temp_1_r_MSB_1 : ");
		    	Serial.println(flash_Page_r[index_page_number_r * 23 + 14]);
		    	Serial.print("flash_Page_r temp_1_r_MSB_2 : ");
		    	Serial.println(flash_Page_r[index_page_number_r * 23 + 15]);
		    	Serial.print("flash_Page_r temp_1_r_LSB_1 : ");
		    	Serial.println(flash_Page_r[index_page_number_r * 23 + 16]);
		    	Serial.print("flash_Page_r temp_1_r_LSB_2 : ");
		    	Serial.println(flash_Page_r[index_page_number_r * 23 + 17]);
		    	Serial.println(" ");

		    	flash_Page_r[index_page_number_r * 23 + 18]  = (data_r.temp_30_r & 0xFF000000) >> 24;
		    	flash_Page_r[index_page_number_r * 23 + 19]  = (data_r.temp_30_r & 0x00FF0000) >> 16;
		    	flash_Page_r[index_page_number_r * 23 + 20]  = (data_r.temp_30_r & 0x0000FF00) >> 8;
		    	flash_Page_r[index_page_number_r * 23 + 21]  = (data_r.temp_30_r & 0x000000FF);
		    	Serial.print("flash_Page_r temp_30_r_MSB_1 : ");
		    	Serial.println(flash_Page_r[index_page_number_r * 23 + 18]);
		    	Serial.print("flash_Page_r temp_30_r_MSB_2 : ");
		    	Serial.println(flash_Page_r[index_page_number_r * 23 + 19]);
		    	Serial.print("flash_Page_r temp_30_r_LSB_1 : ");
		    	Serial.println(flash_Page_r[index_page_number_r * 23 + 20]);
		    	Serial.print("flash_Page_r temp_30_r_LSB_2 : ");
		    	Serial.println(flash_Page_r[index_page_number_r * 23 + 21]);
		    	Serial.println(" ");

		    	flash_Page_r[index_page_number_r * 23 + 22] = neg_data_r;
	    		Serial.print("flash_Page_r sign_data : ");
	    		Serial.println(flash_Page_r[index_page_number_r * 23 + 22]);
	    		Serial.println(" ");

		    	index_page_number_r++;
		    	neg_data_r = 0;

		    	Serial.print("index_page_number_r : ");
		    	Serial.println(index_page_number_r);
		    	Serial.print("page_number_r : ");
		    	Serial.println(page_number_r);
		    	Serial.println(" ");
		    	delay(1000);
		    }
		    else if (index_page_number_r == nb_sector_page_r && page_number_r < max_last_page_r) {// on ecrit la page
	   			//Serial.println("entre ici 3");
	   			digitalWrite(LED, LOW); //led on
	            //SPI.begin();           // When exiting STOP mode, re-enable the SPI peripheral
	            //SPIFlash.powerUp();
	   			SPIFlash.flash_page_program(flash_Page_r, page_number_r);  // Temps d'ecriture mesure : 1 ms
	   			Serial.print("***Wrote flash page: "); Serial.println(page_number_r); Serial.println(" ");
	   			index_page_number_r = 0;
	   			page_number_r++;
	   			digitalWrite(LED, HIGH); //led off
	            //SPIFlash.powerDown();  // Put SPI flash into power down mode
	            //SPI.end();             // End SPI peripheral to save power in STOP mode
	   		}
		    else {
		    	Serial.println("Reached last page of SPI flash!");
		    	Serial.println("Data logging stopped!");
		    	Serial.println(" ");
		    }
		}
	}


void callback_pressureTimer_r() {
	newPRESSURE_Data_r = true;
}

/**
    @brief  interrupt handler for the accelerometer interruption 1
    @param  None
    @retval None
 */

void myinthandler_1_r() {
	newLSM303AGR_A_Data_r = true;
	STM32L0.wakeup();
}

/**
    @brief  interrupt handler for the accelerometer interruption 2
    @param  None
    @retval None
 */

void myinthandler_2_r() {
	newLSM303AGR_A_Activity_r = true;
	STM32L0.wakeup();
}
