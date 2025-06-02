/*
 * test_jellyfish.cpp
 *
 *  Created on: 6 avr. 2020
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
#include "test_jellyfish.h"

//---------------------------- DATALOG PARAMETERS --------------------------------------
#define pressure_sensor_sampling_time_ 100
//--------------------------------------------------------------------------------------

// SPI Flash: 64 MBit (8 MByte) SPI Flash 32,768, 256-byte pages
//page 0 and 1 reserved, accelero values start at page 2
uint16_t  max_last_page_ = 0x7FFF;  // lower it if you want to reserve some pages for other use...
uint32_t  force_last_page_;         // force last usable page number (max 32767 or 0x7FFF) for debug (no stop button), min = 3 (2 config + 1 values). <3 => disable
uint16_t  page_number_ = 0;
uint16_t  last_page_;
uint8_t   flash_Page[256];          // array to hold the data for flash page write
uint8_t   index_page_number_ = 0;         // compteur de triplets dans une page (max 42 = 252 octets)
//uint16_t  i;
uint8_t index_first_param_ = 2;

TimerMillis pressure_sensor_timer_;  // instantiate high-frequency timer

extern I2Cdev             i2c_0;
extern SPIFlash SPIFlash;

uint32_t UID_[3] = {0, 0, 0};
char buffer_[32];

//bool SerialDebug = true;

const char        *build_date_jelly = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time_jelly = __TIME__;   // 8 characters HH:MM:SS
uint8_t hours_ = 12, minutes_ = 0, seconds_ = 0, year_ = 1, month_ = 1, day_ = 1;
uint32_t subSeconds_, milliseconds_;

// battery voltage monitor definitions
float VDDA_, VBAT_, VBUS_, STM32L0Temp_;

// Cricket pin assignments
#define my_VBAT_en  2 // enable VBat read
#define my_VBAT    A1 // VBat analog read pin

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G
      AODR_1Hz, AODR_10Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_400Hz
*/
uint8_t A_scale = AFS_2G, A_ODR = AODR_100Hz; // assuming normal mode operation

float a_Res;
float param_accel_bias[3] = {0.0f, 0.0f, 0.0f}; // offset biases copy for fonction call
int16_t accel_data[3], acc_temp_data;  // Stores the 10-bit signed accel output if normal mode
float acc_temperature;             // Stores the real internal gyro temperature in degrees Celsius
float x_accel, y_accel, z_accel;                   // variables to hold latest accel data values

//LSM303AGR_A LSM303AGR_A(&i2c_0); // instantiate LSM303AGR accel class
extern LSM303AGR_A LSM303AGR_A;

volatile bool newLSM303AGR_A_Data = false; // used for data ready interrupt handling
volatile bool newLSM303AGR_A_Activity  = false; // used for activity interrupt handling

/* Specify sensor parameters (sample rate is twice the bandwidth)
   choices are: MODR_10Hz, MOIDR_20Hz, MODR_50 Hz and MODR_100Hz
 */
uint8_t M_ODR = MODR_10Hz;

float m_Res = 0.0015f;            // mag
float param_mag_bias[3] = {0.0f, 0.0f, 0.0f}, param_mag_scale[3]  = {0.0f, 0.0f, 0.0f}; // Bias corrections for magnetometer
int16_t mag_data[4];              // Stores the 16-bit signed sensor output
float mag_temperature;            // Stores the real internal chip temperature in degrees Celsius
float x_mag, y_mag, z_mag;                // variables to hold latest mag data values
uint8_t LSM303AGR_M_Status;

volatile bool newLSM303AGR_M_Data = false; // LIS2MDL magnetometer interrupt flag

//LSM303AGR_M LSM303AGR_M(&i2c_0); // instantiate LSM303AGR_M class
extern LSM303AGR_M LSM303AGR_M;

volatile bool newPRESSURE_Data = false;
/*
//Choices are:
// IT: (IT_25  25 ms, IT_50  50 ms,) IT_100 = 100 ms, IT_200 = 200 ms, IT_400 = 400 ms, IT_800 = 800 ms
// Gain: Gain_1x 1x gain, Gain_2x 2x gain, Gain_0_125 1/8 x gain, Gain_0_25 1/4 x gain
// Persistance: 0x00 = 1, 0x01 = 2, 0x02 = 4, 0x03 = 8 // Num,ber of times a threshold must be crossed to trigger an interrupt
// Power save Mode = 0x00, 0x01, 0x02, 0x03 higher the mode, longer time between data but lower the current usage
uint8_t IT = IT_100, Gain = Gain_1x, Persistance = 0x00, powerMode = 0x00;  // configuration variable

uint16_t ALSData = 0, WhiteData = 0, IntStatus = 0;
// lux/LSBit, ambient light sensitivity increases with integration time
float Sensitivity = 0.0288f/((float) (1 << IT) ); // for IT = 100 ms, 200 ms, 400 ms or 800 ms only
float ambientLight, whiteLight;
volatile bool VEML6030_flag = false;

//extern VEML6030 VEML6030;
VEML6030 VEML6030(&i2c_0);
*/
dataFlash_t data_;
dataParamFlash_t param_data_;

uint8_t neg_data = 0;

GNSSLocation myPosition;
GNSSSatellites mySatellites;
TimerMillis timer_;

#define TICK_MAIN	1000
#define RESTART_DELAY_	60000

bool run_GNSS = false;
volatile bool flag_tick = false;
/*
// Surface sensor
static appMode_t application_Mode = app_boot;

#define SERIAL_RESET_DELAY_	(1800L)
static int16_t count_Tick = SERIAL_RESET_DELAY_;

static uint8_t dive_Zone = 0;

data_histo_t mainMessage_;

bool firstSubsurface = true;

TimerMillis mainTimer;
const uint32_t mainTimer_period = 1000;
void appTickCallback(void);
volatile bool flagTickMain_ = false;

TimerMillis surfaceTimer_;
const uint32_t surfaceTimer_period = deltaT_surface_detect;
void surfaceTickCallback(void);
volatile bool flagTickSurface_ = false;

uint32_t tsSurfacingTime_; //timestamp of the surfacing time
uint32_t tsBackToSurfaceTime_; //timestamp when we go back to the surface
uint32_t tsSubsurfacingTime_; //timestamp enter in subsurface
uint32_t tsDiveStartTime_; //timestamp enter in subsurface

static float last_EHPE = 10000;

//application processes jump Table (constant array of function pointer)
void (* const appProcess[app_size])(void) = {
	appProcess_boot, // [tagApp_unknow] = _process_unknow,
	appProcess_surface, //[tagApp_surface] = _process_surface,
	appProcess_subsurface, //[tagApp_subsurface] = _process_subsurface,
	appProcess_surfacing, //[tagApp_surfacing] = _process_surfacing,
	appProcess_dive //[tagApp_dive] = _process_dive
};

const char *const TriggerStr_[4] = { "THR", "GRAD", "1BA", "30BA"};

const char* const AppModeStr[app_size] = { "Boot", "Surface", "Subsurface", "Surfacing", "Dive"};

#define SURFACE_TRIGGER_SENSOR_THRESOLD_ 	0
#define SURFACE_TRIGGER_SENSOR_GRADIENT_	1
#define SURFACE_TRIGGER_PRESSURE_1BA_		2
#define SURFACE_TRIGGER_PRESSURE_30BA_ 		3
*/
/**
    @brief  Main function of test_jellyfish
    @param  None
    @retval None
 */

void test_jellyfish() {

	delay(4000);
	Serial.begin(115200);
	delay(4000);

	uint8_t nb_byte_write = sizeof(data_) + 7;
	uint8_t nb_sector_page = 254/nb_byte_write;

	Serial.print("Nb byte write : ");
	Serial.println(nb_byte_write);
	Serial.print("Nb secteur par page : ");
	Serial.println(nb_sector_page);

	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);  // start with blue led on (since active LOW)

	pinMode(my_VBAT_en, OUTPUT);
	digitalWrite(my_VBAT_en, LOW); // start with battery voltage monirot off
	pinMode(my_VBAT, INPUT);
	analogReadResolution(12);
	// set up Accelero
	pinMode(LSM303AGR_M_intPin_, INPUT);    // set up interrupt pins
	pinMode(LSM303AGR_A_intPin1_, INPUT);
	pinMode(LSM303AGR_A_intPin2_, INPUT);

	// Set the RTC time to firmware build time
//	manager_rtc_setDefaultTime();

	char Build_mo_jelly[3];
	//uint8_t hours = 12, minutes = 0, seconds = 0, year = 1, month = 1, day = 1;

	String build_mo_jelly = "";

	Build_mo_jelly[0] = build_date_jelly[0];    // Convert month string to integer
	Build_mo_jelly[1] = build_date_jelly[1];
	Build_mo_jelly[2] = build_date_jelly[2];
	for(uint8_t i=0; i<3; i++)
	{
		build_mo_jelly += Build_mo_jelly[i];
	}
	if(build_mo_jelly == "Jan")
	{
		month_ = 1;
	} else if(build_mo_jelly == "Feb")
	{
		month_ = 2;
	} else if(build_mo_jelly == "Mar")
	{
		month_ = 3;
	} else if(build_mo_jelly == "Apr")
	{
		month_ = 4;
	} else if(build_mo_jelly == "May")
	{
		month_ = 5;
	} else if(build_mo_jelly == "Jun")
	{
		month_ = 6;
	} else if(build_mo_jelly == "Jul")
	{
		month_ = 7;
	} else if(build_mo_jelly == "Aug")
	{
		month_ = 8;
	} else if(build_mo_jelly == "Sep")
	{
		month_ = 9;
	} else if(build_mo_jelly == "Oct")
	{
		month_ = 10;
	} else if(build_mo_jelly == "Nov")
	{
		month_ = 11;
	} else if(build_mo_jelly == "Dec")
	{
		month_ = 12;
	} else
	{
		month_ = 1;                                                                                       // Default to January if something goes wrong...
	}
	if(build_date_jelly[4] != 32)                                                                            // If the first digit of the date string is not a space
	{
		day_   = (build_date_jelly[4] - 48)*10 + build_date_jelly[5]  - 48;                                           // Convert ASCII strings to integers; ASCII "0" = 48
	} else
	{
		day_   = build_date_jelly[5]  - 48;
	}
	year_    = (build_date_jelly[9] - 48)*10 + build_date_jelly[10] - 48;
	hours_   = (build_time_jelly[0] - 48)*10 + build_time_jelly[1]  - 48;
	minutes_ = (build_time_jelly[3] - 48)*10 + build_time_jelly[4]  - 48;
	seconds_ = (build_time_jelly[6] - 48)*10 + build_time_jelly[7]  - 48;
	RTC.setDay(day_);                                                                                   // Set the date/time
	RTC.setMonth(month_);
	RTC.setYear(year_);
	RTC.setHours(hours_);
	RTC.setMinutes(minutes_);
	RTC.setSeconds(seconds_);


	Serial.println("Serial enabled!");
	Serial.println(day_);
	Serial.println(month_);
	Serial.println(year_);
	Serial.println(hours_);
	Serial.println(minutes_);
	Serial.println(seconds_);

	STM32L0.getUID(UID_);
	Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID_[0], HEX); Serial.print(UID_[1], HEX); Serial.println(UID_[2], HEX);

	VDDA_ = STM32L0.getVDDA();
	VBUS_ = STM32L0.getVBUS();
	digitalWrite(my_VBAT_en, HIGH);
	VBAT_ = 1.27f * VDDA_ * analogRead(my_VBAT) / 4096.0f;
	digitalWrite(my_VBAT_en, LOW);
	STM32L0Temp_ = STM32L0.getTemperature();

	// Internal STM32L0 functions
	Serial.print("VDDA = "); Serial.print(VDDA_, 2); Serial.println(" V");
	Serial.print("VBAT = "); Serial.print(VBAT_, 2); Serial.println(" V");
	if(VBUS_ ==  1)  Serial.println("USB Connected!");
	Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp_, 2);
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
/*
	//dump SpiFlash si usb connecté
	if(VBUS_ ==  1)
	{
		// Read max page, aRes and biases in SpiFlash page 0
		SPIFlash.flash_read_pages(flash_Page, 0, 1);

		last_page_ = (flash_Page[0] << 8) | flash_Page[1];
		if (last_page_ > max_last_page_) //numero invalide ou flash vide (tout à 1), ou zone réservée
		{
			last_page_ = max_last_page_;
		}

		data_param_flash_t *pParam_flash_read;

		pParam_flash_read = (data_param_flash_t*)(&flash_Page[index_first_param_]);

		//printAscale();
		//printAODR();
		Serial.print("aRes "); Serial.println(pParam_flash_read->aRes, HEX);
		Serial.println("accel biases (mg)"); Serial.println(1000.0f * pParam_flash_read->accelBias[0]); Serial.println(1000.0f * pParam_flash_read->accelBias[1]); Serial.println(1000.0f * pParam_flash_read->accelBias[2]);
		Serial.println("mag biases "); Serial.println(1000.0f * pParam_flash_read->magBias[0]); Serial.println(1000.0f * pParam_flash_read->magBias[1]); Serial.println(1000.0f * pParam_flash_read->magBias[2]);
		Serial.println("mag scale "); Serial.println(1000.0f * pParam_flash_read->magScale[0]); Serial.println(1000.0f * pParam_flash_read->magScale[1]); Serial.println(1000.0f * pParam_flash_read->magScale[2]);


		// Read number of used pages in SpiFlash page 1
		SPIFlash.flash_read_pages(flash_Page, 1, 1);

		page_number_ = (flash_Page[0] << 8) | flash_Page[1];
		if ((page_number_ != 0xFFFF) && (page_number_ <= max_last_page_)) //numero valide
		{
			last_page_ = page_number_;
		}


		//Debug until button to stop recording
		force_last_page_ = ((uint32_t)logFreq * (uint32_t)logTime * 60) / nb_sector_page + index_first_param_;
		if (force_last_page_ > 2)
		{
			if (force_last_page_ > 0x7FFF)
			{
				force_last_page_ = 0x7FFF;
			}
			last_page_ = (uint16_t)force_last_page_;
			Serial.print("Last page force "); Serial.println(last_page_); Serial.println("");
		}

		Serial.println("SpiFLASH dump start in 5s...\r\n");
		delay(5000);

		for (page_number_ = index_first_param_; page_number_ < last_page_; page_number_++)
		{
			digitalWrite(LED, LOW); delay(1); digitalWrite(LED, HIGH);
			//Serial.print("Read Page 0x"); Serial.println(page_number_, HEX);  //debug
			SPIFlash.flash_read_pages(flash_Page, page_number_, 1);

			for (index_page_number_ = 0; index_page_number_ < nb_sector_page; index_page_number_++)
			{

				data_flash_t *pData_flash_read;

				pData_flash_read = (data_flash_t*)(&flash_Page[index_page_number_ * 16]);

				/*x_accel = (float)pData_flash_read->accel_data[0] * pParam_flash_read->aRes - pParam_flash_read->accelBias[0];
				y_accel = (float)pData_flash_read->accel_data[1] * pParam_flash_read->aRes - pParam_flash_read->accelBias[1];
				z_accel = (float)pData_flash_read->accel_data[2] * pParam_flash_read->aRes - pParam_flash_read->accelBias[2];

				x_mag = (float)pData_flash_read->mag_data[0] * pParam_flash_read->mRes - pParam_flash_read->magBias[0]; // get actual G value
				x_mag = (float)pData_flash_read->mag_data[1] * pParam_flash_read->mRes - pParam_flash_read->magBias[1];
				x_mag = (float)pData_flash_read->mag_data[2] * pParam_flash_read->mRes - pParam_flash_read->magBias[2];
				x_mag *= pParam_flash_read->magScale[0];
				y_mag *= pParam_flash_read->magScale[1];
				z_mag *= pParam_flash_read->magScale[2];*/

/*
				Serial.print(pData_flash_read->accel_data[0]); Serial.print(";");
				Serial.print(pData_flash_read->accel_data[0]); Serial.print(";");
				Serial.print(pData_flash_read->accel_data[0]);
				Serial.print(" ; ");
				Serial.print(pData_flash_read->mag_data[0]); Serial.print(";");
				Serial.print(pData_flash_read->mag_data[1]); Serial.print(";");
				Serial.print(pData_flash_read->mag_data[2]);
				Serial.print(" ; ");
				Serial.println(pData_flash_read->depth_mm);
				Serial.print("\r\n");

			}
		}

		//Serial.print("Total used pages = ");Serial.print(page_number_); Serial.println("\r\n");
		Serial.print("\r\nTotal Measurements : "); Serial.print(((uint32_t)page_number_ - 2) * 21);

		Serial.println("\r\nEnd of SpiFLASH dump");

		Serial.print("\r\nSpiFlash erase in 10s...");
		delay(10000);
		Serial.print("Erasing...");
		digitalWrite(LED, LOW); //led on
	} // fin dump si usb connecté
	else
	{
		last_page_ = max_last_page_;

		//Debug until button to stop recording
		force_last_page_ = ((uint32_t)logFreq * (uint32_t)logTime * 60) / nb_sector_page + index_first_param_;;
		if (force_last_page_ > 2)
		{
			if (force_last_page_ > 0x7FFF)
			{
				force_last_page_ = 0x7FFF;
			}
			last_page_ = (uint16_t)force_last_page_;
			Serial.print("Last page force "); Serial.println(last_page_); Serial.println("\r\n");
		}
	}

	if (VBUS_ == 1){

		eraseFlash(last_page_);

	}

	page_number_ = index_first_param_;      // reset first values page for main loop
	index_page_number_ = 0;  // reset first value number for main loop
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

	param_data_.A_ODR = AODR_100Hz;
	param_data_.A_scale = AFS_2G;
	param_data_.a_Res = LSM303AGR_A.getAres(param_data_.A_scale); // get sensor resolution, only need to do this once

	blink_led(3);
	delay(5000);
	LSM303AGR_A.selfTest();
	LSM303AGR_A.reset();
	LSM303AGR_A.init(param_data_.A_scale, param_data_.A_ODR);

	LSM303AGR_A.offsetBias(param_accel_bias);

	param_data_.accel_bias[0] = param_accel_bias[0];
	param_data_.accel_bias[1] = param_accel_bias[1];
	param_data_.accel_bias[2] = param_accel_bias[2];

	Serial.println("Accel biases (mg)");
	Serial.println(1000.0f * param_data_.accel_bias[0]);
	Serial.println(1000.0f * param_data_.accel_bias[1]);
	Serial.println(1000.0f * param_data_.accel_bias[2]);
	Serial.println(" ");

	param_data_.m_Res = 0.0015f;  // fixed sensitivity and full scale (+/- 49.152 Gauss);

	blink_led(5);
	delay(5000);
	LSM303AGR_M.selfTest();
	LSM303AGR_M.reset(); // software reset LIS2MDL to default registers
	LSM303AGR_M.init(param_data_.M_ODR);

	LSM303AGR_M.offsetBias(param_mag_bias, param_mag_scale);

	param_data_.mag_bias[0] = param_mag_bias[0];
    param_data_.mag_bias[1] = param_mag_bias[1];
    param_data_.mag_bias[2] = param_mag_bias[2];
    param_data_.mag_scale[0] = param_mag_scale[0];
    param_data_.mag_scale[1] = param_mag_scale[1];
    param_data_.mag_scale[2] = param_mag_scale[2];

	Serial.println("Mag biases (mG)"); Serial.println(1000.0f * param_data_.mag_bias[0]); Serial.println(1000.0f * param_data_.mag_bias[1]); Serial.println(1000.0f * param_data_.mag_bias[2]);
	Serial.println("Mag scale (mG)"); Serial.println(1000.0f * param_data_.mag_scale[0]); Serial.println(1000.0f * param_data_.mag_scale[1]); Serial.println(1000.0f * param_data_.mag_scale[2]);
	Serial.println(" ");
/*
	// Write default number of used pages, aRes and biases in SpiFlash page 0
	flash_Page[0]  = (max_last_page_ >> 8) & 0x00FF;  // MSB page max
	flash_Page[1]  = max_last_page_ & 0x00FF;         // LSB page max

	memcpy(&flash_Page[index_first_param_], &param_data_, sizeof(param_data_));


	SPIFlash.flash_page_program(flash_Page, 0); // write page zero
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
	newLSM303AGR_A_Activity = false;
    int32_t depth_m = 0;
	manager_depth_init();
	manager_depth_get_mm(pressureSensor_1BA, &depth_m);
	data_.depth_mm = depth_m;
	//Serial.println("Depth : ");
	//Serial.println(data_.depth_mm);

	//get_RTC(0); //timestamp

	//Serial.println("Waiting tap");
*/

    attachInterrupt(LSM303AGR_A_intPin1_, myinthandler_1, RISING);  // define data ready interrupt for intPin1 output of LSM303AGR
    attachInterrupt(LSM303AGR_A_intPin2_, myinthandler_2, RISING);  // define no-motion activity interrupt for intPin2 output of LSM303AGR
    attachInterrupt(LSM303AGR_M_intPin_ , myinthandler_3, RISING);  // define data ready interrupt for intPin  output of LIS2MDL

    LSM303AGR_A.readAccData(accel_data); // INT1 cleared on any read
    LSM303AGR_M.readData(mag_data);  // read data register to clear interrupt before main loop

/*    VEML6030.init(IT, Gain, Persistance); // initialize the VEML6030 ALS
    VEML6030.enablepowerSave(powerMode);
    VEML6030.setHighThreshold(0x0400); // set high threshold to 1024/65,536
    VEML6030.setLowThreshold(0x0008);  // set  low threshold to    8/65,536
    uint16_t HiThrs = VEML6030.getHighThreshold();
    uint16_t LoThrs = VEML6030.getLowThreshold();
    Serial.print("High Threshold is : 0x"); Serial.println(HiThrs, HEX);
    Serial.print("Lo Threshold is : 0x"); Serial.println(LoThrs, HEX);
    Serial.println(" ");

    // GNSS + LoRa init
    uint32_t start_time, fix_time;
    uint8_t first_fix = 0; //1 for first Fix
    data_TTFF_t ttff_;
    data_gnssTracking_jellyfish_t gnss_Point;

    const float EHPE_ = 200;

    manager_gnss_init();

    LoRa_configuration_connection();

    manager_gnss_setState(GNSS_ON);
    while(manager_gnss_getState() != GNSS_ON)
    	manager_gnss_processState();

    run_GNSS = true;
    start_time = RTC.getEpoch();

    LOGLN("Start Timer");
    timer_.start(callback_tick, TICK_MAIN, TICK_MAIN);
    // END GNSS + LoRA init
*/
    // PRESSURE
    int32_t depth_1 = 0;
    int32_t depth_30 = 0;
    manager_depth_init();
    manager_depth_get_mm(pressureSensor_1BA, &depth_1);
    manager_depth_get_mm(pressureSensor_30BA, &depth_30);
    manager_depth_readPressure_mPa_1BA(&depth_1);
    manager_depth_readPressure_mPa_30BA(&depth_30);
    data_.depth_mm_1 = depth_1;
    data_.depth_mm_30 = depth_30;
    pressure_sensor_timer_.start(callback_pressureTimer, 0,  pressure_sensor_sampling_time_);
    // END PRESSURE
/*
    // SURFACE
	manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_SURFACE);
	surface_init();
	// SURFACE

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
*/

	Serial.println("Wait double tap");

	while (newLSM303AGR_A_Activity == false);
	digitalWrite(LED_BUILTIN, LOW);

//	get_RTC(0);
//	print_RTC(0);

	while (1) {

		if (newLSM303AGR_A_Activity == true)
		{
			Serial.println("\r\nStarting DataLog...\r\n");
			//Serial.println("No motion activity detected!");
			newLSM303AGR_A_Activity = false;
			Serial.println("Double tap");
		}

		// RTC
		RTC.getDate(day_, month_, year_);
		RTC.getTime(hours_, minutes_, seconds_, subSeconds_);

		milliseconds_ = ((subSeconds_ >> 17) * 1000 + 16384) / 32768;

		Serial.print("RTC Time = ");
		if (hours_ < 10)
		{
			Serial.print("0");Serial.print(hours_);
		}
		else
			Serial.print(hours_);

		Serial.print(":");
		if (minutes_ < 10)
		{
			Serial.print("0"); Serial.print(minutes_);
		}
		else
			Serial.print(minutes_);

		Serial.print(":");
		if (seconds_ < 10)
		{
			Serial.print("0"); Serial.print(seconds_);
		}
		else
			Serial.print(seconds_);

		Serial.print(".");
		if (milliseconds_ <= 9)
		{
			Serial.print("0");
		}
		if (milliseconds_ <= 99)
		{
			Serial.print("0");
		}
		Serial.print(milliseconds_);
		Serial.println(" ");

		Serial.print("RTC Date = ");
		Serial.print(day_); Serial.print(":"); Serial.print(month_); Serial.print(":20"); Serial.print(year_);
		Serial.println(" ");
		// END RTC
/*
		surface_process();
		if (application_Mode == app_surface) {
			if(flag_tick) {
				flag_tick = false;
				LOG(".");

				if(!run_GNSS) {
					LOG("Starting GPS");
					manager_gnss_setState(GNSS_ON);
				    while(manager_gnss_getState() != GNSS_ON)
				    	manager_gnss_processState();

				    start_time = RTC.getEpoch();

				    run_GNSS = true;
				    timer_.restart(TICK_MAIN, TICK_MAIN);
					LOG("Launch Main Tick");
				}

				else {
					GNSS.location(myPosition);
					LOGLN("Try to fix");
					//manager_gnss_logSattelites();

					GNSS.satellites(mySatellites);

//					get_RTC(0);
//					print_RTC(0);

					LOG("Fix ? ");LOGLN(fixGPS());
					delay(1000);
					if (fixGPS()) {
						LOG("EHPE : ");
						LOG(myPosition.ehpe());
						LOGLN("");

					//	if(myPosition.ehpe() < EHPE_) {

							uint32_t ctime = RTC.getEpoch();

							gnss_Point.lat_jelly = myPosition.latitude();
							LOG("LAT : ");
							LOG(myPosition.latitude());
							LOGLN("");
							gnss_Point.lng_jelly = myPosition.longitude();
							LOG("LONG : ");
							LOG(myPosition.longitude());
							LOGLN("");
							gnss_Point.ehpe_jelly = myPosition.ehpe();
							gnss_Point.ttf_jelly = ctime - start_time;
							gnss_Point.nb_sat = mySatellites.count();
							LOG("NB SAT : ");
							Serial.println(gnss_Point.nb_sat);
							LOGLN("");
							LOGLN("Lora Send GNSS");
							LoRaWAN.sendPacket(loraPortNumber[ilp_GnssTracking], (uint8_t*)&gnss_Point, sizeof(data_gnssTracking_jellyfish_t));
							delay(1000);
							//stop the GPS
							/*manager_gnss_setState(GNSS_OFF);
							  while(manager_gnss_getState() != GNSS_OFF)
							    	manager_gnss_processState();

							  run_GNSS = false;
							  timer_.restart(RESTART_DELAY_, RESTART_DELAY_);

							  LOG("DELAY TIMER");*/
					//	}
/*					}
				}
			}
		}

		// VEML6030 threshold interrupt handling
		if(VEML6030_flag) {
			VEML6030_flag = false;
		    IntStatus = VEML6030.getIntStatus();
		    if(IntStatus & 0x8000) Serial.println("Low Threshold Crossed!");
		    if(IntStatus & 0x4000) Serial.println("High Threshold Crossed!");
		    Serial.println(" ");
		} /* end of VEML6030 interrupt handling */

	    // VEML6030 Data
/*	    ALSData = VEML6030.getALSData();
	    ambientLight = ((float)ALSData)*Sensitivity; // ALS in lux
	    WhiteData = VEML6030.getWhiteData();
	    whiteLight = ((float)WhiteData)*Sensitivity; // White light in lux
	    Serial.print("VEML6030 ALS : "); Serial.print(ambientLight, 2); Serial.println(" lux");
	    Serial.print("VEML6030 White : "); Serial.print(whiteLight, 2); Serial.println(" lux"); Serial.println(" ");
*/


		if(newPRESSURE_Data == true){
			newPRESSURE_Data = false;
			manager_depth_get_mm_temp(pressureSensor_1BA,&data_.depth_mm_1,&data_.temp_1);
			manager_depth_get_mm_temp(pressureSensor_30BA,&data_.depth_mm_30,&data_.temp_30);
			Serial.println(" ");
		}


		if(newLSM303AGR_A_Data == true) // on interrupt, read data
		{
			digitalWrite(LED, HIGH);    // turn off led
			newLSM303AGR_A_Data = false;     // reset newData flag
			LSM303AGR_A.readAccData(data_.accel_data); // INT1 cleared on any read
			// Now we'll calculate the accleration value into actual g's
			x_accel = (float)data_.accel_data[0]*param_data_.a_Res - param_data_.accel_bias[0];  // get actual g value, this depends on scale being set
			y_accel = (float)data_.accel_data[1]*param_data_.a_Res - param_data_.accel_bias[1];
			z_accel = (float)data_.accel_data[2]*param_data_.a_Res - param_data_.accel_bias[2];

			//Serial.println(accel_data[0], HEX);
			//Serial.println(accel_data[1], HEX);
			//Serial.println(accel_data[2], HEX);
			Serial.print("ax = ");Serial.print((int)1000*x_accel); Serial.print(";");
			Serial.print("ay = ");Serial.print((int)1000*y_accel); Serial.print(";");
			Serial.print("az = ");Serial.println((int)1000*z_accel); Serial.println(" ");
/*			acc_temp_data = LSM303AGR_A.readAccTempData();
			acc_temperature = ((float) acc_temp_data) + 25.0f; // Accel chip temperature in degrees Centigrade
			Serial.print("Accel temperature is ");  Serial.print(acc_temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
*/
			//Serial.print(" Value_number="); Serial.println(index_page_number_);  //debug
		    //Serial.print("\r\n");
		}

			if (newLSM303AGR_M_Data == true) {
				newLSM303AGR_M_Data = false;     // reset newData flag*/
				LSM303AGR_M_Status = LSM303AGR_M.status();

				if (LSM303AGR_M_Status & 0x08) // if all axes have new data ready
				{
					LSM303AGR_M.readData(data_.mag_data);
					// Now we'll calculate the accleration value into actual G's
					x_mag = (float)data_.mag_data[0] * param_data_.m_Res - param_data_.mag_bias[0]; // get actual G value
					y_mag = (float)data_.mag_data[1] * param_data_.m_Res - param_data_.mag_bias[1];
					z_mag = (float)data_.mag_data[2] * param_data_.m_Res - param_data_.mag_bias[2];
					x_mag *= param_data_.mag_scale[0];
					y_mag *= param_data_.mag_scale[1];
					z_mag *= param_data_.mag_scale[2];
					Serial.print("mx = ");Serial.print((int)1000*x_mag); Serial.print(";");
					Serial.print("my = ");Serial.print((int)1000*y_mag); Serial.print(";");
					Serial.print("mz = ");Serial.println((int)1000*z_mag); Serial.println(" ");
				}
			}

			// 32,768 256-byte pages in a 8 MByte flash
			// 42 valeurs accelero -> 252 octets -> ~ 1 page
			//store RAW accel_data (3 x 2 octets)
			//mise en forme + bias au dump


		    // Send some data to the SPI flash
		    if (index_page_number_ < nb_sector_page && page_number_ < max_last_page_) { // 32,768 256-byte pages in a 8 MByte flash
		    	Serial.println("entre ici");

	/*	    	Serial.print("x_accel : ");
		    	Serial.println(x_accel);
		    	Serial.print("x_accel * 1000 : ");
		    	Serial.println((int)1000*x_accel);
		    	Serial.print("x_accel * 1000 int16 : ");
		    	Serial.println(int16_t((int)1000*x_accel));
		    	Serial.print("x_accel * 1000 int16 & 0xFF00 : ");
		    	Serial.println((int16_t((int)1000*x_accel) & 0xFF00));
		    	Serial.print("x_accel * 1000 int16 & 0xFF00 >> 8 : ");
		    	Serial.println((int16_t((int)1000*x_accel) & 0xFF00) >> 8);
		    	Serial.print("x_accel * 1000 int16 & 0x00FF : ");
		    	Serial.println(int16_t((int)1000*x_accel) & 0x00FF);
	*/
		    	flash_Page[index_page_number_ * 35 + 0]  = day_;
		    	flash_Page[index_page_number_ * 35 + 1]  = month_;
		    	flash_Page[index_page_number_ * 35 + 2]  = year_;
		    	flash_Page[index_page_number_ * 35 + 3]  = hours_;
		    	flash_Page[index_page_number_ * 35 + 4]  = minutes_;
		    	flash_Page[index_page_number_ * 35 + 5]  = seconds_;

		    	if (x_accel > 0) {
		    		flash_Page[index_page_number_ * 35 + 6]  = (int16_t((int)1000*x_accel) & 0xFF00) >> 8;
		    		flash_Page[index_page_number_ * 35 + 7]  = (int16_t((int)1000*x_accel) & 0x00FF);
		    		Serial.print("flash_Page x_accel_MSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 6]);
		    		Serial.print("flash_Page x_accel_LSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 7]);
		    		Serial.println(" ");
		    	}
		    	else {
		    		float x_accel_temp = - x_accel;
		    		int16_t x_accel_temp_MSB = (int16_t((int)1000*x_accel_temp) & 0xFF00);
		    		int16_t x_accel_temp_LSB = (int16_t((int)1000*x_accel_temp) & 0x00FF);
			    	Serial.print("x_accel_temp_MSB : ");
			    	Serial.println(x_accel_temp_MSB);
			    	Serial.print("x_accel_temp_LSB : ");
			    	Serial.println(x_accel_temp_LSB);
		    		flash_Page[index_page_number_ * 35 + 6]  = x_accel_temp_MSB >> 8;
		    		flash_Page[index_page_number_ * 35 + 7]  = x_accel_temp_LSB;
		    		Serial.print("flash_Page x_accel_MSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 6]);
		    		Serial.print("flash_Page x_accel_LSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 7]);
		    		Serial.println(" ");
		    		neg_data = neg_data + 1;
		    	}

		    	if (y_accel > 0) {
			        flash_Page[index_page_number_ * 35 + 8]  = (int16_t((int)1000*y_accel) & 0xFF00) >> 8;
			        flash_Page[index_page_number_ * 35 + 9]  = (int16_t((int)1000*y_accel) & 0x00FF);
		    		Serial.print("flash_Page y_accel_MSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 8]);
		    		Serial.print("flash_Page y_accel_LSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 9]);
		    		Serial.println(" ");
		    	}
		    	else {
		    		float y_accel_temp = - y_accel;
		    		int16_t y_accel_temp_MSB = (int16_t((int)1000*y_accel_temp) & 0xFF00);
		    		int16_t y_accel_temp_LSB = (int16_t((int)1000*y_accel_temp) & 0x00FF);
			    	Serial.print("y_accel_temp_MSB : ");
			    	Serial.println(y_accel_temp_MSB);
			    	Serial.print("y_accel_temp_LSB : ");
			    	Serial.println(y_accel_temp_LSB);
		    		flash_Page[index_page_number_ * 35 + 8]  = y_accel_temp_MSB >> 8;
		    		flash_Page[index_page_number_ * 35 + 9]  = y_accel_temp_LSB;
		    		neg_data = neg_data + 2;
		    		Serial.print("flash_Page y_accel_MSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 8]);
		    		Serial.print("flash_Page y_accel_LSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 9]);
		    		Serial.println(" ");
		     	}

		    	if (z_accel > 0) {
			        flash_Page[index_page_number_ * 35 + 10]  = (int16_t((int)1000*z_accel) & 0xFF00) >> 8;
			        flash_Page[index_page_number_ * 35 + 11]  = (int16_t((int)1000*z_accel) & 0x00FF);
		    		Serial.print("flash_Page z_accel_MSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 10]);
		    		Serial.print("flash_Page z_accel_LSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 11]);
		    		Serial.println(" ");
		    	}
		    	else {
		    		float z_accel_temp = - z_accel;
		    		int16_t z_accel_temp_MSB = (int16_t((int)1000*z_accel_temp) & 0xFF00);
		    		int16_t z_accel_temp_LSB = (int16_t((int)1000*z_accel_temp) & 0x00FF);
			    	Serial.print("z_accel_temp_MSB : ");
			    	Serial.println(z_accel_temp_MSB);
			    	Serial.print("z_accel_temp_LSB : ");
			    	Serial.println(z_accel_temp_LSB);
		    		flash_Page[index_page_number_ * 35 + 10]  = z_accel_temp_MSB >> 8;
		    		flash_Page[index_page_number_ * 35 + 11]  = z_accel_temp_LSB;
		    		neg_data = neg_data + 4;
		    		Serial.print("flash_Page z_accel_MSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 10]);
		    		Serial.print("flash_Page z_accel_LSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 11]);
		    		Serial.println(" ");
		     	}

		    	if (x_mag > 0) {
		    		flash_Page[index_page_number_ * 35 + 12]  = (int16_t((int)1000*x_mag) & 0xFF00) >> 8;
		    		flash_Page[index_page_number_ * 35 + 13]  = (int16_t((int)1000*x_mag) & 0x00FF);
		    		Serial.print("flash_Page x_mag_MSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 12]);
		    		Serial.print("flash_Page x_mag_LSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 13]);
		    		Serial.println(" ");
		    	}
		    	else {
		    		float x_mag_temp = - x_mag;
		    		int16_t x_mag_temp_MSB = (int16_t((int)1000*x_mag_temp) & 0xFF00);
		    		int16_t x_mag_temp_LSB = (int16_t((int)1000*x_mag_temp) & 0x00FF);
			    	Serial.print("x_mag_temp_MSB : ");
			    	Serial.println(x_mag_temp_MSB);
			    	Serial.print("x_mag_temp_LSB : ");
			    	Serial.println(x_mag_temp_LSB);
		    		flash_Page[index_page_number_ * 35 + 12]  = x_mag_temp_MSB >> 8;
		    		flash_Page[index_page_number_ * 35 + 13]  = x_mag_temp_LSB;
		    		neg_data = neg_data + 8;
		    		Serial.print("flash_Page x_mag_MSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 12]);
		    		Serial.print("flash_Page x_mag_LSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 13]);
		    		Serial.println(" ");
		    	}

		    	if (y_mag > 0) {
		    		flash_Page[index_page_number_ * 35 + 14]  = (int16_t((int)1000*y_mag) & 0xFF00) >> 8;
		    		flash_Page[index_page_number_ * 35 + 15]  = (int16_t((int)1000*y_mag) & 0x00FF);
		    		Serial.print("flash_Page y_mag_MSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 14]);
		    		Serial.print("flash_Page y_mag_LSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 15]);
		    		Serial.println(" ");
		    	}
		    	else {
		    		float y_mag_temp = - y_mag;
		    		int16_t y_mag_temp_MSB = (int16_t((int)1000*y_mag_temp) & 0xFF00);
		    		int16_t y_mag_temp_LSB = (int16_t((int)1000*y_mag_temp) & 0x00FF);
			    	Serial.print("y_mag_temp_MSB : ");
			    	Serial.println(y_mag_temp_MSB);
			    	Serial.print("y_mag_temp_LSB : ");
			    	Serial.println(y_mag_temp_LSB);
		    		flash_Page[index_page_number_ * 35 + 14]  = y_mag_temp_MSB >> 8;
		    		flash_Page[index_page_number_ * 35 + 15]  = y_mag_temp_LSB;
		    		neg_data = neg_data + 16;
		    		Serial.print("flash_Page y_mag_MSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 14]);
		    		Serial.print("flash_Page y_mag_LSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 15]);
		    		Serial.println(" ");
		     	}

		    	if (z_mag > 0) {
		    		flash_Page[index_page_number_ * 35 + 16]  = (int16_t((int)1000*z_mag) & 0xFF00) >> 8;
		    		flash_Page[index_page_number_ * 35 + 17]  = (int16_t((int)1000*z_mag) & 0x00FF);
		    		Serial.print("flash_Page z_mag_MSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 16]);
		    		Serial.print("flash_Page z_mag_LSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 17]);
		    		Serial.println(" ");
		    	}
		    	else {
		    		float z_mag_temp = - z_mag;
		    		int16_t z_mag_temp_MSB = (int16_t((int)1000*z_mag_temp) & 0xFF00);
		    		int16_t z_mag_temp_LSB = (int16_t((int)1000*z_mag_temp) & 0x00FF);
			    	Serial.print("z_mag_temp_MSB : ");
			    	Serial.println(z_mag_temp_MSB);
			    	Serial.print("z_mag_temp_LSB : ");
			    	Serial.println(z_mag_temp_LSB);
		    		flash_Page[index_page_number_ * 35 + 16]  = z_mag_temp_MSB >> 8;
		    		flash_Page[index_page_number_ * 35 + 17]  = z_mag_temp_LSB;
		    		neg_data = neg_data + 32;
		    		Serial.print("flash_Page z_mag_MSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 16]);
		    		Serial.print("flash_Page z_mag_LSB : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 17]);
		    		Serial.println(" ");
		     	}

		    	if (data_.depth_mm_1 > 0) {
		    		flash_Page[index_page_number_ * 35 + 18]  = (data_.depth_mm_1 & 0xFF000000) >> 24;
		    		flash_Page[index_page_number_ * 35 + 19]  = (data_.depth_mm_1 & 0x00FF0000) >> 16;
			        flash_Page[index_page_number_ * 35 + 20]  = (data_.depth_mm_1 & 0x0000FF00) >> 8;
			        flash_Page[index_page_number_ * 35 + 21]  = (data_.depth_mm_1 & 0x000000FF);
		    		Serial.print("flash_Page depth_mm_1_MSB_1 : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 18]);
		    		Serial.print("flash_Page depth_mm_1_MSB_2 : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 19]);
		    		Serial.print("flash_Page depth_mm_1_LSB_1 : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 20]);
		    		Serial.print("flash_Page depth_mm_1_LSB_2 : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 21]);
		    		Serial.println(" ");
		    	}
		    	else {
		    		int32_t depth_mm_1_temp = - data_.depth_mm_1;
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
			    	flash_Page[index_page_number_ * 35 + 18]  = depth_mm_1_temp_MSB_1 >> 24;
			    	flash_Page[index_page_number_ * 35 + 19]  = depth_mm_1_temp_MSB_2 >> 16;
			    	flash_Page[index_page_number_ * 35 + 20]  = depth_mm_1_temp_LSB_1 >> 8;
			    	flash_Page[index_page_number_ * 35 + 21]  = depth_mm_1_temp_LSB_2;
			    	neg_data = neg_data + 64;
		    		Serial.print("flash_Page depth_mm_1_MSB_1 : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 18]);
		    		Serial.print("flash_Page depth_mm_1_MSB_2 : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 19]);
		    		Serial.print("flash_Page depth_mm_1_LSB_1 : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 20]);
		    		Serial.print("flash_Page depth_mm_1_LSB_2 : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 21]);
		    		Serial.println(" ");
		     	}

		    	if (data_.depth_mm_30 > 0) {
		    		flash_Page[index_page_number_ * 35 + 22]  = (data_.depth_mm_30 & 0xFF000000) >> 24;
		    		flash_Page[index_page_number_ * 35 + 23]  = (data_.depth_mm_30 & 0x00FF0000) >> 16;
			        flash_Page[index_page_number_ * 35 + 24]  = (data_.depth_mm_30 & 0x0000FF00) >> 8;
			        flash_Page[index_page_number_ * 35 + 25]  = (data_.depth_mm_30 & 0x000000FF);
		    		Serial.print("flash_Page depth_mm_30_MSB_1 : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 22]);
		    		Serial.print("flash_Page depth_mm_30_MSB_2 : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 23]);
		    		Serial.print("flash_Page depth_mm_30_LSB_1 : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 24]);
		    		Serial.print("flash_Page depth_mm_30_LSB_2 : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 25]);
		    		Serial.println(" ");
		    	}
		    	else {
		    		int32_t depth_mm_30_temp = - data_.depth_mm_30;
		    		int32_t depth_mm_30_temp_MSB_1 = (depth_mm_30_temp & 0xFF000000);
		    		int32_t depth_mm_30_temp_MSB_2 = (depth_mm_30_temp & 0x00FF0000);
		    		int32_t depth_mm_30_temp_LSB_1 = (depth_mm_30_temp & 0x0000FF00);
		    		int32_t depth_mm_30_temp_LSB_2 = (depth_mm_30_temp & 0x000000FF);
			    	Serial.print("depth_mm_30_temp_MSB_1 : ");
			    	Serial.println(depth_mm_30_temp_MSB_1);
			    	Serial.print("depth_mm_30_temp_MSB_2 : ");
			    	Serial.println(depth_mm_30_temp_MSB_2);
			    	Serial.print("depth_mm_30_temp_LSB_1 : ");
			    	Serial.println(depth_mm_30_temp_LSB_1);
			    	Serial.print("depth_mm_30_temp_LSB_2 : ");
			    	Serial.println(depth_mm_30_temp_LSB_2);
			    	flash_Page[index_page_number_ * 35 + 22]  = depth_mm_30_temp_MSB_1 >> 24;
			    	flash_Page[index_page_number_ * 35 + 23]  = depth_mm_30_temp_MSB_2 >> 16;
			    	flash_Page[index_page_number_ * 35 + 24]  = depth_mm_30_temp_LSB_1 >> 8;
			    	flash_Page[index_page_number_ * 35 + 25]  = depth_mm_30_temp_LSB_2;
			    	neg_data = neg_data + 128;
		    		Serial.print("flash_Page depth_mm_30_MSB_1 : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 22]);
		    		Serial.print("flash_Page depth_mm_30_MSB_2 : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 23]);
		    		Serial.print("flash_Page depth_mm_30_LSB_1 : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 24]);
		    		Serial.print("flash_Page depth_mm_30_LSB_2 : ");
		    		Serial.println(flash_Page[index_page_number_ * 35 + 25]);
		    		Serial.println(" ");
		     	}

		    	flash_Page[index_page_number_ * 35 + 26]  = (data_.temp_1 & 0xFF000000) >> 24;
		    	flash_Page[index_page_number_ * 35 + 27]  = (data_.temp_1 & 0x00FF0000) >> 16;
		    	flash_Page[index_page_number_ * 35 + 28]  = (data_.temp_1 & 0x0000FF00) >> 8;
		    	flash_Page[index_page_number_ * 35 + 29]  = (data_.temp_1 & 0x000000FF);
		    	Serial.print("flash_Page temp_1_MSB_1 : ");
		    	Serial.println(flash_Page[index_page_number_ * 35 + 26]);
		    	Serial.print("flash_Page temp_1_MSB_2 : ");
		    	Serial.println(flash_Page[index_page_number_ * 35 + 27]);
		    	Serial.print("flash_Page temp_1_LSB_1 : ");
		    	Serial.println(flash_Page[index_page_number_ * 35 + 28]);
		    	Serial.print("flash_Page temp_1_LSB_2 : ");
		    	Serial.println(flash_Page[index_page_number_ * 35 + 29]);
		    	Serial.println(" ");

		    	flash_Page[index_page_number_ * 35 + 30]  = (data_.temp_30 & 0xFF000000) >> 24;
		    	flash_Page[index_page_number_ * 35 + 31]  = (data_.temp_30 & 0x00FF0000) >> 16;
		    	flash_Page[index_page_number_ * 35 + 32]  = (data_.temp_30 & 0x0000FF00) >> 8;
		    	flash_Page[index_page_number_ * 35 + 33]  = (data_.temp_30 & 0x000000FF);
		    	Serial.print("flash_Page temp_30_MSB_1 : ");
		    	Serial.println(flash_Page[index_page_number_ * 35 + 30]);
		    	Serial.print("flash_Page temp_30_MSB_2 : ");
		    	Serial.println(flash_Page[index_page_number_ * 35 + 31]);
		    	Serial.print("flash_Page temp_30_LSB_1 : ");
		    	Serial.println(flash_Page[index_page_number_ * 35 + 32]);
		    	Serial.print("flash_Page temp_30_LSB_2 : ");
		    	Serial.println(flash_Page[index_page_number_ * 35 + 33]);
		    	Serial.println(" ");

		    	flash_Page[index_page_number_ * 35 + 34] = neg_data;
	    		Serial.print("flash_Page sign_data : ");
	    		Serial.println(flash_Page[index_page_number_ * 35 + 34]);
	    		Serial.println(" ");

		    	index_page_number_++;
		    	neg_data = 0;

		    	Serial.print("index_page_number_ : ");
		    	Serial.println(index_page_number_);
		    	Serial.print("page_number_ : ");
		    	Serial.println(page_number_);
		    	Serial.println(" ");
		    	delay(30000);
		    }
		    else if (index_page_number_ == nb_sector_page && page_number_ < max_last_page_) {// on ecrit la page
	   			//Serial.println("entre ici 3");
	   			digitalWrite(LED, LOW); //led on
	            //SPI.begin();           // When exiting STOP mode, re-enable the SPI peripheral
	            //SPIFlash.powerUp();
	   			SPIFlash.flash_page_program(flash_Page, page_number_);  // Temps d'ecriture mesure : 1 ms
	   			Serial.print("***Wrote flash page: "); Serial.println(page_number_); Serial.println(" ");
	   			index_page_number_ = 0;
	   			page_number_++;
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

/*
void LoRa_configuration_connection() {
	LoRaWAN.begin(EU868);
	LoRaWAN.setADR(false);
	LoRaWAN.setDataRate(DATARATE);
	LoRaWAN.setTxPower(10);
	LoRaWAN.setSubBand(1);
	LoRaWAN.setDutyCycle(false);

	char _DevEUI[17];
	delay(5000);
	Serial.println("Start");

	LoRaWAN.getDevEui(_DevEUI,17);
	Serial.print("DevEUI is : ");
	Serial.println(_DevEUI);

	while(!LoRaWAN.joined()) {
		if(!LoRaWAN.busy()) {

			LoRaWAN.joinOTAA(APPEUI, APPKEY, _DevEUI);
			Serial.println("Join attempt");
		}
		else
			Serial.println("Lora busy");
		delay(1000);
	}
	LOGLN("Join Success");
	delay(20000);
}

bool fixGPS(void) {
	return (myPosition.fixType() != GNSSLocation::TYPE_NONE) && (myPosition.fixType() != GNSSLocation::TYPE_TIME);
}
*/
void callback_tick(void) {
	flag_tick = true;
}

void callback_pressureTimer() {
	newPRESSURE_Data = true;
}

/**
    @brief  interrupt handler for the accelerometer interruption 1
    @param  None
    @retval None
 */

void myinthandler_1() {
	newLSM303AGR_A_Data = true;
	STM32L0.wakeup();
}

/**
    @brief  interrupt handler for the accelerometer interruption 2
    @param  None
    @retval None
 */

void myinthandler_2() {
	newLSM303AGR_A_Activity = true;
	STM32L0.wakeup();
}

/**
    @brief  interrupt handler for the magnetometer interruption 1
    @param  None
    @retval None
 */

void myinthandler_3() {
	newLSM303AGR_M_Data = true;
	STM32L0.wakeup();
}
/*
void get_RTC (uint8_t index) {
	RTC.getDate(day_, month_, year_);
	RTC.getTime(hours_[index], minutes_[index], seconds_[index], subSeconds_[index]);
}

void print_RTC (uint8_t index) {
	Serial.print(day_); Serial.print(":"); Serial.print(month_); Serial.print(":20"); Serial.print(year_);
	Serial.print(" ");

	milliseconds_ = ((subSeconds_[index] >> 17) * 1000 + 16384) / 32768;

	if (hours_[index] < 10)
	{
		Serial.print("0");Serial.print(hours_[index]);
	}
	else
		Serial.print(hours_[index]);

	Serial.print(":");
	if (minutes_[index] < 10)
	{
		Serial.print("0"); Serial.print(minutes_[index]);
	}
	else
		Serial.print(minutes_[index]);

	Serial.print(":");
	if (seconds_[index] < 10)
	{
		Serial.print("0"); Serial.print(seconds_[index]);
	}
	else
		Serial.print(seconds_[index]);

	Serial.print(".");
	if (milliseconds_ <= 9)
	{
		Serial.print("0");
	}
	if (milliseconds_ <= 99)
	{
		Serial.print("0");
	}
	Serial.print(milliseconds_);
	Serial.println(" ");
}
/*
void surface_init(void) {
	//init system time
	//manager_rtc_setDefaultTime();
/*	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
*/
/*	application_Mode = app_boot;

	delay(5000);
//	digitalWrite(LED_BUILTIN, HIGH);
	/*for(uint8_t i=0;i<20;i++){
		delay(1000);
		LOG(".");
	}

	LOGLN("");
	LOGLN("Staring app");
*/
/*	if(!mainTimer.start(appTickCallback, 0, mainTimer_period)) {
		LOGLN("App Timer Error");
	}
	else
		LOGLN("App Timer Init");
	if(!surfaceTimer_.start(surfaceTickCallback, 0, surfaceTimer_period)) {
		LOGLN("Surf Timer Error");
	}
	else{
		LOGLN("Surf Timer Init");
		//surfaceTimer_.stop();
	}

	manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_SURFACE);
	//manager_depth_init();

	//manager_gnss_init();

    //test_ss_gnss_in_use = false;

	//pinMode(myVBat_en, OUTPUT);
	//digitalWrite(myVBat_en, LOW); // start with battery voltage monirot off
	//pinMode(myVBat, INPUT);
	//analogReadResolution(12);

	#ifndef DISABLE_IWDG
	STM32L0.wdtEnable(18000);
	#endif

	//manager_lowpower_needSleep();
	LOGLN("Init End");

}

void surface_process(void) {
	manager_surfaceSensor_process();

	if(flagTickSurface_) {
		flagTickSurface_ = false;

		if(application_Mode == app_dive) {
			//manager_surfaceSensor_process();

			//set flagTickMain to true to enable instant send
			if(manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_threshold)) {
				//LOGLN("[DEBUG] TRIGGER THR ");
				if(stop_Diving(SURFACE_TRIGGER_SENSOR_THRESOLD_) == true)
					flagTickMain_ = true;
			}
			else if(manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_gradient)) {
				if(stop_Diving(SURFACE_TRIGGER_SENSOR_GRADIENT_) == true)
					flagTickMain_ = true;
				//LOGLN("[DEBUG] TRIGGER GRAD ");
			}
		}
	}

	/*if((applicationMode == tagApp_surface) && (firstSubsurface)){
		manager_surfaceSensor_process();
	}*/

/*	if(flagTickMain_) {
		#ifndef DISABLE_IWDG
		STM32L0.wdtReset();
		#endif
		flagTickMain_ = false;

		LOGLN(count_Tick);
		if(count_Tick-- <0) {
			LOGLN("RESET SERIAL");
			delay(100);
			Serial.end();
			delay(100);
			Serial.begin(9600);
			count_Tick = SERIAL_RESET_DELAY_;
		}
		appProcess[application_Mode]();
	}

	manager_gnss_processState(); // for to reprocess the state of the GNSS if busy when you try to put it on or off
	shell_receive();
	//LOG("LOCK STOP = ");LOGLN(stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_STOP]);
	manager_lowpower_manage();
}

void appProcess_boot(void) {
	//init the surfacingTime
	tsSurfacingTime_ = RTC.getEpoch();
	tsBackToSurfaceTime_ = tsSurfacingTime_;
	//go to surface
	changeAppMode(app_surface);
}

void appProcess_surface(void) {
	LOGLN("--- SURFACE ----");

	//check subsurface triggers  ----------------------------------------- Ajouter un booleen juste pour le premier passage en subsurface (on le remet a false en dive)
	int32_t depth_mm_surf;

	if (firstSubsurface) {
		//LOGLN("profondeur:");
		//LOGLN(manager_depth_get_mm(pressureSensor_1BA,&depth_mm_surf));
		if(manager_surfaceSensor_diveDetection(surfaceSensorAlgo_threshold)) {
				changeAppMode(app_subsurface);
				tsSubsurfacingTime_ = RTC.getEpoch();
				LOGLN("Premiere replongee: detection par capteur surface");
				mainMessage_.pureSurfaceTime_s += tsSubsurfacingTime_ - tsBackToSurfaceTime_;

				surfaceTimer_.stop();
				LOGLN("STOP CAPTEUR SURFACE - prochains trigs surface/plongee avec 1BA");
				//manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_DIVE);
				//test_ss_surfaceTimer.restart( 0, test_ss_surfaceTimer_period);

				firstSubsurface = false;
				//digitalWrite(LED_BUILTIN, HIGH);									// De-commenter ici pour extinction LED en subsurface

				return;
			}
	}
	else {

		//if(manager_surfaceSensor_diveDetection(surfaceSensorAlgo_threshold)){
		if(manager_depth_get_mm(pressureSensor_1BA, &depth_mm_surf) == psr_OK) {
			if( depth_mm_surf > (SURFACE_DEPTH_CM *10)){//enter in subsurface
				changeAppMode(app_subsurface);
				tsSubsurfacingTime_ = RTC.getEpoch();
				LOGLN("Detection subsurface par 1BA");
				mainMessage_.pureSurfaceTime_s += tsSubsurfacingTime_ - tsBackToSurfaceTime_;

				//manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_DIVE);

				//digitalWrite(LED_BUILTIN, HIGH);							// De-commenter ici pour extinction LED en subsurface

				return;
			}
		}
	}
}

void appProcess_subsurface(void) {
	LOGLN("--- SUBSURFACE ---");
	uint32_t current_Time = RTC.getEpoch();

	//check the subsurfacing time
	/*if( (current_Time-tsSubsurfacingTime) > (SUBSURFACE_MAXTIME_S)  ){
		LOG("TimeOut");
		_startDiving();
		return;
	}*/

/*	int32_t depth_subsurf;

	pressureSensor_result_t res_subsurf;

	res_subsurf = manager_depth_get_mm(pressureSensor_1BA, &depth_subsurf);

	// Subsurface: on detecte les re-surfacages au 1BA (capteur surface desactive apres premier passage surface -> subsurface)

	//if(manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_threshold)){
	if(res_subsurf == psr_OK) {
		//surface detection only with the 1BA
		if(depth_subsurf < (SURFACE_DEPTH_CM *10)) {
			//back to the surface
			mainMessage_.backToSurfaceCount++;
			tsBackToSurfaceTime_ = current_Time;
			changeAppMode(app_surface);
			//digitalWrite(LED_BUILTIN, LOW);							// De-commenter ici pour allumage LED pour detection subsurface -> surface au 1BA
			return;
		}
	}
	if(res_subsurf == psr_OVERLIMIT){
		//active the 30BA sensor
		res_subsurf = manager_depth_get_mm(pressureSensor_30BA, &depth_subsurf);
	}

	//if(manager_surfaceSensor_diveDetection(surfaceSensorAlgo_threshold)){
	if(res_subsurf == psr_OK){
		//at least one sensor
		if(depth_subsurf > (SUBSURFACE_DEPTH_CM *10)) {
			start_Diving();
			return;
		}
	}
	//nothing special to do in subsurface
}

void appProcess_surfacing(void) {
	LOGLN("--- SURFACING ---");
	//digitalWrite(LED_BUILTIN, LOW);
	//LOGLN("Surfacing");
	//build the message

	//start the GPS
	if(!run_GNSS){
		manager_gnss_setState(GNSS_ON);
		//reset the state even if the GPS et not working
		last_EHPE = 10000;
		tsSurfacingTime_ = RTC.getEpoch();
		tsBackToSurfaceTime_ = tsSurfacingTime_;

		if(manager_gnss_getState()) {
			run_GNSS = true;
			LOGLN("Start GPS");
		}
		else
			LOGLN("Start GPS - delayed");
	}
	//always change to surface mode
	changeAppMode(app_surface);
}

void appProcess_dive(void) {
	LOGLN("----- DIVE ------");

	//check surface triggers
	int32_t depth_mm_dive;

	pressureSensor_result_t res_dive = manager_depth_get_mm(pressureSensor_1BA, &depth_mm_dive) ;

	// TRIG SURFACE UNIQUEMENT AU CAPTEUR SURFACE

	if(manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_threshold)) {
		stop_Diving(SURFACE_TRIGGER_SENSOR_THRESOLD_);
		//digitalWrite(LED_BUILTIN, LOW);
		return;
	}

	if(manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_gradient)) {
		stop_Diving(SURFACE_TRIGGER_SENSOR_GRADIENT_);
		//digitalWrite(LED_BUILTIN, LOW);
		return;

	}

	/*if(res == psr_OK ){
		//surface detection is only done by 1BA
		if(depth_mm < (SURFACE_DEPTH_CM * 10)){
			//Surfacing  Now
			#ifdef TRIG_ON_1BA
			_stopDiving(SURFACE_TRIGGER_PRESSURE_1BA);

			return;
			#endif
		}
	}
	else if(res == psr_OVERLIMIT){
		//active the 30BA
		res = manager_depth_get_mm(pressureSensor_30BA, &depth_mm);

	}*/

	//Execute dive mode
/*	if(res == psr_OK){
		//at least one of the sensor
		//message_histo_addDepthPoint_cm(&test_ss_mainMessage, depth_mm /10);
		diveZone = message_histo_getDiveZone(depth_mm/10);
		message_histo_addDepthPoint_diveZone(&test_ss_mainMessage,diveZone);
		LOG("Zone : ");LOGLN(diveZone);
	}*/
/*}

void start_Diving(void) {
	 // RTC.attachInterrupt(alarmMatch);
	//digitalWrite(LED_BUILTIN, HIGH);						// De-commenter ici pour extinction LED en dive

	changeAppMode(app_dive);
	tsDiveStartTime_ = RTC.getEpoch();

	//stop the GPS
	if(run_GNSS) {
		manager_gnss_setState(GNSS_OFF);
		if(manager_gnss_getState() == GNSS_OFF)
			run_GNSS = false;
		else
			LOGLN("STOP GPS - delayed");
	}

	manager_depth_calibrate30BA();

	manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_DIVE);
	surfaceTimer_.restart( 0, surfaceTimer_period);
	LOGLN("RESTART CAPTEUR SURFACE");

	dive_Zone = 0;

	firstSubsurface = true;
}

bool stop_Diving(uint8_t triggerId) {
	int32_t depth_;
	pressureSensor_result_t res_;

	res_ = manager_depth_get_mm(pressureSensor_1BA, &depth_);

	LOG("Surface trigged by : ");LOGLN(TriggerStr_[triggerId]);
	//test_ss_surfaceTimer.stop();											// CA a enlever pour ne pas desactiver a la surface

	changeAppMode(app_surfacing);
	return true;
}

void appTickCallback(void) {
	flagTickMain_ = true;
	STM32L0.wakeup();
}

void surfaceTickCallback(void) {
	flagTickSurface_ = true;
	STM32L0.wakeup();
}

void changeAppMode(appMode_t mode) {
	application_Mode = mode;
	LOG("Mode change to : ");
	LOGLN(AppModeStr[application_Mode]);
}
*/
