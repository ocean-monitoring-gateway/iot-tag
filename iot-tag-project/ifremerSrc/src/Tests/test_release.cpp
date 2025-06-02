/*
 * test_release.cpp
 *
 *  Created on: 8 juin 2020
 *      Author: jnguyen
 */


/*
La carte va devoir être testée avant et après chaque étape de l'intégration dans la marque. Il va falloir un ou plusieurs programmes.

Je verrais bien un programme pour tester chaque partie de la carte sauf la communication LoRa. Le programme fait un retour d’information sur le port série.
Le programme va écrire et lire dans la flash et ressort sur le port série un résultat: OK pour pas OK
Le programme peut également ressortir la date et l’heure.
Ensuite il mesure la pression avec un capteur et ressort la valeur.
Puis la même chose avec l’autre capteur de pression, le capteur de température, le capteur de lumière, gps.
Potentiellement, il faut pouvoir choisir quels capteurs on veut tester. Par exemple le GPS n’est pas utilisé dans la marque release, donc pas d’antenne connectée, le test est inutile.
Ca peut être aussi simple qu’une variable initalisée à 0 ou 1 pour conditionner la partie du programme qui concerne tel capteur.
Voila les choses auxquelles je pense, si tu as d’autres idées je suis ouvert aux suggestions.
*/

#include "LoRaWAN.h"

#include "../_appDirectives.h"
#include "../LoraApplication/ifremerLoraMsg.h"

#include "../Manager/_manager.h"
#include "../_board.h"
#include "../Tools/shell.h"
#include "SPIFlash.h"
#include "VEML6030.h"
#include "test_release.h"


//---------------------------- DATALOG PARAMETERS --------------------------------------
#define pressure_sensor_sampling_time_release 100
//--------------------------------------------------------------------------------------

// SPI Flash: 64 MBit (8 MByte) SPI Flash 32,768, 256-byte pages
//page 0 and 1 reserved, accelero values start at page 2
uint8_t   flash_Page_release[256];          // array to hold the data for flash page write

TimerMillis pressure_sensor_timer_release;  // instantiate high-frequency timer

extern I2Cdev             i2c_0;
extern SPIFlash SPIFlash;

uint32_t UID_release[3] = {0, 0, 0};

uint8_t hours_release[4] = {12,12,12,12}, minutes_release[4] = {0,0,0,0}, seconds_release[4] = {0,0,0,0}, year_release = 1, month_release = 1, day_release = 1;
uint32_t subSeconds_release[4], milliseconds_release;

// Battery voltage monitor definitions
float VDDA_release, VBAT_release, VBUS_release, STM32L0Temp_release;

// Cricket pin assignments
#define my_VBAT_release_en  2 // enable VBat read
#define my_VBAT_release    A1 // VBat analog read pin

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G
      AODR_1Hz, AODR_10Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_400Hz
*/
uint8_t A_scale_release = AFS_2G, A_ODR_release = AODR_100Hz; // assuming normal mode operation

float a_Res_release;
float param_accel_bias_release[3] = {0.0f, 0.0f, 0.0f}; // offset biases copy for fonction call
int16_t accel_data_release[3], acc_temp_data_release;  // Stores the 10-bit signed accel output if normal mode
//float acc_temperature;             // Stores the real internal gyro temperature in degrees Celsius
float x_accel_release, y_accel_release, z_accel_release;                   // variables to hold latest accel data values

//LSM303AGR_A LSM303AGR_A(&i2c_0); // instantiate LSM303AGR accel class
extern LSM303AGR_A LSM303AGR_A;

volatile bool newLSM303AGR_A_Data_release = false; // used for data ready interrupt handling
volatile bool newLSM303AGR_A_Activity_release  = false; // used for activity interrupt handling

/* Specify sensor parameters (sample rate is twice the bandwidth)
   choices are: MODR_10Hz, MOIDR_20Hz, MODR_50 Hz and MODR_100Hz
 */
uint8_t M_ODR_release = MODR_10Hz;

float m_Res_release = 0.0015f;            // mag
float param_mag_bias_release[3] = {0.0f, 0.0f, 0.0f}, param_mag_scale_release[3]  = {0.0f, 0.0f, 0.0f}; // Bias corrections for magnetometer
int16_t mag_data_release[4];              // Stores the 16-bit signed sensor output
//float mag_temperature;            // Stores the real internal chip temperature in degrees Celsius
float x_mag_release, y_mag_release, z_mag_release;                // variables to hold latest mag data values
uint8_t LSM303AGR_M_Status_release;

volatile bool newLSM303AGR_M_Data_release = false; // LIS2MDL magnetometer interrupt flag

//LSM303AGR_M LSM303AGR_M(&i2c_0); // instantiate LSM303AGR_M class
extern LSM303AGR_M LSM303AGR_M;

volatile bool newPRESSURE_Data_release = false;

//Choices are:
// IT: (IT_25  25 ms, IT_50  50 ms,) IT_100 = 100 ms, IT_200 = 200 ms, IT_400 = 400 ms, IT_800 = 800 ms
// Gain: Gain_1x 1x gain, Gain_2x 2x gain, Gain_0_125 1/8 x gain, Gain_0_25 1/4 x gain
// Persistance: 0x00 = 1, 0x01 = 2, 0x02 = 4, 0x03 = 8 // Num,ber of times a threshold must be crossed to trigger an interrupt
// Power save Mode = 0x00, 0x01, 0x02, 0x03 higher the mode, longer time between data but lower the current usage
uint8_t IT_release = IT_100, Gain_release = Gain_1x, Persistance_release = 0x00, powerMode_release = 0x00;  // configuration variable

uint16_t ALSData_release = 0, WhiteData_release = 0, IntStatus_release = 0;
// lux/LSBit, ambient light sensitivity increases with integration time
float Sensitivity_release = 0.0288f/((float) (1 << IT_release) ); // for IT_release = 100 ms, 200 ms, 400 ms or 800 ms only
float ambientLight_release, whiteLight_release;
volatile bool VEML6030_flag_release = false;

//Extern VEML6030 VEML6030;
VEML6030 VEML6030_release(&i2c_0);

dataFlash_release_t data_release;
dataParamFlash_release_t param_data_release;

// GNSS
GNSSLocation myPosition_release;
TimerMillis timer_release;
data_gnssTracking_release_t gnss_Point_release;

#define TICK_MAIN_release	1000

bool run_GNSS_release = false;
volatile bool flag_tick_release = false;

// Surface sensor
static appMode_release_t application_Mode_release = app_boot_release;

#define SERIAL_RESET_DELAY_release	(1800L)
static int16_t count_Tick_release = SERIAL_RESET_DELAY_release;

static uint8_t dive_Zone_release = 0;

data_histo_t mainMessage_release;

bool firstSubsurface_release = true;

TimerMillis mainTimer_release;
const uint32_t mainTimer_release_period = 1000;
void appTickCallback_release(void);
volatile bool flagTickMain_release = false;

TimerMillis surfaceTimer_release;
const uint32_t surfaceTimer_release_period = deltaT_surface_detect;
void surfaceTickCallback_release(void);
volatile bool flagTickSurface_release = false;

uint32_t tsSurfacingTime_release; //timestamp of the surfacing time
uint32_t tsBackToSurfaceTime_release; //timestamp when we go back to the surface
uint32_t tsSubsurfacingTime_release; //timestamp enter in subsurface
uint32_t tsDiveStartTime_release; //timestamp enter in subsurface

static float last_EHPE_release = 10000;

// Application processes jump Table (constant array of function pointer)
void (* const appProcess[app_size_release])(void) = {
	appProcess_boot_release, // [tagApp_unknow] = _process_unknow,
	appProcess_surface_release, //[tagApp_surface] = _process_surface,
	appProcess_subsurface_release, //[tagApp_subsurface] = _process_subsurface,
	appProcess_surfacing_release, //[tagApp_surfacing] = _process_surfacing,
	appProcess_dive_release //[tagApp_dive] = _process_dive
};

const char *const TriggerStr_release[4] = { "THR", "GRAD", "1BA", "30BA"};

const char* const AppModeStr_release[app_size_release] = { "Boot", "Surface", "Subsurface", "Surfacing", "Dive"};

#define SURFACE_TRIGGER_SENSOR_THRESOLD_release 	0
#define SURFACE_TRIGGER_SENSOR_GRADIENT_release		1
#define SURFACE_TRIGGER_PRESSURE_1BA_release		2
#define SURFACE_TRIGGER_PRESSURE_30BA_release 		3

/**
    @brief  Main function of test_release_no_LoRa
    @param  None
    @retval None
 */

//--------------------A MODIFIER POUR CHOISIR LE CAPTEUR A TESTER-----------------------
// 1 : à tester, autre valeur : pas testé
// Le test pour la flash doit se faire seul : si var_flash = 1, toutes les autres var_ = 0
uint8_t var_press_temp = 1;
uint8_t var_lum = 0;
uint8_t var_surf = 0;
uint8_t var_accel = 1;
uint8_t var_mag = 0;
uint8_t var_gps = 1;
uint8_t var_lora = 1;
uint8_t var_flash = 0;
//--------------------------------------------------------------------------------------

void test_release() {

	delay(4000);
	Serial.begin(115200);
	delay(4000);

	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);  // start with blue led on (since active LOW)

	pinMode(my_VBAT_release_en, OUTPUT);
	digitalWrite(my_VBAT_release_en, LOW); // start with battery voltage monirot off
	pinMode(my_VBAT_release, INPUT);
	analogReadResolution(12);

	// Set the RTC time to firmware build time
	manager_rtc_setDefaultTime();

	Serial.println("Serial enabled!");

	STM32L0.getUID(UID_release);
	Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID_release[0], HEX); Serial.print(UID_release[1], HEX); Serial.println(UID_release[2], HEX);

	VDDA_release = STM32L0.getVDDA();
	VBUS_release = STM32L0.getVBUS();
	digitalWrite(my_VBAT_release_en, HIGH);
	VBAT_release = 1.27f * VDDA_release * analogRead(my_VBAT_release) / 4096.0f;
	digitalWrite(my_VBAT_release_en, LOW);
	STM32L0Temp_release = STM32L0.getTemperature();

	// Internal STM32L0 functions
	Serial.print("VDDA = "); Serial.print(VDDA_release, 2); Serial.println(" V");
	Serial.print("VBAT = "); Serial.print(VBAT_release, 2); Serial.println(" V");
	if(VBUS_release ==  1)  Serial.println("USB Connected!");
	Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp_release, 2);
	Serial.println(" ");

	if (var_flash == 1 || var_gps == 1){
		//SPI FLASH
		pinMode(csPin, OUTPUT); // set SPI chip select as L082 output
		digitalWrite(csPin, HIGH);

		// check SPI Flash ID
		SPIFlash.init();      // start SPI (include SPI.begin)
		SPIFlash.powerUp();   // MX25R6435FZAI defaults to power down state
		SPIFlash.getChipID(); // Verify SPI flash communication
		if (var_flash == 1)
			SPIFlash.flash_chip_erase(1);
		//delay(15000);
	}

	I2C_BUS.begin();                                      // Set master mode, default on SDA/SCL for STM32L4
	delay(1000);
	I2C_BUS.setClock(400000);                             // I2C frequency at 400 kHz
	delay(1000);

	if (var_accel == 1) {
		// Set up accelero
		pinMode(LSM303AGR_A_intPin1_release, INPUT);
		pinMode(LSM303AGR_A_intPin2_release, INPUT);

		// Read the LSM303AGR Chip ID register, this is a good test of communication
		Serial.println("LSM303AGR accel/gyro...");
		byte c = LSM303AGR_A.getChipID();  // Read CHIP_ID register for LSM303AGR
		Serial.print("LSM303AGR "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x33, HEX);
		delay(1000);

		param_data_release.A_ODR_release = AODR_100Hz;
		param_data_release.A_scale_release = AFS_2G;
		param_data_release.a_Res_release = LSM303AGR_A.getAres(param_data_release.A_scale_release); // get sensor resolution, only need to do this once

		blink_led(3);
		delay(5000);
		LSM303AGR_A.selfTest();
		LSM303AGR_A.reset();
		LSM303AGR_A.init(param_data_release.A_scale_release, param_data_release.A_ODR_release);

		LSM303AGR_A.offsetBias(param_accel_bias_release);

		param_data_release.accel_bias_release[0] = param_accel_bias_release[0];
		param_data_release.accel_bias_release[1] = param_accel_bias_release[1];
		param_data_release.accel_bias_release[2] = param_accel_bias_release[2];

		Serial.println("Accel biases (mg)");
		Serial.println(1000.0f * param_data_release.accel_bias_release[0]);
		Serial.println(1000.0f * param_data_release.accel_bias_release[1]);
		Serial.println(1000.0f * param_data_release.accel_bias_release[2]);
		Serial.println(" ");

		attachInterrupt(LSM303AGR_A_intPin1_release, myinthandler_1_release, RISING);  // define data ready interrupt for intPin1 output of LSM303AGR
		attachInterrupt(LSM303AGR_A_intPin2_release, myinthandler_2_release, RISING);  // define no-motion activity interrupt for intPin2 output of LSM303AGR

		LSM303AGR_A.readAccData(accel_data_release); // INT1 cleared on any read
		// END ACCELERO
	}

	if (var_mag == 1) {
		// Set up magneto
		pinMode(LSM303AGR_M_intPin_release, INPUT);    // Set up interrupt pins

		// Read the LSM303AGR_M Chip ID register, this is a good test of communication
		Serial.println("LSM303AGR_M mag...");
		byte d = LSM303AGR_M.getChipID();  // Read CHIP_ID register for LSM303AGR
		Serial.print("LSM303AGR_M "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x40, HEX);
		Serial.println(" ");
		delay(1000);

		param_data_release.m_Res_release = 0.0015f;  // fixed sensitivity and full scale (+/- 49.152 Gauss);

		blink_led(5);
		delay(5000);
		LSM303AGR_M.selfTest();
		LSM303AGR_M.reset(); // software reset LIS2MDL to default registers
		LSM303AGR_M.init(param_data_release.M_ODR_release);

		LSM303AGR_M.offsetBias(param_mag_bias_release, param_mag_scale_release);

		param_data_release.mag_bias_release[0] = param_mag_bias_release[0];
		param_data_release.mag_bias_release[1] = param_mag_bias_release[1];
		param_data_release.mag_bias_release[2] = param_mag_bias_release[2];
		param_data_release.mag_scale_release[0] = param_mag_scale_release[0];
		param_data_release.mag_scale_release[1] = param_mag_scale_release[1];
		param_data_release.mag_scale_release[2] = param_mag_scale_release[2];

		Serial.println("Mag biases (mG)"); Serial.println(1000.0f * param_data_release.mag_bias_release[0]); Serial.println(1000.0f * param_data_release.mag_bias_release[1]); Serial.println(1000.0f * param_data_release.mag_bias_release[2]);
		Serial.println("Mag scale (mG)"); Serial.println(1000.0f * param_data_release.mag_scale_release[0]); Serial.println(1000.0f * param_data_release.mag_scale_release[1]); Serial.println(1000.0f * param_data_release.mag_scale_release[2]);
		Serial.println(" ");

		attachInterrupt(LSM303AGR_M_intPin_release , myinthandler_3_release, RISING);  // define data ready interrupt for intPin  output of LIS2MDL

		LSM303AGR_M.readData(mag_data_release);  // read data register to clear interrupt before main loop
		// END MAGNETO
	}

    if (var_lum == 1) {
    	// LUMIERE
    	VEML6030_release.init(IT_release, Gain_release, Persistance_release); // initialize the VEML6030 ALS
    	VEML6030_release.enablepowerSave(powerMode_release);
    	VEML6030_release.setHighThreshold(0x0400); // set high threshold to 1024/65,536
    	VEML6030_release.setLowThreshold(0x0008);  // set  low threshold to    8/65,536
    	uint16_t HiThrs = VEML6030_release.getHighThreshold();
    	uint16_t LoThrs = VEML6030_release.getLowThreshold();
    	Serial.print("High Threshold is : 0x"); Serial.println(HiThrs, HEX);
    	Serial.print("Lo Threshold is : 0x"); Serial.println(LoThrs, HEX);
    	Serial.println(" ");
    	//END LUMIERE
	}


    // GNSS + LoRa init
    uint32_t start_time, fix_time;
    uint8_t first_fix = 0; //1 for first Fix
    data_TTFF_t ttff_;
    data_gnssTracking_t gnss_Point;

   	const float EHPE_ = 200;

    	if (var_gps == 1 || var_lora == 1) {
    		if (var_gps == 1)
    			manager_gnss_init();

    		if (var_gps == 1 || var_lora == 1) {
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

    		if (var_gps == 1) {
        		manager_gnss_setState(GNSS_ON);
        		while(manager_gnss_getState() != GNSS_ON)
        			manager_gnss_processState();

        		run_GNSS_release = true;
        		start_time = RTC.getEpoch();

        		LOGLN("Start Timer");
        		timer_release.start(callback_tick_release, TICK_MAIN_release, TICK_MAIN_release);
    		}
       // END GNSS + LoRA init
    }

    if (var_press_temp == 1){
    	// PRESSURE
    	int32_t depth_1_release = 0;
    	int32_t depth_30_release = 0;
    	manager_depth_init();
    	manager_depth_get_mm(pressureSensor_1BA, &depth_1_release);
    	manager_depth_get_mm(pressureSensor_30BA, &depth_30_release);
    	manager_depth_readPressure_mPa_1BA(&depth_1_release);
    	manager_depth_readPressure_mPa_30BA(&depth_30_release);
    	data_release.depth_mm_1_release = depth_1_release;
    	data_release.depth_mm_30_release = depth_30_release;
    	pressure_sensor_timer_release.start(callback_pressureTimer_release, 0,  pressure_sensor_sampling_time_release);
    	// END PRESSURE
    }

	if (var_surf == 1) {
		// SURFACE
		manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_SURFACE);
		surface_init_release();
		// SURFACE
	}

	while (1) {

		if (var_press_temp == 1) {
			Serial.println("------------------------------------------------------------------------------------------");
			Serial.print("\t");
			Serial.println("Le test concerne : les capteurs de pression/temperature");
			Serial.println("------------------------------------------------------------------------------------------");

			if(newPRESSURE_Data_release == true){
				newPRESSURE_Data_release = false;
				manager_depth_get_mm(pressureSensor_1BA,&data_release.depth_mm_1_release);
				manager_depth_get_mm(pressureSensor_30BA,&data_release.depth_mm_30_release);
				Serial.println(" ");
			}
			delay(1000);
		}

		if (var_lum == 1) {
			Serial.println("------------------------------------------------------------------------------------------");
			Serial.print("\t");
			Serial.println("Le test concerne : le capteur de lumiere");
			Serial.println("------------------------------------------------------------------------------------------");

			ALSData_release = VEML6030_release.getALSData();
			ambientLight_release = ((float)ALSData_release)*Sensitivity_release; // ALS in lux
			WhiteData_release = VEML6030_release.getWhiteData();
			whiteLight_release = ((float)WhiteData_release)*Sensitivity_release; // White light in lux
			Serial.print("VEML6030 ALS : "); Serial.print(ambientLight_release, 2); Serial.println(" lux");
			Serial.print("VEML6030 White : "); Serial.print(whiteLight_release, 2); Serial.println(" lux"); Serial.println(" ");
			delay(1000);
		}

		if (var_surf == 1) {
			Serial.println("------------------------------------------------------------------------------------------");
			Serial.print("\t");
			Serial.println("Le test concerne : le capteur de surface");
			Serial.println("------------------------------------------------------------------------------------------");

			//manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_SURFACE);
			//surface_init_release();
			surface_process_release();
			delay(1000);
		}

		if (var_accel == 1) {
			Serial.println("------------------------------------------------------------------------------------------");
			Serial.print("\t");
			Serial.println("Le test concerne : l'accelerometre");
			Serial.println("------------------------------------------------------------------------------------------");

			if(newLSM303AGR_A_Data_release == true) // on interrupt, read data
			{
				digitalWrite(LED, HIGH);    // turn off led
				newLSM303AGR_A_Data_release = false;     // reset newData flag
				LSM303AGR_A.readAccData(data_release.accel_data_release); // INT1 cleared on any read
				// Now we'll calculate the accleration value into actual g's
				x_accel_release = (float)data_release.accel_data_release[0]*param_data_release.a_Res_release - param_data_release.accel_bias_release[0];  // get actual g value, this depends on scale being set
				y_accel_release = (float)data_release.accel_data_release[1]*param_data_release.a_Res_release - param_data_release.accel_bias_release[1];
				z_accel_release = (float)data_release.accel_data_release[2]*param_data_release.a_Res_release - param_data_release.accel_bias_release[2];

				Serial.print("ax = ");Serial.print((int)1000*x_accel_release); Serial.print(";");
				Serial.print("ay = ");Serial.print((int)1000*y_accel_release); Serial.print(";");
				Serial.print("az = ");Serial.println((int)1000*z_accel_release); Serial.println(" ");
			}
			delay(1000);
		}

		if (var_mag == 1) {
			Serial.println("------------------------------------------------------------------------------------------");
			Serial.print("\t");
			Serial.println("Le test concerne : le magnetometre");
			Serial.println("------------------------------------------------------------------------------------------");

			if (newLSM303AGR_M_Data_release == true) {
				newLSM303AGR_M_Data_release = false;     // reset newData flag*/
				LSM303AGR_M_Status_release = LSM303AGR_M.status();

				if (LSM303AGR_M_Status_release & 0x08) // if all axes have new data ready
				{
					LSM303AGR_M.readData(data_release.mag_data_release);
					// Now we'll calculate the accleration value into actual G's
					x_mag_release = (float)data_release.mag_data_release[0] * param_data_release.m_Res_release - param_data_release.mag_bias_release[0]; // get actual G value
					y_mag_release = (float)data_release.mag_data_release[1] * param_data_release.m_Res_release - param_data_release.mag_bias_release[1];
					z_mag_release = (float)data_release.mag_data_release[2] * param_data_release.m_Res_release - param_data_release.mag_bias_release[2];
					x_mag_release *= param_data_release.mag_scale_release[0];
					y_mag_release *= param_data_release.mag_scale_release[1];
					z_mag_release *= param_data_release.mag_scale_release[2];
					Serial.print("mx = ");Serial.print((int)1000*x_mag_release); Serial.print(";");
					Serial.print("my = ");Serial.print((int)1000*y_mag_release); Serial.print(";");
					Serial.print("mz = ");Serial.println((int)1000*z_mag_release); Serial.println(" ");
				}
			}
			delay(1000);
		}

		if (var_gps == 1) {
			Serial.println("------------------------------------------------------------------------------------------");
			Serial.print("\t");
			Serial.println("Le test concerne : le GPS");
			Serial.println("------------------------------------------------------------------------------------------");

			if(flag_tick_release){
							flag_tick_release = false;
							LOG(".");

							if(!run_GNSS_release) {
								LOG("Starting GPS");
								manager_gnss_setState(GNSS_ON);
							    while(manager_gnss_getState() != GNSS_ON)
							    	manager_gnss_processState();

							    start_time = RTC.getEpoch();

							    run_GNSS_release = true;
							    timer_release.restart(TICK_MAIN_release, TICK_MAIN_release);
								LOG("Launch Main Tick");
							}

							else {
								GNSS.location(myPosition_release);
								LOGLN("Try to fix");
								manager_gnss_logSattelites();
								LOG("Fix ? ");LOGLN(fixGPS_release());
								delay(1000);
								if (fixGPS_release()) {
									LOG("EHPE : ");
									LOG(myPosition_release.ehpe());
									LOGLN("");

									uint32_t ctime = RTC.getEpoch();

									gnss_Point.lat = myPosition_release.latitude();
									LOG("LAT : ");
									LOG(myPosition_release.latitude());
									LOGLN("");
									gnss_Point.lng = myPosition_release.longitude();
									LOG("LONG : ");
									LOG(myPosition_release.longitude());
									LOGLN("");
									gnss_Point.ehpe = myPosition_release.ehpe();
									gnss_Point.ttf = ctime - start_time;
									delay(1000);
								}
							}
			}
		}

		if (var_lora == 1) {
			Serial.println("------------------------------------------------------------------------------------------");
			Serial.print("\t");
			Serial.println("Le test concerne : LoRa");
			Serial.println("------------------------------------------------------------------------------------------");

			if(LoRaWAN.joined()) {
				LOGLN("Lora joined");
				if (var_gps == 1) {
					gnss_Point_release.lat_release = myPosition_release.latitude();
					LOG("LAT : ");
					LOG(myPosition_release.latitude());
					LOGLN("");
					gnss_Point_release.lng_release = myPosition_release.longitude();
					LOG("LONG : ");
					LOG(myPosition_release.longitude());
					LOGLN("");
					LoRaWAN.sendPacket(loraPortNumber[ilp_GnssTracking], (uint8_t*)&gnss_Point_release, sizeof(data_gnssTracking_release_t));
				}
				else {
					const char *strHello = "Hello";
					LOGLN(strHello);
					LoRaWAN.sendPacket(loraPortNumber[ilp_StringMsg], (uint8_t*)strHello, strlen(strHello), false);
					// La trame obtenue sur Orange Live Objects doit être : 48656c6c6f qui correspond à Hello en code hexa

				}
			}
			delay(1000);
		}

		if (var_flash == 1) {
			Serial.println("------------------------------------------------------------------------------------------");
			Serial.print("\t");
			Serial.println("Le test concerne : l'ecriture en flash");
			Serial.println("------------------------------------------------------------------------------------------");

			for (uint16_t i=0; i<256; i++){
				flash_Page_release[i] = i;
				Serial.println(flash_Page_release[i]);
			}
			SPIFlash.flash_page_program(flash_Page_release, 0);
			Serial.println("Ecriture en flash : OK");
			Serial.println("------------------------------------------------------------------------------------------");
			Serial.print("\t");
			Serial.println("Le test concerne : la lecture en flash");
			Serial.println("------------------------------------------------------------------------------------------");
				for (uint16_t j=0; j<2; j++){
					for (uint16_t k=0; k<256; k++){
						SPIFlash.flash_read_pages(flash_Page_release, j, j+1);
						Serial.println(flash_Page_release[k]);
					}
				}
				Serial.println("Lecture de la flash : OK");
				break;
		}

		if (var_press_temp != 1 && var_lum != 1 && var_surf != 1 && var_accel != 1 && var_mag != 1 && var_gps != 1 && var_flash != 1) {
			Serial.println("------------------------------------------------------------------------------------------");
			Serial.print("\t");
			Serial.println("Aucun capteur a tester !!!");
			Serial.println("------------------------------------------------------------------------------------------");
			break;
		}
	}
}

bool fixGPS_release(void) {
	return (myPosition_release.fixType() != GNSSLocation::TYPE_NONE) && (myPosition_release.fixType() != GNSSLocation::TYPE_TIME);
}

void callback_tick_release(void) {
	flag_tick_release = true;
}

void callback_pressureTimer_release() {
	newPRESSURE_Data_release = true;
}

/**
    @brief  interrupt handler for the accelerometer interruption 1
    @param  None
    @retval None
 */

void myinthandler_1_release() {
	newLSM303AGR_A_Data_release = true;
	STM32L0.wakeup();
}

/**
    @brief  interrupt handler for the accelerometer interruption 2
    @param  None
    @retval None
 */

void myinthandler_2_release() {
	newLSM303AGR_A_Activity_release = true;
	STM32L0.wakeup();
}

/**
    @brief  interrupt handler for the magnetometer interruption 1
    @param  None
    @retval None
 */

void myinthandler_3_release() {
	newLSM303AGR_M_Data_release = true;
	STM32L0.wakeup();
}

void get_RTC_release (uint8_t index) {
	RTC.getDate(day_release, month_release, year_release);
	RTC.getTime(hours_release[index], minutes_release[index], seconds_release[index], subSeconds_release[index]);
}

void print_RTC_release (uint8_t index) {
	Serial.print(day_release); Serial.print(":"); Serial.print(month_release); Serial.print(":20"); Serial.print(year_release);
	Serial.print(" ");

	milliseconds_release = ((subSeconds_release[index] >> 17) * 1000 + 16384) / 32768;

	if (hours_release[index] < 10)
	{
		Serial.print("0");Serial.print(hours_release[index]);
	}
	else
		Serial.print(hours_release[index]);

	Serial.print(":");
	if (minutes_release[index] < 10)
	{
		Serial.print("0"); Serial.print(minutes_release[index]);
	}
	else
		Serial.print(minutes_release[index]);

	Serial.print(":");
	if (seconds_release[index] < 10)
	{
		Serial.print("0"); Serial.print(seconds_release[index]);
	}
	else
		Serial.print(seconds_release[index]);

	Serial.print(".");
	if (milliseconds_release <= 9)
	{
		Serial.print("0");
	}
	if (milliseconds_release <= 99)
	{
		Serial.print("0");
	}
	Serial.print(milliseconds_release);
	Serial.println(" ");
}

void surface_init_release(void) {
	application_Mode_release = app_boot_release;

	delay(5000);

	if(!mainTimer_release.start(appTickCallback_release, 0, mainTimer_release_period)) {
		LOGLN("App Timer Error");
	}
	else
		LOGLN("App Timer Init");
	if(!surfaceTimer_release.start(surfaceTickCallback_release, 0, surfaceTimer_release_period)) {
		LOGLN("Surf Timer Error");
	}
	else{
		LOGLN("Surf Timer Init");
		//surfaceTimer_release.stop();
	}

	manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_SURFACE);

	#ifndef DISABLE_IWDG
	STM32L0.wdtEnable(18000);
	#endif

	LOGLN("Init End");

}

void surface_process_release(void) {
	manager_surfaceSensor_process();

	if(flagTickSurface_release) {
		flagTickSurface_release = false;

		if(application_Mode_release == app_dive_release) {
			//manager_surfaceSensor_process();

			//set flagTickMain to true to enable instant send
			if(manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_threshold)) {
				//LOGLN("[DEBUG] TRIGGER THR ");
				if(stop_Diving_release(SURFACE_TRIGGER_SENSOR_THRESOLD_release) == true)
					flagTickMain_release = true;
			}
			else if(manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_gradient)) {
				if(stop_Diving_release(SURFACE_TRIGGER_SENSOR_GRADIENT_release) == true)
					flagTickMain_release = true;
				//LOGLN("[DEBUG] TRIGGER GRAD ");
			}
		}
	}

	if(flagTickMain_release) {
		#ifndef DISABLE_IWDG
		STM32L0.wdtReset();
		#endif
		flagTickMain_release = false;

		LOGLN(count_Tick_release);
		if(count_Tick_release-- <0) {
			LOGLN("RESET SERIAL");
			delay(100);
			Serial.end();
			delay(100);
			Serial.begin(9600);
			count_Tick_release = SERIAL_RESET_DELAY_release;
		}
		appProcess[application_Mode_release]();
	}

	manager_gnss_processState(); // for to reprocess the state of the GNSS if busy when you try to put it on or off
	shell_receive();
	//LOG("LOCK STOP = ");LOGLN(stm32l0_system_device.lock[STM32L0_SYSTEM_LOCK_STOP]);
	manager_lowpower_manage();
}

void appProcess_boot_release(void) {
	//init the surfacingTime
	tsSurfacingTime_release = RTC.getEpoch();
	tsBackToSurfaceTime_release = tsSurfacingTime_release;
	//go to surface
	changeAppMode_release(app_surface_release);
}

void appProcess_surface_release(void) {
	LOGLN("--- SURFACE ----");

	//check subsurface triggers  ----------------------------------------- Ajouter un booleen juste pour le premier passage en subsurface (on le remet a false en dive)
	int32_t depth_mm_surf;

	if (firstSubsurface_release) {
		//LOGLN("profondeur:");
		//LOGLN(manager_depth_get_mm(pressureSensor_1BA,&depth_mm_surf));
		if(manager_surfaceSensor_diveDetection(surfaceSensorAlgo_threshold)) {
				changeAppMode_release(app_subsurface_release);
				tsSubsurfacingTime_release = RTC.getEpoch();
				LOGLN("Premiere replongee: detection par capteur surface");
				//mainMessage_release.pureSurfaceTime_s += tsSubsurfacingTime_release - tsBackToSurfaceTime_release;

				surfaceTimer_release.stop();
				LOGLN("STOP CAPTEUR SURFACE - prochains trigs surface/plongee avec 1BA");
				//manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_DIVE);
				//test_ss_surfaceTimer.restart( 0, test_ss_surfaceTimer_release_period);

				firstSubsurface_release = false;
				//digitalWrite(LED_BUILTIN, HIGH);									// De-commenter ici pour extinction LED en subsurface

				return;
			}
	}
	else {

		//if(manager_surfaceSensor_diveDetection(surfaceSensorAlgo_threshold)){
		if(manager_depth_get_mm(pressureSensor_1BA, &depth_mm_surf) == psr_OK) {
			if( depth_mm_surf > (SURFACE_DEPTH_CM *10)){//enter in subsurface
				changeAppMode_release(app_subsurface_release);
				tsSubsurfacingTime_release = RTC.getEpoch();
				LOGLN("Detection subsurface par 1BA");
				//mainMessage_release.pureSurfaceTime_s += tsSubsurfacingTime_release - tsBackToSurfaceTime_release;

				//manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_DIVE);

				//digitalWrite(LED_BUILTIN, HIGH);							// De-commenter ici pour extinction LED en subsurface

				return;
			}
		}
	}
}

void appProcess_subsurface_release(void) {
	LOGLN("--- SUBSURFACE ---");
	uint32_t current_Time = RTC.getEpoch();

	int32_t depth_subsurf;

	pressureSensor_result_t res_subsurf;

	res_subsurf = manager_depth_get_mm(pressureSensor_1BA, &depth_subsurf);

	// Subsurface: on detecte les re-surfacages au 1BA (capteur surface desactive apres premier passage surface -> subsurface)

	//if(manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_threshold)){
	if(res_subsurf == psr_OK) {
		//surface detection only with the 1BA
		if(depth_subsurf < (SURFACE_DEPTH_CM *10)) {
			//back to the surface
			//mainMessage_release.backToSurfaceCount++;
			tsBackToSurfaceTime_release = current_Time;
			changeAppMode_release(app_surface_release);
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
			start_Diving_release();
			return;
		}
	}
	//nothing special to do in subsurface
}

void appProcess_surfacing_release(void) {
	LOGLN("--- SURFACING ---");

	//start the GPS
	if(!run_GNSS_release){
		manager_gnss_setState(GNSS_ON);
		//reset the state even if the GPS et not working
		last_EHPE_release = 10000;
		tsSurfacingTime_release = RTC.getEpoch();
		tsBackToSurfaceTime_release = tsSurfacingTime_release;

		if(manager_gnss_getState()) {
			run_GNSS_release = true;
			LOGLN("Start GPS");
		}
		else
			LOGLN("Start GPS - delayed");
	}
	//always change to surface mode
	changeAppMode_release(app_surface_release);
}

void appProcess_dive_release(void) {
	LOGLN("----- DIVE ------");

	//check surface triggers
	int32_t depth_mm_dive;

	pressureSensor_result_t res_dive = manager_depth_get_mm(pressureSensor_1BA, &depth_mm_dive) ;

	// TRIG SURFACE UNIQUEMENT AU CAPTEUR SURFACE

	if(manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_threshold)) {
		stop_Diving_release(SURFACE_TRIGGER_SENSOR_THRESOLD_release);
		//digitalWrite(LED_BUILTIN, LOW);
		return;
	}

	if(manager_surfaceSensor_surfaceDetection(surfaceSensorAlgo_gradient)) {
		stop_Diving_release(SURFACE_TRIGGER_SENSOR_GRADIENT_release);
		//digitalWrite(LED_BUILTIN, LOW);
		return;

	}
}

void start_Diving_release(void) {
	 // RTC.attachInterrupt(alarmMatch);
	//digitalWrite(LED_BUILTIN, HIGH);						// De-commenter ici pour extinction LED en dive

	changeAppMode_release(app_dive_release);
	tsDiveStartTime_release = RTC.getEpoch();

	//stop the GPS
	if(run_GNSS_release) {
		manager_gnss_setState(GNSS_OFF);
		if(manager_gnss_getState() == GNSS_OFF)
			run_GNSS_release = false;
		else
			LOGLN("STOP GPS - delayed");
	}

	manager_depth_calibrate30BA();

	manager_surfaceSensor_init(SURFACESENSOR_INITSTATE_DIVE);
	surfaceTimer_release.restart( 0, surfaceTimer_release_period);
	LOGLN("RESTART CAPTEUR SURFACE");

	dive_Zone_release = 0;

	firstSubsurface_release = true;
}

bool stop_Diving_release(uint8_t triggerId) {
	int32_t depth_;
	pressureSensor_result_t res_;

	res_ = manager_depth_get_mm(pressureSensor_1BA, &depth_);

	LOG("Surface trigged by : ");LOGLN(TriggerStr_release[triggerId]);
	//test_ss_surfaceTimer.stop();											// CA a enlever pour ne pas desactiver a la surface

	changeAppMode_release(app_surfacing_release);
	return true;
}

void appTickCallback_release(void) {
	flagTickMain_release = true;
	STM32L0.wakeup();
}

void surfaceTickCallback_release(void) {
	flagTickSurface_release = true;
	STM32L0.wakeup();
}

void changeAppMode_release(appMode_release_t mode) {
	application_Mode_release = mode;
	LOG("Mode change to : ");
	LOGLN(AppModeStr_release[application_Mode_release]);
}
