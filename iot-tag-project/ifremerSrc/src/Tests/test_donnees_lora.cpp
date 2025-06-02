/*
 * test_donnees_lora.cpp
 *
 *  Created on: 1 juil. 2020
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
#include "test_donnees_lora.h"

//---------------------------- DATALOG PARAMETERS --------------------------------------
#define pressure_sensor_sampling_time_donnees_lora 100
//--------------------------------------------------------------------------------------
TimerMillis pressure_sensor_timer_donnees_lora;  // instantiate high-frequency timer

extern I2Cdev             i2c_0;
extern SPIFlash SPIFlash;

uint32_t UID_donnees_lora[3] = {0, 0, 0};

// Battery voltage monitor definitions
float VDDA_donnees_lora, VBAT_donnees_lora, VBUS_donnees_lora, STM32L0Temp_donnees_lora;

// Cricket pin assignments
#define my_VBAT_donnees_lora_en  2 // enable VBat read
#define my_VBAT_donnees_lora    A1 // VBat analog read pin

volatile bool newPRESSURE_Data_donnees_lora = false;

// GNSS
GNSSLocation myPosition_donnees_lora;
TimerMillis timer_donnees_lora;
data_donnees_lora_t donnees_lora;

#define TICK_MAIN_donnees_lora	1000

bool run_GNSS_donnees_lora = false;
volatile bool flag_tick_donnees_lora = false;
static float last_EHPE_donnees_lora = 10000;


/**
    @brief  Main function of test_donnees_lora
    @param  None
    @retval None
 */

void test_donnees_lora() {

	delay(4000);
	Serial.begin(115200);
	delay(4000);

	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);  // start with blue led on (since active LOW)

	pinMode(my_VBAT_donnees_lora_en, OUTPUT);
	digitalWrite(my_VBAT_donnees_lora_en, LOW); // start with battery voltage monirot off
	pinMode(my_VBAT_donnees_lora, INPUT);
	analogReadResolution(12);

	// Set the RTC time to firmware build time
	manager_rtc_setDefaultTime();

	Serial.println("Serial enabled!");

	STM32L0.getUID(UID_donnees_lora);
	Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID_donnees_lora[0], HEX); Serial.print(UID_donnees_lora[1], HEX); Serial.println(UID_donnees_lora[2], HEX);

	VDDA_donnees_lora = STM32L0.getVDDA();
	VBUS_donnees_lora = STM32L0.getVBUS();
	digitalWrite(my_VBAT_donnees_lora_en, HIGH);
	VBAT_donnees_lora = 1.27f * VDDA_donnees_lora * analogRead(my_VBAT_donnees_lora) / 4096.0f;
	digitalWrite(my_VBAT_donnees_lora_en, LOW);
	STM32L0Temp_donnees_lora = STM32L0.getTemperature();

	// Internal STM32L0 functions
	Serial.print("VDDA = "); Serial.print(VDDA_donnees_lora, 2); Serial.println(" V");
	Serial.print("VBAT = "); Serial.print(VBAT_donnees_lora, 2); Serial.println(" V");
	if(VBUS_donnees_lora ==  1)  Serial.println("USB Connected!");
	Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp_donnees_lora, 2);
	Serial.println(" ");

	//SPI FLASH
	pinMode(csPin, OUTPUT); // set SPI chip select as L082 output
	digitalWrite(csPin, HIGH);

	// check SPI Flash ID
	SPIFlash.init();      // start SPI (include SPI.begin)
	SPIFlash.powerUp();   // MX25R6435FZAI defaults to power down state
	SPIFlash.getChipID(); // Verify SPI flash communication
	//SPIFlash.flash_chip_erase(1);
	//delay(15000);

	I2C_BUS.begin();                                      // Set master mode, default on SDA/SCL for STM32L4
	delay(1000);
	I2C_BUS.setClock(400000);                             // I2C frequency at 400 kHz
	delay(1000);

    // GNSS + LoRa init
    uint32_t start_time;
    //uint8_t first_fix = 0; //1 for first Fix
    //data_TTFF_t ttff_;

   	//const float EHPE_ = 200;

	//manager_gnss_init();

	LoRaWAN.begin(EU868);
	LoRaWAN.setADR(false);
	LoRaWAN.setDataRate(0);
	LoRaWAN.setTxPower(20);
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
    		digitalWrite(LED, HIGH);
    		delay(100);
    		digitalWrite(LED, LOW);
    		delay(100);
    	}
    	else
    		Serial.println("Lora busy");
			digitalWrite(LED, HIGH);
			delay(100);
			digitalWrite(LED, LOW);
			delay(100);
      	}
       	LOGLN("Join Success");
	    digitalWrite(LED, HIGH);
       	delay(20000);

    //manager_gnss_setState(GNSS_ON);
    //while(manager_gnss_getState() != GNSS_ON)
    	//manager_gnss_processState();

    //run_GNSS_donnees_lora = true;
    start_time = RTC.getEpoch();

    LOGLN("Start Timer");
    timer_donnees_lora.start(callback_tick_donnees_lora, TICK_MAIN_donnees_lora, TICK_MAIN_donnees_lora);
    // END GNSS + LoRA init

    // PRESSURE
    int32_t depth_1_donnees_lora = 0;
    int32_t depth_30_donnees_lora = 0;
    manager_depth_init();
    manager_depth_get_mm(pressureSensor_1BA, &depth_1_donnees_lora);
    manager_depth_get_mm(pressureSensor_30BA, &depth_30_donnees_lora);
    manager_depth_readPressure_mPa_1BA(&depth_1_donnees_lora);
    manager_depth_readPressure_mPa_30BA(&depth_30_donnees_lora);
    donnees_lora.depth_mm_1_donnees_lora = depth_1_donnees_lora;
    donnees_lora.depth_mm_30_donnees_lora = depth_30_donnees_lora;
    pressure_sensor_timer_donnees_lora.start(callback_pressureTimer_donnees_lora, 0,  pressure_sensor_sampling_time_donnees_lora);
    // END PRESSURE


	while (1) {

		Serial.println("------------------------------------------------------------------------------------------");
		Serial.print("\t");
		Serial.println("Le test concerne : la collecte des donnees de pression/temperature et l'envoi en LoRa");
		Serial.println("------------------------------------------------------------------------------------------");

		if(newPRESSURE_Data_donnees_lora == true){
			newPRESSURE_Data_donnees_lora = false;
			manager_depth_get_mm_temp(pressureSensor_1BA,&donnees_lora.depth_mm_1_donnees_lora,&donnees_lora.temp_1_donnees_lora);
			manager_depth_get_mm_temp(pressureSensor_30BA,&donnees_lora.depth_mm_30_donnees_lora,&donnees_lora.temp_30_donnees_lora);
			Serial.println(" ");
			LoRaWAN.sendPacket(loraPortNumber[ilp_GnssTracking], (uint8_t*)&donnees_lora, sizeof(data_donnees_lora_t));
			digitalWrite(LED, LOW);
			delay(1000);
			digitalWrite(LED, HIGH);
		}
		delay(14000);
	}
}

void callback_tick_donnees_lora(void) {
	flag_tick_donnees_lora = true;
}

void callback_pressureTimer_donnees_lora() {
	newPRESSURE_Data_donnees_lora = true;
}
