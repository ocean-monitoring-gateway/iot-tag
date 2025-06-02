/*
 * test_fix_lora.cpp
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
#include "test_fix_lora.h"


extern I2Cdev             i2c_0;
extern SPIFlash SPIFlash;

uint32_t UID_fix_lora[3] = {0, 0, 0};

// Battery voltage monitor definitions
float VDDA_fix_lora, VBAT_fix_lora, VBUS_fix_lora, STM32L0Temp_fix_lora;

// Cricket pin assignments
#define my_VBAT_fix_lora_en  2 // enable VBat read
#define my_VBAT_fix_lora    A1 // VBat analog read pin

// GNSS
GNSSLocation myPosition_fix_lora;
GNSSSatellites mySatellites_fix_lora;
TimerMillis timer_fix_lora;
data_gnssTracking_fix_lora_t gnss_Point_fix_lora;

#define TICK_MAIN_fix_lora	1000

bool run_GNSS_fix_lora = false;
volatile bool flag_tick_fix_lora = false;
static float last_EHPE_fix_lora = 10000;


/**
    @brief  Main function of test_fix_lora
    @param  None
    @retval None
 */

void test_fix_lora() {

	delay(4000);
	Serial.begin(115200);
	delay(4000);

	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);  // start with blue led on (since active LOW)

	pinMode(my_VBAT_fix_lora_en, OUTPUT);
	digitalWrite(my_VBAT_fix_lora_en, LOW); // start with battery voltage monirot off
	pinMode(my_VBAT_fix_lora, INPUT);
	analogReadResolution(12);

	// Set the RTC time to firmware build time
	manager_rtc_setDefaultTime();

	Serial.println("Serial enabled!");

	STM32L0.getUID(UID_fix_lora);
	Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID_fix_lora[0], HEX); Serial.print(UID_fix_lora[1], HEX); Serial.println(UID_fix_lora[2], HEX);

	VDDA_fix_lora = STM32L0.getVDDA();
	VBUS_fix_lora = STM32L0.getVBUS();
	digitalWrite(my_VBAT_fix_lora_en, HIGH);
	VBAT_fix_lora = 1.27f * VDDA_fix_lora * analogRead(my_VBAT_fix_lora) / 4096.0f;
	digitalWrite(my_VBAT_fix_lora_en, LOW);
	STM32L0Temp_fix_lora = STM32L0.getTemperature();

	// Internal STM32L0 functions
	Serial.print("VDDA = "); Serial.print(VDDA_fix_lora, 2); Serial.println(" V");
	Serial.print("VBAT = "); Serial.print(VBAT_fix_lora, 2); Serial.println(" V");
	if(VBUS_fix_lora ==  1)  Serial.println("USB Connected!");
	Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp_fix_lora, 2);
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
    uint32_t start_time, fix_time;
    uint8_t first_fix = 0; //1 for first Fix
    data_TTFF_t ttff_;
    data_gnssTracking_t gnss_Point;

   	const float EHPE_ = 200;

	manager_gnss_init();

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

    manager_gnss_setState(GNSS_ON);
    while(manager_gnss_getState() != GNSS_ON)
    	manager_gnss_processState();

    run_GNSS_fix_lora = true;
    start_time = RTC.getEpoch();

    LOGLN("Start Timer");
    timer_fix_lora.start(callback_tick_fix_lora, TICK_MAIN_fix_lora, TICK_MAIN_fix_lora);
    // END GNSS + LoRA init


	while (1) {

		Serial.println("------------------------------------------------------------------------------------------");
		Serial.print("\t");
		Serial.println("Le test concerne : le fix GPS et l'envoi en LoRa");
		Serial.println("------------------------------------------------------------------------------------------");

		if(flag_tick_fix_lora){
			flag_tick_fix_lora = false;
			LOG(".");

			if(!run_GNSS_fix_lora) {
				LOG("Starting GPS");
				manager_gnss_setState(GNSS_ON);
			    while(manager_gnss_getState() != GNSS_ON)
			    	manager_gnss_processState();

			    start_time = RTC.getEpoch();

			    run_GNSS_fix_lora = true;
			    timer_fix_lora.restart(TICK_MAIN_fix_lora, TICK_MAIN_fix_lora);
				LOG("Launch Main Tick");
			}

			else {
				GNSS.location(myPosition_fix_lora);
				LOGLN("Try to fix");
				GNSS.satellites(mySatellites_fix_lora);
				LOG("Fix ? "); LOGLN(fixGPS_fix_lora());
				delay(1000);
				// Decommenter ligne 187 et 206 si on veut envoyer que lorsqu'il y a un fix
//				if (fixGPS_fix_lora()) {
					LOG("EHPE : ");
					LOG(myPosition_fix_lora.ehpe());
					LOGLN("");
					uint32_t ctime = RTC.getEpoch();
					gnss_Point_fix_lora.lat_fix_lora = myPosition_fix_lora.latitude();
					LOG("LAT : ");
					LOG(myPosition_fix_lora.latitude());
					LOGLN("");
					gnss_Point_fix_lora.lng_fix_lora = myPosition_fix_lora.longitude();
					LOG("LONG : ");
					LOG(myPosition_fix_lora.longitude());
					LOGLN("");
					gnss_Point_fix_lora.ehpe_fix_lora = myPosition_fix_lora.ehpe();
					gnss_Point_fix_lora.nb_sat_fix_lora = mySatellites_fix_lora.count();
					LOG("NB SAT : ");
					Serial.println(gnss_Point_fix_lora.nb_sat_fix_lora);
					LoRaWAN.sendPacket(loraPortNumber[ilp_GnssTracking], (uint8_t*)&gnss_Point_fix_lora, sizeof(data_gnssTracking_fix_lora_t));
					digitalWrite(LED, LOW);
					delay(1000);
					digitalWrite(LED, HIGH);
					delay(179000);
//				}
			}
		}
	}
}

bool fixGPS_fix_lora(void) {
	return (myPosition_fix_lora.fixType() != GNSSLocation::TYPE_NONE) && (myPosition_fix_lora.fixType() != GNSSLocation::TYPE_TIME);
}

void callback_tick_fix_lora(void) {
	flag_tick_fix_lora = true;
}
