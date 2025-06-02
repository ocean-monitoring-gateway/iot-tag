/*
 * test_flash_read.cpp
 *
 *  Created on: 4 mai 2020
 *      Author: jnguyen
 */

#include "SPIFlash.h"
#include "test_flash_read.h"

extern SPIFlash SPIFlash;

uint8_t   flash_Page_f[256];          // array to hold the data for flash page write
uint16_t  page_number_f = 0;

void test_flash_read() {
	delay(4000);
	Serial.begin(115200);
	Serial.println("Debut");
	delay(4000);

	//SPI FLASH
	pinMode(25, 0x1); // set SPI chip select as L082 output
	digitalWrite(25, 0x1);
	SPIFlash.init();
	SPIFlash.powerUp();

	for (uint16_t j=0; j<32768; j++){
		Serial.print("Page n ");
		Serial.println(j);
		for (uint16_t k=0; k<256; k++){
			SPIFlash.flash_read_pages(flash_Page_f, j, 1);
			Serial.println(flash_Page_f[k]);
		}
	}

	Serial.println("Fin");
}

