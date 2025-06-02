/* 13/01/2020 Pierre Gogendeau

  Libraries of the functions using the flash memory

*/
#include "../_board.h"
#include "flash_functions.h"

extern SPIFlash SPIFlash;
extern TimerMillis pressure_sensor_timer;


unsigned char flash_wait_for_write = 0;


/**
    @brief  Function to erase the flash
    @param  None
    @retval None
*/


/**
    @brief  function used in the test_accelero_1 to log in flash memory
    @param  None
    @retval None
*/


/*void write_flash_test_accelero_1(uint16_t *pPage_number , uint8_t *pIndex_page_number, uint16_t last_page, data_flash_t *data_flash) {

  uint8_t flashPage[256];      // array to hold the data for flash page write
  uint16_t page_number = *pPage_number;
  uint8_t  index_page_number = *pIndex_page_number;


  // 32,768 256-byte pages in a 8 MByte flash
  // 42 valeurs accelero -> 252 octets -> ~ 1 page
  //store RAW accelData (3 x 2 octets)
  //mise en forme + bias au dump

  //depth_mm = (int32_t*)(&flashPage[ index_page_number * 16 + 12]) = ;

  //Serial.print("Depth mm 2 :");
  //Serial.println(depth_value_mm.fval);

  memcpy(flashPage, &data_flash, sizeof data_flash);


  //depth_mm = (int32_t*)(&flashPage[ index_page_number * 16 + 12]);

  if ( index_page_number < 15 && page_number < last_page)   // on remplit la page
  {
    index_page_number++;
  }
  else if ( index_page_number == 15 && page_number < last_page) // on Ã©crit la page
  {
    digitalWrite(LED, LOW); //led on
    SPIFlash.flash_page_program(flashPage, page_number);  // Temps d'Ã©criture mesurÃ© : 1 ms
    //Serial.print("***Wrote flash page: "); Serial.println(page_number);
    index_page_number = 0;
    page_number++;
    digitalWrite(LED, HIGH); //led off
  }
  else  // fin de log
  {
    //getRTC(1); //timestamp

    detachInterrupt(LSM303AGR_A_intPin1);  // detach interrupt to stay in stop mode
    detachInterrupt(LSM303AGR_A_intPin2);  // detach interrupt to stay in stop mode
    detachInterrupt(LSM303AGR_M_intPin);  // detach interrupt to stay in stop mode

    pressure_sensor_timer.stop();
    //LSM303AGR.reset();
    //LIS2MDL.reset();

    Serial.print("DataLog started : ");
    //printRTC(0); //timestamp
    Serial.print("DataLog stopped : ");
    //printRTC(1); //timestamp

    // record number of written pages in page 1
    flashPage[0]  = (page_number >> 8) & 0x00FF;  // MSB derniÃ¨re page
    flashPage[1]  = page_number & 0x00FF;         // LSB derniÃ¨re page

    SPIFlash.flash_page_program(flashPage, 1); // write page 1
    //Serial.println("Page 1 written.");
    //Serial.print("Total used pages = ");Serial.print(page_number); Serial.println("\r\n");
    Serial.print("\r\nTotal Measurements : "); Serial.print(((uint32_t)page_number - 2) * 15); Serial.println("\r\n");

    //Serial.print("Start Log: ");
    //printRTC(0); //timestamp
    //Serial.print("End Log:   ");
    //printRTC(1); //timestamp

    digitalWrite(LED, HIGH); // turn off led

    
  }

  *pPage_number = page_number;
  *pIndex_page_number = index_page_number;

}*/


/**
    @brief  function used in the test_accelero_1 to log in flash memory
    @param  None
    @retval None
*/
void write_flash_paramater_init_test_accelero_1(uint16_t  max_last_page, uint8_t AODR, uint8_t MODR, uint8_t Ascale) {

  /*uint8_t flashPage[256];      // array to hold the data for flash page write

  // Write default number of used pages, aRes and biases in SpiFlash page 0
  flashPage[0]  = (max_last_page >> 8) & 0x00FF;  // MSB page max
  flashPage[1]  = max_last_page & 0x00FF;         // LSB page max

  Serial.print("Ares float: ");Serial.println(aRes.fval);

  flashPage[2]  = aRes.bval[0];
  flashPage[3]  = aRes.bval[1];
  flashPage[4]  = aRes.bval[2];
  flashPage[5]  = aRes.bval[3];

  Serial.print("accel bias 0 float: ");Serial.println(accelBias[0].fval);

  flashPage[6]  = accelBias[0].bval[0];
  flashPage[7]  = accelBias[0].bval[1];
  flashPage[8]  = accelBias[0].bval[2];
  flashPage[9]  = accelBias[0].bval[3];

  Serial.print("accel bias 1 float: ");Serial.println(accelBias[1].fval);

  flashPage[10]  = accelBias[1].bval[0];
  flashPage[11]  = accelBias[1].bval[1];
  flashPage[12]  = accelBias[1].bval[2];
  flashPage[13]  = accelBias[1].bval[3];

  Serial.print("accel bias 2 float: ");Serial.println(accelBias[2].fval);

  flashPage[14]  = accelBias[2].bval[0];
  flashPage[15]  = accelBias[2].bval[1];
  flashPage[16]  = accelBias[2].bval[2];
  flashPage[17]  = accelBias[2].bval[3];

  flashPage[18]  = AODR;
  flashPage[19]  = Ascale;

  flashPage[20]  = magBias[0].bval[0];
  flashPage[21]  = magBias[0].bval[1];
  flashPage[22]  = magBias[0].bval[2];
  flashPage[23]  = magBias[0].bval[3];

  flashPage[24]  = magBias[1].bval[0];
  flashPage[25]  = magBias[1].bval[1];
  flashPage[26]  = magBias[1].bval[2];
  flashPage[27]  = magBias[1].bval[3];

  flashPage[28]  = magBias[2].bval[0];
  flashPage[29]  = magBias[2].bval[1];
  flashPage[30]  = magBias[2].bval[2];
  flashPage[31]  = magBias[2].bval[3];

  flashPage[32]  = MODR;

  SPIFlash.flash_page_program(flashPage, 0); // write page zero

  flashPage[10]  = magScale[0].bval[0];
  flashPage[11]  = magScale[0].bval[1];
  flashPage[12]  = magScale[0].bval[2];
  flashPage[13]  = magScale[0].bval[3];

  flashPage[14]  = magScale[1].bval[0];
  flashPage[15]  = magScale[1].bval[1];
  flashPage[16]  = magScale[1].bval[2];
  flashPage[17]  = magScale[1].bval[3];

  flashPage[18]  = magScale[2].bval[0];
  flashPage[19]  = magScale[2].bval[1];
  flashPage[20]  = magScale[2].bval[2];
  flashPage[21]  = magScale[2].bval[3];

  SPIFlash.flash_page_program(flashPage, 1); // write page zero*/

}

void read_flash_test_accelero_1(){

 
}

void eraseFlash(uint16_t  last_page){
	// Erase Flash
		if (last_page > 27072) // Full erase plus rapide que page par page
		{
			//Serial.println("Start Full Erase (2 mn)");
			SPIFlash.flash_chip_erase(1);
		}
		else
		{
			// WARNING : efface le secteur (4ko = 16 pages) qui contient le N° de page...
			//SPIFlash.flash_erase_pages_sector((int)5); // efface les pages 0 à 15
			for (int i=15; i <= (last_page / 16) * 16 + 15; i+=16)
			{
				//Serial.print(i); Serial.println("");
				// WARNING : efface le secteur (4ko = 16 pages) qui contient le N° de page...
				//Exemple: SPIFlash.flash_erase_pages_sector((int)7); Efface les pages 0 à 15
				SPIFlash.flash_erase_pages_sector((int)i);
			}
		}
		//Serial.println("Erase done.\r\n");
}







/*********************************************************************************************/
// Functions from Tlera corp
/*********************************************************************************************/


void write_pause()
{
  if (flash_wait_for_write) {
    while (flash_read_status() & STAT_WIP);
    flash_wait_for_write = 0;
  }
}

//=====================================
// convert a page number to a 24-bit address
int page_to_address(int pn)
{
  return (pn << 8);
}

//=====================================
// convert a 24-bit address to a page number
int address_to_page(int addr)
{
  return (addr >> 8);
}

//=====================================
void flash_read_id(unsigned char *idt)
{
  write_pause();
  //set control register
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  SPI.transfer(CMD_READ_ID);
  for (uint16_t i = 0; i < 20; i++) {
    *idt++ = SPI.transfer(0x00);
  }
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}

//=====================================
unsigned char flash_read_status()
{
  unsigned char c;

  // This can't do a write_pause
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  SPI.transfer(CMD_READ_STATUS_REG);
  c = SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return (c);
}

//=====================================

void flash_hard_reset()
{
  // Make sure that the device is not busy before
  // doing the hard reset sequence
  // At the moment this does NOT check the
  // SUSpend status bit in Status Register 2
  // but the library does not support suspend
  // mode yet anyway
  write_pause();

  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  SPI.transfer(CMD_RESET_DEVICE );
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  delayMicroseconds(50);
  // Wait for the hard reset to finish
  // Don't use flash_wait_for_write here
  while (flash_read_status() & STAT_WIP);
  // The spec says "the device will take
  // approximately tRST=30 microseconds
  // to reset"
}

//=====================================
void flash_chip_erase(boolean wait)
{
  write_pause();
  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  SPI.transfer(CMD_WRITE_ENABLE);
  digitalWrite(csPin, HIGH);
  digitalWrite(csPin, LOW);
  SPI.transfer(CMD_CHIP_ERASE);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  flash_wait_for_write = 1;
  if (wait)write_pause();
}

//=====================================
// Tse Typ=0.6sec Max=3sec
// measured 549.024ms
// Erase the sector which contains the specified
// page number.
// The smallest unit of memory which can be erased
// is the 4kB sector (which is 16 pages)
void flash_erase_pages_sector(int pn)
{
  int address;

  write_pause();
  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  SPI.transfer(CMD_WRITE_ENABLE);
  digitalWrite(csPin, HIGH);

  digitalWrite(csPin, LOW);
  SPI.transfer(CMD_SECTOR_ERASE);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xff);
  SPI.transfer((address >> 8) & 0xff);
  SPI.transfer(address & 0xff);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  // Indicate that next I/O must wait for this write to finish
  flash_wait_for_write = 1;
}

//=====================================
// Erase the 32kb block which contains the specified
// page number.
void flash_erase_pages_block32k(int pn)
{
  int address;

  write_pause();
  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  SPI.transfer(CMD_WRITE_ENABLE);
  digitalWrite(csPin, HIGH);
  digitalWrite(csPin, LOW);
  SPI.transfer(CMD_BLOCK32K_ERASE);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  // Indicate that next I/O must wait for this write to finish
  flash_wait_for_write = 1;
}

//=====================================
// Erase the 64kb block which contains the specified
// page number.
void flash_erase_pages_block64k(int pn)
{
  int address;

  write_pause();
  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  SPI.transfer(CMD_WRITE_ENABLE);
  digitalWrite(csPin, HIGH);
  digitalWrite(csPin, LOW);
  SPI.transfer(CMD_BLOCK64K_ERASE);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  // Indicate that next I/O must wait for this write to finish
  flash_wait_for_write = 1;
}

//=====================================
void flash_page_program(unsigned char *wp, int pn)
{
  int address;

  write_pause();
  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  SPI.transfer(CMD_WRITE_ENABLE);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();

  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  SPI.transfer(CMD_PAGE_PROGRAM);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  // Now write 256 bytes to the page
  for (uint16_t i = 0; i < 256; i++) {
    SPI.transfer(*wp++);
  }
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  // Indicate that next I/O must wait for this write to finish
  flash_wait_for_write = 1;
}

//=====================================
void flash_read_pages(unsigned char *p, int pn, const int n_pages)
{
  int address;
  unsigned char *rp = p;

  write_pause();
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  SPI.transfer(CMD_READ_DATA);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  // Now read the page's data bytes
  for (uint16_t i = 0; i < n_pages * 256; i++) {
    *rp++ = SPI.transfer(0);
  }
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}

//=====================================
// Read specified number of pages starting with pn
void flash_fast_read_pages(unsigned char *p, int pn, const int n_pages)
{
  int address;
  unsigned char *rp = p;

  write_pause();
  // The chip doesn't run at the higher clock speed until
  // after the command and address have been sent
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  SPI.transfer(CMD_READ_HIGH_SPEED);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  // send dummy byte
  SPI.transfer(0);
  // Now read the number of pages required
  for (uint16_t i = 0; i < n_pages * 256; i++) {
    *rp++ = SPI.transfer(0);
  }
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}
