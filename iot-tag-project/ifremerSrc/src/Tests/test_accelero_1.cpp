/* 09/01/2020 Pierre Gogendeau

  Libraries of the for testing the accelerometer

 */

#include "../Manager/_manager.h"
#include "../_board.h"
#include "SPIFlash.h"
#include "LSM303AGR_M.h"
#include "LSM303AGR_A.h"
#include "test_accelero_1.h"

//---------------------------- DATALOG PARAMETERS --------------------------------------
#define logTime 1        //durée de log en minutes
#define logFreq 100      //frequence de mesure en Hz ( 10, 100)
#define FullScale 4      //pleine echelle en G (2, 4, 8, 16)
#define pressure_sensor_sampling_time 100

//--------------------------------------------------------------------------------------

// SPI Flash: 64 MBit (8 MByte) SPI Flash 32,768, 256-byte pages
//page 0 and 1 reserved, accelero values start at page 2)
uint16_t  max_last_page = 0x7FFF;  // lower it if you want to reserve some pages for other use...
uint32_t  force_last_page;         // force last usable page number (max 32767 or 0x7FFF) for debug (no stop button), min = 3 (2 config + 1 values). <3 => disable
uint16_t  page_number;
uint16_t  last_page;
uint8_t   flashPage[256];          // array to hold the data for flash page write
uint8_t   index_page_number=0;         // compteur de triplets dans une page (max 42 = 252 octets)
uint16_t  i;
uint8_t index_first_param = 2;

TimerMillis pressure_sensor_timer;  // instantiate high-frequency timer

extern I2Cdev             i2c_0;
extern SPIFlash SPIFlash;

uint32_t UID[3] = {0, 0, 0};

bool SerialDebug = true;

uint8_t hours[4] = {12,12,12,12}, minutes[4] = {0,0,0,0}, seconds[4] = {0,0,0,0}, year = 1, month = 1, day = 1;
uint32_t subSeconds[4], milliseconds;

// battery voltage monitor definitions
float VDDA, VBAT, VBUS, STM32L0Temp;


float param_accelBias[3] = {0.0f, 0.0f, 0.0f}; // offset biases copy for fonction call
int16_t accelData[3], accTempData;  // Stores the 10-bit signed accel output if normal mode
float ax, ay, az;                   // variables to hold latest accel data values


//LSM303AGR_A LSM303AGR_A(&i2c_0); // instantiate LSM303AGR accel class
extern LSM303AGR_A LSM303AGR_A;

//volatile bool newLSM303AGR_MData = false; // LSM303AGR_M magnetometer interrupt flag
volatile bool newLSM303AGR_AData = false; // used for data ready interrupt handling
volatile bool newLSM303AGR_Aactivity  = false; // used for activity interrupt handling

/* Specify sensor parameters (sample rate is twice the bandwidth)
   choices are: MODR_10Hz, MOIDR_20Hz, MODR_50 Hz and MODR_100Hz
 */

uint8_t MODR;
float param_magBias[3] = {0.0f, 0.0f, 0.0f}, param_magScale[3]  = {0.0f, 0.0f, 0.0f}; // Bias corrections for magnetometer
int16_t magData[4];              // Stores the 16-bit signed sensor output
float magTemperature;            // Stores the real internal chip temperature in degrees Celsius
float mx, my, mz;                // variables to hold latest mag data values
uint8_t LSM303AGR_Mstatus;

volatile bool newLSM303AGR_MData = false; // LIS2MDL magnetometer interrupt flag

//LSM303AGR_M LSM303AGR_M(&i2c_0); // instantiate LSM303AGR_M class
extern LSM303AGR_M LSM303AGR_M;

volatile bool newPRESSUREData = false;

data_flash_t data;
data_param_flash_t param_data;



/**
    @brief  Init accelerometer et magnetometer
    @param  None
    @retval None
 */

//void init_test_accelero_1(LSM303AGR_M LSM303AGR_M, uint8_t magFreq, LSM303AGR_A LSM303AGR_A, uint8_t Ascale, uint8_t accelFreq, SPIFlash SPIFlash, uint8_t MODR, uint8_t AODR) {

//}

/**
    @brief  Main function of test_accelero_1
    @param  None
    @retval None
 */

void test_accelero_1() {


	uint8_t nb_byte_write = sizeof(data);
	uint8_t nb_sector_page = 254/nb_byte_write;

	Serial.print("Nb sectore per page :");
	Serial.println(nb_sector_page);

	//init_test_accelero_1(LSM303AGR_M, magFreq, LSM303AGR_A, Ascale, accelFreq, SPIFlash, MODR, AODR);
	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);  // start with blue led on (since active LOW)

	// Set the RTC time to firmware build time
	manager_rtc_setDefaultTime();

	Serial.begin(115200);
	delay(2000);
	Serial.println("Serial enabled!");

	STM32L0.getUID(UID);
	Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX);



	VDDA = STM32L0.getVDDA();
	VBUS = STM32L0.getVBUS();
	/*digitalWrite(myVBat_en, HIGH);
	VBAT = 1.27f * VDDA * analogRead(myVBat) / 4096.0f;
	digitalWrite(myVBat_en, LOW);*/
	STM32L0Temp = STM32L0.getTemperature();

	// Internal STM32L0 functions
	Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
	Serial.print("VBAT = "); Serial.print(VBAT, 2); Serial.println(" V");
	if(VBUS ==  1)  Serial.println("USB Connected!");
	Serial.print("STM32L0 MCU Temperature = "); Serial.println(STM32L0Temp, 2);
	Serial.println(" ");


	//SPI FLASH
	pinMode(csPin, OUTPUT); // set SPI chip select as L082 output
	digitalWrite(csPin, HIGH);

	// check SPI Flash ID
	SPIFlash.init();      // start SPI (include SPI.begin)
	SPIFlash.powerUp();   // MX25R6435FZAI defaults to power down state
	SPIFlash.getChipID(); // Verify SPI flash communication

	delay(15000);

	//dump SpiFlash si usb connecté
	if(VBUS ==  1)
	{
		// Read max page, aRes and biases in SpiFlash page 0
		SPIFlash.flash_read_pages(flashPage, 0, 1);

		last_page = (flashPage[0] << 8) | flashPage[1];
		if (last_page > max_last_page) //numero invalide ou flash vide (tout à 1), ou zone réservée
		{
			last_page = max_last_page;
		}

		data_param_flash_t *pParam_flash_read;

		pParam_flash_read = (data_param_flash_t*)(&flashPage[index_first_param]);

		//printAscale();
		//printAODR();
		Serial.print("aRes "); Serial.println(pParam_flash_read->aRes, HEX);
		Serial.println("accel biases (mg)"); Serial.println(1000.0f * pParam_flash_read->accelBias[0]); Serial.println(1000.0f * pParam_flash_read->accelBias[1]); Serial.println(1000.0f * pParam_flash_read->accelBias[2]);
		Serial.println("mag biases "); Serial.println(1000.0f * pParam_flash_read->magBias[0]); Serial.println(1000.0f * pParam_flash_read->magBias[1]); Serial.println(1000.0f * pParam_flash_read->magBias[2]);
		Serial.println("mag scale "); Serial.println(1000.0f * pParam_flash_read->magScale[0]); Serial.println(1000.0f * pParam_flash_read->magScale[1]); Serial.println(1000.0f * pParam_flash_read->magScale[2]);


		// Read number of used pages in SpiFlash page 1
		SPIFlash.flash_read_pages(flashPage, 1, 1);

		page_number = (flashPage[0] << 8) | flashPage[1];
		if ((page_number != 0xFFFF) && (page_number <= max_last_page)) //numero valide
		{
			last_page = page_number;
		}


		//Debug until button to stop recording
		force_last_page = ((uint32_t)logFreq * (uint32_t)logTime * 60) / nb_sector_page + index_first_param;
		if (force_last_page > 2)
		{
			if (force_last_page > 0x7FFF)
			{
				force_last_page = 0x7FFF;
			}
			last_page = (uint16_t)force_last_page;
			Serial.print("Last page force "); Serial.println(last_page); Serial.println("");
		}

		Serial.println("SpiFLASH dump start in 5s...\r\n");
		delay(5000);

		for (page_number = index_first_param; page_number < last_page; page_number++)
		{
			digitalWrite(LED, LOW); delay(1); digitalWrite(LED, HIGH);
			//Serial.print("Read Page 0x"); Serial.println(page_number, HEX);  //debug
			SPIFlash.flash_read_pages(flashPage, page_number, 1);

			for (index_page_number = 0; index_page_number < nb_sector_page; index_page_number++)
			{

				data_flash_t *pData_flash_read;

				pData_flash_read = (data_flash_t*)(&flashPage[index_page_number * 16]);

				/*ax = (float)pData_flash_read->accelData[0] * pParam_flash_read->aRes - pParam_flash_read->accelBias[0];
				ay = (float)pData_flash_read->accelData[1] * pParam_flash_read->aRes - pParam_flash_read->accelBias[1];
				az = (float)pData_flash_read->accelData[2] * pParam_flash_read->aRes - pParam_flash_read->accelBias[2];

				mx = (float)pData_flash_read->magData[0] * pParam_flash_read->mRes - pParam_flash_read->magBias[0]; // get actual G value
				mx = (float)pData_flash_read->magData[1] * pParam_flash_read->mRes - pParam_flash_read->magBias[1];
				mx = (float)pData_flash_read->magData[2] * pParam_flash_read->mRes - pParam_flash_read->magBias[2];
				mx *= pParam_flash_read->magScale[0];
				my *= pParam_flash_read->magScale[1];
				mz *= pParam_flash_read->magScale[2];*/


				Serial.print(pData_flash_read->accelData[0]); Serial.print(";");
				Serial.print(pData_flash_read->accelData[0]); Serial.print(";");
				Serial.print(pData_flash_read->accelData[0]);
				Serial.print(" ; ");
				Serial.print(pData_flash_read->magData[0]); Serial.print(";");
				Serial.print(pData_flash_read->magData[1]); Serial.print(";");
				Serial.print(pData_flash_read->magData[2]);
				Serial.print(" ; ");
				Serial.println(pData_flash_read->depth_mm);
				Serial.print("\r\n");

			}
		}

		//Serial.print("Total used pages = ");Serial.print(page_number); Serial.println("\r\n");
		Serial.print("\r\nTotal Measurements : "); Serial.print(((uint32_t)page_number - 2) * 21);

		Serial.println("\r\nEnd of SpiFLASH dump");

		Serial.print("\r\nSpiFlash erase in 10s...");
		delay(10000);
		Serial.print("Erasing...");
		digitalWrite(LED, LOW); //led on
	} // fin dump si usb connecté
	else
	{
		last_page = max_last_page;

		//Debug until button to stop recording
		force_last_page = ((uint32_t)logFreq * (uint32_t)logTime * 60) / nb_sector_page + index_first_param;;
		if (force_last_page > 2)
		{
			if (force_last_page > 0x7FFF)
			{
				force_last_page = 0x7FFF;
			}
			last_page = (uint16_t)force_last_page;
			Serial.print("Last page force "); Serial.println(last_page); Serial.println("\r\n");
		}
	}

	if (VBUS == 1){

		eraseFlash(last_page);

	}

	page_number = index_first_param;      // reset first values page for main loop
	index_page_number = 0;  // reset first value number for main loop

	// set up Accelero
	pinMode(LSM303AGR_A_intPin1, INPUT);
	pinMode(LSM303AGR_M_intPin, INPUT);    // set up interrupt pins
	pinMode(LSM303AGR_A_intPin2, INPUT);

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

	//if(c == 0x33) // check if I2C sensor has acknowledged
	//{
	//Serial.println("LSM303AGR is online..."); Serial.println(" ");

	switch (logFreq)
	{
	case 1:
		param_data.AODR = AODR_1Hz;
		break;
	case 10:
		param_data.AODR = AODR_10Hz;
		param_data.MODR = MODR_10Hz;
		break;
	case 25:
		param_data.AODR = AODR_25Hz;
		break;
	case 50:
		param_data.AODR = AODR_50Hz;
		break;
	case 100:
		param_data.AODR = AODR_100Hz;
		param_data.MODR = MODR_100Hz;
		break;
	default:
		param_data.AODR = AODR_10Hz;
		param_data.MODR = MODR_10Hz;
		break;
	}

	switch (FullScale)
	{
	case 2:
		param_data.Ascale = AFS_2G;
		break;
	case 4:
		param_data.Ascale = AFS_4G;
		break;
	case 8:
		param_data.Ascale = AFS_8G;
		break;
	case 16:
		param_data.Ascale = AFS_16G;
		break;
	default:
		param_data.Ascale = AFS_4G;
		break;
	}


	param_data.aRes = LSM303AGR_A.getAres(param_data.Ascale); // get sensor resolution, only need to do this once

	blink_led(3);
	delay(5000);
	LSM303AGR_A.selfTest();
	LSM303AGR_A.reset();
	LSM303AGR_A.init(param_data.Ascale, param_data.AODR);

	LSM303AGR_A.offsetBias(param_accelBias);

	param_data.accelBias[0] = param_accelBias[0];
	param_data.accelBias[1] = param_accelBias[1];
	param_data.accelBias[2] = param_accelBias[2];

	Serial.println("accel biases (mg)");
	Serial.println(1000.0f * param_data.accelBias[0]);
	Serial.println(1000.0f * param_data.accelBias[1]);
	Serial.println(1000.0f * param_data.accelBias[2]);

	param_data.mRes = 0.0015f;  // fixed sensitivity and full scale (+/- 49.152 Gauss);

	blink_led(5);
	delay(5000);
	LSM303AGR_M.selfTest();
	LSM303AGR_M.reset(); // software reset LIS2MDL to default registers
	LSM303AGR_M.init(param_data.MODR);

	LSM303AGR_M.offsetBias(param_magBias, param_magScale);

	param_data.magBias[0] = param_magBias[0];
    param_data.magBias[1] = param_magBias[1];
    param_data.magBias[2] = param_magBias[2];
    param_data.magScale[0] = param_magScale[0];
    param_data.magScale[1] = param_magScale[1];
    param_data.magScale[2] = param_magScale[2];

	Serial.println("mag biases (mG)"); Serial.println(1000.0f * param_data.magBias[0]); Serial.println(1000.0f * param_data.magBias[1]); Serial.println(1000.0f * param_data.magBias[2]);
	Serial.println("mag scale (mG)"); Serial.println(1000.0f * param_data.magScale[0]); Serial.println(1000.0f * param_data.magScale[1]); Serial.println(1000.0f * param_data.magScale[2]);

	// Write default number of used pages, aRes and biases in SpiFlash page 0
	flashPage[0]  = (max_last_page >> 8) & 0x00FF;  // MSB page max
	flashPage[1]  = max_last_page & 0x00FF;         // LSB page max

	memcpy(&flashPage[index_first_param], &param_data, sizeof(param_data));


	SPIFlash.flash_page_program(flashPage, 0); // write page zero
	//Serial.println("Page 0 written.");

	/*else
			{
				if(c != 0x33) Serial.println(" LSM303AGR_A not functioning!"); // otherwise there is a problem somewhere
				if(d != 0x40) Serial.println("LSM303AGR_M not functioning!");
				while(1){};
			}
	 */

	digitalWrite(LED, HIGH); // turn off led when configuration successfully completed

	attachInterrupt(LSM303AGR_A_intPin2, myinthandler2, RISING);  // define double click interrupt for intPin2 output of LSM303AGR
	newLSM303AGR_Aactivity = false;
    int32_t depth_m = 0;
	manager_depth_init();
	manager_depth_get_mm(pressureSensor_1BA, &depth_m);
	data.depth_mm = depth_m;
	//Serial.println("Depth : ");
	//Serial.println(data.depth_mm);

	//getRTC(0); //timestamp

	//Serial.println("Waiting tap");


	while (1) {


		// When exiting STOP mode, re-enable the SPI peripheral
		/*SPIFlash.init();      // start SPI (include SPI.begin)
		SPIFlash.powerUp();   // MX25R6435FZAI defaults to power down state*/

		if (newLSM303AGR_Aactivity == true)
		{

			Serial.println("\r\nStarting DataLog...\r\n");

			newLSM303AGR_Aactivity = false;
			Serial.println("double tap");
			attachInterrupt(LSM303AGR_A_intPin1, myinthandler1, RISING);  // define data ready interrupt for intPin1 output of LSM303AGR
			attachInterrupt(LSM303AGR_M_intPin ,   myinthandler3, RISING);  // define data ready interrupt for intPin  output of LIS2MDL

			pressure_sensor_timer.start(callbackPressureTimer, 0,  pressure_sensor_sampling_time);    // high freq (one minute) timer

			detachInterrupt(LSM303AGR_A_intPin2);  // define double click interrupt for intPin2 output of LSM303AGR
			//LSM303AGR.doubleClickToMotion();
			//attachInterrupt(LSM303AGR_intPin2, myinthandler2, RISING);  // define double click interrupt for intPin2 output of LSM303AGR

			LSM303AGR_A.readAccData(accelData); // INT1 cleared on any read
			LSM303AGR_M.readData(magData);  // read data register to clear interrupt before main loop

			digitalWrite(LED, LOW); // turn on led when configuration successfully completed

			getRTC(0); //timestamp
		}

		// When exiting STOP mode, re-enable the SPI peripheral
		SPIFlash.init();      // start SPI (include SPI.begin)
		SPIFlash.powerUp();   // MX25R6435FZAI defaults to power down state

		if(newPRESSUREData == true){
			newPRESSUREData = false;
			manager_depth_get_mm(pressureSensor_1BA,&data.depth_mm);
		}


		if(newLSM303AGR_AData == true) // on interrupt, read data
		{
			digitalWrite(LED, HIGH);    // turn off led
			newLSM303AGR_AData = false;     // reset newData flag

			LSM303AGR_A.readAccData(data.accelData); // INT1 cleared on any read

			// Now we'll calculate the accleration value into actual g's
			/*ax = (float)data.accelData[0]*param_data.aRes - param_data.accelBias[0];  // get actual g value, this depends on scale being set
			ay = (float)data.accelData[1]*param_data.aRes - param_data.accelBias[1];
			az = (float)data.accelData[2]*param_data.aRes - param_data.accelBias[2];*/

			//    Serial.println(accelData[0], HEX);
			//    Serial.println(accelData[1], HEX);
			//    Serial.println(accelData[2], HEX);
			//Serial.print((int)1000*ax); Serial.print(";");
			//Serial.print((int)1000*ay); Serial.print(";");
			//Serial.print((int)1000*az);
			//    Serial.print(" Value_number="); Serial.println(index_page_number);  //debug
			//    Serial.print("\r\n");

			if ( newLSM303AGR_MData == true) {
				newLSM303AGR_MData = false;     // reset newData flag*/

				LSM303AGR_Mstatus = LSM303AGR_M.status();

				if (LSM303AGR_Mstatus & 0x08) // if all axes have new data ready
				{
					LSM303AGR_M.readData(data.magData);

					// Now we'll calculate the accleration value into actual G's
					/*mx = (float)data.magData[0] * param_data.mRes - param_data.magBias[0]; // get actual G value
					my = (float)data.magData[1] * param_data.mRes - param_data.magBias[1];
					mz = (float)data.magData[2] * param_data.mRes - param_data.magBias[2];
					mx *= param_data.magScale[0];
					my *= param_data.magScale[1];
					mz *= param_data.magScale[2];*/
				}
			}

			// 32,768 256-byte pages in a 8 MByte flash
			// 42 valeurs accelero -> 252 octets -> ~ 1 page
			//store RAW accelData (3 x 2 octets)
			//mise en forme + bias au dump


			memcpy(&flashPage[index_page_number*16], &data, sizeof data);


			if ( index_page_number < nb_sector_page && page_number < last_page)   // on remplit la page
			{
				index_page_number++;
			}
			else if ( index_page_number == nb_sector_page && page_number < last_page) // on Ã©crit la page
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
				Serial.print("\r\nTotal Measurements : "); Serial.print(((uint32_t)page_number - index_first_param) * nb_sector_page); Serial.println("\r\n");

				//Serial.print("Start Log: ");
				//printRTC(0); //timestamp
				//Serial.print("End Log:   ");
				//printRTC(1); //timestamp

				digitalWrite(LED, HIGH); // turn off led


			}


			/*SPIFlash.powerDown();  // Put SPI flash into power down mode
		SPI.end();             // End SPI peripheral to save power in STOP mode
		STM32L0.stop();        // Enter STOP mode and wait for an interrupt*/
		}
	}
}





/**
    @brief  interrupt handler for the accelerometer interuption 1
    @param  None
    @retval None
 */

void myinthandler1()
{
	newLSM303AGR_AData = true;
	STM32L0.wakeup();
}

/**
    @brief  interrupt handler for the accelerometer interuption 2
    @param  None
    @retval None
 */

void myinthandler2()
{

	newLSM303AGR_Aactivity = true;
	STM32L0.wakeup();
}

/**
    @brief  interrupt handler for the magnetometer sinteruption 1
    @param  None
    @retval None
 */

void myinthandler3()
{
	newLSM303AGR_MData = true;
	STM32L0.wakeup();
}


void callbackPressureTimer() {
	newPRESSUREData = true;
}


/*void printAODR()
{
	switch (AODR)
	{
	case 0x01:
		Serial.println("AODR = 1Hz");
		break;
	case 0x02:
		Serial.println("AODR = 10Hz");
		break;
	case 0x03:
		Serial.println("AODR = 25Hz");
		break;
	case 0x04:
		Serial.println("AODR = 50Hz");
		break;
	case 0x05:
		Serial.println("AODR = 100Hz");
		break;
	default:
		Serial.println("AODR Error");
		break;
	}
}*/


/*void printAscale()
{
	switch (Ascale)
	{
	case 0:
		Serial.println("Ascale = 2G");
		break;
	case 1:
		Serial.println("Ascale = 4G");
		break;
	case 2:
		Serial.println("Ascale = 8G");
		break;
	case 3:
		Serial.println("Ascale = 16G");
		break;
	default:
		Serial.println("Ascale Error");
		break;
	}
}*/


void getRTC (uint8_t index)
{
	RTC.getDate(day, month, year);
	RTC.getTime(hours[index], minutes[index], seconds[index], subSeconds[index]);
}

void printRTC (uint8_t index)
{
	Serial.print(day); Serial.print(":"); Serial.print(month); Serial.print(":20"); Serial.print(year);
	Serial.print(" ");

	milliseconds = ((subSeconds[index] >> 17) * 1000 + 16384) / 32768;

	if (hours[index] < 10)
	{
		Serial.print("0");Serial.print(hours[index]);
	}
	else
		Serial.print(hours[index]);

	Serial.print(":");
	if (minutes[index] < 10)
	{
		Serial.print("0"); Serial.print(minutes[index]);
	}
	else
		Serial.print(minutes[index]);

	Serial.print(":");
	if (seconds[index] < 10)
	{
		Serial.print("0"); Serial.print(seconds[index]);
	}
	else
		Serial.print(seconds[index]);

	Serial.print(".");
	if (milliseconds <= 9)
	{
		Serial.print("0");
	}
	if (milliseconds <= 99)
	{
		Serial.print("0");
	}
	Serial.print(milliseconds);
	Serial.println(" ");
}


void initLSM303AGR(){



}

