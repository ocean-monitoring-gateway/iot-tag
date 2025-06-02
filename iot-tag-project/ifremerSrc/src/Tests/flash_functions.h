/* 13/01/2020 Pierre Gogendeau  

Libraries of the functions using the flash memory

*/

#ifndef _FLASH_FUNCTIONS_H_
#define _FLASH_FUNCTIONS_H_

#include "utils.h"
#include <SPI.h>
#include "SPIFlash.h"
#include "TimerMillis.h"
#include "test_accelero_1.h"



#define STAT_WIP 1
#define STAT_WEL 2

#define CMD_WRITE_STATUS_REG   0x01
#define CMD_PAGE_PROGRAM       0x02
#define CMD_READ_DATA          0x03
#define CMD_WRITE_DISABLE      0x04//not tested
#define CMD_READ_STATUS_REG    0x05
#define CMD_WRITE_ENABLE       0x06
#define CMD_READ_HIGH_SPEED    0x0B//not tested
#define CMD_SECTOR_ERASE       0x20//not tested
#define CMD_BLOCK32K_ERASE     0x52//not tested
#define CMD_RESET_DEVICE       0xF0//<<-different from winbond
#define CMD_READ_ID            0x9F
#define CMD_RELEASE_POWER_DOWN 0xAB//not tested
#define CMD_POWER_DOWN         0xB9//not tested
#define CMD_CHIP_ERASE         0xC7
#define CMD_BLOCK64K_ERASE     0xD8//not tested

void eraseFlash(uint16_t  last_page);
//void write_flash_test_accelero_1(uint16_t *pPage_number , uint8_t *pIndex_page_number, uint16_t last_page, data_flash_t *param_flash);
void write_flash_paramater_init_test_accelero_1(uint16_t  max_last_page, uint8_t AODR, uint8_t MODR, uint8_t Ascale);

void write_pause();
int page_to_address(int pn);
int address_to_page(int addr);
void flash_read_id(unsigned char *idt);
unsigned char flash_read_status();
void flash_hard_reset();
void flash_chip_erase(boolean wait);
void flash_erase_pages_sector(int pn);
void flash_erase_pages_block32k(int pn);
void flash_erase_pages_block64k(int pn);
void flash_page_program(unsigned char *wp, int pn);
void flash_read_pages(unsigned char *p, int pn, const int n_pages);
void flash_fast_read_pages(unsigned char *p, int pn, const int n_pages);

#endif
