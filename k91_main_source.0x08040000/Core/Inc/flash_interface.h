#ifndef FLASH_INTERFACE_H
#define FLASH_INTERFACE_H

#include "main.h"

// ===================== Memory Map Define ======================//

// 0x08000000 ~ 0x0800FFFF 64KB bootloader
// 0x08010000 ~ 0x0801FFFF 64KB for bootup infomation
// 0x08020000 ~ 0x0803FFFF 128KB for calibration info
// 0x08040000 ~ 0x0809FFFF 384KB APP1 Partition
// 0x080A0000 ~ 0x080FFFFF 384KB APP2 Partition

/*
 * STM32 FLASH MAP
 * Block Name Block base addresses Size

Sector 0 0x0800 0000 - 0x0800 3FFF 16 Kbytes
Sector 1 0x0800 4000 - 0x0800 7FFF 16 Kbytes
Sector 2 0x0800 8000 - 0x0800 BFFF 16 Kbytes
Sector 3 0x0800 C000 - 0x0800 FFFF 16 Kbytes
Sector 4 0x0801 0000 - 0x0801 FFFF 64 Kbytes
Sector 5 0x0802 0000 - 0x0803 FFFF 128 Kbytes
Sector 6 0x0804 0000 - 0x0805 FFFF 128 Kbytes
Sector 7 0x0806 0000 - 0x0807 FFFF 128 Kbytes
Sector 8 0x0808 0000 - 0x0809 FFFF 128 Kbytes
Sector 9 0x080A 0000 - 0x080B FFFF 128 Kbytes
Sector 10 0x080C 0000 - 0x080D FFFF 128 Kbytes
Sector 11 0x080E 0000 - 0x080F FFFF 128 Kbytes
 */

// ==============================================================//

//STORGY

#define BOOTLOADER_START_ADDR   ((uint32_t)0X08000000)
#define USER_CODE1_START_ADDR   ((uint32_t)0x08040000)
#define USER_CODE2_START_ADDR   ((uint32_t)0x080A0000)

#define RAM_MAGIC_WORD   (0X89AB)

typedef struct otaInfo {
  uint32_t boot_mode;           //1 stands for normal boot, 0 stands for OTA boot
  uint32_t using_app_id;        //can be 0 or 1, 0 stands for USER_CODE1_START_ADDR
  uint32_t updating_app_id;     //can be 0 or 1, 0 stands for USER_CODE1_START_ADDR
} otaInfo;

#define FLASH_OTAINFO_ADD ((uint32_t)0x08010000)
#define FLASH_CALIBRATION_DATA ((uint32_t)0x08020000)

#define flashOtaInfo    (*((struct otaInfo*)FLASH_OTAINFO_ADD))
#define flashCaliInfo   (*((struct calibrationData*)FLASH_CALIBRATION_DATA))

typedef void (*pFunction) (void);
pFunction JumpToApplication;
uint32_t JumpAddress;

//================================================================//

uint32_t flash_lock(void);
uint32_t flash_unlock(void);
uint32_t flash_erase_by_sector(uint32_t start_address, uint8_t sector_num);
uint32_t flash_program_by_word(uint32_t address, uint32_t data);
void flash_update_bootup_sector(void);
void flash_update_boot_magic_number(void);
void flash_clear_boot_magic_number(void);

#endif
