#include "flash_interface.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000)  /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000)  /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000)  /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000)  /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000)  /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000)  /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000)  /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000)  /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000)  /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000)  /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000)  /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000)  /* Base @ of Sector 11, 128 Kbytes */

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if ((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0)) {
    sector = FLASH_SECTOR_0;
  } else if ((Address < ADDR_FLASH_SECTOR_2)
             && (Address >= ADDR_FLASH_SECTOR_1)) {
    sector = FLASH_SECTOR_1;
  } else if ((Address < ADDR_FLASH_SECTOR_3)
             && (Address >= ADDR_FLASH_SECTOR_2)) {
    sector = FLASH_SECTOR_2;
  } else if ((Address < ADDR_FLASH_SECTOR_4)
             && (Address >= ADDR_FLASH_SECTOR_3)) {
    sector = FLASH_SECTOR_3;
  } else if ((Address < ADDR_FLASH_SECTOR_5)
             && (Address >= ADDR_FLASH_SECTOR_4)) {
    sector = FLASH_SECTOR_4;
  } else if ((Address < ADDR_FLASH_SECTOR_6)
             && (Address >= ADDR_FLASH_SECTOR_5)) {
    sector = FLASH_SECTOR_5;
  } else if ((Address < ADDR_FLASH_SECTOR_7)
             && (Address >= ADDR_FLASH_SECTOR_6)) {
    sector = FLASH_SECTOR_6;
  } else if ((Address < ADDR_FLASH_SECTOR_8)
             && (Address >= ADDR_FLASH_SECTOR_7)) {
    sector = FLASH_SECTOR_7;
  } else if ((Address < ADDR_FLASH_SECTOR_9)
             && (Address >= ADDR_FLASH_SECTOR_8)) {
    sector = FLASH_SECTOR_8;
  } else if ((Address < ADDR_FLASH_SECTOR_10)
             && (Address >= ADDR_FLASH_SECTOR_9)) {
    sector = FLASH_SECTOR_9;
  } else if ((Address < ADDR_FLASH_SECTOR_11)
             && (Address >= ADDR_FLASH_SECTOR_10)) {
    sector = FLASH_SECTOR_10;
  } else {                      /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */

    sector = FLASH_SECTOR_11;
  }

  return sector;
}

/**
  * @brief  Gets sector Size
  * @param  None
  * @retval The size of a given sector
  */
static uint32_t GetSectorSize(uint32_t Sector)
{
  uint32_t sectorsize = 0x00;

  if ((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1)
      || (Sector == FLASH_SECTOR_2) || (Sector == FLASH_SECTOR_3)) {
    sectorsize = 16 * 1024;
  } else if (Sector == FLASH_SECTOR_4) {
    sectorsize = 64 * 1024;
  } else {
    sectorsize = 128 * 1024;
  }
  return sectorsize;
}

uint32_t flash_unlock(void)
{
  HAL_StatusTypeDef ret = 0;
  ret = HAL_FLASH_Unlock();
  return ret;
}

uint32_t flash_lock(void)
{
  HAL_StatusTypeDef ret = 0;
  ret = HAL_FLASH_Lock();
  return ret;
}

uint32_t flash_erase_by_sector(uint32_t start_address, uint8_t sector_num)
{
  uint32_t FirstSector = 0, NbOfSectors = 0;
  uint32_t SectorError = 0;

  /* Get the 1st sector to erase */
  FirstSector = GetSector(start_address);
  /* Get the number of sector to erase from 1st sector */
  NbOfSectors = sector_num;

  /* Fill EraseInit structure */
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FirstSector;
  EraseInitStruct.NbSectors = NbOfSectors;
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
    /* 
       Error occurred while sector erase. 
       User can add here some code to deal with this error. 
       SectorError will contain the faulty sector and then to know the code error on this sector,
       user can call function 'HAL_FLASH_GetError()'
     */
    /*
       FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
     */
    Error_Handler();
  }
}

uint32_t flash_program_by_word(uint32_t address, uint32_t data)
{
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data) == HAL_OK) {
    return HAL_OK;
  } else {
    /* Error occurred while writing data in Flash memory. 
       User can add here some code to deal with this error */
    /*
       FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
     */
    Error_Handler();
  }
}

void flash_update_bootup_sector(void)
{
  otaInfo temp_ota_info = { 0 };

  temp_ota_info.boot_mode = flashOtaInfo.boot_mode;
  temp_ota_info.using_app_id = flashOtaInfo.using_app_id;
  temp_ota_info.updating_app_id = flashOtaInfo.updating_app_id;

  if (flashOtaInfo.using_app_id == 0) {
    temp_ota_info.using_app_id = 1;
    temp_ota_info.updating_app_id = 0;

    //call flash ram api to change using_app_id to 1
    //call flash ram api to change updating_app_id to 0
  } else {
    temp_ota_info.using_app_id = 0;
    temp_ota_info.updating_app_id = 1;

    //call flash ram api to change using_app_id to 0
    //call flash ram api to change updating_app_id to 1
  }
  flash_erase_by_sector(FLASH_OTAINFO_ADD, 1);

  flash_program_by_word(FLASH_OTAINFO_ADD, temp_ota_info.boot_mode);
  flash_program_by_word(FLASH_OTAINFO_ADD + 4, temp_ota_info.using_app_id);
  flash_program_by_word(FLASH_OTAINFO_ADD + 8, temp_ota_info.updating_app_id);

}

void flash_update_boot_magic_number(void)
{
  otaInfo temp_ota_info = { 0 };

  temp_ota_info.boot_mode = RAM_MAGIC_WORD;
  temp_ota_info.using_app_id = flashOtaInfo.using_app_id;
  temp_ota_info.updating_app_id = flashOtaInfo.updating_app_id;

  flash_erase_by_sector(FLASH_OTAINFO_ADD, 1);

  flash_program_by_word(FLASH_OTAINFO_ADD, temp_ota_info.boot_mode);
  flash_program_by_word(FLASH_OTAINFO_ADD + 4, temp_ota_info.using_app_id);
  flash_program_by_word(FLASH_OTAINFO_ADD + 8, temp_ota_info.updating_app_id);

}

void flash_clear_boot_magic_number(void)
{
  otaInfo temp_ota_info = { 0 };

  temp_ota_info.boot_mode = 0;
  temp_ota_info.using_app_id = flashOtaInfo.using_app_id;
  temp_ota_info.updating_app_id = flashOtaInfo.updating_app_id;

  flash_erase_by_sector(FLASH_OTAINFO_ADD, 1);

  flash_program_by_word(FLASH_OTAINFO_ADD, temp_ota_info.boot_mode);
  flash_program_by_word(FLASH_OTAINFO_ADD + 4, temp_ota_info.using_app_id);
  flash_program_by_word(FLASH_OTAINFO_ADD + 8, temp_ota_info.updating_app_id);
}
