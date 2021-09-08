#include "flash_interface.h"
#include "calibration_data.h"

int update_calibration_data_to_flash(calibrationData * cali_data)
{
  int ret = -1, count = 0;
  uint32_t calibration_data_size = 0;
  uint32_t *calibration_data_ptr = NULL;
  calibration_data_ptr = cali_data;
  //return byte num of calibration data
  if (sizeof(calibrationData) % 4 == 0) {
    calibration_data_size = sizeof(calibrationData) / 4;
  } else {
    calibration_data_size = sizeof(calibrationData) / 4 + 1;
  }

  flash_unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
                         FLASH_FLAG_PGSERR);
  flash_erase_by_sector(FLASH_CALIBRATION_DATA, 1);
  do {
    flash_program_by_word(FLASH_CALIBRATION_DATA + count * 4,
                          *(calibration_data_ptr + count));
    count++;
  } while (count < calibration_data_size);

  flash_lock();
}
