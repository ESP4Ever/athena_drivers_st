/*! \file chbsp_chirp_dvb.c
 \brief Board support package functions for the Chirp DVB-01 development board.
 */

/*
 Copyright � 2016-2019, Chirp Microsystems. All rights reserved.

 Chirp Microsystems CONFIDENTIAL

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL CHIRP MICROSYSTEMS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 You can contact the authors of this program by email at support@chirpmicro.com
 or by mail at 2560 Ninth Street, Suite 220A, Berkeley, CA 94710.
 */

/* Includes */
#include "chirp_dvb.h"          // board-specific defines
#include "soniclib.h"           // Chirp SonicLib API defines
#include "chirp_bsp.h"          // standard Chirp BSP defines
#include "stm32f4xx_hal.h"      // STM32 HAL library defines
#include "sensor_device.h"

#include <stdint.h>
#include <stdio.h>

//static uint32_t i2c_clockspeed = I2C_BUS_SPEED;
static uint8_t sensor_activate_count = 0;
static uint8_t chirp_i2c_addrs[] = CHIRP_I2C_ADDRS;
static uint8_t chirp_i2c_buses[] = CHIRP_I2C_BUSES;
static bool uitralsonic_proximity_enabled = false;

/*
 * Here you set the pin masks for each of the prog pins
 *   These arrays are indexed by the sensor device number, as returned by ch_get_dev_num().
 */

uint16_t chirp_pin_prog[] = { PROG0_PIN };

GPIO_TypeDef *chirp_port_prog[] = { PROG0_PORT };

/*
 * Here you set the pin masks for each of the INT pins.
 *   These arrays are indexed by the sensor device number, as returned by ch_get_dev_num().
 */
uint16_t chirp_pin_io[] = { IO_PIN };

/*
 * Here you set the interrupt vectors associated with each INT pin
 *   This array is indexed by the sensor device number, as returned by ch_get_dev_num().
 */
IRQn_Type chirp_pin_io_irq[] = { IO_EXTI_IRQn };

RCC_ClkInitTypeDef InitialClockConfig;
RCC_OscInitTypeDef InitialOscConfig;
uint32_t InitialpFLatency;

/* Chirp sensor group pointer */
ch_group_t *sensor_group_ptr;

/* Callback function pointers */
ch_io_int_callback_t io_int_callback_ptr = NULL;
static ch_timer_callback_t periodic_timer_callback_ptr = NULL;

static uint16_t periodic_timer_interval_ms;

I2C_HandleTypeDef hi2c[CHBSP_NUM_I2C_BUSES];
uint32_t hardwareID[3];

/*
 * tim3 is used for timing the interval between sensor measurements (required)
 */
TIM_HandleTypeDef *tim3;        // XXX move and rename

static platform_prams *chirp_init_parms;

static void getHardwareID()
{
  hardwareID[0] = TM_ID_GetUnique32(0);
  hardwareID[1] = TM_ID_GetUnique32(1);
  hardwareID[2] = TM_ID_GetUnique32(2);
}

/*!
 * \brief Initialize board hardware
 *
 * \note This function performs all necessary initialization on the board.
 */

void chbsp_board_init(ch_group_t * grp_ptr)
{

  /* Make local copy of group pointer */
  sensor_group_ptr = grp_ptr;

  /* Initialize group descriptor */
  grp_ptr->num_ports = CHBSP_MAX_DEVICES;
  grp_ptr->num_i2c_buses = CHBSP_NUM_I2C_BUSES;
  grp_ptr->rtc_cal_pulse_ms = CHBSP_RTC_CAL_PULSE_MS;

  //HAL_PWR_DisableSleepOnExit();
}

/*!
 * \brief Assert the reset pin
 *
 * \note This function should drive the Chirp sensor reset pin low.
 */
void chbsp_reset_assert(void)
{
  HAL_GPIO_WritePin(RESET_N_PORT, RESET_N_PIN, RESET_N_ASSERTED_LEVEL);
}

/*!
 * \brief Deassert the reset pin
 *
 * \note This function should drive the Chirp sensor reset pin high (or open drain if there is a pull-up).
 */
void chbsp_reset_release(void)
{
  HAL_GPIO_WritePin(RESET_N_PORT, RESET_N_PIN, RESET_N_DEASSERTED_LEVEL);
}

/*!
 * \brief Assert the PROG pin
 *
 * \note This function should drive the Chirp sensor PROG pin high on the specified port.
 */
void chbsp_program_enable(ch_dev_t * dev_ptr)
{
  HAL_GPIO_WritePin(chirp_port_prog[dev_ptr->io_index],
                    chirp_pin_prog[dev_ptr->io_index], PROG_ASSERTED_LEVEL);
}

/*!
 * \brief Deassert the PROG pin
 *
 * \note This function should drive the Chirp sensor PROG pin low on the specified port.
 */
void chbsp_program_disable(ch_dev_t * dev_ptr)
{
  HAL_GPIO_WritePin(chirp_port_prog[dev_ptr->io_index],
                    chirp_pin_prog[dev_ptr->io_index], PROG_DEASSERTED_LEVEL);
}

/*!
 * \brief Configure the host side of the CH101 interrupt pin as an output
 *
 * \note
 */
void chbsp_group_set_io_dir_out(ch_group_t * grp_ptr)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  uint16_t iomask = 0;
  int i;
  for (i = 0; i < grp_ptr->num_ports; i++) {
    if (grp_ptr->device[i]->sensor_connected) {
      iomask |= chirp_pin_io[i];
    }
  }
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pin = iomask;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CM_IO_PORT, &GPIO_InitStructure);

  //set PC10 to low to make the gpio output enable
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
}

/*!
 * \brief Configure the host side of the CH101 interrupt pin as an input
 *
 * \note This function assumes a bidirectional level shifter is interfacing the ICs.
 */
void chbsp_group_set_io_dir_in(ch_group_t * grp_ptr)
{

  //set PC10 to high to make the int input enable
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Pin = IO_PIN_ALL;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CM_IO_PORT, &GPIO_InitStructure);
}

/*!
 * \brief Initialize the I/O pins.
 *
 * Configure reset and program pins as outputs. Assert reset and program. Configure IO pin as input.
 */
void chbsp_group_pin_init(ch_group_t * grp_ptr)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable necessary clocks */
  RESET_N_PORT_ENABLE();
  PROG0_PORT_ENABLE();
  //PROG1_PORT_ENABLE();
  CM_IO_PORT_ENABLE();

  /* Initialize reset */
  GPIO_InitStructure.Mode = RESET_N_MODE;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = RESET_N_PIN;
  HAL_GPIO_Init(RESET_N_PORT, &GPIO_InitStructure);
  chbsp_reset_assert();

  /* Initialize program pins */
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  uint8_t i;
  for (i = 0; i < grp_ptr->num_ports; i++) {
    GPIO_InitStructure.Pin = chirp_pin_prog[i];
    HAL_GPIO_Init(chirp_port_prog[i], &GPIO_InitStructure);
    chbsp_program_enable(grp_ptr->device[i]);
  }

  /* Initialize IO pins */
  chbsp_group_set_io_dir_in(grp_ptr);
  for (i = 0; i < grp_ptr->num_ports; i++) {
    __HAL_GPIO_EXTI_CLEAR_FLAG(chirp_pin_io[i]);
    __HAL_GPIO_EXTI_CLEAR_IT(chirp_pin_io[i]);
    HAL_NVIC_SetPriority(chirp_pin_io_irq[i], CH_SENSOR_IRQ_PRIORITY, 0);
    HAL_NVIC_DisableIRQ(chirp_pin_io_irq[i]);
  }
  //cmPinInterruptDisable(grp_ptr);
}

/*!
 * \brief Set the IO pin low.
 *
 * \note If directly coupled to the Chirp sensor it is recommended to use a passive pull-down to ensure the supply is never shorted through the I/O.
 */
void chbsp_group_io_clear(ch_group_t * grp_ptr)
{
  uint16_t iomask = 0;
  int i;
  for (i = 0; i < grp_ptr->num_ports; i++) {
    if (grp_ptr->device[i]->sensor_connected) {
      iomask |= chirp_pin_io[i];
    }
  }
  HAL_GPIO_WritePin(CM_IO_PORT, iomask, GPIO_PIN_RESET);
}

/*!
 * \brief Set the IO pin high.
 *
 * \note
 */
void chbsp_group_io_set(ch_group_t * grp_ptr)
{
  uint16_t iomask = 0;
  int i;
  for (i = 0; i < grp_ptr->num_ports; i++) {
    if (grp_ptr->device[i]->sensor_connected) {
      iomask |= chirp_pin_io[i];
    }
  }
  HAL_GPIO_WritePin(CM_IO_PORT, iomask, GPIO_PIN_SET);
}

/*!
 * \brief Enable the interrupt
 *
 * \note
 */
void chbsp_group_io_interrupt_enable(ch_group_t * grp_ptr)
{
  uint8_t i;

  for (i = 0; i < grp_ptr->num_ports; i++) {
    //printf("chbsp_group_io_interrupt_enable! %u\r\n", osKernelGetTickCount());
    chbsp_io_interrupt_enable(grp_ptr->device[i]);
  }
}

void chbsp_io_interrupt_enable(ch_dev_t * dev_ptr)
{
  if (dev_ptr->sensor_connected) {
    GPIO_InitTypeDef GPIO_InitStructure;
    //printf("chbsp_io_interrupt_enable: %d\r\n", dev_ptr->io_index);
    /* Enable GPIOA clock */
    CM_IO_PORT_ENABLE();

    /* Configure PA0 pin as input pull down */
    __HAL_GPIO_EXTI_CLEAR_FLAG(chirp_pin_io[dev_ptr->io_index]);
    __HAL_GPIO_EXTI_CLEAR_IT(chirp_pin_io[dev_ptr->io_index]);
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin = chirp_pin_io[dev_ptr->io_index];

    HAL_GPIO_Init(CM_IO_PORT, &GPIO_InitStructure);
    HAL_NVIC_SetPriority(chirp_pin_io_irq[dev_ptr->io_index],
                         CH_SENSOR_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(chirp_pin_io_irq[dev_ptr->io_index]);
  }
}

/*!
 * \brief Disable the interrupt
 *
 * \note
 */
void chbsp_group_io_interrupt_disable(ch_group_t * grp_ptr)
{

  uint8_t i;
  for (i = 0; i < grp_ptr->num_ports; i++) {
    //printf("chbsp_io_interrupt_disable! %u\r\n", osKernelGetTickCount());
    chbsp_io_interrupt_disable(grp_ptr->device[i]);
  }
}

void chbsp_io_interrupt_disable(ch_dev_t * dev_ptr)
{

  if (dev_ptr->sensor_connected) {
    /* Configure PA0 pin as input floating */
    //printf("chbsp_group_io_interrupt_disable: %d\r\n", dev_ptr->io_index);
    HAL_NVIC_DisableIRQ(chirp_pin_io_irq[dev_ptr->io_index]);
  }
}

/*!
 * \brief Set callback function for Chirp sensor I/O interrupt
 *
 * \note
 */
void chbsp_io_int_callback_set(ch_group_t * grp_ptr,
                               ch_io_int_callback_t callback_func_ptr)
{

  io_int_callback_ptr = callback_func_ptr;
}

/*!
 * \brief Busy wait delay for us microseconds
 *
 * \note
 */
void chbsp_delay_us(uint32_t us)
{
  uint32_t cycles_per_us = HAL_RCC_GetSysClockFreq() / 10000000;
  volatile uint32_t i;

  for (i = 0; i < (us * cycles_per_us); i++) {
    ;
  }
}

/*!
 * \brief Busy wait delay for ms milliseconds
 *
 * \note
 */
void chbsp_delay_ms(uint32_t ms)
{
  HAL_Delay(ms);
}

/*!
 * \brief Initialize the host's I2C hardware.
 * Return 0 if successful, non-zero otherwise
 */
int chbsp_i2c_init(void)
{
  HAL_StatusTypeDef res = HAL_OK;

  //cp chirp_init_parms->i2chandle to hi2c[0] using memory cpy
  if (chirp_init_parms != NULL) {
    hi2c[0] = chirp_init_parms->i2c_handle;
  } else {
    printf("chirp_init_pram is NULL!\r\n");
    res = HAL_ERROR;
  }

  return (res != HAL_OK);
}

/*!
 * \brief Write a byte to the slave.  Function assumes start condition has been issued.
 *
 * \param data Data byte to be transmitted.
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * \note The blocking-mode driver does not always seem to wait for the bus to be free.  This function checks the busy flag
 * explicitly.
 */
int chbsp_i2c_write(ch_dev_t * dev_ptr, uint8_t * data, uint16_t n)
{
  int error = (HAL_I2C_Master_Transmit(&hi2c[dev_ptr->i2c_bus_index],
                                       (uint16_t) (dev_ptr->i2c_address) << 1,
                                       data, n, 10000));

  if (!error) {
    while (__HAL_I2C_GET_FLAG(&hi2c[dev_ptr->i2c_bus_index], I2C_FLAG_BUSY)) {
      ;                         // wait here until bus not busy
    }
  } else {
    //printf("sensor i2c write Error:%d\n",error);
  }
  return error;
}

// XXX need comment block
int chbsp_i2c_mem_write(ch_dev_t * dev_ptr, uint16_t mem_addr,
                        uint8_t * data_ptr, uint16_t num_bytes)
{
  I2C_HandleTypeDef *handle = &hi2c[dev_ptr->i2c_bus_index];
  HAL_StatusTypeDef hal_status;
  uint16_t dev_addr = (dev_ptr->i2c_address << 1);      // I2C address for device (shifted)
  int ret_val = 1;

#if 1
  printf("chbsp_i2c_mem_write:  calling HAL_I2C_Mem_Read(0x%x, 0x%x, 0x%x, %d, 0x%x, %d, %d)\r\n",      // XXX debug
         handle, dev_addr, mem_addr, sizeof(uint8_t), data_ptr, num_bytes, I2C_TIMEOUT);        // XXX debug
#endif

  hal_status = HAL_I2C_Mem_Write(handle, dev_addr, mem_addr, sizeof(uint8_t),
                                 data_ptr, num_bytes, I2C_TIMEOUT);
#if 1
  printf(" chbsp_i2c_mem_write hal_status = 0x%x\r\n", hal_status);     // XXX debug
#endif
  if (hal_status == HAL_OK) {
    ret_val = 0;                // indicate success
  }

  return ret_val;

}

int chbsp_i2c_write_nb(ch_dev_t * dev_ptr, uint8_t * data, uint16_t n)
{
  HAL_StatusTypeDef hal_status;
  int ret_val = 1;

  hal_status = HAL_I2C_Master_Transmit_DMA(&hi2c[dev_ptr->i2c_bus_index],
                                           (uint16_t) (dev_ptr->
                                                       i2c_address << 1), data,
                                           (uint16_t) n);
  if (hal_status == HAL_OK) {
    ret_val = 0;                // indicate success
  }

  return ret_val;
}

// XXX need commment block
int chbsp_i2c_mem_write_nb(ch_dev_t * dev_ptr, uint16_t mem_addr,
                           uint8_t * data, uint16_t len)
{
  HAL_StatusTypeDef hal_status;
  int ret_val = 1;

  hal_status = HAL_I2C_Mem_Write_DMA(&hi2c[dev_ptr->i2c_bus_index],
                                     (uint16_t) (dev_ptr->i2c_address << 1),
                                     mem_addr, sizeof(uint8_t), data,
                                     (uint16_t) len);
  if (hal_status == HAL_OK) {
    ret_val = 0;                // indicate success
  }

  return ret_val;
}

/*!
 * \brief Read a specified number of bytes from an I2C slave.
 *
 * \param data Pointer to receive data buffer.
 * \param length Number of bytes to read.
 * \return Currently, always 0.  TODO: add error checking
 *
 * \note The blocking-mode driver does not always seem to wait for the bus to be free.  This function checks the busy flag
 * explicitly.
 */
//#define I2CDEBUG
int chbsp_i2c_read(ch_dev_t * dev_ptr, uint8_t * data, uint16_t len)
{
#ifdef I2CDEBUG
  int32_t timeout = 10000000;
#endif
  int error = (HAL_I2C_Master_Receive(&hi2c[dev_ptr->i2c_bus_index],
                                      (uint16_t) (dev_ptr->i2c_address) << 1,
                                      data, len, 100) != HAL_OK);
  if (!error)
#ifdef I2CDEBUG
    if (timeout == 10000000)
      printf("len: %u\n", len);
#endif
  while (__HAL_I2C_GET_FLAG(&hi2c[dev_ptr->i2c_bus_index], I2C_FLAG_BUSY)) {
#ifdef I2CDEBUG
    timeout--;
    if (timeout < 0) {
      printf("i2c Timed out!!!!!!!!!!!!!!!\n");
    }
#endif
  }
  if (error) {
    printf("i2c error: %u ! %u:%u\n", error, dev_ptr->i2c_bus_index,
           dev_ptr->i2c_address);
  }
  return error;
}

// XXX need comment block
//
int chbsp_i2c_mem_read(ch_dev_t * dev_ptr, uint16_t mem_addr,
                       uint8_t * data_ptr, uint16_t num_bytes)
{
  I2C_HandleTypeDef *handle = &hi2c[dev_ptr->i2c_bus_index];
  HAL_StatusTypeDef hal_status;
  uint16_t dev_addr = (dev_ptr->i2c_address << 1);      // I2C address for device (shifted)
  int ret_val = 1;

#if 1
  printf("chbsp_i2c_mem_read:  calling HAL_I2C_Mem_Read(0x%x, 0x%x, 0x%x, %d, 0x%x, %d, %d)\r\n",       // XXX debug
         handle, dev_addr, mem_addr, sizeof(uint8_t), data_ptr, num_bytes, I2C_TIMEOUT);        // XXX debug
#endif
  hal_status = HAL_I2C_Mem_Read(handle, dev_addr, mem_addr, sizeof(uint8_t),
                                data_ptr, num_bytes, I2C_TIMEOUT);

#if 1
  printf("chbsp_i2c_mem_read hal_status = 0x%x\r\n", hal_status);       // XXX debug
#endif

  if (hal_status == HAL_OK) {
    ret_val = 0;                // indicate success
  }

  return ret_val;
}

int chbsp_i2c_read_nb(ch_dev_t * dev_ptr, uint8_t * data, uint16_t len)
{
  HAL_StatusTypeDef hal_status;
  int ret_val = 1;

  hal_status = HAL_I2C_Master_Receive_DMA(&hi2c[dev_ptr->i2c_bus_index],
                                          (uint16_t) (dev_ptr->
                                                      i2c_address << 1), data,
                                          (uint16_t) len);
  if (hal_status == HAL_OK) {
    ret_val = 0;                // indicate success
  }

  return ret_val;
}

// XXX need commment block
int chbsp_i2c_mem_read_nb(ch_dev_t * dev_ptr, uint16_t mem_addr, uint8_t * data,
                          uint16_t len)
{
  HAL_StatusTypeDef hal_status;
  int ret_val = 1;

  hal_status = HAL_I2C_Mem_Read_DMA(&hi2c[dev_ptr->i2c_bus_index],
                                    (uint16_t) (dev_ptr->i2c_address << 1),
                                    mem_addr, sizeof(uint8_t), data,
                                    (uint16_t) len);
  if (hal_status == HAL_OK) {
    ret_val = 0;                // indicate success
  }

  return ret_val;
}

void chbsp_print_str(char *str)
{
  printf(str);
}

void chbsp_debug_on(uint8_t dbg_pin_num)
{
  //DBG_PIN_ON(dbg_pin_num);
}

void chbsp_debug_off(uint8_t dbg_pin_num)
{
  //DBG_PIN_OFF(dbg_pin_num);
}

void chbsp_debug_toggle(uint8_t dbg_pin_num)
{
  //DBG_PIN_TOGGLE(dbg_pin_num);
}

void chbsp_i2c_reset(ch_dev_t * dev_ptr)
{
  HAL_I2C_Init(&hi2c[dev_ptr->i2c_bus_index]);
}

/*
 * This function should set up a timer with a resolution of 20us.
 * This will be used to set the interval between pulses on the base station
 */
uint8_t chbsp_periodic_timer_init(uint16_t interval_ms,
                                  ch_timer_callback_t callback_func_ptr)
{

  periodic_timer_callback_ptr = callback_func_ptr;
  periodic_timer_interval_ms = interval_ms;

  /*
   * Set timer up with resolution of 320ns
   *
   */
  if (chirp_init_parms != NULL) {
    tim3 = &chirp_init_parms->OperateTimerHandle;
  } else {
    Error_Handler();
  }

  return 0;
}

/*
 * This function is called by the timer interrupt to indicate that the periodic timer has expired.
 */
void chbsp_periodic_timer_handler(void)
{
  ch_timer_callback_t func_ptr = periodic_timer_callback_ptr;
  //printf("chbsp_periodic_timer_handler: %p\r\n", func_ptr);
  if (func_ptr != NULL) {
    //printf("chbsp_periodic_timer_handler\r\n");
    (*func_ptr) ();             // call application timer callback routine
  }
}

/*
 * This function is called to obtain the I2C address and bus info for a device specified by the
 * group and I/O index value.
 *
 * Note: grp_ptr is not used by this implementation - all I2C addresses are from same set
 */
uint8_t chbsp_i2c_get_info(ch_group_t * grp_ptr, uint8_t io_index,
                           ch_i2c_info_t * info_ptr)
{
  uint8_t ret_val = 1;

  if (io_index < CHBSP_MAX_DEVICES) {
    info_ptr->address = chirp_i2c_addrs[io_index];
    info_ptr->bus_num = chirp_i2c_buses[io_index];

    info_ptr->drv_flags = I2C_DRV_FLAGS;        // i2c driver special handling flags, from board header file

    ret_val = 0;
  }

  return ret_val;
}

/*
 * This function puts the processor into a low-power sleep mode that can be awakened
 * by interrupt activity etc.
 */
void chbsp_proc_sleep(void)
{
  //HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

void chbsp_led_on(uint8_t dev_num)
{

  /* The DVB board does not have an LED for each (possible) sensor,
   * so just use one LED for all.
   */
  //led_on(LED1_BLUE_PIN);
}

void chbsp_led_off(uint8_t dev_num)
{

  /* The DVB board does not have an LED for each (possible) sensor,
   * so just use one LED for all.
   */
  //led_off(LED1_BLUE_PIN);
}

/* =========================================== sensor related API starts from here ===================================================== */

/* Bit flags used in main loop to check for completion of sensor I/O.  */
#define DATA_READY_FLAG		(1 << 0)
#define IQ_READY_FLAG		(1 << 1)

/* Array of structs to hold measurement data, one for each possible device */
chirp_data_t chirp_data[CHIRP_MAX_NUM_SENSORS];

/* Array of ch_dev_t device descriptors, one for each possible device */
ch_dev_t chirp_devices[CHIRP_MAX_NUM_SENSORS];

/* Configuration structure for group of sensors */
ch_group_t chirp_group;

/* Detection level settings - for CH201 sensors only
 *   Each threshold entry includes the starting sample number & threshold level.
 */
ch_thresholds_t chirp_ch201_thresholds = { 0, 5000,     /* threshold 0 */
  26, 2000,                     /* threshold 1 */
  39, 800,                      /* threshold 2 */
  56, 400,                      /* threshold 3 */
  79, 250,                      /* threshold 4 */
  89, 175
};                              /* threshold 5 */

/* Task flag word
 *   This variable contains the DATA_READY_FLAG and IQ_READY_FLAG bit flags 
 *   that are set in I/O processing routines.  The flags are checked in the 
 *   main() loop and, if set, will cause an appropriate handler function to 
 *   be called to process sensor data.  
 */
volatile uint32_t taskflags = 0;

/* Device tracking variables
 *   These are bit-field variables which contain a separate bit assigned to
 *   each (possible) sensor, indexed by the device number.  The active_devices
 *   variable contains the bit pattern describing which ports have active
 *   sensors connected.  The data_ready_devices variable is set bit-by-bit
 *   as sensors interrupt, indicating they have completed a measurement
 *   cycle.  The two variables are compared to determine when all active
 *   devices have interrupted.
 */
static uint32_t active_devices;
static uint32_t data_ready_devices;

/* Forward declarations */
/*
 * periodic_timer_callback() - periodic timer callback routine
 *
 * This function is called by the periodic timer interrupt when the timer 
 * expires.  Because the periodic timer is used to initiate a new measurement 
 * cycle on a group of sensors, this function calls ch_group_trigger() during 
 * each execution.
 *
 * This callback function is registered by the call to chbsp_periodic_timer_init() 
 * in main().
 */

static void periodic_timer_callback(void)
{
  //printf("periodic_timer_callback\r\n");
  ch_group_trigger(&chirp_group);
}

/*
 * sensor_int_callback() - sensor interrupt callback routine
 *
 * This function is called by the board support package's interrupt handler for 
 * the sensor's INT line every time that the sensor interrupts.  The device 
 * number parameter, dev_num, is used to identify the interrupting device
 * within the sensor group.  (Generally the device number is same as the port 
 * number used in the BSP to manage I/O pins, etc.)
 *
 * This callback function is registered by the call to ch_io_int_callback_set() 
 * in main().
 */
static void sensor_int_callback(ch_group_t * grp_ptr, uint8_t dev_num)
{
  ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

  data_ready_devices |= (1 << dev_num); // add to data-ready bit mask

  if (data_ready_devices == active_devices) {
    /* All active sensors have interrupted after performing a measurement */
    data_ready_devices = 0;

    /* Set data-ready flag - it will be checked in main() loop */
    taskflags |= DATA_READY_FLAG;

    /* Disable interrupt unless in free-running mode
     *   It will automatically be re-enabled during the next trigger 
     */
    if (ch_get_mode(dev_ptr) != CH_MODE_FREERUN) {
      chbsp_group_io_interrupt_disable(grp_ptr);
    }
  }
}

/*
 * display_config_info() - display the configuration values for a sensor
 *
 * This function displays the current configuration settings for an individual 
 * sensor.  The operating mode, maximum range, and static target rejection 
 * range (if used) are displayed.
 *
 * For CH201 sensors only, the multiple detection threshold values are also 
 * displayed.
 */
static uint8_t display_config_info(ch_dev_t * dev_ptr)
{
  ch_config_t read_config;
  uint8_t chirp_error;
  uint8_t dev_num = ch_get_dev_num(dev_ptr);

  /* Read configuration values for the device into ch_config_t structure */
  chirp_error = ch_get_config(dev_ptr, &read_config);

  if (!chirp_error) {
    char *mode_string;

    switch (read_config.mode) {
    case CH_MODE_IDLE:
      mode_string = "IDLE";
      break;
    case CH_MODE_FREERUN:
      mode_string = "FREERUN";
      break;
    case CH_MODE_TRIGGERED_TX_RX:
      mode_string = "TRIGGERED_TX_RX";
      break;
    case CH_MODE_TRIGGERED_RX_ONLY:
      mode_string = "TRIGGERED_RX_ONLY";
      break;
    default:
      mode_string = "UNKNOWN";
    }

    /* Display sensor number, mode and max range */
    printf("Sensor %d:\tmax_range=%dmm \tmode=%s  \r\n", dev_num,
           read_config.max_range, mode_string);

    /* Display static target rejection range, if used */
    if (read_config.static_range != 0) {
      printf("static_range=%d samples \r\n", read_config.static_range);
    }

    /* Display detection thresholds (only supported on CH201) */
    if (ch_get_part_number(dev_ptr) == CH201_PART_NUMBER) {
      ch_thresholds_t read_thresholds;

      /* Get threshold values in structure */
      chirp_error = ch_get_thresholds(dev_ptr, &read_thresholds);

      if (!chirp_error) {
        printf("\r\n  Detection thresholds:\r\n");
        for (int i = 0; i < CH_NUM_THRESHOLDS; i++) {
          printf("     %d\tstart: %2d\tlevel: %d\r\n", i,
                 read_thresholds.threshold[i].start_sample,
                 read_thresholds.threshold[i].level);
        }
      } else {
        printf(" Device %d: Error during ch_get_thresholds()\r\n", dev_num);
      }
    }
    printf("\n");

  } else {
    printf(" Device %d: Error during ch_get_config()\r\n", dev_num);
  }

  return chirp_error;
}

/*
 * handle_data_ready() - get data from all sensors
 *
 * This routine is called from the main() loop after all sensors have 
 * interrupted. It shows how to read the sensor data once a measurement is 
 * complete.  This routine always reads out the range and amplitude, and 
 * optionally performs either a blocking or non-blocking read of the raw I/Q 
 * data.   See the comments in hello_chirp.h for information about the
 * I/Q readout build options.
 *
 * If a blocking I/Q read is requested, this function will read the data from 
 * the sensor into the application's "chirp_data" structure for this device 
 * before returning.  
 *
 * Optionally, if a I/Q blocking read is requested and the OUTPUT_IQ_DATA_CSV 
 * build symbol is defined, this function will output the full I/Q data as a 
 * series of comma-separated value pairs (Q, I), each on a separate line.  This 
 * may be a useful step toward making the data available in an external 
 * application for analysis (e.g. by copying the CSV values into a spreadsheet 
 * program).
 *
 * If a non-blocking I/Q is read is initiated, a callback routine will be called
 * when the operation is complete.  The callback routine must have been 
 * registered using the ch_io_complete_callback_set function.
 */
static uint8_t handle_data_ready(ch_group_t * grp_ptr)
{
  uint8_t dev_num;
  int error;
  int num_samples = 0;
  uint16_t start_sample = 0;
  uint8_t iq_data_addr;
  uint8_t ret_val = 0;
  sensors_event_t sensor_data = { 0 };

  /* Read and display data from each connected sensor 
   *   This loop will write the sensor data to this application's "chirp_data"
   *   array.  Each sensor has a separate chirp_data_t structure in that 
   *   array, so the device number is used as an index.
   */

  for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
    ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

    if (ch_sensor_is_connected(dev_ptr)) {

      /* Get measurement results from each connected sensor 
       *   For sensor in transmit/receive mode, report one-way echo 
       *   distance,  For sensor(s) in receive-only mode, report direct 
       *   one-way distance from transmitting sensor 
       */

      if (ch_get_mode(dev_ptr) == CH_MODE_TRIGGERED_RX_ONLY) {
        chirp_data[dev_num].range = ch_get_range(dev_ptr, CH_RANGE_DIRECT);
      } else {
        chirp_data[dev_num].range = ch_get_range(dev_ptr,
                                                 CH_RANGE_ECHO_ONE_WAY);
      }

      if (chirp_data[dev_num].range == CH_NO_TARGET) {
        /* No target object was detected - no range value */

        //chirp_data[dev_num].amplitude = 0; /* no updated amplitude */
        chirp_data[dev_num].amplitude = ch_get_amplitude(dev_ptr);
        printf("Port %d:          no target found        \r\n", dev_num);

        if (chirp_data[dev_num].amplitude > 3000) {
          printf("ch_get_amplitude: %u, cover very near occurred! \r\n",
                 chirp_data[dev_num].amplitude);
          chirp_data[dev_num].range = 200.0f * 32.0f;
        }

      } else {
        /* Target object was successfully detected (range available) */

        /* Get the new amplitude value - it's only updated if range 
         * was successfully measured.  */
        chirp_data[dev_num].amplitude = ch_get_amplitude(dev_ptr);

        printf("Port %d:  Range: %0.1f mm  Amplitude: %u  \r\n",
               dev_num, (float)chirp_data[dev_num].range / 32.0f,
               chirp_data[dev_num].amplitude);
      }

      /* Get number of active samples in this measurement */
      num_samples = ch_get_num_samples(dev_ptr);
      chirp_data[dev_num].num_samples = num_samples;

      /* Read full IQ data from device into buffer or queue read 
       * request, based on build-time options  */

      /* Reading I/Q data in normal, blocking mode */
      error = ch_get_iq_data(dev_ptr, chirp_data[dev_num].iq_data,
                             start_sample, num_samples, CH_IO_MODE_BLOCK);

      if (!error) {
        printf("     %d IQ samples copied \r\n", num_samples);
      } else {
        printf("     Error reading %d IQ samples \r\n", num_samples);
      }

      //send related messages to SensorDataQ
      if (uitralsonic_proximity_enabled) {
        sensor_data.sensor_type = SENSOR_TYPE_PROXIMITY_HEAD;
        if (chirp_init_parms->boardID == REAR_BOARD) {
          sensor_data.sensor_type = SENSOR_TYPE_PROXIMITY_REAR;
        }
        sensor_data.accuracy = 3;
        sensor_data.timestamp = sensor_get_timestamp();
        sensor_data.sensor_data_t.vec.data[0] =
            (float)chirp_data[dev_num].range / 32.0f;
        sensor_data.sensor_data_t.vec.data[1] =
            (float)chirp_data[dev_num].amplitude;
        osMessageQueuePut(chirp_init_parms->SensorDataQHandle, &sensor_data,
                          0, 0);
      }
    }
  }

  return ret_val;
}

/*
 * handle_iq_data() - handle raw I/Q data from a non-blocking read
 *
 * This function is called from the main() loop when a non-blocking readout of 
 * the raw I/Q data has completed for all sensors.  The data will have been 
 * placed in this application's "chirp_data" array, in the chirp_data_t 
 * structure for each sensor, indexed by the device number.  
 *
 * By default, this function takes no action on the I/Q data, except to display 
 * the number of samples that were read from the device.
 *
 * Optionally, if the OUTPUT_IQ_DATA_CSV build symbol is defined, this function 
 * will output the full I/Q data as a series of comma-separated value pairs 
 * (Q, I), each on a separate line.  This may be a useful step toward making 
 * the data available in an external application for analysis (e.g. by copying 
 * the CSV values into a spreadsheet program).
 */
static uint8_t handle_iq_data(ch_group_t * grp_ptr)
{
  int dev_num;
  uint16_t num_samples;
  ch_iq_sample_t *iq_ptr;       // pointer to an I/Q sample pair

  for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {

    ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

    if (ch_sensor_is_connected(dev_ptr)) {

      num_samples = ch_get_num_samples(dev_ptr);
      iq_ptr = chirp_data[dev_num].iq_data;

      printf("Read %d samples from device %d:\n", num_samples, dev_num);
    }
  }

  return 0;
}

/* ------------------------------------------ public functions starts from here ---------------------------------*/
int prox_init(void *para1, void *para2)
{
  chirp_init_parms = (platform_prams *) para1;

  ch_group_t *grp_ptr = &chirp_group;
  uint8_t chirp_error = 0;
  uint8_t num_ports;
  uint8_t dev_num;

  /* if REAR BOARD, use related source */
  if (chirp_init_parms->boardID == REAR_BOARD) {
    chirp_pin_prog[0] = PROG1_PIN;
    chirp_port_prog[0] = PROG1_PORT;
  }

  chbsp_board_init(grp_ptr);

  num_ports = ch_get_num_ports(grp_ptr);

  for (dev_num = 0; dev_num < num_ports; dev_num++) {
    ch_dev_t *dev_ptr = &(chirp_devices[dev_num]);      // init struct in array
    chirp_error |= ch_init(dev_ptr, grp_ptr, dev_num,
                           CHIRP_SENSOR_FW_INIT_FUNC);
  }

  if (chirp_error == 0) {
    printf("starting group... \r\n");
    chirp_error = ch_group_start(grp_ptr);
  }

  if (chirp_error == 0) {
    printf("OK\r\n");
  } else {
    printf("FAILED: %d\r\n", chirp_error);
  }
  printf("\r\n");

  chbsp_periodic_timer_init(MEASUREMENT_INTERVAL_MS, periodic_timer_callback);

  ch_io_int_callback_set(grp_ptr, sensor_int_callback);

  return chirp_error;
}

int prox_init_complete(void *para)
{
  ch_group_t *grp_ptr = &chirp_group;
  uint8_t num_ports;
  uint8_t dev_num;

  num_ports = ch_get_num_ports(grp_ptr);

  printf("Sensor\tType \t   Freq\t\t RTC Cal \tFirmware\r\n");

  for (dev_num = 0; dev_num < num_ports; dev_num++) {
    ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

    if (ch_sensor_is_connected(dev_ptr)) {

      printf("%d\tCH%d\t %u Hz\t%lu@%ums\t%s\r\n", dev_num,
             ch_get_part_number(dev_ptr), ch_get_frequency(dev_ptr),
             ch_get_rtc_cal_result(dev_ptr),
             ch_get_rtc_cal_pulselength(dev_ptr),
             ch_get_fw_version_string(dev_ptr));
    }
  }
  printf("\r\n");

  return 0;
}

int prox_activate(bool activate)
{
  ch_group_t *grp_ptr = &chirp_group;
  uint8_t chirp_error = 0;
  uint8_t num_connected = 0;
  uint8_t num_ports;
  uint8_t dev_num;

  num_ports = ch_get_num_ports(grp_ptr);

  if (activate) {
    //enable
    if (sensor_activate_count == 0) {
      for (dev_num = 0; dev_num < num_ports; dev_num++) {
        printf("Configuring sensor(s)...， dev_num: %d, num_ports: %d\r\n",
               dev_num, num_ports);
        ch_config_t dev_config;
        ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

        if (ch_sensor_is_connected(dev_ptr)) {

          /* Select sensor mode
           *   All connected sensors are placed in hardware triggered mode.
           *   The first connected (lowest numbered) sensor will transmit and
           *   receive, all others will only receive.
           */

          num_connected++;      // count one more connected
          active_devices |= (1 << dev_num);     // add to active device bit mask

          if (num_connected == 1) {     // if this is the first sensor
            dev_config.mode = CH_MODE_TRIGGERED_TX_RX;
            printf("****mode: %d\r\n", dev_config.mode);
          } else {
            dev_config.mode = CH_MODE_TRIGGERED_RX_ONLY;
          }

          /* Init config structure with values from hello_chirp.h */
          dev_config.max_range = CHIRP_SENSOR_MAX_RANGE_MM;
          dev_config.static_range = CHIRP_SENSOR_STATIC_RANGE;
          dev_config.sample_interval = CHIRP_SENSOR_SAMPLE_INTERVAL;

          /* Set detection thresholds (CH201 only) */
          if (ch_get_part_number(dev_ptr) == CH201_PART_NUMBER) {
            /* Set pointer to struct containing detection thresholds */
            dev_config.thresh_ptr = &chirp_ch201_thresholds;
          } else {
            dev_config.thresh_ptr = 0;
          }

          /* Apply sensor configuration */
          chirp_error = ch_set_config(dev_ptr, &dev_config);

          /* Enable sensor interrupt if using free-running mode
           *   Note that interrupt is automatically enabled if using
           *   triggered modes.
           */
          if ((!chirp_error) && (dev_config.mode == CH_MODE_FREERUN)) {
            printf("chirp interrupt enabled\r\n");
            chbsp_io_interrupt_enable(dev_ptr);
          }

          /* Read back and display config settings */
          if (!chirp_error) {
            display_config_info(dev_ptr);
          } else {
            printf("Device %d: Error during ch_set_config() \r\n", dev_num);
          }
        }
      }
      sensor_op_timer_enable(tim3);
      uitralsonic_proximity_enabled = true;
    }
    sensor_activate_count++;
  } else {
    if (sensor_activate_count == 0) {
      return SENSOR_FAILED;
    }
    sensor_activate_count--;
    if (sensor_activate_count == 0) {
      sensor_op_timer_disable(tim3);
      uitralsonic_proximity_enabled = false;
    }
  }

  return 0;
}

int prox_publish_sensor_data(void *para)
{
  int gpio_pin = 0;
  memcpy(&gpio_pin, para, sizeof(int));
  ch_group_t *grp_ptr = &chirp_group;
  ch_io_int_callback_t func_ptr = sensor_group_ptr->io_int_callback;
  uint8_t num_ports = sensor_group_ptr->num_ports;
  uint8_t pin_found = 0;
  uint8_t idx;

  printf("prox publish sensor event gpio: %d\r\n", gpio_pin);

  if (func_ptr != NULL) {
    for (idx = 0; idx < num_ports; idx++) {
      if (gpio_pin == chirp_pin_io[idx]) {
        pin_found = 1;
        break;
      }
    }

    if (pin_found) {
      // Call application callback function - pass I/O index to identify interrupting device
      (*func_ptr) (sensor_group_ptr, idx);
      handle_data_ready(grp_ptr);
    }
  }
  return 0;
}

int prox_config(uint8_t config_type, void *para)
{
  int res = 0;
  switch (config_type) {
  case SENSOR_CONFIG_TIMEOUT:
    //printf("@@@SENSOR_CONFIG_TIMEOUT!\r\n");
    chbsp_periodic_timer_handler();
    break;
  default:
    break;
  }
  return res;
}

int prox_publish_config_resp(void *para)
{
  return 0;
}
