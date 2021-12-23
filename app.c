
/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include "em_common.h"
#include "sl_app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_burtc.h"
#include "em_rmu.h"
#include "stdio.h"
#include "sl_simple_led.h"
#include "sl_simple_led_instances.h"
#include "sl_i2cspm_instances.h"
#include "ustimer.h"
#include "sl_uartdrv_instances.h"
#include "sl_spidrv_instances.h"
#include "em_gpio.h"
#include "sl_emlib_gpio_simple_init.h"
#include "gpiointerrupt.h"
#include "em_rtc.h"
#include "em_rtcc.h"
#include "sl_sleeptimer.h"
#include "sl_status.h"

#include <stdbool.h>
#include <string.h>
#include <inttypes.h>

#define BSP_GPIO_LED0_PORT            gpioPortA
#define BSP_GPIO_LED0_PIN             4
#define BURTC_IRQ_PERIOD              10000    // in milliseconds

#define AS3935_IC_ADDRESS 0x00
#define AS3935_REG_GAIN  0x00
#define AS3935_REG_NOISE 0x01
#define AS3935_REG_STAT  0x02
#define AS3935_REG_INT   0x03
#define AS3935_REG_ELSB  0x04
#define AS3935_REG_EMSB  0x05
#define AS3935_REG_EMMSB 0x06
#define AS3935_REG_DIST  0x07
#define AS3935_REG_TUNE  0x08

#define AS3935_REG_PRESET_DEFAULT 0x3C
#define AS3935_REG_CALIB_RCO      0x3D
#define AS3935_DIRECT             0x96

// flash command definitions
#define FLASH_PAGE_PRGM     0x02
#define FLASH_READ          0x03
#define FLASH_READ_STATUS   0x05
#define FLASH_WREN          0x06
#define FLASH_CHIP_ERASE    0x60
#define FLASH_READ_ID       0x9F

uint8_t i2cWriteData[20];
uint8_t i2cReadData[20];
I2C_TransferReturn_TypeDef sta;
I2C_TransferSeq_TypeDef seq;

//Functions to interact with flash memory
void FlashReadId(void);
void FlashErase(void);
void FlashProgramWord(uint32_t address, uint32_t value);
uint8_t FlashCheckBusy(void);
uint32_t FlashReadWord(uint32_t address);

//Callback function for interrupt
void myCallback(void);
uint8_t AS3935_GetDistance(void);
uint32_t AS3935_GetEnergy(void);

uint8_t AS3935Flag = 0;

//variable to store lightning strike distance
uint8_t distance_light;

//sleeptimer data type to store date structure
sl_sleeptimer_date_t myDate;
char date_string[100];

//function to print lightning strike distance and date and time of strike
void printStrikeAndDate(uint8_t tempDistance);

typedef union {uint32_t FullWord; uint8_t WordPart[4];}UNION328;

uint32_t val_m; // value to save manufacturer ID for flash chip
uint32_t val_w; // value to check flash write routine functionality

uint8_t SPI_input[10];
uint8_t SPI_output[10];
char UART_RXData[10];
char UART_TXData[10];

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;
static uint8_t conn_handle = 0xff;
/* Allocate buffer to store the ut_user characteristic value */
static uint8_t user_char_buf[4] = { 0x02, 0x00, 0x00, 0x00 };

static void notify(uint16_t which);

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
  //Timer initialization
 CHIP_Init();
 //BURTC_IntClear((uint32_t)BURTC_IF_COMP);
 GPIO_PinOutToggle(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
 CMU_ClockSelectSet(cmuClock_EM4GRPACLK, cmuSelect_ULFRCO);
 CMU_ClockEnable(cmuClock_BURTC, true);

 BURTC_Init_TypeDef burtcInit = BURTC_INIT_DEFAULT;
 burtcInit.start = true;  //Starts the counter after initialization
 burtcInit.compare0Top = true; //Reset counter when counter reaches compare value

 BURTC_Init(&burtcInit);
 BURTC_CounterReset();
 BURTC_CompareSet(0, BURTC_IRQ_PERIOD);

 //BURTC_IntEnable(BURTC_IEN_COMP);
 NVIC_EnableIRQ(BURTC_IRQn);
 BURTC_Enable(true);
 BURTC_SyncWait();
 BURTC_Start();

// sl_led_init(&sl_led_led0);

 // interrupt initialization..................................
   sl_emlib_gpio_simple_init();
   GPIOINT_Init();
   GPIOINT_CallbackRegister(6,(GPIOINT_IrqCallbackPtr_t) myCallback);
   GPIO_ExtIntConfig(gpioPortA, 7, 6, true, false, true);

   sl_i2cspm_init_instances();
   sl_uartdrv_init_instances();
   sl_spidrv_init_instances();

   // init of the AS3935 ...........................................
   // default all parameters
   seq.addr = AS3935_IC_ADDRESS;
   seq.flags = I2C_FLAG_WRITE;
   i2cWriteData[0] = AS3935_REG_PRESET_DEFAULT;
   i2cWriteData[1] = AS3935_DIRECT;
   seq.buf[0].data = i2cWriteData;
   seq.buf[0].len = 2;
   I2CSPM_Transfer(sl_i2cspm_inst1, &seq);

   USTIMER_Delay(100000);

   // Calibrate RCOs
   seq.addr = AS3935_IC_ADDRESS;
   seq.flags = I2C_FLAG_WRITE;
   i2cWriteData[0] = AS3935_REG_CALIB_RCO;
   i2cWriteData[1] = AS3935_DIRECT;
   seq.buf[0].data = i2cWriteData;
   seq.buf[0].len = 2;
   I2CSPM_Transfer(sl_i2cspm_inst1, &seq);

   USTIMER_Delay(100000);

   // turn on disturbance masker
   seq.addr = AS3935_IC_ADDRESS;
   seq.flags = I2C_FLAG_WRITE;
   i2cWriteData[0] = AS3935_REG_INT;
   i2cWriteData[1] = 0xA0;
   seq.buf[0].data = i2cWriteData;
   seq.buf[0].len = 2;
   I2CSPM_Transfer(sl_i2cspm_inst1, &seq);

   // initialize the antenna parameters
 //  i2cWriteData[0] = AS3935_REG_TUNE;
 //  i2cWriteData[1] = 0x00;
 //  i2cWriteData[1] = 0x80;               // run this to see antenna freq on IRQ pin
 //  seq.flags = I2C_FLAG_WRITE;
 //  seq.buf[0].data = i2cWriteData;
 //  seq.buf[0].len = 2;
 //  I2CSPM_Transfer(sl_i2cspm_inst1, &seq);

   // initialize the noise floor
   i2cWriteData[0] = AS3935_REG_NOISE;
   i2cWriteData[1] = 0x41;               // noise floor and threshold
   seq.flags = I2C_FLAG_WRITE;
   seq.buf[0].data = i2cWriteData;
   seq.buf[0].len = 2;
   I2CSPM_Transfer(sl_i2cspm_inst1, &seq);


   // initialize the AFE
 //  i2cWriteData[0] = AS3935_REG_GAIN;
 //  i2cWriteData[1] = 0x24;               // gain
 //  seq.flags = I2C_FLAG_WRITE;
 //  seq.buf[0].data = i2cWriteData;
 //  seq.buf[0].len = 2;
 //  I2CSPM_Transfer(sl_i2cspm_inst1, &seq);

   // set leds
   GPIO_PinOutSet(gpioPortA, 0);
   GPIO_PinOutSet(gpioPortB, 2);
   // initialize timer
   USTIMER_Init();

   //initialize RTCC and sleeptimer
   CHIP_Init();
   CMU_ClockSelectSet(cmuClock_RTCCCLK, cmuSelect_LFRCO);
   CMU_ClockEnable(cmuClock_RTCC, true);
   sl_sleeptimer_init();

   //Set current date for sleeptimer
   myDate.day_of_week = DAY_WEDNESDAY;
   myDate.day_of_year = 356;
   myDate.hour = 13;
   myDate.min = 0;
   myDate.month = MONTH_DECEMBER;
   myDate.month_day = 22;
   myDate.sec = 0;
   myDate.time_zone = 0;
   myDate.year = 2021;
   sl_sleeptimer_build_datetime(&myDate, myDate.year, myDate.month,
                                 myDate.month_day, myDate.hour, myDate.min,
                                 myDate.sec, myDate.time_zone);
   sl_sleeptimer_set_datetime(&myDate);

   FlashErase();
   FlashReadId();

}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
static uint8_t myCount;
static uint8_t myVar;

  myCount +=1;
  if(myCount == 200){
//      sl_led_toggle(&sl_led_led0);
      myCount = 0;
      myVar +=1;
      user_char_buf[0] = BURTC_CounterGet()/1000;
      user_char_buf[3] = myVar - 1;

      if(AS3935Flag == 1){
            AS3935Flag = 0;
            GPIO_PinOutSet(gpioPortB,2);
            seq.addr = AS3935_IC_ADDRESS;
            seq.flags = I2C_FLAG_WRITE_READ;
            i2cWriteData[0] = AS3935_REG_INT;
            seq.buf[0].data = i2cWriteData;
            seq.buf[0].len = 1;
            seq.buf[1].data = i2cReadData;
            seq.buf[1].len = 1;
            sta = I2CSPM_Transfer(sl_i2cspm_inst1, &seq);

            // if there is a valid lighting hit
            if((i2cReadData[0] & 0x08)==0x08){
                AS3935_GetEnergy();
                distance_light = AS3935_GetDistance();
                printStrikeAndDate(distance_light);
            }
        }

        // is led blinking then code is running
      USTIMER_Delay(100000);
      GPIO_PinOutToggle(gpioPortA, 0);
      USTIMER_Delay(100000);
      GPIO_PinOutToggle(gpioPortB, 1);

  //  /*Testing Flash read routine*/
  //  FlashReadId();
  //  printf("value is 0x%2X%2X%2X%2X \n\r",
      //          SPI_input[0],SPI_input[1],SPI_input[2],SPI_input[3]);

  //  val_m = FlashReadWord(0x0000);
  //      //0xC2 should be printed to the screen as that is the manufacturer ID,
  //  printf("Manufacturer ID read correctly, 0x%2lX \n\r", val_m);


 //        /*Testing the Flash erase routine*/
  //  FlashErase();
  //   //Check status register WIP and WEL bits
  //  if(FlashCheckBusy()== 0x00){
  //     printf("Flash erase routine is working \n\r");
  //   }


  //  /*Testing the Flash write routine*/
  //  FlashProgramWord(0x00000000, 0x67686970);
  //  while(FlashCheckBusy()){} //wait till status register WIP bit is 0
  //  val_w = FlashReadWord(0x00000000);
  //  if(val_w == 0x67686970){
  //    printf("Flash write routine is working \n\r");
  //    }
  }

}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Print boot message.
      app_log("Bluetooth stack booted: v%d.%d.%d-b%d\n",
                 evt->data.evt_system_boot.major,
                 evt->data.evt_system_boot.minor,
                 evt->data.evt_system_boot.patch,
                 evt->data.evt_system_boot.build);

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to get Bluetooth address\n",
                    (int)sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      app_log("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                 address_type ? "static random" : "public device",
                 address.addr[5],
                 address.addr[4],
                 address.addr[3],
                 address.addr[2],
                 address.addr[1],
                 address.addr[0]);

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to write attribute\n",
                    (int)sc);

      app_log("boot event - starting advertising\r\n");

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to create advertising set\n",
                    (int)sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to set advertising timing\n",
                    (int)sc);
      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_general_discoverable,
        advertiser_connectable_scannable);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to start advertising\n",
                    (int)sc);
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      app_log("connection opened\r\n");
      conn_handle = evt->data.evt_connection_opened.connection;
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      app_log("connection closed, reason: 0x%2.2x\r\n", evt->data.evt_connection_closed.reason);
      conn_handle = 0xff;
      // Restart advertising after client has disconnected.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_general_discoverable,
        advertiser_connectable_scannable);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to start advertising\n",
                    (int)sc);
      break;

      /* TAG: when the remote device subscribes for notification,
       * start a timer to send out notifications periodically */
      case sl_bt_evt_gatt_server_characteristic_status_id:
        if (evt->data.evt_gatt_server_characteristic_status.status_flags
            != gatt_server_client_config) {
          break;
        }
        if (!(evt->data.evt_gatt_server_characteristic_status.characteristic
              == gattdb_vt_hex
              || evt->data.evt_gatt_server_characteristic_status.characteristic
              == gattdb_vt_user)) {
          break;
        }
        /* use the gattdb handle as the timer handle here */
        sc = sl_bt_system_set_soft_timer(
            evt->data.evt_gatt_server_characteristic_status.client_config_flags
            ? 32768 * 3 : 0,
            evt->data.evt_gatt_server_characteristic_status.characteristic,
            0);
        sl_app_assert(sc == SL_STATUS_OK,
                      "[E: 0x%04x] Failed to start/stop software timer\n",
                      (int)sc);
      break;

      case sl_bt_evt_system_soft_timer_id:
        notify(evt->data.evt_system_soft_timer.handle);
        break;

      /* TAG: When a "hex" type characteristic is written, the Bluetooth stack
       * stores the value and notifies the application about the change with this event */
      case sl_bt_evt_gatt_server_attribute_value_id:
        if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_vt_hex) {
          app_log("Characterisitic <%u> value changed by a remote request.\n"
                   "Application callback here\n", gattdb_vt_hex);
        }
        break;

      /* TAG: Example of handling read request for reading a characteristic with "user" type */
      case sl_bt_evt_gatt_server_user_read_request_id:
      {
        uint16_t sent_len;

        if (evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_vt_user) {
          /* For simplicity, not consider the offset > length of the buffer situation */
          sc = sl_bt_gatt_server_send_user_read_response(
            evt->data.evt_gatt_server_user_read_request.connection,
            gattdb_vt_user,
            (uint8_t)SL_STATUS_OK,
            sizeof(user_char_buf) - evt->data.evt_gatt_server_user_read_request.offset,
            user_char_buf + evt->data.evt_gatt_server_user_read_request.offset,
            &sent_len);
          sl_app_assert(sc == SL_STATUS_OK,
                        "[E: 0x%04x] Failed to send a read response\n",
                        (int)sc);
        }
      }
      break;

      /* TAG: Example of handling write request for writing a characteristic with "user" type */
      case sl_bt_evt_gatt_server_user_write_request_id:
        if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_vt_user) {
          /* For simplicity, assuming the opcode is gatt_write_request */
          uint16_t len = evt->data.evt_gatt_server_user_write_request.offset
                         + evt->data.evt_gatt_server_user_write_request.value.len;
          if (len <= sizeof(user_char_buf)) {
            memcpy(user_char_buf + evt->data.evt_gatt_server_user_write_request.offset,
                   evt->data.evt_gatt_server_user_write_request.value.data,
                   evt->data.evt_gatt_server_user_write_request.value.len);
            sc = sl_bt_gatt_server_send_user_write_response(
              evt->data.evt_gatt_server_user_write_request.connection,
              evt->data.evt_gatt_server_user_write_request.characteristic,
              (uint8_t)SL_STATUS_OK);
            sl_app_assert(sc == SL_STATUS_OK,
                          "[E: 0x%04x] Failed to send a write response\n",
                          (int)sc);
          } else {
            sc = sl_bt_gatt_server_send_user_write_response(
              evt->data.evt_gatt_server_user_write_request.connection,
              evt->data.evt_gatt_server_user_write_request.characteristic,
              (uint8_t)SL_STATUS_BT_ATT_INVALID_ATT_LENGTH);
            sl_app_assert(sc == SL_STATUS_OK,
                          "[E: 0x%04x] Failed to send a write response\n",
                          (int)sc);
          }
        }
        break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}

/*
 * For simplicity and demonstration the change, the function will increment
 * the first byte of the characteristic value and send.
 */
static void notify(uint16_t which)
{
  sl_status_t sc;
  size_t len;
  uint8_t data[4];

  if (which == gattdb_vt_user) {
    /* TAG: Example of sending notification for a "user" characteristic value
     * and modify the value */
    sc = sl_bt_gatt_server_send_notification(conn_handle,
                                              which,
                                              sizeof(user_char_buf),
                                              user_char_buf);
    sl_app_assert(sc == SL_STATUS_OK,
                  "[E: 0x%04x] Failed to send a notification\n",
                  (int)sc);
  //  user_char_buf[0]++;
  } else if (which  == gattdb_vt_hex) {
    /* TAG: Example of sending notification for a "hex" characteristic value and
     * modify the value */
    uint8_t tmp[4] = { 0 };
    sc = sl_bt_gatt_server_read_attribute_value(which,
                                                 0,
                                                 4,
                                                 &len,
                                                 data);
    sl_app_assert(sc == SL_STATUS_OK,
                  "[E: 0x%04x] Failed to read attribute\n",
                  (int)sc);
    memcpy(tmp, data, 4);

    sc = sl_bt_gatt_server_send_notification(conn_handle,
                                              which,
                                              4,
                                              tmp);
    sl_app_assert(sc == SL_STATUS_OK,
                  "[E: 0x%04x] Failed to send a notification\n",
                  (int)sc);
    tmp[0]++;
    sc = sl_bt_gatt_server_write_attribute_value(which,
                                                  0,
                                                  1,
                                                  tmp);
    sl_app_assert(sc == SL_STATUS_OK,
                  "[E: 0x%04x] Failed to write attribute\n",
                  (int)sc);
  }
}

uint32_t AS3935_GetEnergy(void){
union {uint32_t word; uint8_t data[4];}energy;

  seq.addr = AS3935_IC_ADDRESS;
  seq.flags = I2C_FLAG_WRITE_READ;
  i2cWriteData[0] = AS3935_REG_ELSB;
  seq.buf[0].data = i2cWriteData;
  seq.buf[0].len = 1;
  seq.buf[1].data = i2cReadData;
  seq.buf[1].len = 3;
  sta = I2CSPM_Transfer(sl_i2cspm_inst1, &seq);
  energy.word = 0;
  energy.data[0] = i2cReadData[0];
  energy.data[1] = i2cReadData[1];
  energy.data[2] = i2cReadData[2];

  return(energy.word);
}


uint8_t AS3935_GetDistance(void){
uint8_t distance;
  seq.addr = AS3935_IC_ADDRESS;
  seq.flags = I2C_FLAG_WRITE_READ;
  i2cWriteData[0] = AS3935_REG_DIST;
  seq.buf[0].data = i2cWriteData;
  seq.buf[0].len = 1;
  seq.buf[1].data = i2cReadData;
  seq.buf[1].len = 1;

  sta = I2CSPM_Transfer(sl_i2cspm_inst1, &seq);

  distance = i2cReadData[0] &0x3F;

  return(distance);

}


void myCallback(void){
  GPIO_PinOutClear(gpioPortB,2);
  AS3935Flag = 1;
}

void printStrikeAndDate(uint8_t tempDistance){
  sl_status_t status = sl_sleeptimer_get_datetime(&myDate);
  const uint8_t *formatS = "%A, %B %d %Y - %I:%M:%S%p";
  uint32_t check = sl_sleeptimer_convert_date_to_str(date_string,
                                                     sizeof(date_string),
                                                     formatS, &myDate);

  if(status == SL_STATUS_OK && check != 0){
      printf("The estimated lightning strike distance "
                 "value is %d. The strike occurred on %s\n\r",
                  tempDistance, date_string);
    }
  else{
      printf("Sleeptimer date extraction error.\n\r");
      }
}


// ================= Flash Routines =================

void FlashReadId(void){
  uint8_t i;
  SPI_output[0] = FLASH_READ_ID;    // command
  SPI_output[1] = 0x00;             // dummy
  SPI_output[2] = 0x00;             // dummy
  SPI_output[3] = 0x00;             // dummy

  SPI_input[0] = 0x00;              // dummy
  SPI_input[1] = 0x00;              // manuf ID
  SPI_input[2] = 0x00;              // device ID part 1
  SPI_input[3] = 0x00;              // device ID part 2


  GPIO_PinOutClear (gpioPortC, 3);    // sets flash chip select

  for (i = 0; i < 4; i++)
    {
      SPI_input[i] = USART_SpiTransfer (USART0, SPI_output[i]);
    }

    GPIO_PinOutSet(gpioPortC, 3);       // clears flash chip select

}

uint32_t FlashReadWord(uint32_t address){

//UNION328 TXData;
UNION328 RXData;
UNION328 TXAddress;
uint8_t scratch;

  TXAddress.FullWord = address;

  GPIO_PinOutClear (gpioPortC, 3);
  scratch = USART_SpiTransfer (USART0, FLASH_READ);        // send command
  scratch = USART_SpiTransfer (USART0, TXAddress.WordPart[2]);  // send address
  scratch = USART_SpiTransfer (USART0, TXAddress.WordPart[1]);  // send address
  scratch = USART_SpiTransfer (USART0, TXAddress.WordPart[0]);  // send address
  RXData.WordPart[3] = USART_SpiTransfer (USART0, 0x00);        // send value
  RXData.WordPart[2] = USART_SpiTransfer (USART0, 0x00);        // send value
  RXData.WordPart[1] = USART_SpiTransfer (USART0, 0x00);        // send value
  RXData.WordPart[0] = USART_SpiTransfer (USART0, 0x00);        // send value
  GPIO_PinOutSet(gpioPortC, 3);

  return(RXData.FullWord);
}




void FlashErase(void){
uint32_t i;
uint8_t busyFlag = 1;

  // write enable
  SPI_output[0] = FLASH_WREN;
  GPIO_PinOutClear (gpioPortC, 3);
  SPI_input[0] = USART_SpiTransfer (USART0, SPI_output[0]);
  GPIO_PinOutSet(gpioPortC, 3);

  // send command to erase flash
  SPI_output[0] = FLASH_CHIP_ERASE;
  GPIO_PinOutClear (gpioPortC, 3);
  SPI_input[0] = USART_SpiTransfer (USART0, SPI_output[0]);
  GPIO_PinOutSet(gpioPortC, 3);

  // check status and wait for it complete
  while(busyFlag !=0){
    for(i=0;i<1000;i++){}
    busyFlag = FlashCheckBusy();
  }
}


void FlashProgramWord(uint32_t address, uint32_t value){
UNION328 TXData;
UNION328 TXAddress;
uint8_t scratch;

  // write enable
  SPI_output[0] = FLASH_WREN;
  GPIO_PinOutClear (gpioPortC, 3);
  SPI_input[0] = USART_SpiTransfer (USART0, SPI_output[0]);
  GPIO_PinOutSet(gpioPortC, 3);



  TXAddress.FullWord = address;
  TXData.FullWord = value;

  GPIO_PinOutClear (gpioPortC, 3);
  scratch = USART_SpiTransfer (USART0, FLASH_PAGE_PRGM);        // send command
  scratch = USART_SpiTransfer (USART0, TXAddress.WordPart[2]);  // send address
  scratch = USART_SpiTransfer (USART0, TXAddress.WordPart[1]);  // send address
  scratch = USART_SpiTransfer (USART0, TXAddress.WordPart[0]);  // send address
  scratch = USART_SpiTransfer (USART0, TXData.WordPart[3]);     // send value
  scratch = USART_SpiTransfer (USART0, TXData.WordPart[2]);     // send value
  scratch = USART_SpiTransfer (USART0, TXData.WordPart[1]);     // send value
  scratch = USART_SpiTransfer (USART0, TXData.WordPart[0]);     // send value
  GPIO_PinOutSet(gpioPortC, 3);

}


uint8_t FlashCheckBusy(void){
uint8_t busy;
    SPI_output[0] = FLASH_READ_STATUS;
    SPI_output[1] = 0xFF;                 // dummy
    GPIO_PinOutClear (gpioPortC, 3);
    SPI_input[0] = USART_SpiTransfer (USART0, SPI_output[0]);   // sent command
    SPI_input[1] = USART_SpiTransfer (USART0, SPI_output[1]);   // dummy
    GPIO_PinOutSet(gpioPortC, 3);
    busy  = SPI_input[1] & 0x01;
    return (busy);
}

