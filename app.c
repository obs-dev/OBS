
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

#define BSP_GPIO_LED0_PORT            gpioPortA
#define BSP_GPIO_LED0_PIN             4
#define BURTC_IRQ_PERIOD              10000    // in milliseconds


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

 sl_led_init(&sl_led_led0);

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
      sl_led_toggle(&sl_led_led0);
      myCount = 0;
      myVar +=1;
      user_char_buf[0] = BURTC_CounterGet()/1000;
      user_char_buf[3] = myVar - 1;
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

