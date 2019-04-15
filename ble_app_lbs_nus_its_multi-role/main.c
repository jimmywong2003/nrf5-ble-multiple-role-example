/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "app_uart.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"

#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "app_scheduler.h"
#include "nrf_delay.h"

#include "ble_conn_state.h"
#include "nrf_ble_scan.h"

#include "ble_lbs.h"
#include "ble_nus.h"
#include "ble_image_transfer_service.h"

#include "ble_nus_c.h"
#include "ble_lbs_c.h"
#include "ble_image_transfer_service_c.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */
#define ADV_BUTTON                      BSP_BUTTON_1
#define SCAN_BUTTON                     BSP_BUTTON_2
#define PRINT_BUTTON                    BSP_BUTTON_3

#define DEVICE_NAME                     "LBS_NUS_Node"                         /**< Name of device. Will be included in the advertising data. */

#define LINK_TOTAL                      NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + \
        NRF_SDH_BLE_CENTRAL_LINK_COUNT

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */
#define ITS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN+1                 /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(6000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(50000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(20000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SCHED_MAX_EVENT_DATA_SIZE           APP_TIMER_SCHED_EVENT_DATA_SIZE            /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                    20                                         /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE                    10                                         /**< Maximum number of events in the scheduler queue. */
#endif

/* Central Role for scanning paramater */
#define SCAN_INTERVAL               160//0x00F0                         /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                 80// 0x0050                         /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT                0x0200                              /**< Timout when scanning. 0x0000 disables timeout. */


/* Default TX Power */
#ifdef NRF52840_XXAA
#define APP_DEFAULT_TX_POWER        (8)
#else
#define APP_DEFAULT_TX_POWER        (4)
#endif

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
BLE_LBS_DEF(m_lbs);                                                             /**< LED Button Service instance. */
BLE_ITS_DEF(m_its, NRF_SDH_BLE_TOTAL_LINK_COUNT); /**< BLE IMAGE TRANSFER service instance. */

NRF_BLE_SCAN_DEF(m_scan);                                       /**< Scanning module instance. */

/* Central role GATT Client */
BLE_NUS_C_ARRAY_DEF(m_ble_nus_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
BLE_LBS_C_ARRAY_DEF(m_ble_lbs_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);           /**< LED button client instances. */
BLE_ITS_C_ARRAY_DEF(m_ble_its_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);          /**< BLE Nordic Image Transfer Service (ITS) client instance. */

BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */


NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
//NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_TOTAL_LINK_COUNT);                          /**< Context for the Queued Write module.*/

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */


static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;              /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static uint16_t m_ble_its_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static char const m_target_periph_name[] = "LBS_NUS_Node";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/


typedef struct
{
        bool is_connected;
        ble_gap_addr_t address;
} conn_peer_t;

static char * roles_str[] =
{
        "INVALID_ROLE",
        "PERIPHERAL",
        "CENTRAL",
};

static conn_peer_t m_connected_peers[NRF_SDH_BLE_TOTAL_LINK_COUNT];                                /**< Array of connected peers. */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
        .adv_data =
        {
                .p_data = m_enc_advdata,
                .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
        },
        .scan_rsp_data =
        {
                .p_data = m_enc_scan_response_data,
                .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

        }
};

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
        app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
        bsp_board_init(BSP_INIT_LEDS);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
        // Initialize timer module, making it use the scheduler
        ret_code_t err_code = app_timer_init();
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for changing the tx power.
 */
static void tx_power_set(void)
{
        ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, APP_DEFAULT_TX_POWER);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
        ret_code_t err_code;
        ble_gap_conn_params_t gap_conn_params;
        ble_gap_conn_sec_mode_t sec_mode;

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

        err_code = sd_ble_gap_device_name_set(&sec_mode,
                                              (const uint8_t *)DEVICE_NAME,
                                              strlen(DEVICE_NAME));
        APP_ERROR_CHECK(err_code);

        memset(&gap_conn_params, 0, sizeof(gap_conn_params));

        gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
        gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
        gap_conn_params.slave_latency     = SLAVE_LATENCY;
        gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

        err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
        APP_ERROR_CHECK(err_code);

//        ble_gap_addr_t ble_address = {.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
//                                      .addr_id_peer = 0,
//                                      .addr = {0xC3,0x11,0x99,0x33,0x44,0xFF}};
//        err_code = sd_ble_gap_addr_set(&ble_address);
//        APP_ERROR_CHECK(err_code);
}


static void its_data_handler(ble_its_t *p_its, uint8_t const *p_data, uint16_t length)
{
        //if (p_evt->type == BLE_ITS_EVT_RX_DATA)
        {


        }
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
        if (p_evt->type == BLE_NUS_EVT_RX_DATA)
        {
                uint32_t err_code;

                NRF_LOG_DEBUG("Receive the BLE UART data");
                for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
                {
                        do
                        {
                                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                                {
                                        NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                                        APP_ERROR_CHECK(err_code);
                                }
                        } while (err_code == NRF_ERROR_BUSY);
                }
                if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
                {
                        while (app_uart_put('\n') == NRF_ERROR_BUSY);
                }
                NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        }
        else if (p_evt->type == BLE_NUS_EVT_COMM_STARTED)
        {
                NRF_LOG_DEBUG("NUS : BLE_NUS_EVT_COMM_STARTED");
        }
        else if (p_evt->type == BLE_NUS_EVT_COMM_STOPPED)
        {
                NRF_LOG_DEBUG("NUS : BLE_NUS_EVT_COMM_STOPPED");
        }
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
        ret_code_t err_code;

        NRF_LOG_INFO("Start to Scanning!!")
        err_code = nrf_ble_scan_start(&m_scan);
        APP_ERROR_CHECK(err_code);


}

/**@brief Function for handling Scaning events.
 *
 * @param[in]   p_scan_evt   Scanning event.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
        ret_code_t err_code;

        switch(p_scan_evt->scan_evt_id)
        {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
                err_code = p_scan_evt->params.connecting_err.err_code;
                APP_ERROR_CHECK(err_code);
                break;

        case NRF_BLE_SCAN_EVT_CONNECTED:
        {
                ble_gap_evt_connected_t const * p_connected =
                        p_scan_evt->params.connected.p_connected;
                // Scan is automatically stopped by the connection.
                NRF_LOG_INFO("Connecting to target 0x%02x%02x%02x%02x%02x%02x",
                             p_connected->peer_addr.addr[0],
                             p_connected->peer_addr.addr[1],
                             p_connected->peer_addr.addr[2],
                             p_connected->peer_addr.addr[3],
                             p_connected->peer_addr.addr[4],
                             p_connected->peer_addr.addr[5]
                             );
        } break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
                NRF_LOG_INFO("Scan timed out.");
                scan_start();
        } break;
        default:
                break;
        }
}


static void scan_init(void)
{
        ret_code_t err_code;
        nrf_ble_scan_init_t init_scan;

        memset(&init_scan, 0, sizeof(init_scan));

        init_scan.connect_if_match = true;
        init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

        err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
        APP_ERROR_CHECK(err_code);

        // Setting filters for scanning.
        err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
        // ble_lbs_on_db_disc_evt(&m_ble_lbs_c, p_evt);
        // ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
        NRF_LOG_DEBUG("call to ble_lbs_on_db_disc_evt for instance %d and link 0x%x!",
                      p_evt->conn_handle,
                      p_evt->conn_handle);
        ble_nus_c_on_db_disc_evt(&m_ble_nus_c[p_evt->conn_handle], p_evt);
        ble_lbs_on_db_disc_evt(&m_ble_lbs_c[p_evt->conn_handle], p_evt);
        ble_its_c_on_db_disc_evt(&m_ble_its_c[p_evt->conn_handle], p_evt);
}

/** @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
        ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
        APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
        uint32_t data_length;
        if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                //m_ble_params_info.mtu = m_ble_its_max_data_len;

                m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                m_ble_its_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                NRF_LOG_DEBUG("gatt_event: ATT MTU is set to 0x%X (%d)", data_length, data_length);
        }
        else if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED))
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH - 4;
                m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                m_ble_its_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                NRF_LOG_DEBUG("gatt_event: Data len is set to 0x%X (%d)", data_length, data_length);
        }
        //NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
        //            p_gatt->att_mtu_desired_central,
        //        p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
        ret_code_t err_code;

        err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
        ret_code_t err_code;
        ble_advdata_t advdata;
        ble_advdata_t srdata;

        ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};

        // Build and set advertising data.
        memset(&advdata, 0, sizeof(advdata));

        advdata.name_type          = BLE_ADVDATA_FULL_NAME;
        advdata.include_appearance = true;
        advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


        memset(&srdata, 0, sizeof(srdata));
        srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
        srdata.uuids_complete.p_uuids  = adv_uuids;

        err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
        APP_ERROR_CHECK(err_code);

        err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
        APP_ERROR_CHECK(err_code);

        ble_gap_adv_params_t adv_params;

        // Set advertising parameters.
        memset(&adv_params, 0, sizeof(adv_params));

        adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
        adv_params.duration        = APP_ADV_DURATION;
        adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
        adv_params.p_peer_addr     = NULL;
        adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
        adv_params.interval        = APP_ADV_INTERVAL;

        err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
        APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state)
{
        if (led_state)
        {
                bsp_board_led_on(LEDBUTTON_LED);
                NRF_LOG_DEBUG("Received LED ON!");
        }
        else
        {
                bsp_board_led_off(LEDBUTTON_LED);
                NRF_LOG_DEBUG("Received LED OFF!");
        }
}
/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
        static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
        static uint8_t index = 0;
        uint32_t err_code;

        switch (p_event->evt_type)
        {
        case APP_UART_DATA_READY:
                UNUSED_VARIABLE(app_uart_get(&data_array[index]));
                index++;

                if ((data_array[index - 1] == '\n') ||
                    (data_array[index - 1] == '\r') ||
                    (index >= m_ble_nus_max_data_len))
                {
                        if (index > 1)
                        {
                                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                                NRF_LOG_HEXDUMP_INFO(data_array, index);
                                do
                                {
                                        uint16_t length = (uint16_t)index;
                                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                                            (err_code != NRF_ERROR_RESOURCES) &&
                                            (err_code != NRF_ERROR_NOT_FOUND))
                                        {
                                                APP_ERROR_CHECK(err_code);
                                        }
                                } while (err_code == NRF_ERROR_RESOURCES);
                        }
                        // app_uart_flush();
                        index = 0;
                }

                break;

        case APP_UART_COMMUNICATION_ERROR:
                APP_ERROR_HANDLER(p_event->data.error_communication);
                break;

        case APP_UART_FIFO_ERROR:
                APP_ERROR_HANDLER(p_event->data.error_code);
                break;

        default:
                break;
        }
}

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
        uint32_t err_code;
        app_uart_comm_params_t const comm_params =
        {
                .rx_pin_no    = RX_PIN_NUMBER,
                .tx_pin_no    = TX_PIN_NUMBER,
                .rts_pin_no   = RTS_PIN_NUMBER,
                .cts_pin_no   = CTS_PIN_NUMBER,
                .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
                .use_parity   = false,
#if defined (UART_PRESENT)
                .baud_rate    = NRF_UART_BAUDRATE_115200
#else
                .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
        };

        APP_UART_FIFO_INIT(&comm_params,
                           UART_RX_BUF_SIZE,
                           UART_TX_BUF_SIZE,
                           uart_event_handle,
                           APP_IRQ_PRIORITY_LOWEST,
                           err_code);
        APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
        ret_code_t err_code;
        ble_lbs_init_t lbs_init     = {0};
        nrf_ble_qwr_init_t qwr_init = {0};
        ble_nus_init_t nus_init;
        ble_its_init_t its_init;

        // // Initialize Queued Write Module.
        // qwr_init.error_handler = nrf_qwr_error_handler;
        //
        // err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
        // APP_ERROR_CHECK(err_code);

        // Initialize Queued Write Module instances.
        qwr_init.error_handler = nrf_qwr_error_handler;

        for (uint32_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
        {
                err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init);
                APP_ERROR_CHECK(err_code);
        }


        // Initialize LBS.
        lbs_init.led_write_handler = led_write_handler;

        err_code = ble_lbs_init(&m_lbs, &lbs_init);
        APP_ERROR_CHECK(err_code);

        // Initialize NUS.
        memset(&nus_init, 0, sizeof(nus_init));

        nus_init.data_handler = nus_data_handler;

        err_code = ble_nus_init(&m_nus, &nus_init);

        APP_ERROR_CHECK(err_code);

        // Initialize ITS.
        memset(&its_init, 0, sizeof(its_init));

        its_init.data_handler = its_data_handler;

        err_code = ble_its_init(&m_its, &its_init);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
        ret_code_t err_code;

        if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
        {
                err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
                APP_ERROR_CHECK(err_code);
        }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
        APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
        ret_code_t err_code;
        ble_conn_params_init_t cp_init;

        memset(&cp_init, 0, sizeof(cp_init));

        cp_init.p_conn_params                  = NULL;
        cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
        cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
        cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
        cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
        cp_init.disconnect_on_fail             = false;
        cp_init.evt_handler                    = on_conn_params_evt;
        cp_init.error_handler                  = conn_params_error_handler;

        err_code = ble_conn_params_init(&cp_init);
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
        ret_code_t err_code;

        NRF_LOG_INFO("Start to advertising!");

        err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
        APP_ERROR_CHECK(err_code);

        bsp_board_led_on(ADVERTISING_LED);
}

static void on_peripheral_connected(ble_gap_evt_t const * p_gap_evt)
{
        ret_code_t err_code;

        uint32_t periph_link_cnt = ble_conn_state_peripheral_conn_count();    // Number of peripheral links.

        NRF_LOG_INFO("on_peripheral_connected");
        NRF_LOG_INFO("Connection with link 0x%x established.", p_gap_evt->conn_handle);


        err_code = app_button_enable();
        APP_ERROR_CHECK(err_code);

        if (periph_link_cnt == NRF_SDH_BLE_PERIPHERAL_LINK_COUNT)
        {
                bsp_board_led_off(ADVERTISING_LED);
        }
        else
        {
                // Continue advertising. More connections can be established because the maximum link count has not been reached.
                advertising_start();
        }

}


static void on_peripheral_disconnected(ble_gap_evt_t const * p_gap_evt)
{

        ret_code_t err_code;

        NRF_LOG_DEBUG("on_peripheral_disconnected");

        uint32_t periph_link_cnt = ble_conn_state_peripheral_conn_count();    // Number of peripheral links.

        NRF_LOG_INFO("Connection 0x%x has been disconnected. Reason: 0x%X",
                     p_gap_evt->conn_handle,
                     p_gap_evt->params.disconnected.reason);

        if (periph_link_cnt == 0)
        {
                bsp_board_led_off(CONNECTED_LED);
                err_code = app_button_disable();
                APP_ERROR_CHECK(err_code);
        }

        if (periph_link_cnt == (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT - 1))
        {
                // Advertising is not running when all connections are taken, and must therefore be started.
                advertising_start();
        }
}

static void on_central_connected(ble_gap_evt_t const * p_gap_evt)
{
        ret_code_t err_code;

        NRF_LOG_INFO("Connection 0x%x established, starting DB discovery.",
                     p_gap_evt->conn_handle);

        APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);

        err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
        APP_ERROR_CHECK(err_code);

        err_code = ble_nus_c_handles_assign(&m_ble_nus_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
        APP_ERROR_CHECK(err_code);

        err_code = ble_its_c_handles_assign(&m_ble_its_c[p_gap_evt->conn_handle], p_gap_evt->conn_handle, NULL);
        APP_ERROR_CHECK(err_code);

        err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle], p_gap_evt->conn_handle);
        APP_ERROR_CHECK(err_code);

        // tx_power_set();

        // Update LEDs status and check whether it is needed to look for more
        // peripherals to connect to.
        // bsp_board_led_on(CENTRAL_CONNECTED_LED);
        if (ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT)
        {
                // bsp_board_led_off(CENTRAL_SCANNING_LED);
        }
        else
        {
                // Resume scanning.
                scan_start();
        }

}

static void on_central_disconnected(ble_gap_evt_t const * p_gap_evt)
{
        ret_code_t err_code;
}

/**@brief Function for checking whether a link already exists with a newly connected peer.
 *
 * @details This function checks whether the newly connected device is already connected.
 *
 * @param[in]   p_connected_evt Bluetooth connected event.
 * @return                      True if the peer's address is found in the list of connected peers,
 *                              false otherwise.
 */
static bool is_already_connected(ble_gap_addr_t const * p_connected_adr)
{
        for (uint32_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
        {
                if (m_connected_peers[i].is_connected)
                {
                        if (m_connected_peers[i].address.addr_type == p_connected_adr->addr_type)
                        {
                                if (memcmp(m_connected_peers[i].address.addr,
                                           p_connected_adr->addr,
                                           sizeof(m_connected_peers[i].address.addr)) == 0)
                                {
                                        return true;
                                }
                        }
                }
        }
        return false;
}

static void print_connection_status(void)
{
        for (uint8_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
        {
                if (m_connected_peers[i].is_connected)
                {
                        // NRF_LOG_INFO("%02d, Connecting to target 0x%02x%02x%02x%02x%02x%02x", i,  \
                        //              m_connected_peers[i].address.addr[5],  \
                        //              m_connected_peers[i].address.addr[4],  \
                        //              m_connected_peers[i].address.addr[3],  \
                        //              m_connected_peers[i].address.addr[2],  \
                        //              m_connected_peers[i].address.addr[1],  \
                        //              m_connected_peers[i].address.addr[0],  \
                        //              );
                }

        }

}
/**@brief Function for assigning new connection handle to the available instance of QWR module.
 *
 * @param[in] conn_handle New connection handle.
 */
static void multi_qwr_conn_handle_assign(uint16_t conn_handle)
{
        for (uint32_t i = 0; i < NRF_SDH_BLE_TOTAL_LINK_COUNT; i++)
        {
                if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID)
                {
                        ret_code_t err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], conn_handle);
                        APP_ERROR_CHECK(err_code);
                        break;
                }
        }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
        ret_code_t err_code;
        // For readability.
        uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

        uint16_t role        = ble_conn_state_role(conn_handle);
        ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

        if (    (p_ble_evt->header.evt_id == BLE_GAP_EVT_CONNECTED)
                &&  (is_already_connected(&p_ble_evt->evt.gap_evt.params.connected.peer_addr)))
        {
                NRF_LOG_INFO("%s: Already connected to this device as %s (handle: %d), disconnecting.",
                             (role == BLE_GAP_ROLE_PERIPH) ? "PERIPHERAL" : "CENTRAL",
                             (role == BLE_GAP_ROLE_PERIPH) ? "CENTRAL"    : "PERIPHERAL",
                             conn_handle);

                (void)sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

                // Do not process the event further.
                return;
        }

        switch (p_ble_evt->header.evt_id)
        {
        case BLE_GAP_EVT_CONNECTED:

                // Assign connection handle to available instance of QWR module.
                multi_qwr_conn_handle_assign(conn_handle);

                m_connected_peers[conn_handle].is_connected = true;
                m_connected_peers[conn_handle].address = p_ble_evt->evt.gap_evt.params.connected.peer_addr;

                // Handle central connections
                if (p_gap_evt->params.connected.role == BLE_GAP_ROLE_PERIPH)
                {
                        on_peripheral_connected(p_gap_evt);
                }
                else if ((p_gap_evt->params.connected.role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT))
                {
                        on_central_connected(p_gap_evt);
                }
                break;

        case BLE_GAP_EVT_DISCONNECTED:
                NRF_LOG_INFO("Connection 0x%x has been disconnected. Reason: 0x%X",
                             p_gap_evt->conn_handle,
                             p_gap_evt->params.disconnected.reason);
                NRF_LOG_INFO("p_gap_evt->params.connected.role = %x", p_gap_evt->params.connected.role);
                memset(&m_connected_peers[conn_handle], 0x00, sizeof(m_connected_peers[0]));

                // Handle central connections
                if (p_gap_evt->params.connected.role == BLE_GAP_ROLE_PERIPH)
                {
                        on_peripheral_disconnected(p_gap_evt);
                }
                else if (p_gap_evt->params.connected.role == BLE_GAP_ROLE_CENTRAL)
                {
                        on_central_disconnected(p_gap_evt);
                }

                scan_start();

                break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
                // Pairing not supported
                err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                       BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                       NULL,
                                                       NULL);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
                NRF_LOG_DEBUG("PHY update request.");
                ble_gap_phys_t const phys =
                {
                        .rx_phys = BLE_GAP_PHY_AUTO,
                        .tx_phys = BLE_GAP_PHY_AUTO,
                };
                err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
                APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
                // No system attributes have been stored.
                err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
                APP_ERROR_CHECK(err_code);
                break;

        case BLE_GATTC_EVT_TIMEOUT:
                // Disconnect on GATT Client timeout event.
                NRF_LOG_DEBUG("GATT Client Timeout.");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);

                break;

        case BLE_GATTS_EVT_TIMEOUT:
                // Disconnect on GATT Server timeout event.
                NRF_LOG_DEBUG("GATT Server Timeout.");
                err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
                break;

        default:
                // No implementation needed.
                break;
        }
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
        APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
        ret_code_t ret_val;

        NRF_LOG_DEBUG("Receiving data.");
        NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);

        for (uint32_t i = 0; i < data_len; i++)
        {
                do
                {
                        ret_val = app_uart_put(p_data[i]);
                        if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
                        {
                                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                                APP_ERROR_CHECK(ret_val);
                        }
                } while (ret_val == NRF_ERROR_BUSY);
        }
        if (p_data[data_len-1] == '\r')
        {
                while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
}



/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_c_evt)
{
        ret_code_t err_code;

        switch (p_ble_nus_c_evt->evt_type)
        {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
                NRF_LOG_DEBUG("NUS Service discovered on conn_handle 0x%x",
                              p_ble_nus_c_evt->conn_handle);

                err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_c_evt->conn_handle, &p_ble_nus_c_evt->handles);
                APP_ERROR_CHECK(err_code);


                NRF_LOG_DEBUG("Before enable the tx notification");
                NRF_LOG_HEXDUMP_DEBUG(p_ble_nus_c, sizeof(ble_nus_c_t));
                err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_DEBUG("Connected to device with Nordic UART Service.\n\n");
                break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
                ble_nus_chars_received_uart_print(p_ble_nus_c_evt->p_data, p_ble_nus_c_evt->data_len);
                break;

        case BLE_NUS_C_EVT_DISCONNECTED:
                NRF_LOG_DEBUG("Disconnected.");
                // scan_start();
                break;
        }
}

/**@brief Handles events coming from the LED Button central module.
 */
static void lbs_c_evt_handler(ble_lbs_c_t * p_lbs_c, ble_lbs_c_evt_t * p_lbs_c_evt)
{
        switch (p_lbs_c_evt->evt_type)
        {
        case BLE_LBS_C_EVT_DISCOVERY_COMPLETE:
        {
                ret_code_t err_code;

//                err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c,
//                                                     p_lbs_c_evt->conn_handle,
//                                                     &p_lbs_c_evt->params.peer_db);
                NRF_LOG_DEBUG("LED Button service discovered on conn_handle 0x%x.", p_lbs_c_evt->conn_handle);



                err_code = app_button_enable();
                APP_ERROR_CHECK(err_code);


                NRF_LOG_DEBUG("Enable the LBS button notification");
                NRF_LOG_HEXDUMP_INFO(p_lbs_c, sizeof(ble_lbs_c_t));
                // LED Button service discovered. Enable notification of Button.
                err_code = ble_lbs_c_button_notif_enable(p_lbs_c);
                APP_ERROR_CHECK(err_code);
        } break; // BLE_LBS_C_EVT_DISCOVERY_COMPLETE

        case BLE_LBS_C_EVT_BUTTON_NOTIFICATION:
        {
                NRF_LOG_DEBUG("Link 0x%x, Button state changed on peer to 0x%x",
                              p_lbs_c_evt->conn_handle,
                              p_lbs_c_evt->params.button.button_state);
                // NRF_LOG_INFO("Button state changed on peer to 0x%x.", p_lbs_c_evt->params.button.button_state);
                if (p_lbs_c_evt->params.button.button_state)
                {
                        bsp_board_led_on(LEDBUTTON_LED);
                }
                else
                {
                        bsp_board_led_off(LEDBUTTON_LED);
                }
        } break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION

        default:
                // No implementation needed.
                break;
        }
}

static void ble_its_c_evt_handler(ble_its_c_t *p_ble_its_c, ble_its_c_evt_t const *p_ble_its_evt)
{
        ret_code_t err_code;
        uint32_t receive_byte = 0;

        switch (p_ble_its_evt->evt_type)
        {
        case BLE_ITS_C_EVT_DISCOVERY_COMPLETE:
                NRF_LOG_DEBUG("ITS Service: Discovery complete.");
                err_code = ble_its_c_handles_assign(p_ble_its_c, p_ble_its_evt->conn_handle, &p_ble_its_evt->handles);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_DEBUG("ble_its_c_tx_notif_enable.");
                err_code = ble_its_c_tx_notif_enable(p_ble_its_c);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_DEBUG("ble_its_c_img_info_notif_enable.");
                err_code = ble_its_c_img_info_notif_enable(p_ble_its_c);
                APP_ERROR_CHECK(err_code);

                NRF_LOG_DEBUG("Connected to device with Nordic ITS Service.");
                break;

        case BLE_ITS_C_EVT_ITS_RX_EVT:
                NRF_LOG_DEBUG("BLE_ITS_C_EVT_ITS_RX_EVT");
                break;

        case BLE_ITS_C_EVT_ITS_TX_EVT:
                NRF_LOG_DEBUG("BLE_ITS_C_EVT_ITS_TX_EVT %04d", receive_byte);
                break;

        case BLE_ITS_C_EVT_ITS_IMG_INFO_EVT:
                NRF_LOG_DEBUG("BLE_ITS_C_EVT_ITS_IMG_INFO_EVT %04d", receive_byte);
                break;

        case BLE_ITS_C_EVT_DISCONNECTED:
                NRF_LOG_DEBUG("Disconnected.");
                //scan_start();
                break;
        }
}

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
        ret_code_t err_code;
        ble_nus_c_init_t init;

        init.evt_handler = ble_nus_c_evt_handler;

        for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
        {
                err_code = ble_nus_c_init(&m_ble_nus_c[i], &init);
                APP_ERROR_CHECK(err_code);
        }
}

/**@brief LED Button client initialization.
 */
static void lbs_c_init(void)
{
        ret_code_t err_code;
        ble_lbs_c_init_t lbs_c_init_obj;

        lbs_c_init_obj.evt_handler = lbs_c_evt_handler;

        for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
        {
                err_code = ble_lbs_c_init(&m_ble_lbs_c[i], &lbs_c_init_obj);
                APP_ERROR_CHECK(err_code);
        }
        // err_code = ble_lbs_c_init(&m_ble_lbs_c, &lbs_c_init_obj);
        // APP_ERROR_CHECK(err_code);
}


static void its_c_init(void)
{
        ret_code_t err_code;
        ble_its_c_init_t its_init;

        its_init.evt_handler = ble_its_c_evt_handler;
        for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
        {
                err_code = ble_its_c_init(&m_ble_its_c[i], &its_init);
                APP_ERROR_CHECK(err_code);
        }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
        ret_code_t err_code;

        err_code = nrf_sdh_enable_request();
        APP_ERROR_CHECK(err_code);

        // Configure the BLE stack using the default settings.
        // Fetch the start address of the application RAM.
        uint32_t ram_start = 0;
        err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
        APP_ERROR_CHECK(err_code);


        ble_cfg_t ble_cfg;
        // Configure the GATTS attribute table.
        memset(&ble_cfg, 0x00, sizeof(ble_cfg));
        ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = NRF_SDH_BLE_PERIPHERAL_LINK_COUNT;
        ble_cfg.gap_cfg.role_count_cfg.central_role_count = NRF_SDH_BLE_CENTRAL_LINK_COUNT;
        //ble_cfg.gap_cfg.role_count_cfg.qos_channel_survey_role_available = true; /* Enable channel survey role */

        err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, &ram_start);
        if (err_code != NRF_SUCCESS)
        {
                NRF_LOG_ERROR("sd_ble_cfg_set() returned %s when attempting to set BLE_GAP_CFG_ROLE_COUNT.",
                              nrf_strerror_get(err_code));
        }

        // Enable BLE stack.
        err_code = nrf_sdh_ble_enable(&ram_start);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
        APP_ERROR_CHECK(err_code);

        // Register a handler for BLE events.
        NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
        ret_code_t err_code;

        switch (pin_no)
        {
        case LEDBUTTON_BUTTON:
                NRF_LOG_DEBUG("Send button state change.");
                err_code = ble_lbs_on_button_change(m_conn_handle, &m_lbs, button_action);
                if (err_code != NRF_SUCCESS &&
                    err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                    err_code != NRF_ERROR_INVALID_STATE &&
                    err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                {
                        APP_ERROR_CHECK(err_code);
                }
                break;
        case ADV_BUTTON:
                NRF_LOG_INFO("Start advertising!");
                advertising_start();
                break;

        case SCAN_BUTTON:
                NRF_LOG_INFO("Start Scanning!!");
                scan_start();
                break;

        case PRINT_BUTTON:
                NRF_LOG_INFO("Show connection summary!");
                print_connection_status();
                break;


        default:
                APP_ERROR_HANDLER(pin_no);
                break;
        }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
        ret_code_t err_code;

        //The array must be static because a pointer to it will be saved in the button handler module.
        static app_button_cfg_t buttons[] =
        {
                {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler},
                {ADV_BUTTON, false, BUTTON_PULL, button_event_handler},
                {SCAN_BUTTON, false, BUTTON_PULL, button_event_handler},
                {PRINT_BUTTON, false, BUTTON_PULL, button_event_handler},
        };

        err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                                   BUTTON_DETECTION_DELAY);
        APP_ERROR_CHECK(err_code);
}


static void log_init(void)
{
        ret_code_t err_code = NRF_LOG_INIT(NULL);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
        ret_code_t err_code;
        err_code = nrf_pwr_mgmt_init();
        APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
        app_sched_execute();
        while(NRF_LOG_PROCESS());
        //UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
        nrf_pwr_mgmt_run();
}


/**@brief Function for application main entry.
 */
int main(void)
{

        bool erase_bonds;
        ret_code_t err_code;

        /* enable instruction cache */
        NRF_NVMC->ICACHECNF = (NVMC_ICACHECNF_CACHEEN_Enabled << NVMC_ICACHECNF_CACHEEN_Pos) +
                              (NVMC_ICACHECNF_CACHEPROFEN_Disabled << NVMC_ICACHECNF_CACHEPROFEN_Pos);


        // Initialize.
        log_init();
        uart_init();
        leds_init();
        timers_init();
        buttons_init();
        power_management_init();
        ble_stack_init();
        scheduler_init();
        scan_init();
        db_discovery_init();
        gap_params_init();
        gatt_init();
        services_init();
        advertising_init();
        conn_params_init();
        ble_conn_state_init();
        lbs_c_init();
        nus_c_init();
        its_c_init();

        // Start execution.
        NRF_LOG_INFO("Multi-Role Example Start!!");
        scan_start();
        advertising_start();


        // Enter main loop.
        for (;;)
        {
                idle_state_handle();
        }
}


/**
 * @}
 */
