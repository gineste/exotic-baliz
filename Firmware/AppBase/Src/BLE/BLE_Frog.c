/* 
 *    ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 *   (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *    ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \     
 *   (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2017 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO WARRANTY 
 * of ANY KIND is provided. This heading must NOT be removed from the file.
 *
 * Date:          9/10/2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   BLE interface of SuperFrog
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "BLE_Uart.h"

#include "fds.h"
#include "peer_manager.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"

#if (BLE_DFU_ENABLED == 1)
   #include "ble_dfu.h"
   #include "nrf_pwr_mgmt.h"
#endif

/* nRF LOG (unused) */
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "BoardConfig.h"
#include "GlobalDefs.h"
#include "sdk_config.h"

#include "BLE_Frog.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2     /**< Reply when unsupported features are requested. */

#define DEVICE_NAME                     "SF_HardwareTest"                         /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                    /**< Manufacturer. Will be passed to Device Information Service. */

   
#define APP_ADV_FAST_INTERVAL           400                                      /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_FAST_TIMEOUT_IN_SECONDS FAST_ADV_TIMEOUT_BLE                     /**< The advertising timeout in units of seconds. */
#define APP_ADV_SLOW_INTERVAL           1600                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_SLOW_TIMEOUT_IN_SECONDS SLOW_ADV_TIMEOUT_BLE                     /**< The advertising timeout in units of seconds. */
   
#define APP_BLE_OBSERVER_PRIO           1                                        /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                        /**< A tag identifying the SoftDevice BLE configuration. */
   
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(12, UNIT_1_25_MS)          /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(48, UNIT_1_25_MS)          /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                        /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)          /**< Connection supervisory timeout (4 seconds). */
   
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                    /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(3000)                    /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3/*16*/                                       /**< Number of attempts before giving up the connection parameter negotiation. */
   
#define SEC_PARAM_BOND                  1                                        /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                        /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                        /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                        /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                     /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                        /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                        /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                       /**< Maximum encryption key size. */

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/

/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static void vOnAdvEvt(ble_adv_evt_t p_eBleAdvEvt);
static void vPeerManagerEvtHandler(pm_evt_t const * p_cpsEvt);
static void vOnConnParamsEvt(ble_conn_params_evt_t * p_psEvt);
static void vConnParamsErrorHandler(uint32_t p_u32NrfError);
static void vBleEvtHandler(ble_evt_t const * p_ble_evt, void * p_context);
static void vPeerManagerInit(void);
static void vAdvertisingInit(void);
static void vConnParamsInit(void);
static void vBleStackInit(void);
static void vGapParamsInit(void);
static void vGattInit(void);
static void vServicesInit(void);
static void vAdvertisingStart(bool p_bEraseBonds);
static void vDeleteBonds(void);

#if (BLE_DFU_ENABLED == 1)
   static void vOnBleDFUEvtHandler(ble_dfu_buttonless_evt_type_t p_sDFUEvent);
   static bool bShutdownHandler(nrf_pwr_mgmt_evt_t p_sEvent);
#endif
/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/ 
/* YOUR_JOB: Declare all services structure your application is using
 *  BLE_XYZ_DEF(m_xyz);
 */
NRF_BLE_GATT_DEF(g_GattInstance);                                             /**< GATT module instance. (Generic Attribute Service) */
BLE_ADVERTISING_DEF(g_AdvertisingInstance);                                   /**< Advertising module instance. */
NRF_PWR_MGMT_HANDLER_REGISTER(bShutdownHandler, 0);                       /**< Register application shutdown handler with priority 0. */

static e_BleFrog_Event_t g_LastBleEvent = BLE_EVENT_INIT;
static uint16_t g_u16ConnHandle = BLE_CONN_HANDLE_INVALID;                    /**< Handle of the current connection. */

// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] =                                             /**< Universally unique service identifiers. */
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
/**@brief  Initialise the SuperFrog BLE module.
 *
 * @return None.
 */
void vBLE_Frog_Init(void)
{  
   vBleStackInit();
   vGapParamsInit();
   vGattInit();
   vAdvertisingInit();
   vServicesInit();
   vConnParamsInit();
   vPeerManagerInit();

   // Start execution.
   //application_timers_start();
   //vBLE_Frog_AdvertisingStart();
   //vAdvertisingStart(erase_bonds);   
}

/**@brief  Start Advertising BLE.
 * @return None.
 */
void vBLE_Frog_AdvertisingStart(void)
{
   uint32_t l_u32ErrCode = ble_advertising_start(&g_AdvertisingInstance, BLE_ADV_MODE_FAST);
   APP_ERROR_CHECK(l_u32ErrCode);
}

/**@brief  Stop Advertising BLE.
 * @return None.
 */
void vBLE_Frog_AdvertisingStop(void)
{
   APP_ERROR_CHECK(sd_ble_gap_adv_stop());
}


#include "app_timer.h"
/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
void timers_init(void)
{
    // Initialize timer module.
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
       uint32_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); */
}

/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
}
*/




/**@brief Function for starting timers.
 */
void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       uint32_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */

}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
void vBLE_Frog_SleepModeEnter(void)
{
   PRINT_ERROR("%s","Power Off");
   
   //    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
   //    APP_ERROR_CHECK(err_code);

   //    // Prepare wakeup buttons.
   //    err_code = bsp_btn_ble_sleep_mode_prepare();
   //    APP_ERROR_CHECK(err_code);

   // Go to system-off mode (this function will not return; wakeup will cause a reset).
   APP_ERROR_CHECK(sd_power_system_off());
}

e_BleFrog_Event_t eBleFrog_EventGet(void)
{
   return g_LastBleEvent;
}

/**@brief Function for initializing the nrf log module.
 */
void log_init(void)
{
   uint32_t err_code = NRF_LOG_INIT(NULL);
   APP_ERROR_CHECK(err_code);

   NRF_LOG_DEFAULT_BACKENDS_INIT();
}



/****************************************************************************************
 * Private functions
 ****************************************************************************************/

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void vOnAdvEvt(ble_adv_evt_t p_eBleAdvEvt)
{
   switch (p_eBleAdvEvt)
   {
      case BLE_ADV_EVT_FAST:
         NRF_LOG_INFO("Fast advertising.");
      PRINT_INFO("%s","Ble : ");
      PRINT_STATE("%s\n","Fast advertising");
         break;
      case BLE_ADV_EVT_SLOW:
         NRF_LOG_INFO("Slow advertising.");
      PRINT_INFO("%s","Ble : ");
      PRINT_STATE("%s\n","Slow advertising");
         break;

      case BLE_ADV_EVT_IDLE:
      PRINT_INFO("%s","Ble : ");
      PRINT_WARNING("%s","Advertise OFF");
//         vBLE_Frog_SleepModeEnter();
         break;       

      default:
      break;
   }
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void vPeerManagerEvtHandler(pm_evt_t const * p_cpsEvt)
{
   uint32_t l_u32ErrCode;

   switch (p_cpsEvt->evt_id)
   {
      case PM_EVT_BONDED_PEER_CONNECTED:
      {
         NRF_LOG_INFO("Connected to a previously bonded device.");
      } break;

      case PM_EVT_CONN_SEC_SUCCEEDED:
      {
         NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                ble_conn_state_role(p_cpsEvt->conn_handle),
                p_cpsEvt->conn_handle,
                p_cpsEvt->params.conn_sec_succeeded.procedure);
      } break;

      case PM_EVT_CONN_SEC_FAILED:
      {
         /* Often, when securing fails, it shouldn't be restarted, for security reasons.
         * Other times, it can be restarted directly.
         * Sometimes it can be restarted, but only after changing some Security Parameters.
         * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
         * Sometimes it is impossible, to secure the link, or the peer device does not support it.
         * How to handle this error is highly application dependent. */
      } break;

      case PM_EVT_CONN_SEC_CONFIG_REQ:
      {
         // Reject pairing request from an already bonded peer.
         pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
         pm_conn_sec_config_reply(p_cpsEvt->conn_handle, &conn_sec_config);
      } break;

      case PM_EVT_STORAGE_FULL:
      {
         // Run garbage collection on the flash.
         l_u32ErrCode = fds_gc();
         if (l_u32ErrCode == FDS_ERR_BUSY || l_u32ErrCode == FDS_ERR_NO_SPACE_IN_QUEUES)
         {
            // Retry.
         }
         else
         {
            APP_ERROR_CHECK(l_u32ErrCode);
         }
      } break;

      case PM_EVT_PEERS_DELETE_SUCCEEDED:
      {
         vAdvertisingStart(false);
      } break;

      case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
      {
         // The local database has likely changed, send service changed indications.
         pm_local_database_has_changed();
      } break;

      case PM_EVT_PEER_DATA_UPDATE_FAILED:
      {
         // Assert.
         APP_ERROR_CHECK(p_cpsEvt->params.peer_data_update_failed.error);
      } break;

      case PM_EVT_PEER_DELETE_FAILED:
      {
         // Assert.
         APP_ERROR_CHECK(p_cpsEvt->params.peer_delete_failed.error);
      } break;

      case PM_EVT_PEERS_DELETE_FAILED:
      {
         // Assert.
         APP_ERROR_CHECK(p_cpsEvt->params.peers_delete_failed_evt.error);
      } break;

      case PM_EVT_ERROR_UNEXPECTED:
      {
         // Assert.
         APP_ERROR_CHECK(p_cpsEvt->params.error_unexpected.error);
      } break;

      case PM_EVT_CONN_SEC_START:
      case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
      case PM_EVT_PEER_DELETE_SUCCEEDED:
      case PM_EVT_LOCAL_DB_CACHE_APPLIED:
      case PM_EVT_SERVICE_CHANGED_IND_SENT:
      case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
      default:
      break;
   }
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_psEvt  Event received from the Connection Parameters Module.
 */
static void vOnConnParamsEvt(ble_conn_params_evt_t * p_psEvt)
{
   uint32_t l_u32ErrCode;

   if (p_psEvt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
   {
      l_u32ErrCode = sd_ble_gap_disconnect(g_u16ConnHandle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
      APP_ERROR_CHECK(l_u32ErrCode);
   }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void vConnParamsErrorHandler(uint32_t p_u32NrfError)
{
   APP_ERROR_HANDLER(p_u32NrfError);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void vBleEvtHandler(ble_evt_t const * p_ble_evt, void * p_context)
{
   uint32_t l_u32ErrCode = NRF_SUCCESS;

   switch (p_ble_evt->header.evt_id)
   {
      case BLE_GAP_EVT_DISCONNECTED:
         NRF_LOG_INFO("Disconnected.");
         g_LastBleEvent = BLE_EVENT_DISCONNECTED;
         break;

      case BLE_GAP_EVT_CONNECTED:
         NRF_LOG_INFO("Connected.");
         g_LastBleEvent = BLE_EVENT_CONNECTED;
         g_u16ConnHandle = p_ble_evt->evt.gap_evt.conn_handle;
         break;

   #if defined(S132)
      case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
      {
         NRF_LOG_DEBUG("PHY update request.");
         ble_gap_phys_t const phys = {
          .rx_phys = BLE_GAP_PHY_AUTO,
          .tx_phys = BLE_GAP_PHY_AUTO,
         };
         l_u32ErrCode = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
         APP_ERROR_CHECK(l_u32ErrCode);
      }
      break;
   #endif

      case BLE_GATTC_EVT_TIMEOUT:
         // Disconnect on GATT Client timeout event.
         NRF_LOG_DEBUG("GATT Client Timeout.");
         l_u32ErrCode = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
         APP_ERROR_CHECK(l_u32ErrCode);
      break;

      case BLE_GATTS_EVT_TIMEOUT:
         // Disconnect on GATT Server timeout event.
         NRF_LOG_DEBUG("GATT Server Timeout.");
         l_u32ErrCode = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
         APP_ERROR_CHECK(l_u32ErrCode);
      break;

      case BLE_EVT_USER_MEM_REQUEST:
         l_u32ErrCode = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
         APP_ERROR_CHECK(l_u32ErrCode);
      break;

      case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
      {
      ble_gatts_evt_rw_authorize_request_t  req;
      ble_gatts_rw_authorize_reply_params_t auth_reply;

      req = p_ble_evt->evt.gatts_evt.params.authorize_request;

         if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
         {
            if((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
               (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
               (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
            {
               if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
               {
                  auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
               }
               else
               {
                  auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
               }
               auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
               l_u32ErrCode = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle, &auth_reply);
               APP_ERROR_CHECK(l_u32ErrCode);
            }
         }
      } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

      default:
      // No implementation needed.
      break;
   }
}

/**@brief Function for the Peer Manager initialization.
 */
static void vPeerManagerInit(void)
{
   uint32_t l_u32ErrCode;
   ble_gap_sec_params_t l_sSecParam;

   l_u32ErrCode = pm_init();
   APP_ERROR_CHECK(l_u32ErrCode);

   memset(&l_sSecParam, 0, sizeof(ble_gap_sec_params_t));

   // Security parameters to be used for all security procedures.
   l_sSecParam.bond           = SEC_PARAM_BOND;
   l_sSecParam.mitm           = SEC_PARAM_MITM;
   l_sSecParam.lesc           = SEC_PARAM_LESC;
   l_sSecParam.keypress       = SEC_PARAM_KEYPRESS;
   l_sSecParam.io_caps        = SEC_PARAM_IO_CAPABILITIES;
   l_sSecParam.oob            = SEC_PARAM_OOB;
   l_sSecParam.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
   l_sSecParam.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
   l_sSecParam.kdist_own.enc  = 1;
   l_sSecParam.kdist_own.id   = 1;
   l_sSecParam.kdist_peer.enc = 1;
   l_sSecParam.kdist_peer.id  = 1;

   l_u32ErrCode = pm_sec_params_set(&l_sSecParam);
   APP_ERROR_CHECK(l_u32ErrCode);

   l_u32ErrCode = pm_register(vPeerManagerEvtHandler);
   APP_ERROR_CHECK(l_u32ErrCode);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void vAdvertisingInit(void)
{
   uint32_t l_u32ErrCode;
   ble_advertising_init_t l_sInit;

   memset(&l_sInit, 0, sizeof(l_sInit));

   l_sInit.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
   l_sInit.advdata.include_appearance      = true;
   l_sInit.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
   l_sInit.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
   l_sInit.advdata.uuids_complete.p_uuids  = m_adv_uuids;
#if (ENABLE_BLE == 1)
   l_sInit.config.ble_adv_fast_enabled  = true;
   l_sInit.config.ble_adv_slow_enabled  = true;   
   l_sInit.config.ble_adv_fast_interval = APP_ADV_FAST_INTERVAL;
   l_sInit.config.ble_adv_fast_timeout  = APP_ADV_FAST_TIMEOUT_IN_SECONDS;
   l_sInit.config.ble_adv_slow_interval = APP_ADV_SLOW_INTERVAL;
   l_sInit.config.ble_adv_slow_timeout  = APP_ADV_SLOW_TIMEOUT_IN_SECONDS;
   
#else   
   l_sInit.config.ble_adv_fast_enabled  = false;
   l_sInit.config.ble_adv_slow_enabled  = false;
   l_sInit.config.ble_adv_fast_interval = APP_ADV_FAST_INTERVAL;
   l_sInit.config.ble_adv_fast_timeout  = 0;
   l_sInit.config.ble_adv_slow_interval = APP_ADV_SLOW_INTERVAL;
   l_sInit.config.ble_adv_slow_timeout  = 0;
#endif   // (ENABLE_BLE == 1)

   l_sInit.evt_handler = vOnAdvEvt;
   l_u32ErrCode = ble_advertising_init(&g_AdvertisingInstance, &l_sInit);
   APP_ERROR_CHECK(l_u32ErrCode);

   ble_advertising_conn_cfg_tag_set(&g_AdvertisingInstance, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void vConnParamsInit(void)
{
   uint32_t l_u32ErrCode;
   ble_conn_params_init_t l_sCpInit;

   memset(&l_sCpInit, 0, sizeof(l_sCpInit));

   l_sCpInit.p_conn_params                  = NULL;
   l_sCpInit.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
   l_sCpInit.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
   l_sCpInit.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
   l_sCpInit.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
   l_sCpInit.disconnect_on_fail             = false;
   l_sCpInit.evt_handler                    = vOnConnParamsEvt;
   l_sCpInit.error_handler                  = vConnParamsErrorHandler;

   l_u32ErrCode = ble_conn_params_init(&l_sCpInit);
   APP_ERROR_CHECK(l_u32ErrCode);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void vBleStackInit(void)
{
   uint32_t l_u32ErrCode;
   uint32_t ram_start = 0;

   l_u32ErrCode = nrf_sdh_enable_request();
   APP_ERROR_CHECK(l_u32ErrCode);

   // Configure the BLE stack using the default settings.
   // Fetch the start address of the application RAM.
   l_u32ErrCode = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
   APP_ERROR_CHECK(l_u32ErrCode);
   // Enable BLE stack.
   l_u32ErrCode = nrf_sdh_ble_enable(&ram_start);
   APP_ERROR_CHECK(l_u32ErrCode);
   
   // Register a handler for BLE events.
   NRF_SDH_BLE_OBSERVER(g_BLE_Frog_Observer, APP_BLE_OBSERVER_PRIO, vBleEvtHandler, NULL);
   
   NRF_SDH_BLE_OBSERVER(g_BLE_Uart_Observer, APP_BLE_OBSERVER_PRIO, vBLE_Uart_onBleEvt, NULL);
   
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void vGapParamsInit(void)
{
   uint32_t l_u32ErrCode;
   ble_gap_conn_params_t l_sGapConnParams;
   ble_gap_conn_sec_mode_t l_sSecMode;

   BLE_GAP_CONN_SEC_MODE_SET_OPEN(&l_sSecMode);

   l_u32ErrCode = sd_ble_gap_device_name_set(&l_sSecMode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
   APP_ERROR_CHECK(l_u32ErrCode);

   /* YOUR_JOB: Use an appearance value matching the application's use case.
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
    APP_ERROR_CHECK(err_code); */

   memset(&l_sGapConnParams, 0, sizeof(l_sGapConnParams));

   l_sGapConnParams.min_conn_interval = MIN_CONN_INTERVAL;
   l_sGapConnParams.max_conn_interval = MAX_CONN_INTERVAL;
   l_sGapConnParams.slave_latency     = SLAVE_LATENCY;
   l_sGapConnParams.conn_sup_timeout  = CONN_SUP_TIMEOUT;

   l_u32ErrCode = sd_ble_gap_ppcp_set(&l_sGapConnParams);
   APP_ERROR_CHECK(l_u32ErrCode);
}


/**@brief Function for initializing the GATT module.
 */
static void vGattInit(void)
{
   uint32_t l_u32ErrCode = nrf_ble_gatt_init(&g_GattInstance, NULL);
   APP_ERROR_CHECK(l_u32ErrCode);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void vServicesInit(void)
{
   /* YOUR_JOB: Add code to initialize the services used by the application.
   uint32_t                         err_code;
   ble_xxs_init_t                     xxs_init;
   ble_yys_init_t                     yys_init;

   // Initialize XXX Service.
   memset(&xxs_init, 0, sizeof(xxs_init));

   xxs_init.evt_handler                = NULL;
   xxs_init.is_xxx_notify_supported    = true;
   xxs_init.ble_xx_initial_value.level = 100;

   err_code = ble_bas_init(&m_xxs, &xxs_init);
   APP_ERROR_CHECK(err_code);

   // Initialize YYY Service.
   memset(&yys_init, 0, sizeof(yys_init));
   yys_init.evt_handler                  = on_yys_evt;
   yys_init.ble_yy_initial_value.counter = 0;

   err_code = ble_yy_service_init(&yys_init, &yy_init);
   APP_ERROR_CHECK(err_code);
   */
   uint32_t l_u32ErrCode;
//   ble_nus_init_t l_sNUSInit;
//   
//   memset(&l_sNUSInit, 0, sizeof(l_sNUSInit));

//   l_sNUSInit.data_handler = vNUSDataHandler;
//   
//   l_u32ErrCode = ble_nus_init(&g_NUSInstance, &l_sNUSInit);
//   APP_ERROR_CHECK(l_u32ErrCode);
   vBLE_Uart_ServiceInit();
#if (BLE_DFU_ENABLED == 1)
   ble_dfu_buttonless_init_t l_sSecureDFUInit = {
      .evt_handler = vOnBleDFUEvtHandler
   };

    // Initialize the async SVCI interface to bootloader.
    l_u32ErrCode = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(l_u32ErrCode);

    l_u32ErrCode = ble_dfu_buttonless_init(&l_sSecureDFUInit);
    APP_ERROR_CHECK(l_u32ErrCode);
#endif
   
}



/**@brief Function for starting advertising.
 */
static void vAdvertisingStart(bool p_bEraseBonds)
{
   uint32_t l_u32ErrCode;
   
   if (p_bEraseBonds == true)
   {
      vDeleteBonds();
      // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED evetnt
   }
   else
   {
      l_u32ErrCode = ble_advertising_start(&g_AdvertisingInstance, BLE_ADV_MODE_FAST);
      APP_ERROR_CHECK(l_u32ErrCode);
   }
}

/**@brief Clear bond information from persistent storage.
 */
static void vDeleteBonds(void)
{
   uint32_t l_u32ErrCode;

   NRF_LOG_INFO("Erase bonds!");

   l_u32ErrCode = pm_peers_delete();
   APP_ERROR_CHECK(l_u32ErrCode);
}

#if (BLE_DFU_ENABLED == 1)
// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void vOnBleDFUEvtHandler(ble_dfu_buttonless_evt_type_t p_sDFUEvent)
{
    switch(p_sDFUEvent)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
            // YOUR_JOB: Disconnect all bonded devices that currently are connected.
            //           This is required to receive a service changed indication
            //           on bootup after a successful (or aborted) Device Firmware Update.
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            break;
    }
}

/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool bShutdownHandler(nrf_pwr_mgmt_evt_t p_sEvent)
{
   uint32_t l_u32ErrCode;
   
   switch (p_sEvent)
   {
   #if (BLE_DFU_ENABLED == 1)
      case NRF_PWR_MGMT_EVT_PREPARE_DFU:
         // YOUR_JOB: Get ready to reset into DFU mode
         //
         // If you aren't finished with any ongoing tasks, return "false" to
         // signal to the system that reset is impossible at this stage.
         //
         // Here is an example using a variable to delay resetting the device.
         //
         // if (!m_ready_for_reset)
         // {
         //      return false;
         // }
         // else
         //{
         //
         //    // Device ready to enter
         //    uint32_t err_code;
             l_u32ErrCode = nrf_sdh_disable_request();
             APP_ERROR_CHECK(l_u32ErrCode);
         //    err_code = app_timer_stop_all();
         //    APP_ERROR_CHECK(err_code);
         //}
         break;
   #endif
      case NRF_PWR_MGMT_EVT_PREPARE_SYSOFF:
      case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
      case NRF_PWR_MGMT_EVT_PREPARE_RESET:
      default:
         // YOUR_JOB: Implement any of the other events available from the power management module:
         //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
         //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
         //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
         break;
   }

   return true;
}

#endif /* (BLE_DFU_ENABLE == 1) */
/****************************************************************************************
 * End Of File
 ****************************************************************************************/
 

