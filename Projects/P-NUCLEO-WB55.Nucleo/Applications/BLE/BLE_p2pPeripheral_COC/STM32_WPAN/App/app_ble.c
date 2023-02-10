/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_ble.c
  * @author  MCD Application Team
  * @brief   BLE Application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "app_common.h"

#include "dbg_trace.h"
#include "ble.h"
#include "tl.h"
#include "app_ble.h"

#include "stm32_seq.h"
#include "shci.h"
#include "stm32_lpm.h"
#include "otp.h"

#include "p2p_peripheral_app.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/**
 * security parameters structure
 */
typedef struct _tSecurityParams
{
  /**
   * IO capability of the device
   */
  uint8_t ioCapability;

  /**
   * Authentication requirement of the device
   * Man In the Middle protection required?
   */
  uint8_t mitm_mode;

  /**
   * bonding mode of the device
   */
  uint8_t bonding_mode;

  /**
   * this variable indicates whether to use a fixed pin
   * during the pairing process or a passkey has to be
   * requested to the application during the pairing process
   * 0 implies use fixed pin and 1 implies request for passkey
   */
  uint8_t Use_Fixed_Pin;

  /**
   * minimum encryption key size requirement
   */
  uint8_t encryptionKeySizeMin;

  /**
   * maximum encryption key size requirement
   */
  uint8_t encryptionKeySizeMax;

  /**
   * fixed pin to be used in the pairing process if
   * Use_Fixed_Pin is set to 1
   */
  uint32_t Fixed_Pin;

  /**
   * this flag indicates whether the host has to initiate
   * the security, wait for pairing or does not have any security
   * requirements.
   * 0x00 : no security required
   * 0x01 : host should initiate security by sending the slave security
   *        request command
   * 0x02 : host need not send the clave security request but it
   * has to wait for paiirng to complete before doing any other
   * processing
   */
  uint8_t initiateSecurity;
  /* USER CODE BEGIN tSecurityParams*/

  /* USER CODE END tSecurityParams */
}tSecurityParams;

/**
 * global context
 * contains the variables common to all
 * services
 */
typedef struct _tBLEProfileGlobalContext
{
  /**
   * security requirements of the host
   */
  tSecurityParams bleSecurityParam;

  /**
   * gap service handle
   */
  uint16_t gapServiceHandle;

  /**
   * device name characteristic handle
   */
  uint16_t devNameCharHandle;

  /**
   * appearance characteristic handle
   */
  uint16_t appearanceCharHandle;

  /**
   * connection handle of the current active connection
   * When not in connection, the handle is set to 0xFFFF
   */
  uint16_t connectionHandle;

  /**
   * length of the UUID list to be used while advertising
   */
  uint8_t advtServUUIDlen;

  /**
   * the UUID list to be used while advertising
   */
  uint8_t advtServUUID[100];
  /* USER CODE BEGIN BleGlobalContext_t*/

  /* USER CODE END BleGlobalContext_t */
}BleGlobalContext_t;

typedef struct
{
  BleGlobalContext_t BleApplicationContext_legacy;
  APP_BLE_ConnStatus_t Device_Connection_Status;

  /**
   * ID of the Advertising Timeout
   */
  uint8_t Advertising_mgr_timer_Id;

  uint8_t SwitchOffGPIO_timer_Id;
  /* USER CODE BEGIN PTD_1*/
  uint8_t connIntervalFlag;
  /* USER CODE END PTD_1 */
}BleApplicationContext_t;

/* USER CODE BEGIN PTD */
CoC_App_Context_t CoC_App_Context;
/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define APPBLE_GAP_DEVICE_NAME_LENGTH 7
#define FAST_ADV_TIMEOUT               (30*1000*1000/CFG_TS_TICK_VAL) /**< 30s */
#define INITIAL_ADV_TIMEOUT            (60*1000*1000/CFG_TS_TICK_VAL) /**< 60s */

#define BD_ADDR_SIZE_LOCAL    6

/* USER CODE BEGIN PD */
#define LED_ON_TIMEOUT                 (0.005*1000*1000/CFG_TS_TICK_VAL) /**< 5ms */
#define MAX_TX_OCTETS                   251
#define MAX_TX_TIME                     2120
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CONN_P1_400                                                (CONN_P(400))
#define CONN_P2_400                                                (CONN_P(400))
#define CONN_P45                (CONN_P(45))

#define CONN_P1         (CONN_P(7.5)) 
#define CONN_P2         (CONN_P(7.5)) 
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

static const uint8_t a_MBdAddr[BD_ADDR_SIZE_LOCAL] =
{
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000000000FF)),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00000000FF00) >> 8),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x000000FF0000) >> 16),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000FF000000) >> 24),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00FF00000000) >> 32),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0xFF0000000000) >> 40)
};

static uint8_t a_BdAddrUdn[BD_ADDR_SIZE_LOCAL];

/**
 *   Identity root key used to derive LTK and CSRK
 */
static const uint8_t a_BLE_CfgIrValue[16] = CFG_BLE_IRK;

/**
 * Encryption root key used to derive LTK and CSRK
 */
static const uint8_t a_BLE_CfgErValue[16] = CFG_BLE_ERK;

/**
 * These are the two tags used to manage a power failure during OTA
 * The MagicKeywordAdress shall be mapped @0x140 from start of the binary image
 * The MagicKeywordvalue is checked in the ble_ota application
 */
PLACE_IN_SECTION("TAG_OTA_END") const uint32_t MagicKeywordValue = 0x94448A29 ;
PLACE_IN_SECTION("TAG_OTA_START") const uint32_t MagicKeywordAddress = (uint32_t)&MagicKeywordValue;

PLACE_IN_SECTION("BLE_APP_CONTEXT") static BleApplicationContext_t BleApplicationContext;
PLACE_IN_SECTION("BLE_APP_CONTEXT") static uint16_t AdvIntervalMin, AdvIntervalMax;

BleCoCContext_t BleCoCContext;

COC_APP_ConnHandle_Not_evt_t HandleNotification;
uint8_t TimerL2capDataThroughputWrite_Id;
static uint16_t packet_lost;
uint32_t N=0;

/**
 * Advertising Data
 */
#if (BLE_COC != 0)
static const char a_LocalName[] = {AD_TYPE_COMPLETE_LOCAL_NAME , 'B', 'L', 'E', '_', 'C', 'O', 'C'};
uint8_t a_ManufData[14] = {sizeof(a_ManufData)-1,
                           AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
                           0x01,                               /*SKD version */
                           CFG_DEV_ID_BLE_COC,                 /* STM32WB - BLE ADV*/
                           0x00,                               /* GROUP A Feature */
                           0x00,                               /* GROUP A Feature */
                           0x00,                               /* GROUP B Feature */
                           0x00,                               /* GROUP B Feature */
                           0x00,                               /* BLE MAC start -MSB */
                           0x00,
                           0x00,
                           0x00,
                           0x00,
                           0x00,                               /* BLE MAC stop */
                          };
#endif /* BLE_COC != 0 */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void BLE_UserEvtRx(void *p_Payload);
static void BLE_StatusNot(HCI_TL_CmdStatus_t Status);
static void Ble_Tl_Init(void);
static void Ble_Hci_Gap_Gatt_Init(void);
static const uint8_t* BleGetBdAddress(void);
static void Adv_Request(APP_BLE_ConnStatus_t NewStatus);
static void Adv_Cancel(void);
static void Adv_Cancel_Req(void);
static void Switch_OFF_GPIO(void);
static void L2cap_CoC_Throughput_proc();

static void BLE_App_Toggle_Connection_Interval(void);
static void Connection_Interval_Update_Req(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Functions Definition ------------------------------------------------------*/
void APP_BLE_Init(void)
{
  SHCI_CmdStatus_t status;
#if (RADIO_ACTIVITY_EVENT != 0)
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
#endif /* RADIO_ACTIVITY_EVENT != 0 */
  /* USER CODE BEGIN APP_BLE_Init_1 */

  /* USER CODE END APP_BLE_Init_1 */
  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
  {
    {{0,0,0}},                          /**< Header unused */
    {0,                                 /** pBleBufferAddress not used */
     0,                                 /** BleBufferSize not used */
     CFG_BLE_NUM_GATT_ATTRIBUTES,
     CFG_BLE_NUM_GATT_SERVICES,
     CFG_BLE_ATT_VALUE_ARRAY_SIZE,
     CFG_BLE_NUM_LINK,
     CFG_BLE_DATA_LENGTH_EXTENSION,
     CFG_BLE_PREPARE_WRITE_LIST_SIZE,
     CFG_BLE_MBLOCK_COUNT,
     CFG_BLE_MAX_ATT_MTU,
     CFG_BLE_SLAVE_SCA,
     CFG_BLE_MASTER_SCA,
     CFG_BLE_LS_SOURCE,
     CFG_BLE_MAX_CONN_EVENT_LENGTH,
     CFG_BLE_HSE_STARTUP_TIME,
     CFG_BLE_VITERBI_MODE,
     CFG_BLE_OPTIONS,
     0,
     CFG_BLE_MAX_COC_INITIATOR_NBR,
     CFG_BLE_MIN_TX_POWER,
     CFG_BLE_MAX_TX_POWER,
     CFG_BLE_RX_MODEL_CONFIG,
     CFG_BLE_MAX_ADV_SET_NBR,
     CFG_BLE_MAX_ADV_DATA_LEN,
     CFG_BLE_TX_PATH_COMPENS,
     CFG_BLE_RX_PATH_COMPENS,
     CFG_BLE_CORE_VERSION
    }
  };

  /**
   * Initialize Ble Transport Layer
   */
  Ble_Tl_Init();

  /**
   * Do not allow standby in the application
   */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

  /**
   * Register the hci transport layer to handle BLE User Asynchronous Events
   */
  UTIL_SEQ_RegTask(1<<CFG_TASK_HCI_ASYNCH_EVT_ID, UTIL_SEQ_RFU, hci_user_evt_proc);

  /**
   * Starts the BLE Stack on CPU2
   */
  status = SHCI_C2_BLE_Init(&ble_init_cmd_packet);
  if (status != SHCI_Success)
  {
    APP_DBG_MSG("  Fail   : SHCI_C2_BLE_Init command, result: 0x%02x\n\r", status);
    /* if you are here, maybe CPU2 doesn't contain STM32WB_Copro_Wireless_Binaries, see Release_Notes.html */
    Error_Handler();
  }
  else
  {
    APP_DBG_MSG("  Success: SHCI_C2_BLE_Init command\n\r");
  }

  /**
   * Initialization of HCI & GATT & GAP layer
   */
  Ble_Hci_Gap_Gatt_Init();

  /**
   * Initialization of the BLE Services
   */
  SVCCTL_Init();

  /**
   * Initialization of the BLE App Context
   */
  BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
  BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0xFFFF;

  /**
   * From here, all initialization are BLE application specific
   */
  UTIL_SEQ_RegTask(1<<CFG_TASK_ADV_CANCEL_ID, UTIL_SEQ_RFU, Adv_Cancel);
  UTIL_SEQ_RegTask(1<<CFG_TASK_CONN_UPDATE_REG_ID, UTIL_SEQ_RFU, Connection_Interval_Update_Req);


#if (RADIO_ACTIVITY_EVENT != 0)
  ret = aci_hal_set_radio_activity_mask(0x0006);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_set_radio_activity_mask command, result: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_set_radio_activity_mask command\n\r");
  }
#endif /* RADIO_ACTIVITY_EVENT != 0 */

#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
  index_con_int = 0;
  mutex = 1;
#endif /* L2CAP_REQUEST_NEW_CONN_PARAM != 0 */

  /**
   * Initialize COC peripheral Application
   */
  COC_APP_Init();

  /* USER CODE BEGIN APP_BLE_Init_3 */
  UTIL_SEQ_RegTask( 1<<CFG_TASK_TOGGLE_CONN_INT_ID, UTIL_SEQ_RFU, BLE_App_Toggle_Connection_Interval);
  /* USER CODE END APP_BLE_Init_3 */

  /**
   * Create timer to handle the Advertising Stop
   */
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.Advertising_mgr_timer_Id), hw_ts_SingleShot, Adv_Cancel_Req);
  /**
   * Create timer to handle the Led Switch OFF
   */
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.SwitchOffGPIO_timer_Id), hw_ts_SingleShot, Switch_OFF_GPIO);
  /**
    * Create timer for L2CAP Data Throughput process (write data)
    */
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(TimerL2capDataThroughputWrite_Id), hw_ts_SingleShot, L2cap_CoC_Throughput_proc);

  /**
   * Make device discoverable
   */
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = NULL;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 0;

  /* Initialize intervals for reconnexion without intervals update */
  AdvIntervalMin = CFG_FAST_CONN_ADV_INTERVAL_MIN;
  AdvIntervalMax = CFG_FAST_CONN_ADV_INTERVAL_MAX;

  /**
   * Start to Advertise to be connected by Central Client
   */
  Adv_Request(APP_BLE_FAST_ADV);

  /* USER CODE BEGIN APP_BLE_Init_2 */

  /* USER CODE END APP_BLE_Init_2 */

  return;
}

SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification(void *p_Pckt)
{
  hci_event_pckt    *p_event_pckt;
  evt_le_meta_event *p_meta_evt;
  evt_blecore_aci   *p_blecore_evt;
  uint8_t           Tx_phy, Rx_phy;
  tBleStatus        ret = BLE_STATUS_INVALID_PARAMS;
  hci_le_connection_complete_event_rp0        *p_connection_complete_event;
  hci_disconnection_complete_event_rp0        *p_disconnection_complete_event;
  hci_le_phy_update_complete_event_rp0        *p_evt_le_phy_update_complete;
  aci_l2cap_coc_rx_data_event_rp0             *coc_rx_data_event;
  aci_att_exchange_mtu_resp_event_rp0 * exchange_mtu_resp;
  uint8_t CRC_Result;
  uint8_t CRC_Received;
    
#if (CFG_DEBUG_APP_TRACE != 0)
  hci_le_connection_update_complete_event_rp0 *p_connection_update_complete_event;
#endif /* CFG_DEBUG_APP_TRACE != 0 */

  /* USER CODE BEGIN SVCCTL_App_Notification */

  /* USER CODE END SVCCTL_App_Notification */

  p_event_pckt = (hci_event_pckt*) ((hci_uart_pckt *) p_Pckt)->data;

  switch (p_event_pckt->evt)
  {
    case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
    {
      p_disconnection_complete_event = (hci_disconnection_complete_event_rp0 *) p_event_pckt->data;

      if (p_disconnection_complete_event->Connection_Handle == BleApplicationContext.BleApplicationContext_legacy.connectionHandle)
      {
        BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0;
        BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
        APP_DBG_MSG(">>== HCI_DISCONNECTION_COMPLETE_EVT_CODE\n");
        APP_DBG_MSG("     - Connection Handle:   0x%x\n     - Reason:    0x%x\n\r",
                    p_disconnection_complete_event->Connection_Handle,
                    p_disconnection_complete_event->Reason);

        /* USER CODE BEGIN EVT_DISCONN_COMPLETE_2 */

        /* USER CODE END EVT_DISCONN_COMPLETE_2 */
      }

      /* USER CODE BEGIN EVT_DISCONN_COMPLETE_1 */

      /* USER CODE END EVT_DISCONN_COMPLETE_1 */

      /* restart advertising */
      Adv_Request(APP_BLE_FAST_ADV);

      HandleNotification.CoC_Evt_Opcode = BLE_DISCON_HANDLE_EVT;
      HandleNotification.ConnectionHandle = BleApplicationContext.BleApplicationContext_legacy.connectionHandle;
      COC_APP_Notification(&HandleNotification);
      /* USER CODE BEGIN EVT_DISCONN_COMPLETE */

      /* USER CODE END EVT_DISCONN_COMPLETE */
      break; /* HCI_DISCONNECTION_COMPLETE_EVT_CODE */
    }

    case HCI_LE_META_EVT_CODE:
    {
      p_meta_evt = (evt_le_meta_event*) p_event_pckt->data;
      /* USER CODE BEGIN EVT_LE_META_EVENT */

      /* USER CODE END EVT_LE_META_EVENT */
      switch (p_meta_evt->subevent)
      {
        case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
#if (CFG_DEBUG_APP_TRACE != 0)
          p_connection_update_complete_event = (hci_le_connection_update_complete_event_rp0 *) p_meta_evt->data;
          APP_DBG_MSG(">>== HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE\n");
          APP_DBG_MSG("     - Connection Interval:   %.2f ms\n     - Connection latency:    %d\n     - Supervision Timeout: %d ms\n\r",
                       p_connection_update_complete_event->Conn_Interval*1.25,
                       p_connection_update_complete_event->Conn_Latency,
                       p_connection_update_complete_event->Supervision_Timeout*10);
#endif /* CFG_DEBUG_APP_TRACE != 0 */

          /* USER CODE BEGIN EVT_LE_CONN_UPDATE_COMPLETE */

          /* USER CODE END EVT_LE_CONN_UPDATE_COMPLETE */
          break;

        case HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE:
          p_evt_le_phy_update_complete = (hci_le_phy_update_complete_event_rp0*)p_meta_evt->data;
          APP_DBG_MSG("==>> HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE - ");
          if (p_evt_le_phy_update_complete->Status == 0)
          {
            APP_DBG_MSG("status ok \n");
          }
          else
          {
            APP_DBG_MSG("status nok \n");
          }

          ret = hci_le_read_phy(BleApplicationContext.BleApplicationContext_legacy.connectionHandle, &Tx_phy, &Rx_phy);
          if (ret != BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("==>> hci_le_read_phy : fail\n\r");
          }
          else
          {
            APP_DBG_MSG("==>> hci_le_read_phy - Success \n");

            if ((Tx_phy == TX_2M) && (Rx_phy == RX_2M))
            {
              APP_DBG_MSG("==>> PHY Param  TX= %d, RX= %d \n\r", Tx_phy, Rx_phy);
            }
            else
            {
              APP_DBG_MSG("==>> PHY Param  TX= %d, RX= %d \n\r", Tx_phy, Rx_phy);
            }
          }
          /* USER CODE BEGIN EVT_LE_PHY_UPDATE_COMPLETE */

          /* USER CODE END EVT_LE_PHY_UPDATE_COMPLETE */
          break;

        case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
        {
          p_connection_complete_event = (hci_le_connection_complete_event_rp0 *) p_meta_evt->data;
          /**
           * The connection is done, there is no need anymore to schedule the LP ADV
           */

          HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);

          APP_DBG_MSG(">>== HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE - Connection handle: 0x%x\n", p_connection_complete_event->Connection_Handle);
          APP_DBG_MSG("     - Connection established with Central: @:%02x:%02x:%02x:%02x:%02x:%02x\n",
                      p_connection_complete_event->Peer_Address[5],
                      p_connection_complete_event->Peer_Address[4],
                      p_connection_complete_event->Peer_Address[3],
                      p_connection_complete_event->Peer_Address[2],
                      p_connection_complete_event->Peer_Address[1],
                      p_connection_complete_event->Peer_Address[0]);
          APP_DBG_MSG("     - Connection Interval:   %.2f ms\n     - Connection latency:    %d\n     - Supervision Timeout: %d ms\n\r",
                      p_connection_complete_event->Conn_Interval*1.25,
                      p_connection_complete_event->Conn_Latency,
                      p_connection_complete_event->Supervision_Timeout*10
                     );

          if (BleApplicationContext.Device_Connection_Status == APP_BLE_LP_CONNECTING)
          {
            /* Connection as client */
            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_CLIENT;
          }
          else
          {
            /* Connection as server */
            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_SERVER;
          }
          BleApplicationContext.BleApplicationContext_legacy.connectionHandle = p_connection_complete_event->Connection_Handle;

          HandleNotification.CoC_Evt_Opcode = BLE_CONN_HANDLE_EVT;
          HandleNotification.ConnectionHandle = BleApplicationContext.BleApplicationContext_legacy.connectionHandle;
          COC_APP_Notification(&HandleNotification);
          /* USER CODE BEGIN HCI_EVT_LE_CONN_COMPLETE */

          /* USER CODE END HCI_EVT_LE_CONN_COMPLETE */
          break; /* HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE */
        }

        default:
          /* USER CODE BEGIN SUBEVENT_DEFAULT */

          /* USER CODE END SUBEVENT_DEFAULT */
          break;
      }

      /* USER CODE BEGIN META_EVT */

      /* USER CODE END META_EVT */
      break; /* HCI_LE_META_EVT_CODE */
    }

    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      p_blecore_evt = (evt_blecore_aci*) p_event_pckt->data;
      /* USER CODE BEGIN EVT_VENDOR */

      /* USER CODE END EVT_VENDOR */
      switch (p_blecore_evt->ecode)
      {
        /* USER CODE BEGIN ecode */
        aci_gap_pairing_complete_event_rp0 *pairing_complete;
        aci_l2cap_coc_connect_event_rp0 *coc_connect_complete;
        uint8_t Channel_Number;
        uint8_t Channel_Index_List;

        case ACI_GAP_LIMITED_DISCOVERABLE_VSEVT_CODE: 
          APP_DBG_MSG(">>== ACI_GAP_LIMITED_DISCOVERABLE_VSEVT_CODE \n");
          break; /* ACI_GAP_LIMITED_DISCOVERABLE_VSEVT_CODE */
          
        case ACI_GAP_PASS_KEY_REQ_VSEVT_CODE:  
          APP_DBG_MSG(">>== ACI_GAP_PASS_KEY_REQ_VSEVT_CODE \n");
		  
          ret = aci_gap_pass_key_resp(BleApplicationContext.BleApplicationContext_legacy.connectionHandle,123456);
          if (ret != BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("==>> aci_gap_pass_key_resp : Fail, reason: 0x%x\n", ret);
          } 
          else 
          {
            APP_DBG_MSG("==>> aci_gap_pass_key_resp : Success \n");
          }
          break; /* ACI_GAP_PASS_KEY_REQ_VSEVT_CODE */

        case ACI_GAP_AUTHORIZATION_REQ_VSEVT_CODE:    
          APP_DBG_MSG(">>== ACI_GAP_AUTHORIZATION_REQ_VSEVT_CODE\n");
          break; /* ACI_GAP_AUTHORIZATION_REQ_VSEVT_CODE */

        case ACI_GAP_SLAVE_SECURITY_INITIATED_VSEVT_CODE:   
          APP_DBG_MSG("==>> ACI_GAP_SLAVE_SECURITY_INITIATED_VSEVT_CODE \n");
          break; /* ACI_GAP_SLAVE_SECURITY_INITIATED_VSEVT_CODE */

        case ACI_GAP_BOND_LOST_VSEVT_CODE:    
          APP_DBG_MSG("==>> ACI_GAP_BOND_LOST_VSEVT_CODE \n");
          ret = aci_gap_allow_rebond(BleApplicationContext.BleApplicationContext_legacy.connectionHandle);
          if (ret != BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("==>> aci_gap_allow_rebond : Fail, reason: 0x%x\n", ret);
          } 
          else 
          {
            APP_DBG_MSG("==>> aci_gap_allow_rebond : Success \n");
          }
          break; /* ACI_GAP_BOND_LOST_VSEVT_CODE */

        case ACI_GAP_ADDR_NOT_RESOLVED_VSEVT_CODE:
          APP_DBG_MSG(">>== ACI_GAP_ADDR_NOT_RESOLVED_VSEVT_CODE \n");
          break; /* ACI_GAP_ADDR_NOT_RESOLVED_VSEVT_CODE */
      
        case (ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE):
          APP_DBG_MSG(">>== ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE\n");
          break; /* ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE */    

        case (ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE):
          APP_DBG_MSG(">>== ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE\n");
          APP_DBG_MSG("     - numeric_value = %ld\n",
                      ((aci_gap_numeric_comparison_value_event_rp0 *)(p_blecore_evt->data))->Numeric_Value);
          APP_DBG_MSG("     - Hex_value = %lx\n",
                      ((aci_gap_numeric_comparison_value_event_rp0 *)(p_blecore_evt->data))->Numeric_Value);
          ret = aci_gap_numeric_comparison_value_confirm_yesno(BleApplicationContext.BleApplicationContext_legacy.connectionHandle, YES); /* CONFIRM_YES = 1 */
          if (ret != BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("==>> aci_gap_numeric_comparison_value_confirm_yesno-->YES : Fail, reason: 0x%x\n", ret);
          } 
          else 
          {
            APP_DBG_MSG("==>> aci_gap_numeric_comparison_value_confirm_yesno-->YES : Success \n");
          }
          break;

        case (ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE):
        {
          pairing_complete = (aci_gap_pairing_complete_event_rp0*)p_blecore_evt->data;

          APP_DBG_MSG(">>== ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE\n");
          if (pairing_complete->Status == 0)
          {
            APP_DBG_MSG("     - Pairing Success\n");
          }
          else
          {
            APP_DBG_MSG("     - Pairing KO \n     - Status: 0x%x\n     - Reason: 0x%x\n",pairing_complete->Status, pairing_complete->Reason);
          }
          APP_DBG_MSG("\n");
        }
          break;  
          
        case (ACI_L2CAP_COC_CONNECT_VSEVT_CODE):
        {
          coc_connect_complete = (aci_l2cap_coc_connect_event_rp0*)p_blecore_evt->data;

          APP_DBG_MSG(">>== ACI_L2CAP_COC_CONNECT_VSEVT_CODE\n");
          BleCoCContext.Conn_Handle = coc_connect_complete->Connection_Handle;
          BleCoCContext.Max_Transmission_Unit = coc_connect_complete->MTU;
          BleCoCContext.Max_Payload_Size = coc_connect_complete->MPS;
          BleCoCContext.Initial_Credits = coc_connect_complete->Initial_Credits;
          BleCoCContext.Channel_Number = coc_connect_complete->Channel_Number;
          BleCoCContext.SPSM = coc_connect_complete->SPSM;
          
          ret = aci_l2cap_coc_connect_confirm(BleCoCContext.Conn_Handle, BleCoCContext.Max_Transmission_Unit, BleCoCContext.Max_Payload_Size, BleCoCContext.Initial_Credits, 0x0000, &Channel_Number, &Channel_Index_List); 
          BleCoCContext.Channel_Number = Channel_Number;
          BleCoCContext.Channel_Index_List = Channel_Index_List;
          
          if (ret != BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("==>> aci_l2cap_coc_connect_confirm : Fail, reason: 0x%x\n", ret);
          } 
          else 
          {
            APP_DBG_MSG("==>> aci_l2cap_coc_connect_confirm : Success \n");
          }

        }
          break; 
          
        case ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE:
        {
          exchange_mtu_resp = (aci_att_exchange_mtu_resp_event_rp0 *)p_blecore_evt->data;
          APP_DBG_MSG("**MTU_size = %d \n",exchange_mtu_resp->Server_RX_MTU );
          APP_DBG_MSG("\r\n\r");
          
          APP_DBG_MSG("set data length \n");
          ret = hci_le_set_data_length(BleCoCContext.Conn_Handle,MAX_TX_OCTETS,MAX_TX_TIME);

          if (ret != BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("set_data_length_command : Fail, reason: 0x%x\n", ret);
          }
        }
        break;
        
        case ACI_L2CAP_COC_TX_POOL_AVAILABLE_VSEVT_CODE:         
        {
          Resume_DataSend();
        }
        break;
          
        case (ACI_L2CAP_COC_RX_DATA_VSEVT_CODE):
        {
          coc_rx_data_event = (aci_l2cap_coc_rx_data_event_rp0*)p_blecore_evt->data;

          HandleNotification.CoC_Evt_Opcode = L2CAP_DATA_RECEIVED;
          HandleNotification.ConnectionHandle = BleApplicationContext.BleApplicationContext_legacy.connectionHandle;
          HandleNotification.DataLength = coc_rx_data_event->Length;
          HandleNotification.DataTransfered.pPayload = (coc_rx_data_event->Data);
          HandleNotification.DataTransfered.pPayload_n = *((uint32_t*) &(coc_rx_data_event->Data[0]));
          __disable_irq();
          if (HandleNotification.DataTransfered.pPayload_n >= (HandleNotification.DataTransfered.pPayload_n_1 + 2))
          {
            packet_lost +=
              ((HandleNotification.DataTransfered.pPayload_n - HandleNotification.DataTransfered.pPayload_n_1) - 1);
          }
          __enable_irq();
          /* CRC computation */
          CRC_Result = APP_BLE_ComputeCRC8((uint8_t*) (coc_rx_data_event->Data), (coc_rx_data_event->Length) - 1);
          //APP_DBG_MSG("CRC_Result = %x /s\n", CRC_Result);
          /* get low weight byte */
          CRC_Received = (uint8_t) (coc_rx_data_event->Data[DATA_MAX_PACKET_SIZE-1]);
          //APP_DBG_MSG("CRC_Received = %x /s\n", CRC_Received);
          
          if (CRC_Received != CRC_Result)
          {
            N+=1;
            APP_DBG_MSG("** data error **  N= %d \r\n",(int)N);
          }
          HandleNotification.DataTransfered.pPayload_n_1 = HandleNotification.DataTransfered.pPayload_n;
            
          COC_APP_Notification(&HandleNotification);
          
        }
        break;
        /* USER CODE END ecode */

        case ACI_L2CAP_CONNECTION_UPDATE_RESP_VSEVT_CODE:
#if (L2CAP_REQUEST_NEW_CONN_PARAM != 0)
          mutex = 1;
#endif /* L2CAP_REQUEST_NEW_CONN_PARAM != 0 */
          /* USER CODE BEGIN EVT_BLUE_L2CAP_CONNECTION_UPDATE_RESP */

          /* USER CODE END EVT_BLUE_L2CAP_CONNECTION_UPDATE_RESP */
          break;

        case ACI_GAP_PROC_COMPLETE_VSEVT_CODE:
          APP_DBG_MSG(">>== ACI_GAP_PROC_COMPLETE_VSEVT_CODE \r");
          /* USER CODE BEGIN EVT_BLUE_GAP_PROCEDURE_COMPLETE */

          /* USER CODE END EVT_BLUE_GAP_PROCEDURE_COMPLETE */
          break; /* ACI_GAP_PROC_COMPLETE_VSEVT_CODE */

#if (RADIO_ACTIVITY_EVENT != 0)
        case ACI_HAL_END_OF_RADIO_ACTIVITY_VSEVT_CODE:
          /* USER CODE BEGIN RADIO_ACTIVITY_EVENT*/
          BSP_LED_On(LED_GREEN);
          HW_TS_Start(BleApplicationContext.SwitchOffGPIO_timer_Id, (uint32_t)LED_ON_TIMEOUT);
          /* USER CODE END RADIO_ACTIVITY_EVENT*/
          break; /* ACI_HAL_END_OF_RADIO_ACTIVITY_VSEVT_CODE */
#endif /* RADIO_ACTIVITY_EVENT != 0 */

        /* USER CODE BEGIN BLUE_EVT */

        /* USER CODE END BLUE_EVT */
      }
      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT */

      /* USER CODE END EVENT_PCKT */

    default:
      /* USER CODE BEGIN ECODE_DEFAULT*/

      /* USER CODE END ECODE_DEFAULT*/
      break;
  }

  return (SVCCTL_UserEvtFlowEnable);
}

APP_BLE_ConnStatus_t APP_BLE_Get_Server_Connection_Status(void)
{
  return BleApplicationContext.Device_Connection_Status;
}

/* USER CODE BEGIN FD*/
void APP_BLE_Key_Button1_Action(void)
{
  COC_App_KeyButton1Action();
}

void APP_BLE_Key_Button2_Action(void)
{
  COC_App_KeyButton2Action(); 
}

void APP_BLE_Key_Button3_Action(void)
{
  COC_App_KeyButton3Action();
}

#if defined(__GNUC__)
uint8_t __attribute__((optimize("Os"))) APP_BLE_ComputeCRC8( uint8_t *DataPtr , uint8_t Datalen )
#else
uint8_t APP_BLE_ComputeCRC8( uint8_t *DataPtr , uint8_t Datalen )
#endif
{
  uint8_t i, j;
  const uint8_t PolynomeCRC = 0x97;
  uint8_t CRC8 = 0x00;

  for (i = 0; i < Datalen; i++)
  {
    CRC8 ^= DataPtr[i];
    for (j = 0; j < 8; j++)
    {
      if ((CRC8 & 0x80) != 0)
      {
        CRC8 = (uint8_t) ((CRC8 << 1) ^ PolynomeCRC);
      }
      else
      {
        CRC8 <<= 1;
      }
    }
  }
  return (CRC8);
}

void BLE_SVC_GAP_Security_Req(void)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  
  ret = aci_gap_slave_security_req(BleApplicationContext.BleApplicationContext_legacy.connectionHandle);
  if (ret == BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("slave req security cmd OK ok\n");
  }
  else 
  {
    APP_DBG_MSG("slave req security cmd NOK\n");
  }
}
/* USER CODE END FD*/

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
static void Ble_Tl_Init(void)
{
  HCI_TL_HciInitConf_t Hci_Tl_Init_Conf;

  Hci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&BleCmdBuffer;
  Hci_Tl_Init_Conf.StatusNotCallBack = BLE_StatusNot;
  hci_init(BLE_UserEvtRx, (void*) &Hci_Tl_Init_Conf);

  return;
}

static void Ble_Hci_Gap_Gatt_Init(void)
{
  uint8_t role;
  uint16_t gap_service_handle, gap_dev_name_char_handle, gap_appearance_char_handle;
  const uint8_t *p_bd_addr;

#if (CFG_BLE_ADDRESS_TYPE != PUBLIC_ADDR)
  uint32_t a_srd_bd_addr[2] = {0,0};
#endif
  uint16_t a_appearance[1] = {BLE_CFG_GAP_APPEARANCE};
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Ble_Hci_Gap_Gatt_Init*/

  /* USER CODE END Ble_Hci_Gap_Gatt_Init*/

  APP_DBG_MSG("==>> Start Ble_Hci_Gap_Gatt_Init function\n");

  /**
   * Initialize HCI layer
   */
  /*HCI Reset to synchronise BLE Stack*/
  ret = hci_reset();
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : hci_reset command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: hci_reset command\n");
  }

  /**
   * Write the BD Address
   */
  p_bd_addr = BleGetBdAddress();
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                            CONFIG_DATA_PUBADDR_LEN,
                            (uint8_t*) p_bd_addr);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_PUBADDR_OFFSET, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_PUBADDR_OFFSET\n");
    APP_DBG_MSG("  Public Bluetooth Address: %02x:%02x:%02x:%02x:%02x:%02x\n",p_bd_addr[5],p_bd_addr[4],p_bd_addr[3],p_bd_addr[2],p_bd_addr[1],p_bd_addr[0]);
  }

#if (CFG_BLE_ADDRESS_TYPE == PUBLIC_ADDR)
  /* BLE MAC in ADV Packet */
  a_ManufData[ sizeof(a_ManufData)-6] = p_bd_addr[5];
  a_ManufData[ sizeof(a_ManufData)-5] = p_bd_addr[4];
  a_ManufData[ sizeof(a_ManufData)-4] = p_bd_addr[3];
  a_ManufData[ sizeof(a_ManufData)-3] = p_bd_addr[2];
  a_ManufData[ sizeof(a_ManufData)-2] = p_bd_addr[1];
  a_ManufData[ sizeof(a_ManufData)-1] = p_bd_addr[0];
#endif /* CFG_BLE_ADDRESS_TYPE == PUBLIC_ADDR */

  /**
   * Static random Address
   * The two upper bits shall be set to 1
   * The lowest 32bits is read from the UDN to differentiate between devices
   * The RNG may be used to provide a random number on each power on
   */
#if (CFG_IDENTITY_ADDRESS == GAP_STATIC_RANDOM_ADDR)
#if defined(CFG_STATIC_RANDOM_ADDRESS)
  a_srd_bd_addr[0] = CFG_STATIC_RANDOM_ADDRESS & 0xFFFFFFFF;
  a_srd_bd_addr[1] = (uint32_t)((uint64_t)CFG_STATIC_RANDOM_ADDRESS >> 32);
  a_srd_bd_addr[1] |= 0xC000; /* The two upper bits shall be set to 1 */
#else
  /* Get RNG semaphore */
  while(LL_HSEM_1StepLock(HSEM, CFG_HW_RNG_SEMID));

  /* Enable RNG */
  __HAL_RNG_ENABLE(&hrng);

  /* Enable HSI48 oscillator */
  LL_RCC_HSI48_Enable();
  /* Wait until HSI48 is ready */
  while(! LL_RCC_HSI48_IsReady());

  if (HAL_RNG_GenerateRandomNumber(&hrng, &a_srd_bd_addr[1]) != HAL_OK)
  {
    /* Random number generation error */
    Error_Handler();
  }
  if (HAL_RNG_GenerateRandomNumber(&hrng, &a_srd_bd_addr[0]) != HAL_OK)
  {
    /* Random number generation error */
    Error_Handler();
  }
  a_srd_bd_addr[1] |= 0xC000; /* The two upper bits shall be set to 1 */

  /* Disable HSI48 oscillator */
  LL_RCC_HSI48_Disable();

  /* Disable RNG */
  __HAL_RNG_DISABLE(&hrng);

  /* Release RNG semaphore */
  LL_HSEM_ReleaseLock(HSEM, CFG_HW_RNG_SEMID, 0);
#endif /* CFG_STATIC_RANDOM_ADDRESS */
#endif

#if (CFG_BLE_ADDRESS_TYPE == GAP_STATIC_RANDOM_ADDR)
  /* BLE MAC in ADV Packet */
  a_ManufData[ sizeof(a_ManufData)-6] = a_srd_bd_addr[1] >> 8 ;
  a_ManufData[ sizeof(a_ManufData)-5] = a_srd_bd_addr[1];
  a_ManufData[ sizeof(a_ManufData)-4] = a_srd_bd_addr[0] >> 24;
  a_ManufData[ sizeof(a_ManufData)-3] = a_srd_bd_addr[0] >> 16;
  a_ManufData[ sizeof(a_ManufData)-2] = a_srd_bd_addr[0] >> 8;
  a_ManufData[ sizeof(a_ManufData)-1] = a_srd_bd_addr[0];
#endif

#if (CFG_BLE_ADDRESS_TYPE != PUBLIC_ADDR)
  ret = aci_hal_write_config_data(CONFIG_DATA_RANDOM_ADDRESS_OFFSET, CONFIG_DATA_RANDOM_ADDRESS_LEN, (uint8_t*)a_srd_bd_addr);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_RANDOM_ADDRESS_OFFSET, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_RANDOM_ADDRESS_OFFSET\n");
    APP_DBG_MSG("  Random Bluetooth Address: %02x:%02x:%02x:%02x:%02x:%02x\n", (uint8_t)(a_srd_bd_addr[1] >> 8),
                                                                               (uint8_t)(a_srd_bd_addr[1]),
                                                                               (uint8_t)(a_srd_bd_addr[0] >> 24),
                                                                               (uint8_t)(a_srd_bd_addr[0] >> 16),
                                                                               (uint8_t)(a_srd_bd_addr[0] >> 8),
                                                                               (uint8_t)(a_srd_bd_addr[0]));
  }
#endif /* CFG_BLE_ADDRESS_TYPE != PUBLIC_ADDR */

  /**
   * Write Identity root key used to derive LTK and CSRK
   */
  ret = aci_hal_write_config_data(CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, (uint8_t*)a_BLE_CfgIrValue);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_IR_OFFSET, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_IR_OFFSET\n");
  }

  /**
   * Write Encryption root key used to derive LTK and CSRK
   */
  ret = aci_hal_write_config_data(CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, (uint8_t*)a_BLE_CfgErValue);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_ER_OFFSET, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_ER_OFFSET\n");
  }

  /**
   * Set TX Power.
   */
  ret = aci_hal_set_tx_power_level(1, CFG_TX_POWER);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_set_tx_power_level command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_set_tx_power_level command\n");
  }

  /**
   * Initialize GATT interface
   */
  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_init command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_init command\n");
  }

  /**
   * Initialize GAP interface
   */
  role = 0;

#if (BLE_CFG_PERIPHERAL == 1)
  role |= GAP_PERIPHERAL_ROLE;
#endif /* BLE_CFG_PERIPHERAL == 1 */

  if (role > 0)
  {
    const char *name = "BLE_COC";
    ret = aci_gap_init(role,
                       CFG_PRIVACY,
                       APPBLE_GAP_DEVICE_NAME_LENGTH,
                       &gap_service_handle,
                       &gap_dev_name_char_handle,
                       &gap_appearance_char_handle);

    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gap_init command, result: 0x%x \n", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gap_init command\n");
    }

    ret = aci_gatt_update_char_value(gap_service_handle, gap_dev_name_char_handle, 0, strlen(name), (uint8_t *) name);
    if (ret != BLE_STATUS_SUCCESS)
    {
      BLE_DBG_SVCCTL_MSG("  Fail   : aci_gatt_update_char_value - Device Name\n");
    }
    else
    {
      BLE_DBG_SVCCTL_MSG("  Success: aci_gatt_update_char_value - Device Name\n");
    }
  }

  ret = aci_gatt_update_char_value(gap_service_handle,
                                   gap_appearance_char_handle,
                                   0,
                                   2,
                                   (uint8_t *)&a_appearance);
  if (ret != BLE_STATUS_SUCCESS)
  {
    BLE_DBG_SVCCTL_MSG("  Fail   : aci_gatt_update_char_value - Appearance\n");
  }
  else
  {
    BLE_DBG_SVCCTL_MSG("  Success: aci_gatt_update_char_value - Appearance\n");
  }

  /**
   * Initialize Default PHY
   */
  ret = hci_le_set_default_phy(ALL_PHYS_PREFERENCE,
                               TX_1M | TX_2M, 
                               RX_1M | RX_2M);
  
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : hci_le_set_default_phy command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: hci_le_set_default_phy command\n");
  }

  /**
   * Initialize IO capability
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability = CFG_IO_CAPABILITY;
  ret = aci_gap_set_io_capability(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gap_set_io_capability command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gap_set_io_capability command\n");
  }

  /**
   * Initialize authentication
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode = CFG_MITM_PROTECTION;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin = CFG_ENCRYPTION_KEY_SIZE_MIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax = CFG_ENCRYPTION_KEY_SIZE_MAX;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin = CFG_USED_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin = CFG_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode = CFG_BONDING_MODE;
  /* USER CODE BEGIN Ble_Hci_Gap_Gatt_Init_1*/

  /* USER CODE END Ble_Hci_Gap_Gatt_Init_1*/

  ret = aci_gap_set_authentication_requirement(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
                                               CFG_SC_SUPPORT,
                                               CFG_KEYPRESS_NOTIFICATION_SUPPORT,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin,
                                               CFG_IDENTITY_ADDRESS);

  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gap_set_authentication_requirement command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gap_set_authentication_requirement command\n");
  }

  /**
   * Initialize whitelist
   */
  if (BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode)
  {
    ret = aci_gap_configure_whitelist();
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gap_configure_whitelist command, result: 0x%x \n", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gap_configure_whitelist command\n");
    }
  }
  APP_DBG_MSG("==>> End Ble_Hci_Gap_Gatt_Init function\n\r");
}

static void Adv_Request(APP_BLE_ConnStatus_t NewStatus)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint16_t Min_Inter, Max_Inter;

  if (NewStatus == APP_BLE_FAST_ADV)
  {
    Min_Inter = AdvIntervalMin;
    Max_Inter = AdvIntervalMax;
  }
  else
  {
    Min_Inter = CFG_LP_CONN_ADV_INTERVAL_MIN;
    Max_Inter = CFG_LP_CONN_ADV_INTERVAL_MAX;
  }

  /**
   * Stop the timer, it will be restarted for a new shot
   * It does not hurt if the timer was not running
   */
  HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);

  if ((NewStatus == APP_BLE_LP_ADV)
      && ((BleApplicationContext.Device_Connection_Status == APP_BLE_FAST_ADV)
          || (BleApplicationContext.Device_Connection_Status == APP_BLE_LP_ADV)))
  {
    /* Connection in ADVERTISE mode have to stop the current advertising */
    ret = aci_gap_set_non_discoverable();
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("==>> aci_gap_set_non_discoverable - Stop Advertising Failed , result: %d \n", ret);
    }
    else
    {
      APP_DBG_MSG("==>> aci_gap_set_non_discoverable - Successfully Stopped Advertising \n");
    }
  }

  BleApplicationContext.Device_Connection_Status = NewStatus;
  /* Start Fast or Low Power Advertising */
  ret = aci_gap_set_discoverable(ADV_IND,
                                 Min_Inter,
                                 Max_Inter,
                                 CFG_BLE_ADDRESS_TYPE,
                                 NO_WHITE_LIST_USE, /* use white list */
                                 sizeof(a_LocalName),
                                 (uint8_t*) &a_LocalName,
                                 BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen,
                                 BleApplicationContext.BleApplicationContext_legacy.advtServUUID,
                                 0,
                                 0);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("==>> aci_gap_set_discoverable - fail, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("==>> aci_gap_set_discoverable - Success\n");
  }

  /* Update Advertising data */
  ret = aci_gap_update_adv_data(sizeof(a_ManufData), (uint8_t*) a_ManufData);
  if (ret != BLE_STATUS_SUCCESS)
  {
    if (NewStatus == APP_BLE_FAST_ADV)
    {
      APP_DBG_MSG("==>> Start Fast Advertising Failed , result: %d \n\r", ret);
    }
    else
    {
      APP_DBG_MSG("==>> Start Low Power Advertising Failed , result: %d \n\r", ret);
    }
  }
  else
  {
    if (NewStatus == APP_BLE_FAST_ADV)
    {
      APP_DBG_MSG("==>> Success: Start Fast Advertising \n\r");
      /* Start Timer to STOP ADV - TIMEOUT - and next Restart Low Power Advertising */
      HW_TS_Start(BleApplicationContext.Advertising_mgr_timer_Id, INITIAL_ADV_TIMEOUT);
    }
    else
    {
      APP_DBG_MSG("==>> Success: Start Low Power Advertising \n\r");
    }
  }

  return;
}

const uint8_t* BleGetBdAddress(void)
{
  uint8_t *p_otp_addr;
  const uint8_t *p_bd_addr;
  uint32_t udn;
  uint32_t company_id;
  uint32_t device_id;

  udn = LL_FLASH_GetUDN();

  if (udn != 0xFFFFFFFF)
  {
    company_id = LL_FLASH_GetSTCompanyID();
    device_id = LL_FLASH_GetDeviceID();

    /**
     * Public Address with the ST company ID
     * bit[47:24] : 24bits (OUI) equal to the company ID
     * bit[23:16] : Device ID.
     * bit[15:0] : The last 16bits from the UDN
     * Note: In order to use the Public Address in a final product, a dedicated
     * 24bits company ID (OUI) shall be bought.
     */
    a_BdAddrUdn[0] = (uint8_t)(udn & 0x000000FF);
    a_BdAddrUdn[1] = (uint8_t)((udn & 0x0000FF00) >> 8);
    a_BdAddrUdn[2] = (uint8_t)device_id;
    a_BdAddrUdn[3] = (uint8_t)(company_id & 0x000000FF);
    a_BdAddrUdn[4] = (uint8_t)((company_id & 0x0000FF00) >> 8);
    a_BdAddrUdn[5] = (uint8_t)((company_id & 0x00FF0000) >> 16);

    p_bd_addr = (const uint8_t *)a_BdAddrUdn;
  }
  else
  {
    p_otp_addr = OTP_Read(0);
    if (p_otp_addr)
    {
      p_bd_addr = ((OTP_ID0_t*)p_otp_addr)->bd_address;
    }
    else
    {
      p_bd_addr = a_MBdAddr;
    }
  }

  return p_bd_addr;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTION */

/* USER CODE END FD_LOCAL_FUNCTION */

static void Adv_Cancel(void)
{
  /* USER CODE BEGIN Adv_Cancel_1 */
  BSP_LED_Off(LED_GREEN);
  /* USER CODE END Adv_Cancel_1 */

  if (BleApplicationContext.Device_Connection_Status != APP_BLE_CONNECTED_SERVER)
  {
    tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

    ret = aci_gap_set_non_discoverable();

    BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("** STOP ADVERTISING **  Failed \r\n\r");
    }
    else
    {
      APP_DBG_MSG("  \r\n\r");
      APP_DBG_MSG("** STOP ADVERTISING **  \r\n\r");
    }
  }

  /* USER CODE BEGIN Adv_Cancel_2 */

  /* USER CODE END Adv_Cancel_2 */

  return;
}

static void Adv_Cancel_Req(void)
{
  /* USER CODE BEGIN Adv_Cancel_Req_1 */

  /* USER CODE END Adv_Cancel_Req_1 */

  UTIL_SEQ_SetTask(1 << CFG_TASK_ADV_CANCEL_ID, CFG_SCH_PRIO_0);

  /* USER CODE BEGIN Adv_Cancel_Req_2 */

  /* USER CODE END Adv_Cancel_Req_2 */

  return;
}

static void Switch_OFF_GPIO()
{
  /* USER CODE BEGIN Switch_OFF_GPIO */
  BSP_LED_Off(LED_GREEN);
  /* USER CODE END Switch_OFF_GPIO */
}

void BLE_SVC_L2CAP_Conn_Update(uint16_t ConnectionHandle)
{
  /* USER CODE BEGIN BLE_SVC_L2CAP_Conn_Update_1 */

  /* USER CODE END BLE_SVC_L2CAP_Conn_Update_1 */

    tBleStatus ret;

    ret = aci_l2cap_connection_parameter_update_req(ConnectionHandle,
                                                    CONN_P45, CONN_P45,
                                                    0, 0x3E8);
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("BLE_SVC_L2CAP_Conn_Update(), Failed \r\n\r");
    }
    else
    {
      APP_DBG_MSG("BLE_SVC_L2CAP_Conn_Update(), Successfully \r\n\r");
    }

  /* USER CODE BEGIN BLE_SVC_L2CAP_Conn_Update_2 */

  /* USER CODE END BLE_SVC_L2CAP_Conn_Update_2 */

  return;
}

static void Connection_Interval_Update_Req(void)
{
  if (BleApplicationContext.Device_Connection_Status != APP_BLE_FAST_ADV && BleApplicationContext.Device_Connection_Status != APP_BLE_IDLE)
  {
    BLE_SVC_L2CAP_Conn_Update(BleApplicationContext.BleApplicationContext_legacy.connectionHandle);
  }

  return;
}


/* USER CODE BEGIN FD_SPECIFIC_FUNCTIONS */
void Resume_DataSend(void)
{
  CoC_App_Context.CoCFlowStatus = COC_APP_FLOW_ON;
  UTIL_SEQ_SetTask(1 << CFG_TASK_DATA_TRANSFER_ID, CFG_SCH_PRIO_0);
}

static void L2cap_CoC_Throughput_proc(){
  
  UTIL_SEQ_SetTask(1 << CFG_TASK_DATA_WRITE_ID, CFG_SCH_PRIO_0);
}
/* USER CODE END FD_SPECIFIC_FUNCTIONS */
/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/
void hci_notify_asynch_evt(void* p_Data)
{
  UTIL_SEQ_SetTask(1 << CFG_TASK_HCI_ASYNCH_EVT_ID, CFG_SCH_PRIO_0);

  return;
}

void hci_cmd_resp_release(uint32_t Flag)
{
  UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);

  return;
}

void hci_cmd_resp_wait(uint32_t Timeout)
{
  UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);

  return;
}

static void BLE_UserEvtRx(void *p_Payload)
{
  SVCCTL_UserEvtFlowStatus_t svctl_return_status;
  tHCI_UserEvtRxParam *p_param;

  p_param = (tHCI_UserEvtRxParam *)p_Payload;

  svctl_return_status = SVCCTL_UserEvtRx((void *)&(p_param->pckt->evtserial));
  if (svctl_return_status != SVCCTL_UserEvtFlowDisable)
  {
    p_param->status = HCI_TL_UserEventFlow_Enable;
  }
  else
  {
    p_param->status = HCI_TL_UserEventFlow_Disable;
  }

  return;
}

static void BLE_StatusNot(HCI_TL_CmdStatus_t Status)
{
  uint32_t task_id_list;
  switch (Status)
  {
    case HCI_TL_CmdBusy:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_PauseTask(task_id_list);
      /* USER CODE BEGIN HCI_TL_CmdBusy */

      /* USER CODE END HCI_TL_CmdBusy */
      break;

    case HCI_TL_CmdAvailable:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_ResumeTask(task_id_list);
      /* USER CODE BEGIN HCI_TL_CmdAvailable */

      /* USER CODE END HCI_TL_CmdAvailable */
      break;

    default:
      /* USER CODE BEGIN Status */

      /* USER CODE END Status */
      break;
  }

  return;
}

void SVCCTL_ResumeUserEventFlow(void)
{
  hci_resume_flow();

  return;
}

/* USER CODE BEGIN FD_WRAP_FUNCTIONS */
void BLE_App_Toggle_Connection_Interval( void)
{
  /* USER CODE BEGIN BLE_SVC_L2CAP_Conn_Update_1 */
  tBleStatus status;
  uint32_t paramA, paramB, paramC, paramD;
  paramC = 0x0000;
  paramD = 0x01F4;
  
  if (BleApplicationContext.connIntervalFlag != 0)
  {
    BleApplicationContext.connIntervalFlag = 0;
    paramA = CONN_P(11.25);
    paramB = CONN_P(11.25);
  }
  else
  {
    BleApplicationContext.connIntervalFlag = 1;
    paramA = CONN_P(26.25);
    paramB = CONN_P(26.25);
  }
  /* USER CODE END BLE_SVC_L2CAP_Conn_Update_1 */
    
  status = aci_l2cap_connection_parameter_update_req(BleApplicationContext.BleApplicationContext_legacy.connectionHandle,
                                                     paramA,
                                                     paramB,
                                                     paramC,
                                                     paramD);
  if (status != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : BLE_SVC_L2CAP_Conn_Update() 0x%x \r\n\r", status);
  }
  else
  {
    APP_DBG_MSG("  Success: BLE_SVC_L2CAP_Conn_Update() \r\n\r");
  }

  /* USER CODE BEGIN BLE_SVC_L2CAP_Conn_Update_2 */

  /* USER CODE END BLE_SVC_L2CAP_Conn_Update_2 */
  return;
}

void BLE_SVC_GAP_Clear_DataBase(void)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  
  ret = aci_gap_clear_security_db();
  if (ret == BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("clear security db cmd OK ok\n");
  }
  else 
  {
    APP_DBG_MSG("clear security db  cmd NOK\n");
  }
}
/* USER CODE END FD_WRAP_FUNCTIONS */
