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

#include "p2p_central_app.h"

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
} tSecurityParams;

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
} BleGlobalContext_t;

typedef struct
{
  BleGlobalContext_t BleApplicationContext_legacy;
  APP_BLE_ConnStatus_t Device_Connection_Status;
  uint8_t SwitchOffGPIO_timer_Id;
  uint8_t DeviceServerFound;
} BleApplicationContext_t;

#if OOB_DEMO != 0
typedef struct
{
  uint8_t  Identifier;
  uint16_t L2CAP_Length;
  uint16_t Interval_Min;
  uint16_t Interval_Max;
  uint16_t Slave_Latency;
  uint16_t Timeout_Multiplier;
} APP_BLE_p2p_Conn_Update_req_t;
#endif

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define APPBLE_GAP_DEVICE_NAME_LENGTH 7
#define BD_ADDR_SIZE_LOCAL    6

#define FORCE_REBOND                                                        0x01

/* USER CODE BEGIN PD */
#if OOB_DEMO != 0 
#define LED_ON_TIMEOUT            (0.005*1000*1000/CFG_TS_TICK_VAL) /**< 5ms */
#define MAX_TX_OCTETS                   251
#define MAX_TX_TIME                     2120
#endif 
/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

static const uint8_t M_bd_addr[BD_ADDR_SIZE_LOCAL] =
{
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000000000FF)),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00000000FF00) >> 8),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x000000FF0000) >> 16),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000FF000000) >> 24),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00FF00000000) >> 32),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0xFF0000000000) >> 40)
};

static uint8_t bd_addr_udn[BD_ADDR_SIZE_LOCAL];

/**
*   Identity root key used to derive LTK and CSRK
*/
static const uint8_t BLE_CFG_IR_VALUE[16] = CFG_BLE_IRK;

/**
* Encryption root key used to derive LTK and CSRK
*/
static const uint8_t BLE_CFG_ER_VALUE[16] = CFG_BLE_ERK;

tBDAddr SERVER_REMOTE_BDADDR;

COC_APP_ConnHandle_Not_evt_t handleNotification;
static uint16_t packet_lost;
uint32_t Number_Corrupted_Data = 0;

PLACE_IN_SECTION("BLE_APP_CONTEXT") static BleApplicationContext_t BleApplicationContext;

#if OOB_DEMO != 0
APP_BLE_p2p_Conn_Update_req_t APP_BLE_p2p_Conn_Update_req;
#endif

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void BLE_UserEvtRx(void * pPayload);
static void BLE_StatusNot(HCI_TL_CmdStatus_t status);
static void Ble_Tl_Init(void);
static void Ble_Hci_Gap_Gatt_Init(void);
static const uint8_t* BleGetBdAddress(void);
static void Scan_Request(void);
static void Connect_Request(void);
static void Switch_OFF_GPIO(void);
static void Resume_DataSend(void);
static void L2cap_CoC_Throughput_proc();

/* USER CODE BEGIN PFP */
extern BleCoCContext_t BleCoCContext;
CoC_Central_App_Context_t CoC_Central_App_Context;
uint8_t TimerL2capDataThroughputWrite_Id;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Functions Definition ------------------------------------------------------*/
void APP_BLE_Init(void)
{
  SHCI_CmdStatus_t status;
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
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
   * From here, all initialization are BLE application specific
   */
  UTIL_SEQ_RegTask(1<<CFG_TASK_START_SCAN_ID, UTIL_SEQ_RFU, Scan_Request);
  UTIL_SEQ_RegTask(1<<CFG_TASK_CONN_DEV_1_ID, UTIL_SEQ_RFU, Connect_Request);

  /**
   * Initialization of the BLE App Context
   */
  BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;

  /*Radio mask Activity*/
#if (OOB_DEMO != 0)
  ret = aci_hal_set_radio_activity_mask(0x0020);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_set_radio_activity_mask command, result: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_set_radio_activity_mask command\n\r");
  }
  APP_DBG_MSG("\n");
#endif

  COC_APP_Init();

  /* USER CODE BEGIN APP_BLE_Init_3 */
  /**
    * Create timer for L2CAP Data Throughput process (write data)
    */
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(TimerL2capDataThroughputWrite_Id), hw_ts_SingleShot, L2cap_CoC_Throughput_proc);
  /* USER CODE END APP_BLE_Init_3 */

#if (OOB_DEMO != 0)
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.SwitchOffGPIO_timer_Id), hw_ts_SingleShot, Switch_OFF_GPIO);
#endif

  /**
   * Start scanning
   */
  UTIL_SEQ_SetTask(1 << CFG_TASK_START_SCAN_ID, CFG_SCH_PRIO_0);

  /* USER CODE BEGIN APP_BLE_Init_2 */

  /* USER CODE END APP_BLE_Init_2 */
  return;
}

SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification(void *pckt)
{
  hci_event_pckt *event_pckt;
  evt_le_meta_event *meta_evt;
  hci_le_connection_complete_event_rp0 * connection_complete_event;
  hci_le_connection_update_complete_event_rp0 * connection_update_complete_event;
  aci_l2cap_coc_connect_confirm_event_rp0 *coc_connect_confirm_event;
  aci_l2cap_coc_rx_data_event_rp0 *coc_rx_data_event;
  aci_att_exchange_mtu_resp_event_rp0 * exchange_mtu_resp;
  uint8_t Tx_phy, Rx_phy;
  evt_blecore_aci *blecore_evt;
  hci_le_advertising_report_event_rp0 * le_advertising_event;
  event_pckt = (hci_event_pckt*) ((hci_uart_pckt *) pckt)->data;
  hci_disconnection_complete_event_rp0 *cc = (void *) event_pckt->data;
  hci_le_phy_update_complete_event_rp0        *p_evt_le_phy_update_complete;
  aci_gap_pairing_complete_event_rp0 * pairing_complete;
  uint8_t event_type, event_data_size;
  int k = 0;
  uint8_t adtype, adlength;
  uint8_t CRC_Result;
  uint8_t CRC_Received;
  
#if (OOB_DEMO != 0)
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
#endif

  switch (event_pckt->evt)
  {
    /* USER CODE BEGIN evt */

    /* USER CODE END evt */
    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      {
        handleNotification.COC_Evt_Opcode = BLE_DISCON_HANDLE_EVT;
        blecore_evt = (evt_blecore_aci*) event_pckt->data;
        /* USER CODE BEGIN EVT_VENDOR */

        /* USER CODE END EVT_VENDOR */
        switch (blecore_evt->ecode)
        {
          /* USER CODE BEGIN ecode */
        case ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE:
          {
            exchange_mtu_resp = (aci_att_exchange_mtu_resp_event_rp0 *)blecore_evt->data;
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
          /* USER CODE END ecode */

          case ACI_GAP_PROC_COMPLETE_VSEVT_CODE:
            {
              /* USER CODE BEGIN EVT_BLUE_GAP_PROCEDURE_COMPLETE */

              /* USER CODE END EVT_BLUE_GAP_PROCEDURE_COMPLETE */
              aci_gap_proc_complete_event_rp0 *gap_evt_proc_complete = (void*) blecore_evt->data;
              /* CHECK GAP GENERAL DISCOVERY PROCEDURE COMPLETED & SUCCEED */
              if (gap_evt_proc_complete->Procedure_Code == GAP_GENERAL_DISCOVERY_PROC
                  && gap_evt_proc_complete->Status == 0x00)
              {
                /* USER CODE BEGIN GAP_GENERAL_DISCOVERY_PROC */
                BSP_LED_Off(LED_BLUE);
                /* USER CODE END GAP_GENERAL_DISCOVERY_PROC */
                APP_DBG_MSG("-- GAP GENERAL DISCOVERY PROCEDURE_COMPLETED\n\r");
                /*if a device found, connect to it, device 1 being chosen first if both found*/
                if (BleApplicationContext.DeviceServerFound == 0x01 && BleApplicationContext.Device_Connection_Status != APP_BLE_CONNECTED_CLIENT)
                {
                  UTIL_SEQ_SetTask(1 << CFG_TASK_CONN_DEV_1_ID, CFG_SCH_PRIO_0);
                }
              }
            }
            break;

#if (OOB_DEMO != 0)
          case ACI_L2CAP_CONNECTION_UPDATE_REQ_VSEVT_CODE:
            {
              /* USER CODE BEGIN EVT_BLUE_L2CAP_CONNECTION_UPDATE_REQ */

              /* USER CODE END EVT_BLUE_L2CAP_CONNECTION_UPDATE_REQ */
              aci_l2cap_connection_update_req_event_rp0 *pr = (aci_l2cap_connection_update_req_event_rp0 *) blecore_evt->data;
              ret = aci_hal_set_radio_activity_mask(0x0000);
              if (ret != BLE_STATUS_SUCCESS)
              {
                APP_DBG_MSG("  Fail   : aci_hal_set_radio_activity_mask command, result: 0x%x \n\r", ret);
              }
              else
              {
                APP_DBG_MSG("  Success: aci_hal_set_radio_activity_mask command\n\r");
              }

              APP_BLE_p2p_Conn_Update_req.Identifier = pr->Identifier;
              APP_BLE_p2p_Conn_Update_req.L2CAP_Length = pr->L2CAP_Length;
              APP_BLE_p2p_Conn_Update_req.Interval_Min = pr->Interval_Min;
              APP_BLE_p2p_Conn_Update_req.Interval_Max = pr->Interval_Max;
              APP_BLE_p2p_Conn_Update_req.Slave_Latency = pr->Slave_Latency;
              APP_BLE_p2p_Conn_Update_req.Timeout_Multiplier = pr->Timeout_Multiplier;

              ret = aci_l2cap_connection_parameter_update_resp(BleApplicationContext.BleApplicationContext_legacy.connectionHandle,
                                                               APP_BLE_p2p_Conn_Update_req.Interval_Min,
                                                               APP_BLE_p2p_Conn_Update_req.Interval_Max,
                                                               APP_BLE_p2p_Conn_Update_req.Slave_Latency,
                                                               APP_BLE_p2p_Conn_Update_req.Timeout_Multiplier,
                                                               CONN_L1,
                                                               CONN_L2,
                                                               APP_BLE_p2p_Conn_Update_req.Identifier,
                                                               0x01);
              if(ret != BLE_STATUS_SUCCESS)
              {
                APP_DBG_MSG("  Fail   : aci_l2cap_connection_parameter_update_resp command, result: 0x%x \n\r", ret);
                /* USER CODE BEGIN BLE_STATUS_SUCCESS */
                BSP_LED_On(LED_RED);
                /* USER CODE END BLE_STATUS_SUCCESS */
              }
              else
              {
                APP_DBG_MSG("  Success: aci_l2cap_connection_parameter_update_resp command\n\r");
              }

              ret = aci_hal_set_radio_activity_mask(0x0020);
              if (ret != BLE_STATUS_SUCCESS)
              {
                APP_DBG_MSG("  Fail   : aci_hal_set_radio_activity_mask command, result: 0x%x \n\r", ret);
              }
              else
              {
                APP_DBG_MSG("  Success: aci_hal_set_radio_activity_mask command\n\r");
              }
            }
            break;

          case 0x0004:
            {
              /* USER CODE BEGIN RADIO_ACTIVITY_EVENT */
              BSP_LED_On(LED_GREEN);
              HW_TS_Start(BleApplicationContext.SwitchOffGPIO_timer_Id, (uint32_t)LED_ON_TIMEOUT);
              /* USER CODE END RADIO_ACTIVITY_EVENT */
            }
            break;
#endif

          /* USER CODE BEGIN BLUE_EVT */
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
          
        case (ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE):
          APP_DBG_MSG(">>== ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE\n");
          APP_DBG_MSG("     - numeric_value = %ld\n",
                      ((aci_gap_numeric_comparison_value_event_rp0 *)(blecore_evt))->Numeric_Value);
          APP_DBG_MSG("     - Hex_value = %lx\n",
                      ((aci_gap_numeric_comparison_value_event_rp0 *)(blecore_evt->data))->Numeric_Value);
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
            pairing_complete = (aci_gap_pairing_complete_event_rp0*)blecore_evt->data;
            
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
          
        case (ACI_L2CAP_COC_CONNECT_CONFIRM_VSEVT_CODE):
        {
          coc_connect_confirm_event = (aci_l2cap_coc_connect_confirm_event_rp0*)blecore_evt->data;

          APP_DBG_MSG(">>== ACI_L2CAP_COC_CONNECT_CONFIRM_VSEVT_CODE\n");
          BleCoCContext.Conn_Handle = coc_connect_confirm_event->Connection_Handle;
          BleCoCContext.Max_Transmission_Unit = coc_connect_confirm_event->MTU;
          BleCoCContext.Max_Payload_Size = coc_connect_confirm_event->MPS;
          BleCoCContext.Initial_Credits = coc_connect_confirm_event->Initial_Credits;
          BleCoCContext.Channel_Number = coc_connect_confirm_event->Channel_Number;
          BleCoCContext.Channel_Index_List = coc_connect_confirm_event->Channel_Index_List[(BleCoCContext.Channel_Number)-1];
          
          APP_DBG_MSG("\r\n\r** COC CONNECTION EVENT WITH SERVER \n\r");
          BleCoCContext.Flag_COC_established = 1;
          handleNotification.COC_Evt_Opcode = EXCHANGE_ATT_MTU;
          handleNotification.ConnectionHandle = BleCoCContext.Conn_Handle;
          COC_APP_Notification(&handleNotification);
         
        }
        break;
        
        case (ACI_L2CAP_COC_RX_DATA_VSEVT_CODE):
        {
          coc_rx_data_event = (aci_l2cap_coc_rx_data_event_rp0*)blecore_evt->data;
          
          handleNotification.COC_Evt_Opcode = L2CAP_DATA_RECEIVED;
          handleNotification.ConnectionHandle = BleApplicationContext.BleApplicationContext_legacy.connectionHandle;
          handleNotification.DataLength = coc_rx_data_event->Length;
          COC_APP_Notification(&handleNotification);
          
          handleNotification.DataTransfered.pPayload = (coc_rx_data_event->Data);
          handleNotification.DataTransfered.pPayload_n = *((uint32_t*) &(coc_rx_data_event->Data[0]));
          __disable_irq();
          if (handleNotification.DataTransfered.pPayload_n >= (handleNotification.DataTransfered.pPayload_n_1 + 2))
          {
            packet_lost +=
              ((handleNotification.DataTransfered.pPayload_n - handleNotification.DataTransfered.pPayload_n_1) - 1);
          }
          __enable_irq();
          /* CRC computation */
          CRC_Result = APP_BLE_ComputeCRC8((uint8_t*) (coc_rx_data_event->Data), (coc_rx_data_event->Length) - 1);
          /* get low weight byte */
          CRC_Received = (uint8_t) (coc_rx_data_event->Data[DATA_MAX_PACKET_SIZE-1]);
          
          if (CRC_Received != CRC_Result)
          {
            Number_Corrupted_Data += 1;
            APP_DBG_MSG("** data error **  N= %d \r\n",(int)Number_Corrupted_Data);
          }
          handleNotification.DataTransfered.pPayload_n_1 = handleNotification.DataTransfered.pPayload_n;
          
        }
        break;
          /* USER CODE END BLUE_EVT */

          default:
            /* USER CODE BEGIN ecode_default */

            /* USER CODE END ecode_default */
            break;
        }
      }
      break;

    case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
      {
        /* USER CODE BEGIN EVT_DISCONN_COMPLETE */

        /* USER CODE END EVT_DISCONN_COMPLETE */
        if (cc->Connection_Handle == BleApplicationContext.BleApplicationContext_legacy.connectionHandle)
        {
          BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0;
          BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
          APP_DBG_MSG("\r\n\r** DISCONNECTION EVENT WITH SERVER \n\r");
          handleNotification.COC_Evt_Opcode = BLE_DISCON_HANDLE_EVT;
          handleNotification.ConnectionHandle = BleApplicationContext.BleApplicationContext_legacy.connectionHandle;
          COC_APP_Notification(&handleNotification);
        }
      }
      break; /* HCI_DISCONNECTION_COMPLETE_EVT_CODE */

    case HCI_LE_META_EVT_CODE:
      {
        /* USER CODE BEGIN EVT_LE_META_EVENT */

        /* USER CODE END EVT_LE_META_EVENT */
        meta_evt = (evt_le_meta_event*) event_pckt->data;

        switch (meta_evt->subevent)
        {
          /* USER CODE BEGIN subevent */

          /* USER CODE END subevent */
          case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
            /* USER CODE BEGIN EVT_LE_CONN_COMPLETE */

            /* USER CODE END EVT_LE_CONN_COMPLETE */
            /**
             * The connection is done,
             */
            connection_complete_event = (hci_le_connection_complete_event_rp0 *) meta_evt->data;
            BleApplicationContext.BleApplicationContext_legacy.connectionHandle = connection_complete_event->Connection_Handle;
            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_CLIENT;

            /* CONNECTION WITH CLIENT */
            APP_DBG_MSG("\r\n\r** CONNECTION EVENT WITH SERVER \n\r");
            handleNotification.COC_Evt_Opcode = BLE_CONN_HANDLE_EVT;
            handleNotification.ConnectionHandle = BleApplicationContext.BleApplicationContext_legacy.connectionHandle;
            COC_APP_Notification(&handleNotification);

            break; /* HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE */
            
        case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
            connection_update_complete_event = (hci_le_connection_update_complete_event_rp0 *) meta_evt->data;
            APP_DBG_MSG("\r\n\r** CONNECTION UPDATE EVENT WITH SERVER \n\r");
#if (CFG_DEBUG_APP_TRACE != 0)
            APP_DBG_MSG(">>== HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE\n");
            APP_DBG_MSG("     - Connection Interval:   %.2f ms\n     - Connection latency:    %d\n     - Supervision Timeout: %d ms\n\r",
                        connection_update_complete_event->Conn_Interval*1.25,
                        connection_update_complete_event->Conn_Latency,
                        connection_update_complete_event->Supervision_Timeout*10);
#endif /* CFG_DEBUG_APP_TRACE != 0 */
          
            handleNotification.COC_Evt_Opcode = BLE_CONN_UPDATE_EVT;
            handleNotification.ConnectionHandle = connection_update_complete_event->Connection_Handle;
            COC_APP_Notification(&handleNotification);
            
          break;
            
          case HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE:
            p_evt_le_phy_update_complete = (hci_le_phy_update_complete_event_rp0*)meta_evt->data;
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

          case HCI_LE_ADVERTISING_REPORT_SUBEVT_CODE:
            {
              uint8_t *adv_report_data;
              /* USER CODE BEGIN EVT_LE_ADVERTISING_REPORT */

              /* USER CODE END EVT_LE_ADVERTISING_REPORT */
              le_advertising_event = (hci_le_advertising_report_event_rp0 *) meta_evt->data;

              event_type = le_advertising_event->Advertising_Report[0].Event_Type;

              event_data_size = le_advertising_event->Advertising_Report[0].Length_Data;

              /* WARNING: be careful when decoding advertising report as its raw format cannot be mapped on a C structure.
              The data and RSSI values could not be directly decoded from the RAM using the data and RSSI field from hci_le_advertising_report_event_rp0 structure.
              Instead they must be read by using offsets (please refer to BLE specification).
              RSSI = (int8_t)*(uint8_t*) (adv_report_data + le_advertising_event->Advertising_Report[0].Length_Data);
              */
              adv_report_data = (uint8_t*)(&le_advertising_event->Advertising_Report[0].Length_Data) + 1;
              k = 0;

              /* search AD TYPE 0x09 (Complete Local Name) */
              /* search AD Type 0x02 (16 bits UUIDS) */
              if (event_type == ADV_IND)
              {
                /* ISOLATION OF BD ADDRESS AND LOCAL NAME */

                while(k < event_data_size)
                {
                  adlength = adv_report_data[k];
                  adtype = adv_report_data[k + 1];
                  switch (adtype)
                  {
                    case AD_TYPE_FLAGS: /* now get flags */
                      /* USER CODE BEGIN AD_TYPE_FLAGS */

                      /* USER CODE END AD_TYPE_FLAGS */
                      break;

                    case AD_TYPE_TX_POWER_LEVEL: /* Tx power level */
                      /* USER CODE BEGIN AD_TYPE_TX_POWER_LEVEL */

                      /* USER CODE END AD_TYPE_TX_POWER_LEVEL */
                      break;

                    case AD_TYPE_MANUFACTURER_SPECIFIC_DATA: /* Manufacturer Specific */
                      /* USER CODE BEGIN AD_TYPE_MANUFACTURER_SPECIFIC_DATA */

                      /* USER CODE END AD_TYPE_MANUFACTURER_SPECIFIC_DATA */
                      if (adlength >= 7 && adv_report_data[k + 2] == 0x01)
                      { /* ST VERSION ID 01 */
                        APP_DBG_MSG("--- ST MANUFACTURER ID --- \n\r");
                        switch (adv_report_data[k + 3])
                        {   /* Demo ID */
                          case CFG_DEV_ID_BLE_COC: /* End Device 1 */
                            APP_DBG_MSG("-- SERVER DETECTED -- VIA MAN ID\n\r");
                            BleApplicationContext.DeviceServerFound = 0x01;
                            SERVER_REMOTE_BDADDR[0] = le_advertising_event->Advertising_Report[0].Address[0];
                            SERVER_REMOTE_BDADDR[1] = le_advertising_event->Advertising_Report[0].Address[1];
                            SERVER_REMOTE_BDADDR[2] = le_advertising_event->Advertising_Report[0].Address[2];
                            SERVER_REMOTE_BDADDR[3] = le_advertising_event->Advertising_Report[0].Address[3];
                            SERVER_REMOTE_BDADDR[4] = le_advertising_event->Advertising_Report[0].Address[4];
                            SERVER_REMOTE_BDADDR[5] = le_advertising_event->Advertising_Report[0].Address[5];
                            
                            /* The device has been found - scan may be stopped */
                            aci_gap_terminate_gap_proc(GAP_GENERAL_DISCOVERY_PROC);
                            break;

                          default:
                            break;
                        }
                      }
                      if (adlength >= 7 && adv_report_data[k + 4] == 0x02)
                      { /* ST VERSION ID 02 */
                        APP_DBG_MSG("--- ST MANUFACTURER ID --- \n");
                        switch (adv_report_data[k + 6])
                        {   /* Demo ID */
                        case CFG_DEV_ID_BLE_COC:   /* (Periph Server DT) */
                          {
                            APP_DBG_MSG("-- SERVER DETECTED -- VIA MAN ID\n");
                            BleApplicationContext.DeviceServerFound = 0x01;
                            SERVER_REMOTE_BDADDR[0] = le_advertising_event->Advertising_Report[0].Address[0];
                            SERVER_REMOTE_BDADDR[1] = le_advertising_event->Advertising_Report[0].Address[1];
                            SERVER_REMOTE_BDADDR[2] = le_advertising_event->Advertising_Report[0].Address[2];
                            SERVER_REMOTE_BDADDR[3] = le_advertising_event->Advertising_Report[0].Address[3];
                            SERVER_REMOTE_BDADDR[4] = le_advertising_event->Advertising_Report[0].Address[4];
                            SERVER_REMOTE_BDADDR[5] = le_advertising_event->Advertising_Report[0].Address[5];
                            
                            /* The device has been found - scan may be stopped */
                            aci_gap_terminate_gap_proc(GAP_GENERAL_DISCOVERY_PROC);
                          }
                          break;
                          
                        default:
                          break;
                        }
                      }
                      
                      if ((adlength >= 7) && (adv_report_data[k + 4] == 0x02))
                      {
                        APP_DBG_MSG("--- ST MANUFACTURER ID SDKv2---\n");                        
                        switch (adv_report_data[k + 7])
                        {
                        case CFG_DEV_ID_BLE_COC:
                          SERVER_REMOTE_BDADDR[0] = le_advertising_event->Advertising_Report[0].Address[0];
                          SERVER_REMOTE_BDADDR[1] = le_advertising_event->Advertising_Report[0].Address[1];
                          SERVER_REMOTE_BDADDR[2] = le_advertising_event->Advertising_Report[0].Address[2];
                          SERVER_REMOTE_BDADDR[3] = le_advertising_event->Advertising_Report[0].Address[3];
                          SERVER_REMOTE_BDADDR[4] = le_advertising_event->Advertising_Report[0].Address[4];
                          SERVER_REMOTE_BDADDR[5] = le_advertising_event->Advertising_Report[0].Address[5];
                          
                          BleApplicationContext.DeviceServerFound = 0x01;
                          //found_status = 1;
                          APP_DBG_MSG("-- SERVER DETECTED -- VIA MAN ID\n");
                          /* The device has been found - scan may be stopped */
                          aci_gap_terminate_gap_proc(GAP_GENERAL_DISCOVERY_PROC);
                          break;
                        default:
                          //found_status = 0;
                          APP_DBG_MSG("-- SERVER NOT DETECTED ...\n\r");
                          break;
                        }
                      }
                      break;

                    case AD_TYPE_SERVICE_DATA: /* service data 16 bits */
                      /* USER CODE BEGIN AD_TYPE_SERVICE_DATA */

                      /* USER CODE END AD_TYPE_SERVICE_DATA */
                      break;

                    default:
                      /* USER CODE BEGIN adtype_default */

                      /* USER CODE END adtype_default */
                      break;
                  } /* end switch adtype */
                  k += adlength + 1;
                } /* end while */
              } /* end if ADV_IND */
            }
            break;

          /* USER CODE BEGIN META_EVT */

          /* USER CODE END META_EVT */

          default:
            /* USER CODE BEGIN subevent_default */

            /* USER CODE END subevent_default */
            break;
        }
      }
      break; /* HCI_LE_META_EVT_CODE */

    /* USER CODE BEGIN EVENT_PCKT */

    /* USER CODE END EVENT_PCKT */

    default:
      /* USER CODE BEGIN evt_default */

      /* USER CODE END evt_default */
      break;
  }

  return (SVCCTL_UserEvtFlowEnable);
}

APP_BLE_ConnStatus_t APP_BLE_Get_Client_Connection_Status(uint16_t Connection_Handle)
{
  if (BleApplicationContext.BleApplicationContext_legacy.connectionHandle == Connection_Handle)
  {
    return BleApplicationContext.Device_Connection_Status;
  }
  return APP_BLE_IDLE;
}
/* USER CODE BEGIN FD */
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

/* USER CODE END FD */
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
  const uint8_t *bd_addr;
  uint32_t srd_bd_addr[2];
  uint16_t appearance[1] = { BLE_CFG_GAP_APPEARANCE };
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  APP_DBG_MSG("Start Ble_Hci_Gap_Gatt_Init function\n\r");

  /**
   * Initialize HCI layer
   */
  /*HCI Reset to synchronise BLE Stack*/
  ret = hci_reset();
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : hci_reset command, result: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: hci_reset command\n\r");
  }

  /**
   * Write the BD Address
   */
  bd_addr = BleGetBdAddress();
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  (uint8_t*) bd_addr);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command, result: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command\n\r");
  }

  /**
   * Static random Address
   * The two upper bits shall be set to 1
   * The lowest 32bits is read from the UDN to differentiate between devices
   * The RNG may be used to provide a random number on each power on
   */
  srd_bd_addr[1] =  0x0000ED6E;
  srd_bd_addr[0] =  LL_FLASH_GetUDN();
  ret = aci_hal_write_config_data(CONFIG_DATA_RANDOM_ADDRESS_OFFSET, CONFIG_DATA_RANDOM_ADDRESS_LEN, (uint8_t*)srd_bd_addr);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command, result: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command\n\r");
  }

  /**
   * Write Identity root key used to derive LTK and CSRK
   */
  ret = aci_hal_write_config_data(CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, (uint8_t*)BLE_CFG_IR_VALUE);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command, result: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command\n\r");
  }

  /**
   * Write Encryption root key used to derive LTK and CSRK
   */
  ret = aci_hal_write_config_data(CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, (uint8_t*)BLE_CFG_ER_VALUE);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command, result: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command\n\r");
  }

  /**
   * Set TX Power to 0dBm.
   */
  ret = aci_hal_set_tx_power_level(1, CFG_TX_POWER);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_set_tx_power_level command, result: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_set_tx_power_level command\n\r");
  }

  /**
   * Initialize GATT interface
   */
  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_init command, result: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_init command\n\r");
  }

  /**
   * Initialize GAP interface
   */
  role = 0;

#if (BLE_CFG_CENTRAL == 1)
  role |= GAP_CENTRAL_ROLE;
#endif

  if (role > 0)
  {
    const char *name = "COC_C";

    ret = aci_gap_init(role,
                       0,
                       APPBLE_GAP_DEVICE_NAME_LENGTH,
                       &gap_service_handle,
                       &gap_dev_name_char_handle,
                       &gap_appearance_char_handle);
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gap_init command, result: 0x%x \n\r", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gap_init command\n\r");
    }

    if (aci_gatt_update_char_value(gap_service_handle, gap_dev_name_char_handle, 0, strlen(name), (uint8_t *) name))
    {
      BLE_DBG_SVCCTL_MSG("Device Name aci_gatt_update_char_value failed.\n\r");
    }
  }

  if(aci_gatt_update_char_value(gap_service_handle,
                                gap_appearance_char_handle,
                                0,
                                2,
                                (uint8_t *)&appearance))
  {
    BLE_DBG_SVCCTL_MSG("Appearance aci_gatt_update_char_value failed.\n\r");
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
    APP_DBG_MSG("  Fail   : aci_gap_set_io_capability command, result: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gap_set_io_capability command\n\r");
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

  ret = aci_gap_set_authentication_requirement(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
                                               CFG_SC_SUPPORT,
                                               CFG_KEYPRESS_NOTIFICATION_SUPPORT,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin,
                                               PUBLIC_ADDR
                                              );
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gap_set_authentication_requirement command, result: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gap_set_authentication_requirement command\n\r");
  }

  /**
   * Initialize whitelist
   */
  if (BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode)
  {
    ret = aci_gap_configure_whitelist();
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gap_configure_whitelist command, result: 0x%x \n\r", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gap_configure_whitelist command\n\r");
    }
  }
}

static void Scan_Request(void)
{
  /* USER CODE BEGIN Scan_Request_1 */

  /* USER CODE END Scan_Request_1 */
  tBleStatus result;
  if (BleApplicationContext.Device_Connection_Status != APP_BLE_CONNECTED_CLIENT)
  {
    /* USER CODE BEGIN APP_BLE_CONNECTED_CLIENT */
    BSP_LED_On(LED_BLUE);
    /* USER CODE END APP_BLE_CONNECTED_CLIENT */
    result = aci_gap_start_general_discovery_proc(SCAN_P, SCAN_L, PUBLIC_ADDR, 1);
    if (result == BLE_STATUS_SUCCESS)
    {
    /* USER CODE BEGIN BLE_SCAN_SUCCESS */

    /* USER CODE END BLE_SCAN_SUCCESS */
      APP_DBG_MSG(" \r\n\r** START GENERAL DISCOVERY (SCAN) **  \r\n\r");
    }
    else
    {
    /* USER CODE BEGIN BLE_SCAN_FAILED */
      BSP_LED_On(LED_RED);
    /* USER CODE END BLE_SCAN_FAILED */
      APP_DBG_MSG("-- BLE_App_Start_Limited_Disc_Req, Failed \r\n\r");
    }
  }
  /* USER CODE BEGIN Scan_Request_2 */

  /* USER CODE END Scan_Request_2 */
  return;
}

static void Connect_Request(void)
{
  /* USER CODE BEGIN Connect_Request_1 */

  /* USER CODE END Connect_Request_1 */
  tBleStatus result;

  APP_DBG_MSG("\r\n\r** CREATE CONNECTION TO SERVER **  \r\n\r");

  if (BleApplicationContext.Device_Connection_Status != APP_BLE_CONNECTED_CLIENT)
  {
    result = aci_gap_create_connection(SCAN_P,
                                       SCAN_L,
                                       PUBLIC_ADDR, SERVER_REMOTE_BDADDR,
                                       PUBLIC_ADDR,
                                       CONN_P1,
                                       CONN_P2,
                                       0,
                                       SUPERV_TIMEOUT,
                                       0,
                                       0x280);

    if (result == BLE_STATUS_SUCCESS)
    {
      /* USER CODE BEGIN BLE_CONNECT_SUCCESS */

      /* USER CODE END BLE_CONNECT_SUCCESS */
      BleApplicationContext.Device_Connection_Status = APP_BLE_LP_CONNECTING;

    }
    else
    {
      /* USER CODE BEGIN BLE_CONNECT_FAILED */
      BSP_LED_On(LED_RED);
      /* USER CODE END BLE_CONNECT_FAILED */
      BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;

    }
  }
  /* USER CODE BEGIN Connect_Request_2 */

  /* USER CODE END Connect_Request_2 */
  return;
}

static void Switch_OFF_GPIO()
{
  /* USER CODE BEGIN Switch_OFF_GPIO */
  BSP_LED_Off(LED_GREEN);
  /* USER CODE END Switch_OFF_GPIO */
}

const uint8_t* BleGetBdAddress(void)
{
  uint8_t *otp_addr;
  const uint8_t *bd_addr;
  uint32_t udn;
  uint32_t company_id;
  uint32_t device_id;

  udn = LL_FLASH_GetUDN();

  if(udn != 0xFFFFFFFF)
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
   bd_addr_udn[0] = (uint8_t)(udn & 0x000000FF);
   bd_addr_udn[1] = (uint8_t)((udn & 0x0000FF00) >> 8);
   bd_addr_udn[2] = (uint8_t)device_id;
   bd_addr_udn[3] = (uint8_t)(company_id & 0x000000FF);
   bd_addr_udn[4] = (uint8_t)((company_id & 0x0000FF00) >> 8);
   bd_addr_udn[5] = (uint8_t)((company_id & 0x00FF0000) >> 16);

   bd_addr = (const uint8_t *)bd_addr_udn;
  }
  else
  {
    otp_addr = OTP_Read(0);
    if(otp_addr)
    {
      bd_addr = ((OTP_ID0_t*)otp_addr)->bd_address;
    }
    else
    {
      bd_addr = M_bd_addr;
    }
  }

  return bd_addr;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */


static void Resume_DataSend(void)
{
  CoC_Central_App_Context.CoCFlowStatus = COC_APP_FLOW_ON;
  UTIL_SEQ_SetTask(1 << CFG_TASK_DATA_TRANSFER_ID, CFG_SCH_PRIO_0);
}

static void L2cap_CoC_Throughput_proc(){
  
  UTIL_SEQ_SetTask(1 << CFG_TASK_DATA_WRITE_ID, CFG_SCH_PRIO_0);
}
/* USER CODE END FD_LOCAL_FUNCTIONS */

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/
void hci_notify_asynch_evt(void* pdata)
{
  UTIL_SEQ_SetTask(1 << CFG_TASK_HCI_ASYNCH_EVT_ID, CFG_SCH_PRIO_0);
  return;
}

void hci_cmd_resp_release(uint32_t flag)
{
  UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);
  return;
}

void hci_cmd_resp_wait(uint32_t timeout)
{
  UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID);
  return;
}

static void BLE_UserEvtRx(void * pPayload)
{
  SVCCTL_UserEvtFlowStatus_t svctl_return_status;
  tHCI_UserEvtRxParam *pParam;

  pParam = (tHCI_UserEvtRxParam *)pPayload;

  svctl_return_status = SVCCTL_UserEvtRx((void *)&(pParam->pckt->evtserial));
  if (svctl_return_status != SVCCTL_UserEvtFlowDisable)
  {
    pParam->status = HCI_TL_UserEventFlow_Enable;
  }
  else
  {
    pParam->status = HCI_TL_UserEventFlow_Disable;
  }

  return;
}

static void BLE_StatusNot(HCI_TL_CmdStatus_t status)
{
  uint32_t task_id_list;
  switch (status)
  {
    case HCI_TL_CmdBusy:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_PauseTask(task_id_list);
      break;

    case HCI_TL_CmdAvailable:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_ResumeTask(task_id_list);
      break;

    default:
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
void BLE_GAP_Pairing_Req(void)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  
  ret = aci_gap_send_pairing_req(BleApplicationContext.BleApplicationContext_legacy.connectionHandle, FORCE_REBOND);
  if (ret == BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("pairing req cmd OK ok\n");
  }
  else 
  {
    APP_DBG_MSG("pairing req cmd NOK\n");
  }  
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
