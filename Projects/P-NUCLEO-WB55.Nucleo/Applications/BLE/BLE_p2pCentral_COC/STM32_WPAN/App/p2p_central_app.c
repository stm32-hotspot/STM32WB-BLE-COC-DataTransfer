/**
  ******************************************************************************
  * @file    p2p_central_app.c
  * @author  MCD Application Team
  * @brief   peer to peer central COC Application
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

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "app_common.h"

#include "dbg_trace.h"

#include "ble.h"
#include "p2p_central_app.h"

#include "stm32_seq.h"
#include "app_ble.h"

/* Private typedef -----------------------------------------------------------*/

typedef enum
{
  COC_START_TIMER_EVT,
  COC_STOP_TIMER_EVT,
  COC_NOTIFICATION_INFO_RECEIVED_EVT,
} COC_Client_Opcode_Notification_evt_t;

typedef struct
{
  uint8_t * pPayload;
  uint8_t     Length;
}COC_Client_Data_t;

typedef struct
{
  COC_Client_Opcode_Notification_evt_t  COC_Client_Evt_Opcode;
  COC_Client_Data_t DataTransfered;
}COC_Client_App_Notification_evt_t;

/* Private defines ------------------------------------------------------------*/

/* Private macros -------------------------------------------------------------*/
#define UNPACK_2_BYTE_PARAMETER(ptr)  \
        (uint16_t)((uint16_t)(*((uint8_t *)ptr))) |   \
        (uint16_t)((((uint16_t)(*((uint8_t *)ptr + 1))) << 8))

/* Private variables ---------------------------------------------------------*/
#define DEFAULT_TS_MEASUREMENT_INTERVAL   (1000000/CFG_TS_TICK_VAL)  /**< 1s */
#define DELAY_1s  (1*DEFAULT_TS_MEASUREMENT_INTERVAL)
#define TIMEUNIT  1
  
#define BOUNCE_THRESHOLD                                                   20U
#define LONG_PRESS_THRESHOLD                                               1000U
          
/**
 * START of Section BLE_APP_CONTEXT
 */

/**
 * END of Section BLE_APP_CONTEXT
 */

/* Private function prototypes -----------------------------------------------*/
static void SendData( void );
static void BLE_App_Central_CoC_Throughput(void);
static void L2CAP_Change_PHY(void);
static void COC_Button1TriggerReceived( void );
static void COC_Button2TriggerReceived(void);
static void COC_App_Button1_Trigger_Received(void);
static void COC_App_Button2_Trigger_Received(void);
static void COC_App_Button3_Trigger_Received(void);

BleCoCContext_t BleCoCContext;

uint8_t Tab_Data[DATA_MAX_PACKET_SIZE];
uint32_t DataReceived;
extern uint8_t TimerL2capDataThroughputWrite_Id;

/* Functions Definition ------------------------------------------------------*/
/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void COC_APP_Init(void)
{
  uint16_t i;
  
  UTIL_SEQ_RegTask( 1<<CFG_TASK_SW1_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, COC_App_Button1_Trigger_Received);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_SW2_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, COC_App_Button2_Trigger_Received);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_SW3_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, COC_App_Button3_Trigger_Received);
  
  UTIL_SEQ_RegTask( 1<<CFG_TASK_DATA_TRANSFER_ID, UTIL_SEQ_RFU, SendData);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_DATA_PHY_UPDATE_ID, UTIL_SEQ_RFU, L2CAP_Change_PHY);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_DATA_WRITE_ID, UTIL_SEQ_RFU, BLE_App_Central_CoC_Throughput);
  
  CoC_Central_App_Context.NotificationTransferReq = COC_APP_TRANSFER_REQ_OFF;
  CoC_Central_App_Context.ButtonTransferReq = COC_APP_TRANSFER_REQ_OFF;
  CoC_Central_App_Context.CoCFlowStatus = COC_APP_FLOW_ON;
  BleCoCContext.Flag_COC_established = 0;
  
  for (i=0;i<sizeof(Tab_Data);i++)
  {
    Tab_Data[i] = i;
  }
}

void COC_App_KeyButton1Action( void )
{
  UTIL_SEQ_SetTask(1 << CFG_TASK_SW1_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);
}

void COC_App_KeyButton2Action( void )
{
  UTIL_SEQ_SetTask(1 << CFG_TASK_SW2_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);
}

void COC_App_KeyButton3Action( void )
{
  UTIL_SEQ_SetTask(1 << CFG_TASK_SW3_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);
}

static void Appli_UpdateButtonState(Button_TypeDef button, int isPressed)
{
  uint32_t t0 = 0,t1 = 1;

  t0 = HAL_GetTick(); /* SW3 press timing */
  
  while(BSP_PB_GetState(button) == BUTTON_PRESSED);
  t1 = HAL_GetTick(); /* SW3 release timing */
  
  if((t1 - t0) > LONG_PRESS_THRESHOLD)
  {
    if(button == BUTTON_SW1)
    {
      /* Button 1 long press action */
      COC_Button1TriggerReceived();
    }
    else if(button == BUTTON_SW2)
    {
      /* Button 2 long press action */
      COC_Button2TriggerReceived();
    }
    else if(button == BUTTON_SW3)
    {
      /* Button 3 long press action */
      APP_DBG_MSG("clear database \n");
      BLE_SVC_GAP_Clear_DataBase();
    }
  }
  else if((t1 - t0) > BOUNCE_THRESHOLD)
  {
    if(button == BUTTON_SW1)
    {
      /* Button 1 short press action */
      COC_Button1TriggerReceived();
    }
    else if(button == BUTTON_SW2)
    {
      /* Button 2 short press action */
      COC_Button2TriggerReceived();
    }
    else if(button == BUTTON_SW3)
    {
      /* Button 3 short press action */
      APP_DBG_MSG("pairing request \n");
      BLE_GAP_Pairing_Req();
    }
  }
}

static void COC_App_Button1_Trigger_Received(void)
{
  Appli_UpdateButtonState(BUTTON_SW1, BSP_PB_GetState(BUTTON_SW1) == BUTTON_PRESSED);
  
  return;
}

static void COC_App_Button2_Trigger_Received(void)
{
  Appli_UpdateButtonState(BUTTON_SW2, BSP_PB_GetState(BUTTON_SW2) == BUTTON_PRESSED);
  
  return;
}

static void COC_App_Button3_Trigger_Received(void)
{
  Appli_UpdateButtonState(BUTTON_SW3, BSP_PB_GetState(BUTTON_SW3) == BUTTON_PRESSED);
  
  return;
}

void COC_APP_Notification(COC_APP_ConnHandle_Not_evt_t *pNotification)
{
tBleStatus        ret = BLE_STATUS_INVALID_PARAMS;

  switch(pNotification->COC_Evt_Opcode)
  {
  case EXCHANGE_ATT_MTU:
    
    ret = aci_gatt_exchange_config(BleCoCContext.Conn_Handle);
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail: change MTU: 0x%x\n", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: change MTU\n");
    }
    break;
    
  case BLE_CONN_HANDLE_EVT :    
    BleCoCContext.Conn_Handle = pNotification->ConnectionHandle;;
    
    break;
  
  case BLE_CONN_UPDATE_EVT:
    if (BleCoCContext.Flag_COC_established != 1)
    {
      BleCoCContext.SPSM = 0x0002;
      BleCoCContext.Max_Transmission_Unit = 0x0023;
      BleCoCContext.Max_Payload_Size = 0x0023;
      BleCoCContext.Initial_Credits = 0x0003;
      BleCoCContext.Channel_Number = 0x01;
      
      ret = aci_l2cap_coc_connect(BleCoCContext.Conn_Handle, 
                                  BleCoCContext.SPSM, 
                                  BleCoCContext.Max_Transmission_Unit, 
                                  BleCoCContext.Max_Payload_Size, 
                                  BleCoCContext.Initial_Credits, 
                                  BleCoCContext.Channel_Number);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_l2cap_coc_connect command, result: 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_l2cap_coc_connect command\n\r");
      }
    }
    break;

    case BLE_DISCON_HANDLE_EVT :
      {
      BSP_LED_Off(LED_BLUE); 
        
#if OOB_DEMO == 0
      UTIL_SEQ_SetTask(1<<CFG_TASK_CONN_DEV_1_ID, CFG_SCH_PRIO_0);
#endif 
      }
      break;
      
    case L2CAP_DATA_RECEIVED:
    {
      if (DataReceived == 0)
      {
        /* start timer */
        DataReceived += pNotification->DataLength;
        HW_TS_Start(TimerL2capDataThroughputWrite_Id, DELAY_1s);
      }
      else
      {
        DataReceived += pNotification->DataLength;
      }
    }
    break;

    default:

      break;
  }
  return;
}

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

static void COC_Button1TriggerReceived( void )
{
  if(CoC_Central_App_Context.ButtonTransferReq != COC_APP_TRANSFER_REQ_OFF)
  {
    BSP_LED_Off(LED_BLUE);
    CoC_Central_App_Context.ButtonTransferReq = COC_APP_TRANSFER_REQ_OFF;
  }
  else
  {
    BSP_LED_On(LED_BLUE);
    CoC_Central_App_Context.ButtonTransferReq = COC_APP_TRANSFER_REQ_ON;
    UTIL_SEQ_SetTask(1 << CFG_TASK_DATA_TRANSFER_ID, CFG_SCH_PRIO_0);
  }

  return;
}

static void COC_Button2TriggerReceived( void )
{
  APP_DBG_MSG("**CHANGE PHY \n");
  UTIL_SEQ_SetTask(1 << CFG_TASK_DATA_PHY_UPDATE_ID, CFG_SCH_PRIO_0);
  return;
}

static void L2CAP_Change_PHY(void)
{
  uint8_t TX_PHY, RX_PHY;
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  ret = hci_le_read_phy(BleCoCContext.Conn_Handle,
                        &TX_PHY,
                        &RX_PHY);
  if (ret == BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("READ PHY : ");
    APP_DBG_MSG("PHY Param  TX= %d, RX= %d \n", TX_PHY, RX_PHY);
    if ((TX_PHY == TX_2M) && (RX_PHY == RX_2M))
    {
      APP_DBG_MSG("**TX= %d, **RX= %d \n", TX_1M, RX_1M);
      ret = hci_le_set_phy(BleCoCContext.Conn_Handle,
                           ALL_PHYS_PREFERENCE,
                           TX_1M,
                           RX_1M,
                           0);
      if(ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("hci_le_set_phy failed 0x%x\n", ret);
      }
    }
    else
    {
      APP_DBG_MSG("**TX= %d, **RX= %d \n", TX_2M_PREFERRED, RX_2M_PREFERRED);
      ret = hci_le_set_phy(BleCoCContext.Conn_Handle,
                           ALL_PHYS_PREFERENCE,
                           TX_2M,
                           RX_2M,
                           0);
      if(ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("hci_le_set_phy failed 0x%x\n", ret);
      }
    } 
  }
  else
  {
    APP_DBG_MSG("Read conf not succeess 0x%x\n", ret);
  }
  
  return;
}

static void SendData( void )
{
  tBleStatus status = BLE_STATUS_INVALID_PARAMS;
  uint8_t crc_result;

  if ((CoC_Central_App_Context.ButtonTransferReq != COC_APP_TRANSFER_REQ_OFF) &&(CoC_Central_App_Context.CoCFlowStatus != COC_APP_FLOW_OFF))
  {   
    /*Data Packet to send to remote*/
    Tab_Data[0] += 1;
    /* compute CRC */
    crc_result = APP_BLE_ComputeCRC8((uint8_t*) Tab_Data, (DATA_MAX_PACKET_SIZE - 1));
    Tab_Data[DATA_MAX_PACKET_SIZE - 1] = crc_result;

    status = aci_l2cap_coc_tx_data(BleCoCContext.Channel_Index_List, 
                                   sizeof(Tab_Data), 
                                   Tab_Data);
    if (status == BLE_STATUS_INSUFFICIENT_RESOURCES)
    {
      CoC_Central_App_Context.CoCFlowStatus = COC_APP_FLOW_OFF;
      (Tab_Data[0])-=1;
    }
    else if (status == BLE_STATUS_SUCCESS)
    {
      UTIL_SEQ_SetTask(1 << CFG_TASK_DATA_TRANSFER_ID, CFG_SCH_PRIO_0);
    }
  }
  return;
}

static void BLE_App_Central_CoC_Throughput(void)
{
  uint32_t L2cap_Central_CoC_Throughput;
  
  L2cap_Central_CoC_Throughput = (uint32_t)(DataReceived/TIMEUNIT);
  APP_DBG_MSG("L2cap_Central_CoC_Throughput = %ld  bytes/s\n", L2cap_Central_CoC_Throughput);
  
  DataReceived = 0;
}
