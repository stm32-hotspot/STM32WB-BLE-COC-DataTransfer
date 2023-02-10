/**
  ******************************************************************************
  * @file    p2p_peripheral_app.c
  * @author  MCD Application Team
  * @brief   BLE peripheral COC Application
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
#include "p2p_peripheral_app.h"
#include "app_ble.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private defines ------------------------------------------------------------*/

/* Private macros -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
#define DEFAULT_TS_MEASUREMENT_INTERVAL   (1000000/CFG_TS_TICK_VAL)  /**< 1s */
#define DELAY_1s  (1*DEFAULT_TS_MEASUREMENT_INTERVAL)
#define TIMEUNIT  1

#define BOUNCE_THRESHOLD                                                   20U
#define LONG_PRESS_THRESHOLD                                               1000U
/**
 * START of Section BLE_APP_CONTEXT
 */

uint8_t Tab_Data[DATA_MAX_PACKET_SIZE];
uint32_t DataReceived;
extern uint8_t TimerL2capDataThroughputWrite_Id;
/**
 * END of Section BLE_APP_CONTEXT
 */

/* Private function prototypes -----------------------------------------------*/

static void SendData( void );
static void BLE_App_Periph_CoC_Throughput(void);

static void COC_App_Button1_Trigger_Received(void);
static void COC_App_Button2_Trigger_Received(void);
static void COC_App_Button3_Trigger_Received(void);
static void COC_Button1TriggerReceived( void );
static void COC_Button2TriggerReceived( void );

/* Functions Definition ------------------------------------------------------*/

void COC_APP_Notification(COC_APP_ConnHandle_Not_evt_t *pNotification)
{
  switch(pNotification->CoC_Evt_Opcode)
  {
    case BLE_CONN_HANDLE_EVT :
    UTIL_SEQ_SetTask( 1<<CFG_TASK_CONN_UPDATE_REG_ID, CFG_SCH_PRIO_0); 
    break;

    case BLE_DISCON_HANDLE_EVT :

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

void COC_APP_Init(void)
{
  uint16_t i;
  

  UTIL_SEQ_RegTask( 1<<CFG_TASK_SW1_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, COC_App_Button1_Trigger_Received);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_SW2_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, COC_App_Button2_Trigger_Received);
  UTIL_SEQ_RegTask( 1<<CFG_TASK_SW3_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, COC_App_Button3_Trigger_Received);

  UTIL_SEQ_RegTask( 1<<CFG_TASK_DATA_TRANSFER_ID, UTIL_SEQ_RFU, SendData); 
  UTIL_SEQ_RegTask( 1<<CFG_TASK_DATA_WRITE_ID, UTIL_SEQ_RFU, BLE_App_Periph_CoC_Throughput);
  
  CoC_App_Context.NotificationTransferReq = COC_APP_TRANSFER_REQ_OFF;
  CoC_App_Context.ButtonTransferReq = COC_APP_TRANSFER_REQ_OFF;
  CoC_App_Context.CoCFlowStatus = COC_APP_FLOW_ON;
  
  for (i=0;i<sizeof(Tab_Data);i++)
  {
    Tab_Data[i] = i;
  }

  return;
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
      APP_DBG_MSG("peripheral security request \n");
      BLE_SVC_GAP_Security_Req();
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


/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

static void COC_Button1TriggerReceived( void )
{
  if(CoC_App_Context.ButtonTransferReq != COC_APP_TRANSFER_REQ_OFF)
  {
    BSP_LED_Off(LED_BLUE);
    CoC_App_Context.ButtonTransferReq = COC_APP_TRANSFER_REQ_OFF;
  }
  else
  {
    BSP_LED_On(LED_BLUE);
    CoC_App_Context.ButtonTransferReq = COC_APP_TRANSFER_REQ_ON;
    UTIL_SEQ_SetTask(1 << CFG_TASK_DATA_TRANSFER_ID, CFG_SCH_PRIO_0);
  }

  return;
}

static void COC_Button2TriggerReceived( void )
{
  UTIL_SEQ_SetTask(1 << CFG_TASK_TOGGLE_CONN_INT_ID, CFG_SCH_PRIO_0);
  return;
}

static void SendData( void )
{
  tBleStatus status = BLE_STATUS_INVALID_PARAMS;
  uint8_t crc_result;

  if ((CoC_App_Context.ButtonTransferReq != COC_APP_TRANSFER_REQ_OFF) &&(CoC_App_Context.CoCFlowStatus != COC_APP_FLOW_OFF))
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
      CoC_App_Context.CoCFlowStatus = COC_APP_FLOW_OFF;
      (Tab_Data[0])-=1;
    }
    else if (status == BLE_STATUS_SUCCESS)
    {
      UTIL_SEQ_SetTask(1 << CFG_TASK_DATA_TRANSFER_ID, CFG_SCH_PRIO_0);
    }
  }
  return;
}

static void BLE_App_Periph_CoC_Throughput(void)
{
  uint32_t L2cap_Periph_CoC_Throughput;
  
  L2cap_Periph_CoC_Throughput = (uint32_t)(DataReceived/TIMEUNIT);
  APP_DBG_MSG("L2cap_Periph_CoC_Throughput = %ld  bytes/s\n", L2cap_Periph_CoC_Throughput);
  
  DataReceived = 0;
}
