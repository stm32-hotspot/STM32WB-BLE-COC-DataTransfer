/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_ble.h
  * @author  MCD Application Team
  * @brief   Header for ble application
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_BLE_H
#define APP_BLE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hci_tl.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/

typedef enum
{
  APP_BLE_IDLE,
  APP_BLE_FAST_ADV,
  APP_BLE_LP_ADV,
  APP_BLE_SCAN,
  APP_BLE_LP_CONNECTING,
  APP_BLE_CONNECTED_SERVER,
  APP_BLE_CONNECTED_CLIENT
} APP_BLE_ConnStatus_t;

/* USER CODE BEGIN ET */
typedef struct
{
  uint16_t Conn_Handle;
  uint16_t Max_Transmission_Unit;
  uint16_t Max_Payload_Size;
  uint16_t Initial_Credits;
  uint16_t SPSM;
  uint8_t Channel_Number;
  uint8_t Channel_Index_List; 
}BleCoCContext_t;

typedef enum
{
  COC_APP_FLOW_OFF,
  COC_APP_FLOW_ON
} CoC_App_Flow_Status_t;
 
typedef enum
{
  COC_APP_TRANSFER_REQ_OFF,
  COC_APP_TRANSFER_REQ_ON
} CoC_App_Transfer_Req_Status_t;

typedef struct
{
  CoC_App_Transfer_Req_Status_t NotificationTransferReq;
  CoC_App_Transfer_Req_Status_t ButtonTransferReq;
  CoC_App_Flow_Status_t CoCFlowStatus;
} CoC_App_Context_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern BleCoCContext_t BleCoCContext;  
extern CoC_App_Context_t CoC_App_Context;
/* USER CODE END EV */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ---------------------------------------------*/
void APP_BLE_Init(void);

APP_BLE_ConnStatus_t APP_BLE_Get_Server_Connection_Status(void);

/* USER CODE BEGIN EF */
void APP_BLE_Key_Button1_Action(void);
void APP_BLE_Key_Button2_Action(void);
void APP_BLE_Key_Button3_Action(void);
void BLE_SVC_GAP_Security_Req(void);
void BLE_SVC_GAP_Clear_DataBase(void);
uint8_t APP_BLE_ComputeCRC8( uint8_t *DataPtr , uint8_t Datalen );
/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*APP_BLE_H */
