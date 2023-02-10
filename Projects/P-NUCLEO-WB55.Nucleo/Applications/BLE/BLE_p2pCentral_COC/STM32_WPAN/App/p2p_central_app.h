/**
  ******************************************************************************
  * @file    p2p_central_app.h
  * @author  MCD Application Team
  * @brief   Header for p2p_central_app.c module
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
#ifndef COC_APPLICATION_H
#define COC_APPLICATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#define BUTTON_PRESSED                                            GPIO_PIN_RESET
/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  BLE_CONN_HANDLE_EVT,
  BLE_CONN_UPDATE_EVT,
  BLE_DISCON_HANDLE_EVT,
  EXCHANGE_ATT_MTU,
  L2CAP_DATA_RECEIVED,
} COC_APP_Opcode_Notification_evt_t;

typedef struct
{
  uint8_t *pPayload;
  uint32_t pPayload_n_1;
  uint32_t pPayload_n;
} COC_Payload_t;

typedef struct
{
  COC_APP_Opcode_Notification_evt_t           COC_Evt_Opcode;
  COC_Payload_t                               DataTransfered;
  uint16_t                                    ConnectionHandle;
  uint8_t                                     DataLength;

} COC_APP_ConnHandle_Not_evt_t;

/* Exported constants --------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Exported macros ------------------------------------------------------------*/

/* Exported functions ---------------------------------------------*/
void COC_APP_Init( void );
void COC_APP_Notification( COC_APP_ConnHandle_Not_evt_t *pNotification );
void COC_APP_SW1_Button_Action(void);
void COC_APP_SW3_Button_Action(void);
void COC_App_KeyButton1Action( void );
void COC_App_KeyButton2Action( void );
void COC_App_KeyButton3Action( void );

#ifdef __cplusplus
}
#endif

#endif /*COC_APPLICATION_H */
