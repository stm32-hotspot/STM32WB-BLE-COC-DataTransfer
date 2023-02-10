/**
  @page BLE_p2pCentral_COC application 
  
  @verbatim
  ******************************************************************************
  * @file    BLE/BLE_p2pCentral_COC/readme.txt 
  * @author  MCD Application Team
  * @brief   Data transfer via L2CAP data from peripheral to central or from central to peripheral 
  * 		 with a timer that enables to calculate the throughput at application level.
  ******************************************************************************
  *
  * Copyright (c) 2019-2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @endverbatim

@par application Description

How to use data transfer through Connected Oriented Channel (COC) using BLE component.
 
@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Keywords

Connectivity, BLE, IPCC, HSEM, RTC, UART, PWR, BLE protocol, BLE pairing, Dual core

@par Directory contents 
  
  - BLE/BLE_p2pCentral_COC/Core/Inc/app_common.h            	Header for all modules with common definition
  - BLE/BLE_p2pCentral_COC/Core/Inc/app_conf.h              	Parameters configuration file of the application
  - BLE/BLE_p2pCentral_COC/Core/Inc/app_debug.h               	Header for app_debug.c module
  - BLE/BLE_p2pCentral_COC/Core/Inc/app_entry.h              	Parameters configuration file of the application
  - BLE/BLE_p2pCentral_COC/Core/Inc/hw_conf.h                 	Configuration file of the HW
  - BLE/BLE_p2pCentral_COC/Core/Inc/hw_if.h                   	Hardware Interface
  - BLE/BLE_p2pCentral_COC/Core/Inc/main.h                  	Header for main.c module
  - BLE/BLE_p2pCentral_COC/Core/Inc/stm32wbxx_hal_conf.h		HAL configuration file
  - BLE/BLE_p2pCentral_COC/Core/Inc/stm32wbxx_it.h          	Interrupt handlers header file
  - BLE/BLE_p2pCentral_COC/Core/Inc/stm32_lpm_if.h            	Header for stm32_lpm_if.c module (device specific LP management)
  - BLE/BLE_p2pCentral_COC/Core/Inc/utilities_conf.h    		Configuration file of the utilities
  - BLE/BLE_p2pCentral_COC/STM32_WPAN/App/app_ble.h           	Header for app_ble.c module
  - BLE/BLE_p2pCentral_COC/STM32_WPAN/App/ble_conf.h          	BLE Services configuration
  - BLE/BLE_p2pCentral_COC/STM32_WPAN/App/ble_dbg_conf.h      	BLE Traces configuration of the BLE services
  - BLE/BLE_p2pCentral_COC/STM32_WPAN/App/p2p_central_app.h     Header for COC central Application implementation
  - BLE/BLE_p2pCentral_COC/STM32_WPAN/App/tl_dbg_conf.h        	Debug configuration file for stm32wpan transport layer interface
  - BLE/BLE_p2pCentral_COC/Core/Src/app_debug.c               	Debug capabilities source file for STM32WPAN Middleware
  - BLE/BLE_p2pCentral_COC/Core/Src/app_entry.c      			Initialization of the application
  - BLE/BLE_p2pCentral_COC/Core/Src/hw_timerserver.c 			Timer Server based on RTC
  - BLE/BLE_p2pCentral_COC/Core/Src/hw_uart.c 					UART Driver
  - BLE/BLE_p2pCentral_COC/Core/Src/main.c                  	Main program
  - BLE/BLE_p2pCentral_COC/Core/Src/stm32wbxx_hal_msp.c       	This file provides code for the MSP Initialization and de-Initialization
  - BLE/BLE_p2pCentral_COC/Core/Src/stm32wbxx_it.c          	Interrupt handlers
  - BLE/BLE_p2pCentral_COC/Core/Src/stm32_lpm_if.c				Low Power Manager Interface
  - BLE/BLE_p2pCentral_COC/Core/Src/system_stm32wbxx.c      	stm32wbxx system source file
  - BLE/BLE_p2pCentral_COC/STM32_WPAN/App/app_ble.c      		BLE Profile implementation
  - BLE/BLE_p2pCentral_COC/STM32_WPAN/App/p2p_central_app.c     Data Transfer via COC central Application implementation
  - BLE/BLE_p2pCentral_COC/STM32_WPAN/Target/hw_ipcc.c      	IPCC Driver
     
@par Hardware and Software environment

  - This application runs on STM32WB55xx devices.
  
  - This application has been tested with an STMicroelectronics P-NUCLEO-WB55
    board and can be easily tailored to any other supported device 
    and development board.

  - This application is by default configured to support low power mode ( No traces - No debugger )
    This can be modified in app_conf.h (CFG_LPM_SUPPORTED, CFG_DEBUG_BLE_TRACE, CFG_DEBUG_APP_TRACE)

  - SMPS can be used to obtain the best performance in power consumption.
    This feature depends on board components and its activation can be done through specific define (CFG_USE_SMPS)
    configurable in app_conf.h
    See AN5246 - Usage of STM32WB Series microcontrollers.

@par How to use it ? 

This application requires having the stm32wb5x_BLE_Stack_full_extended_fw.bin binary flashed on the Wireless Coprocessor.
If it is not the case, you need to use STM32CubeProgrammer to load the appropriate binary.
All available binaries are located under /Projects/STM32_Copro_Wireless_Binaries directory.
Refer to /Projects/STM32_Copro_Wireless_Binaries/ReleaseNote.html for the detailed procedure to change the
Wireless Coprocessor binary or see following wiki for Hardware setup:
https://wiki.st.com/stm32mcu/wiki/Connectivity:STM32WB_BLE_Hardware_Setup

In order to make the program work, you must do the following:
 - Open your toolchain 
 - Rebuild all files and flash the board with the executable file
 
This application has to be used in front of BLE_p2pPeripheral_COC.
Two P-NUCLEO-WB55 boards are used, one central and one peripheral. 
One board is configured as central thanks to BLE_p2pCentral_COC and the other as peripheral thanks to BLE_p2pPeripheral_COC.
The central board supports the Data transfer by sending L2CAP data through a COC link.

Open a VT100 terminal on Central and Peripheral side (ST Link Com Port, @115200 bauds).
At startup the connection is established.
 - The peripheral device starts advertising.
 - The central device scans and automatically connects to the peripheral (use of CFG_DEV_ID_BLE_COC). 
 - After BLE connection, the central establishes a COC link.  

On central:
SW1 starts/stops data transfer (blue LED ON/OFF)
SW2 changes the PHY (1M or 2M). 
SW3 (short push) sends the pairing request (aci_gap_send_pairing_req command)
SW3 (long push) clears the security database (aci_gap_clear_security_db command)

On peripheral:
SW1 starts/stops data transfer (blue LED ON/OFF)
SW2 toggles connection interval 
SW3 (short push) sends the slave pairing request (aci_gap_slave_security_req command)
SW3 (long push) clears the security database (aci_gap_clear_security_db command)

On terminal receiving data, the number of bytes per second is displayed. 
 
Available Wiki page:
  - https://wiki.st.com/stm32mcu/wiki/Connectivity:BLE_overview


For more details refer to the Application Note: 
  AN5289 - Building a Wireless application

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
