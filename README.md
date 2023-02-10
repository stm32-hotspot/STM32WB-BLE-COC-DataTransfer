# Connection Oriented Channel (COC) Data Transfer

A COC is a logical channel established over a BLE link. 
It allows to exchange data at L2CAP level without GATT layer.    
Two applications are included in the example.    
One with central role and COC initiator BLE_p2pCentral_COC, and one with peripheral role BLE_p2pPeripheral_COC.  

## Setup
These applications are running on two **NUCLEO-WB55RG boards**.    
Open a VT100 terminal on Central and Peripheral side (ST Link Com Port, @115200 bauds).
At startup the BLE connection is established.
 - The peripheral device starts advertising.
 - The central device scans and automatically connects to the peripheral (use of CFG_DEV_ID_BLE_COC).
 - When BLE link is established, the central initiates a COC connection.

## Application description
These applications are based on P2P server and P2P client from STM32CubeWB package v1.15.0.  
On both sides:
- SW1 starts/stops the sending of data. 

On the central/peripheral terminal receiving the current data, the number of bytes per second is displayed.

On peripheral: 
- SW2 toggles connection interval update (aci_l2cap_connection_parameter_update_req).
- SW3 (short push) sends the slave pairing request (aci_gap_slave_security_req command).
- SW3 (long push) sends clear database (aci_gap_clear_security_db).

On central:
- SW2 changes the PHY (1M or 2M). 
- SW3 (short push) sends the pairing request (aci_gap_send_pairing_req command).
- SW3 (long push) clears the security database (aci_gap_clear_security_db command).

## Troubleshooting

**Caution** : Issues and the pull-requests are **not supported** to submit problems or suggestions related to the software delivered in this repository. The STM32WB-BLE-COC-DataTransfer example is being delivered as-is, and not necessarily supported by ST.

**For any other question** related to the product, the hardware performance or characteristics, the tools, the environment, you can submit it to the **ST Community** on the STM32 MCUs related [page](https://community.st.com/s/topic/0TO0X000000BSqSWAW/stm32-mcus).
