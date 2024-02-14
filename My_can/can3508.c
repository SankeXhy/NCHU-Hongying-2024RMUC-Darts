
#include "can3508.h"

M3508_information M3508_1;
M3508_information M3508_2;
M3508_information M2006_1;

void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void set_M3508_1_motor_voltage(CAN_HandleTypeDef* hcan,int16_t v1,int16_t v2,int16_t v3,int16_t v4)
{
  static CAN_TxHeaderTypeDef tx_header;
  static uint8_t             tx_data[8] = {0};

  tx_header.StdId = 0x200;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 0x08;

  tx_data[0] = v1>>8;
  tx_data[1] = v1;
  tx_data[2] = v2>>8;
  tx_data[3] = v2;
  tx_data[4] = v3>>8;
  tx_data[5] = v3;
  tx_data[6] = v4>>8;
  tx_data[7] = v4;

  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];
  if(hcan->Instance == CAN1)
  {
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data
  switch(rx_header.StdId)
	{
	  case 0x201:
	{
		M3508_1.rotor_angle    = (uint16_t)((rx_data[0] << 8) | rx_data[1]);
		M3508_1.rotor_speed    = (uint16_t)((rx_data[2] << 8) | rx_data[3]);
		M3508_1.torque_current = (uint16_t)((rx_data[4] << 8) | rx_data[5]);
		M3508_1.temp           = (uint16_t)rx_data[6];

		break;
	}
	  case 0x202:
	{
		M3508_2.rotor_angle    = (uint16_t)((rx_data[0] << 8) | rx_data[1]);
		M3508_2.rotor_speed    = (uint16_t)((rx_data[2] << 8) | rx_data[3]);
		M3508_2.torque_current = (uint16_t)((rx_data[4] << 8) | rx_data[5]);
		M3508_2.temp           = (uint16_t)rx_data[6];

		break;
	}
	  case 0x203:
	{
		M2006_1.rotor_angle    = (uint16_t)((rx_data[0] << 8) | rx_data[1]);
		M2006_1.rotor_speed    = (uint16_t)((rx_data[2] << 8) | rx_data[3]);
		M2006_1.torque_current = (uint16_t)((rx_data[4] << 8) | rx_data[5]);
		M2006_1.temp           = (uint16_t)rx_data[6];

		break;
	}
	}
  }
}
