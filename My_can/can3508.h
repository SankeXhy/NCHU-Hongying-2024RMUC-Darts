#include "main.h"
#include "can.h"
#include "stm32f4xx.h"

typedef struct
{
    uint16_t can_id;//电机ID
    int16_t  set_voltage;//设定的电压值
    uint16_t rotor_angle;//机械角度
    int16_t  rotor_speed;//转速
    int16_t  torque_current;//扭矩电流
    uint8_t  temp;//温度
}M3508_information;

void can_filter_init(void);
void set_M3508_1_motor_voltage(CAN_HandleTypeDef* hcan,int16_t v1,int16_t v2,int16_t v3,int16_t v4);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
