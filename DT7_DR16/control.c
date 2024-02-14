#include "main.h"
#include "control.h"

extern RC_ctrl ctl;
extern uint8_t sbus_buf;
extern int M3508_1_Speed;
extern int M3508_2_Speed;
extern int M2006_1_Speed;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern Filter_RC_ty Filter_RC;


double msp(double x, double in_min, double in_max, double out_min, double out_max)
{
	return out_min+((double)(x-in_min)/(in_max-in_min)*(out_max-out_min));
}

void DR16_Analysis(uint8_t *sbus_buf)
{
	ctl.rc.ch0 = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff; //!< Channel 0
	ctl.rc.ch0-=1024;
	ctl.rc.ch1 = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
	ctl.rc.ch1-=1024;
	ctl.rc.ch2 = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | //!< Channel 2
	(sbus_buf[4] << 10)) &0x07ff;
	ctl.rc.ch2-=1024;
	ctl.rc.ch3 = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
	ctl.rc.ch3-=1024;
	ctl.rc.s1 = ((sbus_buf[5] >> 4) & 0x0003); //!< Switch left
	ctl.rc.s2 = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch right
	ctl.mouse.x = sbus_buf[6] | (sbus_buf[7] << 8); //!< Mouse X axis
	ctl.mouse.y = sbus_buf[8] | (sbus_buf[9] << 8); //!< Mouse Y axis
	ctl.mouse.z = sbus_buf[10] | (sbus_buf[11] << 8); //!< Mouse Z axis
	ctl.mouse.press_l = sbus_buf[12]; //!< Mouse Left Is Press ?
	ctl.mouse.press_r = sbus_buf[13]; //!< Mouse Right Is Press ?
	ctl.key.v = sbus_buf[14] | (sbus_buf[15] << 8); //!< KeyBoard value
	Filter_DT7_DR16();
}

void Filter_DT7_DR16()
{
	Filter_RC.Now_Sap_ch0=ctl.rc.ch0;
	Filter_RC.Now_Sap_ch1=ctl.rc.ch1;
	Filter_RC.Now_Sap_ch2=ctl.rc.ch2;
	Filter_RC.Now_Sap_ch3=ctl.rc.ch3;
}

void LIMIT_Speed_M3508()
{
	if(M3508_1_Speed>3000) M3508_1_Speed=3000;
	if(M3508_1_Speed<-3000) M3508_1_Speed=-3000;
	if(M3508_2_Speed>3000) M3508_2_Speed=3000;
	if(M3508_2_Speed<-3000) M3508_2_Speed=-3000;
}

void LIMIT_Speed_M2006()
{
	if(M2006_1_Speed>15000) M2006_1_Speed=15000;
	if(M2006_1_Speed<-15000) M2006_1_Speed=-15000;
}
