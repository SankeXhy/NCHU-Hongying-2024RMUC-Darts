typedef struct
{
	struct
	{
		int ch0;//Right0
		int ch1;//Right1
		int ch2;//Left2
		int ch3;//Left3
		int ch4;//NULL
		unsigned char s1;//Right_ON
		unsigned char s2;//Left_ON
	}rc;
	struct
	{
		unsigned short x;
		unsigned short y;
		unsigned short z;
		unsigned char press_l;
		unsigned char press_r;
	}mouse;
	struct
	{
		unsigned short v;//按键
	}key;
}RC_ctrl;

typedef struct
{
	double Filter_Last_ch0;
	double Filter_Last_ch1;
	double Filter_Last_ch2;
	double Filter_Last_ch3;
	double Now_Sap_ch0;
	double Now_Sap_ch1;
	double Now_Sap_ch2;
	double Now_Sap_ch3;
	double Filter_a;
	double Filter_Now_ch0;
	double Filter_Now_ch1;
	double Filter_Now_ch2;
	double Filter_Now_ch3;
}Filter_RC_ty;

extern RC_ctrl ctl;
double msp(double x, double in_min, double in_max, double out_min, double out_max);
void DR16_Analysis(uint8_t *sbus_buf);
void Infantry_chassis_Speed_set();
void MSP_1234_to_3214(int *v1, int *v3);
void LIMIT_Speed_M3508();
void LIMIT_Speed_M2006();
void Filter_DT7_DR16();
