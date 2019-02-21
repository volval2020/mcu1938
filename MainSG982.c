/***************************************
//// IC 型号 PIC16F1938
	machine model:SG982
/// 功能：1、环境亮度采集（AD）；
          2、环境温度采集；
          3、PIR温度补偿
          4、与DSP i2C通讯
//// IO口定义：
-----1938
Pin	name   input/output      Net	    note
1 VPP
2 RA0  I  AD_LIGHT  //采集环境亮度
3 RA1  I  AD_PIR  
4 RA2  I  AD_TEMP   //采集环境温度 
5 RA3  O  MODE      //1：TEST 0：ARM
6 RA4  O  PWMOUT    //
7 RA5  I  433M   
8 VSS-GND         
9 RA7 I SYS_WAKEUP  //按键唤醒
10 RA6 O DSP_POWER_ON  //DSP开关
11 RC0 I SYS_POWER_ON  //开关断电
12 RC1 I USB_VBUSI  //检测USB
13 RC2 O LESER_EN   //镭射灯
14 RC3 O P_SCK

15 RC4 I/O P_SDA
16 RC5 O  S_UART 0:MCU->GSM 1：DSP->GSM
17 RC6 TX
18 RC7 RX
19 GND  
20 VDD 3.3V
21 RB0 O GSM_POWERKEY  //开机。关机 HIGH 
22 RB1 O GSM_POWER_EN // 模块电源 HIGH EFFECT
23 RB2 O GSM_DTR  //GSM省电 HIGH EFFECT 1：SLEEP 0：WAKEUP
24 RB3 I GSM_RI   //GSM唤醒 LOW EFFECT
25 RB4 I GSM_STATUS

****************************************/
#include <pic.h>
#include <stdio.h>
#include <htc.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include ".\I2C\I2C.h"
#include "BmcProject.h"
//#include ".\PirScout\BmcPirProcess.h"
//__CONFIG(FOSC_INTOSC &WDTE_SWDTEN& PWRTE_ON&MCLRE_OFF&CP_ON&CPD_ON& BOREN_OFF& CLKOUTEN_OFF& IESO_OFF& FCMEN_OFF);
__CONFIG(FOSC_INTOSC &WDTE_SWDTEN& PWRTE_ON&MCLRE_OFF&CP_OFF&CPD_OFF& BOREN_OFF& CLKOUTEN_OFF& IESO_OFF& FCMEN_OFF);

__CONFIG(WRT_OFF & PLLEN_ON & STVREN_ON & BORV_HI & LVP_OFF);

#define DISABLE	0
#define ENABLE	1
#define DEBUG_LEVEL                  ENABLE
#define DEBUG_LIGHT					 DISABLE

#define TEP_ADC_SAMPLE_MAX      8
#define TEP_ADC_BAD_SAMPMAX     3
#define Light_AD_Offset	7

bit P_OFF,P_ON,Frist_PowerON,PIR_Interval_Flag=FALSE,Power_Down,FL_IndoorRestart=0,FL_StartOK=0,FL_Night2Day=0,FL_RestartGSM=0,FL_ReScanGSM=0,Time_Lapse_Flag=0,FL_CountTimeLapse=0;
bit Time_Lapse_Update_EN,Time_Lapse_Update_Flag,Time_Lapse_EN;
unsigned int wTempCount,tempi,temp,Restart_Count=0,Count_Time=0,Light_avg,Temp_avg;
unsigned int PIR_Interval_Counter,PIR_Interval_Set,Time_Lapse_Set,Work_Hour_Set,Time_Lapse_Start,Work_Hour_Start,Work_Hour_Stop,PIR_Trigger_Set,SMS_Control_Set;//V17_LXY01:
unsigned int RTC_Time,RTC_TimeWH,Current_Time,Configure_433M,Configure_PowerOff,Configure_FlashType;
unsigned char RTC_Hour,RTC_Min,RTC_Second,RWU_Hour,RWU_Min,DR_Hour,DR_Min,GC_Hour,GC_Min,WH_StartHour,WH_StartMin,WH_StopHour,WH_StopMin,temp_i,ReScanGSMMin = 0,ScanGSMMinCount=0,ReScanGSMHour=0;
char RWU_OnOff, DR_OnOff, GC_OnOff,RWU_Flag=0,DR_Flag=0,GC_Flag=0,PB3_RI=0,Work_Hour_Flag=1,temp_ret;
char Light,Light_AD_Value,FL_ARM,FL_SIM_INSERT=0,FL_START=0,Time_Lapse_Counter,Time_Lapse_Update_Count,DSP_MODE,Timer_cnt=0,State_mode,FL_RestartGSM_EN=1;//A0:Power Down mode, A1:Pir Take Photo, A8:test mode
char  ADCNT,Temp_revise,Receive_Buffer;
char ii=0,receive_temp,check_dc=0,check_18650=0,check_battery=0,apply=0;
//unsigned int adc[10]={0};
#if(DEBUG_LEVEL == ENABLE)
char strtest[64]="";
#endif
unsigned char FL_GSM_start=0;

///////////////433////////////////
unsigned short g_wDecode433MH[2];   //地址码16位
unsigned char g_bDecode433ML[2];	//数据码8位
unsigned char Para_M433Zone[24];
unsigned char Modetest_tim=0,Modearm_tim=0,g_bRFEnable ; //接收有效数
char Studyrch,En_Zone0,En_Zone1,En_Zone2,En_Zone3,En_Zone4,En_Zone5,En_Zone6,En_Zone7;
unsigned int RFCOM1[2],ik=0,j=0,k=0,rep=0,short_k=0,head_k=0; 

unsigned char aLight_Low[][2] = 
	{
		{250, 0},
		{235, 1},//{220 1},
		{220, 2},
		{200, 3},
		{140, 4},
		{70, 5},
		{16, 6},
	};

unsigned char aLight_High[][2] = 
	{
		{245, 7},
		{233, 8},
		{192, 9},
		{118, 10},
		{62, 11},
		{37, 12},
		{28, 13},
		{20, 14},
		{0,  15},
	};


void Write_Arm_Mode(void);
void Wait_PowerOff(void);
void StartUp_DSP(void);
void CheckDSPStartup(void);
void Read_SMS_Control(void);//V17_LXY01:
void Read_Work_Hour(void);//V30_LXY020:
char BmcGSMStart(unsigned char  StartStatus);//V48_LXY010: StartStatus : 1: First start; 0: Restart.
unsigned char bmcReadLightFromADC(void);

//V22_LXY01: Save ADC power
void ADC_Stop(void);
void ADC_Start(void);
void WDT_Clear(void)
{
	CLRWDT();
}
	
void WDT_Init(char time)
{
	char value;
	value = time;
	GIE = 0; 	//Global Interrupt Enable
	SWDTEN=0;	
	WDTCON=(value<<1);     //0x05=32ms 
	SWDTEN=1;	
	WDT_Clear();
	GIE = 1;
}

void GSM_Interrupt_Open(void)
{
	GIE = 0; 	//Global Interrupt Enable
	PB3_RI=0;//清除标志位
	IOCBN3=1;
	IOCBF3=0;
	IOCIE=1;
	GIE = 1;
}

GPIO_Interrupt_Close()
{
	IOCIE=0;
}
void delay_ms(int i)
{  
	for(;i>0;i--)
	{  
		WDT_Clear(); 
		for(tempi=0;tempi<640;tempi++)	
			;
	} 
}

void delayus(unsigned char us)
{
	while(us--);
}



void main(void)
{ 
	NOP();
	Init_GPIO(); 
	NOP();
	BmcWriteVersion();
	eeprom_write(0x3f,66);
	Frist_PowerON = 1; 	
#if(DEBUG_LEVEL == ENABLE)
	SendToGSMDebug((char *)"Main Start\r\n");
	sprintf((char *)strtest,"PCON=%x\r\n",PCON);//PCON=%d,5
	SendToGSMDebug(strtest);
#endif

	while(1)
	{ 
		SendToGSMDebug((char *)"while(1)\r\n");		
#if 1
		//==================================================================
	//	if(1==SYS_TEST_18650||(1==SYS_TEST_18650&&0==SYS_TEST_DC))
		if(1==SYS_25_18650&&1==SYS_26_DC)	//RB4 25  BAT_CHK
		{
			SYS_WUP_KEY=1;	  //add yang  eeprom_write(0x5f,0X00);
			eeprom_write(0x5f,0X00);
			SendToGSMDebug((char *)"SYS_TEST_HIGH\r\n");
		}	
	//	if(1==SYS_TEST_DC||(0==SYS_TEST_DC&&0==SYS_TEST_18650))
		if(1==SYS_26_DC)   //RB5  26
		{
			SYS_WUP_KEY=1;
			eeprom_write(0x5f,0X00);
			SendToGSMDebug((char *)"SYS_TEST_DC\r\n");
		}
		if(0==SYS_26_DC&&1==SYS_25_18650)	//RB5  26
		{
			SYS_WUP_KEY=0;
			eeprom_write(0x5f,0X01);
			SendToGSMDebug((char *)"SYS\r\n");
		}
		if(0==SYS_26_DC&&0==SYS_25_18650)	//RB5  26
		//if(GSM_PWR_KEY==1)
		{
			SYS_WUP_KEY=1;
			eeprom_write(0x5f,0X02);
			SendToGSMDebug((char *)"BATTERY\r\n");
		}
		//==================================================================
#endif
		DSP_MODE=MODE_TEST;
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& 
//开机模式开始  
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
		while(ONOFF == 1) 
		{ 
			delay_ms(10); //Trigger delay 2ms
			WDT_Init(0x0A);     
			SendToGSMDebug((char *)"ONOFF == 1\r\n");
			while(ONOFF == 1)
			{      
				Power_Down=0;
				Detect_Temperature();
				FL_IndoorRestart=0;
				FL_START = 1;
				WDT_Clear(); 
				/*
				if(MODE == MODE_TEST)
				{
					DSP_MODE = MODE_TEST;
					Power_ON();
				}
				else if(MODE == MODE_ARM)
				{
					DSP_MODE = MODE_ARM_OUTDOOR;
				}
				*/	
//====================================================
//测试模式－START
//=====================================================
				while(MODE == MODE_TEST)	 
				{ 
					if(ONOFF==0) break; 

					SendToGSMDebug((char *)"\r\nMODE TEST\r\n");	
					State_mode = 0xA8;	
					FL_StartOK = FALSE;
					Restart_Count = 0;
					Power_ON();
					P_OFF=0;
					if(Frist_PowerON)    //首次上电
					{  
						WDT_Clear();
						Frist_PowerON = 0;	
						PWM_OUT = 1;
						wTempCount = 0;
						while(wTempCount < 100)
						{
							delay_ms(100); 
							ADC_change(PIR_AD);
							if(ADRESH < 40)
							break;
							//SendToGSMDebug((char *)"PIR Charge!\r\n");
							wTempCount++;
						}
						ADC_Stop();//V22_LXY01: Save ADC power
						PWM_OUT = 0;
					}
					BmcReadCameraInfo();
					WDT_Init(0x0A);
					eeprom_write(0xC1,1);
					eeprom_write(0xC0,0x00); //clear echo code
					eeprom_write(0xC5,0X00);//ZONE ALARM
					FL_IndoorRestart=0;
					//BmcGSMPowerOn();
					DSP_MODE = MODE_TEST;
					GSM_PWR_EN = 0;
					BmcGSMWake();
					while(DSP_MODE ==MODE_TEST)
					{
						if(ONOFF==0) break; 
						if(MODE!=MODE_TEST)
							break;
						State_mode = 0xA8;	
						IOCIF = 0;     
						GIE = 1;
						if(FL_START == 1)
							CheckDSPStartup();
						WDT_Clear();
						
//====================================================
//测试模式－检测433信号
//====================================================
						//if(CAMERA_TYPE == MG982K)//V36_LXY01: merge 982 and 882.
						if(Configure_433M == ENABLE)//V46_LXY01: merge all.
						{
							Modetest_tim=0;
							Studyrch = eeprom_read(0xC0);
							BmcRFprocessDel();
//							Shut_433=1;
							while(Modetest_tim<150)		
							{
								WDT_Clear();
							    RFIN();
								if(g_bRFEnable==RF_DETECT_SUCC)
								{
									WDT_Clear();
									//DbgPirLedFlk();
									if(Studyrch==0xB0||Studyrch==0xB1||Studyrch==0xB2||Studyrch==0xB3\
										||Studyrch==0xB4||Studyrch==0xB5||Studyrch==0xB6||Studyrch==0xB7)//((Studyrch|=0x0f)^0xBF)==0Studyrch==0xB0
									{
										WDT_Clear(); 
										BmcRFprocessStd();
										BuzzerSound();
										break;
									}
								}
								Modetest_tim++;
							}
							g_bRFEnable=RF_DETECT_FAIL;					
						}
//====================================================
//测试模式－检测到PIR感应闪灯   
//====================================================
						Detect_Temperature();
						PIR_scout();   
						if(P_ON)
						{ 
							SendToGSMDebug((char *)"P_ON ......\r\n");
							PirLedOn();
							delay_ms(100);
							PirLedOff();
							delay_ms(100);
						}	
//====================================================
//测试模式进户内模式或户外模式
//====================================================
						if(P_OFF==1)
						{ 
							/*Modesel = eeprom_read(0x42);
							#if(DEBUG_LEVEL == ENABLE)
							sprintf(strtest,"\r\nWorkMode=%d\r\n",Modesel);
							SendToGSMDebug((char *)strtest);
							#endif
							if(Modesel==0x01) //户内模式超时不进入监控
							{
								SendToGSMDebug((char *)"indjump-arm \r\n");
								//DSP_MODE=MODE_ARM_INDOOR;
								Shut_433=0;
								Camera_OFF();
								//break;
							}
							else //户外模式
							{
								SendToGSMDebug((char *)"outdjump-arm \r\n");
								DSP_MODE=MODE_ARM_OUTDOOR;
								Shut_433=0;
								Camera_OFF();
								//break;
							}*/
							SendToGSMDebug((char *)"DSP TEST MODE POWER DOWN\r\n");
							//V26_LXY030: Power on/off not work immediately .
							if(1)//DSP_MODE == MODE_TEST)
							{
								DSP_MODE = MODE_POWER_OFF;
						//		Shut_433=0;
								delay_ms(1000);
								Power_OFF();
								delay_ms(1000);
								break;
							}
							else
							{
					//			Shut_433=0;
								Camera_OFF();
							}
							
						}
					}	
						
//====================================================
//Indoor监控模式－Start
//====================================================	
					while(DSP_MODE==MODE_ARM_INDOOR)
					{

						if(ONOFF==0) break; 	   
						if(MODE!=MODE_TEST) break;
						SendToGSMDebug((char *)"\r\nMODE_ARM_INDOOR\r\n");	
						DSP_Update_RTC();
						Read_PIR_Trigger();
						Read_PIR_Interval();
						WDT_Clear();
						Read_Time_Lapse();
						Set_Time_Lapse_Start();
						WDT_Clear();
						Write_Arm_Mode();
						//Read_Work_Mode();
						WDT_Init(0x0A);
						eeprom_write(0xC2,0x11);
						Count_Time=0;
						IOCIE = 1;
						IOCBF3 = 0;
						PB3_RI = 0;
						if(eeprom_read(0xC3)==0x01)
						{
							//V20_LXY01:
							FL_SIM_INSERT = 1;
							BmcGSMSleep();
						}
						else
						{
							//V20_LXY01:
							FL_SIM_INSERT = 0;
							BmcGSMWake(); 
						}
						PIR_Interval_Counter=PIR_Interval_Set;
						FL_START = 0;
						while(DSP_MODE==MODE_ARM_INDOOR)
						{
							if(ONOFF==0) break; 	   
							if(MODE!=MODE_TEST) break;
							
//====================================================
//   Indoor监控模式－扫描433信号
//====================================================	
							//if(CAMERA_TYPE == MG982K)//V36_LXY01: merge 982 and 882.
							if(Configure_433M == ENABLE)//V46_LXY01: merge all.
							{
								Modearm_tim=0;
						//		Shut_433=1;     //433 device on
								while(Modearm_tim<150)			
								{			
									WDT_Clear();
									RFIN();
									if(g_bRFEnable==RF_DETECT_SUCC)  //表示已接收到数据
									{
										SendToGSMDebug((char *)"RF_DETECT_SUCC\r\n");
										BmcRespond_RFCtrl();
										g_bRFEnable=RF_DETECT_FAIL;
										break;
									}		
									Modearm_tim++;
									BmcIndoorCountTime();
								}
								g_bRFEnable=RF_DETECT_FAIL;
						//		Shut_433=0;//433 device off
							}
							BmcIndoorCountTime();
				
//====================================================
//Indoor监控模式－判断中断标志位开始启动并传解读短信或者电话
//====================================================				
							if(PB3_RI==1&&FL_SIM_INSERT == 1)		//接收到电话或者短信
							{	
								IOCIE=0;
								WDT_Clear();
								delay_ms(100);
								State_mode =0xA3; // sms and call in
								SendToGSMDebug((char *)"sms\r\n");
								Camera_ON();
								Camera_OFF(); 
								GSM_Interrupt_Open();									
								if(ONOFF==0) break; 	   
								if(MODE!=MODE_TEST) break;
							}

//=======================================================
// Indoor监控模式－布防模式下检测拍照时间间隔和定时拍照
//======================================================= 
							if(FL_ARM==1)
							{
								if((PIR_Trigger_Set<=2)&&(PIR_Interval_Counter>=PIR_Interval_Set))         
								{ 
									//SendToGSMDebug("(char *)PIR_scout\r\n");
									PIR_Interval_Flag = TRUE;
									PIR_scout();
								}
								if(PIR_Interval_Flag == TRUE&&P_ON==1)
								{
									State_mode=0xA1;
									Camera_ON();  
									Camera_OFF(); 
									PIR_Interval_Flag = FALSE;
									if(ONOFF==0) break; 	   
									if(MODE!=MODE_TEST) break;
								}
								//开机时间段内检测Time Lapse
								BmcCountTimeLapse();
								if(Time_Lapse_Flag == TRUE && Time_Lapse_EN ==TRUE)
								{
									State_mode=0xA2;
									eeprom_write(0x29,0x00);
									Camera_ON();  
									Camera_OFF(); 
									if(ONOFF==0) break; 	   
									if(MODE!=MODE_TEST) break;
								}
								else if(Time_Lapse_Update_Flag == TRUE && Time_Lapse_Update_EN ==TRUE)
								{
									State_mode=0xAE;//V40_LXY03: Adjust time ahead.
									Camera_ON();  
									Camera_OFF(); 
									if(ONOFF==0) break; 	   
									if(MODE!=MODE_TEST) break;
								}
							}
							if(eeprom_read(0xC2)==0x88)
							{
								DSP_MODE=MODE_TEST;
								SendToGSMDebug((char *)"\r\nINDOOR:SYS_WUP_KEY\r\n");
							}	

//====================================================
// Indoor监控模式－检测电池电量
//==================================================== 

						}
					}
//====================================================
//Indoor监控模式－ END
//====================================================
					if(MODE!=MODE_TEST)
					{	
						Wait_PowerOff();
						Power_OFF(); 
						delay_ms(1000);   
						break;
					}
					WDT_Init(0x0A);    //V30_LXY030: Save power when turn off take movie. 
					while(DSP_MODE == MODE_POWER_OFF)	//V26_LXY030: Power on/off not work immediately .
					{ 
						SendToGSMDebug((char *)"TEST OUT OFF\r\n");
						if(ONOFF==0) break;    
						if(MODE!=MODE_TEST) break;
			
						IOCBP1=0; //中断1 Power On/Off	
						delay_ms(5);
						IOCBP1=1; //中断1 Power On/Off	
						IOCIE=1;
						GIE = 1;	//总中断允许
						PirLedOff();
						//CLRWDT();   
						PWM_OUT = 0; 
						P_ON = 0; 
						SWDTEN=1;
						BmcGSMPowerOff();
						PORTA=0X00;
						PORTB=PORTB;
						
						Power_OFF(); 
						NOP();
						SLEEP();
						NOP();	
						NOP();
					}
				}	
//==================================================== 
//从测试模式切换到监控模式下先闪灯10s
//====================================================
				while(MODE==MODE_ARM)
				{ 
					if(ONOFF==0) break;  
					if(MODE!=MODE_ARM) break;
					SendToGSMDebug((char *)"MODE_ARM_OUTDOOR\r\n");
					Detect_Battery();
					BmcEnterArmIndicate();
					Read_Adjust_RTC();
					//DSP_Update_RTC();
					if(ONOFF==0) break;  
					if(MODE!=MODE_ARM) break;

//====================================================
// /读取拍照参数
//==================================================== 
					
					GIE = 0;
					Read_PIR_Trigger();
					Read_PIR_Interval();
					WDT_Clear();
					Read_Time_Lapse();
					Read_Work_Hour();//V30_LXY020:
					Set_Time_Lapse_Start();
					WDT_Clear();
					Write_Arm_Mode();//V36_LXY04: add arm/disarm function in hunting mode.
					//Read_Work_Mode();
					Read_RegularWakeUpTime();
					Read_DailyReportTime();
					Read_GameCallTime();
					GSM_Interrupt_Open();
					Count_Time=0;
			//		Shut_433=0;
					WDT_Clear();
//====================================================
// Outdoor监控模式－开始
//==================================================== 
					delay_ms(10);
					DSP_MODE=MODE_ARM_OUTDOOR;
					WDT_Init(0x05);
					PIR_Interval_Counter=PIR_Interval_Set;
					//V36_LXY02: Restart GSM when GSM abnormal.
					eeprom_write(0xC6,0);//V48_LXY020:Initial restart GSM buffer.
					eeprom_write(0xC7,0);//V48_LXY030:Initial rescan GSM buffer.
					FL_RestartGSM=0;
					FL_ReScanGSM=0;//V48_LXY030:Clear rescan GSM Flag.
					ReScanGSMHour = 0;
					Timer_cnt=0;
					P_ON = 0;
					Time_Lapse_Flag = FALSE;
					Time_Lapse_Update_Flag = FALSE;
					PIR_Interval_Flag = FALSE;
					GIE = 1;
					Camera_OFF();
					while(MODE==MODE_ARM)//&&(DSP_MODE==MODE_ARM_OUTDOOR)
					{ 
						if(ONOFF==0) break;  
						if(MODE!=MODE_ARM) break;
						
						BmcGSMSleep();//GSM启动休眠
						WDT_Clear();
						NOP();
						SLEEP();
						NOP();
						IOCIE=1;
						BmcOutdoorCountTime();

//V30_LXY01:
//====================================================
//Outdoor监控模式－判断WorkHour
//====================================================	
						if(Work_Hour_Set>0)             //Time_switch ON 
						{ 
							//Time_switch_check();  
							RTC_Time=RTC_Hour*60+RTC_Min;
							RTC_TimeWH = RTC_Time;//V40_LXY02: Update time for work hour.
							if(FL_Night2Day == 1) 	
							{
								if(RTC_Time<Work_Hour_Start)          
								{
									RTC_Time+=1440;//24*60;
								}
							} 
							if((Work_Hour_Start<=RTC_Time)&&(RTC_Time<=Work_Hour_Stop)) 
							{
								Work_Hour_Flag=1;  
							}
							else			//IN Seting time
							{
								Work_Hour_Flag=0;  
								//V40_LXY020: Update time for work hour.
								if(RTC_Second == 50 )
								{	
									if(((Work_Hour_Start>RTC_TimeWH)&&(Work_Hour_Start-RTC_TimeWH ==40))||((Work_Hour_Start<RTC_TimeWH)&&(Work_Hour_Start+1440-RTC_TimeWH ==40)))
									{
										State_mode =0xAE; 
										SendToGSMDebug((char *)"WorkHour Pre update time\r\n");
										StartUp_DSP();
										if(ONOFF==0) break; 	   
										if(MODE!=MODE_ARM) break;
									}
								}
							}	       
						} 
						else       //Timer_switch=0;
						{
							Work_Hour_Flag=1;  
						}   
//V17_LXY03:
//====================================================
//Outdoor监控模式－判断中断标志位开始启动并传解读短信或者电话
//====================================================	
						if((PB3_RI==1) && (FL_SIM_INSERT == 1) && (SMS_Control_Set == 1))		//接收到电话或者短信
						{	
							IOCIE=0;
							WDT_Clear();
							delay_ms(100);
							State_mode =0xA3; // sms and call in
							SendToGSMDebug((char *)"sms\r\n");
							Camera_ON();
							Camera_OFF(); 
							GSM_Interrupt_Open();									
							if(ONOFF==0) break; 	   
							if(MODE!=MODE_ARM) break;
						}
						//V48_LXY03: Rescan GSM 5 Minute after GSM abnormal.
						if((SMS_Control_Set == 1)&&(FL_ReScanGSM == 1)&&(ScanGSMMinCount>=ReScanGSMMin))//ReScanGSMMin
						{
							State_mode = 0xA5;
							StartUp_DSP();
							FL_ReScanGSM = FALSE;
							if(ONOFF == 0) break; 	   
							if(MODE != MODE_ARM) break;
						}
						
						//V48_LXY03: Rescan GSM 5 Hours after DSP not power on.
						if( (SMS_Control_Set == 1)&&(FL_ReScanGSM == 1)&&(ReScanGSMHour >= 5))
						{
							State_mode = 0xA5;
							StartUp_DSP();
							FL_ReScanGSM = FALSE;
							if(ONOFF == 0) break; 	   
							if(MODE != MODE_ARM) break;
						}
						
						//V36_LXY02: Restart GSM when GSM abnormal.
						if( (SMS_Control_Set == 1)&&(FL_RestartGSM == 1))
						{
							
							//V48_LXY010:Restart GSM When DSP not work 10 hours. 
							BmcGSMStart(0);
							Camera_OFF(); //V48_LXY010: Power off after GSM start.
							if(ONOFF == 0) break; 	   
							if(MODE != MODE_ARM) break;
						}
//====================================================
// Outdoor监控模式－布防模式下检测触发拍照和定时拍照
//==================================================== 
						if(FL_ARM==1)//V36_LXY04: add arm/disarm function in hunting mode.
						{
							if((Work_Hour_Flag == TRUE)&&(PIR_Trigger_Set<=2)&&(PIR_Interval_Counter>=PIR_Interval_Set)&&(Timer_cnt%3==0))//V30_LXY020: //V24_LXY01:Save ADC power        
							{ 
								SendToGSMDebug("(char *)PIR_scout\r\n");
								PIR_Interval_Flag = TRUE;
								PIR_scout();
							}
							WDT_Clear();
							if(PIR_Interval_Flag == TRUE && P_ON==1)
							{
							    SendToGSMDebug("(char *)PIR_Interval_Flag\r\n");
								State_mode = 0xA1;
								StartUp_DSP();
								PIR_Interval_Flag = FALSE;
								if(ONOFF == 0) break; 	   
								if(MODE != MODE_ARM) break;
							}
							if(Work_Hour_Flag == TRUE)//V30_LXY010:
							{
								BmcCountTimeLapse();
							}
							if((Time_Lapse_Flag == TRUE)&&(Time_Lapse_EN == TRUE))
							{
							    SendToGSMDebug("(char *)Time_Lapse_Flag\r\n");
								State_mode = 0xA2;
								eeprom_write(0x29,0x00);
								StartUp_DSP();
								if(ONOFF == 0) break; 	   
								if(MODE != MODE_ARM) break;
							}
							else if((Time_Lapse_Update_Flag==TRUE)&&(Time_Lapse_Update_EN == TRUE))//V42_LXY01:Deal with Power up twice when update time.
							{
							    SendToGSMDebug("(char *)Time_Lapse_Update_Flag\r\n");
								State_mode = 0xAE;
								StartUp_DSP();
								if(ONOFF == 0) break; 	   
								if(MODE != MODE_ARM) break;
							}
						}

						if(RTC_Min%8==0&&RTC_Second == 0)
						{
							Detect_Battery();
						}

//====================================================
// Outdoor监控模式－检测Daily Report
//==================================================== 
						if(DR_Flag==1&&(RTC_Hour>DR_Hour||(RTC_Hour==DR_Hour&&RTC_Min>=DR_Min)))
						{
							State_mode=0xA9;
							SendToGSMDebug((char *)"Daily Report Start!\r\n");
							StartUp_DSP();
							DR_Flag=0;
							if(ONOFF ==0) break; 	   
							if(MODE != MODE_ARM) break;
						}
//====================================================
//Outdoor监控模式－检测Game Call
//==================================================== 
						if(GC_Flag==1&&(RTC_Hour>GC_Hour||(RTC_Hour==GC_Hour&&RTC_Min>=GC_Min)))
						{
							State_mode=0xAB;
							SendToGSMDebug((char *)"Game Call Start!\r\n");
							StartUp_DSP();
							GC_Flag=0;
							if(ONOFF == 0) break; 	   
							if(MODE != MODE_ARM) break;
						}							
//====================================================
// Outdoor监控模式－检测Regular Wake Up
//==================================================== 
						if(RWU_Flag==1&&(RTC_Hour>RWU_Hour||(RTC_Hour==RWU_Hour&&RTC_Min>=RWU_Min)))
						{
							State_mode=0xAA;
							SendToGSMDebug((char *)"Regular Wake Up Start!\r\n");
							PB3_RI=0;
							StartUp_DSP();
							SendToGSMDebug((char *)"Regular Wake Up Stop!\r\n");
							Read_PIR_Trigger();
							Read_PIR_Interval();
							WDT_Clear();
							Read_Time_Lapse();
							DSP_Update_RTC();
							WDT_Clear();
							Read_RegularWakeUpTime();
							Read_DailyReportTime();
							Read_GameCallTime();
							RWU_Flag=0;
						}	
					}
					//MCU_Update_RTC();
				}  //while(!MODE) 
//====================================================
//Outdoor监控模式－ END
//====================================================
			/*	NOP();
				delay_ms(1000);   
				Power_OFF();
				delay_ms(2000);   */

			}
			if(ONOFF==0)
			{	
				Wait_PowerOff();
				Camera_OFF();
				break;
			}
		}//while(ONFF)
//====================================================
//开机模式结束  
//====================================================		
		//delay_ms(250);
//==================================================== 
//测试模式－Outdoor Mode 自动关机
//====================================================
		while(ONOFF==0)//save power
		{ 
			SendToGSMDebug((char *)"ONOFF==0\r\n");
			WDT_Init(0x0A);     //V30_LXY030: Save power when turn off take movie.  
			while(ONOFF==0)//save power
			{ 
				IOCBP1=0; //中断1 Power On/Off	
				delay_ms(5);
				IOCBP1=1; //中断1 Power On/Off	
				IOCIE=1;
				GIE = 1;	//总中断允许
				PirLedOff();
				//CLRWDT();   
				PWM_OUT = 0; 
				P_ON = 0; 
				SWDTEN=1;
				BmcGSMPowerOff();
				PORTA=0X00;
				PORTB=PORTB;
				//SendToGSMDebug((char *)"ONOFF...==0\r\n");
				
				Power_OFF(); 
				NOP();
				SLEEP();
				NOP();	
				NOP();
			}
		}
		delay_ms(250);
	}  //while(1)
}

void USARTsendByte(char senddata)
{
	while(TRMT==0);
	if(TRMT) 
	{
		TXREG=senddata;
	}
	while(TRMT==0);
}

void SendToGSMDebug(char *pstring)
{ 
#if(DEBUG_LEVEL == ENABLE)
	char *pstrTemp;
	unsigned char i,l;
	l=strlen(pstring);
	pstrTemp = pstring;
	for(i=0;i<l;i++)
	{ 
		USARTsendByte(*pstrTemp++);
	}
#endif
}

//==================================================== 
//查看I2C通讯功能子程序
//==================================================== 
char I2C_error(void)
{ 

	char I2CERROR,I2C_i;
	I2CERROR = 0;
	if(SCL&&(!SDA))
	{ 
		delay_ms(100);      
		if(SCL&&(!SDA))
		{ 
			for(I2C_i = 16;I2C_i>0;I2C_i--)	
			{	 
				if(SDA)
				{
					I2CERROR = 0; 
					break;
				}
				else
				{ 
					I2CERROR = 1;
				}
				if(MODE==MODE_ARM) 
				{ 
					I2CERROR = 0; 
					return I2CERROR;
				}
				delay_ms(2);
			}
		}
	}
	return 0;//I2CERROR;  
}	

//==================================================== 
//GPIO初始化子程序
//==================================================== 
void Init_GPIO(void)
{ 
	OSCCON = 0xF0;
	NOP();

	PORTA = 0x00;
	PORTB=0XA2;   //  0X1010 0010
	OPTION_REG = 0x80;       //PORTA pull up enable lxy optimize for save power
	WPUE=0x00;
	//V36_LXY01: Merge 982 and 882.
	TRISA = 0x0f;			///0B00101111
	PORTA = 0x00;
	TRISB = 0x3b;			// 0B00111011
	TRISC = 0b00011000;		//RC2  :  LIGHT_CTL
	//TRISC = 0b00011100;	
	PORTB=0X0b;//0X3B;   //indicator light//   00001010
	PORTC=0X00;//0X3B;   //indicator light

	ANSELA = 0x07 ;  //AN1 AN5 AN6 AN7 is analog 
	ANSELB = 0x00 ;  
	ADCON1 = 0x50; //ADControl clock Fosc/8=1/16	
//TIMER1 INIT	
	TMR1H = 0x0B;//0x12;//0x0B;
	TMR1L = 0xDC; //  0x88;//0xDC;  
	T1CON = 0x71;//0x01;
	T1GCON=0x00;
	TMR1IE=1;
	TMR1IF=0;

	DbgPirLedOff();
	I2C_Slave_Init();
//interupter RB GPIO	
	GSM_Interrupt_Open();
	IOCBP1=1; //中断1 Power On/Off


/*   
//INIT PWM 
	CCPR5L=0X7F;
	CCP5CON=0X3C;
	CCPTMRS1=0X02;
	INTCON=0X00;
	PR6=0XFF;
	T6CON=0X04;
*/

//INIT USART //CQL DEBUG USART
#if(DEBUG_LEVEL == ENABLE)
	RCSTA=0B10110100;  //允许串行口接收工作8位 异步
	TXSTA=0X84;   //0B00000100选择异步高速方式传输8位数据0X04
	SYNC=0;		//选择异步通信模式
	BRGH=1;		//选择高速波特率发生模式
	BRG16=1;
	SPBRG=138;  //将传输的波特率设为约 16 9600-207  32M--138-57600

	TXEN=1;		//允许发送数据
	TXIE=0;  	//TXIE=0;发送不需要中断处理
	RCIE=0;		//串口接收中断允许//lxydebug 1
#endif
	PEIE=1;     //外围中断允许
	GIE = 1;	//总中断允许



}

void I2C_Slave_Init(void)
{
	IOCIE = 0;
	PEIE=1;	//Peripheral Event Interrupt Enable,Bit PEIE of the INTCON register must be set to enable any peripheral interrupt
	SSPIE=1; //Enables the MSSP interrupt;	
	SSPCON1=0x06;//0x0e	//I2C Mode,Slave,7 bits address, permit start and pose interrupt 
	SSPSTAT = 0x80;
	SSPADD = 0x30;
	SSPEN= 1;// Synchronous Serial Port Enable bit, Enables the serial port and configures the SDA and SCL pins as the source of the serial port pins
	CKP = 1;
	GCEN = 0;//General call address disabled
	SCIE = 0;// Start Condition Interrupt Enable bit (I2C mode only)
	PCIE = 0;// Stop Condition Interrupt Enable bit (I2C mode only)
	BOEN= 1;	// Buffer Overwrite Enable bit In I2C Slave mode:
				//1 = SSPBUF is updated and ACK is generated for a received address/data byte, ignoring the
				//state of the SSPOV bit only if the BF bit = 0.
				//0 = SSPBUF is only updated when SSPOV is clear
	SSPMSK = 0x00; //The received address is not used to detect I2C address match

	SDAHT =1; //SDA Hold Time(on SDA after the falling edge of SCL) Selection bit (I2C mode only) 1 = Minimum of 300 ns ;0  = Minimum of 100 ns
	AHEN = 0;// Address Hold Enable bit (I2C Slave mode only)
	DHEN= 0;// Data Hold Enable bit (I2C Slave mode only)
	SBCDE = 0;// Slave Mode Bus Collision Detect Enable bit (I2C Slave mode only)
	SEN = 0;//SEN bit of the SSP1CON2 register is set,SCL will be held low (clock stretch) following each received byte(ACK then held low).
	TRISC| = 0b00011000;// RC3 I2C SDA RC4 SCL
	IOCIE = 1;

}



//==================================================== 
//中断服务程序
//====================================================

void interrupt ISR(void)
{

	GIE = 0;

	if(SSPIF==1)     //detect start 
	{ 
		SSPIE = 0;
		if((D_nA==0))//Address 
		{
			Receive_Address=1;
			if(R_nW==1)//Transmission 
			{
				if(Receive_Buffer == 0x40)      
				{
					SSPBUF =State_mode;
					FL_StartOK=TRUE;					
				}
				else if(Receive_Buffer == 0x41)   
				{
					SSPBUF =Light;
					FL_StartOK=TRUE;
				}
				else if(Receive_Buffer == 0x47)   
				{
					SSPBUF =Light_AD_Value;
					FL_StartOK=TRUE;
				}
				else
				{ 
					ii = eeprom_read(Receive_Buffer);	  
					SSPBUF =ii;
				}
			}
		}
		else
		{
			if(Receive_Address==1)
			{
				Receive_Address=SSPBUF; 
				if(R_nW==0)
				{
					Receive_Buffer=Receive_Address;
				}
			}
			else //Address Receiving Data
			{
				receive_temp=SSPBUF;
				if(Receive_Address == 0x55)        //关机
				{ 
					//receive_temp=SSPBUF;  
					//if(receive_temp==0x88&&DSP_MODE==MODE_ARM_INDOOR)
					//	DSP_MODE=MODE_TEST;
					//else
					if(receive_temp==0x66&&DSP_MODE==MODE_TEST)
						DSP_MODE=MODE_ARM_INDOOR;
					else if(receive_temp==0xaa)
						P_OFF=1;
					//eeprom_write(Receive_Address,receive_temp); 
				}
				
				else if(Receive_Address == 0xC8)        
				{ 
					//receive_temp=SSPBUF;  
					if(receive_temp==0x01&&DSP_MODE==MODE_ARM_OUTDOOR)
						FL_GSM_start=1;
					//eeprom_write(Receive_Address,receive_temp); 
				}
				
				else
				{
					//receive_temp=SSPBUF;  
					//eeprom_write(Receive_Address,receive_temp); 
				}
				eeprom_write(Receive_Address,receive_temp);
			}
		}
		CKP=1;	
		receive_temp=SSPBUF; 
		SSPOV=0;	
		BF = 0;
		SSPIF=0;
  		BCLIF = 0;
		SSPIE = 1;
	} 
/*	else if(RCIF&&RCIE)      //接收到数据
	{
		GSMRcvBuf[iGSMRcvLength] = RCREG;
		if(OERR==1)   //如果有溢出错误
		{
			CREN = 0;     //清零CREN位可将此位OERR清零
			delay_ms(10);
			CREN = 1;
		}
		iGSMRcvLength ++;
		if(iGSMRcvLength >30) 
		{
			iGSMRcvLength = 0;
		}
		RCIF=0;
	}*/
	else if(IOCBF3&&IOCIE)  //RI INTERUPTER
	{
		//DbgPirLedFlk();	
		PB3_RI=1;
		IOCBF3=0;
		IOCIE=1;
	}
	else if(IOCBF1&&IOCIE)  //POWER ON /OFF  Save Power
	{
		//DSP_MODE=MODE_TEST;
		//ONOFF=1;
		IOCBF1=0;
		IOCIE=1;
	}		
	if(TMR1IF==1) //Timer1 Count 16.2ms
	{
		
		Count_Time++;
        TMR1IF=0;
#if 1
		//==================================================================
	//	if(1==SYS_TEST_18650||(1==SYS_TEST_18650&&0==SYS_TEST_DC))
		if(1==SYS_25_18650&&1==SYS_26_DC)	//RB4 25  BAT_CHK
		{
			SYS_WUP_KEY=1;	  //add yang  eeprom_write(0x5f,0X00);
			eeprom_write(0x5f,0X00);
	//		SendToGSMDebug((char *)"SYS_TEST_HIGH\r\n");
		}	
	//	if(1==SYS_TEST_DC||(0==SYS_TEST_DC&&0==SYS_TEST_18650))
		if(1==SYS_26_DC)   //RB5  26
		{
			SYS_WUP_KEY=1;
			eeprom_write(0x5f,0X00);
	//		SendToGSMDebug((char *)"SYS_TEST_DC\r\n");
		}
		if(0==SYS_26_DC&&1==SYS_25_18650)	//RB5  26
		{
			SYS_WUP_KEY=0;
			eeprom_write(0x5f,0X01);
	//		SendToGSMDebug((char *)"SYS\r\n");
		}
		if(0==SYS_26_DC&&0==SYS_25_18650)	//RB5  26
		//if(GSM_PWR_KEY==1)
		{
			SYS_WUP_KEY=1;
			eeprom_write(0x5f,0X02);
	//		SendToGSMDebug((char *)"BATTERY\r\n");
		}
		//==================================================================
#endif
	}
	GIE = 1;
} 




//==================================================== 
//PIR感应检测子程序
//有感应：P_ON＝1，无感应：P_ON=0
//==================================================== 
void PIR_scout(void)
{
	GIE = 0;  
	ADC_change(PIR_AD);        // WDT 32ms pull up 1 time  
	//sprintf(strtest,"=====PIR===== %x\r\n",ADRESH);
	//SendToGSMDebug((char *)strtest);
	P_ON = 0;  	    
	//((0x55*4)/1024)*3.3=1.09
	if((ADRESH>(0x6A+Temp_revise-PIR_Trigger_Set*4))||(ADRESH<(0x3A-Temp_revise+PIR_Trigger_Set*4)))      //6A 38 
	{ 
		P_ON = 1;
#if(DEBUG_LEVEL == ENABLE)
		//sprintf(strtest,"PIR %x\ T %x S %dr\n",ADRESH,Temp_revise,PIR_Trigger_Set);
		//SendToGSMDebug((char *)strtest);
#endif
		//V22_LXY01: Save ADC power
		for(ADCNT = 4;ADCNT>0;ADCNT--)       
		{ 
			ADC_Start();
			delay_ms(1);      
			//ADC_change(PIR_AD); 
			if((ADRESH<(0x6A+Temp_revise-PIR_Trigger_Set*4))&&(ADRESH>(0x3A-Temp_revise+PIR_Trigger_Set*4)))  //如果检测到信号，测试8次都在感应区间内
			P_ON = 0;
		}
	}   
#if(DEBUG_LEVEL == ENABLE)
	//P_ON=1;//lxydebug add for dead
#endif
	ADC_Stop();//V22_LXY01: Save ADC power
	GIE = 1;  
}

//==================================================== 
//电池电量检测子程序
//电池电量过低则Led灯闪烁4下关机，
//==================================================== 
void Detect_Battery(void)
{
	char i,APP;
    int BATTERY;
    APP = eeprom_read(0x5f); 
    if(1==APP)BATTERY=11;
    if(0==APP)BATTERY=13;
    if(2==APP)BATTERY=12;
	ADC_change(BATTERY);     
    //sprintf(strtest,"BAT %x\ Power_Down\r\n",ADRESH);
    //SendToGSMDebug((char *)strtest);
	Power_Down = 0;
	if(ADRESH<0x4D)           //5D = 1.2V    
	{ 
		Power_Down = 1;
		for(ADCNT = 8;ADCNT>0;ADCNT--)       
		{ 
			delay_ms(1);      
			ADC_change(BATTERY); 
			if(ADRESH>0x4D)
			{
				Power_Down = 0;
			}
		}
	}
	ADC_Stop();	//V22_LXY01: Save ADC power
	while(Power_Down)   
	{  
		for(i = 0;i<10;i++)
		{ 
			PirLedFlash();
			delay_ms(100);
			wTempCount++;
		}
		if(wTempCount>300)
		{
			for(ADCNT = 8;ADCNT>0;ADCNT--)       
			{ 
				delay_ms(1);      
				ADC_change(BATTERY); 
				if(ADRESH>0x4D)
				{
					Power_Down = 0;
				}
			}
			wTempCount = 0;
		}
		//sprintf(strtest,"BAT %x\ Power_Down %x ONOFF %dr\n",ADRESH,Power_Down,ONOFF);
		//SendToGSMDebug((char *)strtest);
		if(MODE==MODE_TEST)
		{
			break;
		}
		if(!ONOFF)            
		{
			break;
		}
	}
}

//==================================================== 
//温度检测子程序
//Temp_revise: 0~19.5
//==================================================== 
void Detect_Temperature(void)
{
	for(tempi=0;tempi<8;tempi++)
	{
		ADC_change(Temperature_AD);
		Temp_avg+=ADRESH;
	}
	ADC_Stop();	//V22_LXY01: Save ADC power
	Temp_avg=(Temp_avg>>3);
        
	if(Temp_avg<50)
	{
		Temp_avg = 50;
	}
	if(Temp_avg>128) 
	{
		Temp_avg = 128;
	}
	Temp_revise = (Temp_avg-50)/4;
}

//==================================================== 
//关照相子程序
//==================================================== 
void Camera_OFF(void)
{ 
	if(DSP_MODE==MODE_ARM_OUTDOOR)
	{
		GIE = 1;
		IOCIE = 0;
		IOCIF = 0;
		delay_ms(1);
		PirLedOff();
		if(ONOFF==0||MODE!=MODE_ARM)
		{	
			Wait_PowerOff();
			Power_OFF(); 
			delay_ms(1000);   
		}
		else
		{
			//V42_LXY010: Deal with power off ahead.
			//if(CAMERA_TYPE == MG983K||CAMERA_TYPE == MG982K)
			delay_ms(300);   //V40_LXY05:
			Power_OFF(); 
			delay_ms(1000);   
		}
		P_OFF = 0; 
		PEIE=1;
		IOCIE = 1;
		
		FL_GSM_start=0;
		
		BmcGSMSleep();
		//PIR_Interval_Counter = 0; //V34_LXY01:Not trigger imidiately
		SendToGSMDebug((char *)"OUTDOOR:Camera_OFF\r\n");

	}
	else if(DSP_MODE==MODE_ARM_INDOOR)
	{
		delay_ms(1);
		PirLedOff();
		if(eeprom_read(0xC3)==0x01)
			BmcGSMSleep(); 
		else
			BmcGSMWake();
		if(FL_IndoorRestart==1)
		{
			FL_IndoorRestart=0;
			Power_OFF(); 
			delay_ms(3500);
			DSP_MODE=MODE_TEST;
			SendToGSMDebug((char *)"indoor restart\r\n");
			State_mode=0xAF;
			Power_ON(); 
			delay_ms(2000);
		}
		BmcGSMSleep();
		P_OFF = 0; 
		SendToGSMDebug((char *)"INDOOR:Camera_OFF\r\n");
	}
}

void Wait_PowerOff(void)
{
	temp=0;
	while(!P_OFF)   
	{
		WDT_Clear();
		temp++;
		delay_ms(50);   
		
		//启动异常退出
		if(temp>=60)
			break;
	}
	delay_ms(50);   
}

//==================================================== 
//开照相子程序
//==================================================== 
void Camera_ON(void)
{  
	char PIR_i;
	char Dsp_Ack;

	char Dsp_answer=0;
//	unsigned short OutTimes;
	unsigned int COUNT;

	if(State_mode==0xA3)
	{
		BmcGSMWake();
	}
	
	//SendToGSMDebug((char *)"Camera_ON\r\n");
	eeprom_write(0x30,RTC_Hour);//protect current time 
	eeprom_write(0x31,RTC_Min); 
	eeprom_write(0x32,RTC_Second+1); 
	if(DSP_MODE==MODE_ARM_OUTDOOR)
	{
//==================================================== 
//读亮度值并进行换算
// 0x00~0x80~0xE0~0xFF
//    15 ~   7   ~  0	   ~    0	
//==================================================== 
		SendToGSMDebug((char *)"\r\nMODE_ARM_OUTDOOR:Camera_ON\r\n");
		#if(DEBUG_LEVEL == ENABLE)//V36_LXY: add for debug
		sprintf((char *)strtest,"State_mode=%x\r\n",State_mode);
		SendToGSMDebug(strtest);
		#endif
		if(State_mode==0xA3||State_mode==0xA4)
		{
			delay_ms(1500);	 //SMS interrupt
		}
		P_OFF=0;
		FL_StartOK=FALSE;
		Power_ON();            //开机
		delay_ms(64);
//==================================================== 
//读亮度值并进行换算
// 0x00~0x80~0xE0~0xFF
//    15 ~   7   ~  0	   ~    0	
//==================================================== 
        #if 0
		Light_avg=0;
		for(PIR_i=0;PIR_i<8;PIR_i++)
		{
			ADC_change(Light_AD);
			delay_ms(5);
			adc[PIR_i]=ADRESH;
			Light_avg+=adc[PIR_i];
		}
		ADC_Stop();	//V22_LXY01: Save ADC power
		Light_avg=(Light_avg>>3);
		if(Light_avg>0xE0)	
		{
			Light = 0x00;
		}
		else  
		{
			if(Light_avg>0x80)	
			{
				//Light = 6-(Light_avg-0x88)/16;
				Light = 6-((Light_avg-0x88)>>4);
			}
			else		
			{
				//Light = (0x0F-Light_avg/16);
				Light =0x0F-(Light_avg>>4);
			}        
		}
        #else
        bmcReadLightFromADC();
        #endif
		#if(DEBUG_LEVEL == ENABLE)
		sprintf((char *)strtest,"light=%x\r\n",Light);//PCON=%d,5
		SendToGSMDebug(strtest);
		#endif
		delay_ms(140);     //再次触发测试   //140  check again	

		if(0)//State_mode == 0xA1)  //Check PIR Triger again//lxy close for pir err
		{
			for(PIR_i = 4;PIR_i>0;PIR_i--)
			{
				delay_ms(5); 
				ADC_change(PIR_AD);
				if((ADRESH<(0x66+Temp_revise-PIR_Trigger_Set*4))&&(ADRESH>(0x42-Temp_revise+PIR_Trigger_Set*4)))  //如果检测到信号，测试8次都在感应区间内
				{
					State_mode = 0xA0;
				}
			} 	     
			ADC_Stop();	     //V22_LXY01: Save ADC power
			if(State_mode == 0xA0)
			{ 
				P_ON = 0;
				GIE = 0;
				IOCIE = 0;
				IOCIF = 0;
				GIE = 1;
				delay_ms(1);
				Power_OFF();  
				PirLedOff();
				PIR_Interval_Flag = 1;
				SendToGSMDebug((char *)"PIR Trigger Fail!\r\n");	
				return;
			}
		}
		delay_ms(10);

		//SendToGSM(strtest);

		IOCIF = 0;     
		GIE = 1;
		PWM_OUT = 0;
		delay_ms(250);   
		delay_ms(250);  
		COUNT = 0;
		SendToGSMDebug((char *)"Wait Take Photo\r\n");	      

		P_ON=0;
		if(State_mode==0xAB)//GAME CALL
			PIR_Trigger_Set=0;

		while(!P_OFF)   
		{
			if(State_mode==0xAB)//GAME CALL
			{
				if(P_ON==0)
					PIR_scout();
				delay_ms(250);
				if(P_ON==1)
				{
					PIR_scout();
					if(P_ON==1)
					{
						DSP_MCU_COM =1;
						delay_ms(700);
						DSP_MCU_COM =0;
						P_ON=2;
						SendToGSMDebug((char *)"\r\nGC: Pir Trigger!\r\n");
					}
					else
					{
						P_ON=0;
					}
				}

				WDT_Clear();
				COUNT++;
				
				//启动异常退出
				if(COUNT==20&&FL_StartOK==FALSE)
					break;
				
				//超时退出
				if(COUNT>7200)	//check again //254	//waiting time more than 64s will break out
				{
					SendToGSMDebug((char *)"Wait Out Time\r\n");
					break;
				}
				//监测到相机关机退出
				if((!ONOFF)||(MODE==MODE_TEST)) 
				{
					delay_ms(1000);//V30_LXY020:
					break;
				}
			}
			else if(State_mode==0xAA)
			{
				WDT_Clear();
				COUNT++;
				delay_ms(1000);   
/*				if(PB3_RI==1)		//Regular Wake Up 接收到电话或者短信
				{	
					DSP_MCU_COM =1;
					IOCIE=0;
					WDT_Clear();
					delay_ms(700);
					SendToGSMDebug((char *)"sms\r\n");
					DSP_MCU_COM =0;
					GSM_Interrupt_Open();
					PB3_RI=0;
				}
*/
				DSP_MCU_COM =1;
				IOCIE=0;
				if(COUNT>720)	//check again //254	//waiting time more than 64s will break out
				{
					DSP_MCU_COM =1;
					IOCIE=1;
				}
								
				//启动异常退出
				if(COUNT==5&&FL_StartOK==FALSE)
					break;
				
				//超时退出
				if(COUNT>1000)	//check again //254	//waiting time more than 64s will break out
				{
					SendToGSMDebug((char *)"RWU Time Out\r\n");
					break;
				}
				//监测到相机关机退出
				if((!ONOFF)||(MODE==MODE_TEST)) 
				{
					delay_ms(1000);//V30_LXY020:
					break;
				}
			}
			else
			{
				WDT_Clear();
				COUNT++;
				delay_ms(120);   //V40_LXY05:

				if(FL_GSM_start==1)
				{
					BmcGSMWake();
					FL_GSM_start=0;
				}
				
				//启动异常退出
				if(COUNT==50&&FL_StartOK==FALSE)//V40_LXY05:
				{
					SendToGSMDebug((char *)"DSP Start Fail\r\n");
					break;
				}
				
				//超时退出
				//V46_LXY04: Compatible MG982K/MG983K/MG882K. add for SG983, 8 min time out.
				if(COUNT>(Configure_PowerOff*600))//V40_LXY05:
				{
					SendToGSMDebug((char *)"Wait Out Time\r\n");
					break;
				}
				//监测到相机关机退出
				if((!ONOFF)||(MODE==MODE_TEST)) 
				{
					delay_ms(1000);//V30_LXY020:
					break;
				}
			}
		}
		//SendToGSMDebug((char *)"Take Photo End\r\n");	      
		if(State_mode==0xAB)//GAME CALL
			Read_PIR_Trigger();
		//V48_LXY020:DSP ask MCU restart GSM	
		if(eeprom_read(0xC6) == 1)
		{
			FL_RestartGSM = 1;
			eeprom_write(0xC6,0); 
		}
		//V48_LXY030:DSP ask MCU rescan GSM	
		delay_ms(1);
		ReScanGSMMin = eeprom_read(0xC7);
		if( ReScanGSMMin >= 1)
		{
			FL_ReScanGSM = 1;
			eeprom_write(0xC7,0);
			ScanGSMMinCount = 0;
		}
		ReScanGSMHour = 0;//V36_LXY02: Restart GSM when GSM abnormal.
		P_ON = 0;
	}
	else if(DSP_MODE==MODE_ARM_INDOOR)
	{
		SendToGSMDebug((char *)"MODE_ARM_INDOOR:Camera_ON\r\n");
		#if(DEBUG_LEVEL == ENABLE)//V36_LXY:add for debug
		sprintf((char *)strtest,"State_mode=%x\r\n",State_mode);
		SendToGSMDebug(strtest);
		#endif
		Power_ON();
		DSP_MCU_COM =1;
		Dsp_Ack=eeprom_read(0xC2);
		COUNT=0;
		P_OFF=0;
		FL_IndoorRestart=0;
		Dsp_answer=0;
		while(COUNT<50)
		{
			WDT_Clear();
			Dsp_Ack=eeprom_read(0xC2);
			if(Dsp_Ack==0x66)//||P_OFF==1
			{
				Dsp_answer = 1;
				SendToGSMDebug((char *)"Dsp_answer = 1\r\n");
				break;
			}
			else
				SendToGSMDebug((char *)"Dsp_answer = 0\r\n");
			COUNT++;
			if(COUNT>=49)	//check again //254	//waiting time more than 64s will break out
			{
				SendToGSMDebug((char *)"Wait 0x66 Out Time\r\n");
				break;
			}
			if(COUNT>=10)	//check again //254	//waiting time more than 64s will break out
			{
				if((State_mode == 0xA1)||(State_mode == 0xA2))
				{
					Dsp_answer = 1;
					eeprom_write(0xC2,0x66);
				}	
				break;
			}

			if(DSP_MODE!=MODE_ARM_INDOOR)
				break;
			if(MODE==MODE_ARM)
				break;

			if(P_OFF==1)
				break;

			if(!ONOFF) 
			{
				//SendToGSMDebug((char *)"Turn Off go out\r\n");
				break;
			}    	     
			delay_ms(70);   
		}
		//SendToGSMDebug((char *)"Dsp_ack 0x66\r\n");
		DSP_MCU_COM = 0;
		Dsp_Ack=eeprom_read(0xC2);
		COUNT=0;
/*	
		#if(DEBUG_LEVEL == ENABLE)
		sprintf((char *)strtest,"P_OFF=%x\r\n",P_OFF);//PCON=%d,5
		SendToGSMDebug(strtest);
		sprintf((char *)strtest,"State_mode=%x\r\n",State_mode);//PCON=%d,5
		SendToGSMDebug(strtest);
		#endif
*/

		if((Dsp_answer==1))
		{
			SendToGSMDebug((char *)"Waiting for 0x00\r\n");
			while(COUNT<600)
			{
				WDT_Clear();
				Dsp_Ack=eeprom_read(0xC2);
				if(Dsp_Ack==0x00||Dsp_Ack==0x88)//0x00:触发工作结束;//||P_OFF==1
					break;
				COUNT++;
				if(COUNT>=599)	//check again //254	//waiting time more than 64s will break out
				{
					SendToGSMDebug((char *)"Wait 0x00 Out Time\r\n");
					PirLedOff();
					delay_ms(200);
					BmcGSMSleep();
					delay_ms(2000);
					Power_OFF();
					delay_ms(1000);
					FL_IndoorRestart=1;
					break;
				}
				if(!ONOFF) 
				{
					//SendToGSMDebug((char *)"Turn Off go out\r\n");
					break;
				}    	     
				if(DSP_MODE!=MODE_ARM_INDOOR)
					break;
				if(MODE==MODE_ARM)
					break;				
				if(P_OFF==1)
					break;
				delay_ms(400);
			}
		}
		SendToGSMDebug((char *)"Dsp_ack 0x00\r\n");
		DSP_MCU_COM = 0;
		Dsp_Ack=eeprom_read(0xC2);
		if(Dsp_Ack!=0x88)//wake up
			eeprom_write(0xC2,0x11);
		else
			DSP_MODE=MODE_TEST;
	}
	if(State_mode == 0xA1)
	{
		PIR_Interval_Counter = 0; 
	}
	else if(State_mode == 0xA2)
	{
		Dsp_Ack=eeprom_read(0x29);
		if(Dsp_Ack==0x66)
		{
			Time_Lapse_Flag=FALSE;
			Time_Lapse_EN = FALSE;
			Time_Lapse_Counter=0;
		}
		else
		{
			Time_Lapse_Flag=FALSE;
			Time_Lapse_EN = TRUE;
			Time_Lapse_Counter=0;
		}
	}
	else if(State_mode == 0xA3)
	{
		WDT_Clear();
		Read_PIR_Trigger();
		//V36_LXY03:SMS change parameter
		temp_ret = Read_PIR_Interval();
		if(temp_ret)
		{
			PIR_Interval_Counter = PIR_Interval_Set;
		}
		WDT_Clear();
		Read_Work_Hour();//V30_LXY030:
		//V36_LXY03:SMS change parameter
		temp_ret = Read_Time_Lapse();
		if(temp_ret)
		{
			Set_Time_Lapse_Start();
		}
		Read_Arm_Mode();	
		Read_SMS_Control();//V17_LXY01:
		//V20_LXY02:
		WDT_Clear();
		Read_RegularWakeUpTime();
		Read_DailyReportTime();
		Read_GameCallTime();		
	}
	//V42_LXY01:Deal with Power up twice when update time.
	else if(State_mode == 0xAE)
	{
		Time_Lapse_Update_Flag=FALSE;
		Time_Lapse_Update_EN = FALSE;
		Time_Lapse_Update_Count = 0;
		SendToGSMDebug((char *)"Time_Lapse_Update_EN = FALSE;\r\n");
	}
	DSP_Update_RTC();
}
  

void StartUp_DSP(void)
{
	Camera_ON();
	Camera_OFF();
	for(temp_i = 0; temp_i < 3; temp_i++ )
	{
		if(FL_StartOK==FALSE)
		{
			Camera_ON();
			Camera_OFF();
		}
	}	
}  
//==================================================== 
//ADC转换子程序
//入口条件：通道选择
//==================================================== 
void ADC_change(char AN)
{  	
    ADCON1 = 0x50; //ADC clock 1/16
    ADCON0 = (AN<<2)|0x01;  //AN5 is select
    delay_ms(1);
    GO_nDONE = 1;   
}
//V22_LXY01: Save ADC power
//==================================================== 
//ADC转换结束开始子程序
//==================================================== 
void ADC_Start(void)
{  	
    GO_nDONE = 1;   
}

//==================================================== 
//ADC转换结束子程序
//==================================================== 
void ADC_Stop(void)
{  	
    GO_nDONE = 0;   
	ADON = 0;
}
//==================================================== 
//Read 当前时间：RTC
//读取 RTC,并将进入打猎模式闪灯的11s更新到RTC
//==================================================== 
void Read_Adjust_RTC(void)
{ 
	delay_ms(5);
	RTC_Hour = eeprom_read(0x30); 
	RTC_Min = eeprom_read(0x31);  
	//V38_LXY02: Not take the first photo of time lapse when SMS on. 
	RTC_Second = eeprom_read(0x32)+6;
	if(RTC_Second>=60)
	{
		RTC_Second=RTC_Second-60;
		RTC_Min++;
		if(RTC_Min>=60)
		{
			RTC_Min=RTC_Min-60;
			RTC_Hour++;
			if(RTC_Hour>=24)
				RTC_Hour=RTC_Hour-24;
		}
	}
	eeprom_write(0x30,RTC_Hour);
	eeprom_write(0x31,RTC_Min);
	eeprom_write(0x32,RTC_Second);
	#if(DEBUG_LEVEL == ENABLE)
	RTC_Time= (RTC_Hour*24+RTC_Min)*60+RTC_Second;
	SendToGSMDebug((char *)"\r\nDSP_Update_RTC\r\n"); 
	sprintf(strtest,"\r\nRTC  H=%d, M=%d,S=%d\r\n",RTC_Hour,RTC_Min,RTC_Second);//,Time=%d,RTC_Time
	SendToGSMDebug(strtest);
	#endif

}

//==================================================== 
//Read DSP当前时间：RTC
//读取 RTC,并以min为单位存于RTC_Time
//==================================================== 
void DSP_Update_RTC(void)
{ 
	delay_ms(5);
	RTC_Hour = eeprom_read(0x30); 
	RTC_Min = eeprom_read(0x31);  
	RTC_Second = eeprom_read(0x32);  
	#if(DEBUG_LEVEL == ENABLE)
	RTC_Time= (RTC_Hour*24+RTC_Min)*60+RTC_Second;
	SendToGSMDebug((char *)"DSP_Update_RTC\r\n"); 
	sprintf(strtest,"DSP_RTC  H=%d, M=%d,S=%d\r\n",RTC_Hour,RTC_Min,RTC_Second);//,Time=%d,RTC_Time
	SendToGSMDebug(strtest);
	#endif

}
//==================================================== 
//Read DSP当前时间：RTC
//读取 RTC,并以min为单位存于RTC_Time
//==================================================== 
/*
void MCU_Update_RTC(void)
{ 
	delay_ms(5);
	eeprom_write(0x30,RTC_Hour); 
	eeprom_write(0x31,RTC_Min);  
	eeprom_write(0x32,RTC_Second);  
}
*/
//==================================================== 
//Read Daily Report时间：
//==================================================== 
void Read_DailyReportTime(void)
{ 
	delay_ms(5);
	DR_Hour = eeprom_read(0x2A); 
	DR_Min = eeprom_read(0x2B);  
	DR_OnOff = eeprom_read(0x2C);  
	DR_Flag = DR_OnOff;
	if((RTC_Hour>DR_Hour)||(RTC_Hour==DR_Hour&&RTC_Min>=DR_Min))
		DR_Flag = 0;
	#if(DEBUG_LEVEL == ENABLE)
	SendToGSMDebug((char *)"\r\nRead_DailyReportTime\r\n"); 
	sprintf(strtest,"\r\nDR OnOff=%d, Hour=%d, Min=%d\r\n",DR_OnOff,DR_Hour,DR_Min);
	SendToGSMDebug(strtest);
	#endif

}

//==================================================== 
//Read Regular Wake Up时间：
//==================================================== 
void Read_RegularWakeUpTime(void)
{ 
	delay_ms(5);
	RWU_OnOff = eeprom_read(0x33);  
	RWU_Hour = eeprom_read(0x34); 
	RWU_Min = eeprom_read(0x35);  
	RWU_Flag = RWU_OnOff;
	if((RTC_Hour>RWU_Hour)||(RTC_Hour==RWU_Hour&&RTC_Min>=RWU_Min))
		RWU_Flag = 0;
	#if(DEBUG_LEVEL == ENABLE)
	SendToGSMDebug((char *)"\r\nRead_RegularWakeUpTime\r\n"); 
	sprintf(strtest,"\r\nRWU OnOff=%d, H=%d, M=%d\r\n",RWU_OnOff,RWU_Hour,RWU_Min);
	SendToGSMDebug(strtest);
	#endif
}

//==================================================== 
//Read Regular Wake Up时间：
//==================================================== 
void Read_GameCallTime(void)
{ 
	delay_ms(5);
	GC_OnOff = eeprom_read(0x36);  
	GC_Hour = eeprom_read(0x37); 
	GC_Min = eeprom_read(0x38);  
	GC_Flag = GC_OnOff;
	if((RTC_Hour>GC_Hour)||(RTC_Hour==GC_Hour&&RTC_Min>=GC_Min))
		GC_Flag = 0;
	#if(DEBUG_LEVEL == ENABLE)
	SendToGSMDebug((char *)"\r\nRead_GameCallTime\r\n"); 
	sprintf(strtest,"\r\nGC OnOff=%d, Hour=%d, Min=%d\r\n",GC_OnOff,GC_Hour,GC_Min);
	SendToGSMDebug(strtest);
	#endif

}

//==================================================== 
//Read PIR 工作参数：PIR Interval
//读取PIR Interval,并以s为单位存于PIR_Interval_Set
// min：1～60；Sec：128～187；其余默认为60s
//==================================================== 
//V36_LXY03:SMS change parameter
char Read_PIR_Interval(void)
{
	char ret = FALSE;
	delay_ms(5);
	temp = eeprom_read(0x19);      
	if(temp<61)                
	{
		if(PIR_Interval_Set != temp*60)
		{
			ret = TRUE;
		}
		PIR_Interval_Set = temp*60;
	}
	else
	{
		if(temp>= 0x80)
		{ 
			if(PIR_Interval_Set != temp-0x80)
			{
				ret = TRUE;
			}
			PIR_Interval_Set = temp-0x80;
			if(PIR_Interval_Set>59)
			{
				PIR_Interval_Set = 60;
			}
		}
		else
		{
			if(PIR_Interval_Set != 60)
			{
				ret = TRUE;
			}
			PIR_Interval_Set = 60;	
		}      
	}
	if(PIR_Interval_Set == 0)                 
	{
		PIR_Interval_Set = 2;
	} 	
	return ret;
}

//==================================================== 
//Read Time Lapse 工作参数：Time Lapse
//读取 TimeLaplse,Min: <64(5min, 10min,...,55min); Hour: >64 (1hour,2hour,...8hour）
// M:0~60; H:65~72; 其余默认60s
//==================================================== 
//V36_LXY03:SMS change parameter
char Read_Time_Lapse(void)
{
	char ret;
	delay_ms(5);
	temp = eeprom_read(0x28);      
	if(temp< 64)
	{
		if(temp == Time_Lapse_Set)
		{
			ret = FALSE;
		}
		else
		{
			ret = TRUE;
		}
		Time_Lapse_Set = temp; 
	}
	else
	{
		if((temp>= 65)&(temp<=72))
		{
			if(Time_Lapse_Set == (temp-64)*60)	
			{
				ret = FALSE;
			}
			else
			{
				ret = TRUE;
			}
			Time_Lapse_Set = (temp-64)*60;
		}
		else
		{
			Time_Lapse_Set = 0;
			ret = FALSE;
		}
	} 
	#if(DEBUG_LEVEL == ENABLE)
	SendToGSMDebug((char *)"\r\nRead_TimeLapse\r\n"); 
	sprintf(strtest,"\r\nTime_Lapse_Set=%d\r\n",Time_Lapse_Set);
	SendToGSMDebug(strtest);
	#endif
	return ret;
}
//V30_LXY
//==================================================== 
//Read Work Hour 工作参数：
//读取 WorkHour,
//==================================================== 
void Read_Work_Hour(void)
{
	delay_ms(5);
	Work_Hour_Set = eeprom_read(0x23);      
	if(Work_Hour_Set == 1)
	{
		WH_StartHour=eeprom_read(0x24);
		WH_StartMin=eeprom_read(0x25);
		WH_StopHour=eeprom_read(0x26);
		WH_StopMin=eeprom_read(0x27); 

		Work_Hour_Start=WH_StartHour*60+WH_StartMin;
		Work_Hour_Stop=WH_StopHour*60+WH_StopMin;  
		
		if(Work_Hour_Stop<Work_Hour_Start)
		{
			FL_Night2Day = 1;
			Work_Hour_Stop += 1440;//24*60
		}
	}
	else if(Work_Hour_Set >= 2)
	{
		Work_Hour_Set = 0;
	} 
}
//==================================================== 
//Read PIR 工作参数：Time_Lapse（定时间隔时间）
//读取PIR Time_Lapse,并以s为单位存于PIR_In_Set
// M:0~60; H:65~72; 其余默认60s
//====================================================				
void Set_Time_Lapse_Start(void)
{
	temp = eeprom_read(0x28);    
	//V30_LXY040:
	//V38_LXY02:Not take the first photo of time lapse.
	Time_Lapse_Start=RTC_Hour*60+RTC_Min+1;
	eeprom_write(0x44,RTC_Min+1);
	eeprom_write(0x43,RTC_Hour);
	Time_Lapse_Flag = 0;//V38_LXY02: Deal with not take the first photo when enter arm again.
	Time_Lapse_EN = TRUE;
	Time_Lapse_Update_Flag = FALSE;	//V42_LXY01:Deal with Power up twice when update time.
	Time_Lapse_Update_EN = TRUE;
	#if(DEBUG_LEVEL == ENABLE)
	sprintf(strtest,"\r\n%dh:%dm:%ds,Start=%dm\r\n",RTC_Hour,RTC_Min,RTC_Second,Time_Lapse_Start);
	SendToGSMDebug(strtest);	
	#endif

}


//==================================================== 
//Read PIR 工作参数：Time_Lapse（定时间隔时间）
//读取PIR Time_Lapse,并以s为单位存于PIR_In_Set
// M:0~60; H:65~72; 其余默认60s
//====================================================				
void Read_Arm_Mode(void)
{
	delay_ms(5);
	temp = eeprom_read(0xC1);       

	if(temp==0x01)
	{
		FL_ARM=1;	
		SendToGSMDebug((char *)"\r\nArm....\r\n"); 
	}
	else if(temp==0)
	{
		FL_ARM=0;
		SendToGSMDebug((char *)"\r\nDisarm....\r\n"); 
	} 
}

//==================================================== 
//Read PIR 工作参数：Time_Lapse（定时间隔时间）
//读取PIR Time_Lapse,并以s为单位存于PIR_In_Set
// M:0~60; H:65~72; 其余默认60s
//====================================================				
void Write_Arm_Mode(void)
{
	delay_ms(5);
	temp = eeprom_write(0xC1,0x01);       
	FL_ARM=1;	
	SendToGSMDebug((char *)"\r\nArm....\r\n"); 
}
//==================================================== 
//Read WorkMode工作参数：Work Mode（定时间隔时间）
//读取Work Mode, 0: Huntting, 1: Security
//====================================================				

void Read_Work_Mode(void)
{
	delay_ms(5);
	temp = eeprom_read(0x42);       

	if(temp==0x01)
	{
		DSP_MODE=MODE_ARM_INDOOR;	
		SendToGSMDebug((char *)"\r\nIndoor....\r\n"); 
	}
	else if(temp==0)
	{
		DSP_MODE=MODE_ARM_OUTDOOR;
		SendToGSMDebug((char *)"\r\nOutdoor....\r\n"); 
	} 
}

//==================================================== 
//Read PIR 工作参数：PIR_Trigger_Set
//LOW  =  0，NORMAL = 1，HIGH = 2，OFF=3 默认为NORMAL
//====================================================
void Read_PIR_Trigger(void)
{
	delay_ms(5);
	PIR_Trigger_Set  =  eeprom_read(0x1E);   
	if(PIR_Trigger_Set>3) PIR_Trigger_Set  =  0x01;  
}

//V17_LXY01:
//==================================================== 
//Read SMS Control 工作参数：SMS_Control
//OFF  =  0，ON = 1 默认为OFF
//====================================================
void Read_SMS_Control(void)
{
	delay_ms(5);
	SMS_Control_Set  =  eeprom_read(0x39);   
	if(SMS_Control_Set>1) SMS_Control_Set  =  0x00;  
	if((MODE==MODE_ARM)&&(SMS_Control_Set  ==  0x00))
		GSM_PWR_EN = 0;
}

//==================================================== 
//Red Led灯闪烁
//====================================================
void PirLedFlash(void)
{
	PirLedFlk();
}

void BmcRespond_RFCtrl(void)
{
	unsigned char Data433buf[3]={0};
	En_Zone0=0,En_Zone1=0,En_Zone2=0,En_Zone3=0,En_Zone4=0,En_Zone5=0,En_Zone6=0,En_Zone7=0;
	if((g_wDecode433MH[0]==g_wDecode433MH[1]) && (g_bDecode433ML[0]==g_bDecode433ML[1]))	
	{	
		Data433buf[0]=(g_wDecode433MH[0]>>8)&0xff;
		Data433buf[1]=g_wDecode433MH[0]&0xff;
		Data433buf[2]=g_bDecode433ML[0];
		//rch
		Para_M433Zone[0] = eeprom_read(0x80);
		Para_M433Zone[1] = eeprom_read(0x81);
		Para_M433Zone[2] = eeprom_read(0x82);
		En_Zone0 = eeprom_read(0xB0);
		//zone1
		Para_M433Zone[3] = eeprom_read(0x83);
		Para_M433Zone[4] = eeprom_read(0x84);
		Para_M433Zone[5] = eeprom_read(0x85);	
		En_Zone1 = eeprom_read(0xB1);
		//zone2
		Para_M433Zone[6] = eeprom_read(0x86);
		Para_M433Zone[7] = eeprom_read(0x87);
		Para_M433Zone[8] = eeprom_read(0x88);	
		En_Zone2 = eeprom_read(0xB2);
		//zone3
		Para_M433Zone[9] = eeprom_read(0x89);
		Para_M433Zone[10] = eeprom_read(0x8A);
		Para_M433Zone[11] = eeprom_read(0x8B);	
		En_Zone3 = eeprom_read(0xB3);
		//zone4
		Para_M433Zone[12] = eeprom_read(0x8C);
		Para_M433Zone[13] = eeprom_read(0x8D);
		Para_M433Zone[14] = eeprom_read(0x8E);	
		En_Zone4 = eeprom_read(0xB4);
		//zone5
		Para_M433Zone[15] = eeprom_read(0x8F);
		Para_M433Zone[16] = eeprom_read(0x90);
		Para_M433Zone[17] = eeprom_read(0x91);	
		En_Zone5 = eeprom_read(0xB5);
		//zone6
		Para_M433Zone[18] = eeprom_read(0x92);
		Para_M433Zone[19] = eeprom_read(0x93);
		Para_M433Zone[20] = eeprom_read(0x94);	
		En_Zone6 = eeprom_read(0xB6);
		//zone7
		Para_M433Zone[21] = eeprom_read(0x95);
		Para_M433Zone[22] = eeprom_read(0x96);
		Para_M433Zone[23] = eeprom_read(0x97);	
		En_Zone7 = eeprom_read(0xB7);
		SendToGSMDebug((char *)"BmcRespond_RFCtrl\r\n");
		if(En_Zone0==1)
		{
			if(Para_M433Zone[0]==Data433buf[0]&&Para_M433Zone[1]==Data433buf[1]\
				&&Data433buf[2]==1)  //Disarm
			{
				if(DSP_MODE==MODE_ARM_INDOOR)//只允许在indoor内控制
				{
					State_mode =0xA7; // 0xA1;94
					delay_ms(10);
					eeprom_write(0xC1,0);	//set arm 
					FL_ARM=0;
					Camera_ON();            //开机
					Camera_OFF(); 
					delay_ms(500);				
					SendToGSMDebug((char *)"\r\nDISARM\r\n");
				}
			}
			else if(Para_M433Zone[0]==Data433buf[0]&&Para_M433Zone[1]==Data433buf[1]\
				&&Data433buf[2]==2) 		//B
			{
					State_mode =0xAC; // 0xA1;94
					delay_ms(10);
					SendToGSMDebug((char *)"Laser \r\n");
					Camera_ON();            //开机
					PirLedOff();	
					Camera_OFF(); 
					delay_ms(100);
			}
			else if(Para_M433Zone[0]==Data433buf[0]&&Para_M433Zone[1]==Data433buf[1]\
				&&Data433buf[2]==4) 		//Arm
			{
				if(DSP_MODE==MODE_ARM_INDOOR) //只允许在indoor内控制
				{
					State_mode =0xA7; // 0xA1;94
					delay_ms(10);
					eeprom_write(0xC1,1);	//set arm 
					FL_ARM=1;
					Camera_ON();            //开机
					Camera_OFF(); 
					delay_ms(500);				
					SendToGSMDebug((char *)"ARM TIME\r\n");
				}
			}
			else if(Para_M433Zone[0]==Data433buf[0]&&Para_M433Zone[1]==Data433buf[1]\
				&&Data433buf[2]==8) //Call
			{
				if(DSP_MODE==MODE_ARM_INDOOR)//只允许在indoor内控制
				{
					IOCIE=0;
					State_mode =0xA4; // 0xA1;94
					delay_ms(10);
					SendToGSMDebug((char *)"ARM Call\r\n");
					Camera_ON();            //开机
					PirLedOff();	
					Camera_OFF(); 
					delay_ms(100);
					IOCBF3=0;
					IOCIE=1;
				}
			}
		}
		if(DSP_MODE==MODE_ARM_INDOOR)
		{
			if(En_Zone1==1)
			{
				if(Para_M433Zone[3]==Data433buf[0]&&Para_M433Zone[4]==Data433buf[1]\
				&&Para_M433Zone[5]==Data433buf[2])  //zone 1
				{
					IOCIE=0;
					State_mode =0xA6;  //send zone sms
					eeprom_write(0xC5,0X01);
					delay_ms(10);
					SendToGSMDebug((char *)"zone1 sms\r\n");
					Camera_ON();            //开机
					PirLedOff();	
					Camera_OFF(); 
					delay_ms(100);
					IOCBF3=0;
					IOCIE=1;
				}
			}
			if(En_Zone2==1)
			{
				if(Para_M433Zone[6]==Data433buf[0]&&Para_M433Zone[7]==Data433buf[1]\
				&&Para_M433Zone[8]==Data433buf[2])  //zone 2
				{
					IOCIE=0;
					State_mode =0xA6;  //send zone sms
					eeprom_write(0xC5,0X02);
					delay_ms(10);
					SendToGSMDebug((char *)"zone2 sms\r\n");
					Camera_ON();            //开机
					PirLedOff();	
					Camera_OFF(); 
					delay_ms(100);
					IOCBF3=0;
					IOCIE=1;			
				}
			}
			if(En_Zone3==1)
			{
				if(Para_M433Zone[9]==Data433buf[0]&&Para_M433Zone[10]==Data433buf[1]\
				&&Para_M433Zone[11]==Data433buf[2])  //zone 3
				{
					IOCIE=0;
					State_mode =0xA6;  //send zone sms
					eeprom_write(0xC5,0X03);
					delay_ms(10);
					SendToGSMDebug((char *)"zone3 sms\r\n");
					Camera_ON();            //开机
					PirLedOff();	
					Camera_OFF(); 
					delay_ms(100);
					IOCBF3=0;
					IOCIE=1;				
				}
			}
			if(En_Zone4==1)
			{
				if(Para_M433Zone[12]==Data433buf[0]&&Para_M433Zone[13]==Data433buf[1]\
				&&Para_M433Zone[14]==Data433buf[2])  //zone 4
				{
					IOCIE=0;
					State_mode =0xA6;  //send zone sms
					eeprom_write(0xC5,0X04);
					delay_ms(10);
					SendToGSMDebug((char *)"zone4 sms\r\n");
					Camera_ON();            //开机
					PirLedOff();	
					Camera_OFF(); 
					delay_ms(100);
					IOCBF3=0;
					IOCIE=1;				
				}
			}			
			if(En_Zone5==1)
			{
				if(Para_M433Zone[15]==Data433buf[0]&&Para_M433Zone[16]==Data433buf[1]\
				&&Para_M433Zone[17]==Data433buf[2])  //zone 5
				{
					IOCIE=0;
					State_mode =0xA6;  //send zone sms
					eeprom_write(0xC5,0X05);
					delay_ms(10);
					SendToGSMDebug((char *)"zone5 sms\r\n");
					Camera_ON();            //开机
					PirLedOff();	
					Camera_OFF(); 
					delay_ms(100);
					IOCBF3=0;
					IOCIE=1;			
				}
			}			
			if(En_Zone6==1)
			{
				if(Para_M433Zone[18]==Data433buf[0]&&Para_M433Zone[19]==Data433buf[1]\
				&&Para_M433Zone[20]==Data433buf[2])  //zone 6
				{
					IOCIE=0;
					State_mode =0xA6;  //send zone sms
					eeprom_write(0xC5,0X06);
					delay_ms(10);
					SendToGSMDebug((char *)"zone6 sms\r\n");
					Camera_ON();            //开机
					PirLedOff();	
					Camera_OFF(); 
					delay_ms(100);
					IOCBF3=0;
					IOCIE=1;		
				}
			}			
			if(En_Zone7==1)
			{
				if(Para_M433Zone[21]==Data433buf[0]&&Para_M433Zone[22]==Data433buf[1]\
				&&Para_M433Zone[23]==Data433buf[2])  //zone 7
				{
					IOCIE=0;
					State_mode =0xA6;  //send zone sms
					eeprom_write(0xC5,0X07);
					delay_ms(10);
					SendToGSMDebug((char *)"zone7 sms\r\n");
					Camera_ON();            //开机
					PirLedOff();	
					Camera_OFF(); 
					delay_ms(100);
					IOCBF3=0;
					IOCIE=1;			
				}
			}			
		}
			
		g_bRFEnable=RF_DETECT_FAIL;
	}

}

void BmcRFprocessStd(void)
{
	unsigned char Data433buf[3]={0};
	if((g_wDecode433MH[0]==g_wDecode433MH[1]) && (g_bDecode433ML[0]==g_bDecode433ML[1]))	
	{
		Data433buf[0]=(g_wDecode433MH[0]>>8)&0xff;
		Data433buf[1]=g_wDecode433MH[0]&0xff;
		Data433buf[2]=g_bDecode433ML[0];		

		if(Studyrch==0xB0)
		{	  
			eeprom_write(0x80,Data433buf[0]);   //set rch code
			eeprom_write(0x81,Data433buf[1]);   
			eeprom_write(0x82,Data433buf[2]);   
			eeprom_write(0xB0,0x01);   		//set rch 
			eeprom_write(0xC0,0x66);  		//echo ok
			
		}else
		if(Studyrch==0xB1)
		{	
			eeprom_write(0x83,Data433buf[0]);   		//set ZONE1 code
			eeprom_write(0x84,Data433buf[1]);   
			eeprom_write(0x85,Data433buf[2]);   
			eeprom_write(0xB1,0x01);   		//set ZONE 1	
			eeprom_write(0xC0,0x66);  		//echo ok
		}else
		if(Studyrch==0xB2)
		{	  
			eeprom_write(0x86,Data433buf[0]);   		//set ZONE2 code
			eeprom_write(0x87,Data433buf[1]);   
			eeprom_write(0x88,Data433buf[2]);   
			eeprom_write(0xB2,0x01);   		//set ZONE 2	
			eeprom_write(0xC0,0x66);  		//echo ok
		}else
		if(Studyrch==0xB3)
		{	  
			eeprom_write(0x89,Data433buf[0]);   		//set ZONE3 code
			eeprom_write(0x8A,Data433buf[1]);   
			eeprom_write(0x8B,Data433buf[2]);   
			eeprom_write(0xB3,0x01);   		//set ZONE 3	
			eeprom_write(0xC0,0x66);  		//echo ok
		}else
		if(Studyrch==0xB4)
		{	  
			eeprom_write(0x8C,Data433buf[0]);   		//set ZONE4 code
			eeprom_write(0x8D,Data433buf[1]);   
			eeprom_write(0x8E,Data433buf[2]);   
			eeprom_write(0xB4,0x01);   		//set ZONE 4	
			eeprom_write(0xC0,0x66);  		//echo ok
		}else
		if(Studyrch==0xB5)
		{	  
			eeprom_write(0x8F,Data433buf[0]);   		//set ZONE5 code
			eeprom_write(0x90,Data433buf[1]);   
			eeprom_write(0x91,Data433buf[2]);   
			eeprom_write(0xB5,0x01);   		//set ZONE5	
			eeprom_write(0xC0,0x66);  		//echo ok
		}else
		if(Studyrch==0xB6)
		{	  
			eeprom_write(0x92,Data433buf[0]);   		//set ZONE6 code
			eeprom_write(0x93,Data433buf[1]);   
			eeprom_write(0x94,Data433buf[2]);   
			eeprom_write(0xB6,0x01);   		//set ZONE 6	
			eeprom_write(0xC0,0x66);  		//echo ok
		}else
		if(Studyrch==0xB7)
		{	  
			eeprom_write(0x95,Data433buf[0]);   		//set ZONE7 code
			eeprom_write(0x96,Data433buf[1]);   
			eeprom_write(0x97,Data433buf[2]);   
			eeprom_write(0xB7,0x01);   		//set ZONE 7	
			eeprom_write(0xC0,0x66);  		//echo ok
		}

		g_bRFEnable=RF_DETECT_FAIL;
	}

}

void BmcRFprocessDel(void)
{
	WDT_Clear(); 
	if(Studyrch==0xB8)
	{
		eeprom_write(0x80,0x00);   		//set rch code
		eeprom_write(0x81,0x00);   
		eeprom_write(0x82,0x00);   
		eeprom_write(0xB0,0x00);   		//set rch 1
		eeprom_write(0xC0,0x66);  		//echo ok
	}
	if(Studyrch==0xB9)
	{
		eeprom_write(0x83,0x00);   		//set ZONE code
		eeprom_write(0x84,0x00);   
		eeprom_write(0x85,0x00);   
		eeprom_write(0xB1,0x00);   		//set ZONE 1
		eeprom_write(0xC0,0x66);  		//echo ok
	}
	if(Studyrch==0xBA)
	{
		eeprom_write(0x86,0x00);   		//set ZONE code
		eeprom_write(0x87,0x00);   
		eeprom_write(0x88,0x00);   
		eeprom_write(0xB2,0x00);   		//set ZONE 2
		eeprom_write(0xC0,0x66);  		//echo ok
	}	
	if(Studyrch==0xBB)
	{
		eeprom_write(0x89,0x00);   		//set ZONE code
		eeprom_write(0x8A,0x00);   
		eeprom_write(0x8B,0x00);   
		eeprom_write(0xB3,0x00);   		//set ZONE 3
		eeprom_write(0xC0,0x66);  		//echo ok
	}
	if(Studyrch==0xBC)
	{
		eeprom_write(0x8C,0x00);   		//set ZONE code
		eeprom_write(0x8D,0x00);   
		eeprom_write(0x8E,0x00);   
		eeprom_write(0xB4,0x00);   		//set ZONE 4
		eeprom_write(0xC0,0x66);  		//echo ok
	}
	if(Studyrch==0xBD)
	{
		eeprom_write(0x8F,0x00);   		//set ZONE code
		eeprom_write(0x90,0x00);   
		eeprom_write(0x91,0x00);   
		eeprom_write(0xB5,0x00);   		//set ZONE 5
		eeprom_write(0xC0,0x66);  		//echo ok
	}	
	if(Studyrch==0xBE)
	{
		eeprom_write(0x92,0x00);   		//set ZONE code
		eeprom_write(0x93,0x00);   
		eeprom_write(0x94,0x00);   
		eeprom_write(0xB6,0x00);   		//set ZONE 6
		eeprom_write(0xC0,0x66);  		//echo ok
	}	
	if(Studyrch==0xBF)
	{
		eeprom_write(0x95,0x00);   		//set ZONE code
		eeprom_write(0x96,0x00);   
		eeprom_write(0x97,0x00);   
		eeprom_write(0xB7,0x00);   		//set ZONE 7
		eeprom_write(0xC0,0x66);  		//echo ok
	}		
}


void BuzzerSound(void)
{
/*	char n;
	for(n=0;n<140;n++)
	{
		BeepSound=0;
		delay_us(1);
		BeepSound=1;	
		delay_us(1);
	}*/
}

/////////GSM初始化////////////////////


unsigned char BmcGSMPowerOn(void)
{
	GSM_PWR_KEY=1;
	GSM_PWR_EN=1; 
	delay_ms(900);
	if(ONOFF==0) 
	{
		return 1;
	}	
	GSM_PWR_KEY=0;
	GSM_PWR_EN=0; 
	delay_ms(900);	
	GSM_PWR_KEY=1;
	GSM_PWR_EN=1; 
	GSM_DTR=0;//wake up
	return 1;
}

unsigned char BmcGSMPowerOff(void)
{
	GSM_PWR_EN=0; 	//gsm power off
//	GSM_PWR_KEY=0;
	GSM_DTR=0;
	return 1 ;
}

void BmcGSMSleep(void)
{
	if((FL_SIM_INSERT == 1)&&(((MODE==MODE_ARM)&&(SMS_Control_Set  ==  0x01))||(DSP_MODE==MODE_ARM_INDOOR)))//V20_LXY01:
		GSM_DTR=1;	//V17_LXY04:
}

void BmcGSMWake(void)
{
	GSM_DTR=0;	//V17_LXY04:
}

////////////////////////////

void RFIN(void) 
{ 
          
  if(g_bRFEnable==RF_DETECT_FAIL)
    {
      		short_k=0;
	  	 CLRWDT(); 
		while(RF_IN==1&& short_k<650 ) //800
		{
			delayus(3);
			short_k++;
		}
		while(RF_IN==0&& head_k<2500) //8000
		{
			delayus(3);
			head_k++;
		}
      if(((short_k<<5)<head_k) && (head_k<((short_k<<5)+(short_k<<3)))&&(head_k>1000))   
         { 
			g_bDecode433ML[0]=0;g_wDecode433MH[0]=0;g_bDecode433ML[1]=0;g_wDecode433MH[1]=0;	
	   		   for(rep=0;rep<2;rep++)
	          {		                  
        	   for(ik=0;ik<12;ik++)
        	     {
        		   for(k=0;k<2;k++)
        		      {        	 	  	   
					    j=0;
						while(RF_IN==1 && j<2000) 
						{
							delayus(4);
							j++;
						} 
        	 	  	    if((j>(short_k>>3))&&(j<((short_k>>1)+short_k))) 
						{
						RFCOM1[k]=0; 
						}     
        	  	  	    else if((j>(short_k<<1))&&(j<(short_k<<2))) 
						{
						RFCOM1[k]=1;
						}
        			    else 
						{
							short_k=0; head_k=0; 
							g_bRFEnable=RF_DETECT_FAIL; 
							return;
						}        	
        		   	    j=0;
						while(RF_IN==0 && j<8000)//10000
						{
						delayus(2);
						j++; 
						} 
						j=0;     
        		      }				  
				   if((RFCOM1[0]==0)&&(RFCOM1[1]==1))
               		  {
        				//if(ik<8) g_wDecode433MH[rep]|=(0x02<<((7-ik)*2)); 
        				if(ik<8) g_wDecode433MH[rep]|=(0x02<<((7-ik)<<1)); 
					  }
        	       else if((RFCOM1[0]==1)&&(RFCOM1[1]==1))
        	          {
        				//if(ik<8) g_wDecode433MH[rep]|=(0x01<<((7-ik)*2)); 	
        				if(ik<8) g_wDecode433MH[rep]|=(0x01<<((7-ik)<<1)); 
        				else g_bDecode433ML[rep]|=(0x01<<11-ik);										  	 
 					  }	
	               else if((RFCOM1[0]==0)&&(RFCOM1[1]==0))
				     {
                       if(ik<8) g_wDecode433MH[rep]&=~(0x00<<((7-ik)*2));  		
        			   else g_bDecode433ML[rep]&=~(0x00<<11-ik);	
					 }
        		 } 
       	   
			   if(ik==12) {g_bRFEnable++; }
			   if(g_bRFEnable==RF_DETECT1_SUC) {g_bRFEnable=RF_DETECT_SUCC; return;}			   
			   j=0;
				while(RF_IN==1 && (j<700))//800
				{
				delayus(3);
				j++;
				}            			   
			   head_k=0;
				while(RF_IN==0 && head_k<2500) //8000
				{
				delayus(3);
				head_k++;
				}
			   if((head_k<(short_k<<5)) || ( head_k>((short_k<<5)+(short_k<<3))))  
				{
				head_k=0;short_k=0;
				g_bRFEnable=RF_DETECT_FAIL; 
				return;
				} 
	         } 
         }
		 else
		 {
			head_k=0;short_k=0;
			g_bRFEnable=RF_DETECT_FAIL; 
			return;
		 } 		  
	}  
}

void BmcWriteVersion(void)
{
	eeprom_write(0x70,0x11);
	eeprom_write(0x71,0x6d);
	eeprom_write(0x72,0x70);
	eeprom_write(0x73,0x42);
	eeprom_write(0x74,0x4f);
	eeprom_write(0x75,0x4c);
	eeprom_write(0x76,0x59);
	eeprom_write(0x77,0x12);
	eeprom_write(0x78,0x53);
	eeprom_write(0x79,0x47);
	eeprom_write(0x7a,0x35);
	eeprom_write(0x7b,0x35);
	eeprom_write(0x7c,0x30);
	eeprom_write(0x7d,0x4d);
}

void BmcOutdoorCountTime(void)
{
	Timer_cnt++;
	if(Timer_cnt>=28)////32ms计秒时为28
	{
		Timer_cnt=0;
		PIR_Interval_Counter++;  
		FL_CountTimeLapse = TRUE;	//V44_LXY: Deal with sometimes not take photo
		RTC_Second++;
		if(RTC_Second>= 60)
		{ 
			RTC_Min++;
			if(FL_ReScanGSM == 1)
			{
				ScanGSMMinCount ++;
			}
			RTC_Second = 0;           
			wTempCount = RTC_Min%8;
			if(wTempCount == 0)
			{
				CLRWDT(); 
				Detect_Temperature();
			}
			//V42_LXY01:Deal with Power up twice when update time.
			if(Time_Lapse_EN==FALSE)
			{
				Time_Lapse_Counter++;
				if(Time_Lapse_Counter>=2)
				{
					Time_Lapse_EN=TRUE;
				}
			}
			if(Time_Lapse_Update_EN== FALSE)
			{
				Time_Lapse_Update_Count++;
				if(Time_Lapse_Update_Count>=3)
				{
					Time_Lapse_Update_EN=TRUE;
				}
			}
		}
		if(RTC_Min>= 60)
		{ 
			RTC_Hour++;
			RTC_Min = 0;

			//V36_LXY02: Restart GSM when GSM abnormal.
			ReScanGSMHour++;
			if((SMS_Control_Set==1)&&(ReScanGSMHour>=5))
			{
				FL_ReScanGSM=1;
				ReScanGSMHour = 0;
			}

			if((SMS_Control_Set==1)&&(RTC_Hour == 1)&&(FL_RestartGSM_EN == 1))
			{
				FL_RestartGSM=1;
				FL_RestartGSM_EN = 0;
			}
		}
		if(RTC_Hour>= 24)          
		{
			RTC_Hour = 0;
			DR_Flag=DR_OnOff;  
			RWU_Flag=RWU_OnOff;   
			GC_Flag=GC_OnOff;
			FL_RestartGSM_EN = 1;
		}
	}

}


void BmcIndoorCountTime(void)
{
	if(Count_Time>=61)
	{
		Count_Time=0;										
		PIR_Interval_Counter++;  
		FL_CountTimeLapse = TRUE;	//V44_LXY: Deal with sometimes not take photo
		RTC_Second++;
		if(RTC_Second>= 60)
		{ 
			RTC_Min++;
			RTC_Second = 0;           
			wTempCount = RTC_Min%8;
			if(wTempCount == 0)
			{
				WDT_Clear();
				Detect_Temperature();
			}
			//V42_LXY01:Deal with Power up twice when update time.
			if(Time_Lapse_EN==FALSE)
			{
				Time_Lapse_Counter++;
				if(Time_Lapse_Counter>=2)
				{
					Time_Lapse_EN=TRUE;
				}
			}
			if(Time_Lapse_Update_EN== FALSE)
			{
				Time_Lapse_Update_Count++;
				if(Time_Lapse_Update_Count>=3)
				{
					Time_Lapse_Update_EN=TRUE;
				}
			}
		}
		if(RTC_Min>= 60)
		{ 
			RTC_Hour++;
			RTC_Min = 0;
		}
		if(RTC_Hour>= 24)          
		{
			RTC_Hour = 0;
			DR_Flag=DR_OnOff;   
			DR_Flag=DR_OnOff;  
			RWU_Flag=RWU_OnOff;   
			GC_Flag=GC_OnOff;

		}
	}
}

void BmcCountTimeLapse(void)
{
	if((Time_Lapse_Set!=0)&&(RTC_Second>=5)&&(RTC_Second<=55)&&(FL_CountTimeLapse == TRUE))//V44_LXY: Deal with sometimes not take photo
	{  
		FL_CountTimeLapse = FALSE;//V44_LXY: Deal with sometimes not take photo
		if(Time_Lapse_Set<60)
		{
			RTC_Time=RTC_Hour*60+RTC_Min;
			Current_Time=RTC_Time;
			if(RTC_Time>=Time_Lapse_Start)
				RTC_Time=RTC_Time-Time_Lapse_Start;
			else
				RTC_Time=RTC_Time+24*60-Time_Lapse_Start;
			if(RTC_Time%Time_Lapse_Set==0)
			{
				Time_Lapse_Flag = TRUE;
				#if(DEBUG_LEVEL == ENABLE)
				sprintf(strtest,"\r\nMCU_RTC=%d:%d:%d\r\n",RTC_Hour,RTC_Min,RTC_Second);
				SendToGSMDebug(strtest);
				#endif
			}
			//V40_LXY020: Update time for time lapse.
			else if(RTC_Time%Time_Lapse_Set==(Time_Lapse_Set-(Time_Lapse_Set/10)-1))
			{
				Time_Lapse_Update_Flag = TRUE;
				#if(DEBUG_LEVEL == ENABLE)
				sprintf(strtest,"\r\nMCU_RTC=%d:%d:%d\r\n",RTC_Hour,RTC_Min,RTC_Second);
				SendToGSMDebug(strtest);
				#endif
			}
			//V42_LXY010: Update time for time lapse to some special set.
			else if(1440%Time_Lapse_Set!=0)
			{
				if(Time_Lapse_Start>((Time_Lapse_Set/10)+1))
				{
					if(Current_Time==(Time_Lapse_Start-((Time_Lapse_Set/10)+1)))
					{
						Time_Lapse_Update_Flag = TRUE;
					}
				}
				else
				{
					if(Current_Time==(1440-((Time_Lapse_Set/10)+1)+Time_Lapse_Start))
					{
						Time_Lapse_Update_Flag = TRUE;
					}
				}
				//V44_LXY020:Deal with time lapse bug.
				if(Time_Lapse_Start>5)
				{
					if(Current_Time==(Time_Lapse_Start-5))
					{
						Time_Lapse_Update_Flag = TRUE;
					}
				}
				else
				{
					if(Current_Time==(1440-5+Time_Lapse_Start))
					{
						Time_Lapse_Update_Flag = TRUE;
					}
				}

			}
			else
			{
				Time_Lapse_Flag = FALSE;
			}
		}
		else
		{

			RTC_Time=RTC_Hour*60+RTC_Min;
			Current_Time=RTC_Time;
			if(RTC_Time>=Time_Lapse_Start)
				RTC_Time=RTC_Time-Time_Lapse_Start;
			else
				RTC_Time=RTC_Time+24*60-Time_Lapse_Start;
			if(((RTC_Time%Time_Lapse_Set==0)))
	//		if(((RTC_Time%Time_Lapse_Set==0)||(RTC_Time%10==0))&&RTC_Second>=2&&RTC_Second<=4)
			{
				Time_Lapse_Flag = TRUE;
				#if(DEBUG_LEVEL == ENABLE)
				sprintf(strtest,"\r\nMCU_RTC=%d:%d:%d\r\n",RTC_Hour,RTC_Min,RTC_Second);
				SendToGSMDebug(strtest);
				#endif
			}
			else if((RTC_Time%Time_Lapse_Set==(Time_Lapse_Set-(Time_Lapse_Set/12)))||(RTC_Time%Time_Lapse_Set==(Time_Lapse_Set-5)))
			{
				Time_Lapse_Update_Flag = TRUE;
				#if(DEBUG_LEVEL == ENABLE)
				sprintf(strtest,"\r\nMCU_RTC=%d:%d:%d\r\n",RTC_Hour,RTC_Min,RTC_Second);
				SendToGSMDebug(strtest);
				#endif
			}
			//V42_LXY010: Update time for time lapse to some special set.
			else if(1440%Time_Lapse_Set!=0)
			{
				if(Time_Lapse_Start>((Time_Lapse_Set/10)+1))
				{
					if(Current_Time==(Time_Lapse_Start-((Time_Lapse_Set/10)+1)))
					{
						Time_Lapse_Update_Flag = TRUE;
					}
				}
				else
				{
					if(Current_Time==(1440-((Time_Lapse_Set/10)+1)+Time_Lapse_Start))
					{
						Time_Lapse_Update_Flag = TRUE;
					}
				}
				//V42_LXY020:Deal with time lapse bug.
				if(Time_Lapse_Start>5)
				{
					if(Current_Time==(Time_Lapse_Start-5))
					{
						Time_Lapse_Update_Flag = TRUE;
					}
				}
				else
				{
					if(Current_Time==(1440-5+Time_Lapse_Start))
					{
						Time_Lapse_Update_Flag = TRUE;
					}
				}
			}
			else
			{
				Time_Lapse_Flag = FALSE;
			}
		}
		//V44_LXY: Deal with take photo more than need.
		if(Time_Lapse_Update_EN == FALSE)
			Time_Lapse_Update_Flag = FALSE;
		if(Time_Lapse_EN == FALSE)
			Time_Lapse_Flag = FALSE;
	} 
}


//V48_LXY010:Restart GSM Function. 
char BmcGSMStart(unsigned char  StartStatus) //StartStatus: 0:Start 1:First Start 
{
	unsigned char LEDFlash;
	GSM_PWR_EN = 0;//V48_LXY010: Restart GSM model.
	Read_SMS_Control();
	eeprom_write(0xC6,0);//V48_LXY020:DSP ask MCU restart GSM
	eeprom_write(0xC3,0);//V48_LXY010:Initial SIM status.
	FL_RestartGSM = 0;
	ReScanGSMHour = 0;
	if(StartStatus == 0)
	{
		for(LEDFlash=0;LEDFlash<20;LEDFlash++)
		{
			delay_ms(1000);
		}
	}
	else
	{
		delay_ms(1000);
	}
	if(SMS_Control_Set == 1)
	{
		State_mode =0xAD;  //Ask DSP init GSM.
		SendToGSMDebug((char *)"BmcEnterArmIndicate\r\n");
		delay_ms(500);
		if((ONOFF==0)||(MODE==MODE_TEST)) return 0;    //V34_LXY02: Deal with restart.
		Power_ON();
		delay_ms(500);
		//GSM_PWR_EN = 1;//V48_LXY010:
		P_OFF=0;
		for(LEDFlash=0;LEDFlash<250;LEDFlash++)   //GPS add
		{
			if(StartStatus == 1&&LEDFlash%2==0)
			{
				PirLedFlk();	
			}
			delay_ms(500);
			if(P_OFF==1)
			{
				//GSM_PWR_KEY = 0;		
				break;
			}
			//V48_LXY010:
			if((eeprom_read(0xC3)==0x01))//Read SIM status insert.
			{
				FL_SIM_INSERT = 1;
				GSM_PWR_EN = 1;
//				GSM_PWR_KEY = 0;
			}
			else
			{
				FL_SIM_INSERT = 0;
				GSM_PWR_EN = 0;
			}
			

			//V34_LXY02:Deal with not start from arm to test.
			if(ONOFF==0) return 0;    
			if(MODE==MODE_TEST)
			{
				Power_OFF();
				delay_ms(1000);
				return 0;
			}
		}
		//PirLedOff();
		PB3_RI = 0;
		if(eeprom_read(0xC3)==0x01)
		{
			//V20_LXY01:
			FL_SIM_INSERT = 1;
			BmcGSMSleep();
		}
		else
		{
			//V20_LXY01:
			FL_SIM_INSERT = 0;
			BmcGSMWake(); 
			GSM_PWR_EN = 0;
		}
	}
	else if(StartStatus == 1)
	{
		GSM_PWR_EN = 0;
		delay_ms(1000);
		if((ONOFF==0)||(MODE==MODE_TEST)) return 0;    //V34_LXY02: Deal with restart.
		for(LEDFlash=0;LEDFlash<9;LEDFlash++)
		{
			PirLedFlk();	
			delay_ms(1000);
			//V34_LXY02:Deal with not start from arm to test.
			if(ONOFF==0) return 0;    
			if(MODE==MODE_TEST)
			{
				Power_OFF();
				delay_ms(1000);
				return 0;
			}
		}
	}
	return 1;
}
char BmcEnterArmIndicate(void)
{
	char Flashtime;
	//V17_LXY02:
	PirLedOn();
	Power_OFF();
	Read_SMS_Control();

	delay_ms(1000);
	PirLedFlk();
	if(Frist_PowerON)    //首次上电
	{  
		WDT_Clear();
		Frist_PowerON = 0;	
		PWM_OUT = 1;
		wTempCount = 0;
		while(wTempCount < 100)
		{
			delay_ms(100); 
			ADC_change(PIR_AD);
			if(ADRESH < 40)
			break;
			SendToGSMDebug((char *)"PIR Charge!\r\n");
			wTempCount++;
			if(wTempCount%10 == 0)
				 PirLedFlk();
		}
		ADC_Stop();//V22_LXY01: Save ADC power
		PWM_OUT = 0;
	}
	BmcGSMStart(1);
	//CAMERA_TYPE=eeprom_read(0x1F);
	BmcReadCameraInfo();
	Power_OFF();
	PirLedFlk();	
	delay_ms(1000);

	//V40_LXY06: Uniform update time.
	State_mode =0xAE;  //Ask DSP update time.
	P_OFF = 0;
	if((ONOFF==0)||(MODE==MODE_TEST)) return 0; 
	Power_ON();
	for(Flashtime=0;Flashtime<4;Flashtime++)
	{
		if(ONOFF==0) return 0;    
		if(MODE==MODE_TEST)
		{
			Power_OFF();
			delay_ms(1000);
			return 0;
		}
		PirLedFlk();	
		delay_ms(1000);
	}
	Power_OFF();
	PirLedOff();
	delay_ms(1000);
	return 0;
}

void CheckDSPStartup(void)
{
	//V30_LXY060:
	unsigned short StartupCount;
	//V34_LXY02:Deal with not start from arm to test.
	if(Configure_FlashType ==  FlashType_Nand)
	{
		if(Configure_433M == 1)
		{
			StartupCount = 200;
		}
		else if(Configure_433M == 0)
		{
			StartupCount = 5000;
		}
		else
		{
			StartupCount = 5000;
		}
		if(FL_StartOK == FALSE&&Restart_Count<=StartupCount)
		{
			Restart_Count++;
		}
		else if(FL_StartOK == FALSE && Restart_Count>StartupCount)
		{
			Power_OFF();
			delay_ms(1000);
			Power_ON();
			Restart_Count =0;
		}
	}
	else if(Configure_FlashType ==  FlashType_SPI)
	{
		if(Configure_433M == 1)
		{
			StartupCount = 1600;
		}
		else if(Configure_433M == 0)
		{
			StartupCount = 40000;
		}
		else
		{
			StartupCount = 40000;
		}
		if(FL_StartOK == FALSE&&Restart_Count<=StartupCount)
		{
			Restart_Count++;
		}
		else if(FL_StartOK == FALSE && Restart_Count>StartupCount)
		{
			Power_OFF();
			delay_ms(1000);
			Power_ON();
			Restart_Count =0;
		}


	}
	#if(DEBUG_LEVEL == ENABLE)
	//sprintf(strtest,"\r\nStartupCount=%d,Restart_Count=%d\r\n",StartupCount,Restart_Count);
	//SendToGSMDebug(strtest);
	#endif

}
//=========================================================//
//Camera_Info:  bit7	bit6		bit5			bit4		bit3	bit2	bit1	bit0
//				1		reserve		FlashType		433M		|<---- PowerOff	Time------->|
//=========================================================//
void BmcReadCameraInfo(void)
{
	unsigned char Camera_Info=0,Camera_Info_Bake=1;
	while(Camera_Info != Camera_Info_Bake)
	{
		Camera_Info=eeprom_read(0x1F);
		delay_ms(500);
		Camera_Info_Bake=eeprom_read(0x1F);
	}

	if(Camera_Info&0x80 == 0x80)
	{
		Configure_433M = (Camera_Info>>4)&0x01;
		Configure_PowerOff =  Camera_Info&0x0F;
		Configure_FlashType =  (Camera_Info>>5)&0x01;
	}
	else
	{
		Configure_433M = ENABLE;
		Configure_PowerOff =  8;
		Configure_FlashType =  FlashType_SPI;
	}
	#if(DEBUG_LEVEL == ENABLE)
	sprintf(strtest,"\r\nCamera_Info=%x\r\n",Camera_Info);
	SendToGSMDebug(strtest);
	#endif
	

}


unsigned char bmcReadValueFromADC(void)
{
    unsigned char tep_adc[TEP_ADC_SAMPLE_MAX];
    int   tep_avg=0,tep_value_avg=0;
	unsigned char   fail_adc_cnt=0;
	unsigned char   i;
    
	
    for(i=0;i<TEP_ADC_SAMPLE_MAX+1;i++)
	{
	   ADC_change(Light_AD);
       tep_adc[i] = ADRESH;
    	#if(DEBUG_LEVEL == ENABLE)
    	sprintf(strtest,"\r\ntep_adc[%d] = %d\r\n",i,tep_adc[i]);
    	SendToGSMDebug(strtest);
    	#endif
        delay_ms(1);
	   
	   tep_avg+=tep_adc[i];
	}
	tep_avg = (tep_avg-tep_adc[0])>>3;
	#if(DEBUG_LEVEL == ENABLE)
	sprintf(strtest,"tep_avg = %d\r\n",tep_avg);
	SendToGSMDebug(strtest);
	#endif

	//检测不合格样本
	
	for(i=1;i<TEP_ADC_SAMPLE_MAX+1;i++)
    {
		if(abs(tep_adc[i]-tep_avg)>20)
			fail_adc_cnt++;
		else
			tep_value_avg+=tep_adc[i];
    }
	if(fail_adc_cnt>TEP_ADC_BAD_SAMPMAX)
	{
		//return 0xFF;
    }
    tep_value_avg/=(TEP_ADC_SAMPLE_MAX-fail_adc_cnt);
    
    return  tep_avg;
}

unsigned char bmcReadLightFromADC(void)
{
    unsigned char Light_value_avg = 0;
	
	static unsigned char Pre_Light_level = 0;
	static unsigned char Light_level_flag = 0;
	
    unsigned char Light_level = 0;
    unsigned char nCnt = 0;
    LIGHT_CTRL = 0;
    delay_ms(60);
 	#if (DEBUG_LEVEL == ENABLE)
	sprintf(strtest,"RA = 0\r\n");
	SendToGSMDebug(strtest);
	#endif

    Light_value_avg = bmcReadValueFromADC();

    if(Light_value_avg<aLight_Low[6][0])
    {
        LIGHT_CTRL = 1;
        delay_ms(20);

     	#if (DEBUG_LEVEL == ENABLE)
    	sprintf(strtest,"RA = 1\r\n");
    	SendToGSMDebug(strtest);
    	#endif
        
       Light_value_avg = bmcReadValueFromADC();
        do
    	{
    		if(Light_value_avg >= aLight_High[nCnt][0])
    		{
    			Light_level = aLight_High[nCnt][1];
    			break;
    		}
    		else
    		{
    			Light_level = 15;
    		}
    	}while(aLight_High[++nCnt][0] != 0);


    }
	else			
	{
    	do
    	{
    		if(Light_value_avg >= aLight_Low[nCnt][0])
    		{
    			Light_level = aLight_Low[nCnt][1];
    			break;
    		}
    		else
    		{
    			Light_level = 6;
    		}
    	}while(aLight_Low[++nCnt][0] != 0);
	} 
    ADC_Stop();	
    Light_AD_Value = Light_value_avg;
    Light = Light_level;

	if((Light_level_flag == 1)&&(Light_AD_Value>=(aLight_Low[1][0]-Light_AD_Offset)&&Light_AD_Value<=(aLight_Low[1][0]+Light_AD_Offset)))
	{	
		Light = Pre_Light_level;
	}
	
	if(Light_AD_Value>=(aLight_Low[1][0]-Light_AD_Offset)&&Light_AD_Value<=(aLight_Low[1][0]+Light_AD_Offset))
	{
		Pre_Light_level = Light;
		Light_level_flag = 1;
	}
	else
	{
		Light_level_flag = 0;
	}

	#if (DEBUG_LEVEL == ENABLE)
	sprintf(strtest,"V = %d L =%d\r\n",Light_value_avg,Light_level);
	SendToGSMDebug(strtest);
	#endif

    return Light_level;
}

///////////END GSMNORMAL//////////////////////////////
