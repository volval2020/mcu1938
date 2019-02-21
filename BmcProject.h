#ifndef __BMCPROJECT_H
//#define __BMCPROJECT_H
//#include <stdio.h>
//#include <htc.h>
//#include <string.h>
//#include".\BMCGSM\GSM.h"
//#include<pic.h>
//#include".\I2C\I2C.h"
////////////GSM/////////
#define GSM_PWR_KEY		RB0
#define GSM_PWR_EN		RC0//RB1
#define GSM_DTR			RB2
#define GSM_RI			RB3
#define GSM_STATUS		RC2//RB4
#define	LIGHT_CTRL		RA5

////////////LED SHOW/////////
#define BLED		RB6
#define GLED		RB7
//////////PIR///////////
#define Light_AD			0	//RA0	Light
#define PIR_AD			1	//RA1	PIR
#define Temperature_AD  	2	//RA2	Temperature
#define BAT_AD  	11	//RB4	Temperature


#define PWM_OUT 			RA4
#define DSP_MCU_COM 		RC5
/////////DSP///////////
#define  Power_ON()		    RA6 = 1
#define  Power_OFF()		RA6 = 0
#define  MODE				RA3

//////////LASER////////
#define  LASER			RC0           //// 1:OFF, 0:ON
//#define  PIR_LED			RB2           //// 1:OFF, 0:ON
#define  PirLedOn()           RC1 = 0
#define  PirLedOff()          RC1 = 1
#define  PirLedFlk()          RC1 ^= 1

#define  DbgPirLedOff()       RC1 = 0
#define  DbgPirLedOn()        RC1 = 1
#define  DbgPirLedFlk()       RC1 ^= 1


//////////COM  SELECT////////

#define USART_DSP   		 1
#define USART_MCU		0
////////////USB CONTROL//////////
//#define USBDETECT			RC1

////// I2C module uses PORT C///////
//#define SDA     		RC4                //RA2     /* data on port A bit 1 */
//#define SCL             RC3          //RA4  /* clock on port A bit 2 */
//#define	SDA_DIR		TRISC4
//#define	SCL_DIR		TRISC3

///////////////RFIN 433/////////////
#define	RF_IN		RC5
//#define	RF_ON		RB5
//////////BMC CAMERA TYPE////////

#define MG982K   		 1
#define MG882K		2
#define MG983K   		 3//V40_LXY: Compatible MG983K
//#define CAMERA_TYPE	MG882K	//CS Add Project Flag
//////////variable/////////
#define  Trigger_delay  		2
#define  Flash_ChargeTime	180               //闪光间隔充电时间设置
#define  FLASH_EN
#define 	ON					1
#define	OFF				0

/////////////////MCU //////////////
#define  SYS_WUP_KEY	 			RA7 //INTO TEST MODE
#define  SYS_25_18650	 			RB4  //INTO TEST MODE
#define  SYS_26_DC	 			RB5 //INTO TEST MODE

//#define Shut_433	 			RB1//RA7
#define  ONOFF	 			RB1//RC1  //CONTROL POWER ON OR OFF

/////////////REGISTER//////////
#define RAIF	IOCIF	//
#define RAIE	IOCIE      //电平中断允许位
#define CMCON0	CM1CON0	
#define IOCA2	IOCAF2

///////////////debug////////////////
#define MODE_TEST 		1
#define MODE_ARM	 		0
#define MODE_ARM_INDOOR	 		2
#define MODE_ARM_OUTDOOR	 		3
#define MODE_POWER_OFF		4

/////////////////////433////////////////////
#define RF_DETECT_FAIL 	0
#define RF_DETECT1_SUC	2
#define RF_DETECT_SUCC	4
//********************************************************************
#define BMC_PIR_SG560P      1
#define BMC_PIR_SG550M      2
#define BMC_PIR_TYPE        BMC_PIR_SG550M
//********************************************************************
#define FlashType_Nand	0
#define FlashType_SPI	1  
void Init_GPIO(void);
void ADC_change(char AN);
void Camera_ON(void);
void Camera_OFF(void);
void Read_PIR_Trigger(void);
void BmcWriteVersion(void);
void BmcIndoorCountTime(void);
void BmcOutdoorCountTime(void);
void BmcCountTimeLapse(void);
char BmcEnterArmIndicate(void);
void BmcReadCameraInfo(void);
char Read_Time_Lapse(void);
char Read_PIR_Interval(void);
void Read_Work_Mode(void);
void Read_Update_RTC(void);
void Read_Adjust_RTC(void);
void Read_DailyReportTime(void);
void Read_RegularWakeUpTime(void);
void Read_GameCallTime(void);
void PIR_scout(void);
void FlashCapacitance_Charge(char ik);
void Detect_Battery(void);
void Detect_Temperature(void);
char I2C_error(void);
void PirLedFlash(void);
void I2C_Slave_Init(void);
void Read_Work_Day(void);
void Set_Time_Lapse_Start(void);
void DSP_Update_RTC(void);
//void MCU_Update_RTC(void);
void SendToGSM(char *pstring);
void SendToGSMDebug(char *pstring);
void BmcRespond_RFCtrl(void);
void BmcRFprocessStd(void);
void BmcRFprocessDel(void);
void Read_Arm_Mode(void);

void BuzzerSound(void);

/////////////////////gsm//////////////////////

unsigned char BmcGSMPowerOn(void);
unsigned char BmcGSMPowerOff(void);
void BmcGSMSleep(void);
void BmcGSMWake(void);

void RFIN(void) ;
///////////////////////////////////////////////////////////////////
#endif

