#include	<htc.h>
#include 	"i2c.h"

/*
 *	I2C functions for HI-TECH PIC C - master mode only
 */

/*
 * 	TIMING - see Philips document: THE I2C-BUS SPECIFICATION
 */

void I2C_Stop(void)
{
	char temp_m;
	temp_m=0x40;
	while(!SCL)    //�ȴ�������
	{ 
		temp_m--; 
		CLRWDT();
		if(temp_m==0) 
			break;
	}
	temp_m=0x40;
	CLRWDT();
	while(1)
	{
		if(SDA)    
			return;
		temp_m--;
		if(temp_m==0)
			break;   
	}
}
//================================================
//Receive a ACK=0 or NACK=1  from the master device
//*=================================================
char I2C_ReceiveACK(void)
{
	char temp_i,rt; 
	temp_i=0x40;
#ifdef __PIC16F1828_H
	WPUB=0b01010000;
#else
//	WPUC=0b11000000;
#endif
	while(!SCL)    //�ȴ�������
	{
		temp_i--;
		CLRWDT();
		if(temp_i==0) 
		{
			SDA_IN; 
			break;
		}
	}
	if(SDA)
	{
		rt=1;
	}
	else
	{
		rt=0;
	}
	temp_i=0x40;       
	while(SCL)     //�½���
	{
		temp_i--;
		CLRWDT();
		if(temp_i==0)
		{
			break;
		}
	} 
	return rt;
}
//==================================================
//Send a ACK to the master device
//================================================
void I2C_SendACK(char Sack)
{
	char temp_i; 
	temp_i=0x40;  
	
	while(SCL)     //�ȴ��½���
	{
		temp_i--;
		CLRWDT();
		if(temp_i==0) 
		{
			break;
		}
	} 
	if(!Sack)
	{
		SDA_0; 
	}
	else
	{
		SDA_1;
	}
	temp_i=0x40; 
	while(!SCL)	  //������
	{
		temp_i--;
		CLRWDT();
		if(temp_i==0)
		{
			break;
		}
	 } 
	SDA_OUT;
	NOP();
	NOP();
	NOP();
	NOP(); 
	NOP();
	NOP(); 
	temp_i=0x40;
//	while(SCL)    //�ȴ��½���
	while(SCL)
	{
		temp_i--;
		CLRWDT();
		if(temp_i==0)
		{ 
			break;
		}
	}   
	//ACK ���������ߵ�ƽ�ź�����
	SDA_IN;	    //�ָ�SDAΪ����״̬
//	 TRISA|=0x04;
#ifdef __PIC16F1828_H
	WPUB=0b00000000;
#else
//	WPUC=0x00;
#endif

}

//Receive a BYTE  from the master device
char  I2C_ReceiveByte(void)
{ 
	char Rec_temp=0,i,rectime;
#ifdef __PIC16F1828_H
	WPUB=0b01010000;
#else
//	WPUC=0b11000000;
#endif
	for(i=8;i>0;i--)
	{  
		rectime=0x40;
		while(SCL)         //�½���
		{
			rectime--;
			CLRWDT();
			if(rectime==0)
			{
				return 0;
			}
		}      
		rectime=0x40;
		while(!SCL)       //������
		{   
			rectime--;
			CLRWDT();
			if(rectime==0)
			{
				return 0;
			}
		} 
		NOP();
		Rec_temp <<= 1;
		if(SDA)
		{
			Rec_temp|=0x01;
		}
	}
	return Rec_temp;    
}
//Send a BYTE to the master device
 void I2C_SendByte(char shu)
{
	char i,temp;
	SDA_OUT; 
	for(i=0;i<8;i++)
	{
		temp=0x40;
		while(SCL)      // �½���   
		{
			temp--;
			CLRWDT();
			if(temp==0)     
				break;
		}   
		NOP(); 
		if((shu&0x80)>0)
		{
			SDA_1;
		}
		else
		{ 
			SDA_0;  
		}
		shu<<=1;            
		temp=0X40;
		while(!SCL)    //������
		{
			temp--;
			CLRWDT();
			if(temp==0)      
				break;
		}
		SDA_OUT;  
		NOP();     
	}
	temp=0X40; 
	while(SCL)     // �½���  
	{
		temp--;
		CLRWDT();
		if(temp==0)   
			break;
	}  
	SDA_IN;
#ifdef __PIC16F1828_H
	WPUB=0b00000000;
#else
//	WPUC=0x00;
#endif
}

