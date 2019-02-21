#ifndef	_I2C_H_
#define _I2C_H_
#include<pic.h>

/*
 *	SDA (data) and SCL (clock) bits
 *	
 *	Special note!!!
 *	
 *	If the clock and data lines are in the same port, you will need
 *	to beware of the Read/Modify/Write issue in the PIC - since
 *	a bit set or clear on any one bit in a port will read and write
 *	back all other bits. 

 */

#ifndef _XTAL_FREQ
 // Unless specified elsewhere, 4MHz system frequency is assumed
 #define _XTAL_FREQ 4000000
#endif

/* Uncomment the next line to use the PIC's SSP Module*/
#define I2C_MODULE 1

#ifdef I2C_MODULE
/* I2C module uses PORT C */
#define SDA     		RC4                //RA2     /* data on port A bit 1 */
#define SCL             RC3          //RA4  /* clock on port A bit 2 */
#define	SDA_DIR		TRISC4
#define	SCL_DIR		TRISC3

#define SDA_1    		RC4=1
#define SCL_1   			RC3=1
#define SDA_0  		 	RC4=0
#define SCL_0  		 	RC3=0

#define SDA_OUT         TRISC4=0
#define SCL_OUT         TRISC3=0
#define SDA_IN   		TRISC4=1
#define SCL_IN   		TRISC3=1

#define I2CTRIS 		TRISC
#define MASTER_MODE     0B1011          /* I2C firmware controlled Master Mode (slave idle) */
#define SSPMode(val)   SSPCON &=0xF0; SSPCON|=(val & 0xf)
#endif
#define ack	    0
#define no_ack  1

char Receive_Address, Receive_Buffer;


#define M_SDA_INP	0x02
#define M_SDA_OUT   0xFD
#define M_SCL_INP   0x04
#define M_SCL_OUT	0xFB

#define I2C_INPUT	1		/* data direction input */
#define I2C_OUTPUT	0		/* data direction output */

#define I2C_READ	0x01		/* read bit used with address */
#define I2C_WRITE	0x00		/* write bit used with address */

#define FALSE		0
#define TRUE		1

#define I2C_ERROR	(-1)
#define I2C_LAST	FALSE		/* SendAck: no more bytes to send */
#define I2C_MORE	TRUE		/* SendAck: more bytes to send */

#define i2c_Start()		i2c_Restart()
#define i2c_WriteTo(address)	i2c_Open((address), I2C_WRITE)
#define i2c_ReadFrom(address)	i2c_Open((address), I2C_READ)

#ifdef I2C_MODULE
#define SCL_HIGH() SCL_DIR = I2C_INPUT
#define SCL_LOW()  SCL_DIR = I2C_OUTPUT
#define SDA_HIGH() SDA_DIR = I2C_INPUT
#define SDA_LOW()  SDA_DIR = I2C_OUTPUT
#else
#define SCL_HIGH() SCL = 1; SCL_DIR = I2C_OUTPUT
#define SCL_LOW()  SCL = 0; SCL_DIR = I2C_OUTPUT
#define SDA_HIGH() SDA = 1; SDA_DIR = I2C_OUTPUT
#define SDA_LOW()  SDA = 0; SDA_DIR = I2C_OUTPUT
#endif

/*
 * Timings for the i2c bus. Times are rounded up to the nearest
 * micro second.
 */

#define I2C_TM_BUS_FREE		5
#define	I2C_TM_START_SU		5
#define I2C_TM_START_HD		4
#define I2C_TM_SCL_LOW			5
#define	I2C_TM_SCL_HIGH		4
#define I2C_TM_DATA_SU			1
#define I2C_TM_DATA_HD          	0
#define I2C_TM_SCL_TO_DATA	4	/* SCL low to data valid */
#define	I2C_TM_STOP_SU			4
#define I2C_TM_SCL_TMO		10	/* clock time out */


extern  void I2C_SendByte(char shu);
extern  char I2C_ReceiveByte(void);
extern  void I2C_SendACK(char Sack);
extern  char I2C_ReceiveACK(void);
extern  void I2C_Stop(void);

#endif			/* _I2C_H_ */
