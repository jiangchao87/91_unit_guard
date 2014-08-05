#include "uart.h"


/*
 *  constants and macros
 */

/* size of RX/TX buffers */
#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1)
#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1)

#define UART2_RX_BUFFER_MASK ( UART2_RX_BUFFER_SIZE - 1)
#define UART2_TX_BUFFER_MASK ( UART2_TX_BUFFER_SIZE - 1)

#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
#error UART1 RX buffer size is not a power of 2
#endif
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
#error UART1 TX buffer size is not a power of 2
#endif

#if ( UART2_RX_BUFFER_SIZE & UART2_RX_BUFFER_MASK )
#error UART2 RX buffer size is not a power of 2
#endif
#if ( UART2_TX_BUFFER_SIZE & UART2_TX_BUFFER_MASK )
#error UART2 TX buffer size is not a power of 2
#endif

#define UART0_RECEIVE_INTERRUPT     interrupt 17 void UART0_Rx_INT(void)
#define UART0_TRANSMIT_INTERRUPT    interrupt 18 void UART0_Tx_INT(void)

#define UART1_RECEIVE_INTERRUPT     interrupt 17 void UART1_Rx_INT(void)
#define UART1_TRANSMIT_INTERRUPT    interrupt 18 void UART1_Tx_INT(void)

#define UART2_RECEIVE_INTERRUPT     interrupt 20 void UART2_Rx_INT(void)
#define UART2_TRANSMIT_INTERRUPT    interrupt 21 void UART2_Tx_INT(void)


/*
 *  module global variables
 */
static volatile unsigned char UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART_RxBuf[UART_RX_BUFFER_SIZE];
static volatile unsigned char UART_TxHead;
static volatile unsigned char UART_TxTail;
static volatile unsigned char UART_RxHead;
static volatile unsigned char UART_RxTail;
static volatile unsigned char UART_LastRxError;

static volatile unsigned char UART2_TxBuf[UART2_TX_BUFFER_SIZE];
static volatile unsigned char UART2_RxBuf[UART2_RX_BUFFER_SIZE];
static volatile unsigned char UART2_TxHead;
static volatile unsigned char UART2_TxTail;
static volatile unsigned char UART2_RxHead;
static volatile unsigned char UART2_RxTail;
static volatile unsigned char UART2_LastRxError;


UART1_RECEIVE_INTERRUPT
/*************************************************************************
Function: UART Receive Complete interrupt
Purpose:  called when the UART has received a character
**************************************************************************/
{
    unsigned char tmphead;
    unsigned char data;
    unsigned char usr;
    unsigned char lastRxError;


    /* read UART status register and UART data register */
    usr  = SCI1S1;
    data = SCI1D;

    /* */
    lastRxError = (usr & 0x0F);

    /* calculate buffer index */
    tmphead = ( UART_RxHead + 1) & UART_RX_BUFFER_MASK;

    if ( tmphead == UART_RxTail ) {
        /* error: receive buffer overflow */
        lastRxError = UART_BUFFER_OVERFLOW >> 8;
    }else{
        /* store new index */
        UART_RxHead = tmphead;
        /* store received data in buffer */
        UART_RxBuf[tmphead] = data;
    }
    UART_LastRxError = lastRxError;
}


UART1_TRANSMIT_INTERRUPT
/*************************************************************************
Function: UART Data Register Empty interrupt
Purpose:  called when the UART is ready to transmit the next byte
**************************************************************************/
{
    unsigned char tmptail;


    if ( UART_TxHead != UART_TxTail) {
        /* calculate and store new buffer index */
        tmptail = (UART_TxTail + 1) & UART_TX_BUFFER_MASK;
        UART_TxTail = tmptail;
        /* get one byte from buffer and write it to UART */
        SCI1D = UART_TxBuf[tmptail];  /* start transmission */
    }else{
        /* tx buffer empty, disable UDRE interrupt */
    	
    }
}


UART2_RECEIVE_INTERRUPT
/*************************************************************************
Function: UART2 Receive Complete interrupt
Purpose:  called when the UART2 has received a character
**************************************************************************/
{
    unsigned char tmphead;
    unsigned char data;
    unsigned char usr;
    unsigned char lastRxError;


    /* read UART2 status register and UART2 data register */
    usr  = SCI2S1;
    data = SCI2D;

    /* */
    lastRxError = (usr & 0x0F);

    /* calculate buffer index */
    tmphead = ( UART2_RxHead + 1) & UART2_RX_BUFFER_MASK;

    if ( tmphead == UART2_RxTail ) {
        /* error: receive buffer overflow */
        lastRxError = UART_BUFFER_OVERFLOW >> 8;
    }else{
        /* store new index */
        UART2_RxHead = tmphead;
        /* store received data in buffer */
        UART2_RxBuf[tmphead] = data;
    }
    UART2_LastRxError = lastRxError;
}


UART2_TRANSMIT_INTERRUPT
/*************************************************************************
Function: UART2 Data Register Empty interrupt
Purpose:  called when the UART2 is ready to transmit the next byte
**************************************************************************/
{
    unsigned char tmptail;


    if ( UART2_TxHead != UART2_TxTail) {
        /* calculate and store new buffer index */
        tmptail = (UART2_TxTail + 1) & UART2_TX_BUFFER_MASK;
        UART2_TxTail = tmptail;
        /* get one byte from buffer and write it to UART2 */
        SCI2D = UART2_TxBuf[tmptail];  /* start transmission */
    }else{
        /* tx buffer empty, disable Uart2 TCIE interrupt */
    	SCI1C2_TCIE = 0;
    }
}


/*************************************************************************
Function: uart_init()
Purpose:  initialize UART and set baudrate
Input:    none
Returns:  none
**************************************************************************/
void uart_init(void)
{
    UART_TxHead = 0;
    UART_TxTail = 0;
    UART_RxHead = 0;
    UART_RxTail = 0;

    UART2_TxHead = 0;
    UART2_TxTail = 0;
    UART2_RxHead = 0;
    UART2_RxTail = 0;

	/* Uart1 */
	SCI1C2 = 0x00;						// TIE=0 TCIE=0 RIE=0 ILIE=0, TE=0 RE=0 RW=0 SBK=0
	(void)(SCI1S1 == 0);				/* Dummy read of the SCI1S1 register to clear flags */
	(void)(SCI1D == 0);					/* Dummy read of the SCI1D register to clear flags */
	SCI1S2 = 0x00;
    /* Set baud rate 9600 */
    SCI1BD = 0x68;
    /* Set control byte 1 */
    SCI1C1 = 0x00;
    /* Set control byte 2: TIE=0 TCIE=0 RIE=1 ILIE=0, TE=1 RE=1 RW=0 SBK=0 */
	SCI1C2 = 0x2C;
    /* Set control byte 3: R8=0 T8=0 TXDIR=0 TXINV=0 ORIE=0 NEIE=0 FEIE=0 PEIE=0 */
	SCI1C3 = 0x00; 
	
	/* Uart2 */
	/* Clear control byte 2: TIE=0 TCIE=0 RIE=0 ILIE=0, TE=0 RE=0 RW=0 SBK=0 */
	SCI2C2 = 0x00;
	/* Dummy read of the SCI1S1 register to clear flags */
	(void)(SCI1S1 == 0);				
	/* Dummy read of the SCI1D register to clear flags */
	(void)(SCI1D == 0);
	/* Clear status byte 2 */
	SCI2S2 = 0x00;
	/* Set baud rate 9600 */
	SCI2BD = 0x68;
	/* Set control byte 1 */
	SCI2C1 = 0x00;
	/* Set control byte 2: TIE=0 TCIE=0 RIE=1 ILIE=0, TE=1 RE=1 RW=0 SBK=0 */
	SCI2C2 = 0x2C;
	/* Set control byte 3: R8=0 T8=0 TXDIR=0 TXINV=0 ORIE=0 NEIE=0 FEIE=0 PEIE=0 */
	SCI2C3 = 0x00; 

}/* uart_init */


/*************************************************************************
Function: uart_send()
Purpose:  start send data in ringbuffer via UART
Input:    uart port to use
Returns:  0 --- transition starts OK
**************************************************************************/
int uart_send(unsigned char uart_num)
{
	switch (uart_num)
	{
	//case 0:
	//	SCI0C2_TCIE = 1;
	//	break;
	case 1:
		SCI1C2_TCIE = 1;
		break;
	case 2:
		SCI2C2_TCIE = 1;
		break;
	}
	return 0;
}



/*************************************************************************
Function: uart_getc()
Purpose:  return byte from ringbuffer
Returns:  lower byte:  received byte from ringbuffer
          higher byte: last receive error
**************************************************************************/
unsigned int uart_getc(void)
{
    unsigned char tmptail;


    if ( UART_RxHead == UART_RxTail ) {
        return UART_NO_DATA;   /* no data available */
    }

    /* calculate /store buffer index */
    tmptail = (UART_RxTail + 1) & UART_RX_BUFFER_MASK;
    UART_RxTail = tmptail;

    /* get data from receive buffer */
    return UART_RxBuf[tmptail];

}/* uart_getc */


/*************************************************************************
Function: uart_ngetc()
Purpose:  non-block return byte from ringbuffer
Returns:  char input
**************************************************************************/
unsigned char uart_ngetc(unsigned char *dest, unsigned char len)
{
	char i;
	unsigned int temp;
	
	if(uart_available() >= len)
	{
		for(i=0;i<len;i++)
		{
			temp = uart_getc();
			
			if((i == 0)&&((temp & 0x00FF) != 0xFA))
				{
					i--;								//check frame header 0xFA till we find it
					continue;
				}
			
			if(!(temp & 0xFF00))
				*(dest++) = (temp & 0x00FF);
			else
				return 0xFF;
		}
		return 0;
	}
	else
	{
		return 0xFF;
	}
	
}/* uart_ngetc */



/*************************************************************************
Function: uart_putc()
Purpose:  write byte to ringbuffer for transmitting via UART
Input:    byte to be transmitted
Returns:  none
**************************************************************************/
void uart_putc(unsigned char data)
{
    unsigned char tmphead;


    tmphead  = (UART_TxHead + 1) & UART_TX_BUFFER_MASK;

    while ( tmphead == UART_TxTail ){
        ;/* wait for free space in buffer */
    }

    UART_TxBuf[tmphead] = data;
    UART_TxHead = tmphead;

    /* enable Uart1 TCIE interrupt */
    SCI1C2_TCIE = 1;

}/* uart_putc */


/*************************************************************************
Function: uart_nputc()
Purpose:  non-block write byte to ringbuffer for transmitting via UART
Input:    byte to be transmitted
Returns:  none
**************************************************************************/
unsigned char uart_nputc(unsigned char *source, unsigned char len)
{
	
	return 0;
}/* uart_nputc */


/*************************************************************************
Function: uart_puts()
Purpose:  transmit string to UART
Input:    string to be transmitted
Returns:  none
**************************************************************************/
void uart_puts(const char *s )
{
    while (*s)
      uart_putc(*s++);

}/* uart_puts */


/*************************************************************************
Function: uart_nputs()
Purpose:  non-block transmit string to UART
Input:    string to be transmitted
Returns:  none
**************************************************************************/
void uart_nputs(const char *s )
{
    //while (*s)
      //uart_nputc(*s++);

}/* uart_nputs */


/*************************************************************************
Function: uart_available()
Purpose:  Determine the number of bytes waiting in the receive buffer
Input:    None
Returns:  Integer number of bytes in the receive buffer
**************************************************************************/
int uart_available(void)
{
    return (UART_RX_BUFFER_MASK + UART_RxHead - UART_RxTail) % UART_RX_BUFFER_MASK;
}/* uart_available */


/*************************************************************************
Function: uart_flush()
Purpose:  Flush bytes waiting the receive buffer.  Acutally ignores them.
Input:    None
Returns:  None
**************************************************************************/
void uart_flush(void)
{
    UART_RxHead = UART_RxTail;
}/* uart_flush */



/*************************************************************************
Function: uart2_getc()
Purpose:  return byte from ringbuffer
Returns:  lower byte:  received byte from ringbuffer
          higher byte: last receive error
**************************************************************************/
unsigned int uart2_getc(void)
{
    unsigned char tmptail;


    if ( UART2_RxHead == UART2_RxTail ) {
        return UART_NO_DATA;   /* no data available */
    }

    /* calculate /store buffer index */
    tmptail = (UART2_RxTail + 1) & UART2_RX_BUFFER_MASK;
    UART2_RxTail = tmptail;

    /* get data from receive buffer */
    return UART2_RxBuf[tmptail];

}/* uart2_getc */


/*************************************************************************
Function: uart2_ngetc()
Purpose:  non-block return byte from ringbuffer
Returns:  char input
**************************************************************************/
unsigned char uart2_ngetc(unsigned char *dest, unsigned char len)
{
	char i;
	unsigned int temp;
	
	if(uart2_available() >= 11)						//minimum package length from ARM
	{
		for(i=0;i<len;i++)
		{
			temp = uart2_getc();
			
			if((i == 0)&&((temp & 0x00FF) != 0xF5))
			{
				i--;								//check frame header 0xF5 till we find it
				continue;
			}

			if(!(temp & 0xFF00))
				*(dest++) = (temp & 0x00FF); 
			else
				return 0xFF;
			
			if((temp & 0x00FF) == 0xFD)				//frame tail 0xFD
				break;
		}
		return 0;
	}
	else
	{
		return 0xFF;
	}
	
}/* uart2_ngetc */


/*************************************************************************
Function: uart2_putc()
Purpose:  write byte to ringbuffer for transmitting via UART2
Input:    byte to be transmitted
Returns:  none
**************************************************************************/
void uart2_putc(unsigned char data)
{
    unsigned char tmphead;


    tmphead  = (UART2_TxHead + 1) & UART2_TX_BUFFER_MASK;

    while ( tmphead == UART2_TxTail ){
        ;/* wait for free space in buffer */
    }

    UART2_TxBuf[tmphead] = data;
    UART2_TxHead = tmphead;

    /* enable Uart2 TCIE interrupt */
    SCI2C2_TCIE = 1;

}/* uart2_putc */


/*************************************************************************
Function: uart2_nputc()
Purpose:  non-block write byte to ringbuffer for transmitting via UART2
Input:    byte to be transmitted
Returns:  none
**************************************************************************/
unsigned char uart2_nputc(unsigned char *source,unsigned char len)
{
	unsigned char i;

	if(source)
	{	
		for(i=0;i<len;i++)
		{
			uart2_putc(*(source++));
		}
		return 0;
	}
	else
		return 0xFF;
}/* uart2_nputc */


/*************************************************************************
Function: uart2_available()
Purpose:  Determine the number of bytes waiting in the receive buffer
Input:    None
Returns:  Integer number of bytes in the receive buffer
**************************************************************************/
int uart2_available(void)
{
    return (UART2_RX_BUFFER_MASK + UART2_RxHead - UART2_RxTail) % UART2_RX_BUFFER_MASK;
}/* uart2_available */


/*************************************************************************
Function: uart2_flush()
Purpose:  Flush bytes waiting the receive buffer.  Acutally ignores them.
Input:    None
Returns:  None
**************************************************************************/
void uart2_flush(void)
{
    UART2_RxHead = UART2_RxTail;
}/* uart2_flush */


/*************************************************************************
Function: uart2_puts()
Purpose:  transmit string to UART
Input:    string to be transmitted
Returns:  none
**************************************************************************/
void uart2_puts(const char *s )
{
    while (*s)
      uart2_putc(*s++);

}/* uart2_puts */


/*************************************************************************
Function: uart2_nputs()
Purpose:  non-block transmit string to UART
Input:    string to be transmitted
Returns:  none
**************************************************************************/
void uart2_nputs(const char *s )
{
    //while (*s)
      //uart_nputc(*s++);

}/* uart2_nputs */

