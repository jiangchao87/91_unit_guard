#include "uart.h"


/*
 *  constants and macros
 */

/* size of RX/TX buffers */
#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1)
#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1)

#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
#error RX buffer size is not a power of 2
#endif
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
#error TX buffer size is not a power of 2
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
static volatile unsigned char uart_frame_length;


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
    uart_frame_length++;

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


/*************************************************************************
Function: uart_init()
Purpose:  initialize UART and set baudrate
Input:    baudrate using macro UART_BAUD_SELECT()
Returns:  none
**************************************************************************/
void uart_init(void)
{
    UART_TxHead = 0;
    UART_TxTail = 0;
    UART_RxHead = 0;
    UART_RxTail = 0;
    uart_frame_length = 0;


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
	unsigned char i;
	unsigned int temp;
	
	if(uart_frame_length%4 == 0)
	{
		for(i=0;i<len;i++)
		{
			temp = uart_getc();
			
			if(temp == UART2_NO_DATA)
				return 0xFF;
			
			if((i==0)&&((temp & 0x00FF) != 0x005A))
				return 0xFF;
			
			if(!(temp & 0xFF00))
				*(dest++) = (temp & 0x00FF);
			else
				return 0xFF;
		}
	}
	else
		return 0xFF;							//frame is not over
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

    /* enable UDRE interrupt */
    //UART0_CONTROL    |= _BV(UART0_UDRIE);

}/* uart_putc */


/*************************************************************************
Function: uart_nputc()
Purpose:  non-block write byte to ringbuffer for transmitting via UART
Input:    byte to be transmitted
Returns:  none
**************************************************************************/
void uart_nputc(unsigned char data)
{
	return;
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
    while (*s)
      uart_nputc(*s++);

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
