#ifndef UART_H
#define UART_H

#include <mc9s08ac60.h>

/*
** constants and macros
*/

/** Size of the circular receive buffer, must be power of 2 */
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE 128
#endif

#ifndef UART2_RX_BUFFER_SIZE
#define UART2_RX_BUFFER_SIZE 128
#endif

/** Size of the circular transmit buffer, must be power of 2 */
#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE 128
#endif

#ifndef UART2_TX_BUFFER_SIZE
#define UART2_TX_BUFFER_SIZE 128
#endif

/*
** high byte error return code of uart_getc()
*/
#define UART_FRAME_ERROR      0x0800              /* Framing Error by UART       */
#define UART_OVERRUN_ERROR    0x0400              /* Overrun condition by UART   */
#define UART_BUFFER_OVERFLOW  0x0200              /* receive ringbuffer overflow */
#define UART_NO_DATA          0x0100              /* no receive data available   */

/*
** high byte error return code of uart_getc()
*/
#define UART2_FRAME_ERROR      0x0800              /* Framing Error by UART       */
#define UART2_OVERRUN_ERROR    0x0400              /* Overrun condition by UART   */
#define UART2_BUFFER_OVERFLOW  0x0200              /* receive ringbuffer overflow */
#define UART2_NO_DATA          0x0100              /* no receive data available   */


/*
** function prototypes
*/

/**
   @brief   Initialize UART and set baudrate
   @param   void
   @return  none
*/
extern void uart_init(void);


/**
 *  @brief   Get received byte from ringbuffer
 *
 * Returns in the lower byte the received character and in the
 * higher byte the last receive error.
 * UART_NO_DATA is returned when no data is available.
 *
 *  @param   void
 *  @return  lower byte:  received byte from ringbuffer
 *  @return  higher byte: last receive status
 *           - \b 0 successfully received data from UART
 *           - \b UART_NO_DATA
 *             <br>no receive data available
 *           - \b UART_BUFFER_OVERFLOW
 *             <br>Receive ringbuffer overflow.
 *             We are not reading the receive buffer fast enough,
 *             one or more received character have been dropped
 *           - \b UART_OVERRUN_ERROR
 *             <br>Overrun condition by UART.
 *             A character already present in the UART UDR register was
 *             not read by the interrupt handler before the next character arrived,
 *             one or more received characters have been dropped.
 *           - \b UART_FRAME_ERROR
 *             <br>Framing Error by UART
 */
extern unsigned int uart_getc(void);


/**
 * @brief   non-block Get received byte from ringbuffer
 * Returns in the lower byte the received character and in the
 * higher byte the last receive error.
 * UART_NO_DATA is returned when no data is available.
 *
 *  @param   void
 *  @return  input char
 */
extern unsigned char uart_ngetc(unsigned char *dest, unsigned char len);


/**
 *  @brief   Put byte to ringbuffer for transmitting via UART
 *  @param   data byte to be transmitted
 *  @return  none
 */
extern void uart_putc(unsigned char data);



/**
 *  @brief   non-block Put byte to ringbuffer for transmitting via UART
 *  @param   data byte to be transmitted
 *  @return  none
 */
extern unsigned char uart_nputc(unsigned char *source, unsigned char len);


/**
 *  @brief   Put string to ringbuffer for transmitting via UART
 *
 *  The string is buffered by the uart library in a circular buffer
 *  and one character at a time is transmitted to the UART using interrupts.
 *  Blocks if it can not write the whole string into the circular buffer.
 *
 *  @param   s string to be transmitted
 *  @return  none
 */
extern void uart_puts(const char *s );


/**
 *  @brief   non-block Put string to ringbuffer for transmitting via UART
 *
 *  The string is buffered by the uart library in a circular buffer
 *  and one character at a time is transmitted to the UART using interrupts.
 *  Blocks if it can not write the whole string into the circular buffer.
 *
 *  @param   s string to be transmitted
 *  @return  none
 */
extern void uart_nputs(const char *s );


/**
 * @brief    Put string from program memory to ringbuffer for transmitting via UART.
 *
 * The string is buffered by the uart library in a circular buffer
 * and one character at a time is transmitted to the UART using interrupts.
 * Blocks if it can not write the whole string into the circular buffer.
 *
 * @param    s program memory string to be transmitted
 * @return   none
 * @see      uart_puts_P
 */
extern void uart_puts_p(const char *s );

/**
 * @brief    Macro to automatically put a string constant into program memory
 */
#define uart_puts_P(__s)       uart_puts_p(PSTR(__s))

/**
 *  @brief   Return number of bytes waiting in the receive buffer
 *  @param   none
 *  @return  bytes waiting in the receive buffer
 */
extern int uart_available(void);

/**
 *  @brief   Flush bytes waiting in receive buffer
 *  @param   none
 *  @return  none
 */
extern void uart_flush(void);


/** @brief  Initialize USART1 (only available on selected ATmegas) @see uart_init */
extern void uart2_init(unsigned int baudrate);
/** @brief  Get received byte of USART1 from ringbuffer. (only available on selected ATmega) @see uart_getc */
extern unsigned int uart2_getc(void);
/** @brief  Put byte to ringbuffer for transmitting via USART1 (only available on selected ATmega) @see uart_putc */
extern void uart2_putc(unsigned char data);
/** @brief  Put string to ringbuffer for transmitting via USART1 (only available on selected ATmega) @see uart_puts */
extern void uart2_puts(const char *s );
/** @brief  Put string from program memory to ringbuffer for transmitting via USART1 (only available on selected ATmega) @see uart_puts_p */
extern void uart2_puts_p(const char *s );
/** @brief  Macro to automatically put a string constant into program memory */
#define uart2_puts_P(__s)       uart2_puts_p(PSTR(__s))
/** @brief   Return number of bytes waiting in the receive buffer */
extern int uart2_available(void);
/** @brief   Flush bytes waiting in receive buffer */
extern void uart2_flush(void);

/**@}*/


#endif // UART_H
