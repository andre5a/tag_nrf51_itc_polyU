#ifndef UART_H__
#define UART_H__
#define UART_TX_BUF_SIZE 256//512                        /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 32                           /**< UART RX buffer size. */
void uart_init(void);
#endif

