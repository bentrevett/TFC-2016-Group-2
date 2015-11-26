#ifndef TFC_UART_H_
#define TFC_UART_H_

void TFC_InitUARTs(uint32_t uart0_baud, uint32_t uart2_baud);
void TFC_UART_Process();
void TFC_BluetoothModuleSetBaud(unsigned int baudRate);
void uart_putchar (UART_MemMapPtr channel, char ch);
int uart_getchar_present (UART_MemMapPtr channel);
char uart_getchar (UART_MemMapPtr channel);
extern ByteQueue SDA_SERIAL_OUTGOING_QUEUE;
extern ByteQueue SDA_SERIAL_INCOMING_QUEUE;


#endif /* TFC_UART_H_ */
