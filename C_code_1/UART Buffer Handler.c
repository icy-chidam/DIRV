#define UART_BUF_SIZE 4096
uint8_t uart_buffer[UART_BUF_SIZE];
uint32_t uart_buf_len = 0;

void uart_rx_handler(uint8_t byte)
{
    if (uart_buf_len < UART_BUF_SIZE-1)
        uart_buffer[uart_buf_len++] = byte;
}
