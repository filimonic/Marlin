# DMA File transfer over UART\USART

DMAFT (DMA File transfer) disables normal marlin serial communication to receive file through serial directly to SD card without parsing data.
When DMAFT activates, it bumps serial communication to very high speeds.

## Class API:

    command_port_index_to_uart_index(int16_t command_port) // Get UART number. command_port is used as queue.command_port();


