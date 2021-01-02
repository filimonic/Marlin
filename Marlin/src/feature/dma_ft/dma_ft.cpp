#include "dma_ft.h"

#if ENABLED(DMA_FILE_TRANSFER)

  void DMAFileTransfer::start_dma_transfer(int16_t serial_port_idx) {
    // Get UART number we need to start DMA Transfer on
    int8_t uart_idx = port_idx_to_uart_idx(serial_port_idx);

    if (uart_idx <= 0) { // UART port must be 1 or higher
      return;
    }

    if (!is_uart_rx_dma_capable(uart_idx)) {
      return;
    }

    if (!allocate_dma_buffers()) {
      return;
    }

    get_prerequisites(uart_idx);
    //!FIXME there should be [PT-READY] sent
    enable_dma_on_port_rx(0x00); //!FIXME this is incorrect value
  }

  int8_t DMAFileTransfer::port_idx_to_uart_idx(int16_t serial_port_idx) {
    #if HAS_MULTI_SERIAL
      return TERN(port_num == 0, SERIAL_PORT, SERIAL_PORT_2);
    #else
      return SERIAL_PORT;
    #endif
  }

  bool DMAFileTransfer::is_uart_rx_dma_capable(int8_t uart_idx) {
    #ifdef STM32F1
      /*
       * On STM32
       *   USART1, USART2, USART3 are capable for DMA on DMA1 controller
       *   UART4 is capable for DMA on DMA2 controller
       */
      return (WITHIN(uart_idx,1,4));
    #else
      return false;
    #endif
  }

  void DMAFileTransfer::get_prerequisites(int8_t uart_idx) {
    #ifdef STM32F1
      //TODO: This can be optimized with compiler directives like ANYEQUAL(SERIAL_PORT, SERIAL_PORT_2)
      switch (uart_idx) {
          case 1:
            uart                = USART1;
            uart_dma_controller = DMA1;
            uart_rx_dma_channel = DMA1_Channel5;
            uart_dma_controller_reg_IFCR  =
              DMA_IFCR_CTEIF5 |
              DMA_IFCR_CHTIF5 |
              DMA_IFCR_CTCIF5 |
              DMA_IFCR_CGIF5  ;

            break;
          case 2:
            uart                = USART2;
            uart_dma_controller = DMA1;
            uart_rx_dma_channel = DMA1_Channel6;
            uart_dma_controller_reg_IFCR  =
              DMA_IFCR_CTEIF6 |
              DMA_IFCR_CHTIF6 |
              DMA_IFCR_CTCIF6 |
              DMA_IFCR_CGIF6  ;
            break;
          case 3:
            uart                = USART3;
            uart_dma_controller = DMA1;
            uart_rx_dma_channel = DMA1_Channel3;
            uart_dma_controller_reg_IFCR  =
              DMA_IFCR_CTEIF3 |
              DMA_IFCR_CHTIF3 |
              DMA_IFCR_CTCIF3 |
              DMA_IFCR_CGIF3  ;
            break;
          case 4:
            uart                = UART4;
            uart_dma_controller = DMA2;
            uart_rx_dma_channel = DMA2_Channel3;
            uart_dma_controller_reg_IFCR  =
              DMA_IFCR_CTEIF3 |
              DMA_IFCR_CHTIF3 |
              DMA_IFCR_CTCIF3 |
              DMA_IFCR_CGIF3  ;
            break;
        }
      #endif
  }

  bool DMAFileTransfer::allocate_dma_buffers() {
    // Allocate buffers with zeros and reset buffer pointer
    dma_buffer_idx = 0;
    dma_buffers[0] = calloc(DMAFT_BUFFER_SIZE, sizeof(uint8_t));
    if (dma_buffers[0] == NULL ) {
      return false;
    }

    dma_buffers[1] = calloc(DMAFT_BUFFER_SIZE, sizeof(uint8_t));
    if (dma_buffers[1] == NULL ) { // If we could not allocate memory for second buffer,
      free(dma_buffers[0]);
      return false;
    }
    return true;
  }

  void DMAFileTransfer::start_dma_to_buffer() {
    // Setup DMA
    uart_rx_dma_channel->CCR =
      DMA_CCR_PL_0 | DMA_CCR_PL_1 | // Highest priority
      DMA_CCR_MINC    | // Memory increase mode
      DMA_CCR_MSIZE_0 | // Memory size is 32-bit
      DMA_CCR_TCIE    ; // Transfer complete interrupt enable

    uart_rx_dma_channel->CNDTR =
      DMAFT_BUFFER_SIZE; // Number of bytes to transfer must be equal our buffer

    uart_rx_dma_channel->CPAR =
      (uint32_t)&uart->DR; // Set source to UART's Data Register

    uart_rx_dma_channel->CMAR =
      (uint32_t)dma_buffers[dma_buffer_idx]; // Set target to dma buffers's address

    uart_dma_controller->IFCR =
      uart_dma_controller_reg_IFCR; // Set some registers.

    uart_rx_dma_channel->CCR |=
      DMA_CCR_EN; // Enable DMA Channel
  }

  void DMAFileTransfer::enable_dma_on_port_rx(uint8_t uart_settings) {
    #ifdef STM32F1

      // reference to RM0008 Reference Manual from ST - Pages 282, 283
      // https://www.st.com/resource/en/reference_manual/cd00171190-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

      // Set UART to disabled mode
      uart->CR1 &= ~USART_CR1_UE;
      safe_delay(100);

      // Save old bitrate and CRx registers
      uart_old_registers[0] = uart->BRR;
      uart_old_registers[1] = uart->CR1;
      uart_old_registers[2] = uart->CR2;
      uart_old_registers[3] = uart->CR3;

      // Set UART registers for new transmission mode
      uart->BRR =
        0x25; //!FIX THIS HARDCODED

      uart->CR1 =
        USART_CR1_TE | // Enable Transmission
        TERN0(uart_settings && DMAFT_SERIAL_PARITY_ODD, USART_CR1_M)  |  // Parity: Set to 9-bit transmission if parity check is enabled
        TERN0(uart_settings && DMAFT_SERIAL_PARITY_ODD, USART_CR1_PCE)|  // Parity: Set if parity check is enabled
        TERN0(uart_settings && DMAFT_SERIAL_PARITY_ODD, USART_CR1_PS );  // Parity: Set ODD PARITY CHECK if parity check is enabled

      uart->CR2 =
        TERN0(uart_settings && DMAFT_SERIAL_STOPBIT_2, USART_CR2_STOP_0); // Stop bits: Set to 2 stop bits (otherwise 1 stop bit)

      uart->CR3 =
        USART_CR3_DMAR; // Enable DMA Receive

      start_dma_to_buffer();

      safe_delay(200);
      uart->CR1 |= USART_CR1_UE; // Enable UART
    #endif
  }

#endif
