#include "../../../inc/MarlinConfig.h"

/**
 * REFERENCE:
 * RM0008 @ page 817
 * https://www.st.com/resource/en/reference_manual/cd00171190-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
 */

#ifdef STM32F1
  #include "../dma_ft.h"

  #define _DMAFT_UART_STATE_SIZE sizeof(USART_TypeDef)

  volatile uint8_t _dmaft_uart_state[_DMAFT_UART_STATE_SIZE]; // Store UART state here.

  typedef struct DMACCPair {
    DMA_TypeDef*         controller;
    DMA_Channel_TypeDef* tx_channel;
    uint32_t             clear_ifcr_mask;
    uint32_t             isr_tcif_mask;
    DMACCPair(DMA_TypeDef* controller, DMA_Channel_TypeDef* channel, uint8_t dma_channel_num) :
      controller(controller),
      tx_channel(channel),
      clear_ifcr_mask((DMA_IFCR_CTEIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CGIF1) << ((dma_channel_num - 1) * 4)),
      isr_tcif_mask(DMA_ISR_TCIF1 << ((dma_channel_num - 1) * 4))
      {};
    DMACCPair():
      controller(NULL),
      tx_channel(NULL),
      clear_ifcr_mask(0),
      isr_tcif_mask(0)
      {};
    //
  } DMACCPair_TypeDef;

  bool DMAFileTransfer::is_uart_available_for_dma(const uint8_t uart_port_index) {
    return WITHIN(uart_port_index, 1, 4);
  }

  // Helper to get reference to USART_TypeDef (CMSIS data type) from port index
  USART_TypeDef* _dmaft_stm32f1_get_usart_by_uart_idx(const uint8_t uart_port_index) {
    switch (uart_port_index)
    {
    case 1: return USART1;
    case 2: return USART2;
    case 3: return USART3;
    case 4: return UART4;
    }
    return NULL;
  }

  const DMACCPair_TypeDef _dmaft_stm32f1_get_dma_cc_pair_by_uart_idx(const uint8_t uart_port_index) {
    // REFERENCE: RM0008
    switch (uart_port_index)
    {
    case 1: return DMACCPair_TypeDef(DMA1, DMA1_Channel4, 4); //DMA1 + USART1_TX
    case 2: return DMACCPair_TypeDef(DMA1, DMA1_Channel7, 7); //DMA1 + USART2_TX
    case 3: return DMACCPair_TypeDef(DMA1, DMA1_Channel2, 2); //DMA1 + USART3_TX
    case 4: return DMACCPair_TypeDef(DMA2, DMA2_Channel5, 5); //DMA2 + UART4_TX
    default:
      return DMACCPair_TypeDef(NULL, NULL, 0);
    }
  }

  const uint32_t _dmaft_stm32f1_port_speed_id_to_baud_rate(const uint8_t port_speed) {
    switch (port_speed) {
      case DMAFT_XFER_SPD_2400_BAUD    : return 2400;
      case DMAFT_XFER_SPD_115200_BAUD  : return 115200;
      case DMAFT_XFER_SPD_230400_BAUD  : return 230400;
      case DMAFT_XFER_SPD_460800_BAUD  : return 460800;
      case DMAFT_XFER_SPD_921600_BAUD  : return 921600;
      case DMAFT_XFER_SPD_2250000_BAUD : return 2250000;
    }
    return 0;
  }

  const uint32_t _dmaft_stm32f1_get_uart_brr(const uint8_t uart_port_index, const uint8_t port_speed) {
    uint32_t baud_rate = _dmaft_stm32f1_port_speed_id_to_baud_rate(port_speed);
    uint32_t fck = uart_port_index == 1
      ? HAL_RCC_GetPCLK2Freq()
      : HAL_RCC_GetPCLK1Freq();
    return UART_BRR_SAMPLING16(fck, baud_rate);
  }

  /**
   * Save/Resote UART state
   */

  void DMAFileTransfer::save_uart_state(const uint8_t uart_port_index) {
    const USART_TypeDef* uart = _dmaft_stm32f1_get_usart_by_uart_idx(uart_port_index);
    memcpy((void*)_dmaft_uart_state, (void*)uart, _DMAFT_UART_STATE_SIZE);
  }

  void DMAFileTransfer::restore_uart_state(const uint8_t uart_port_index) {
    const USART_TypeDef* uart = _dmaft_stm32f1_get_usart_by_uart_idx(uart_port_index);
    memcpy((void*)uart, (void*)_dmaft_uart_state, _DMAFT_UART_STATE_SIZE);
  }

  void DMAFileTransfer::setup_uart_dma(const uint8_t uart_port_index) {
    USART_TypeDef* uart = _dmaft_stm32f1_get_usart_by_uart_idx(uart_port_index);
    DMACCPair_TypeDef dma_cc = _dmaft_stm32f1_get_dma_cc_pair_by_uart_idx(uart_port_index);

    // Wait to complete transmission
    while (uart->SR & USART_SR_TC) {
      safe_delay(50);
    }

    safe_delay(DMAFT_TRANSFER_PORT_TIMEOUT/4);

    // Stop UART
    uart->GTPR = 0x00U;
    uart->CR1  = 0x00U;
    uart->CR2  = 0x00U;
    uart->CR3  = 0x00U;

    // Set UART Baudrate
    uart->BRR = _dmaft_stm32f1_get_uart_brr(uart_port_index, (DMAFT_TRANSFER_PORT_OPTIONS & 0x0F));

    // Set parity control
    #if DMAFT_TRANSFER_PORT_OPTIONS & DMAFT_XFER_PAR_ODD
      // If odd parity control is enabled, we run at 9 bits per packet
      uart->CR1 |= USART_CR1_PS | USART_CR1_PCE | USART_CR1_M;
    #endif

    #if DMAFT_TRANSFER_PORT_OPTIONS & DMAFT_XFER_STB_2
      uart->CR2 |= USART_CR2_STOP_1;
    #endif

    // Set UART to DMA Receive mode
    uart->CR3 |= USART_CR3_DMAR;

    // Set DMA
    dma_cc.tx_channel->CCR  |= DMA_CCR_PL_1 | DMA_CCR_MINC | DMA_CCR_TCIE;
    dma_cc.tx_channel->CNDTR = DMAFT_BUFFER_SIZE;
    dma_cc.tx_channel->CPAR  = (uint32_t)&uart->DR;

    uart->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;

    safe_delay(DMAFT_TRANSFER_PORT_TIMEOUT/4);
  }


  void DMAFileTransfer::start_dma_receive_buffer(uint8_t uart_port_index, uint8_t* buffer) {
    DMACCPair_TypeDef dma_cc = _dmaft_stm32f1_get_dma_cc_pair_by_uart_idx(uart_port_index);
    // TODO: Do not clean buffer here. Maybe call clean_dma_buffer ?
    memset(buffer, 0x00, DMAFT_BUFFER_SIZE);
    dma_cc.tx_channel->CMAR  = (uint32_t)&buffer;

    // TODO: We do not use DMA registers. Do we need this ?
    dma_cc.controller->IFCR  = dma_cc.clear_ifcr_mask;
    dma_cc.tx_channel->CCR |= DMA_CCR_EN;
  }


#endif

