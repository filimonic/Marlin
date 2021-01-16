#include "../../../inc/MarlinConfig.h"

/**
 * REFERENCE:
 * RM0008 @ page 817
 * https://www.st.com/resource/en/reference_manual/cd00171190-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
 */

#ifdef STM32F1
  #include "../dma_ft.h"

  UART_HandleTypeDef dmaft_huart;
  DMA_HandleTypeDef  dmaft_hdma_rx;
/*   DMA_HandleTypeDef  dmaft_hdma_tx; */


  bool DMAFileTransfer::dmaserial_available_on_port(const uint8_t uart_port_index) {
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

  DMA_Channel_TypeDef* _dmaft_stm32f1_get_dma_uart_rx_channel(const uint8_t uart_port_index) {
    switch (uart_port_index)
    {
    case 1: return DMA1_Channel5;
    case 2: return DMA1_Channel6;
    case 3: return DMA1_Channel3;
    case 4: return DMA2_Channel3;
    default:
      return NULL;
    }
  }

/*   DMA_Channel_TypeDef* _dmaft_stm32f1_get_dma_uart_tx_channel(const uint8_t uart_port_index) {
  switch (uart_port_index)
  {
  case 1: return DMA1_Channel4;
  case 2: return DMA1_Channel7;
  case 3: return DMA1_Channel2;
  case 4: return DMA2_Channel5;
  default:
    return NULL;
  }
  } */
/*
  FORCE_INLINE const uint32_t _dmaft_stm32f1_port_speed_id_to_baud_rate(const uint8_t port_speed) {
    switch (port_speed) {
      case DMAFT_XFER_SPD_2400_BAUD    : return 2400;
      case DMAFT_XFER_SPD_115200_BAUD  : return 115200;
      case DMAFT_XFER_SPD_230400_BAUD  : return 230400;
      case DMAFT_XFER_SPD_460800_BAUD  : return 460800;
      case DMAFT_XFER_SPD_921600_BAUD  : return 921600;
      case DMAFT_XFER_SPD_1958400_BAUD : return 1958400;
      case DMAFT_XFER_SPD_2250000_BAUD : return 2250000;
    }
    return 0;
  } */

/*   const uint32_t _dmaft_stm32f1_get_uart_brr(const uint8_t uart_port_index, const uint8_t port_speed) {
    uint32_t baud_rate = _dmaft_stm32f1_port_speed_id_to_baud_rate(port_speed);
    uint32_t fck = uart_port_index == 1
      ? HAL_RCC_GetPCLK2Freq()
      : HAL_RCC_GetPCLK1Freq();
    return UART_BRR_SAMPLING16(fck, baud_rate);
  } */

  const void _dmaft_stm32f1_enable_uart_clk(const uint8_t uart_port_index) {
    switch (uart_port_index)
    {
      case 1 :
        __HAL_RCC_USART1_FORCE_RESET();
        __HAL_RCC_USART1_RELEASE_RESET();
        __HAL_RCC_USART1_CLK_ENABLE();
        break;
      case 2 :
        __HAL_RCC_USART2_FORCE_RESET();
        __HAL_RCC_USART2_RELEASE_RESET();
        __HAL_RCC_USART2_CLK_ENABLE();
        break;
      case 3 :
        __HAL_RCC_USART3_FORCE_RESET();
        __HAL_RCC_USART3_RELEASE_RESET();
        __HAL_RCC_USART3_CLK_ENABLE();
        break;
      case 4 :
        __HAL_RCC_UART4_FORCE_RESET();
        __HAL_RCC_UART4_RELEASE_RESET();
        __HAL_RCC_UART4_CLK_ENABLE();
        break;
    }
    HAL_Delay(20);
  }

  /* FORCE_INLINE void _dmaft_stm32f1_enable_gpio_clk(const uint8_t uart_port_index) {
    switch (uart_port_index)
    {
      case 1: {
        if ((AFIO->MAPR & AFIO_MAPR_USART1_REMAP) && __HAL_RCC_GPIOB_IS_CLK_DISABLED()) {
          __HAL_RCC_GPIOB_CLK_ENABLE();
        } else if (__HAL_RCC_GPIOA_IS_CLK_DISABLED()) {
          __HAL_RCC_GPIOA_CLK_ENABLE();
        }
        break;
      }
      case 2: {
        if ((AFIO->MAPR & AFIO_MAPR_USART2_REMAP) && __HAL_RCC_GPIOD_IS_CLK_DISABLED()) {
          __HAL_RCC_GPIOD_CLK_ENABLE();
        } else if (__HAL_RCC_GPIOA_IS_CLK_DISABLED()) {
          __HAL_RCC_GPIOA_CLK_ENABLE();
        }
        break;
      }
      case 3: {
        if (((AFIO->MAPR & AFIO_MAPR_USART3_REMAP_FULLREMAP) == AFIO_MAPR_USART3_REMAP_FULLREMAP) && __HAL_RCC_GPIOD_IS_CLK_DISABLED()) {
          __HAL_RCC_GPIOD_CLK_ENABLE();
        } else if (((AFIO->MAPR & AFIO_MAPR_USART3_REMAP_PARTIALREMAP) == AFIO_MAPR_USART3_REMAP_PARTIALREMAP) && __HAL_RCC_GPIOC_IS_CLK_DISABLED()) {
          __HAL_RCC_GPIOC_CLK_ENABLE();
        } else if (__HAL_RCC_GPIOB_IS_CLK_DISABLED()) {
          __HAL_RCC_GPIOB_CLK_ENABLE();
        }
        //TODO! UART4
        break;
      }
    }
  } */

  FORCE_INLINE void _dmaft_stm32f1_enable_dma_clk(const uint8_t uart_port_index) {
    if (uart_port_index == 4 ) {
        __HAL_RCC_DMA2_CLK_DISABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();
    } else {
        __HAL_RCC_DMA1_CLK_DISABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();
    }
    HAL_Delay(20);
  }

/*   FORCE_INLINE void _dmaft_stm32f1_enable_dma_clk(const uint8_t uart_port_index) {
    if (uart_port_index == 4 && __HAL_RCC_DMA2_IS_CLK_DISABLED()) {
        __HAL_RCC_DMA2_CLK_DISABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();
    } else if (__HAL_RCC_DMA2_IS_CLK_DISABLED()) {
        __HAL_RCC_DMA1_CLK_DISABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();
    }
    HAL_Delay(20);
  } */

  /**
   * Save/Resote UART state
   */

/*   void DMAFileTransfer::save_uart_state(const uint8_t uart_port_index) {
    const USART_TypeDef* uart = _dmaft_stm32f1_get_usart_by_uart_idx(uart_port_index);
    memcpy((void*)_dmaft_uart_state, (void*)uart, _DMAFT_UART_STATE_SIZE);
  }

  void DMAFileTransfer::restore_uart_state(const uint8_t uart_port_index) {
    const USART_TypeDef* uart = _dmaft_stm32f1_get_usart_by_uart_idx(uart_port_index);
    memcpy((void*)uart, (void*)_dmaft_uart_state, _DMAFT_UART_STATE_SIZE);
  } */
  FORCE_INLINE IRQn_Type _dmaft_stm32f1_get_dma_rx_irqn(const uint8_t uart_port_index) {
    switch (uart_port_index) {
      case 1: return DMA1_Channel5_IRQn;
      case 2: return DMA1_Channel6_IRQn;
      case 3: return DMA1_Channel3_IRQn;
      case 4: return DMA2_Channel3_IRQn;
      default: return UsageFault_IRQn;
    }
  }

  void _dmaft_stm32f1_set_dma_ifcr(const uint8_t uart_port_index) {
    switch (uart_port_index) {
      case 1: { DMA1->IFCR = DMA_IFCR_CTEIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CGIF5; break; }
      case 2: { DMA1->IFCR = DMA_IFCR_CTEIF6 | DMA_IFCR_CHTIF6 | DMA_IFCR_CTCIF6 | DMA_IFCR_CGIF6; break; }
      case 3: { DMA1->IFCR = DMA_IFCR_CTEIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CGIF3; break; }
      case 4: { DMA2->IFCR = DMA_IFCR_CTEIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CGIF3; break; }
    }
  }

/*   FORCE_INLINE IRQn_Type _dmaft_stm32f1_get_dma_tx_irqn(const uint8_t uart_port_index) {
    switch (uart_port_index) {
      case 1: return DMA1_Channel4_IRQn;
      case 2: return DMA1_Channel7_IRQn;
      case 3: return DMA1_Channel2_IRQn;
      case 4: return DMA2_Channel4_5_IRQn;
      default: return UsageFault_IRQn;
    }
  } */

  FORCE_INLINE IRQn_Type _dmaft_stm32f1_get_uart_irqn(const uint8_t uart_port_index) {
    switch (uart_port_index) {
      case 1: return USART1_IRQn;
      case 2: return USART2_IRQn;
      case 3: return USART3_IRQn;
      case 4: return UART4_IRQn;
      default: return UsageFault_IRQn;
    }
  }

  void DMAFileTransfer::dmaserial_start(const uint8_t uart_port_index, const uint32_t dmaft_transfer_port_speed) {
    // REF https://riptutorial.com/ru/stm32/example/30042/%D0%BF%D0%B5%D1%80%D0%B5%D0%B4%D0%B0%D1%87%D0%B0-%D0%B1%D0%BE%D0%BB%D1%8C%D1%88%D0%BE%D0%B3%D0%BE-%D0%BE%D0%B1%D1%8A%D0%B5%D0%BC%D0%B0-%D0%B4%D0%B0%D0%BD%D0%BD%D1%8B%D1%85-%D1%81-%D0%B8%D1%81%D0%BF%D0%BE%D0%BB%D1%8C%D0%B7%D0%BE%D0%B2%D0%B0%D0%BD%D0%B8%D0%B5%D0%BC-dma-%D0%B8-%D0%BF%D1%80%D0%B5%D1%80%D1%8B%D0%B2%D0%B0%D0%BD%D0%B8%D0%B9---%D0%B1%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA%D0%B0-hal
    HAL_StatusTypeDef init_status = HAL_OK;
    // DMA INIT
    //_dmaft_stm32f1_enable_gpio_clk(uart_port_index);
    _dmaft_stm32f1_enable_dma_clk(uart_port_index);

    dmaft_hdma_rx.Instance                 = _dmaft_stm32f1_get_dma_uart_rx_channel(uart_port_index);
    dmaft_hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    dmaft_hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    dmaft_hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    dmaft_hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    dmaft_hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dmaft_hdma_rx.Init.Mode                = DMA_NORMAL;
    dmaft_hdma_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;

/*     dmaft_hdma_tx.Instance                 = _dmaft_stm32f1_get_dma_uart_tx_channel(uart_port_index);
    dmaft_hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    dmaft_hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    dmaft_hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    dmaft_hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    dmaft_hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dmaft_hdma_tx.Init.Mode                = DMA_NORMAL;
    dmaft_hdma_tx.Init.Priority            = DMA_PRIORITY_LOW; */

    init_status = HAL_DMA_Init(&dmaft_hdma_rx);
    if (init_status != HAL_OK) {
      MYSERIAL0.printf("ERROR INIT DMA RX %u\n", init_status);
    }

/*     init_status = HAL_DMA_Init(&dmaft_hdma_tx);
    if (init_status != HAL_OK) {
      MYSERIAL0.printf("ERROR INIT DMA TX %u\n", init_status);
    } */

    __HAL_LINKDMA(&dmaft_huart,hdmarx,dmaft_hdma_rx);
/*     __HAL_LINKDMA(&dmaft_huart,hdmatx,dmaft_hdma_tx); */

    //IRQn_Type dma_rx_IRQn = _dmaft_stm32f1_get_dma_rx_irqn(uart_port_index);
    //HAL_NVIC_SetPriority(dma_rx_IRQn,0,0);
    //HAL_NVIC_EnableIRQ(dma_rx_IRQn);

    //IRQn_Type dma_tx_IRQn = _dmaft_stm32f1_get_dma_tx_irqn(uart_port_index);
    //HAL_NVIC_SetPriority(dma_tx_IRQn,0,0);
    //HAL_NVIC_EnableIRQ(dma_tx_IRQn);

    // UART INIT
    _dmaft_stm32f1_enable_uart_clk(uart_port_index);

    dmaft_huart.Instance          = _dmaft_stm32f1_get_usart_by_uart_idx(uart_port_index);
    dmaft_huart.Init.BaudRate     = 0U;
    dmaft_huart.Init.BaudRate     = dmaft_transfer_port_speed;
    dmaft_huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    dmaft_huart.Init.Mode         = UART_MODE_TX_RX;
    dmaft_huart.Init.OverSampling = UART_OVERSAMPLING_16;
    dmaft_huart.Init.Parity       = UART_PARITY_NONE;
    dmaft_huart.Init.StopBits     = UART_STOPBITS_1;
    dmaft_huart.Init.WordLength   = UART_WORDLENGTH_8B;

    init_status = HAL_UART_Init(&dmaft_huart);
    if (init_status != HAL_OK) {
      MYSERIAL0.printf("ERROR INIT UART %u\n", init_status);
    }

    //IRQn_Type uart_IRQn = _dmaft_stm32f1_get_uart_irqn(uart_port_index);
    //HAL_NVIC_SetPriority(uart_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(uart_IRQn);
  }
/*
  void DMAFileTransfer::dmaserial_start(const uint8_t uart_port_index) {
    USART_TypeDef* const uart = _dmaft_stm32f1_get_usart_by_uart_idx(uart_port_index);
    DMACCPair_TypeDef dma_cc = _dmaft_stm32f1_get_dma_cc_pair_by_uart_idx(uart_port_index);

    // Wait to complete transmission
    while (uart->SR & USART_SR_TC) {
      safe_delay(50);
    }
    // Stop UART
    uart->GTPR = 0x00U;
    uart->CR1  = 0x00U;
    uart->CR2  = 0x00U;
    uart->CR3  = 0x00U;
    // Set UART Baudrate
    uart->BRR = _dmaft_stm32f1_get_uart_brr(uart_port_index, (DMAFT_TRANSFER_PORT_OPTIONS & 0x0F));
    // Set parity control
    //#if DMAFT_TRANSFER_PORT_OPTIONS & DMAFT_XFER_PAR_ODD
      // If odd parity control is enabled, we run at 9 bits per packet
      // uart->CR1 |= USART_CR1_PS | USART_CR1_PCE | USART_CR1_M;
    //#endif

    //#if DMAFT_TRANSFER_PORT_OPTIONS & DMAFT_XFER_STB_2
      // uart->CR2 |= USART_CR2_STOP_1;
    //#endif

    // Set UART to DMA Receive mode
    uart->CR3 |= USART_CR3_DMAR ;

    // Set DMA
    dma_cc.rx_channel->CCR  |= DMA_CCR_PL_1 | DMA_CCR_MINC | DMA_CCR_TCIE;
    dma_cc.rx_channel->CNDTR = DMAFT_BUFFER_SIZE;
    dma_cc.rx_channel->CPAR  = (uint32_t)&uart->DR;


    _dmaft_stm32f1_enable_dma_clk(uart_port_index);
    _dmaft_stm32f1_enable_uart_clk(uart_port_index);

    uart->CR1 |= USART_CR1_RE | USART_CR1_TE;
    uart->CR1 |= USART_CR1_UE;

    //safe_delay(DMAFT_TRANSFER_PORT_TIMEOUT/4);
  } */

  void DMAFileTransfer::dmaserial_write(const uint8_t* buffer, uint16_t size) {
    HAL_UART_Transmit(&dmaft_huart, (uint8_t*)buffer, size, DMAFT_TRANSFER_PORT_TIMEOUT);
  }

  void DMAFileTransfer::dmaserial_set_receive_buffer(uint8_t uart_port_index, uint8_t* buffer) {
      HAL_UART_DMAStop(&dmaft_huart);
      HAL_UART_Receive_DMA(&dmaft_huart, buffer, DMAFT_BUFFER_SIZE);
  }

  void DMAFileTransfer::dmaserial_stop() {
      MYSERIAL0.printf("dmaserial_stop FIXME\n");
  }


/*   void DMAFileTransfer::dmaserial_set_receive_buffer(uint8_t uart_port_index, uint8_t* buffer) {
    DMACCPair_TypeDef dma_cc = _dmaft_stm32f1_get_dma_cc_pair_by_uart_idx(uart_port_index);
    // TODO: Do not clean buffer here. Maybe call clean_dma_buffer ?
    // clean_dma_buffer(buffer);
    dma_cc.rx_channel->CMAR  = (uint32_t)&buffer;

    // TODO: We do not use DMA registers. Do we need this ?
    dma_cc.controller->IFCR  = dma_cc.controller_clear_ifcr_value;
    dma_cc.rx_channel->CCR |= DMA_CCR_EN;
  } */


#endif

