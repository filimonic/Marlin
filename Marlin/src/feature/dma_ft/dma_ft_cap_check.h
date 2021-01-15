/**
 * Capability checks for DMA_FILE_TRANSFER
 *
 */

#pragma once

#include "../../inc/MarlinConfig.h"

#if ENABLED(DMA_FILE_TRANSFER)

  #ifdef STM32F1
    #if !HAS_MULTI_SERIAL && !WITHIN(SERIAL_PORT, 1, 4)
      #error "On STM32F1 devices DMA_FILE_TRANSFER is available only on USART1, USART2, USART3, USART4. Check your SERIAL_PORT or enable SERIAL_PORT_2 on supported port."
    #elif HAS_MULTI_SERIAL && !(WITHIN(SERIAL_PORT, 1, 4) && WITHIN(SERIAL_PORT_2, 1, 4))
      #error "On STM32F1 devices DMA_FILE_TRANSFER is available only on USART1, USART2, USART3, USART4. Both your SERIAL_PORT or SERIAL_PORT_2 do not support this feature."
    #elif HAS_MULTI_SERIAL && !WITHIN(SERIAL_PORT, 1, 4)
      #warning "On STM32F1 devices DMA_FILE_TRANSFER is available only on USART1, USART2, USART3, USART4. Your SERIAL_PORT does not support this feature."
    #elif HAS_MULTI_SERIAL && !WITHIN(SERIAL_PORT_2, 1, 4)
      #warning "On STM32F1 devices DMA_FILE_TRANSFER is available only on USART1, USART2, USART3, USART4. Your SERIAL_PORT_2 does not support this feature."
    #endif

    #if ENABLED(BAUD_RATE_GCODE)
      #warning "!!!"
      #warning "!!!"
      #warning "BAUD_RATE_GCODE is partially incompatible with DMA_FILE_TRANSFER."
      #warning "After DMA_FILE_TRANSFER completes, port will be restarted at default baud rate!"
      #warning "!!!"
      #warning "!!!"
    #endif

    // TODO ADD WARNING FOR DMAFT_XFER_SPD_4500000_BAUD available on UART1 only

  #else
    #error "DMA_FILE_TRANSFER does not support your MCU"
  #endif


#endif
