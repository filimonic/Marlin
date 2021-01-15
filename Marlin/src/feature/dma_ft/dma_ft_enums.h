#pragma once

#include "../../inc/MarlinConfig.h"

#if ENABLED(DMA_FILE_TRANSFER)

  #include "stdint.h"

  #define DMAFT_XFER_SPD_2400_BAUD    2400U    //Transfer speed in DMA mode is 2400    baud (for developement only)
  #define DMAFT_XFER_SPD_115200_BAUD  115200U  //Transfer speed in DMA mode is 115200  baud
  #define DMAFT_XFER_SPD_921600_BAUD  921600U  //Transfer speed in DMA mode is 921600  baud
  #define DMAFT_XFER_SPD_1958400_BAUD 1958400U //Transfer speed in DMA mode is 1958400 baud (This one is used on makerbase MKS WiFi modules)
  #define DMAFT_XFER_SPD_2250000_BAUD 2250000U //Transfer speed in DMA mode is 2250000 baud (Max on STM32F1 for USART != 1)
  #define DMAFT_XFER_SPD_4500000_BAUD 4500000U //Transfer speed in DMA mode is 4500000 baud (Max on STM32F1 for USART 1 and not for other UARTs)


  #define DMAFT_PT_HELLO              0x00 //PT-HELLO   - Packet type "Hello"
  #define DMAFT_PT_REQDATA            0x01 //PT-REQDATA - Packet type "Request Data"
  #define DMAFT_PT_CHUNK              0x02 //PT-CHUNK   - Packet type "Chunk Data", when chunk is not last
  #define DMAFT_PT_LASTCHUNK          0x03 //PT-CHUNK   - Packet type "Chunk Data", when chunk is last
  #define DMAFT_PT_ABORTED            0x04 //PT-ABORTED - Packet type "Transfer aborted", when host aborts transfer

  #define DMAFT_CS_NONE               0x00 // Checksum is none
  #define DMAFT_CS_TWOFLETCHER16      0x01 // Checksum is Fletcher-16

  #define DMAFT_RESULT_OK             0x00
  #define DMAFT_RESULT_OK_FINISHED    0x01
  #define DMAFT_RESULT_OK_ABORTED     0x10
  #define DMAFT_RESULT_BUF_FAILED_CPS 0xC1
  #define DMAFT_RESULT_BUF_FAILED_CRC 0xC2
  #define DMAFT_RESULT_TIMEOUT_EXCEED 0xD1
  #define DMAFT_RESULT_WRITE_FAILED   0xE1
  #define DMAFT_RESULT_UNEXPECTED_ID  0xE2
  #define DMAFT_RESULT_UNKNOWN_PT     0xFF

  #define DMAFT_STR_PROCESSING        "DMAFT CHUNK %hhu"
  #define DMAFT_STR_START             "DMAFT Started"
  #define DMAFT_STR_FINISH            "DMAFT Finished"

#endif
