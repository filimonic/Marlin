#pragma once

#include "../../inc/MarlinConfig.h"

#if ENABLED(DMA_FILE_TRANSFER)

  #include "stdint.h"

  #define DMAFT_XFER_SPD_2400_BAUD    0x00 //Transfer speed in DMA mode is 2400    baud (for developement only)
  #define DMAFT_XFER_SPD_115200_BAUD  0x01 //Transfer speed in DMA mode is 115200  baud
  #define DMAFT_XFER_SPD_230400_BAUD  0x02 //Transfer speed in DMA mode is 230400  baud
  #define DMAFT_XFER_SPD_460800_BAUD  0x03 //Transfer speed in DMA mode is 460800  baud
  #define DMAFT_XFER_SPD_921600_BAUD  0x04 //Transfer speed in DMA mode is 921600  baud
  #define DMAFT_XFER_SPD_2250000_BAUD 0x05 //Transfer speed in DMA mode is 2250000 baud (Max on STM32F1 for USART != 1)

  #define DMAFT_XFER_PAR_NONE         0x00 //Parity check in DMA mode is off
  #define DMAFT_XFER_PAR_ODD          0x80 //Parity check in DMA mode is odd bit.

  #define DMAFT_XFER_STB_1            0x00 //In DMA mode there are 1 stop bit each byte
  #define DMAFT_XFER_STB_2            0x40 //In DMA mode there are 2 stop bits each byte

  #define DMAFT_PT_HELLO              0x00 //PT-HELLO   - Packet type "Hello"
  #define DMAFT_PT_REQDATA            0x01 //PT-REQDATA - Packet type "Request Data"
  #define DMAFT_PT_CHUNK              0x02 //PT-CHUNK   - Packet type "Chunk Data", when chunk is not last
  #define DMAFT_PT_LASTCHUNK          0x03 //PT-CHUNK   - Packet type "Chunk Data", when chunk is last
  #define DMAFT_PT_ABORTED            0x04 //PT-ABORTED - Packet type "Transfer aborted", when host aborts transfer


  #define DMAFT_CS_NONE               0x00 // Checksum is none
  #define DMAFT_CS_FLETCHER16         0x01 // Checksum is Fletcher-16

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
