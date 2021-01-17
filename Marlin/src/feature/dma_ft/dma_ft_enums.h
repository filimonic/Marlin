#pragma once

#include "../../inc/MarlinConfig.h"

#if ENABLED(DMA_FILE_TRANSFER)

  #include "stdint.h"

  #define DMAFT_PT_REQDATA            0x01 //PT-REQDATA - Packet type "Request Data"
  #define DMAFT_PT_CHUNK              0x02 //PT-CHUNK   - Packet type "Chunk Data", when chunk is not last
  #define DMAFT_PT_LENCHUNK           0x04 //PT-LENCHUNK- Packet type "Chunk Data", when chunk is not last but has length. If length is zero, it means end of file
  #define DMAFT_PT_EOF                0x05 //PT-EOF     - Packet type "EOF", last chunk in successful tranmission
  #define DMAFT_PT_ABORTED            0x06 //PT-ABORTED - Packet type "Transfer aborted", when host aborts transfer

  #define DMAFT_CS_NONE               0x00 // Checksum is none
  #define DMAFT_CS_TWOFLETCHER16      0x01 // Checksum is Fletcher-16

/*   #define DMAFT_RESULT_UNKNOWN        0x00 // 000
  #define DMAFT_RESULT_OK             0x01 //   1
  #define DMAFT_RESULT_OK_FINISHED    0x02 //   2
  #define DMAFT_RESULT_OK_ABORTED     0x10 //  16
  #define DMAFT_RESULT_BUF_FAILED_CPS 0xC1 // 193
  #define DMAFT_RESULT_BUF_FAILED_CRC 0xC2 // 194
  #define DMAFT_RESULT_TIMEOUT_EXCEED 0xD1 // 209
  #define DMAFT_RESULT_WRITE_FAILED   0xE1 // 225
  #define DMAFT_RESULT_UNEXPECTED_ID  0xE2 // 226
  #define DMAFT_RESULT_UNKNOWN_PT     0xFF // 255
 */
  #define DMAFT_STR_STATUS_LINE       "Uploading... %3u%%" // "Uploading... 100%"
  #define DMAFT_STR_STATUS_LINE_LEN   18 // With zero termination

#endif
