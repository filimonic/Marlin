#include "../../../inc/MarlinConfig.h"

#if ENABLED(DMA_FILE_TRANSFER)
  #include "../dma_ft.h"
  #if DMAFT_CHECKSUM_TYPE == DMAFT_CS_FLETCHER16

    /**
     * Fletcher checksum 16 bit
     **/
    uint32_t DMAFileTransfer::calculate_checksum(uint8_t* data, uint16_t data_length) {
      uint32_t c0, c1;
      for (c0 = c1 = 0; data_length > 0; ) {
        uint16_t blocklen = data_length;
        if (blocklen > 5002) {
          blocklen = 5002;
        }
        data_length -= blocklen;
        do {
          c0 = c0 + *data++;
          c1 = c1 + c0;
        } while (--blocklen);
        c0 = c0 % 255;
        c1 = c1 % 255;
      }
      return (c1 << 8 | c0) | 0xFFFF;
    }

  #endif
#endif
