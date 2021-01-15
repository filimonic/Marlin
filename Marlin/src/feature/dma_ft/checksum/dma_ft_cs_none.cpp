#include "../../../inc/MarlinConfig.h"

#if ENABLED(DMA_FILE_TRANSFER)
  #include "../dma_ft.h"
  #if DMAFT_CHECKSUM_TYPE == DMAFT_CS_NONE
    #define DMAFT_CHECKSUM_NAME "NONE"

    uint32_t DMAFileTransfer::calculate_checksum(uint8_t* data, uint16_t data_length) {
      return 0;
    }
    const char* const DMAFileTransfer::get_checksum_name() {
      return DMAFT_CHECKSUM_NAME;
    }

  #endif
#endif
