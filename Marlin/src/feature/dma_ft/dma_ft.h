#pragma once

#include "../../inc/MarlinConfig.h"

#if ENABLED(DMA_FILE_TRANSFER)

  #include "dma_ft_cap_check.h"
  #include "dma_ft_enums.h"
  #include "stdint.h"

  /** DMA_FILE_TRANSFER buffer size (two buffers of this size are allocated statically in RAM)
   *  FORMULA: Buffer is calculated as 64*(DMAFT_BUFFER_SIZE_ID + 1) bytes
   *  EXAMPLE:
   *    ID   =     BUF SIZE   =   FORMULA
   *     3   =   256  bytes   =   (3+1)*64
   *     7   =   512  bytes   =   (7+1)*64
   *    15   =   1024 bytes   =   (15+1)*64
   *    31   =   2048 bytes   =   (31+1)*64
   *    63   =   4096 bytes   =   (63+1)*64
   */
  #define DMAFT_BUFFER_SIZE_ID 31U // See explanation of this magic number above


  // DMA_FILE_TRANSFER port options for use in DMA mode
  // * DMAFT_XFER_SPD_* Baud rate (2400, 115200, 230400, 460800, 921600, 1958400*, 2250000), * - recommended
  #define DMAFT_TRANSFER_PORT_SPEED DMAFT_XFER_SPD_2400_BAUD

  // DMAFT_TRANSFER_PORT_TIMEOUT is timeout in 50ms units to assume that Host failed.
  // This timeout is used to start data transfer, so host should switch to receive state in DMAFT_TRANSFER_PORT_TIMEOUT
  #define DMAFT_TRANSFER_PORT_TIMEOUT 2000U // ms


  #define DMAFT_PROTOCOL_VERSION 0x01U //DMAFT Protocol version
  #define DMAFT_CHECKSUM_TYPE DMAFT_CS_TWOFLETCHER16 //DMAFT Checksum type. Use DMAFT_CS_*


  #define DMAFT_BUFFER_SIZE (64U * (DMAFT_BUFFER_SIZE_ID + 1U)) // This is calculated constantly

  #define DMAFT_BUFFER_LAST_BYTE_IDX (DMAFT_BUFFER_SIZE - 1U)

  class DMAFileTransfer
  {
  private:
    static uint32_t             timeout_start;

    static uint8_t              command_port_index_to_uart_index(const int16_t command_port);
    static bool                 precheck_sd_and_file(char* path);
    static MarlinSerial*        get_mSerial(const int16_t command_port);
    static void                 write_dmaft_hello(MarlinSerial* mSerial, bool wait);
    static void                 request_send_chunk(const uint8_t chunk_id, bool wait);
    static const uint8_t* const create_packet(uint8_t packet_type, const uint8_t* const payload, uint16_t payload_length, uint16_t& data_length);
    static void                 write_packet(MarlinSerial* mSerial, uint8_t packet_type, const uint8_t* const payload, uint16_t payload_length, bool wait);
    static void                 clean_dma_buffer(uint8_t* buffer);
    static void                 reset_timeout();
    static bool                 is_timeout_exceed();
    static bool                 is_chunk_received(void *buffer);
    //static uint32_t             get_baudrate_from_transfer_option(const uint8_t dmaft_transfer_port_options);

    static uint8_t              wait_buffer_received(const uint8_t uart_port_index, const uint8_t* buffer);
    static uint8_t              process_buffer(const uint8_t* buffer, const uint8_t expected_chunk_id);

    // PER-PLATFORM, see dma_ft_mcu*.cpp

    static bool                 dmaserial_available_on_port(const uint8_t uart_port_index);
    static void                 dmaserial_start(const uint8_t uart_port_index, const uint32_t dmaft_transfer_port_speed);
    static void                 dmaserial_write(const uint8_t* buffer, uint16_t size, bool wait);
    static void                 dmaserial_set_receive_buffer(const uint8_t uart_port_index, uint8_t* buffer);
    static void                 dmaserial_stop();
    //static void                 save_uart_state(const uint8_t uart_port_index);
    //static void                 restore_uart_state(const uint8_t uart_port_index);


    // PER-CHECKSUM, see ./checksum folder dma_ft_cs*.cpp
    static uint32_t             calculate_checksum(uint8_t* data, uint16_t data_length);
    static const char* const    get_checksum_name();


  public:
    static bool                 is_command_port_available_for_dma(const int16_t command_port);
    static bool                 receive_file(const int16_t command_port, char* path);








  };



#endif
