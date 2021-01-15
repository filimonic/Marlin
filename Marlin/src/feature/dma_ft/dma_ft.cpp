#include "../../inc/MarlinConfig.h"

#if ENABLED(DMA_FILE_TRANSFER)

  #include "dma_ft.h"
  #include "../../sd/cardreader.h"
  #include "../../module/temperature.h"
  #include "../../lcd/marlinui.h"


  // DMA File Transfer Buffers
  volatile uint8_t __attribute__ ((aligned (4))) _dmaft_buf0[DMAFT_BUFFER_SIZE];
  volatile uint8_t __attribute__ ((aligned (4))) _dmaft_buf1[DMAFT_BUFFER_SIZE];
  volatile uint8_t* _dmaft_buffers[2] = {_dmaft_buf0, _dmaft_buf1};
  uint8_t  _dmaft_tx_packet_buffer[64] = {0x52, 0x21}; //0x52, 0x21 is preamble
  char statusmsg[64];


  uint32_t DMAFileTransfer::timeout_start;

  uint8_t DMAFileTransfer::command_port_index_to_uart_index(const int16_t command_port) {
    #if ENABLED(HAS_MULTI_SERIAL)
      return command_port == 1
        ? SERIAL_PORT_2
        : command_port == 0
          ? SERIAL_PORT
          : 0;
    #else
      return command_port == 0
        ? SERIAL_PORT
        : 0;
    #endif
  }

  bool DMAFileTransfer::is_command_port_available_for_dma(const int16_t command_port) {
    return dmaserial_available_on_port(command_port_index_to_uart_index(command_port));
  }

  void DMAFileTransfer::reset_timeout() {
    timeout_start = getCurrentMillis();
    MYSERIAL0.printf("ResetTimeout timeout_start=%lu\n",timeout_start);
  }

  bool DMAFileTransfer::is_timeout_exceed() {
    uint32_t cm = getCurrentMillis();
    // MYSERIAL0.printf("CheckTimeoutExceed timeout_start=%lu PORT_TIMEOUT=%lu cm=%lu\n",timeout_start,DMAFT_TRANSFER_PORT_TIMEOUT,cm);
    // Fixes possible overflow on getCurrentMillis();
    return (cm - timeout_start) > DMAFT_TRANSFER_PORT_TIMEOUT;
  }

  bool DMAFileTransfer::precheck_sd_and_file(char* path) {
    if (card.isPrinting() || card.isPaused()) {
      SERIAL_ERROR_MSG("Printing from SD in progress");
      return false;
    }

    if (card.isFileOpen()) {
      SERIAL_ERROR_MSG("Another file is already opened");
      return false;
    }

    if (!card.isMounted()) {
      card.mount();
      if (!card.isMounted()) {
        SERIAL_ERROR_MSG("Could not mount sd");
        return false;
      }
    }

    if (card.fileExists(path)) {
      SERIAL_ERROR_MSG("File already exists");
      return false;
    }
    return true;
  }

  MarlinSerial* DMAFileTransfer::get_mSerial(const int16_t command_port) {
    return command_port == 0
      ? &MYSERIAL0
      : command_port == 1
        ? &MYSERIAL1
        : NULL;
  }

  const uint8_t* const DMAFileTransfer::create_packet(uint8_t packet_type, const uint8_t* const payload, uint16_t payload_length, uint16_t& data_length)
  {
    //_dmaft_tx_packet_buffer[0] and _dmaft_tx_packet_buffer[1] are filled with preamble already
    uint16_t bytes_filled = 0;

    // First two bytes are never changed
    //_dmaft_tx_packet_buffer[0] = 0x52;
    //_dmaft_tx_packet_buffer[1] = 0x21;
    _dmaft_tx_packet_buffer[2] = packet_type;
    bytes_filled += 3;

    memcpy(&_dmaft_tx_packet_buffer[bytes_filled], payload, payload_length);
    bytes_filled += payload_length;
    //uint32_t checksum = calculate_checksum(_dmaft_tx_packet_buffer, payload_length + 3);
    //MYSERIAL0.printf(".. checksum = %u\n", checksum);
    //memcpy(&_dmaft_tx_packet_buffer[bytes_filled], &checksum, sizeof(checksum));
    //bytes_filled += sizeof(checksum);
    //_dmaft_tx_packet_buffer[bytes_filled] = '\n';
    //bytes_filled++;
    data_length = bytes_filled;

    return _dmaft_tx_packet_buffer;
  }

  void DMAFileTransfer::write_packet(MarlinSerial* mSerial, uint8_t packet_type, const uint8_t* const payload, uint16_t payload_length, bool wait)
  {
    MYSERIAL0.printf("write_packet payload[0] = %u \n", payload[0]);
    MYSERIAL0.printf("write_packet payload[1] = %u \n", payload[1]);
    MYSERIAL0.printf("write_packet payload[2] = %u \n", payload[2]);
    MYSERIAL0.printf("write_packet payload[3] = %u \n", payload[3]);
    MYSERIAL0.printf("write_packet payload[4] = %u \n", payload[4]);
    uint16_t data_length = 0;
    const uint8_t* const data = create_packet(packet_type, payload, payload_length, data_length);
    MYSERIAL0.printf("write_packet dl = %u\n", data_length );
    if (NULL != mSerial) {
      mSerial->write(data, data_length);
    } else {
      dmaserial_write(data, data_length, wait);
    }
  }

  void DMAFileTransfer::write_dmaft_hello(MarlinSerial* mSerial, bool wait=true) {
    /**
     * PT-HELLO:
     *    |        |        |        |        |        |
     *    |PROTOVER|CHKSTYPE|DMABUFSZ|CONNPROP|TIMEOUT |
     */

/*     uint8_t payload[] = {
        DMAFT_PROTOCOL_VERSION,
        DMAFT_CHECKSUM_TYPE,
        DMAFT_BUFFER_SIZE_ID,
        DMAFT_TRANSFER_PORT_SPEED,
        DMAFT_TRANSFER_PORT_TIMEOUT_50MS
        }; */
    mSerial->printf("DMAFT: PT-HELLO|PRV:%u|CHK:%s|BUF:%u|SPD:%lu|RTO:%lu|STO:%lu|\n",
      DMAFT_PROTOCOL_VERSION,
      get_checksum_name(),
      DMAFT_BUFFER_SIZE,
      DMAFT_TRANSFER_PORT_SPEED,
      DMAFT_TRANSFER_PORT_TIMEOUT,
      DMAFT_TRANSFER_PORT_TIMEOUT/2
      );
    //write_packet(mSerial, DMAFT_PT_HELLO, (uint8_t *)buf, buflen * sizeof(char), wait);
    MYSERIAL0.printf("Sent DMAFT_PT_HELLO\n");
    //write_packet(mSerial, DMAFT_PT_RCPREADY, uint8_t[]{DMAFT_PROTOCOL_VERSION,  })
  }

  void DMAFileTransfer::request_send_chunk(const uint8_t chunk_id, bool wait=true) {
    /**
     * PT-REQDATA
     *    |        |
     *    |CHUNK ID|
     */
    const uint8_t payload[] = {chunk_id};
    write_packet(NULL, DMAFT_PT_REQDATA, payload, sizeof(payload), wait);
    MYSERIAL0.printf("Sent DMAFT_PT_REQDATA chunk_id=%u\n", chunk_id);

  }

  void DMAFileTransfer::clean_dma_buffer(uint8_t* buffer) {
    buffer[0] = 0x00;
    buffer[1] = 0x00;
    buffer[2] = 0x00;
    buffer[DMAFT_BUFFER_LAST_BYTE_IDX] = 0x00;
  }

  uint8_t DMAFileTransfer::wait_buffer_received(const uint8_t uart_port_index, const uint8_t* buffer) {
    bool timeout_exceed = false;
    reset_timeout();
    while((!timeout_exceed) && (buffer[DMAFT_BUFFER_LAST_BYTE_IDX] == 0x00)) {
      watchdog_refresh();
      safe_delay(50);
      timeout_exceed = is_timeout_exceed();
    }
    return timeout_exceed
      ? DMAFT_RESULT_TIMEOUT_EXCEED
      : DMAFT_RESULT_OK;
  }

  /**
   *
   * Returns:
   *  OK types:
   *    DMAFT_RESULT_OK           == Chunk processed, request next chunk
   *    DMAFT_RESULT_OK_FINISHED  == Chunk processed, this was last chunk
   *    DMAFT_RESULT_OK_ABORTED   == Host aborted transfer
   *  ERROR types:
   *    DMAFT_RESULT_BUF_FAILED_CPS == Prefix or suffix checks failed
   *    DMAFT_RESULT_BUF_FAILED_CRC == Checksum validation failed (this CAN be not CRC checksum but any chercksum algorythm)
   *    DMAFT_RESULT_WRITE_FAILED   == Failed writing to SD
   *    DMAFT_RESULT_UNEXPECTED_ID  == Received unexpected ID
   *    DMAFT_RESULT_UNKNOWN_PT     == Received unknown packet type
   *
   */
  uint8_t DMAFileTransfer::process_buffer(const uint8_t* buffer, const uint8_t expected_chunk_id) {
    MYSERIAL0.printf("Process buffer chunkid=%hu\n", expected_chunk_id);
    if (!(buffer[0] == _dmaft_tx_packet_buffer[0] && buffer[1] == _dmaft_tx_packet_buffer[1] && buffer[DMAFT_BUFFER_LAST_BYTE_IDX] == (uint8_t)0x0B)) {
      return DMAFT_RESULT_BUF_FAILED_CPS;
    }

    uint32_t calculated_checksum = calculate_checksum((uint8_t *)buffer, DMAFT_BUFFER_SIZE - 5);
    if (calculated_checksum != *((uint32_t *)(&buffer[DMAFT_BUFFER_SIZE - 5]))) {
      return DMAFT_RESULT_BUF_FAILED_CRC;
    }

    uint8_t packet_type = buffer[4];
    switch (packet_type)
    {
      case DMAFT_PT_CHUNK:
        return expected_chunk_id != buffer[3]
          ? DMAFT_RESULT_UNEXPECTED_ID
          : card.write((void *)&buffer[4], DMAFT_BUFFER_SIZE - 9) == -1
            ? DMAFT_RESULT_WRITE_FAILED
            : DMAFT_RESULT_OK;
      case DMAFT_PT_LASTCHUNK: {
        const uint16_t data_size = *((uint16_t *)(&buffer[4]));
        return expected_chunk_id != buffer[3]
          ? DMAFT_RESULT_UNEXPECTED_ID
          : card.write((void *)&buffer[6], data_size) == -1
            ? DMAFT_RESULT_WRITE_FAILED
            : DMAFT_RESULT_OK_FINISHED;
      }
      case DMAFT_PT_ABORTED:
        return DMAFT_RESULT_OK_ABORTED;
      default:
        return DMAFT_RESULT_UNKNOWN_PT;
    }
  }


  bool DMAFileTransfer::receive_file(const int16_t command_port, char* path) {
    MYSERIAL0.printf("dma_ft::receive_file cmd_port: %dh\n", command_port);
    MYSERIAL0.printf("dma_ft::receive_file file: %s\n", path);
    if (!is_command_port_available_for_dma(command_port)) {
       SERIAL_ERROR_MSG("this port does not support DMA");
      return false;
    }
    if (!precheck_sd_and_file(path)) {
      return false;
    }
    MYSERIAL0.write("dma_ft::receive_file checks passed\n");
    // IMPORTANT: Disable all heaters before transferring data!
    thermalManager.disable_all_heaters();
    thermalManager.manage_heater();
    MYSERIAL0.write("dma_ft::receive_file heaters disabled\n");
    uint8_t uart_port_index = command_port_index_to_uart_index(command_port);
    MarlinSerial *mSerial = get_mSerial(command_port);

    ui.set_status((char *)DMAFT_STR_START);
    card.openFileWrite((char *)path);
    MYSERIAL0.write("dma_ft::receive_file file opened\n");

    bool transfer_complete = false;
    bool transfer_success  = true;
    uint8_t current_buffer_index = 0;
    uint8_t chunk_id_being_requested = 0;
    uint8_t chunk_id_being_processed = 0;
    uint8_t retry_count = 0xFF;
    uint8_t process_buffer_result = DMAFT_RESULT_UNKNOWN_PT;

    //save_uart_state(uart_port_index);
    uint32_t time_to_start_after = getCurrentMillis() + DMAFT_TRANSFER_PORT_TIMEOUT;
    write_dmaft_hello(mSerial, true);
    MYSERIAL0.write("HelloSent\n");
    mSerial->flush();
    MYSERIAL0.write("Flush()\n");
    mSerial->end();
    MYSERIAL0.write("SerialStopped\n");

    dmaserial_start(uart_port_index, DMAFT_TRANSFER_PORT_SPEED);
    MYSERIAL0.write("dmaserial_start done\n");

    // Wait for time_to_start_after before start.
    // TODO: Here fix overflow conditions
    while (time_to_start_after < getCurrentMillis()) {
      watchdog_refresh();
      safe_delay(50);
    }
    bool not_first_chunk = false;
    MYSERIAL0.write("BANG!\n");
    while(!transfer_complete && transfer_success) {
      clean_dma_buffer((uint8_t *) _dmaft_buffers[current_buffer_index]);
      dmaserial_set_receive_buffer(uart_port_index, (uint8_t *) _dmaft_buffers[current_buffer_index]);
      request_send_chunk(chunk_id_being_requested);

      // On first chunk we can not process prev buffer because it is empty;
      // Otherwise we process prev buffer while host is sending second one.
      if (not_first_chunk) {
        process_buffer_result = process_buffer((uint8_t *) _dmaft_buffers[1 - current_buffer_index], chunk_id_being_processed); // Process previous buffer
        MYSERIAL0.printf("dma_ft::process_buffer exit code %u", process_buffer_result);
        switch (process_buffer_result)
        {
          case DMAFT_RESULT_OK:
            MYSERIAL0.write("DMAFT_RESULT_OK\n");
            chunk_id_being_processed++;
            break;
          case DMAFT_RESULT_OK_FINISHED:
            MYSERIAL0.write("DMAFT_RESULT_OK_FINISHED\n");
            transfer_complete = true;
            break;
          case DMAFT_RESULT_OK_ABORTED:
            MYSERIAL0.write("DMAFT_RESULT_OK_ABORTED\n");
            transfer_success = false;
            transfer_complete = true;
            break;
          case DMAFT_RESULT_BUF_FAILED_CRC:
          case DMAFT_RESULT_BUF_FAILED_CPS:
          case DMAFT_RESULT_UNEXPECTED_ID:
          case DMAFT_RESULT_UNKNOWN_PT:
            MYSERIAL0.write("FAILED\n");
            retry_count--;
            chunk_id_being_requested = chunk_id_being_processed - 1;
            break;
          case DMAFT_RESULT_WRITE_FAILED:
            MYSERIAL0.write("DMAFT_RESULT_WRITE_FAILED\n");
            transfer_success = false;
            transfer_complete = true;
          default:
            MYSERIAL0.write("default\n");
            transfer_success = false;
            transfer_complete = true;
        } //end of switch (process_buffer_result)
      }
      not_first_chunk = true;

      // Wait buffer received or timeout. On timeout, fail transfer
      if (!transfer_complete && transfer_success && wait_buffer_received(uart_port_index, (uint8_t *)_dmaft_buffers[current_buffer_index]) == DMAFT_RESULT_OK) {
        sprintf(statusmsg,DMAFT_STR_PROCESSING, chunk_id_being_requested);
        ui.set_status(statusmsg, false);
        chunk_id_being_requested++;
        current_buffer_index = 1 - current_buffer_index; // Swap buffer index
      } else {
        transfer_success = false;
        transfer_complete = true;
      }
      MYSERIAL0.write("LE\n");
    } //end of  while(!transfer_complete && transfer_success)
    watchdog_refresh();
    dmaserial_stop();
    mSerial->begin(BAUDRATE);
    MYSERIAL0.write("Serial returned to normal function\n");
    safe_delay(200);
    card.closefile();
    MYSERIAL0.write("File closed\n");

    if (!transfer_success) {
      card.removeFile(card.filename);
      MYSERIAL0.write("File removed\n");
    }
    ui.set_status((char *)DMAFT_STR_FINISH);

    MYSERIAL0.printf("dma_ft::receive_file finished with code %u", process_buffer_result);
    return transfer_success;

  }

/*   uint32_t DMAFileTransfer::get_baudrate_from_transfer_option(const uint8_t dmaft_transfer_port_options) {
    switch (dmaft_transfer_port_options & 0x0F) {
      case DMAFT_XFER_SPD_2400_BAUD    : return 2400;
      case DMAFT_XFER_SPD_115200_BAUD  : return 115200;
      case DMAFT_XFER_SPD_230400_BAUD  : return 230400;
      case DMAFT_XFER_SPD_460800_BAUD  : return 460800;
      case DMAFT_XFER_SPD_921600_BAUD  : return 921600;
      case DMAFT_XFER_SPD_1958400_BAUD : return 1958400;
      case DMAFT_XFER_SPD_2250000_BAUD : return 2250000;
      default:
        return 0;
    }
  } */

#endif
