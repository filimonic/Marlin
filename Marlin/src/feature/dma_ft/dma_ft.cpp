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
  uint8_t  _dmaft_tx_packet_buffer[64] = {0xB7}; //0x52, 0x21 is preamble
  // Buffer for status screen
  // The simplest LCD supports 20 characters wide, so we need to fit here.
  // Filename is maximum 8.3 characters long
  // Percents are maximum 99
  //                 "0123456789ABCDEF0123"
  //                 ">> FILENAME.GCO: 99%"
  char ui_status_msg[DMAFT_STR_STATUS_LINE_LEN];

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
  }

  bool DMAFileTransfer::is_timeout_exceed() {
    // Fixes possible overflow on getCurrentMillis();
    return (getCurrentMillis() - timeout_start) > DMAFT_TRANSFER_PORT_TIMEOUT;
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
    #if ENABLED(HAS_MULTI_SERIAL)
    return command_port == 0
      ? &MYSERIAL0
      : &MYSERIAL1;
    #else
      return &MYSERIAL0
    #endif
  }

  const uint8_t* const DMAFileTransfer::create_packet(uint8_t packet_type, const uint8_t* const payload, uint16_t payload_length, uint16_t& data_length)
  {
    //_dmaft_tx_packet_buffer[0] and _dmaft_tx_packet_buffer[1] are filled with preamble already
    uint16_t bytes_filled = 0;

    // First two bytes are never changed
    //_dmaft_tx_packet_buffer[0] = 0x52;
    //_dmaft_tx_packet_buffer[1] = 0x21;
    _dmaft_tx_packet_buffer[1] = packet_type;
    bytes_filled += 2;

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

  void DMAFileTransfer::write_packet(MarlinSerial* mSerial, uint8_t packet_type, const uint8_t* const payload, uint16_t payload_length)
  {
    /* MYSERIAL0.printf("write_packet payload[0] = %u \n", payload[0]);
    MYSERIAL0.printf("write_packet payload[1] = %u \n", payload[1]);
    MYSERIAL0.printf("write_packet payload[2] = %u \n", payload[2]);
    MYSERIAL0.printf("write_packet payload[3] = %u \n", payload[3]);
    MYSERIAL0.printf("write_packet payload[4] = %u \n", payload[4]); */
    uint16_t data_length = 0;
    const uint8_t* const data = create_packet(packet_type, payload, payload_length, data_length);
    //MYSERIAL0.printf("write_packet dl = %u\n", data_length );
    if (NULL != mSerial) {
      mSerial->write(data, data_length);
    } else {
      dmaserial_write(data, data_length);
    }
  }

  void DMAFileTransfer::write_dmaft_hello(MarlinSerial* mSerial) {
     mSerial->printf("DMAFT: PT-HELLO|PRV:%u|CHK:%s|BUF:%u|SPD:%lu|RTO:%lu|STO:%lu|\n",
      DMAFT_PROTOCOL_VERSION,
      get_checksum_name(),
      DMAFT_BUFFER_SIZE,
      DMAFT_TRANSFER_PORT_SPEED,
      DMAFT_TRANSFER_PORT_TIMEOUT,
      DMAFT_TRANSFER_PORT_TIMEOUT/2
      );
  }



  FORCE_INLINE void DMAFileTransfer::request_send_chunk(const uint8_t chunk_id) {
    /**
     * PT-REQDATA
     *    |        |
     *    |CHUNK ID|
     */
    write_packet(NULL, DMAFT_PT_REQDATA, &chunk_id, 1);
  }

  void DMAFileTransfer::clean_dma_buffer(uint8_t* buffer) {
    buffer[0] = 0x00;
    buffer[1] = 0x00;
    buffer[2] = 0x00;
    buffer[DMAFT_BUFFER_LAST_BYTE_IDX] = 0x00;
  }

  /**
   * Waits for buffer received
   * returns
   *  DMAFT_RESULT_OK
   *  DMAFT_RESULT_TIMEOUT_EXCEED
   */

  DMAFTResult DMAFileTransfer::wait_buffer_received(const uint8_t uart_port_index, const uint8_t* buffer) {
    bool timeout_exceed = false;
    const uint8_t* watch_addr = &buffer[DMAFT_BUFFER_LAST_BYTE_IDX];
    reset_timeout();

    while((!timeout_exceed) && ( *watch_addr == (uint8_t)0x00)) {
      watchdog_refresh();
      timeout_exceed = is_timeout_exceed();
    }

    return timeout_exceed
      ? DMAFTResult::FAIL_TIMEOUT_EXCEED
      : DMAFTResult::OK;
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
  DMAFTResult DMAFileTransfer::process_buffer(const uint8_t* buffer, const uint8_t expected_chunk_id) {
    if (!(buffer[0] == _dmaft_tx_packet_buffer[0]  && buffer[DMAFT_BUFFER_LAST_BYTE_IDX] == (uint8_t)0x0B)) {
      return DMAFTResult::FAIL_NO_MANDATORY_BYTES;
    }

    uint32_t calculated_checksum = calculate_checksum((uint8_t *)buffer, DMAFT_BUFFER_SIZE - 5);
    if (calculated_checksum != *((uint32_t *)(&buffer[DMAFT_BUFFER_SIZE - 5]))) {
      return DMAFTResult::FAIL_WRONG_CHECKSUM;
    }


    uint8_t packet_type = buffer[1];
    MYSERIAL0.printf("write::process_buffer PT %u\n", packet_type);
    switch (packet_type) {
      case DMAFT_PT_CHUNK:
        MYSERIAL0.printf("write::DMAFT_PT_CHUNK\n");
        return expected_chunk_id != buffer[2]
          ? DMAFTResult::FAIL_UNEXPECTED_ID
          : card.write((void *)&buffer[3], DMAFT_BUFFER_SIZE - 8) == -1
            ? DMAFTResult::FAIL_SD_WRITE_ERROR
            : DMAFTResult::OK;
      case DMAFT_PT_LENCHUNK:
        MYSERIAL0.printf("write::DMAFT_PT_LENCHUNK\n");
        return expected_chunk_id != buffer[2]
          ? DMAFTResult::FAIL_UNEXPECTED_ID
          : card.write((void *)&buffer[5], *((uint16_t *)&buffer[3])) == -1
            ? DMAFTResult::FAIL_SD_WRITE_ERROR
            : DMAFTResult::OK;
      case DMAFT_PT_EOF:
        MYSERIAL0.printf("write::DMAFT_PT_EOF\n");
        return expected_chunk_id != buffer[2]
          ? DMAFTResult::FAIL_UNEXPECTED_ID
          : DMAFTResult::OK_FINISH;
      case DMAFT_PT_ABORTED:
        return DMAFTResult::OK_ABORT;
      default:
        return DMAFTResult::FAIL_UNKNOWN_PACKET_TYPE;
    }
  }


  DMAFTResult DMAFileTransfer::receive_file(const int16_t command_port, char* path, uint32_t filesize) {
    if (!is_command_port_available_for_dma(command_port)) {
       SERIAL_ERROR_MSG("this port does not support DMA");
      return DMAFTResult::FAIL_PORT_INCOMPATIBLE;
    }
    if (!precheck_sd_and_file(path)) {
      return DMAFTResult::FAIL_DEVICE_NOT_READY;
    }
    // !IMPORTANT: Disable all heaters before transferring data!
    thermalManager.disable_all_heaters();
    thermalManager.manage_heater();

    uint8_t uart_port_index = command_port_index_to_uart_index(command_port);
    MarlinSerial *mSerial = get_mSerial(command_port);

    ui.return_to_status();
    ui.update();
    card.openFileWrite((char *)path, filesize);

    uint32_t time_to_start_after = getCurrentMillis() + DMAFT_TRANSFER_PORT_TIMEOUT;
    write_dmaft_hello(mSerial);
    mSerial->flush();
    mSerial->end();

    dmaserial_start(uart_port_index, DMAFT_TRANSFER_PORT_SPEED);

    // Wait for time_to_start_after before start.
    // TODO: Here fix overflow conditions
    while (time_to_start_after > getCurrentMillis()) {
      watchdog_refresh();
      safe_delay(50);
    }
    bool not_first_chunk = false;

    uint8_t current_chunk_id = 0;
    uint8_t current_buffer_index = 1;
    bool receive_loop_complete = false;
    DMAFTResult xfer_result = DMAFTResult::UNKNOWN;

    uint8_t percent_done_prev = 100; // This will force refresh
    uint8_t percent_done_cur  = 0;

    while(!receive_loop_complete) {
      current_buffer_index ^= 0x01; // This switches 0 to 1 or 1 to 0
      clean_dma_buffer((uint8_t *) _dmaft_buffers[current_buffer_index]);
      dmaserial_set_receive_buffer(uart_port_index, (uint8_t *) _dmaft_buffers[current_buffer_index]);
      request_send_chunk(current_chunk_id);

      if (not_first_chunk) {
        xfer_result = process_buffer((uint8_t *)_dmaft_buffers[current_buffer_index ^ 0x01], current_chunk_id - 1);
        if (xfer_result != DMAFTResult::OK) {
          switch (xfer_result)
          {
          case DMAFTResult::OK_FINISH:
          case DMAFTResult::OK_ABORT:
          // Errors here
          default:
            receive_loop_complete = true;
            break;
          }
        }
      }
            // Update on-screen status text
      percent_done_cur = card.percentDone();
      if (percent_done_cur != percent_done_prev) {
        percent_done_prev = percent_done_cur;
        snprintf(ui_status_msg, sizeof(ui_status_msg), DMAFT_STR_STATUS_LINE, percent_done_cur);
        ui.set_status(ui_status_msg, false);

      }
      if (!ui.on_status_screen()) {
        ui.goto_screen(ui.status_screen);
      }
      ui.update();

      if (!receive_loop_complete) {
        xfer_result = wait_buffer_received(uart_port_index, (uint8_t *)_dmaft_buffers[current_buffer_index]);
        if (xfer_result != DMAFTResult::OK) {
          receive_loop_complete = true;
        }
        else {
          current_chunk_id++;
        }
      }
      not_first_chunk = true;


    }

    watchdog_refresh();
    dmaserial_stop();
    mSerial->begin(BAUDRATE);
    safe_delay(200);
    ui.reset_status(true);
    ui.status_message[0] = '\0';
    card.closefile();
    card.release();
    card.mount();


    if (xfer_result != DMAFTResult::OK_FINISH) {
      card.removeFile(card.filename);
    }

    return xfer_result;
}


#endif
