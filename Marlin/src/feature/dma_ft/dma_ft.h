

#pragma once

#include "../../inc/MarlinConfig.h"
#include "stdint.h"

#if ENABLED(DMA_FILE_TRANSFER)

#if ANY(STM32F1)
  #warning "DMA_FILE_TRANSFER is experimental feature"
#else
  #error "Your MCU is not supported by DMA_FILE_TRANSFER feature"
#endif

  #define DMAFT_BUFFER_SIZE  2048

  #define DMAFT_PROTO_ENDFLAG  0x0B
  #define DMAFT_PROTO_VERSION  0x00
  #define DMAFT_PROTO_CHECKSUM 0x00

  #define DMAFT_SERIAL_PARITY_NONE 0x00
  #define DMAFT_SERIAL_PARITY_ODD  0x80

  #define DMAFT_SERIAL_STOPBIT_1 0x00
  #define DMAFT_SERIAL_STOPBIT_2 0x40


  static constexpr uint8_t DMAFT_PROTO_PREAMBLE[2] = {0x52, 0x21};


  /**
   * DMA File transfer is enabled using "M28 D1 <Filename.gco>"
   * In short:
   * "M28D1" starts converts MCU RX pin to DMA input pin and starts special protocol on TX for indication of availability.
   * Protocol is binary. Message always starts from preable.
   *
   * All muti-byte values send in Little Endian byte order
   *
   * |        |        |        |...   ...   ...|...    ...   ...|        |
   * |     PREAMBLE    |PACKTYPE|... PAYLOAD ...|... CHECKSUM ...|END FLAG|
   *
   *
   * ![PREAMBLE]
   * Preamble for each message:
   *    2 bytes
   *    0x52 0x21 | b0101_0010 b0010_0001 | "R!"
   *    This preamble should begin each message of response protocol
   *
   * ![PACKTYPE]
   * Packet type:
   *    Packet type is type of packet.
   *
   *    TX ONLY :
   *      0x00 - TX ONLY [PT-READY]  Ready packet. It is sent with information about protocol version and starts binary receive. After that, Host should wait 1s before writing data
   *      0x01 - TX ONLY [PT-RCPCFM] Data receipt confirmation information
   *
   *    RX ONLY :
   *      0x02 - RX ONLY [PT-DCHUNK] Data chunk from Host to MCU.
   *
   *    RX & TX :
   *      0xF0 - RX & TX [PT-CLOSE]  Connection closed. After that, both sides should take a timeout of 1s and switch to normal operation
   *
   *
   * ![CHECKSUM]
   * Checksum for each payload:
   *    Checksum for each message comes last bytes after message and is calculated for each message on [PREAMBLE], [PACKTYPE] and [PAYLOAD]
   *    Checksum is calculated using algorythm depending on handshake packet and can vary in length
   *
   * ![END FLAG]
   *   End of message flag.
   *     0x0B is always message end flag used in protocol
   *     0xD0 !INTERNALLY! used on MCU to mark memory not filled with data
   *
   * ![PAYLOAD FOR PT-CLOSE] RX & TX
   * TX & RX Payload for Connection close
   *     |        |...    ...    ...|
   *     |REASON  |... ALIGNMENT ...|
   *
   *     [REASON]
   *       0x00 - Transfer Complete
   *       0x01 - Transfer Timeout
   *       0x02 - Data lost, protocol broken
   *       0xF0 - No SD Card
   *       0xF1 - File already exists
   *       0xF2 - Printer busy
   *     [ALIGNMENT]
   *       For TX Alignment is always 0 bytes
   *       For RX Alignment size is to make the whole packet be exactly 64*(DMABUFSZ+1) bytes
   *
   * *[PAYLOAD for PT-READY] TX ONLY
   * TX Payload Ready packet
   *    Contains protocol version, Checksum type, DMA buffer size and DMA baud rate ( 7 bytes )
   *
   *    |        |        |        |        |
   *    |PROTOVER|CHKSTYPE|DMABUFSZ|CONNPROP|
   *
   *    [PROTOVER]
   *      One byte containing protocol version.
   *      Current version is 0x00 which is "version 1"
   *      Value 0xFF is reserved
   *    [CHKSTYPE]
   *      One byte containing checksum type
   *      0x00 means there is no checksum supported
   *      0x01 which is Fletcher-16 checksum and takes 2 bytes (ref https://en.wikipedia.org/wiki/Fletcher%27s_checksum)
   *      Value 0xFF is reserved
   *    [DMABUFSZ]
   *      Size of DMA buffer.
   *      Size is calculated as 64*(DMABUFSZ+1), so, this value shows multiplier of 64.
   *      For example
   *        if DMABUFSZ is 0,  then buffer size is 64*(0+1)  = 64   bytes
   *        if DMABUFSZ is 7,  then buffer size is 64*(7+1)  = 512  bytes
   *        if DMABUFSZ is 15, then buffer size is 64*(15+1) = 1024 bytes
   *        if DMABUFSZ is 63, then buffer size is 64*(63+1) = 4096 bytes
   *    [CONNPROP]
   *      CONNPROP is byte divided into pieces:
   *      |7|6|5|4|3|2|1|0|
   *      |P|S| | |BAUDRAT|
   *
   *      [P] is parity option. 2 bits
   *        0x00 - No parity bit, 8 bits\byte
   *        0x01 - Odd parity bit
   *        EVEN, MARK and SPACE parity checks are not supported
   *        Shift left 7 bits
   *      [S] is stop bit length. 2 bits
   *        0x00 - 1 stop bit
   *        0x01 - 2 stop bits
   *        Shift left 6 bits
   *      [BAUDRAT]
   *      Identifier of baud rate. 4 bits
   *        0x00 - 2400 baud (0.2 KiB/s) - For developement use. No reason to use it in production
   *        0x01 - 115200 baud (14 Kib/s)
   *        0x02 - 230400 baud (28 Kib/s)
   *        0x03 - 460800 baud (56 KiB/s)
   *        0x04 - 921600 baud (112 KiB/s)
   *    Payload for PT-READY example:
   *
   *    TODO: FIX EXAMPLE CONNPROP
   *    NAM |PROTOVER|CHKSTYPE|DMABUFSZ|CONNPROP|
   *    HEX |  0x00  |  0x01  |  0x0F  |  0x14  |
   *    DEC |     0  |     1  |    15  |    20  |
   *              |        |        |        |
   *              |        |        |        \------ b00_01_0100 => Parity: None, Stop: 1.5, Speed 921600 baud\s
   *              |        |        \--------------- DMA size is 64*(15+1) = 64*16 = 1024 bytes
   *              |        \------------------------ Checksum type is 0x01 = Fletcher-16
   *              \--------------------------------- Protocol version is 0x00 means "Version 1"
   *
   * *[PAYLOAD for PT-RCPCFM] TX ONLY
   * TX Payload for Reсeipt Confirmation packet
   * This packet is sent from MCU to Host to indicate that chunk successfully processed.
   *
   *    |        |        |
   *    |CHUNK ID|ERRCODE |
   *
   *    [CHUNK ID]
   *      ID of chunk that was processed.
   *      IDs are incremented from 0 to 255 by host.
   *      After chunk with ID 255 comes packet with ID 0
   *    [ERRCODE]
   *      Error code for this packet
   *      0xAA - Packet received but not processed yet, send next packet
   *      0xAC - Packet processed successfully
   *      0xF1 - Packet processing failed. Send packet again
   *
   * ?[PT-DCHUNK] RX ONLY
   * RX Payload for Data Chunk transfer
   *    |        |        |        |...    ...    ...|...    ...    ...|
   *    |CHUNK ID|   DATA LENGTH   |...   DATA    ...|... ALIGNMENT ...|
   *
   *    [CHUNK ID]
   *      Chunk ID. First chunk must have ID=0.
   *      IDs are incremented by 1. After 255, next chunk is 0.
   *    [DATA LENGTH]
   *      Length of [DATA] in bytes
   *      Maximum data length is (64*(DMABUFSZ+1)) - PREAMBLE (2 bytes) - PACKTYPE (1 byte) - CHECKSUM (var length) - END_FLAG (1 byte)
   *      Length less than maximum allowed (including 0) indicates that this was last chunk
   *    [DATA]
   *      Is data bytes
   *    [ALIGNMENT]
   *      Is packet alignment to make the whole packet be exactly 64*(DMABUFSZ+1) bytes. There can be random bytes.
   *
   * Behaviour:
   *    Host -> MCU  | M28 D1 FileName.gco
   *            MCU  | Checks for ability to write to file
   *            MCU  | If there is no such ability, MCU sends to host [PT-CLOSE] packet with corresponding error code and returns to normal function
   *    MCU  -> Host | If MCU is able to receive data, MCU sends to Host [PT-READY] packet indicating new serial communication settings
   *            MCU  | MCU waits  1000ms before switching RX to DMA mode to BUF1
   *            Host | Host waits 2000ms before starting transmission
   *    Host -> MCU  | Host sends [PT-DCHUNK] to MCU
   *    MCU  -> Host | MCU Detects data is sent by checking last byte of chunk and responds through sending [PT-RCPCFM] with ERRCODE 0xAA to continue send data (Internally is switches to BUF2)
   *    Host -> MCU  | Host sends [PT-DCHUNK] to MCU
   *            MCU  | While host sends data, MCU is checking packet data and writes payload to SD Card
   *    MCU ->  Host | If MCU successfully processed packet, it indicates Host [PT-RCPCFM] with ERRCODE 0xAC to forget this packet from Host memory
   *    MCU ->  Host | If MCU UNsuccessfully processed packet, it indicates Host [PT-RCPCFM] with ERRCODE 0xF1 to send this packet again. This also means that all next packets should be sent again
   *    Host -> MCU  | Host sends [PT-DCHUNK] to MCU with length less than maximum allowed.
   *            MCU  | Receives responds with [PT-RCPCFM] with ERRCODE 0xAA
   *            MCU  | Processes data and gets indication of last chunk
   *    MCU ->  Host | MCU sends PT-CLOSE with 0x00 Transfer complete
   *
  **/


  class DMAFileTransfer {
    public:
      void start_dma_transfer(int16_t serial_port_idx);

    private:
      static int8_t port_idx_to_uart_idx(int16_t serial_port_idx);
      static bool is_uart_rx_dma_capable(int8_t uart_idx);
      void get_prerequisites(int8_t uart_idx);
      void enable_dma_on_port_rx(uint8_t uart_settings);
      bool allocate_dma_buffers();
      void start_dma_to_buffer();

      void *dma_buffers[2];
      uint8_t  dma_buffer_idx;
      #ifdef STM32F1
        uint32_t uart_old_registers[4];
        USART_TypeDef *uart;
        DMA_TypeDef *uart_dma_controller;
        DMA_Channel_TypeDef *uart_rx_dma_channel;
        uint32_t uart_dma_controller_reg_IFCR;

      #endif

  };


#endif
