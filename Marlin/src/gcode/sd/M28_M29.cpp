/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(SDSUPPORT)

#include "../gcode.h"
#include "../../sd/cardreader.h"

#if ENABLED(DMA_FILE_TRANSFER)
  #include "../../feature/dma_ft/dma_ft.h"
#endif

#if HAS_MULTI_SERIAL
  #include "../queue.h"
#endif

#define DEBUG_OUT EITHER(MARLIN_DEV_MODE, DEBUG_DMAFT)
#include "../../core/debug_out.h"

/**
 * M28: Start SD Write
 */
void GcodeSuite::M28() {

  // TODO: Patch this to support both BINARY_FILE_TRANSFER and DMA_FILE_TRANSFER modes simultaneously

  #if ENABLED(BINARY_FILE_TRANSFER)

    bool binary_mode = false;
    char *p = parser.string_arg;
    if (p[0] == 'B' && NUMERIC(p[1])) {
      binary_mode = p[1] > '0';
      p += 2;
      while (*p == ' ') ++p;
    }

    // Binary transfer mode
    if ((card.flag.binary_mode = binary_mode)) {
      SERIAL_ECHO_MSG("Switching to Binary Protocol");
      TERN_(HAS_MULTI_SERIAL, card.transfer_port_index = queue.port[queue.index_r]);
    }
    else
      card.openFileWrite(p);

  #else
    #if ENABLED(DMA_FILE_TRANSFER)
      bool dma_transfer_mode = false;
      char *p = parser.string_arg;
      if (p[0] == 'D' && NUMERIC(p[1])) {
        dma_transfer_mode = p[1] > '0';
        p += 2;
        while (*p == ' ') ++p;
      }

      // DMA transfer mode
      if ((card.flag.dma_transfer_mode = dma_transfer_mode)) {
        const int16_t command_port = queue.command_port();
        DEBUG_ECHOLNPAIR("M28: requested DMA FT over Command Port ", command_port);
        bool uart_dma_available = DMAFileTransfer::is_command_port_available_for_dma(command_port);
        DEBUG_ECHOLNPAIR("M28: DMA available: ", (uart_dma_available == true ? "True" : "False"));
        TERN_(HAS_MULTI_SERIAL, card.transfer_port_index = command_port);
        bool res = DMAFileTransfer::receive_file(command_port, p);
        DEBUG_ECHOLNPAIR("M28: DMA complete: ", res);
      }
      else
        card.openFileWrite(p);
    #else
      card.openFileWrite(parser.string_arg);
    #endif

  #endif
}

/**
 * M29: Stop SD Write
 * (Processed in write-to-file routine)
 */
void GcodeSuite::M29() {
  card.flag.saving = false;
}

#endif // SDSUPPORT
