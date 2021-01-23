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
  char ftest[] = "FILETEST.GCO\0";
  uint8_t fdat[DMAFT_BUFFER_SIZE];
  for (int i = 0; i < DMAFT_BUFFER_SIZE; i++) {
    fdat[i] = i && 0xFF;
  }
  card.removeFile(ftest);
  card.openFileWrite(ftest);
  int16_t write = card.write(fdat, DMAFT_BUFFER_SIZE);
  MYSERIAL0.printf("WRITE %d\n", write);
  card.closefile();

  card.openFileRead(ftest);
  MYSERIAL0.printf("FILESIZE %u\n", card.getFileSize());
  int16_t read = card.read(fdat, DMAFT_BUFFER_SIZE/2);
  MYSERIAL0.printf("READ %d\n", read);
  read = card.read(fdat, DMAFT_BUFFER_SIZE/2);
  MYSERIAL0.printf("READ %d\n", read);
  read = card.read(fdat, DMAFT_BUFFER_SIZE/2);
  MYSERIAL0.printf("READ %d\n", read);
  card.closefile();
  return;

  char* p = parser.string_arg;
  // Trim end spaces
  char* e = strchr(p, '\0');
  while (*e == ' ') --e;
  e[0] = '\0';
  // Trim start spaces is unnecessarry

  #if ANY(BINARY_FILE_TRANSFER, DMA_FILE_TRANSFER)
    // If arguments do not contain spaces, we assume this is filename, even if it starts with B or D.
    // This means normal file transfer to SD card
    if (strchr(p, ' ') == NULL) {
      card.openFileWrite(p);
    }
    #if ENABLED(BINARY_FILE_TRANSFER)
      else if (strlen(p) >= 2 && p[0] == 'B' && NUMERIC(p[1])) { // Assume format "M28 B1 FILENAME"
        card.flag.binary_mode = p[1] > '0' ;
        p+=2;
        while (*p == ' ') ++p;
        SERIAL_ECHO_MSG("Switching to Binary Protocol");
        TERN_(HAS_MULTI_SERIAL, card.transfer_port_index = queue.port[queue.index_r]);
        card.openFileWrite(p);
      }
    #endif //ENABLED(BINARY_FILE_TRANSFER)

    #if ENABLED(DMA_FILE_TRANSFER)
      else if (strlen(p) >= 2 && p[0] == 'D' && NUMERIC(p[1])) {
        uint32_t filesize = strtoul(&p[1], &p, 10);
        while (*p == ' ') ++p;
        SERIAL_ECHO_MSG("Switching to DMAFT Protocol");
        DMAFTResult result = DMAFileTransfer::receive_file(queue.command_port(), p, filesize);
        MYSERIAL0.printf("DMAFT Result: %u\n", result);
      }
    #endif // ENABLED(DMA_FILE_TRANSFER)
    else {
      card.openFileWrite(p);
    }
  #else // ANY(BINARY_FILE_TRANSFER, DMA_FILE_TRANSFER)
    card.openFileWrite(p);
  #endif // ANY(BINARY_FILE_TRANSFER, DMA_FILE_TRANSFER)
}

/**
 * M29: Stop SD Write
 * (Processed in write-to-file routine)
 */
void GcodeSuite::M29() {
  card.flag.saving = false;
}

#endif // SDSUPPORT
