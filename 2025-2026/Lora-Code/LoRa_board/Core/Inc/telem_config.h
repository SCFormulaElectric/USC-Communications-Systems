/* Build flags for the LoRa telemetry firmware (BUILD_INSTRUCTIONS.md 5.5).
 * Set them with -D in the build configuration; these are the defaults. */
#ifndef TELEM_CONFIG_H
#define TELEM_CONFIG_H

/* 0 = telemetry only, CAN silent (default for the car)
 * 1 = telemetry + diagnostics, CAN normal mode
 * 2 = diagnostics first (batch filled DIAG_RESP/EVENT first, FAST once per second) */
#ifndef TELEM_BAMO_DIAG
#define TELEM_BAMO_DIAG 0
#endif

/* 1 = Bamocar parameter writes allowed (subject to the diag_core safety rules) */
#ifndef DIAG_ALLOW_WRITES
#define DIAG_ALLOW_WRITES 1
#endif

/* 0 = SEND_FRAME only to 0x201; 1 = any ID (bench use only) */
#ifndef DIAG_ANY_ID
#define DIAG_ANY_ID 0
#endif

/* Command key, sent on the radio as bytes (CMD_KEY >> 8), (CMD_KEY & 0xFF) = 5A C3 */
#ifndef CMD_KEY
#define CMD_KEY 0x5AC3u
#endif

#if (TELEM_BAMO_DIAG < 0) || (TELEM_BAMO_DIAG > 2)
#error "TELEM_BAMO_DIAG must be 0, 1 or 2"
#endif

#endif /* TELEM_CONFIG_H */
