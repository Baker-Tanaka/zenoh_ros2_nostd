/* RP2350 memory layout — 4 MB external QSPI Flash + 520 KB SRAM */
MEMORY {
    /* Main flash (RP2350 bootrom handles the 2nd-stage bootloader internally — no BOOT2 section needed) */
    FLASH : ORIGIN = 0x10000000, LENGTH = 4096K
    /* On-chip SRAM (520 KB) */
    RAM   : ORIGIN = 0x20000000, LENGTH = 520K
}
