/* RP2040 memory layout — 2 MB external QSPI Flash + 264 KB SRAM */
MEMORY {
    /* Second-stage bootloader in the first 256 bytes of Flash */
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    /* Main flash (after bootloader) */
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    /* On-chip SRAM (SRAM0–SRAM3 are 4 × 64 KB, contiguous) */
    RAM   : ORIGIN = 0x20000000, LENGTH = 264K
}

SECTIONS {
    /* Place the second-stage bootloader at the very start of flash */
    .boot2 ORIGIN(BOOT2) : {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;
