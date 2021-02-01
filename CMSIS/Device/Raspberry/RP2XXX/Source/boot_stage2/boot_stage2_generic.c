#include <stddef.h>
#include <stdint.h>

#include "RP2040.h"

// ----------------------------------------------------------------------------
// Config section
// ----------------------------------------------------------------------------
// It should be possible to support most flash devices by modifying this section

// The serial flash interface will run at clk_sys/PICO_FLASH_SPI_CLKDIV.
// This must be a positive, even integer.
// The bootrom is very conservative with SPI frequency, but here we should be
// as aggressive as possible.

#ifndef PICO_FLASH_SPI_CLKDIV
    #define PICO_FLASH_SPI_CLKDIV 4
#endif
#if PICO_FLASH_SPI_CLKDIV & 1
    #error PICO_FLASH_SPI_CLKDIV must be even
#endif

// Define interface width: single/dual/quad IO
#define FRAME_FORMAT 0x00    // 0x00 STD, 0x01 DUAL, 0x02 QUAD

// For W25Q080 this is the "Read data fast quad IO" instruction:
#define CMD_READ 0x03

// "Mode bits" are 8 special bits sent immediately after
// the address bits in a "Read Data Fast Quad I/O" command sequence.
// On W25Q080, the four LSBs are don't care, and if MSBs == 0xa, the
// next read does not require the 0xeb instruction prefix.
#define MODE_CONTINUOUS_READ 0xa0

// The number of address + mode bits, divided by 4 (always 4, not function of
// interface width).
#define ADDR_L 6

// How many clocks of Hi-Z following the mode bits. For W25Q080, 4 dummy cycles
// are required.
#define WAIT_CYCLES 4

// If defined, we will read status reg, compare to SREG_DATA, and overwrite
// with our value if the SR doesn't match.
// We do a two-byte write to SR1 (01h cmd) rather than a one-byte write to
// SR2 (31h cmd) as the latter command isn't supported by WX25Q080.
// This isn't great because it will remove block protections.
// A better solution is to use a volatile SR write if your device supports it.
#define PROGRAM_STATUS_REG

#define CMD_WRITE_ENABLE 0x06
#define CMD_READ_STATUS  0x05
#define CMD_READ_STATUS2 0x35
#define CMD_WRITE_STATUS 0x01
#define SREG_DATA        0x02    // Enable quad-SPI mode

void wait_ssi_rdy(void);
uint32_t read_sreg(uint32_t reg);

__attribute__((naked)) __attribute__((section("entry"))) void _stage2_boot(void)
{
    asm("push {LR};");
    // Disable SSI
    XIP_SSI->SSIENR = 0;

    // Set BR
    XIP_SSI->BAUDR = PICO_FLASH_SPI_CLKDIV;

    XIP_SSI->CTRLR0 = (FRAME_FORMAT << XIP_SSI_CTRLR0_FRF_Pos) |
                      (31 << XIP_SSI_CTRLR0_DFS_32_Pos) |
                      (0x03 << XIP_SSI_CTRLR0_TMOD_Pos);

    XIP_SSI->CTRLR1 = 0;

    XIP_SSI->SPI_CTRLR0 = (CMD_READ << XIP_SSI_SPI_CTRLR0_XIP_CMD_Pos) |
                          (ADDR_L << XIP_SSI_SPI_CTRLR0_ADDR_L_Pos) |
                          (0x02 << XIP_SSI_SPI_CTRLR0_INST_L_Pos) |
                          (0x00 << XIP_SSI_SPI_CTRLR0_TRANS_TYPE_Pos);

    XIP_SSI->SSIENR = 1;

    // Bus accesses to the XIP window will now be transparently serviced by the
    // external flash on cache miss. We are ready to run code from flash.


    uint32_t *lr = NULL;
    asm(" \
        pop {r0}; \
        mov %0, r0; \
    ":"=r" (lr)
    :
    :);

    if (lr == 0) {
        SCB->VTOR = 0x10000100;
        __set_MSP(*(uint32_t*)0x10000100);
        ((void (*)(void)) *(uint32_t*)0x10000104)();
    }

    ((void (*)(void)) lr)();
}

void wait_ssi_rdy(void)
{
    uint8_t sr;
    do {
        sr = (XIP_SSI->SR & (XIP_SSI_SR_TFE_Msk | XIP_SSI_SR_BUSY_Msk));
    } while (sr != XIP_SSI_SR_TFE_Msk);
}

uint32_t read_sreg(uint32_t reg)
{
    XIP_SSI->DR0 = reg;
    XIP_SSI->DR0 = reg;

    wait_ssi_rdy();

    (void) XIP_SSI->DR0;
    return XIP_SSI->DR0;
}