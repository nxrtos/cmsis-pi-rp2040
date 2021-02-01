#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "RP20xx.h"
#include "Driver_USART.h"

extern ARM_DRIVER_USART ARM_Driver_USART_(0);

void delay(void) {
    volatile uint32_t cnt_down = 1200000;
    while (cnt_down--) {
        __NOP();
    }
}

int main(void) {
    CLEAR_BIT(RESETS->RESET, RESETS_RESET_io_bank0_Msk);
    while (!(RESETS->RESET_DONE & RESETS_RESET_DONE_io_bank0_Msk));
    // Enables as output;
    SET_BIT(IO_BANK0->GPIO25_CTRL, IO_BANK0_GPIO25_CTRL_OEOVER_Msk);
    IO_BANK0->GPIO16_CTRL = 0x02 << IO_BANK0_GPIO16_CTRL_FUNCSEL_Pos;
    IO_BANK0->GPIO17_CTRL = 0x02 << IO_BANK0_GPIO17_CTRL_FUNCSEL_Pos;
    CLEAR_BIT(PADS_BANK0->GPIO16, PADS_BANK0_GPIO16_PDE_Msk);
    CLEAR_BIT(PADS_BANK0->GPIO17, PADS_BANK0_GPIO17_PDE_Msk);
    SET_BIT(PADS_BANK0->GPIO16, PADS_BANK0_GPIO16_PUE_Msk);
    SET_BIT(PADS_BANK0->GPIO17, PADS_BANK0_GPIO17_PUE_Msk);

    ARM_Driver_USART_(0).Initialize(NULL);
    while(1) {
        ARM_Driver_USART_(0).Send("Hello World\r\n", strlen("Hello World\r\n"));
        MODIFY_REG(IO_BANK0->GPIO25_CTRL, IO_BANK0_GPIO25_CTRL_OUTOVER_Msk, 0x02 << IO_BANK0_GPIO25_CTRL_OUTOVER_Pos);
        delay();
        MODIFY_REG(IO_BANK0->GPIO25_CTRL, IO_BANK0_GPIO25_CTRL_OUTOVER_Msk, 0x03 << IO_BANK0_GPIO25_CTRL_OUTOVER_Pos);
        delay();
    }
}