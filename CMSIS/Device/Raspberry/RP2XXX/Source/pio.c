/**
 * @file pio.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-02-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <assert.h>
#include <stdint.h>

#include "RP20xx.h"

#include "pio.h"
#include "pio_encoder.h"

volatile pio_sm_registers *PIO0_SM = (pio_sm_registers *) (PIO0_BASE + 0x0C8);
volatile pio_sm_registers *PIO1_SM = (pio_sm_registers *) (PIO1_BASE + 0x0C8);

static uint32_t pio_calculate_div(float div)
{
    uint16_t integer = (uint16_t) div;
    uint8_t  frac    = ((div - integer) * 256);
    return (integer << PIO0_SM0_CLKDIV_INT_Pos | frac << PIO0_SM0_CLKDIV_FRAC_Pos);
}

void pio_get_config_sm(PIO0_Type *PIOx, uint8_t sm, float *div, pio_sm_exec_config *exec_config, pio_sm_shift_config *shift_config)
{
    assert(PIOx == PIO0 || PIOx == PIO1);
    assert(sm < 4);

    volatile pio_sm_registers *PIO_SM;

    if (PIOx == PIO0) {
        PIO_SM = PIO0_SM;
    } else {
        PIO_SM = PIO1_SM;
    }
    *div                       = (float) PIO_SM[sm].CLKDIV;
    *(uint32_t *) exec_config  = PIO_SM[sm].EXECCTRL;
    *(uint32_t *) shift_config = PIO_SM[sm].SHIFTCTRL;
}

void pio_set_config_sm(PIO0_Type *PIOx, uint8_t sm, float div, pio_sm_exec_config *exec_config, pio_sm_shift_config *shift_config)
{
    assert(PIOx == PIO0 || PIOx == PIO1);
    assert(sm < 4);

    volatile pio_sm_registers *PIO_SM;

    if (PIOx == PIO0) {
        PIO_SM = PIO0_SM;
    } else {
        PIO_SM = PIO1_SM;
    }

    PIO_SM[sm].CLKDIV    = pio_calculate_div(div);
    PIO_SM[sm].EXECCTRL  = *(uint32_t *) exec_config;
    PIO_SM[sm].SHIFTCTRL = *(uint32_t *) shift_config;
}

void pio_set_pinctrl_sm(PIO0_Type *PIOx, uint8_t sm, pio_sm_pinctrl_config *pinctrl_config)
{
    assert(PIOx == PIO0 || PIOx == PIO1);
    assert(sm < 4);

    volatile pio_sm_registers *PIO_SM;

    if (PIOx == PIO0) {
        PIO_SM = PIO0_SM;
    } else {
        PIO_SM = PIO1_SM;
    }

    PIO_SM[sm].PINCTRL = *(uint32_t *) pinctrl_config;
}

void pio_enable(PIO0_Type *PIOx, uint8_t sm)
{
    assert(PIOx == PIO0 || PIOx == PIO1);
    assert(sm != 0 && sm <= 0xF);

    uint32_t mask = (sm << PIO0_CTRL_SM_RESTART_Pos) |
                    (sm << PIO0_CTRL_SM_ENABLE_Pos) | (sm << PIO0_CTRL_CLKDIV_RESTART_Pos);
    uint32_t value = (sm << PIO0_CTRL_SM_ENABLE_Pos);

    MODIFY_REG(PIOx->CTRL, mask, value);
}

void pio_disable(PIO0_Type *PIOx, uint8_t sm)
{
    assert(PIOx == PIO0 || PIOx == PIO1);
    assert(sm != 0 && sm <= 0xF);

    uint32_t mask = (sm << PIO0_CTRL_SM_RESTART_Pos) |
                    (sm << PIO0_CTRL_SM_ENABLE_Pos) | (sm << PIO0_CTRL_CLKDIV_RESTART_Pos);
    uint32_t value = (sm << PIO0_CTRL_SM_RESTART_Pos) | (sm << PIO0_CTRL_CLKDIV_RESTART_Pos);

    MODIFY_REG(PIOx->CTRL, mask, value);
}

void pio_execute_instr(PIO0_Type *PIOx, uint8_t sm, uint16_t instr)
{
    assert(PIOx == PIO0 || PIOx == PIO1);
    assert(sm < 4);

    volatile pio_sm_registers *PIO_SM;
    if (PIOx == PIO0) {
        PIO_SM = PIO0_SM;
    } else {
        PIO_SM = PIO1_SM;
    }
    PIO_SM[sm].INSTR = instr;
    while (PIO_SM[sm].EXECCTRL & PIO0_SM0_EXECCTRL_EXEC_STALLED_Msk)
        ;
}

void pio_program(PIO0_Type *PIOx, const uint16_t *buffer, uint16_t size)
{
    assert(PIOx == PIO0 || PIOx == PIO1);
    assert(size <= 32);

    volatile uint32_t *pio_instr_mem = &PIOx->INSTR_MEM0;
    const uint16_t *   pio_buffer    = buffer;

    for (int i = 0; i < size; i++) {
        *pio_instr_mem++ = *pio_buffer++;
    }
}

void pio_drain_fifo(PIO0_Type *PIOx, uint8_t sm)
{
    volatile pio_sm_registers *PIO_SM;
    if (PIOx == PIO0) {
        PIO_SM = PIO0_SM;
    } else {
        PIO_SM = PIO1_SM;
    }

    uint16_t instr = (PIO_SM[sm].SHIFTCTRL & PIO0_SM0_SHIFTCTRL_AUTOPULL_Msk) ? pio_encode_out(pio_null, 32) : pio_encode_pull(false, false);

    while (PIO_GET_TX_FIFO_LEVEL(PIOx, sm)) {
        pio_execute_instr(PIOx, sm, instr);
    }

    while (PIO_GET_RX_FIFO_LEVEL(PIOx, sm)) {
        (void) PIO_RX_FIFO(PIO0, sm);
    }
}

uint32_t pio_set_clock(PIO0_Type *PIOx, uint8_t sm, float divider)
{
    uint32_t                   clkdiv_value = pio_calculate_div(divider);
    volatile pio_sm_registers *PIO_SM;
    if (PIOx == PIO0) {
        PIO_SM = PIO0_SM;
    } else {
        PIO_SM = PIO1_SM;
    }

    PIO_SM->CLKDIV        = clkdiv_value;
    uint32_t real_divider = (uint32_t) (clkdiv_value * 256.0f) / 256;
    return SystemCoreClock / real_divider;
}

void pio_init(PIO0_Type *PIOx)
{
    assert(PIOx == PIO0 || PIOx == PIO1);

    if (PIOx == PIO0) {
        SET_BIT(RESETS->RESET, RESETS_RESET_pio0_Msk);
        while ((RESETS->RESET & RESETS_RESET_DONE_pio0_Msk) == 0)
            ;
        CLEAR_BIT(RESETS->RESET, RESETS_RESET_pio0_Msk);
    } else {
        SET_BIT(RESETS->RESET, RESETS_RESET_pio1_Msk);
        while ((RESETS->RESET & RESETS_RESET_DONE_pio1_Msk) == 0)
            ;
        CLEAR_BIT(RESETS->RESET, RESETS_RESET_pio1_Msk);
    }

    SET_BIT(PIOx->CTRL, PIO0_CTRL_CLKDIV_RESTART_Msk | PIO0_CTRL_SM_RESTART_Msk);
    SET_BIT(PIOx->FDEBUG, PIO0_FDEBUG_RXSTALL_Msk | PIO0_FDEBUG_RXUNDER_Msk |
                              PIO0_FDEBUG_TXSTALL_Msk | PIO0_FDEBUG_TXOVER_Msk);
}

void pio_deinit(PIO0_Type *PIOx)
{
    assert(PIOx == PIO0 || PIOx == PIO1);

    if (PIOx == PIO0) {
        SET_BIT(RESETS->RESET, RESETS_RESET_pio0_Msk);
    } else {
        SET_BIT(RESETS->RESET, RESETS_RESET_pio1_Msk);
    }
}