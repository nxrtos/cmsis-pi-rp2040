#ifndef __PICO_PIO_H
#define __PICO_PIO_H

#define SWDIO_PIN 4
#define SWCLK_PIN 5

#define MSB_FIRST 0
#define LSB_FIRST 1

typedef struct _pio_sm_exec_config {
    uint32_t STATUS_N : 4;
    uint32_t STATUS_SEL : 1;
    uint32_t resv : 2;
    uint32_t WRAP_BOTTOM : 5;
    uint32_t WRAP_TOP : 5;
    uint32_t OUT_STICKY : 1;
    uint32_t INLINE_OUT_EN : 1;
    uint32_t OUT_EN_SEL : 5;
    uint32_t JMP_PIN : 5;
    uint32_t SIDE_PINDIR : 1;
    uint32_t SIDE_EN : 1;
    uint32_t EXEC_STALLED : 1;
} pio_sm_exec_config;

typedef struct _pio_sm_shift_config {
    uint32_t : 16;
    uint32_t AUTOPUSH : 1;
    uint32_t AUTOPULL : 1;
    uint32_t IN_SHIFTDIR : 1;
    uint32_t OUT_SHIFTDIR : 1;
    uint32_t PUSH_THRESH : 5;
    uint32_t PULL_THRESH : 5;
    uint32_t FJOIN_TX : 1;
    uint32_t FJOIN_RX : 1;
} pio_sm_shift_config;

typedef struct _pio_sm_pinctrl_config {
    uint32_t OUT_BASE : 5;
    uint32_t SET_BASE : 5;
    uint32_t SIDESET_BASE : 5;
    uint32_t IN_BASE : 5;
    uint32_t OUT_COUNT : 6;
    uint32_t SET_COUNT : 3;
    uint32_t SIDESET_COUNT : 3;
} pio_sm_pinctrl_config;

typedef struct _pio_sm_registers {
    volatile uint32_t CLKDIV;
    volatile uint32_t EXECCTRL;
    volatile uint32_t SHIFTCTRL;
    volatile uint32_t ADDR;
    volatile uint32_t INSTR;
    volatile uint32_t PINCTRL;
} pio_sm_registers;

extern volatile pio_sm_registers *PIO0_SM;
extern volatile pio_sm_registers *PIO1_SM;

#define PIO_SM_0 (1 << 0)
#define PIO_SM_1 (1 << 1)
#define PIO_SM_2 (1 << 2)
#define PIO_SM_3 (1 << 3)

#define PIO_SET_BASE_SET(PIOx, sm, base, count) (MODIFY_REG(##PIOx##_SM[sm].PINCTRL, PIO0_SM0_PINCTRL_SET_BASE_Msk | PIO0_SM0_PINCTRL_SET_COUNT_Msk, \
                                                            (base) << PIO0_SM0_PINCTRL_SET_BASE_Pos | (count) << PIO0_SM0_PINCTRL_SET_COUNT_Pos))

#define PIO_SET_BASE_SIDESET(PIOx, sm, base, count) (MODIFY_REG(##PIOx##_SM[sm].PINCTRL, PIO0_SM0_PINCTRL_SIDESET_BASE_Msk | PIO0_SM0_PINCTRL_SIDESET_COUNT_Msk, \
                                                                (base) << PIO0_SM0_PINCTRL_SIDESET_BASE_Pos | (count) << PIO0_SM0_PINCTRL_SIDESET_COUNT_Pos))

#define PIO_SET_BASE_OUT(PIOx, sm, base, count) (MODIFY_REG(##PIOx##_SM[sm].PINCTRL, PIO0_SM0_PINCTRL_OUT_BASE_Msk | PIO0_SM0_PINCTRL_OUT_COUNT_Msk, \
                                                            (base) << PIO0_SM0_PINCTRL_OUT_BASE_Pos | (count) << PIO0_SM0_PINCTRL_OUT_COUNT_Pos))

#define PIO_SET_BASE_IN(PIOx, sm, base) (MODIFY_REG(##PIOx##_SM[sm].PINCTRL, PIO0_SM0_PINCTRL_IN_BASE_Msk, \
                                                    (base) << PIO0_SM0_PINCTRL_IN_BASE_Pos))

#define PIO_SET_PC(PIOx, sm, pc) (PIOx##_SM[sm].INSTR = pio_encode_jmp(pc))

#define PIO_RX_FIFO(PIOx, sm) (((volatile uint32_t *) &PIOx->RXF0)[sm])
#define PIO_TX_FIFO(PIOx, sm) (((volatile uint32_t *) &PIOx->TXF0)[sm])

#define PIO_GET_TX_FIFO_LEVEL(PIOx, sm) (PIOx->FLEVEL >> (sm * 0x08) & 0xF)
#define PIO_GET_RX_FIFO_LEVEL(PIOx, sm) (PIOx->FLEVEL >> (sm * 0x08 + 0x04) & 0xF)

/**
 * @brief Get current SM configuration
 * 
 * @param PIOx 
 * @param sm state machine number (0 - 3)
 * @param div requested clock_divider value
 * @param exec_config pointer to a pio_sm_exec_config structure
 * @param shift_config pointere to a pio_sm_shift_config structure
 */
void pio_get_config_sm(PIO0_Type *PIOx, uint8_t sm, float *div, pio_sm_exec_config *exec_config, pio_sm_shift_config *shift_config);

/**
 * @brief Set new SM configuration
 * 
 * @param PIOx 
 * @param sm state machine number (0 - 3)
 * @param div requested clock_divider value
 * @param exec_config pointer to a pio_sm_exec_config structure
 * @param shift_config pointere to a pio_sm_shift_config structure
 */
void pio_set_config_sm(PIO0_Type *PIOx, uint8_t sm, float div, pio_sm_exec_config *exec_config, pio_sm_shift_config *shift_config);

/**
 * @brief Set new pinctrl configuration
 * 
 * @param PIOx 
 * @param sm state machine number (0 - 3)
 * @param pinctrl_config pointer to a pio_sm_pinctrl_config structure
 */
void pio_set_pinctrl_sm(PIO0_Type *PIOx, uint8_t sm, pio_sm_pinctrl_config *pinctrl_config);

/**
 * @brief Enable State Machine
 * 
 * @param PIOx 
 * @param sm state machine bitmask (for example to enable sm 1 us 0x1)
 */
void pio_enable(PIO0_Type *PIOx, uint8_t sm);

/**
 * @brief Disable State Machine
 * 
 * @param PIOx 
 * @param sm state machine bitmask (for example to enable sm 1 us 0x1)
 */
void pio_disable(PIO0_Type *PIOx, uint8_t sm);

/**
 * @brief Execute instruction on specific state machine
 * 
 * @param PIOx 
 * @param sm state machine number (0 - 3)
 * @param instr 16bit PIO instruction
 */
void pio_execute_instr(PIO0_Type *PIOx, uint8_t sm, uint16_t instr);

/**
 * @brief 
 * 
 * @param PIOx 
 * @param buffer 
 * @param size 
 */
void pio_program(PIO0_Type *PIOx, const uint16_t *buffer, uint16_t size);

uint32_t pio_set_clock(PIO0_Type *PIOx, uint8_t sm, float divider);

/**
 * @brief 
 * 
 * @param PIOx 
 */
void pio_init(PIO0_Type *PIOx);

/**
 * @brief 
 * 
 * @param PIOx 
 */
void pio_deinit(PIO0_Type *PIOx);

/**
 * @brief 
 * 
 * @param PIOx 
 * @param sm 
 */
void pio_drain_fifo(PIO0_Type *PIOx, uint8_t sm);

#endif