/*
 * Copyright (c) 2013-2020 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <assert.h>

#include "Driver_SPI.h"
#include "RP20xx.h"

#define ARM_SPI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_SPI_API_VERSION,
    ARM_SPI_DRV_VERSION};

static uint32_t SPI_GetClock(void)
{
    uint8_t clk_src = (CLOCKS->CLK_PERI_CTRL & CLOCKS_CLK_PERI_CTRL_AUXSRC_Msk) >> CLOCKS_CLK_PERI_CTRL_AUXSRC_Pos;
    switch (clk_src) {
    case 0:    // clock sys
        return SystemCoreClock;
    case 1:    // clksrc_pll_sys
    case 2:    // clksrc_pll_usb
    case 3:    // rosc_clksrc_ph
        break;
    case 4:    // xosc_clksrc
        return 12000000;
    case 5:    // clksrc_gpin0
    case 6:    // clksrc_gpin1
        break;
    }

    return 0;
}

static uint32_t SPI_GetBR(uint32_t max_br, uint16_t *div)
{
    uint32_t spi_clk = SPI_GetClock();
    uint32_t prescale, postdiv;

    assert(max_br <= spi_clk);

    // From pico-sdk repository
    // Find smallest prescale value which puts output frequency in range of
    // post-divide. Prescale is an even number from 2 to 254 inclusive.
    for (prescale = 2; prescale <= 254; prescale += 2) {
        if (spi_clk < (prescale + 2) * 256 * (uint64_t) max_br)
            break;
    }
    assert(prescale <= 254);    // Frequency too low

    // Find largest post-divide which makes output <= baudrate. Post-divide is
    // an integer in the range 1 to 256 inclusive.
    for (postdiv = 256; postdiv > 1; --postdiv) {
        if (spi_clk / (prescale * (postdiv - 1)) > max_br)
            break;
    }

    *div       = prescale;
    *(div + 1) = postdiv;

    return spi_clk / (prescale * postdiv);
}


static void ARM_SPI_SignalEvent(uint32_t event)
{
    // function body
}

ARM_SPI_SignalEvent_t spi_callback[2];

/* Driver Capabilities */
static const ARM_SPI_CAPABILITIES DriverCapabilities = {
    0, /* Reserved (must be zero) */
    1, /* TI Synchronous Serial Interface */
    1, /* Microwire Interface */
    1, /* Signal Mode Fault event: \ref ARM_SPI_EVENT_MODE_FAULT */
    0  /* Reserved (must be zero) */
};

//
//  Functions
//

static ARM_DRIVER_VERSION ARM_SPI_GetVersion(void)
{
    return DriverVersion;
}

static ARM_SPI_CAPABILITIES ARM_SPI_GetCapabilities(void)
{
    return DriverCapabilities;
}

static int32_t ARM_SPI_Initialize(SPI_Type *instance, ARM_SPI_SignalEvent_t cb_event)
{
    // Enable clock
    CLOCKS->CLK_PERI_CTRL = CLOCKS_CLK_PERI_CTRL_ENABLE_Msk;

    // Reset peripheral
    if (instance == SPI0) {
        SET_BIT(RESETS->RESET, RESETS_RESET_spi0_Msk);
        CLEAR_BIT(RESETS->RESET, RESETS_RESET_spi0_Msk);
        while (!(RESETS->RESET_DONE & RESETS_RESET_DONE_spi0_Msk))
            ;
    } else if (instance == SPI1) {
        SET_BIT(RESETS->RESET, RESETS_RESET_spi1_Msk);
        CLEAR_BIT(RESETS->RESET, RESETS_RESET_spi1_Msk);
        while (!(RESETS->RESET_DONE & RESETS_RESET_DONE_spi1_Msk))
            ;
    } else {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    // Set defaults 8bits fram CPOL 0 CPHA 0, frame format motorola
    instance->SSPCR0 = (0x07 << SPI0_SSPCR0_DSS_Pos) |
                       (0x00 << SPI0_SSPCR0_FRF_Pos) |
                       (0x00 << SPI0_SSPCR0_SPO_Pos) |
                       (0x00 << SPI0_SSPCR0_SPH_Pos);

    // Set baudrate to 1MHz
    uint16_t  prescalers[2];
    uint32_t actual_br = SPI_GetBR(1000000, prescalers);
    (void) actual_br;
    instance->SSPCPSR = prescalers[0];
    MODIFY_REG(instance->SSPCR0, SPI0_SSPCR0_SCR_Msk, (prescalers[1] - 1) << SPI0_SSPCR0_SCR_Pos);

    // Enable SPI and set master mode
    instance->SSPCR1 =  SPI0_SSPCR1_SSE_Msk;

    if (instance == SPI0) {
        spi_callback[0] = (cb_event) ? cb_event : ARM_SPI_SignalEvent;
    } else if (instance == SPI1) {
        spi_callback[1] = (cb_event) ? cb_event : ARM_SPI_SignalEvent;
    }

    return ARM_DRIVER_OK;
}

static int32_t ARM_SPI_Uninitialize(SPI_Type *instance)
{
    CLEAR_BIT(instance->SSPCR1, SPI0_SSPCR1_SSE_Msk);

    // Reset peripheral
    if (instance == SPI0) {
        SET_BIT(RESETS->RESET, RESETS_RESET_spi0_Msk);
        CLEAR_BIT(RESETS->RESET, RESETS_RESET_spi0_Msk);
        while (!(RESETS->RESET_DONE & RESETS_RESET_DONE_spi0_Msk))
            ;
    } else if (instance == SPI1) {
        SET_BIT(RESETS->RESET, RESETS_RESET_spi1_Msk);
        CLEAR_BIT(RESETS->RESET, RESETS_RESET_spi1_Msk);
        while (!(RESETS->RESET_DONE & RESETS_RESET_DONE_spi1_Msk))
            ;
    } else {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    return ARM_DRIVER_OK;
}

static int32_t ARM_SPI_PowerControl(SPI_Type *instance, ARM_POWER_STATE state)
{
    switch (state) {
    case ARM_POWER_OFF:
        break;

    case ARM_POWER_LOW:
        break;

    case ARM_POWER_FULL:
        break;
    }
    return ARM_DRIVER_OK;
}

static int32_t ARM_SPI_Send(SPI_Type *instance, const void *data, uint32_t num)
{
    uint8_t         data_sent   = 0;
    uint8_t         frame_size  = (instance->SSPCR0 & SPI0_SSPCR0_DSS_Msk) >> SPI0_SSPCR0_DSS_Pos;
    const uint8_t * data_cast8  = data;
    const uint16_t *data_cast16 = data;
    frame_size++;

    while (num--) {
        // write out dummy byte
        while ((instance->SSPSR & SPI0_SSPSR_TNF_Msk) == 0)
            ;
        instance->SSPDR = (frame_size > 8) ? data_cast16[data_sent] : data_cast8[data_sent];
        // Wait until there is space in FIFO
        while ((instance->SSPSR & SPI0_SSPSR_RNE_Msk) == 0)
            ;
        // Put byte in a buffer
        (void) instance->SSPDR;
        data_sent++;
    }

    return ARM_DRIVER_OK;
}

static int32_t ARM_SPI_Receive(SPI_Type *instance, void *data, uint32_t num)
{
    uint8_t   data_sent   = 0;
    uint8_t   frame_size  = instance->SSPCR0 & SPI0_SSPCR0_DSS_Msk >> SPI0_SSPCR0_DSS_Pos;
    uint8_t * data_cast8  = data;
    uint16_t *data_cast16 = data;
    frame_size++;

    while (num--) {
        // write out dummy byte
        while ((instance->SSPSR & SPI0_SSPSR_TNF_Msk) == 0)
            ;
        instance->SSPDR = 0xFF;
        // Wait until there is space in FIFO
        while ((instance->SSPSR & SPI0_SSPSR_RNE_Msk) == 0)
            ;
        // Put byte in a buffer
        if (frame_size > 8) {
            data_cast16[data_sent++] = instance->SSPDR;
        } else {
            data_cast8[data_sent++] = instance->SSPDR;
        }
    }

    return ARM_DRIVER_OK;
}

static int32_t ARM_SPI_Transfer(SPI_Type *instance, const void *data_out, void *data_in, uint32_t num)
{
    uint8_t         data_sent       = 0;
    uint8_t         frame_size      = instance->SSPCR0 & SPI0_SSPCR0_DSS_Msk >> SPI0_SSPCR0_DSS_Pos;
    uint8_t *       data_in_cast8   = data_in;
    uint16_t *      data_in_cast16  = data_in;
    const uint8_t * data_out_cast8  = data_out;
    const uint16_t *data_out_cast16 = data_out;
    frame_size++;

    while (num--) {
        // write out dummy byte
        while ((instance->SSPSR & SPI0_SSPSR_TNF_Msk) == 0)
            ;
        instance->SSPDR = (frame_size > 8) ? data_out_cast16[data_sent] : data_out_cast8[data_sent];
        // Wait until there is space in FIFO
        while ((instance->SSPSR & SPI0_SSPSR_RNE_Msk) == 0)
            ;
        // Put byte in a buffer
        if (frame_size > 8) {
            data_in_cast16[data_sent++] = instance->SSPDR;
        } else {
            data_in_cast8[data_sent++] = instance->SSPDR;
        }
    }

    return ARM_DRIVER_OK;
}

static uint32_t ARM_SPI_GetDataCount(SPI_Type *instance)
{
    return 0;
}

static int32_t ARM_SPI_Control(SPI_Type *instance, uint32_t control, uint32_t arg)
{
    switch (control & ARM_SPI_CONTROL_Msk) {
    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_SPI_MODE_INACTIVE:    // SPI Inactive
        return ARM_DRIVER_OK;

    case ARM_SPI_MODE_MASTER:    // SPI Master (Output on MOSI, Input on MISO); arg = Bus Speed in bps
        break;

    case ARM_SPI_MODE_SLAVE:    // SPI Slave  (Output on MISO, Input on MOSI)
        break;

    case ARM_SPI_SET_BUS_SPEED:    // Set Bus Speed in bps; arg = value
        break;

    case ARM_SPI_GET_BUS_SPEED:    // Get Bus Speed in bps
        break;

    case ARM_SPI_SET_DEFAULT_TX_VALUE:    // Set default Transmit value; arg = value
        break;

    case ARM_SPI_CONTROL_SS:    // Control Slave Select; arg = 0:inactive, 1:active
        break;

    case ARM_SPI_ABORT_TRANSFER:    // Abort current data transfer
        break;
    }

    return ARM_DRIVER_OK;
}

static ARM_SPI_STATUS ARM_SPI_GetStatus(SPI_Type *instance)
{
    ARM_SPI_STATUS status;
    status.busy       = instance->SSPSR & SPI0_SSPSR_BSY_Msk >> SPI0_SSPSR_BSY_Pos;
    status.data_lost  = (instance->SSPSR & (SPI0_SSPRIS_RORRIS_Msk)) == SPI0_SSPRIS_RORRIS_Msk;
    status.mode_fault = (instance->SSPSR & (SPI0_SSPRIS_RTRIS_Msk)) == SPI0_SSPRIS_RTRIS_Msk;
    return status;
}

// End SPI Interface

#define ARM_SPI_FUNCTIONS(x)                                                               \
    static int32_t ARM_SPI_Initialize_##x(ARM_SPI_SignalEvent_t cb_event)                \
    {                                                                                      \
        return ARM_SPI_Initialize(SPI##x, cb_event);                                       \
    }                                                                                      \
    static int32_t ARM_SPI_Uninitialize_##x(void)                                          \
    {                                                                                      \
        return ARM_SPI_Uninitialize(SPI##x);                                               \
    }                                                                                      \
    static int32_t ARM_SPI_PowerControl_##x(ARM_POWER_STATE state)                         \
    {                                                                                      \
        return ARM_SPI_PowerControl(SPI##x, state);                                        \
    }                                                                                      \
    static int32_t ARM_SPI_Send_##x(const void *data, uint32_t num)                        \
    {                                                                                      \
        return ARM_SPI_Send(SPI##x, data, num);                                            \
    }                                                                                      \
    static int32_t ARM_SPI_Receive_##x(void *data, uint32_t num)                           \
    {                                                                                      \
        return ARM_SPI_Receive(SPI##x, data, num);                                         \
    }                                                                                      \
    static int32_t ARM_SPI_Transfer_##x(const void *data_out, void *data_in, uint32_t num) \
    {                                                                                      \
        return ARM_SPI_Transfer(SPI##x, data_out, data_in, num);                           \
    }                                                                                      \
    static uint32_t ARM_SPI_GetDataCount_##x(void)                                         \
    {                                                                                      \
        return ARM_SPI_GetDataCount(SPI##x);                                               \
    }                                                                                      \
    static int32_t ARM_SPI_Control_##x(uint32_t control, uint32_t arg)                     \
    {                                                                                      \
        return ARM_SPI_Control(SPI##x, control, arg);                                      \
    }                                                                                      \
    static ARM_SPI_STATUS ARM_SPI_GetStatus_##x(void)                                      \
    {                                                                                      \
        return ARM_SPI_GetStatus(SPI##x);                                                  \
    }                                                                                      \
                                                                                           \
    extern ARM_DRIVER_SPI Driver_SPI##x;                                                   \
    ARM_DRIVER_SPI        Driver_SPI##x = {                                                \
        ARM_SPI_GetVersion,                                                         \
        ARM_SPI_GetCapabilities,                                                    \
        ARM_SPI_Initialize_##x,                                                     \
        ARM_SPI_Uninitialize_##x,                                                   \
        ARM_SPI_PowerControl_##x,                                                   \
        ARM_SPI_Send_##x,                                                           \
        ARM_SPI_Receive_##x,                                                        \
        ARM_SPI_Transfer_##x,                                                       \
        ARM_SPI_GetDataCount_##x,                                                   \
        ARM_SPI_Control_##x,                                                        \
        ARM_SPI_GetStatus_##x}

ARM_SPI_FUNCTIONS(0);
ARM_SPI_FUNCTIONS(1);