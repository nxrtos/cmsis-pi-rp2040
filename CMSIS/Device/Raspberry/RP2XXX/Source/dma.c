#include "dma.h"

void DMA_Init(uint32_t channel, tDMA_Config *config)
{
    uint32_t *DMA_CTRL_REG = (uint32_t *) (DMA_BASE + (0x40 * channel) + 0x0C);

    *DMA_CTRL_REG = config->high_prio << DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_Pos |
                    config->data_size | config->request |
                    config->incr_data << DMA_CH0_CTRL_TRIG_INCR_WRITE_Pos |
                    config->incr_peri << DMA_CH0_CTRL_TRIG_INCR_READ_Pos;
}

void DMA_TransferConfig(uint32_t channel, uint32_t read_address, uint32_t write_address, uint32_t size)
{
    uint32_t *DMA_READ_ADDR      = (uint32_t *) (DMA_BASE + (0x40 * channel) + 0x00);
    uint32_t *DMA_WRITE_ADDR     = (uint32_t *) (DMA_BASE + (0x40 * channel) + 0x04);
    uint32_t *DMA_TRANSFER_COUNT = (uint32_t *) (DMA_BASE + (0x40 * channel) + 0x08);

    *DMA_READ_ADDR = read_address;
    *DMA_WRITE_ADDR = write_address;
    *DMA_TRANSFER_COUNT = size;
}

void DMA_TransferStart(uint32_t channel)
{
    uint32_t *DMA_CTRL_REG = (uint32_t *) (DMA_BASE + (0x40 * channel) + 0x0C);

    SET_BIT(*DMA_CTRL_REG, DMA_CH0_CTRL_TRIG_EN_Msk);
}
