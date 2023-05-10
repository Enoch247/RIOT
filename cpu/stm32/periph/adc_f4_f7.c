/*
 * Copyright (C) 2014-2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_stm32
 * @ingroup     drivers_periph_adc
 * @{
 *
 * @file
 * @brief       Low-level ADC driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include "cpu.h"
#include "mutex.h"
#include "periph/adc.h"
#include "periph_conf.h"
#include "periph/cpu_dma.h"
#include "periph/vbat.h"

/**
 * @brief   Maximum allowed ADC clock speed
 */
#define MAX_ADC_SPEED           MHZ(12)

/**
 * @brief   Maximum sampling time for each channel (480 cycles)
 *          T_CONV[µs] = (RESOLUTION[bits] + SMP[cycles]) / CLOCK_SPEED[MHz]
 */
#define MAX_ADC_SMP             (7u)

/**
 * @brief   Default VBAT undefined value
 */
#ifndef VBAT_ADC
#define VBAT_ADC    ADC_UNDEF
#endif

/**
 * @brief   Allocate locks for all three available ADC devices
 */
static mutex_t locks[] = {
#if ADC_DEVS > 1
    MUTEX_INIT,
#endif
#if ADC_DEVS > 2
    MUTEX_INIT,
#endif
    MUTEX_INIT
};

static inline ADC_TypeDef *dev(adc_t line)
{
    return (ADC_TypeDef *)(ADC1_BASE + (adc_config[line].dev << 8));
}

static inline void prep(adc_t line)
{
    mutex_lock(&locks[adc_config[line].dev]);
    periph_clk_en(APB2, (RCC_APB2ENR_ADC1EN << adc_config[line].dev));
}

static inline void done(adc_t line)
{
    periph_clk_dis(APB2, (RCC_APB2ENR_ADC1EN << adc_config[line].dev));
    mutex_unlock(&locks[adc_config[line].dev]);
}

int adc_init(adc_t line)
{
    uint32_t clk_div = 2;

    /* check if the line is valid */
    if (line >= ADC_NUMOF) {
        return -1;
    }

    /* lock and power-on the device */
    prep(line);

    /* configure the pin */
    if (adc_config[line].pin != GPIO_UNDEF) {
        gpio_init_analog(adc_config[line].pin);
    }
    /* set sequence length to 1 conversion and enable the ADC device */
    dev(line)->SQR1 = 0;
    dev(line)->CR2 = ADC_CR2_ADON;
    /* set clock prescaler to get the maximal possible ADC clock value */
    for (clk_div = 2; clk_div < 8; clk_div += 2) {
        if ((periph_apb_clk(APB2) / clk_div) <= MAX_ADC_SPEED) {
            break;
        }
    }
    ADC->CCR = ((clk_div / 2) - 1) << 16;
    /* set sampling time to the maximum */
    if (adc_config[line].chan >= 10) {
        dev(line)->SMPR1 &= ~(MAX_ADC_SMP << (3 * (adc_config[line].chan - 10)));
        dev(line)->SMPR1 |= MAX_ADC_SMP << (3 * (adc_config[line].chan - 10));
    }
    else {
        dev(line)->SMPR1 &= ~(MAX_ADC_SMP << (3 * adc_config[line].chan));
        dev(line)->SMPR2 |= MAX_ADC_SMP << (3 * adc_config[line].chan);
    }
    /* free the device again */
    done(line);
    return 0;
}

int32_t adc_sample(adc_t line, adc_res_t res)
{
    int sample;

    /* check if resolution is applicable */
    if (res & 0xff) {
        return -1;
    }

    /* lock and power on the ADC device  */
    prep(line);
    /* check if this channel is an internal ADC channel */
    if (IS_USED(MODULE_PERIPH_VBAT) && line == VBAT_ADC) {
        vbat_enable();
    }
    /* set resolution and conversion channel */
    dev(line)->CR1 = res;
    dev(line)->SQR3 = adc_config[line].chan;
    /* start conversion and wait for results */
    dev(line)->CR2 |= ADC_CR2_SWSTART;
    while (!(dev(line)->SR & ADC_SR_EOC)) {}
    /* finally read sample and reset the STRT bit in the status register */
    sample = (int)dev(line)->DR;
    /* check if this channel was an internal ADC channel */
    if (IS_USED(MODULE_PERIPH_VBAT) && line == VBAT_ADC) {
        vbat_disable();
    }
    /* power off and unlock device again */
    done(line);

    return sample;
}

#ifdef MODULE_PERIPH_ADC_BURST
int adc_sample_burst(adc_t line, adc_res_t res, void* buf, size_t count,
    int adc_trg)
{
    int result;
    dma_t dma = adc_config[line].dma;
    int dma_chan = adc_config[line].dma_chan;
    volatile void *periph_addr = &(dev(line)->DR);
    int dma_size = DMA_DATA_WIDTH_BYTE;

    /* check if resolution is applicable */
    if (res & 0x3) {
        return -1;
    }

    /* calcualte the size of each DMA transfer */
    switch (res)
    {
        case ADC_RES_6BIT:
        case ADC_RES_8BIT:
        dma_size = DMA_DATA_WIDTH_BYTE;
        break;

        case ADC_RES_10BIT:
        case ADC_RES_12BIT:
        case ADC_RES_14BIT:
        case ADC_RES_16BIT:
        dma_size = DMA_DATA_WIDTH_HALF_WORD;
        break;
    }

    /* lock and power on the ADC device */
    prep(line);

    /* check if this is the VBAT line */
    if (IS_USED(MODULE_PERIPH_VBAT) && line == VBAT_ADC) {
        vbat_enable();
    }

    /* set resolution and conversion channel */
    dev(line)->CR1 = res;
    dev(line)->SQR3 = adc_config[line].chan;

    dma_acquire(dma);

    /* get DMA ready to receive data from ADC */
    result = dma_configure(
        dma, dma_chan, periph_addr, buf, count,
        DMA_PERIPH_TO_MEM,
        dma_size | DMA_INC_DST_ADDR
        );
    if (result < 0)
    {
        // release locks and resources held
        dma_release(dma);
        done(line);

        return -1;
    }

    dma_start(dma);

    /* enable ADC to DMA */
    dev(line)->CR2 |= ADC_CR2_DMA;
    //dev(line)->CR2 |= ADC_CR2_DDS; // DMA circular mode

    /* set sample trigger source */
    dev(line)->CR2 &= ~ADC_CR2_EXTSEL;
    dev(line)->CR2 |= adc_trg << ADC_CR2_EXTSEL_Pos;

    /* enable external trigger, and trigger on its rising edge */
    dev(line)->CR2 &= ~ADC_CR2_EXTEN;
    dev(line)->CR2 |= 1 << ADC_CR2_EXTEN_Pos;

    return 0;
}
#endif /* MODULE_PERIPH_ADC_BURST */

#ifdef MODULE_PERIPH_ADC_BURST
void adc_sample_burst_end(adc_t line)
{
    dma_t dma = adc_config[line].dma;
    dma_wait(dma);
    dma_stop(dma);
    dma_release(dma);

    /* disable ADC to DMA */
    dev(line)->CR2 &= ~ADC_CR2_DMA;

    /* disable external triggering of ADC */
    dev(line)->CR2 &= ~ADC_CR2_EXTEN;

    /* check if this is the VBAT line */
    if (IS_USED(MODULE_PERIPH_VBAT) && line == VBAT_ADC) {
        vbat_disable();
    }

    /* power off and unlock device again */
    done(line);
}
#endif /* MODULE_PERIPH_ADC_BURST */
