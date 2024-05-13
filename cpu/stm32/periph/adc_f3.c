/*
 * Copyright (C) 2020 LAAS-CNRS
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32
 * @ingroup     drivers_periph_adc
 * @{
 *
 * @file
 * @brief       Low-level ADC driver implementation
 *
 * @author      Hugues Larrive <hugues.larrive@pm.me>
 *
 * @}
 */

#include "cpu.h"
#include "mutex.h"
#include "periph/adc.h"
#include "periph_conf.h"
#include "periph/cpu_dma.h"
#include "ztimer.h"
#include "periph/vbat.h"

#if defined(MODULE_PERIPH_ADC_BURST) || defined(MODULE_PERIPH_ADC_CONT)
#define ADC_DMA
#endif

//#define SMP_MIN         ADC_SMPR1_SMP0 // this is actually the max
#define SMP_MIN         (0x2) /*< Sampling time for slow channels
                                  (0x2 = 4.5 ADC clock cycles) */
#ifdef ADC1_COMMON
#define ADC_INSTANCE    ADC1_COMMON
#else
#define ADC_INSTANCE    ADC12_COMMON
#endif

/**
 * @brief   Default VBAT undefined value
 */
#ifndef VBAT_ADC
#define VBAT_ADC    ADC_UNDEF
#endif

/**
 * @brief   Allocate locks for all available ADC devices
 */
static mutex_t locks[ADC_DEVS];

static inline ADC_TypeDef *dev(adc_t line)
{
    switch (adc_config[line].dev) {
#ifdef ADC1_BASE
        case 0:
            return (ADC_TypeDef *)(ADC1_BASE);
            break;
#endif
#ifdef ADC2_BASE
        case 1:
            return (ADC_TypeDef *)(ADC2_BASE);
            break;
#endif
#ifdef ADC3_BASE
        case 2:
            return (ADC_TypeDef *)(ADC3_BASE);
            break;
#endif
#ifdef ADC4_BASE
        case 3:
            return (ADC_TypeDef *)(ADC4_BASE);
            break;
#endif
    }

    // should never reach here
    assert(false);
    return NULL;
}

static inline void prep(adc_t line)
{
    mutex_lock(&locks[adc_config[line].dev]);
/* Enable the clock here only if it will be disabled by done, else just
 * enable it once in adc_init() */
#if defined(RCC_AHBENR_ADC1EN)
    periph_clk_en(AHB, RCC_AHBENR_ADC1EN);
#endif
}

static inline void done(adc_t line)
{
/* On some STM32F3 ADC are grouped by paire (ADC12EN or ADC34EN) so
 * don't disable the clock as the other device may still use it. */
#if defined(RCC_AHBENR_ADC1EN)
    periph_clk_dis(AHB, RCC_AHBENR_ADC1EN);
#endif
    mutex_unlock(&locks[adc_config[line].dev]);
}

int adc_init(adc_t line)
{
    /* Check if the line is valid */
    if (line >= ADC_NUMOF) {
        return -1;
    }

    RCC->D3CCIPR |= RCC_D3CCIPR_ADCSEL_1;

    /* Lock device and enable its peripheral clock */
    prep(line);
/* On some STM32F3 ADC are grouped by paire (ADC12EN or ADC34EN) so
 * enable the clock only once here. */
//#if defined(RCC_AHBENR_ADC12EN)
    if (adc_config[line].dev <= 1) {
        periph_clk_en(AHB1, RCC_AHB1ENR_ADC12EN);
    }
//#endif
//#if defined(RCC_AHBENR_ADC34EN)
    if (adc_config[line].dev >= 2) {
        periph_clk_en(AHB4, RCC_AHB4ENR_ADC3EN);
    }
//#endif

#if 0
    /* Setting ADC clock to HCLK/1 is only allowed if AHB clock
     * prescaler is 1 */
    if (!(RCC->CFGR & RCC_D1CFGR_HPRE_3)) {
        /* set ADC clock to HCLK/1 */
        if (adc_config[line].dev <= 1) {
            ADC_INSTANCE->CCR |= ADC_CCR_CKMODE_0;
        }
//#ifdef ADC34_COMMON
        if (adc_config[line].dev >= 2) {
            ADC3_COMMON->CCR |= ADC_CCR_CKMODE_0;
        }
//#endif
    }
    else {
        /* set ADC clock to HCLK/2 otherwise */
        if (adc_config[line].dev <= 1) {
            ADC_INSTANCE->CCR |= ADC_CCR_CKMODE_1;
        }
//#ifdef ADC34_COMMON
        if (adc_config[line].dev >= 2) {
            ADC3_COMMON->CCR |= ADC_CCR_CKMODE_1;
        }
//#endif
    }
#else
ADC_INSTANCE->CCR = ADC_CCR_CKMODE_0;
ADC3_COMMON->CCR = ADC_CCR_CKMODE_0;
#endif

    /* Configure the pin */
    if (adc_config[line].pin != GPIO_UNDEF) {
        gpio_init_analog(adc_config[line].pin);
    }

    /* Init ADC line only if it wasn't already initialized */
    if (!(dev(line)->CR & ADC_CR_ADEN)) {
        // take ADC out of deep sleep
        dev(line)->CR &= ~(ADC_CR_DEEPPWD);
        /* Enable ADC internal voltage regulator and wait for startup period */
        dev(line)->CR |= ADC_CR_ADVREGEN;
#if IS_USED(MODULE_ZTIMER_USEC) && 0
        ztimer_sleep(ZTIMER_USEC, ADC_T_ADCVREG_STUP_US);
#else
        /* to avoid using ZTIMER_USEC unless already included round up the
           internal voltage regulator start up to 1ms */
        ztimer_sleep(ZTIMER_MSEC, 1);
#endif

        if (false) {
            /* Configure calibration for differential inputs */
            dev(line)->CR |= ADC_CR_ADCALDIF;
        }
        else {
            /* Configure calibration for single ended inputs */
            dev(line)->CR &= ~ADC_CR_ADCALDIF;
        }

        // enable linearity cal, and turn on boost supply
        dev(line)->CR |= ADC_CR_ADCALLIN | ADC_CR_BOOST;

        /* Start automatic calibration and wait for it to complete */
        dev(line)->CR |= ADC_CR_ADCAL;
        while (dev(line)->CR & ADC_CR_ADCAL) {}

        /* Clear ADRDY by writing it */
        dev(line)->ISR |= ADC_ISR_ADRDY;

        /* Enable ADC and wait for it to be ready */
        dev(line)->CR |= ADC_CR_ADEN;
        while ((dev(line)->ISR & ADC_ISR_ADRDY) == 0) {}

        /* Set sequence length to 1 conversion */
        dev(line)->SQR1 |= (0 & ADC_SQR1_L);
    }

    // Enable the ADC channel's analog switch
    #if defined(ADC_VER_V5_V90)
    if (adc_config[line].dev < 2)
    {
        dev(line)->PCSEL_RES0 |= ADC_PCSEL_PCSEL_0 << adc_config[line].chan;
        //dev(line)->PCSEL_RES0 |= ADC_PCSEL_PCSEL; //TODO: rm
    }
    #else
    dev(line)->PCSEL |= ADC_PCSEL_PCSEL_0 << adc_config[line].chan;
    #endif

    /* Configure sampling time for the given channel */
    if (adc_config[line].chan < 10) {
        dev(line)->SMPR1 =  (SMP_MIN << (adc_config[line].chan * 3));
    }
    else {
        dev(line)->SMPR2 =  (SMP_MIN << ((adc_config[line].chan - 10) * 3));
    }

    /* Power off and unlock device again */
    done(line);

    return 0;
}

int32_t adc_sample(adc_t line, adc_res_t res)
{
    int sample;

#if 0
    /* Check if resolution is applicable */
    if (res & 0x3) {
        return -1;
    }
#endif

    /* Lock and power on the ADC device */
    prep(line);

    /* check if this is the VBAT line */
    if (IS_USED(MODULE_PERIPH_VBAT) && line == VBAT_ADC) {
        vbat_enable();
    }

    /* Set resolution */
    if (adc_config[line].dev < 2)
    {
        dev(line)->CFGR &= ~ADC_CFGR_RES;
        dev(line)->CFGR |= res;
    }
    else
    {
        dev(line)->CFGR &= ~ADC3_CFGR_RES;
        switch (res)
        {
            case ADC_RES_12BIT:
            break;

            case ADC_RES_10BIT:
            dev(line)->CFGR |= 0x01 << ADC3_CFGR_RES_Pos;
            break;

            case ADC_RES_8BIT:
            dev(line)->CFGR |= 0x02 << ADC3_CFGR_RES_Pos;
            break;

            case ADC_RES_6BIT:
            dev(line)->CFGR |= 0x03 << ADC3_CFGR_RES_Pos;
            break;
        }
    }

    /* Specify channel for regular conversion */
    dev(line)->SQR1 = adc_config[line].chan << ADC_SQR1_SQ1_Pos;

    /* Start conversion and wait for it to complete */
    dev(line)->CR |= ADC_CR_ADSTART;
    while (!(dev(line)->ISR & ADC_ISR_EOC)) {}
    //while (!(dev(line)->CR & ADC_CR_ADSTART)) {}

    /* Read the sample */
    sample = (int)dev(line)->DR;

    /* check if this is the VBAT line */
    if (IS_USED(MODULE_PERIPH_VBAT) && line == VBAT_ADC) {
        vbat_disable();
    }

    /* Power off and unlock device again */
    done(line);

    return sample;
}

#ifdef ADC_DMA
static int _sample_dma(adc_t line, adc_res_t res, void* buf, size_t count,
    int adc_trg, bool circ)
{
    int result;
    dma_t dma = adc_config[line].dma;
    int dma_chan = adc_config[line].dma_chan;
    volatile void *periph_addr = &(dev(line)->DR);
    int dma_size = DMA_DATA_WIDTH_BYTE;

#if  0
    /* check if resolution is applicable */
    if (res & 0x3) {
        return -1;
    }
#endif

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

    /* Set resolution */
    if (adc_config[line].dev < 2)
    {
        dev(line)->CFGR &= ~ADC_CFGR_RES;
        dev(line)->CFGR |= res;
    }
    else
    {
        dev(line)->CFGR &= ~ADC3_CFGR_RES;
        switch (res)
        {
            case ADC_RES_12BIT:
            break;

            case ADC_RES_10BIT:
            dev(line)->CFGR |= 0x01 << ADC3_CFGR_RES_Pos;
            break;

            case ADC_RES_8BIT:
            dev(line)->CFGR |= 0x02 << ADC3_CFGR_RES_Pos;
            break;

            case ADC_RES_6BIT:
            dev(line)->CFGR |= 0x03 << ADC3_CFGR_RES_Pos;
            break;
        }
    }

    /* Specify channel for regular conversion */
    dev(line)->SQR1 = adc_config[line].chan << ADC_SQR1_SQ1_Pos;

    dma_acquire(dma);

    /* get DMA ready to receive data from ADC */
    result = dma_configure(
        dma, dma_chan, periph_addr, buf, count,
        DMA_PERIPH_TO_MEM,
        dma_size | DMA_INC_DST_ADDR |
        ((circ)? DMA_CIRCULAR : 0) |
        ((circ)? DMA_WITH_WAIT_HALF : 0)
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
    if (circ)
    {
        dev(line)->CFGR |= ADC_CFGR_DMNGT;
    }
    else
    {
        dev(line)->CFGR |= ADC_CFGR_DMNGT_0;
    }

    /* Set sample trigger source */
    dev(line)->CFGR &= ~ADC_CFGR_EXTSEL;
    dev(line)->CFGR |= adc_trg << ADC_CFGR_EXTSEL_Pos;

    /* Enable external trigger, and trigger on its rising edge */
    dev(line)->CFGR &= ~ADC_CFGR_EXTEN;
    dev(line)->CFGR |= 1 << ADC_CFGR_EXTEN_Pos;

    /* Enable conversions */
    dev(line)->CR |= ADC_CR_ADSTART;

    return 0;
}
#endif /* ADC_DMA */

#ifdef ADC_DMA
static void _sample_dma_end(adc_t line)
{
    dma_t dma = adc_config[line].dma;

    /* Disable external triggering of ADC */
    dev(line)->CFGR &= ~ADC_CFGR_EXTEN;

    /* disable ADC to DMA */
    dev(line)->CFGR &= ~ADC_CFGR_DMNGT;

    dma_stop(dma);
    dma_release(dma);

    /* check if this is the VBAT line */
    if (IS_USED(MODULE_PERIPH_VBAT) && line == VBAT_ADC) {
        vbat_disable();
    }

    /* power off and unlock device again */
    done(line);
}
#endif /* ADC_DMA */

#ifdef MODULE_PERIPH_ADC_BURST
int adc_sample_burst(adc_t line, adc_res_t res, void* buf, size_t count,
    int adc_trg)
{
    int result;
    dma_t dma = adc_config[line].dma;
    int dma_chan = adc_config[line].dma_chan;
    volatile void *periph_addr = &(dev(line)->DR);
    int dma_size = DMA_DATA_WIDTH_BYTE;

#if 0
    /* Check if resolution is applicable */
    if (res & 0x3) {
        return -1;
    }
#endif

    /* Calculate the size of each DMA transfer */
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

    /* Lock and power on the ADC device */
    prep(line);

    /* check if this is the VBAT line */
    if (IS_USED(MODULE_PERIPH_VBAT) && line == VBAT_ADC) {
        vbat_enable();
    }

    /* Set resolution */
    dev(line)->CFGR &= ~ADC_CFGR_RES;
    dev(line)->CFGR |= res;

    /* Specify channel for regular conversion */
    dev(line)->SQR1 = adc_config[line].chan << ADC_SQR1_SQ1_Pos;

    dma_acquire(dma);

    /* Get DMA ready to receive data from ADC */
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

    /* Enable ADC to DMA */
    dev(line)->CFGR |= ADC_CFGR_DMAEN;
    //dev(line)->CFGR |= ADC_CFGR_DMACFG; // DMA circular mode

    /* Set sample trigger source */
    dev(line)->CFGR &= ~ADC_CFGR_EXTSEL;
    dev(line)->CFGR |= adc_trg << ADC_CFGR_EXTSEL_Pos;

    /* Enable external trigger, and trigger on its rising edge */
    dev(line)->CFGR &= ~ADC_CFGR_EXTEN;
    dev(line)->CFGR |= 1 << ADC_CFGR_EXTEN_Pos;

    /* Enable conversions */
    dev(line)->CR |= ADC_CR_ADSTART;

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

    /* Disable external triggering of ADC */
    dev(line)->CFGR &= ~ADC_CFGR_EXTEN;

    /* Disable ADC to DMA */
    dev(line)->CFGR &= ~ADC_CFGR_DMAEN;

    /* check if this is the VBAT line */
    if (IS_USED(MODULE_PERIPH_VBAT) && line == VBAT_ADC) {
        vbat_disable();
    }

    /* Power off and unlock device again */
    done(line);
}
#endif /* MODULE_PERIPH_ADC_BURST */

#ifdef MODULE_PERIPH_ADC_CONT
int adc_sample_cont(adc_t line, adc_res_t res, void* buf, size_t count,
    int adc_trg)
{
    return _sample_dma(line, res, buf, count, adc_trg, true);
}
#endif /* MODULE_PERIPH_ADC_CONT */

#ifdef MODULE_PERIPH_ADC_CONT
void adc_sample_cont_end(adc_t line)
{
    _sample_dma_end(line);
}
#endif /* MODULE_PERIPH_ADC_CONT */

#ifdef MODULE_PERIPH_ADC_CONT
void adc_sample_cont_wait(adc_t line)
{
    dma_t dma = adc_config[line].dma;

    dma_wait(dma);
}
#endif /* MODULE_PERIPH_ADC_CONT */

#ifdef MODULE_PERIPH_ADC_CONT
void adc_sample_cont_wait_half(adc_t line)
{
    dma_t dma = adc_config[line].dma;

    dma_wait_half(dma);
}
#endif /* MODULE_PERIPH_ADC_CONT */
