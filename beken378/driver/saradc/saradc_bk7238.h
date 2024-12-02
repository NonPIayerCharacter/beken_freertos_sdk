#ifndef _SARADC_BK7238_H_
#define _SARADC_BK7238_H_

#if (CFG_SOC_NAME == SOC_BK7238)
#define SARADC_BASE                     (0x00802C00)
#define SARADC_ADC_CONFIG0              (SARADC_BASE + 0 * 4)
#define SARADC_ADC_START_SADC           (0x01UL << 0) //start
#define SARADC_ADC_FORCE_QUIT           (0x01UL << 1) //stop
#define SARADC_ADC_PWR_XUS_POSI         (2)
#define SARADC_ADC_PWR_XUS_MASK         (0x07FF)      //delay before sampling
#define SARADC_ADC_XTAL_FREQ_POSI       (13)
#define SARADC_ADC_XTAL_FREQ_MASK       (0x7F)
#define SARADC_ADC_NT_NUM_POSI          (23)
#define SARADC_ADC_NT_NUM_MIN           (0x4)
#define SARADC_ADC_NT_NUM_MASK          (0x7F)
#define SARADC_ADC_CLK                  (0x01UL << 30)
#define SARADC_ADC_PWD                  (0x01UL << 31)

#define SARADC_ADC_CONFIG1              (SARADC_BASE + 1 * 4)
#define SARADC_ADC_PRE_DIV_POSI         (0)
#define SARADC_ADC_PRE_DIV_MASK         (0xFF)
#define SARADC_ADC_SAMPLE_RATE_POSI     (8)
#define SARADC_ADC_SAMPLE_RATE_MASK     (0xFF)
#define SARADC_ADC_TARGET_NOPWD         (0x01UL << 19)
#define SARADC_ADC_TARGET_NUM_POSI      (20)
#define SARADC_ADC_TARGET_NUM_MASK      (0xFFF)

#define SARADC_ADC_CONFIG2              (SARADC_BASE + 2 * 4)
#define SARADC_ADC_CHNL_POSI            (0)
#define SARADC_ADC_CHNL_MAX             (12)
#define SARADC_ADC_CHNL_MASK            (0xF)
#define SARADC_ADC_SUM_FILTER_POSI      (8)
#define SARADC_ADC_SUM_FILTER_MASK      (0xFF)
#define SARADC_ADC_SAT_CTRL_POSI        (16)
#define SARADC_ADC_SAT_CTRL_MASK        (0x7)
#define SARADC_ADC_SAT_ENABLE           (0x01UL << 19)
#define SARADC_ADC_DROP_NUM_POSI        (28)
#define SARADC_ADC_DROP_NUM_MASK        (0xF)

#define SARADC_ADC_STATE                (SARADC_BASE + 3 * 4)
#define SARADC_ADC_FIFO_LEVEL_POSI      (0)
#define SARADC_ADC_FIFO_LEVEL_MASK      (0x7)
#define SARADC_ADC_FIFO_INT_ENABLE      (0x01UL << 3)
#define SARADC_ADC_FIFO_INT_STATE       (0x01UL << 4)
#define SARADC_ADC_CONV_INT_ENABLE      (0x01UL << 14)
#define SARADC_ADC_CONV_DONE_STATE      (0x01UL << 15)
#define SARADC_ADC_TIMEOUT_INT_ENABLE   (0x01UL << 16)
#define SARADC_ADC_TIMEOUT_INT_STATE    (0x01UL << 17)
#define SARADC_ADC_FIFO_ENABLE          (0x01UL << 29)
#define SARADC_ADC_FIFO_EMPTY           (0x01UL << 30)
#define SARADC_ADC_FIFO_FULL            (0x01UL << 31)
#define SARADC_ADC_INT_CLR              (SARADC_ADC_FIFO_INT_STATE | SARADC_ADC_CONV_DONE_STATE | SARADC_ADC_TIMEOUT_INT_STATE)
#define SARADC_ADC_CHNL_EN              (SARADC_ADC_FIFO_INT_ENABLE | SARADC_ADC_CONV_INT_ENABLE | SARADC_ADC_TIMEOUT_INT_ENABLE)

#define SARADC_ADC_DATA                 (SARADC_BASE + 4 * 4)
#define SARADC_ADC_DATA_POSI            (0)
#define SARADC_ADC_DATA_MASK            (0xFFFF)

#define SARADC_ADC_CONFIG5              (SARADC_BASE + 5 * 4)
#define SARADC_ADC_TIMEOUT_POSI         (0)
#define SARADC_ADC_TIMEOUT_MASK         (0xFFFFFF)

#define SARADC_ADC_CONFIG6              (SARADC_BASE + 6 * 4)
#define SARADC_ADC_VERSION_POSI         (0)
#define SARADC_ADC_VERSION_MASK         (0xFFFF)
#define SARADC_ADC_CHANNEL_WIDTH_POSI   (8)
#define SARADC_ADC_CHANNEL_WIDTH_MASK   (0xFFFF)
#define SARADC_ADC_DATA_WIDTH_POSI      (16)
#define SARADC_ADC_DATA_WIDTH_MASK      (0xF)
#define SARADC_ADC_FIFO_DEPTH_POSI      (20)
#define SARADC_ADC_FIFO_DEPTH_MASK      (0xF)

#endif

#endif //_SARADC_BK7238_H_
