#ifndef _SARADC_PUB_H_
#define _SARADC_PUB_H_

/**
* @brief  Translate adc value to Voltage value.
* 
* User example:
* @code
*        INT32 channel = atoi(argv[2]);  // adc chan from 1-6
*        UINT32 sum = 0;
*        UINT32 index;
*        typedef UINT16 heap_t;
*        size_t MinHeapInsert(heap_t *heap, size_t heap_size, heap_t x);
*        heap_t MinHeapReplace(heap_t *heap, size_t heap_size, heap_t x);
*
*        p_ADC_drv_desc = (saradc_desc_t *)os_malloc(sizeof(saradc_desc_t));
*        if (p_ADC_drv_desc == NULL)
*        {
*            os_printf("malloc1 failed!\r\n");
*            return;
*        }
*
*        os_memset(p_ADC_drv_desc, 0x00, sizeof(saradc_desc_t));
*        p_ADC_drv_desc->channel = channel;
*        p_ADC_drv_desc->data_buff_size = 100;
*        p_ADC_drv_desc->mode = (ADC_CONFIG_MODE_CONTINUE << 0)
*                                | (ADC_CONFIG_MODE_4CLK_DELAY << 2);
*        p_ADC_drv_desc->has_data                = 0;
*        p_ADC_drv_desc->current_read_data_cnt   = 0;
*        p_ADC_drv_desc->current_sample_data_cnt = 0;
*        p_ADC_drv_desc->pre_div = 0x10;
*        p_ADC_drv_desc->samp_rate = 0x20;
*        p_ADC_drv_desc->pData = (UINT16 *)os_malloc(p_ADC_drv_desc->data_buff_size * sizeof(UINT16));
*        os_memset(p_ADC_drv_desc->pData, 0x00, p_ADC_drv_desc->data_buff_size * sizeof(UINT16));
*
*        if(p_ADC_drv_desc->pData == NULL)
*        {
*            os_printf("malloc1 failed!\r\n");
*            os_free(p_ADC_drv_desc);
*
*            return;
*        }
*
*        saradc_handle = ddev_open(SARADC_DEV_NAME, &status, (UINT32)p_ADC_drv_desc);
*        while (1)
*        {
*            if (p_ADC_drv_desc->current_sample_data_cnt == p_ADC_drv_desc->data_buff_size)
*            {
*                if ((argc > 3) && atoi(argv[3]))
*                {
*                }
*                else
*                {
*                ddev_close(saradc_handle);
*                }
*                break;
*            }
*        }
*
*        heap_t heap[(100 + 1) / 2];
*        int count = 0;
*
*        for (index = 0; index < sizeof(heap) / sizeof(heap[0]); index++) {
*            MinHeapInsert(heap, index, (heap_t)p_ADC_drv_desc->pData[index]);
*
*        }
*
*        for (index = sizeof(heap) / sizeof(heap[0]); index < 100; index++) {
*            if (heap[0] < (heap_t)p_ADC_drv_desc->pData[index])
*            {
*                MinHeapReplace(heap, sizeof(heap) / sizeof(heap[0]), (heap_t)p_ADC_drv_desc->pData[index]);
*            }
*        }
*
*        for (index = 0; index < 100; index++)
*        {
*            //error [-0.5%, 0.5%] ==> [-5, 5]
*            if ((p_ADC_drv_desc->pData[index] > heap[0] + 0x10) || (heap[0] > p_ADC_drv_desc->pData[index] + 0x10))
*            {
*                continue;
*            }
*
*            count++;
*            sum += p_ADC_drv_desc->pData[index];
*        }
*
*        p_ADC_drv_desc->pData[0] = (UINT16)(sum / count);
*        float voltage = saradc_calculate(p_ADC_drv_desc->pData[0]);
*
*        os_printf("voltage is [%f] orignal %d count=%d\r\n", voltage, p_ADC_drv_desc->pData[0], count);
*
*        os_free(p_ADC_drv_desc->pData);
*        os_free(p_ADC_drv_desc);
* @endcode
* @param
*     - adc_val: it better adc val that after do average
*
* @return
*     - Voltage value with float
* 
*/
float saradc_calculate(UINT16 adc_val);

#endif //_SARADC_PUB_H_
