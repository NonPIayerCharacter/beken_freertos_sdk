
:link_to_translation:`zh_CN:[中文]`

SAR ADC
==============


Overview
---------------

The BK7238 has a built-in 10-bit general-purpose SAR ADC with programmable sampling rate from 5 kHz to 26 MHz. The resolution of the ADC can be configured as 12 to 14 bits. The ADC supports up to 6 external channels. It can operate in single-shot mode, software control mode, or continuous mode. The ADC supports the full input range (from 0 V to VBAT) or from 0 to 3.6 V. Generally, the ADC accuracy is not dynamically adjusted in the SAR ADC driver.

SAR ADC operating modes
------------------------

 - Power down mode:

   In sleep mode, the ADC stops any operation, which is equivalent to a power down state.

 - Single-shot mode:

   In single-shot mode, the ADC will only collect one data at a time. After the collection is completed, it will automatically switch to sleep mode, and the corresponding mode status bit will also change to sleep mode. The ADC then wait for the MCU to read the data, and the mode needs to be set to single-shot mode for each sampling.

 - Software control mode

   An ADC interrupt will be generated in software control mode. When in this mode, an interrupt will be generated after the ADC conversion is completed. At this time, it will wait for the MCU to read the data. If the data is not read, no interrupt will be generated. If the MCU reads the data and clears the interrupt, the ADC starts a new conversion and continues to wait for the MCU to read the data.

 - Continuous mode

   Continuous mode is a software control mode without the waiting for the MCU to read data. Regardless of whether the MCU fetches data or not, the ADC always samples and converts data according to a fixed beat without being affected by any signal. Only stopping the ADC can stop the ADC from reading data.

.. note::

  When the ADC is in continuous mode, it will always report interrupts and collect data, which will frequently generate interrupts and affect system performance. Therefore, when the ADC is in continuous mode, it is necessary to stop the ADC every time it collects the desired ADC data length.


SAR ADC data acquisition process
---------------------------------

	The SAR ADC converts data in continuous mode as shown below.

.. figure:: ../../_static/saradc_new.png
    :align: center
    :alt: saradc Overview
    :figclass: align-center

    SARADC Overview


As shown in the figure above:
------------------------------

 - S1, S2...: indicates a sampling and conversion of the voltage on a certain ADC channel. 16 bits of data are output at one sampling time. The sampling voltage range is 0 to 2.4 V and a sampling takes 16 clocks.

 - Sample rate: the reciprocal of the interval between two samplings is the sampling rate, which only takes effect in continuous mode.

 - Sample Cnt: It has two meanings. On the one hand, it represents the size of the ADC hardware buffer in 16 bits. The ADC stores the data obtained from each sampling in the hardware buffer. On the other hand, it represents the interrupt reporting time point, that is, an interrupt is reported every number of sampling times. Sample Cnt can be configured by API bk_adc_set_sample_cnt(), and the default value is 32.

 - ADC hardware buffer: The buffer where the hardware stores sampling data, the size is the same as Sample Cnt.

 - ADC software buffer: The buffer where the ADC driver stores sampling data. Every time an ADC interrupt is generated, the ADC driver will copy the data in the ADC hardware buffer to the ADC software buffer.



Continuous mode ADC test routine
-----------------------------------------------------------------

The routine is located in demos/peripheral/adc/test_adc.c.
::

  saradc_desc_t test_adc_demo_adc1;
  DD_HANDLE test_adc_demo_handle = -1;

  void test_adc_demo_isr_cb(void)
  {
      UINT32 sum = 0;
      float voltage = 0.0;

      if(test_adc_demo_adc1.data_buff_size <= test_adc_demo_adc1.current_sample_data_cnt)
      {
          ddev_close(test_adc_demo_handle);
          saradc_ensure_close();

          for(int i=0;i<test_adc_demo_adc1.data_buff_size;i++)
          {
              sum +=test_adc_demo_adc1.pData[i];
          }
          sum=sum/test_adc_demo_adc1.data_buff_size;

          test_adc_demo_adc1.pData[0] = sum;
          voltage = saradc_calculate(sum);
          os_printf("voltage is [%d] mv\r\n", (UINT32)(voltage * 1000));
          
      }
  }

  VOID test_adc_demo_start(void)
  {
      uint32_t ret;
      UINT32 status;
      GLOBAL_INT_DECLARATION();

      os_memset(&test_adc_demo_adc1, 0x00, sizeof(saradc_desc_t));
      saradc_config_param_init(&test_adc_demo_adc1);

      test_adc_demo_adc1.channel = 1;
      test_adc_demo_adc1.data_buff_size = 20;
      test_adc_demo_adc1.mode = 3;
      test_adc_demo_adc1.current_read_data_cnt = 0;
      test_adc_demo_adc1.current_sample_data_cnt = 0;
      test_adc_demo_adc1.has_data = 0;
      test_adc_demo_adc1.p_Int_Handler = test_adc_demo_isr_cb;
      test_adc_demo_adc1.pData = os_malloc(sizeof(UINT16) * test_adc_demo_adc1.data_buff_size);
      if(!test_adc_demo_adc1.pData)
      {
          os_printf("malloc failed\n");
          return;
      }

      ret = 0;
      do {
          GLOBAL_INT_DISABLE();
          if(saradc_check_busy() == 0) {
              test_adc_demo_handle = ddev_open(SARADC_DEV_NAME, &status, (UINT32)&test_adc_demo_adc1);
              if(DD_HANDLE_UNVALID != test_adc_demo_handle)
              {
                  GLOBAL_INT_RESTORE();
                  break;
              }
          }
      GLOBAL_INT_RESTORE();

      rtos_delay_milliseconds(5);
      ret++;
      } while(ret<5);

      if(ret == 5) {
          os_free(test_adc_demo_adc1.pData);
          os_printf("adc_open failed\n");
          return;
      }
  }



The specific processing flow is as follows:
-----------------------------------------------------------------

 - ADC Start: call ddev_open(SARADC_DEV_NAME, &status, (UINT32)&test_adc_demo_adc1) to enbale ADC and start sampling.

 - ADC hardware sampling: The hardware stores each sampling data in the hardware buffer.

 - ADC interrupt generation: After sampling test_adc_demo_adc1.data_buff_size times, the test_adc_demo_adc1.p data in the ADC software buffer is full, call p_Int_Handler callback to call back the upper layer to obtain the sampling result, and then resample.


