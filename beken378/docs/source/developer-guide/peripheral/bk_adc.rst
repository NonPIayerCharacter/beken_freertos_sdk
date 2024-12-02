SARADC
==============


概述
---------------

  BK7238内置了一个10位通用SAR_ADC，可编程采样时钟范围从5kHz到 26MHz。10位分辨率的ADC可以配置为12~14位。ADC最多支持6个通道。它可以在单次模式或连续模式下运行。ADC支持全幅输入范围（从0V到VBAT）或从0V到3.6V.
  一般精度在 SARADC 驱动中不做动态调整。

SARADC 模式类型：
------------------

 - 休眠模式：

   休眠模式下 ADC 停止任何操作，相当于 power down 的状态

 - 单步模式：

   单步模式下 ADC 每次只会采集一个数据，采集完成之后自动变为休眠模式，相应的模式状态位也会变为休眠模式
   然后等待 MCU 来读取数据，每一次的采样都需要将模式设置为单步模式.

 - 软件控制模式

   软件控制模式下会产生 ADC 中断，当处于此模式时，ADC 转换完成之后会产生一个中断，这个时候会等待 MCU 读取数据，
   如果不读取数据就不会产生中断，如果MCU读取数据之后，清除掉中断后，ADC又开始新一轮的转换，继续等待 MCU 读取数据...

 - 连续模式

   连续模式是在软件控制的模式下，去掉了 MCU 读取数据的等待，不管 MCU 取不取数据，ADC 总是按照固定节拍采样转换数据
   不受任何信号影响，只有停止 ADC 才能停止 ADC 读取数据.

.. note::

  当 ADC 处于连续模式时会一直上报中断和采集数据，这样频繁产生中断，影响系统性能，所以当 ADC 模式处于连续模式的时候需要每采取一次到想要的 ADC 数据长度之后就停止 ADC.

SARADC 获取数据流程：
----------------------

	SARADC 在连续模式下转换数据如下图所示。

.. figure:: ../../_static/saradc_new.png
    :align: center
    :alt: saradc Overview
    :figclass: align-center

    SARADC Overview


如上图所示：
----------------------

 - S1，S2。。。表示对 ADC 某一通道上电压进行一次采样和转换，采样一次输出 16 bits 数据；采样电压范围为 0 到 2.4v; 一次采样时长 16 个 clock.

 - Sample Rate：两次采样间隔的倒数为采样率，仅在连续模式下生效.

 - Sample Cnt：有两层含义，一方面，它表示 ADC 硬件 Buffer 的大小，单位为 16bits, ADC 将每次采样得到的数据存于硬件 Buffer；另一方面，它也表示中断上报时间点，即每经过多少次采样上报一次中断；
   Sample Cnt 可由 API bk_adc_set_sample_cnt() 进行配置，默认值为 32。

 - ADC Hardware Buffer: 即硬件存放采样数据的 Buffer，大小与 Sample Cnt 相同。

 - ADC Software Buffer: ADC 驱动存放采样数据的 Buffer，每次产生 ADC 中断后，ADC 驱动会将 ADC Hardware Buffer 中的数据 copy 到 ADC Software Buffer.



连续模式ADC测试例程
-----------------------------------------------------------------

例程位于 demos/peripheral/adc/test_adc.c
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



具体处理流程如下：
-----------------------------------------------------------------

 - ADC Start: ddev_open(SARADC_DEV_NAME, &status, (UINT32)&test_adc_demo_adc1) 启动 ADC，开始采样。

 - ADC 硬件采样：硬件将每次采样数据存 Hardware Buffer。

 - ADC 中断产生：当采样 test_adc_demo_adc1.data_buff_size 次之后， ADC Software Buffer test_adc_demo_adc1.pData 缓存满了之后，调用 p_Int_Handler callback 回调上层获取采样结果，然后重新进行采样。


