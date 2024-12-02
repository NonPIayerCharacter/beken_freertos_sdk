PWM 使用指南
============

概述
-----------------
BK7238有六个32位PWM通道，标记为PWM0~5（支持定时器模式）。每个PWM通道有三个模式：定时器模式、PWM模式和捕获模式。
每个通道的每个模式都用32位计数进行多路复用。


PWM模块的主要功能如下所示：


-	计数器在一个方向上增加，并在溢出到最大值时自动从0开始继续计数

-	每个通道可以单独启用，每个通道的模式可以单独配置。

-	在捕获模式下，能够在两个上升沿、两个下降沿或双沿之间连续计数

-	每个PWM通道的可配置PWM周期和占空比

-	可以在定时器模式下读取实时计数值。


PWM模式
-----------------

::

	//开启 PWM
	channel=1; 	 	 //pwm1
	frequency=(260000/1000)*2000;  //2 KHz   

	duty_cycle1= frequency/4;  //高电平占空比 1/4
	duty_cycle12= 0;
	duty_cycle13= 0;
	bk_pwm_initialize(channel, (260000/1000)*frequency, duty_cycle1,duty_cycle12,duty_cycle13);
	bk_pwm_start(channel1);		

	//更新pwm 参数
	frequency=(260000/1000)*1000;  //1 KHz   
	duty_cycle1= frequency/2;  //高电平占空比 1/2
	bk_pwm_update_param(channel, frequency, duty_cycle1);	


捕获模式
-----------------

::

	#define PWM_CAP_POS_MODE            (0x04)  // Capture (pos -> pos)
	#define PWM_CAP_NEG_MODE            (0x05)  // Capture (neg -> neg)
	#define PWM_CAP_EDGE_MODE           (0x06)	// Capture (edge -> edge)
	//开启 PWM 捕获
	channel=1; 
	cap_mode=PWM_CAP_POS_MODE;

	bk_pwm_capture_initialize(channel, cap_mode);	
	bk_pwm_start(channel);	

	//读取捕获值
	cap_value = bk_pwm_get_capvalue(channel1);
	bk_printf("pwm : %d cap_value=%x \r\n", channel1, cap_value);


