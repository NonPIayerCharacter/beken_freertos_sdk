
:link_to_translation:`zh_CN:[中文]`

PWM user guide
================

Overview
-----------------
The BK7238 has six 32-bit PWM channels, labeled PWM0-5 (timer mode suppported). Each PWM channel has three modes: timer mode, PWM mode, and capture mode. Each mode of every channel is multiplexed with a 32-bit counter.



The main features of the PWM module are as follows:


-	The counter increases in one direction and automatically continues counting from 0 when it overflows to the maximum value.

-	Each channel can be enabled individually, and the mode of each channel can be configured individually.

-	Capable of continuously counting between two rising edges, two falling edges, or any two edges in capture mode.

-	Configurable PWM period and duty cycle for each PWM channel

-	The real-time count value can be read in timer mode.


PWM mode
-----------------

::

	//Enable PWM
	channel=1; 	 	 //pwm1
	frequency=(260000/1000)*2000;  //2 kHz   

	duty_cycle1= frequency/4;  //High level duty cycle 1/4
	duty_cycle12= 0;
	duty_cycle13= 0;
	bk_pwm_initialize(channel, (260000/1000)*frequency, duty_cycle1,duty_cycle12,duty_cycle13);
	bk_pwm_start(channel1);		

	//Update PWM parameters
	frequency=(260000/1000)*1000;  //1 kHz   
	duty_cycle1= frequency/2;  //High level duty cycle 1/2
	bk_pwm_update_param(channel, frequency, duty_cycle1);	


Capture mode
-----------------

::

	#define PWM_CAP_POS_MODE            (0x04)  // Capture (pos -> pos)
	#define PWM_CAP_NEG_MODE            (0x05)  // Capture (neg -> neg)
	#define PWM_CAP_EDGE_MODE           (0x06)	// Capture (edge -> edge)
	//Enable PWM capture
	channel=1; 
	cap_mode=PWM_CAP_POS_MODE;

	bk_pwm_capture_initialize(channel, cap_mode);	
	bk_pwm_start(channel);	

	//Read the captured value
	cap_value = bk_pwm_get_capvalue(channel1);
	bk_printf("pwm : %d cap_value=%x \r\n", channel1, cap_value);


