#ifndef _GPIO_PUB_H_
#define _GPIO_PUB_H_



typedef enum {
    GMODE_INPUT_PULLDOWN = 0,
    GMODE_OUTPUT,
    GMODE_SECOND_FUNC,
    GMODE_INPUT_PULLUP,
    GMODE_INPUT,
    GMODE_SECOND_FUNC_PULL_UP,//Special for uart1
    GMODE_OUTPUT_PULLUP,
    GMODE_SET_HIGH_IMPENDANCE,
    GMODE_DEEP_PS,
    GMODE_HIGH_Z
}GPIO_MODE;


typedef enum 
{
    GPIO0 = 0,
    GPIO1,
    GPIO6 = 6,
    GPIO7,
    GPIO8,
    GPIO9,
    GPIO10,
    GPIO11,
    GPIO14 = 14,
    GPIO15,
    GPIO16,
    GPIO17,
    GPIO20 = 20,
    GPIO21,
    GPIO22,
    GPIO23,
    GPIO24,
    GPIO26 = 26,
    GPIO28 = 28,   
    GPIONUM,
} GPIO_INDEX ;

/**
* @brief  set the configuration of GPIO
* 
* @param
*     - index: the gpio index
*     - configuration:  the gpio mode,see enum GPIO_MODE
* 
* User example:
* @code
*  gpio_config(GPIO11,GMODE_OUTPUT);  //set gpio 11 output mdoe
*  gpio_config(GPIO11,GMODE_INPUT);  //set gpio 11 intput mdoe
* @endcode
*
*/
OSStatus BkGpioInitialize( bk_gpio_t gpio, bk_gpio_config_t configuration )

/**
* @brief  read the voltage of gpio which in intput mdoe 
* 
* 
* @attention 
*  1. you must use it after  gpio_config(), otherwise you will get err 
* 
* @param
*     - id: the gpio index
* 
* User example:
* @code
*  gpio_config(GPIO11,GMODE_OUTPUT);  //set gpio 11 output mdoe
*  vol=gpio_input(GPIO11);  //get gpio 11 intput mdoe voltage
* @endcode
*
* @return
*    - voltage 0 or 1
*/
UINT32 gpio_input(UINT32 id);

/**
* @brief  Output gpio voltage
* 
* @attention 
*  1. you must use it after  gpio_config(), otherwise you will set err 
* 
* @param
*     - id: the gpio index
*     - val:  voltage 0 or 1
* 
* User example:
* @code
*  gpio_config(GPIO11,GMODE_OUTPUT);  //set gpio 11 output mdoe
*  gpio_output(GPIO11,1);  //set gpio 11 output higt voltage
* @endcode
*
* 
** @return
*    - void
*/
void gpio_output(UINT32 id, UINT32 val);


void gpio_int_enable(UINT32 index, UINT32 mode, void (*p_Int_Handler)(unsigned char));
void gpio_int_disable(UINT32 index);


void gpio_int_mask(UINT32 id, UINT32 mask);
#endif // _GPIO_PUB_H_

// EOF

