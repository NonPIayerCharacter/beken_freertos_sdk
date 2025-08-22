// Copyright 2015-2024 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "include.h"
#include "rtos_pub.h"
#include "BkDriverGpio.h"
#include "drv_model_pub.h"
#include "error.h"
#include "gpio_pub.h"

OSStatus BkGpioAssert( bk_gpio_t gpio )
{
    return 0;
}

OSStatus BkGpioInitialize( bk_gpio_t gpio, bk_gpio_config_t configuration )
{
    UINT32 ret, flag;
    UINT32 mode, param;

    BkGpioAssert(gpio);

    flag = 1;
    switch(configuration)
    {
    case INPUT_PULL_UP:
        mode = GMODE_INPUT_PULLUP;
        break;
    case INPUT_PULL_DOWN:
        mode = GMODE_INPUT_PULLDOWN;
        break;
    case INPUT_NORMAL:
        mode = GMODE_INPUT;
        break;
    case OUTPUT_NORMAL:
        mode = GMODE_OUTPUT;
        break;
    case GPIO_SECOND_FNNC:
        mode = GMODE_SECOND_FUNC;
        break;
    case GPIO_HIGH_IMPENDANCE:
        mode = GMODE_SET_HIGH_IMPENDANCE;
        break;
    default:
        flag = 0;
        break;
    }

    if(flag)
    {
        param = GPIO_CFG_PARAM(gpio, mode);
        ret = sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
        ASSERT(GPIO_SUCCESS == ret);
    }
    return 0;
}

OSStatus BkGpioFinalize( bk_gpio_t gpio )
{
    BkGpioAssert(gpio);
    BkGpioInitialize(gpio, INPUT_NORMAL);
    return 0;
}

OSStatus BkGpioOutputHigh( bk_gpio_t gpio )
{
    BkGpioAssert(gpio);
    bk_gpio_output(gpio, 1);
    return 0;
}

OSStatus BkGpioOutputLow( bk_gpio_t gpio )
{
    BkGpioAssert(gpio);
    bk_gpio_output(gpio, 0);
    return 0;
}

OSStatus BkGpioOutputTrigger( bk_gpio_t gpio )
{
    BkGpioAssert(gpio);
    return 0;
}

bool BkGpioInputGet( bk_gpio_t gpio )
{
    return bk_gpio_input(gpio);
}

OSStatus BkGpioEnableIRQ( bk_gpio_t gpio, bk_gpio_irq_trigger_t trigger, bk_gpio_irq_handler_t handler, void *arg )
{
    BkGpioAssert(gpio);
    GPIO_INT_ST int_struc_param;
    int_struc_param.id = gpio;
    int_struc_param.mode = trigger;
    int_struc_param.phandler = handler;
    return sddev_control(GPIO_DEV_NAME, CMD_GPIO_INT_ENABLE, &int_struc_param);
}

OSStatus BkGpioDisableIRQ( bk_gpio_t gpio )
{
    UINT32 param = gpio;

    BkGpioAssert(gpio);
    return sddev_control(GPIO_DEV_NAME, CMD_GPIO_INT_DISABLE, &param);
}

OSStatus BKGpioOp(char cmd, uint32_t id, char mode)
{
    uint32_t command, mode_set;

    if(cmd == '0')
        command = CMD_GPIO_CFG;
    else if(cmd == '1')
        command = CMD_GPIO_INPUT;
    else if(cmd == '2')
        command = CMD_GPIO_OUTPUT;
    else
        command = CMD_GPIO_OUTPUT_REVERSE;

    if(mode == '0')
        mode_set = GMODE_INPUT_PULLDOWN;
    else if(mode == '1')
        mode_set = GMODE_OUTPUT;
    else if(mode == '2')
        mode_set = GMODE_INPUT_PULLUP;
    else
        mode_set = GMODE_INPUT;
    mode_set = (mode_set << 8) | id;

    return sddev_control(GPIO_DEV_NAME, command, &mode_set);
}

void BKGpioIntcEn(uint8_t cmd, uint8_t id, uint32_t mode, void(*p_handle)(char))
{
    if(cmd == 1)
        BkGpioEnableIRQ((bk_gpio_t)id, (bk_gpio_irq_trigger_t)mode, (bk_gpio_irq_handler_t)p_handle, NULL);
    else
        BkGpioDisableIRQ(id);
}
// eof

