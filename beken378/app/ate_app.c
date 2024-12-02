#include "include.h"
#include "ate_app.h"

#if ATE_APP_FUN
#if (!CFG_SUPPORT_ALIOS) 
#include "app.h"
#include "power_save_pub.h"
#else
#include "sys_ctrl_pub.h"
#endif

#include "uart_pub.h"
#include "wlan_cli_pub.h"
#include "mem_pub.h"
#include "str_pub.h"

char ate_mode_state = 0;
int ate_gpio_port = GPIO0;

#define CMD_SINGLE_WAVE  "txevm -b 0 -r 24 -c 1 -w 1"

#if (CFG_SUPPORT_ALIOS)
extern void power_save_rf_hold_bit_set(UINT32);
#endif

void ate_gpio_init(void)
{
    uint32_t param;

    if (UART1_PORT == uart_print_port)
    {
        ate_gpio_port = GPIO11;
    }

    param = GPIO_CFG_PARAM(ate_gpio_port, GMODE_INPUT_PULLUP);
    gpio_ctrl( CMD_GPIO_CFG, &param);
}

uint32_t ate_mode_check(void)
{
    uint32_t ret;
    uint32_t param;

    param = ate_gpio_port;
    ret = gpio_ctrl( CMD_GPIO_INPUT, &param);

    return (ATE_ENABLE_GIPO_LEVEL == ret);
}

void ate_app_init(void)
{
    uint32_t mode = 0;
    ate_gpio_init();

    mode = ate_mode_check();
    if(mode)
    {
        ate_mode_state = (char)1;
    }
    else if (RESET_SOURCE_FORCE_ATE == bk_misc_get_start_type())
    {
        ate_mode_state = (char)1;
    }
    else
    {
        ate_mode_state = (char)0;
    }
}

uint32_t get_ate_mode_state(void)
{
    if(ate_mode_state != (char)0)
        return 1;
    return 0;
}

#ifdef SINGLE_WAVE_TEST
static void do_single_wave_test(void)
{
    uint32_t cmd_len = os_strlen(CMD_SINGLE_WAVE) + 1;
    uint8 *cmd_buf = os_malloc(cmd_len);
    if (cmd_buf) {
        extern void bk_test_cmd_handle_input(char *inbuf, int len);

        os_memcpy(cmd_buf, CMD_SINGLE_WAVE, cmd_len);
        bk_test_cmd_handle_input((char *)cmd_buf, cmd_len);
        os_free(cmd_buf);
    }
}
#endif

void ate_start(void)
{
    app_pre_start();

#if (CFG_OS_FREERTOS) || (CFG_SUPPORT_LITEOS)
    cli_init();
#endif

    power_save_rf_hold_bit_set(RF_HOLD_RF_SLEEP_BIT);

    ATE_PRT("ate_start\r\n");
}
#else
uint32_t get_ate_mode_state(void)
{
	return 0;
}
#endif /*ATE_APP_FUN */
// eof
