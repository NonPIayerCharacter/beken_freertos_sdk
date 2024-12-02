/**
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 BEKEN Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of BEKEN Corporation.
 *
 */
#include "sys_rtos.h"
#include "rtos_pub.h"
#include "error.h"
#include "wlan_cli_pub.h"
#include "stdarg.h"
#include "include.h"
#include "mem_pub.h"
#include "str_pub.h"
#include "uart_pub.h"
#include "BkDriverFlash.h"
#include "wlan_ui_pub.h"
#include "param_config.h"
#include "gpio_pub.h"
#include "sys_ctrl_pub.h"
#include "icu_pub.h"
#include "power_save_pub.h"
#include "cmd_rx_sensitivity.h"
#include "cmd_evm.h"
#include "BkDriverGpio.h"
#include "ieee802_11_demo.h"
#include "command_line.h"
#include "role_launch.h"
#include "wdt_pub.h"
#include "saradc_pub.h"
#include "bk7011_cal_pub.h"
#include "flash_pub.h"
#include "mcu_ps_pub.h"
#include "manual_ps_pub.h"
#include "phy_trident.h"
#include "lwip/ping.h"
#include "ble_pub.h"
#include "sensor.h"
#include "spi_pub.h"
#include "i2c_pub.h"
#include "BkDriverTimer.h"
#include "BkDriverUart.h"
#include "BkDriverPwm.h"
#include "saradc_intf.h"
#include "spi_pub.h"
#include "ate_app.h"
#if CFG_BK_AWARE
#include "bk_aware.h"
#endif

#if (CFG_SUPPORT_MATTER)
#include "flash_namespace_value.h"
#endif

#if CFG_SUPPORT_LITEOS
#include "los_config.h"
#include "los_context.h"
#include "los_task.h"
#include "los_queue.h"
#include "los_sem.h"
#include "los_mux.h"
#include "los_memory.h"
#include "los_interrupt.h"
#include "los_swtmr.h"
#endif
#if AT_SERVICE_CFG
#include "atsvr_comm.h"
#endif
#if (CFG_SUPPORT_BLE == 1)
#if (CFG_BLE_VERSION == BLE_VERSION_4_2)
#include "application.h"
#endif

#if (CFG_BLE_VERSION == BLE_VERSION_5_1)
#include "app_ble_task.h"
#endif
#endif

#if (CFG_SOC_NAME == SOC_BK7221U)
#include "security_pub.h"
extern void sec_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
#endif

#include "temp_detect_pub.h"
#include "low_voltage_ps.h"
#include "power_save.h"

#ifdef monitor_printf_debug
#define monitor_dbg(fmt, ...)   bk_printf(fmt, ##__VA_ARGS__)
#else
#define monitor_dbg(fmt, ...)
#endif

static void task_Command( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );

bk_uart_t cli_uart = BK_UART_2;
int mico_debug_enabled;
static struct cli_st *pCli = NULL;
beken_semaphore_t log_rx_interrupt_sema = NULL;

extern u8* wpas_get_sta_psk(void);
extern int cli_putstr(const char *msg);
extern int hexstr2bin(const char *hex, u8 *buf, size_t len);
extern void make_tcp_server_command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
extern void net_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
#if (CFG_SUPPORT_MATTER)
extern void BkApplyUpdateCmdHandler(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );
extern void BkQueryImageCmdHandler(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );
extern void BkNotifyUpdateApplied(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );
#endif
uint32_t bk_wlan_reg_rx_mgmt_cb(mgmt_rx_cb_t cb, uint32_t rx_mgmt_flag);

#if CFG_AIRKISS_TEST
extern u32 airkiss_process(u8 start);
extern uint32_t airkiss_is_at_its_context(void);
#endif
#if CFG_QUICK_TRACK
extern void controlappc_start(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
#endif

#if CFG_AP_MONITOR_COEXIST_DEMO
extern u32 monitor_process(u8 start);
#endif

#if CFG_SARADC_CALIBRATE
static void adc_command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
#endif
#if !AT_SERVICE_CFG
static void cli_rx_callback(int uport, void *param);
#endif

static void efuse_cmd_test(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
static void efuse_mac_cmd_test(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);

#if CFG_SUPPORT_BKREG
#define BKREG_MAGIC_WORD0                 (0x01)
#define BKREG_MAGIC_WORD1                 (0xE0)
#define BKREG_MAGIC_WORD2                 (0xFC)
#define BKREG_MIN_LEN                     3
#endif

/* Find the command 'name' in the cli commands table.
* If len is 0 then full match will be performed else upto len bytes.
* Returns: a pointer to the corresponding cli_command struct or NULL.
*/
static const struct cli_command *lookup_command(char *name, int len)
{
    int i = 0;
    int n = 0;

    while (i < MAX_CMD_COUNT && n < pCli->num_commands)
    {
        if (pCli->commands[i]->name == NULL)
        {
            i++;
            continue;
        }
        /* See if partial or full match is expected */
        if (len != 0)
        {
            if (!os_strncmp(pCli->commands[i]->name, name, len))
                return pCli->commands[i];
        }
        else
        {
            if (!os_strcmp(pCli->commands[i]->name, name))
                return pCli->commands[i];
        }

        i++;
        n++;
    }

    return NULL;
}

/* Parse input line and locate arguments (if any), keeping count of the number
* of arguments and their locations.  Look up and call the corresponding cli
* function if one is found and pass it the argv array.
*
* Returns: 0 on success: the input line contained at least a function name and
*          that function exists and was called.
*          1 on lookup failure: there is no corresponding function for the
*          input line.
*          2 on invalid syntax: the arguments list couldn't be parsed
*/
extern void log_output_state(int flag);
 int handle_input(char *inbuf)
{
    struct
    {
        unsigned inArg: 1;
        unsigned inQuote: 1;
        unsigned done: 1;
    } stat;
    static char *argv[16];
    int argc = 0;
    int i = 0;
    const struct cli_command *command = NULL;
    const char *p;

    os_memset((void *)&argv, 0, sizeof(argv));
    os_memset(&stat, 0, sizeof(stat));

    do
    {
        switch (inbuf[i])
        {
        case '\0':
            if (stat.inQuote)
                return 2;
            stat.done = 1;
            break;

        case '"':
            if (i > 0 && inbuf[i - 1] == '\\' && stat.inArg)
            {
                os_memcpy(&inbuf[i - 1], &inbuf[i],
                          os_strlen(&inbuf[i]) + 1);
                --i;
                break;
            }
            if (!stat.inQuote && stat.inArg)
                break;
            if (stat.inQuote && !stat.inArg)
                return 2;

            if (!stat.inQuote && !stat.inArg)
            {
                stat.inArg = 1;
                stat.inQuote = 1;
                argc++;
                argv[argc - 1] = &inbuf[i + 1];
            }
            else if (stat.inQuote && stat.inArg)
            {
                stat.inArg = 0;
                stat.inQuote = 0;
                inbuf[i] = '\0';
            }
            break;

        case ' ':
            if (i > 0 && inbuf[i - 1] == '\\' && stat.inArg)
            {
                os_memcpy(&inbuf[i - 1], &inbuf[i],
                          os_strlen(&inbuf[i]) + 1);
                --i;
                break;
            }
            if (!stat.inQuote && stat.inArg)
            {
                stat.inArg = 0;
                inbuf[i] = '\0';
            }
            break;

        default:
            if (!stat.inArg)
            {
                stat.inArg = 1;
                argc++;
                argv[argc - 1] = &inbuf[i];
            }
            break;
        }
    }
    while (!stat.done && ++i < IN_BUF_LEN);

    if (stat.inQuote)
        return 2;

    if (argc < 1)
        return 0;

    if (!pCli->echo_disabled)
        os_printf("\r\n");

    /*
    * Some comamands can allow extensions like foo.a, foo.b and hence
    * compare commands before first dot.
    */
    i = ((p = os_strchr(argv[0], '.')) == NULL) ? 0 :
        (p - argv[0]);
    command = lookup_command(argv[0], i);
    if (command == NULL)
    {
        return 1;
    }

    /*open log  for ATE test*/
    #if AT_SERVICE_CFG
    if(g_env_param.workmode)
    {
        log_output_state(0);   //close log output
    }
    else
    {
        log_output_state(1); //open log output
    }
    #endif
    os_memset(pCli->outbuf, 0, OUT_BUF_LEN);
    cli_putstr("\r\n");

    #if CFG_USE_STA_PS
    /*if cmd,exit dtim ps*/
    if (os_strncmp(command->name, "ps", 2))
    {
    }
    #endif

    command->function(pCli->outbuf, OUT_BUF_LEN, argc, argv);
    cli_putstr(pCli->outbuf);
    os_printf("heap=%u\n", xPortGetFreeHeapSize());
    return 0;
}
#if !AT_SERVICE_CFG

/* Perform basic tab-completion on the input buffer by string-matching the
* current input line against the cli functions table.  The current input line
* is assumed to be NULL-terminated. */
static void tab_complete(char *inbuf, unsigned int *bp, const char *prompt)
{
    int i, n, m;
    const char *fm = NULL;

    os_printf("\r\n");

    /* show matching commands */
    for (i = 0, n = 0, m = 0; i < MAX_CMD_COUNT && n < pCli->num_commands;
            i++)
    {
        if (pCli->commands[i]->name != NULL)
        {
            if (!os_strncmp(inbuf, pCli->commands[i]->name, *bp))
            {
                m++;
                if (m == 1)
                    fm = pCli->commands[i]->name;
                else if (m == 2)
                    os_printf("%s %s ", fm,
                              pCli->commands[i]->name);
                else
                    os_printf("%s ",
                              pCli->commands[i]->name);
            }
            n++;
        }
    }

    /* there's only one match, so complete the line */
    if (m == 1 && fm)
    {
        n = os_strlen(fm) - *bp;
        if (*bp + n < IN_BUF_LEN)
        {
            os_memcpy(inbuf + *bp, fm + *bp, n);
            *bp += n;
            inbuf[(*bp)++] = ' ';
            inbuf[*bp] = '\0';
        }
    }

    /* just redraw input line */
    os_printf("%s%s", prompt, inbuf);
}

/* Get an input line.
*
* Returns: 1 if there is input, 0 if the line should be ignored. */
static int get_input(char *inbuf, unsigned int *bp, const char *prompt)
{
	if (inbuf == NULL){
		os_printf("inbuf_null\r\n");
		return 0;
	}

	while (cli_getchar(&inbuf[*bp]) == 1)
	{
#if CFG_SUPPORT_BKREG
		if ((0x01U == (UINT8)inbuf[*bp]) && (*bp == 0)) {
			(*bp)++;
			continue;
		} else if ((0xe0U == (UINT8)inbuf[*bp]) && (*bp == 1)) {
			(*bp)++;
			continue;
		} else if ((0xfcU == (UINT8)inbuf[*bp]) && (*bp == 2)) {
			(*bp)++;
			continue;
		} else {
			if ((0x01U == (UINT8)inbuf[0])
				&& (0xe0U == (UINT8)inbuf[1])
				&& (0xfcU == (UINT8)inbuf[2])
				&& (*bp == 3)) {
				uint8_t ch = inbuf[*bp];
				uint8_t left = ch, len = 4 + (uint8_t)ch;
				inbuf[*bp] = ch;
				(*bp)++;

				if (ch >= IN_BUF_LEN) {
					os_printf("Error: input buffer overflow\r\n");
					os_printf("%s",prompt);
					*bp = 0;
					return 0;
				}

				while (left--) {
					if (0 == cli_getchar((char*)&ch))
						break;

					inbuf[*bp] = ch;
					(*bp)++;
				}

				extern int bkreg_run_command(const char *content, int cnt);
				bkreg_run_command(inbuf, len);
				memset(inbuf, 0, len);
				*bp = 0;
				continue;
			}
		}
#endif
		if (inbuf[*bp] == RET_CHAR)
			continue;
		if (inbuf[*bp] == END_CHAR) {   /* end of input line */
			inbuf[*bp] = '\0';
			*bp = 0;
			return 1;
		}

		if ((inbuf[*bp] == 0x08) || /* backspace */
			(inbuf[*bp] == 0x7f)) { /* DEL */
			if (*bp > 0) {
				(*bp)--;
				if (!pCli->echo_disabled)
					os_printf("%c %c", 0x08, 0x08);
			}
			continue;
		}

		if (inbuf[*bp] == '\t') {
			inbuf[*bp] = '\0';
			tab_complete(inbuf, bp,prompt);
			continue;
		}

		if (!pCli->echo_disabled)
			os_printf("%c", inbuf[*bp]);

		(*bp)++;
		if (*bp >= IN_BUF_LEN) {
			os_printf("Error: input buffer overflow\r\n");
			os_printf("%s",prompt);
			*bp = 0;
			return 0;
		}

	}

	return 0;
}



/* Print out a bad command string, including a hex
* representation of non-printable characters.
* Non-printable characters show as "\0xXX".
*/
static void print_bad_command(char *cmd_string)
{
    if (cmd_string != NULL)
    {
        char *c = cmd_string;
        os_printf("command '");
        while (*c != '\0')
        {
            if (is_print(*c))
            {
                os_printf("%c", *c);
            }
            else
            {
                os_printf("\\0x%x", *c);
            }
            ++c;
        }
        os_printf("' not found\r\n");
    }
}

/* Main CLI processing thread
*
* Waits to receive a command buffer pointer from an input collector, and
* then processes.  Note that it must cleanup the buffer when done with it.
*
* Input collectors handle their own lexical analysis and must pass complete
* command lines to CLI.
*/
static void cli_main( uint32_t data )
{
    bk_uart_set_rx_callback(cli_uart, cli_rx_callback, NULL);

	#if CFG_RF_OTA_TEST
	demo_sta_app_init("CMW-AP", "12345678");
	#endif /* CFG_RF_OTA_TEST*/

    char prompt[5];
    if(get_ate_mode_state()==1)
        strcpy(prompt,"\r\n# ");
    else
        strcpy(prompt,"\r\n$ ");

    while (1)
    {
        int ret;
        char *msg = NULL;

        rtos_get_semaphore(&log_rx_interrupt_sema, BEKEN_NEVER_TIMEOUT);

        while(get_input(pCli->inbuf, &pCli->bp,prompt))
        {
            msg = pCli->inbuf;

            if (os_strcmp(msg, EXIT_MSG) == 0)
                goto exit;

            ret = handle_input(msg);
            if (ret == 1)
                print_bad_command(msg);
            else if (ret == 2)
                os_printf("syntax error\r\n");
            os_printf("%s",prompt);
        }
    }

exit:
    os_printf("CLI exited\r\n");
    os_free(pCli);
    pCli = NULL;

    bk_uart_set_rx_callback(cli_uart, NULL, NULL);
    rtos_delete_thread(NULL);
}
#endif
static void task_Command( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv )
{
#if CFG_OS_FREERTOS
    rtos_print_thread_status( pcWriteBuffer, xWriteBufferLen );
#endif
}

#if CFG_MEM_DEBUG
void printLeakMem(int leaktime);

static void memleak_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	uint32_t leaktime = 0;

	if (argc >= 2)
		leaktime = os_strtoul(argv[1], NULL, 10);

	printLeakMem(leaktime);
}
#endif

void tftp_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
}

static void partShow_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    bk_partition_t i;
    bk_logic_partition_t *partition;

    for( i = BK_PARTITION_BOOTLOADER; i <= BK_PARTITION_MAX; i++ )
    {
        partition = bk_flash_get_info( i );
        if (partition == NULL)
            continue;

        os_printf( "%4d | %11s |  Dev:%d  | 0x%08lx | 0x%08lx |\r\n", i,
                   partition->partition_description, partition->partition_owner,
                   partition->partition_start_addr, partition->partition_length);
    };

}

static void uptime_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    os_printf("UP time %ldms\r\n", rtos_get_time());
}

void tftp_ota_thread( beken_thread_arg_t arg )
{
    rtos_delete_thread( NULL );
}

void ota_Command( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv )
{
    rtos_create_thread( NULL,
                        BEKEN_APPLICATION_PRIORITY,
                        "LOCAL OTA",
                        (beken_thread_function_t)tftp_ota_thread,
                        0x4096,
                        0 );
}

void help_command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);

/*
*  Command buffer API
*/
void wifiscan_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
#if CFG_ROLE_LAUNCH
    LAUNCH_REQ param;

    param.req_type = LAUNCH_REQ_PURE_STA_SCAN;
    rl_sta_request_enter(&param, 0);
#else
    demo_scan_app_init();
#endif
}

void wifiadvscan_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    uint8_t *ap_ssid;

    if ( argc < 2 )
    {
        os_printf("Please input ssid\r\n");
        return;
    }
    ap_ssid = (uint8_t*)argv[1];

    demo_scan_adv_app_init(ap_ssid);
}

void softap_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    char *ap_ssid = NULL;
    char *ap_key;

    os_printf("SOFTAP_COMMAND\r\n\r\n");

    if (argc > 1 && strlen(argv[1]) == 0)
    {
        os_printf("invalid ssid\r\n");
        return;
    }

    if (argc == 2)
    {
        ap_ssid = argv[1];
        ap_key = "1";
    }
    else if (argc == 3)
    {
        ap_ssid = argv[1];
        ap_key = argv[2];
    }
    else
    {
        os_printf("parameter invalid\r\n");
    }

    if(ap_ssid)
    {
        demo_softap_app_init(ap_ssid, ap_key);
    }
}

void stop_wlan_intface_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    uint8_t inface = 0;

    os_printf("stop_wlan_intface_Command\r\n");
    if (argc == 2)
    {
        inface = strtoul(argv[1], NULL, 0);
        os_printf("stop wlan intface:%d\r\n", inface);

        if(inface == 0)
        {
            bk_wlan_stop(BK_SOFT_AP);
        }
        else if(inface == 1)
        {
            bk_wlan_stop(BK_STATION);
        }
    }
    else
    {
        os_printf("bad parameters\r\n");
    }
}

void add_virtual_intface(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    VIF_ADDCFG_ST cfg;
    u8 argc_cnt = 1;

    if(argc <= 1)
        return;

    if (!os_strncmp(argv[argc_cnt], "softap", sizeof("softap")))
    {
        cfg.wlan_role = BK_SOFT_AP;
    }
    else if (!os_strncmp(argv[argc_cnt], "sta", sizeof("sta")))
    {
        cfg.wlan_role = BK_STATION;
    }
    else
    {
        os_printf("role error:%s, must be softap or sta\r\n", argv[argc_cnt]);
        return;
    }
    argc_cnt++;

    if(argc == argc_cnt)
        cfg.ssid = "default ssid";
    else
        cfg.ssid = argv[argc_cnt];
    argc_cnt++;

    if(argc == argc_cnt)
        cfg.key = "1234567890";
    else
        cfg.key = argv[argc_cnt];
    argc_cnt++;

    if((cfg.wlan_role == BK_STATION) && (argc > argc_cnt))
    {
        if (!os_strncmp(argv[argc_cnt], "adv", sizeof("adv")))
            cfg.adv = 1;
        else
            cfg.adv = 0;
    }
    else
    {
        cfg.adv = 0;
    }
    argc_cnt++;

    cfg.name = NULL;

    os_printf("role: %d\r\n", cfg.wlan_role);
    os_printf("ssid: %s\r\n", cfg.ssid);
    os_printf("key : %s\r\n", cfg.key);
    os_printf("adv : %d\r\n", cfg.adv);

    demo_wlan_app_init(&cfg);
}

void del_virtual_intface(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    char *type;
    u8 role = 0xff;

	#if CFG_ROLE_LAUNCH
    LAUNCH_REQ param;
	#endif

    if(argc <= 1)
        return;

    type = argv[1];

    if (!os_strncmp(type, "softap", sizeof("softap")))
    {
        role = BK_SOFT_AP;

		#if CFG_ROLE_LAUNCH
        param.req_type = LAUNCH_REQ_DELIF_AP;

		rl_ap_request_enter(&param, 0);
		#endif
    }
    else if(!os_strncmp(type, "sta", sizeof("sta")))
    {
        role = BK_STATION;

		#if CFG_ROLE_LAUNCH
        param.req_type = LAUNCH_REQ_DELIF_STA;

		rl_sta_request_enter(&param, 0);
		#endif
    }

    if(role == 0xff)
    {
        os_printf("%s virtual intface not found\r\n", type );
        return;
    }

#if (0 == CFG_ROLE_LAUNCH)
    bk_wlan_stop(role);
#endif
}

void show_virtual_intface(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    u8 i = 1;
    u8 *mac_ptr;
    char *role;
    const char *name;
    struct netif *lwip_if;
    VIF_INF_PTR vif_entry;

    os_printf("\r\nshow_virtual_intface:\r\n");
    vif_entry = (VIF_INF_PTR)rwm_mgmt_is_vif_first_used();
    while(vif_entry)
    {
        mac_ptr = (u8 *)&vif_entry->mac_addr;
        lwip_if = (struct netif *)vif_entry->priv;
        name = lwip_if->hostname;
        if(vif_entry->type == VIF_AP)
        {
            role = "softap\0";
        }
        else if(vif_entry->type == VIF_STA)
        {
            role = "sta\0";
        }
        else
        {
            role = "others\0";
        }

        os_printf("%1d: %s, %s, %02x:%02x:%02x:%02x:%02x:%02x\r\n", i, name, role,
                  mac_ptr[0], mac_ptr[1], mac_ptr[2], mac_ptr[3], mac_ptr[4], mac_ptr[5]);

        vif_entry = (VIF_INF_PTR)rwm_mgmt_next(vif_entry);
        i++;
    }

    os_printf("\r\nend of show\r\n");

}
#if 1
uint32_t channel_count = 0;
void cli_monitor_cb(uint8_t *data, int len, wifi_link_info_t *info)
{
    uint32_t count, i;

    count = _MIN(32, len);
    monitor_dbg("cli_monitor_cb:%d:%d\r\n", count, len);
    for(i = 0; i < count; i ++)
    {
        monitor_dbg("%x ", data[i]);
    }
    monitor_dbg("\r\n");

    channel_count ++;
}

beken_thread_t mtr_thread_handle = NULL;
beken_thread_t cli_thread_handle = NULL;

uint32_t  mtr_stack_size = 2000;
#define THD_MTR_PRIORITY                    5

void mtr_thread_main( void *arg )
{
    static uint32_t channel_num = 1;

    // stop monitor mode, need stop hal mac first
    bk_wlan_stop_monitor();
    // then set monitor callback
    bk_wlan_register_monitor_cb(NULL);

    // start monitor, need set callback
    bk_wlan_register_monitor_cb(cli_monitor_cb);
    // then start hal mac
    bk_wlan_start_monitor();

    while(1)
    {
        channel_count = 0;

        bk_wlan_set_channel_sync(channel_num);
        channel_num ++;

        if(14 == channel_num)
        {
            channel_num = 1;
        }

        rtos_delay_milliseconds(100);
        os_printf("channel:%d count:%x\r\n", channel_num, channel_count);
    }
}
#endif

void mtr_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    uint32_t channel_num;

    if(argc != 2)
    {
        os_printf("monitor_parameter invalid\r\n");
        return;
    }

    channel_num = os_strtoul(argv[1], NULL, 10);
    if(99 == channel_num)
    {
        cmd_printf("stop monitor\r\n");
        bk_wlan_stop_monitor();
    }
    else
    {
        bk_wlan_register_monitor_cb(cli_monitor_cb);
        bk_wlan_start_monitor();
        bk_wlan_set_channel_sync(channel_num);
    }
}

void sta_adv_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    char *oob_ssid = NULL;
    char *connect_key;

    os_printf("sta_adv_Command\r\n");
    if (argc == 2)
    {
        oob_ssid = argv[1];
        connect_key = "1";
    }
    else if (argc == 3)
    {
        oob_ssid = argv[1];
        connect_key = argv[2];
    }
    else
    {
        os_printf("parameter invalid\r\n");
        return;
    }

    if(oob_ssid)
    {
        demo_sta_adv_app_init(oob_ssid, connect_key);
    }
}

void show_sta_psk(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	uint8_t *psk;
	int i;

	psk = wpas_get_sta_psk();
	os_printf("John# show_sta_psk.r\n");
	for ( i = 0 ; i < 32 ; i++ )
	{
	    os_printf("%02x ", psk[i]);
	}
	os_printf("\r\n");
}

#include "conv_utf8_pub.h"
void sta_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    char *oob_ssid = NULL;
    char *connect_key;

    os_printf("sta_Command\r\n");

    if (argc > 1 && strlen(argv[1]) == 0)
    {
        os_printf("invalid ssid\r\n");
        return;
    }

    if (argc == 2)
    {
        oob_ssid = argv[1];
        connect_key = "1";
    }
    else if (argc == 3)
    {
        oob_ssid = argv[1];
        connect_key = argv[2];
    }
    else
    {
        os_printf("parameter invalid\r\n");
    }

	if (oob_ssid)
    {
		char *oob_ssid_tp = oob_ssid;
#if CFG_USE_CONV_UTF8
		oob_ssid_tp = (char *)conv_utf8((uint8_t *)oob_ssid);
#endif

		if (oob_ssid_tp)
		{
#if CFG_AIRKISS_TEST
			if (airkiss_is_at_its_context()) {
				os_printf("airkiss is on-the-go\r\n");
				return;
			}
#endif
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
#if CFG_TX_EVM_TEST || CFG_RX_SENSITIVITY_TEST
			if (!get_ate_mode_state() && !os_strcasecmp((const char *)oob_ssid_tp, "CMW-AP")) {
				extern char ate_mode_state;
				extern void improve_rx_sensitivity(void);

				ate_mode_state = 2; /* 0: no_ate, 1: ate, 2: fake_ate */
				power_save_rf_hold_bit_set(RF_HOLD_RF_SLEEP_BIT);
				sctrl_flash_select_dco();
				improve_rx_sensitivity();
			}
#endif
#endif
			demo_sta_app_init((char *)oob_ssid_tp, connect_key);
#if CFG_USE_CONV_UTF8
			os_free(oob_ssid_tp);
#endif
		}
        else
        {
            os_printf("not buf for utf8\r\n");
        }
    }
}

#if CFG_WPA2_ENTERPRISE
/**
 * cli command: sta_eap <ssid>, connect to EAP-TLS AP.
 *
 * restrictions: EAP-TLS is based on PKI, both AP and STA may have certificate. So
 * we must install certificate and private key to system. For example, `beken-iot-1.pem'
 * is STA's certificate, `beken-iot-1.key' is private key, `rootca.pem' is the RootCA.
 * These certificates and private key may be registered to system via `register_xfile'
 * function.
 */
void sta_eap_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
#if 0 // TODO: backport to 3.0
	char *ssid = NULL;
	char *ca = "cacert.pem";
	char *client_cert = "beken-iot-1.pem";
	char *private_key = "beken-iot-1.key";
	char *private_key_passwd = "12345678";
	char *identity = "user";

	if ((argc < 2) || (argc > 5)) {
		CLI_LOGI("invalid argc number\n");
		return;
	}

	ssid = argv[1];

	char *oob_ssid_tp = ssid;
#if CFG_USE_CONV_UTF8
	oob_ssid_tp = (char *)conv_utf8((uint8_t *)ssid);
#endif

	if (oob_ssid_tp) {
		int len;
		wifi_sta_config_t *sta_config;

		len = os_strlen((char *)oob_ssid_tp);
		if (SSID_MAX_LEN < len) {
			CLI_LOGI("ssid name more than 32 Bytes\n");
			return;
		}

		sta_config = os_zalloc(sizeof(*sta_config));
		if (!sta_config) {
			CLI_LOGI("Cannot alloc STA config\n");
			return;
		}

		os_strlcpy(sta_config->ssid, oob_ssid_tp, sizeof(sta_config->ssid));
		sta_config->password[0] = '\0';	// No passwd needed fo EAP.
		os_strlcpy(sta_config->eap, "TLS", sizeof(sta_config->eap));
		os_strlcpy(sta_config->identity, identity, sizeof(sta_config->identity));
		os_strlcpy(sta_config->ca, ca, sizeof(sta_config->ca));
		os_strlcpy(sta_config->client_cert, client_cert, sizeof(sta_config->client_cert));
		os_strlcpy(sta_config->private_key, private_key, sizeof(sta_config->private_key));
		os_strlcpy(sta_config->private_key_passwd, private_key_passwd, sizeof(sta_config->private_key_passwd));
		os_strlcpy(sta_config->phase1, "tls_disable_time_checks=1", sizeof(sta_config->phase1));

		CLI_LOGI("ssid:%s key:%s\n", sta_config->ssid, sta_config->password);
		CLI_LOGI("eap:%s identity:%s\n", sta_config->eap, sta_config->identity);
		CLI_LOGI("ca:%s client_cert:%s\n", sta_config->ca, sta_config->client_cert);
		CLI_LOGI("private_key:%s\n", sta_config->private_key);
		BK_LOG_ON_ERR(bk_wifi_sta_set_config(sta_config));
		BK_LOG_ON_ERR(bk_wifi_sta_start());

		os_free(sta_config);

#if CFG_USE_CONV_UTF8
		os_free(oob_ssid_tp);
#endif
	} else {
		os_printf("not buf for utf8\r\n");
	}
#endif
}
#endif

#if CFG_WPA_CTRL_IFACE
void wifi_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    char *oob_ssid = NULL;
    char *connect_key;

    os_printf("wifi_Command\r\n");
    if (argc == 2)
    {
        oob_ssid = argv[1];
        connect_key = "1";
    }
    else if (argc == 3)
    {
        oob_ssid = argv[1];
        connect_key = argv[2];
    }
    else
    {
        os_printf("parameter invalid\r\n");
    }

    if(oob_ssid)
    {
        demo_sta_app_init(oob_ssid, connect_key);
    }
}
#endif

void easylink_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
}

#if CFG_AIRKISS_TEST
void airkiss_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    u8 start = 0;

    if(argc != 2)
    {
        os_printf("need 2 parameters: airkiss 1(start), 0(stop)\r\n");
        return;
    }

    start = strtoul(argv[1], NULL, 0);

    airkiss_process(start);
}

#endif

#if CFG_USE_TEMPERATURE_DETECT
void temp_detect_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    u8 start = 0;

    if(argc != 2)
    {
        os_printf("need 2 parameters: airkiss 1(start), 0(stop)\r\n");
        return;
    }

    start = strtoul(argv[1], NULL, 0);

    if(start == 0)
        temp_detect_pause_timer();
    else
        temp_detect_restart_detect();
}
#endif

void wifistate_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    os_printf("wifistate_Command\r\n");
    demo_state_app_init();
}

void blacklist_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    int blacklist_ena = 0;

    if (argc != 2)
    {
        os_printf("blacklist <0|1>\n");
    }
    else
    {
        blacklist_ena = strtoul(argv[1], NULL, 0);
        if (blacklist_ena)
            wlan_sta_enable_ssid_blacklist();
        else
            wlan_sta_disable_ssid_blacklist();
        os_printf("blacklist %s\n", blacklist_ena ? "enabled" : "disabled");
    }
}


void wifidebug_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    os_printf("wifidebug_Command\r\n");
}

void ifconfig_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    demo_ip_app_init();
}

void arp_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    os_printf("arp_Command\r\n");
}

void ping_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    os_printf("ping_Command\r\n");

    if (argc == 1)
    {
        os_printf("Please input: ping <host address>\n");
    }
    else
    {
        int count = 4;
        if (argc >= 3)
            count = atoi(argv[2]);
            os_printf("ping IP address: %s for %d times\n", argv[1], count);
            ping(argv[1], count, 0);
	}
}

void dns_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    os_printf("dns_Command\r\n");
}

void socket_show_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    os_printf("socket_show_Command\r\n");
}

void memory_show_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
#if CFG_OS_FREERTOS
    cmd_printf("free memory %d\r\n", xPortGetFreeHeapSize());
#elif CFG_SUPPORT_LITEOS
	LOS_MEM_POOL_STATUS status;
	(void)LOS_MemInfoGet(m_aucSysMem0, &status);

    cmd_printf("freeNodeNum:%d, maxFreeNodeSize:%d, usedNodeNum:%d, usageWaterLine:%d\r\n", status.freeNodeNum, status.maxFreeNodeSize, status.usedNodeNum, status.usageWaterLine);
    cmd_printf("used memory %d\r\n", status.totalUsedSize);
    cmd_printf("free memory %d\r\n", status.totalFreeSize);
#else
#endif
}

void memory_dump_Command( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv )
{
    int i;
    uint8_t *pstart;
    uint32_t start, length;

    if (argc != 3)
    {
        cmd_printf("Usage: memdump <addr> <length>.\r\n");
        return;
    }

    start = strtoul(argv[1], NULL, 0);
    length = strtoul(argv[2], NULL, 0);
    pstart = (uint8_t *)start;

    for(i = 0; i < length; i++)
    {
        cmd_printf("%02x ", pstart[i]);
        if (i % 0x10 == 0x0F)
        {
            cmd_printf("\r\n");

        }
    }
}


void memory_set_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    os_printf("memory_set_Command\r\n");
}

void driver_state_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    os_printf("driver_state_Command\r\n");
}

void get_version(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    os_printf("get_version\r\n");
    os_printf("firmware version : %s", BEKEN_SDK_REV);
}

void reboot(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    if (argc == 2)
    {
        if (!os_strcasecmp(argv[1], "ate"))
        {
            bk_reboot_for_ate();
            return;
        }
    }
    bk_reboot();
}

static void echo_cmd_handler(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    if (argc == 1)
    {
        os_printf("Usage: echo on/off. Echo is currently %s\r\n",
                  pCli->echo_disabled ? "Disabled" : "Enabled");
        return;
    }

    if (!os_strcasecmp(argv[1], "on"))
    {
        os_printf("Enable echo\r\n");
        pCli->echo_disabled = 0;
    }
    else if (!os_strcasecmp(argv[1], "off"))
    {
        os_printf("Disable echo\r\n");
        pCli->echo_disabled = 1;
    }
}

static void cli_exit_handler(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    // exit command not executed
}

/*
CMD FORMAT: GPIO CMD index PARAM
exmaple:GPIO 0 18 2               (config GPIO18 input & pull-up)
*/
static void Gpio_op_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    uint32_t ret, id, mode, i;
    char cmd0 = 0;
    char cmd1 = 0;
    char cmd;

    for(i = 0; i < argc; i++)
    {
        os_printf("Argument %d = %s\r\n", i + 1, argv[i]);
    }

    if(argc == 4)
    {
        cmd = argv[1][0];
        mode = argv[3][0];

        cmd0 = argv[2][0] - 0x30;
        cmd1 = argv[2][1] - 0x30;

        id = (uint32_t)(cmd0 * 10 + cmd1);
        os_printf("---%x,%x----\r\n", id, mode);
        ret = BKGpioOp(cmd, id, mode);
        os_printf("gpio op:%x\r\n", ret);
    }
    else
        os_printf("cmd param error\r\n");
}

void test_fun(char para)
{
    os_printf("---%d---\r\n", para);
}
/*
cmd format: GPIO_INT cmd index  triggermode
enable: GPIO_INT 1 18 0
*/
static void Gpio_int_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    uint32_t id, mode;
    char cmd0 = 0;
    char cmd1 = 0;
    char cmd;

    if(argc == 4)
    {
        cmd = argv[1][0] - 0x30;
        mode = argv[3][0] - 0x30;

        cmd0 = argv[2][0] - 0x30;
        cmd1 = argv[2][1] - 0x30;

        id = (uint32_t)(cmd0 * 10 + cmd1);
        BKGpioIntcEn(cmd, id, mode, test_fun);
    }
    else
        os_printf("cmd param error\r\n");

}


#if CFG_USE_SDCARD_HOST
/*
sdtest I 0 --
sdtest R secnum
sdtest W secnum
*/
extern uint32_t sdcard_intf_test(void);
extern uint32_t test_sdcard_read(uint32_t blk);
extern uint32_t test_sdcard_write(uint32_t blk);
extern void sdcard_intf_close(void);

static void sd_operate(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    uint32_t cmd;
    uint32_t blknum;
    uint32_t ret;
    if(argc == 3)
    {
        cmd = argv[1][0];
        blknum = os_strtoul(argv[2], NULL, 16);
        switch(cmd)
        {
        case 'I':
            ret = sdcard_intf_test();
            os_printf("init :%x\r\n", ret);
            break;
        case 'R':
            ret = test_sdcard_read(blknum);
            os_printf("read :%x\r\n", ret);
            break;
        case 'W':
            ret = test_sdcard_write(blknum);
            os_printf("write :%x\r\n", ret);
            break;
        case 'C':
            sdcard_intf_close();
            os_printf("close \r\n");
            break;
        default:
            break;
        }
    }
    else
        os_printf("cmd param error\r\n");
}

#endif
/*
format: FLASH  E/R/W  0xABCD
example:	    FLASH  R  0x00100

*/

extern OSStatus test_flash_write(volatile uint32_t start_addr, uint32_t len);
extern OSStatus test_flash_erase(volatile uint32_t start_addr, uint32_t len);
extern OSStatus test_flash_read(volatile uint32_t start_addr, uint32_t len);
extern OSStatus test_flash_read_time(volatile uint32_t start_addr, uint32_t len);
extern OSStatus test_flash_read_without_print(volatile uint32_t start_addr, uint32_t len);
beken_thread_t idle_read_flash_handle = NULL;

static void test_idle_read_flash(void *arg) {
	while (1) {
		test_flash_read_without_print(0x1000, 1000);
		test_flash_read_without_print(0x100000, 1000);
		test_flash_read_without_print(0x200000, 1000);
		test_flash_read_without_print(0x300000, 0x1000);
	}
	rtos_delete_thread(&idle_read_flash_handle);
}

static void flash_command_test(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    char cmd = 0;
    uint32_t len = 0;
    uint32_t addr = 0;

#if 0//(CONFIG_SYSTEM_CTRL)
	if (os_strcmp(argv[1], "config") == 0) {
		uint32_t flash_src_clk = os_strtoul(argv[2], NULL, 10);
		uint32_t flash_div_clk = os_strtoul(argv[3], NULL, 10);
		uint32_t flash_line_mode = os_strtoul(argv[4], NULL, 10);

		if (FLASH_CLK_XTAL == flash_src_clk) {
			sys_drv_flash_cksel(flash_src_clk);
		}

		if((FLASH_CLK_XTAL != sys_drv_flash_get_clk_sel()) && (0 == flash_div_clk)) {
			os_printf("Config fail. Please set src clk as 26M, or set div larger than 0 firstly.\n");
			return;
		}

		sys_drv_flash_set_clk_div(flash_div_clk);
		sys_drv_flash_cksel(flash_src_clk);
		flash_set_line_mode(flash_line_mode);
		os_printf("flash_src_clk = %u. [0 -> 26M; 1->98M; 2-> 120M]\n", flash_src_clk);
		os_printf("flash_div_clk = %u. \n", flash_div_clk);
		os_printf("flash_line_mode = %u.  \n", flash_line_mode);

		return;
	}
#endif
	if (os_strcmp(argv[1], "idle_read_start") == 0) {
		uint32_t task_prio = os_strtoul(argv[2], NULL, 10);
		os_printf("idle_read_flash task start: task_prio = %u.\n", task_prio);
		rtos_create_thread(&idle_read_flash_handle, task_prio,
			"idle_read_flash",
			(beken_thread_function_t) test_idle_read_flash,
			4096,
			(beken_thread_arg_t)0);

		return;
	} else if (os_strcmp(argv[1], "idle_read_stop") == 0) {
		if (idle_read_flash_handle) {
			rtos_delete_thread(&idle_read_flash_handle);
			idle_read_flash_handle = NULL;
			os_printf("idle_read_flash task stop\n");
		}
		return;
	}

    if(argc == 4)
    {
        cmd = argv[1][0];
		addr = atoi(argv[2]);
		len = atoi(argv[3]);

       	switch(cmd)
        {
	        case 'E':
				bk_flash_enable_security(FLASH_PROTECT_NONE);
				test_flash_erase(addr,len);
				bk_flash_enable_security(FLASH_UNPROTECT_LAST_BLOCK);
	         break;

        	case 'R':
				test_flash_read(addr,len);
	            break;
	        case 'W':
		        bk_flash_enable_security(FLASH_PROTECT_NONE);
				test_flash_write(addr,len);
				bk_flash_enable_security(FLASH_UNPROTECT_LAST_BLOCK);
	         break;
	//to check whether protection mechanism can work
			case 'N':
				test_flash_erase(addr,len);
				break;
			case 'M':
				test_flash_write(addr,len);
				break;
            case 'T':
				test_flash_read_time(addr,len);
				break;
        	default:
            break;
        }
    }
	else
	{
		os_printf("FLASH <R/W/E/M/N/T> <start_addr> <len>\r\n");
	}
}

/*UART  I  index
example:   UART I 0
*/
static void Uart_command_test(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    char cmd;
    uint8_t index = 0;
    if(argc == 3)
    {
        cmd = argv[1][0];
        index = argv[2][0] - 0x30;

        if(cmd == 'I')
        {
            bk_uart_initialize_test(0, index, NULL);
        }
    }
    else
        os_printf("cmd param error\r\n");

}
#if CFG_TX_EVM_TEST
static void tx_evm_cmd_test(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    int ret = do_evm(NULL, 0, argc, argv);
    if(ret)
    {
        os_printf("tx_evm bad parameters\r\n");
    }
}
#endif
#if CFG_RX_SENSITIVITY_TEST
static void rx_sens_cmd_test(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    int ret = do_rx_sensitivity(NULL, 0, argc, argv);
    if(ret)
    {
        os_printf("rx sensitivity bad parameters\r\n");
    }
}
#endif

#if (CFG_SOC_NAME != SOC_BK7231)
static void efuse_cmd_test(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    uint8_t addr, data;

    if(argc == 3)
    {
        if (os_strncmp(argv[1], "-r", 2) == 0) {
            hexstr2bin(argv[2], &addr, 1);
            os_printf("efuse read: addr-0x%02x, data-0x%02x\r\n",
                        addr, wifi_read_efuse(addr));
        }
    }
    else if(argc == 4)
    {
        if(os_strncmp(argv[1], "-w", 2) == 0)  {
            hexstr2bin(argv[2], &addr, 1);
            hexstr2bin(argv[3], &data, 6);
            os_printf("efuse write: addr-0x%02x, data-0x%02x, ret:%d\r\n",
                        addr, data, wifi_write_efuse(addr, data));
        }
    }
    else {
        os_printf("efuse [-r addr] [-w addr data]\r\n");
    }
}

static void efuse_mac_cmd_test(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    uint8_t mac[6];

    if (argc == 1)
    {
        if(wifi_get_mac_address_from_efuse(mac))
            os_printf("MAC address: %02x-%02x-%02x-%02x-%02x-%02x\r\n",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    else if(argc == 2)
    {
        if (os_strncmp(argv[1], "-r", 2) == 0) {
            if(wifi_get_mac_address_from_efuse(mac))
                os_printf("MAC address: %02x-%02x-%02x-%02x-%02x-%02x\r\n",
                        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        }
    }
    else if(argc == 3)
    {
        if(os_strncmp(argv[1], "-w", 2) == 0)  {
            hexstr2bin(argv[2], mac, 6);
            //if(wifi_set_mac_address_to_efuse(mac))
                os_printf("Set MAC address: %02x-%02x-%02x-%02x-%02x-%02x\r\n",
                        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        }
    }
    else {
        os_printf("efusemac [-r] [-w] [mac]\r\n");
    }
}
#endif //(CFG_SOC_NAME != SOC_BK7231)


#if CFG_SUPPORT_BKREG
#define BKCMD_RXSENS_R      'r'
#define BKCMD_RXSENS_X      'x'
#define BKCMD_RXSENS_s      's'

#define BKCMD_TXEVM_T       't'
#define BKCMD_TXEVM_X       'x'
#define BKCMD_TXEVM_E       'e'

void bkreg_cmd_handle_input(char *inbuf, int len)
{
    if(((char)BKREG_MAGIC_WORD0 == inbuf[0])
            && ((char)BKREG_MAGIC_WORD1 == inbuf[1])
            && ((char)BKREG_MAGIC_WORD2 == inbuf[2]))
    {
        if(cli_getchars(inbuf, len))
        {
            bkreg_run_command(inbuf, len);
            os_memset(inbuf, 0, len);
        }
    }
    else if((((char)BKCMD_RXSENS_R == inbuf[0])
             && ((char)BKCMD_RXSENS_X == inbuf[1])
             && ((char)BKCMD_RXSENS_s == inbuf[2]))
            || (((char)BKCMD_TXEVM_T == inbuf[0])
                && ((char)BKCMD_TXEVM_X == inbuf[1])
                && ((char)BKCMD_TXEVM_E == inbuf[2])) )
    {
        if(cli_getchars(inbuf, len))
        {
            handle_input(inbuf);
            os_memset(inbuf, 0, len);
        }
    }

}
#endif

static void phy_cca_test(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	if(argc != 2)
	{
		os_printf("cca open\r\n");
		os_printf("cca close\r\n");
		os_printf("cca show\r\n");
		return;
	}

	if (os_strncmp(argv[1], "open", 4) == 0) {
		phy_open_cca();
		os_printf("cca opened\r\n");
	} else if (os_strncmp(argv[1], "close", 4) == 0) {
		phy_close_cca();
		os_printf("cca closed\r\n");
	} else if (os_strncmp(argv[1], "show", 4) == 0) {
		phy_show_cca();
	} else {
		os_printf("cca open\r\n");
		os_printf("cca close\r\n");
		os_printf("cca show\r\n");
	}
}

#if CFG_SUPPORT_OTA_TFTP
extern void tftp_start(void);
extern void string_to_ip(char *s);
static void tftp_ota_get_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    short len = 0,i;
    extern char		BootFile[] ;

    if(argc > 3 )
        {
        os_printf("ota server_ip ota_file\r\n");
        return;
    }

     os_printf("%s\r\n",argv[1]);

    os_strcpy(BootFile,argv[2]);
    os_printf("%s\r\n",BootFile);
    string_to_ip (argv[1]);


    tftp_start();

    return;

}
#endif

#if CFG_SUPPORT_OTA_HTTP
void http_ota_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    int ret;
    if ( argc != 2 ) {
        goto HTTP_CMD_ERR;
    }
    ret = http_ota_download(argv[1]);

    if (0 != ret) {
        os_printf("http_ota download failed.");
    }

    return;

HTTP_CMD_ERR:
    os_printf("Usage:http_ota [url:]\r\n");
}
#endif

static void reg_write_read_test(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    UINT32 reg_addr = 0, reg_value = 0;
    UINT8 optr_len = 0, optr_tab[9];

    os_memset(optr_tab, 0, 9);
    os_memset(optr_tab, 0x30, 8);

    if(os_strncmp(argv[1], "-r", 2) == 0)
    {
        if(argc != 3)
        {
            os_printf("regshow -r addr\r\n");
            return;
        }

        optr_len = os_strlen(argv[2]);
        if(optr_len > 8)
        {
            os_printf("addr 0-FFFFFFFF\r\n");
            return;
        }
        optr_len = 8 - optr_len;
        os_memcpy(&optr_tab[optr_len], argv[2], os_strlen(argv[2]));

        hexstr2bin((char*)optr_tab, (u8 *)&reg_addr, 4);
        reg_addr = ntohl(reg_addr);
        os_printf("regshow R: addr:0x%08x, value:0x%08x\r\n", reg_addr, REG_READ(reg_addr));
    }
    else if(os_strncmp(argv[1], "-w", 2) == 0)
    {
        if(argc != 4)
        {
            os_printf("regshow -w addr value\r\n");
            return;
        }

        optr_len = os_strlen(argv[2]);
        if(optr_len > 8)
        {
            os_printf("addr 0-FFFFFFFF\r\n");
            return;
        }
        optr_len = 8 - optr_len;
        os_memcpy(&optr_tab[optr_len], argv[2], os_strlen(argv[2]));

        hexstr2bin((char*)optr_tab, (u8 *)&reg_addr, 4);
        reg_addr = ntohl(reg_addr);


        os_memset(optr_tab, 0x30, 8);
        optr_len = os_strlen(argv[3]);
        if(optr_len > 8)
        {
            os_printf("value 0-FFFFFFFF\r\n");
            return;
        }
        optr_len = 8 - optr_len;
        os_memcpy(&optr_tab[optr_len], argv[3], os_strlen(argv[3]));
        hexstr2bin((char*)optr_tab, (u8 *)&reg_value, 4);
        reg_value = ntohl(reg_value);

        REG_WRITE(reg_addr, reg_value);

        extern INT32 rwnx_cal_save_trx_rcbekn_reg_val(void);
        // when write trx and rc beken regs, updata registers save.
        if( (reg_addr & 0xfff0000) == 0x1050000)
            rwnx_cal_save_trx_rcbekn_reg_val();

        os_printf("regshow W: addr:0x%08x, value:0x%08x - check:0x%08x\r\n",
            reg_addr, reg_value, REG_READ(reg_addr));
    }
    else
    {
        os_printf("regshow -w/r addr [value]\r\n");
    }
}

#if ((CFG_SOC_NAME != SOC_BK7231) && (CFG_SUPPORT_BLE == 1) && (CFG_BLE_USE_CLI == 1))
#if (CFG_BLE_VERSION == BLE_VERSION_4_2)
#include "ble_api.h"
#elif ((CFG_BLE_VERSION == BLE_VERSION_5_1) || (CFG_BLE_VERSION == BLE_VERSION_5_2))
#include "ble_api_5_x.h"
#include "rwprf_config.h"
#endif

#define BUILD_UINT16(loByte, hiByte) \
          ((uint16_t)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define BK_ATT_DECL_PRIMARY_SERVICE_128     {0x00,0x28,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
#define BK_ATT_DECL_CHARACTERISTIC_128      {0x03,0x28,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
#define BK_ATT_DESC_CLIENT_CHAR_CFG_128     {0x02,0x29,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

#define WRITE_REQ_CHARACTERISTIC_128        {0x01,0xFF,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0}
#define INDICATE_CHARACTERISTIC_128         {0x02,0xFF,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0}
#define NOTIFY_CHARACTERISTIC_128           {0x03,0xFF,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0}

static const uint8_t test_svc_uuid[16] = {0xFF,0xFF,0,0,0x34,0x56,0,0,0,0,0x28,0x37,0,0,0,0};

enum
{
	TEST_IDX_SVC,
	TEST_IDX_FF01_VAL_CHAR,
	TEST_IDX_FF01_VAL_VALUE,
	TEST_IDX_FF02_VAL_CHAR,
	TEST_IDX_FF02_VAL_VALUE,
	TEST_IDX_FF02_VAL_IND_CFG,
	TEST_IDX_FF03_VAL_CHAR,
	TEST_IDX_FF03_VAL_VALUE,
	TEST_IDX_FF03_VAL_NTF_CFG,
	TEST_IDX_NB,
};

#if (CFG_BLE_VERSION == BLE_VERSION_5_2)
bk_attm_desc_t test_att_db[TEST_IDX_NB] =
{
	//  Service Declaration
	[TEST_IDX_SVC]              = {BK_ATT_DECL_PRIMARY_SERVICE_128, PROP(RD), 0},

	//  Level Characteristic Declaration
	[TEST_IDX_FF01_VAL_CHAR]    = {BK_ATT_DECL_CHARACTERISTIC_128,  PROP(RD), 0},
	//  Level Characteristic Value
	[TEST_IDX_FF01_VAL_VALUE]   = {WRITE_REQ_CHARACTERISTIC_128,    PROP(WR), 128|OPT(NO_OFFSET)},

	[TEST_IDX_FF02_VAL_CHAR]    = {BK_ATT_DECL_CHARACTERISTIC_128,  PROP(RD), 0},
	//  Level Characteristic Value
	[TEST_IDX_FF02_VAL_VALUE]   = {INDICATE_CHARACTERISTIC_128,     PROP(I), 128|OPT(NO_OFFSET)},

	//  Level Characteristic - Client Characteristic Configuration Descriptor

	[TEST_IDX_FF02_VAL_IND_CFG] = {BK_ATT_DESC_CLIENT_CHAR_CFG_128, PROP(RD)|PROP(WR),OPT(NO_OFFSET)},

	[TEST_IDX_FF03_VAL_CHAR]    = {BK_ATT_DECL_CHARACTERISTIC_128,  PROP(RD), 0},
	//  Level Characteristic Value
	[TEST_IDX_FF03_VAL_VALUE]   = {NOTIFY_CHARACTERISTIC_128,       PROP(N), 128|OPT(NO_OFFSET)},

	//  Level Characteristic - Client Characteristic Configuration Descriptor

	[TEST_IDX_FF03_VAL_NTF_CFG] = {BK_ATT_DESC_CLIENT_CHAR_CFG_128, PROP(RD)|PROP(WR), OPT(NO_OFFSET)},
};
#elif (CFG_BLE_VERSION == BLE_VERSION_5_1) || (CFG_BLE_VERSION == BLE_VERSION_4_2)
bk_attm_desc_t test_att_db[TEST_IDX_NB] =
{
	//  Service Declaration
	[TEST_IDX_SVC]              = {BK_ATT_DECL_PRIMARY_SERVICE_128, BK_PERM_SET(RD, ENABLE), 0, 0},

	//  Level Characteristic Declaration
	[TEST_IDX_FF01_VAL_CHAR]    = {BK_ATT_DECL_CHARACTERISTIC_128,  BK_PERM_SET(RD, ENABLE), 0, 0},
	//  Level Characteristic Value
	[TEST_IDX_FF01_VAL_VALUE]   = {WRITE_REQ_CHARACTERISTIC_128,    BK_PERM_SET(WRITE_REQ, ENABLE), BK_PERM_SET(RI, ENABLE)|BK_PERM_SET(UUID_LEN, UUID_16), 128},

	[TEST_IDX_FF02_VAL_CHAR]    = {BK_ATT_DECL_CHARACTERISTIC_128,  BK_PERM_SET(RD, ENABLE), 0, 0},
	//  Level Characteristic Value
	[TEST_IDX_FF02_VAL_VALUE]   = {INDICATE_CHARACTERISTIC_128,     BK_PERM_SET(IND, ENABLE), BK_PERM_SET(RI, ENABLE)|BK_PERM_SET(UUID_LEN, UUID_16), 128},

	//  Level Characteristic - Client Characteristic Configuration Descriptor

	[TEST_IDX_FF02_VAL_IND_CFG] = {BK_ATT_DESC_CLIENT_CHAR_CFG_128, BK_PERM_SET(RD, ENABLE)|BK_PERM_SET(WRITE_REQ, ENABLE), 0, 0},

    [TEST_IDX_FF03_VAL_CHAR]    = {BK_ATT_DECL_CHARACTERISTIC_128,  BK_PERM_SET(RD, ENABLE), 0, 0},
	//  Level Characteristic Value
	[TEST_IDX_FF03_VAL_VALUE]   = {NOTIFY_CHARACTERISTIC_128,       BK_PERM_SET(NTF, ENABLE), BK_PERM_SET(RI, ENABLE)|BK_PERM_SET(UUID_LEN, UUID_16), 128},

	//  Level Characteristic - Client Characteristic Configuration Descriptor

	[TEST_IDX_FF03_VAL_NTF_CFG] = {BK_ATT_DESC_CLIENT_CHAR_CFG_128, BK_PERM_SET(RD, ENABLE)|BK_PERM_SET(WRITE_REQ, ENABLE), 0, 0},
};
#endif//(CFG_BLE_VERSION == BLE_VERSION_5_1) || (CFG_BLE_VERSION == BLE_VERSION_4_2)

ble_err_t bk_ble_init(void)
{
    ble_err_t status = ERR_SUCCESS;

    struct bk_ble_db_cfg ble_db_cfg;

    ble_db_cfg.att_db = test_att_db;
    ble_db_cfg.att_db_nb = TEST_IDX_NB;
    ble_db_cfg.prf_task_id = 0;
    ble_db_cfg.start_hdl = 0;
    ble_db_cfg.svc_perm = BK_PERM_SET(SVC_UUID_LEN, UUID_16);
    memcpy(&(ble_db_cfg.uuid[0]), &test_svc_uuid[0], 16);

    status = bk_ble_create_db(&ble_db_cfg);

    return status;
}

void cli_ble_write_callback(write_req_t *write_req)
{
    bk_printf("write_cb[prf_id:%d, att_idx:%d, len:%d]\r\n", write_req->prf_id, write_req->att_idx, write_req->len);
    for(int i = 0; i < write_req->len; i++)
    {
        bk_printf("0x%x ", write_req->value[i]);
    }
    bk_printf("\r\n");
}

uint8_t cli_ble_read_callback(read_req_t *read_req)
{
    bk_printf("read_cb[prf_id:%d, att_idx:%d]\r\n", read_req->prf_id, read_req->att_idx);
    read_req->value[0] = 0x10;
    read_req->value[1] = 0x20;
    read_req->value[2] = 0x30;
    return 3;
}

#if (CFG_BLE_VERSION == BLE_VERSION_4_2)
void appm_adv_data_decode(uint8_t len, const uint8_t *data)
{
    uint8_t index;
    uint8_t i;
    for(index = 0; index < len;)
    {
        switch(data[index + 1])
        {
            case 0x01:
            {
                bk_printf("AD_TYPE : ");
                for(i = 0; i < data[index] - 1; i++)
                {
                    bk_printf("%02x ",data[index + 2 + i]);
                }
                bk_printf("\r\n");
                index +=(data[index] + 1);
            }
            break;
            case 0x08:
            case 0x09:
            {
                bk_printf("ADV_NAME : ");
                for(i = 0; i < data[index] - 1; i++)
                {
                    bk_printf("%c",data[index + 2 + i]);
                }
                bk_printf("\r\n");
                index +=(data[index] + 1);
            }
            break;
            case 0x02:
            {
                bk_printf("UUID : ");
                for(i = 0; i < data[index] - 1;)
                {
                    bk_printf("%02x%02x  ",data[index + 2 + i],data[index + 3 + i]);
                    i+=2;
                }
                bk_printf("\r\n");
                index +=(data[index] + 1);
            }
            break;
            default:
            {
                index +=(data[index] + 1);
            }
            break;
        }
    }
    return ;
}

void cli_ble_event_callback(ble_event_t event, void *param)
{
    switch(event)
    {
        case BLE_STACK_OK:
        {
            os_printf("STACK INIT OK\r\n");
            bk_ble_init();
        }
        break;
        case BLE_STACK_FAIL:
        {
            os_printf("STACK INIT FAIL\r\n");
        }
        break;
        case BLE_CONNECT:
        {
            os_printf("BLE CONNECT\r\n");
        }
        break;
        case BLE_DISCONNECT:
        {
            os_printf("BLE DISCONNECT\r\n");
        }
        break;
        case BLE_MTU_CHANGE:
        {
            os_printf("BLE_MTU_CHANGE:%d\r\n", *(uint16_t *)param);
        }
        break;
        case BLE_TX_DONE:
        {
            os_printf("BLE_TX_DONE\r\n");
        }
        break;
        case BLE_GEN_DH_KEY:
        {
            os_printf("BLE_GEN_DH_KEY\r\n");
            os_printf("key_len:%d\r\n", ((struct ble_gen_dh_key_ind *)param)->len);
            for(int i = 0; i < ((struct ble_gen_dh_key_ind *)param)->len; i++)
            {
                os_printf("%02x ", ((struct ble_gen_dh_key_ind *)param)->result[i]);
            }
            os_printf("\r\n");
        }
        break;
        case BLE_GET_KEY:
        {
            os_printf("BLE_GET_KEY\r\n");
            os_printf("pub_x_len:%d\r\n", ((struct ble_get_key_ind *)param)->pub_x_len);
            for(int i = 0; i < ((struct ble_get_key_ind *)param)->pub_x_len; i++)
            {
                os_printf("%02x ", ((struct ble_get_key_ind *)param)->pub_key_x[i]);
            }
            os_printf("\r\n");
        }
        break;
        case BLE_CREATE_DB_OK:
        {
            os_printf("CREATE DB SUCCESS\r\n");
        }
        break;
        case BLE_ATT_INFO_REQ:
        {
            att_info_req_t *a_ind = (att_info_req_t *)param;
            bk_printf("BLE_ATT_INFO_REQ\r\n");
            a_ind->length = 128;
            a_ind->status = ERR_SUCCESS;
        }
        break;
        default:
            os_printf("UNKNOW EVENT\r\n");
        break;
    }
}

#if (CFG_SUPPORT_BLE_MESH)
void ble_mesh_event_callback(ble_mesh_event_t event, void *param)
{
    switch(event)
    {
        case BLE_MESH_CREATE_DB_OK:
        {
            os_printf("MESH CREATE DB SUCCESS\r\n");
			app_ai_lights_models_init(0);
			app_store_mesh_info();
        }
        break;

        case BLE_MESH_INIT_DONE:
        {
            os_printf("BLE_MESH_INIT_DONE\r\n");
        }
        break;

        case BLE_MESH_CREATE_DB_FAIL:
        {
            os_printf("MESH CREATE DB FAILED\r\n");
        }
        break;

        case BLE_MESH_APP_KEY_ADD_DONE:
        {
        	os_printf("BLE_MESH_APP_KEY_ADD_DONE\r\n");
        }
        break;

        case BLE_MESH_PROV_INVITE:
        {
        	os_printf("BLE_MESH_PROV_INVITE\r\n");
        }
        break;

        case BLE_MESH_PROV_START:
        {
        	os_printf("BLE_MESH_PROV_START\r\n");
        }
        break;

        case BLE_MESH_PROV_DONE:
        {
        	os_printf("BLE_MESH_PROV_DONE\r\n");
        }
        break;

        default:
        {
            os_printf("unknown mesh event\r\n");
        }
    }
}
#endif

static void ble_command_usage(void)
{
    os_printf("ble help           - Help information\r\n");
    os_printf("ble active         - Active ble to with BK7231BTxxx\r\n");
	os_printf("ble start_adv      - Start advertising as a slave device\r\n");
	os_printf("ble stop_adv       - Stop advertising as a slave device\r\n");
    os_printf("ble notify prf_id att_id value\r\n");
    os_printf("                   - Send ntf value to master\r\n");
    os_printf("ble indicate prf_id att_id value\r\n");
    os_printf("                   - Send ind value to master\r\n");

    os_printf("ble disc           - Disconnect\r\n");
}

typedef adv_info_t ble_adv_param_t;

static void ble_advertise(void)
{
    UINT8 mac[6];
    char ble_name[20];
    UINT8 adv_idx, adv_name_len;

    wifi_get_mac_address((char *)mac, CONFIG_ROLE_STA);
    adv_name_len = snprintf(ble_name, sizeof(ble_name), "bk72xx-%02x%02x", mac[4], mac[5]);

    memset(&adv_info, 0x00, sizeof(adv_info));

    adv_info.channel_map = 7;
    adv_info.interval_min = 160;
    adv_info.interval_max = 160;

    adv_idx = 0;
    adv_info.advData[adv_idx] = 0x02; adv_idx++;
    adv_info.advData[adv_idx] = 0x01; adv_idx++;
    adv_info.advData[adv_idx] = 0x06; adv_idx++;

    adv_info.advData[adv_idx] = adv_name_len + 1; adv_idx +=1;
    adv_info.advData[adv_idx] = 0x09; adv_idx +=1; //name
    memcpy(&adv_info.advData[adv_idx], ble_name, adv_name_len); adv_idx +=adv_name_len;

    adv_info.advDataLen = adv_idx;

    adv_idx = 0;

    adv_info.respData[adv_idx] = adv_name_len + 1; adv_idx +=1;
    adv_info.respData[adv_idx] = 0x08; adv_idx +=1; //name
    memcpy(&adv_info.respData[adv_idx], ble_name, adv_name_len); adv_idx +=adv_name_len;
    adv_info.respDataLen = adv_idx;

    if (ERR_SUCCESS != appm_start_advertising())
    {
        os_printf("ERROR\r\n");
    }
}

static void ble_command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    if ((argc < 2) || (os_strcmp(argv[1], "help") == 0))
    {
        ble_command_usage();
        return;
    }

    if (os_strcmp(argv[1], "active") == 0)
    {
        ble_set_write_cb(cli_ble_write_callback);
        ble_set_read_cb(cli_ble_read_callback);
        ble_set_event_cb(cli_ble_event_callback);
#if (CFG_SUPPORT_BLE_MESH)
        ble_set_mesh_event_cb(ble_mesh_event_callback);
#endif
        ble_activate(NULL);
    }
	else if(os_strcmp(argv[1], "start_adv") == 0)
    {
        ble_advertise();
    }
#if (CFG_SUPPORT_BLE_MESH)
    else if(os_strcmp(argv[1], "mesh_start") == 0)
    {
        bk_ble_mesh_create_db();
    }
    else if(os_strcmp(argv[1], "start_unpb") == 0)
    {
        app_mesh_start_unpb_adv();
    }
    else if(os_strcmp(argv[1], "erase") == 0)
    {
        os_printf("erase nvds and reboot!!\r\n");
        ble_flash_erase(0,(uint32_t)0x1F4000, (4 * 1024), NULL);
        bk_reboot();
    }
#endif
    else if(os_strcmp(argv[1], "stop_adv") == 0)
    {
        if(ERR_SUCCESS != appm_stop_advertising())
        {
            os_printf("ERROR\r\n");
        }
    }
    else if(os_strcmp(argv[1], "notify") == 0)
    {
        uint8 len;
        uint16 prf_id;
        uint16 att_id;
        uint8 write_buffer[20];

        if(argc != 5)
        {
            ble_command_usage();
            return;
        }

        len = os_strlen(argv[4]);
        if ((len % 2 != 0) || (len > 40)) {
            os_printf("notify buffer len error\r\n");
            return;
        }
        hexstr2bin(argv[4], write_buffer, len/2);

        prf_id = atoi(argv[2]);
        att_id = atoi(argv[3]);

        if(ERR_SUCCESS != bk_ble_send_ntf_value(len / 2, write_buffer, prf_id, att_id))
        {
            os_printf("ERROR\r\n");
        }
    }
    else if(os_strcmp(argv[1], "indicate") == 0)
    {
        uint8 len;
        uint16 prf_id;
        uint16 att_id;
        uint8 write_buffer[20];

        if(argc != 5)
        {
            ble_command_usage();
            return;
        }

        len = os_strlen(argv[4]);
        if ((len % 2 != 0) || (len > 40)) {
            os_printf("indicate buffer len error\r\n");
            return;
        }
        hexstr2bin(argv[4], write_buffer, len/2);

        prf_id = atoi(argv[2]);
        att_id = atoi(argv[3]);

        if(ERR_SUCCESS != bk_ble_send_ind_value(len / 2, write_buffer, prf_id, att_id))
        {
            os_printf("ERROR\r\n");
        }
    }
    else if(os_strcmp(argv[1], "disc") == 0)
    {
#if (CFG_BLE_VERSION == BLE_VERSION_4_2)
        appm_disconnect();
#elif (CFG_BLE_VERSION == BLE_VERSION_5_1)
        appm_disconnect(0x13);
#endif
    }
	else if (os_strcmp(argv[1], "dut") == 0)
    {
		ble_dut_start();
    }
}
#elif (CFG_BLE_VERSION == BLE_VERSION_5_1) || (CFG_BLE_VERSION == BLE_VERSION_5_2)
#include "app_ble.h"
#include "app_sdp.h"
#include "app_ble_init.h"
#if (BLE_APP_SEC)
#include "app_sec.h"
#endif

void ble_notice_cb(ble_notice_t notice, void *param)
{
	switch (notice) {
	case BLE_5_STACK_OK:
		bk_printf("ble stack ok");
		break;
	case BLE_5_WRITE_EVENT:
	{
		write_req_t *w_req = (write_req_t *)param;
		bk_printf("write_cb:conn_idx:%d, prf_id:%d, add_id:%d, len:%d, data[0]:%02x\r\n",
			w_req->conn_idx, w_req->prf_id, w_req->att_idx, w_req->len, w_req->value[0]);
		break;
	}
	case BLE_5_READ_EVENT:
	{
		read_req_t *r_req = (read_req_t *)param;
		bk_printf("read_cb:conn_idx:%d, prf_id:%d, add_id:%d\r\n",
			r_req->conn_idx, r_req->prf_id, r_req->att_idx);
		r_req->value[0] = 0x12;
		r_req->value[1] = 0x34;
		r_req->value[2] = 0x56;
		r_req->length = 3;
		break;
	}
	case BLE_5_REPORT_ADV:
	{
		recv_adv_t *r_ind = (recv_adv_t *)param;

		bk_printf("[%s]r_ind:actv_idx:%d,evt_type:%d adv_addr:%02x:%02x:%02x:%02x:%02x:%02x,rssi:%d\r\n",
			((r_ind->evt_type&0x7) == 3)?"scan-rsp":((r_ind->evt_type&0x7) == 1)?"adv":"unknow",
			r_ind->actv_idx,r_ind->evt_type, r_ind->adv_addr[0], r_ind->adv_addr[1], r_ind->adv_addr[2],
			r_ind->adv_addr[3], r_ind->adv_addr[4], r_ind->adv_addr[5],r_ind->rssi);
		break;
	}
	case BLE_5_MTU_CHANGE:
	{
		mtu_change_t *m_ind = (mtu_change_t *)param;
		bk_printf("BLE_5_MTU_CHANGE:conn_idx:%d, mtu_size:%d\r\n", m_ind->conn_idx, m_ind->mtu_size);
		break;
	}
	case BLE_5_CONNECT_EVENT:
	{
		conn_ind_t *c_ind = (conn_ind_t *)param;
		bk_printf("c_ind:conn_idx:%d, addr_type:%d, peer_addr:%02x:%02x:%02x:%02x:%02x:%02x\r\n",
			c_ind->conn_idx, c_ind->peer_addr_type, c_ind->peer_addr[0], c_ind->peer_addr[1],
			c_ind->peer_addr[2], c_ind->peer_addr[3], c_ind->peer_addr[4], c_ind->peer_addr[5]);
		break;
	}
	case BLE_5_DISCONNECT_EVENT:
	{
		discon_ind_t *d_ind = (discon_ind_t *)param;
		bk_printf("d_ind:conn_idx:%d,reason:%d\r\n", d_ind->conn_idx,d_ind->reason);
		break;
	}
	case BLE_5_ATT_INFO_REQ:
	{
		att_info_req_t *a_ind = (att_info_req_t *)param;
		bk_printf("a_ind:conn_idx:%d\r\n", a_ind->conn_idx);
		a_ind->length = 128;
		a_ind->status = ERR_SUCCESS;
		break;
	}
	case BLE_5_CREATE_DB:
	{
		create_db_t *cd_ind = (create_db_t *)param;
		bk_printf("cd_ind:prf_id:%d, status:%d\r\n", cd_ind->prf_id, cd_ind->status);
		break;
	}
	#if (BLE_CENTRAL)
	case BLE_5_INIT_CONNECT_EVENT:
	{
		conn_ind_t *c_ind = (conn_ind_t *)param;
		#if (CFG_SOC_NAME == SOC_BK7238)
		app_ble_get_peer_feature(c_ind->conn_idx);
		app_ble_set_le_pkt_size(c_ind->conn_idx,LE_MAX_OCTETS);
		app_ble_mtu_exchange(c_ind->conn_idx);
		sdp_discover_all_service(c_ind->conn_idx);
		#endif
		bk_printf("BLE_5_INIT_CONNECT_EVENT:conn_idx:%d, addr_type:%d, peer_addr:%02x:%02x:%02x:%02x:%02x:%02x\r\n",
			c_ind->conn_idx, c_ind->peer_addr_type, c_ind->peer_addr[0], c_ind->peer_addr[1],
			c_ind->peer_addr[2], c_ind->peer_addr[3], c_ind->peer_addr[4], c_ind->peer_addr[5]);
		break;
	}
	case BLE_5_INIT_DISCONNECT_EVENT:
	{
		discon_ind_t *d_ind = (discon_ind_t *)param;
		bk_printf("BLE_5_INIT_DISCONNECT_EVENT:conn_idx:%d,reason:0x%x\r\n", d_ind->conn_idx,d_ind->reason);
		break;
	}
	#endif
	case BLE_5_INIT_CONN_PARAM_UPDATE_REQ_EVENT:
	{
		conn_param_t *d_ind = (conn_param_t *)param;
		bk_printf("BLE_5_INIT_CONN_PARAM_UPDATE_REQ_EVENT:conn_idx:%d,intv_min:%d,intv_max:%d,time_out:%d\r\n",d_ind->conn_idx,
			d_ind->intv_min,d_ind->intv_max,d_ind->time_out);
		app_ble_send_conn_param_update_cfm(d_ind->conn_idx,true);
	}break;
	case BLE_5_INIT_CONN_PARAM_UPDATE_IND_EVENT:
	{
		conn_update_ind_t *d_ind = (conn_update_ind_t *)param;
		bk_printf("BLE_5_INIT_CONN_PARAM_UPDATE_IND_EVENT:conn_idx:%d,interval:%d,time_out:%d,latency\r\n",d_ind->conn_idx,
			d_ind->interval,d_ind->time_out,d_ind->latency);
	}break;
	case BLE_5_SDP_REGISTER_FAILED:
		bk_printf("BLE_5_SDP_REGISTER_FAILED\r\n");
		break;
	default:
		break;
	}
}

void ble_cmd_cb(ble_cmd_t cmd, ble_cmd_param_t *param)
{
	bk_printf("cmd:%d idx:%d status:%d\r\n", cmd, param->cmd_idx, param->status);
}
#if BLE_CENTRAL && (CFG_SOC_NAME == SOC_BK7231N)
static void ble_app_sdp_characteristic_cb(unsigned char conidx,uint16_t chars_val_hdl,unsigned char uuid_len,unsigned char *uuid)
{
	bk_printf("[APP]characteristic conidx:%d,handle:0x%02x(%d),UUID:0x",conidx,chars_val_hdl,chars_val_hdl);
	for(int i = 0; i< uuid_len; i++)
	{
		bk_printf("%02x ",uuid[i]);
	}
	bk_printf("\r\n");
}

void app_sdp_charac_cb(CHAR_TYPE type,uint8 conidx,uint16_t hdl,uint16_t len,uint8 *data)
{
	bk_printf("[APP]type:%x conidx:%d,handle:0x%02x(%d),len:%d,0x",type,conidx,hdl,hdl,len);
	for(int i = 0; i< len; i++)
	{
		bk_printf("%02x ",data[i]);
	}
	bk_printf("\r\n");
}
static const app_sdp_service_uuid service_tab[] ={
	{
		.uuid_len = 0x02,
		.uuid[0] = 0x00,
		.uuid[1] = 0x18,
	},
	{
		.uuid_len = 0x02,
		.uuid[0] = 0x01,
		.uuid[1] = 0x18,
	},
};

#elif (CFG_SOC_NAME == SOC_BK7238) && (BLE_GATT_CLI)
void sdp_event_cb(sdp_notice_t notice, void *param)
{
	switch (notice) {
		case SDP_CHARAC_NOTIFY_EVENT:
			{
				sdp_event_t *g_sdp = (sdp_event_t *)param;
				bk_printf("[SDP_CHARAC_NOTIFY_EVENT]con_idx:%d,hdl:0x%x,value_length:%d\r\n",g_sdp->con_idx,g_sdp->hdl,g_sdp->value_length);
			}
			break;
		case SDP_CHARAC_INDICATE_EVENT:
			{
				sdp_event_t *g_sdp = (sdp_event_t *)param;
				bk_printf("[SDP_CHARAC_INDICATE_EVENT]con_idx:%d,hdl:0x%x,value_length:%d\r\n",g_sdp->con_idx,g_sdp->hdl,g_sdp->value_length);
			}
			break;
		case SDP_CHARAC_READ:
			{
				sdp_event_t *g_sdp = (sdp_event_t *)param;
				bk_printf("[SDP_CHARAC_READ]con_idx:%d,hdl:0x%x,value_length:%d\r\n",g_sdp->con_idx,g_sdp->hdl,g_sdp->value_length);
			}
			break;
		case SDP_DISCOVER_SVR_DONE:
			{
				bk_printf("[SDP_DISCOVER_SVR_DONE]\r\n");
			}
			break;
		case SDP_CHARAC_WRITE_DONE:
			{
				bk_printf("[SDP_CHARAC_WRITE_DONE]\r\n");
			}
			break;
		default:
			bk_printf("[%s]Event:%d\r\n",__func__,notice);
			break;
	}
}
#endif
#define BLE_VSN5_DEFAULT_MASTER_IDX      0
#if BLE_BATT_SERVER
#include "app_bass.h"
#elif BLE_HID_DEVICE
#include "app_hogpd.h"
#elif BLE_FINDME_TARGET
#include "app_findt.h"
#elif BLE_DIS_SERVER
#include "app_diss.h"
#endif
#if (BLE_BATT_SERVER | BLE_HID_DEVICE | BLE_FINDME_TARGET | BLE_DIS_SERVER)
void profile_notice_cb(ble_notice_t notice, void *param)
{
	switch (notice) {
	case BLE_5_STACK_OK:
	{
		bk_printf("ble stack ok");
		break;
	}
	case BLE_5_CONNECT_EVENT:
	{
		conn_ind_t *c_ind = (conn_ind_t *)param;
		bk_printf("c_ind:conn_idx:%d, addr_type:%d, peer_addr:%02x:%02x:%02x:%02x:%02x:%02x\r\n",
			c_ind->conn_idx, c_ind->peer_addr_type, c_ind->peer_addr[0], c_ind->peer_addr[1],
			c_ind->peer_addr[2], c_ind->peer_addr[3], c_ind->peer_addr[4], c_ind->peer_addr[5]);
		break;
	}
	case BLE_5_DISCONNECT_EVENT:
	{
		discon_ind_t *d_ind = (discon_ind_t *)param;
		bk_printf("d_ind:conn_idx:%d,reason:%d\r\n", d_ind->conn_idx,d_ind->reason);
		break;
	}
	default:
		break;
	}
}
#endif

#if BLE_APP_SEC
void security_notice_cb(sec_notice_t notice, void *param)
{
	switch (notice) {
		case APP_SEC_PAIRING_SUCCEED:
		{
			bk_printf("BLE PAIRING SUCCEED\r\n");
			break;
		}
		case APP_SEC_PAIRING_FAILED:
		{
			bk_printf("[WARNING]BLE PAIRING FAILED\r\n");
			break;
		}
		default:
			break;
	}
}
#endif

static void ble_command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	uint8_t adv_data[31];
	uint8_t actv_idx;

	if(os_strcmp(argv[1],"pta")==0){
        uint32_t enable = os_strtoul(argv[2], NULL, 10);
		ble_coex_set_pta(enable ? true : false);
	}

	if(os_strcmp(argv[1],"exit")==0){
		ble_thread_exit();
	}
	if(os_strcmp(argv[1],"notify")==0){
		uint32_t len;
		uint16 prf_id;
		uint8 write_buffer[128]={0};
		len=os_strtoul(argv[3], NULL, 10);
		if(argc!=4){
			os_printf("notify arg %d error\r\n",argc);
			return;
		}else{
			if(len>128 || len<4){
				os_printf("The length of the notify should be between 4 and 128 \r\n");
				if(len<4){
					os_printf("Output 4 bytes\r\n");
					len=4;
				}else{
					os_printf("Output the first 128 bytes\r\n");
					len=128;
				}
			}
			for(int i=0;i<4;i++){
				write_buffer[i]=rand()%257;
			}
		}
		prf_id=atoi(argv[2]);
		if(ERR_SUCCESS!=bk_ble_send_ntf_value(len, write_buffer,prf_id,TEST_IDX_FF03_VAL_VALUE)){
			os_printf("ERROR\r\n");
		}
	}
	if(os_strcmp(argv[1],"indicate")==0){
		uint16 prf_id;
		uint32_t len;
		uint8 write_buffer[128]={0};
		len=os_strtoul(argv[3], NULL, 10);
		if(argc!=4){
			os_printf("indicate arg %d error\r\n",argc);
			return;
		}else{
			if(len>128 || len<4){
				os_printf("The length of the indicate should be between 4 and 128 \r\n");
				if(len<4){
					os_printf("Output 4 bytes\r\n");
					len=4;
				}else{
					os_printf("Output the first 128 bytes\r\n");
					len=128;
				}
			}
			for(int i=0;i<4;i++){
				write_buffer[i]=rand()%257;
			}
		}
		prf_id=atoi(argv[2]);
		if(ERR_SUCCESS!=bk_ble_send_ind_value(len, write_buffer,prf_id,TEST_IDX_FF02_VAL_VALUE)){
			os_printf("ERROR\r\n");
		}
	}
	if (os_strcmp(argv[1], "dut") == 0) {
		char *const txevm_exit[3] = {"txevm", "-e", "0"};
		do_evm(NULL, 0, 3, txevm_exit);
		ble_dut_start();
	}
	if (os_strcmp(argv[1], "active") == 0) {
		ble_set_notice_cb(ble_notice_cb);
		bk_ble_init();
	}
	#if (BLE_BATT_SERVER) && (CFG_SOC_NAME == SOC_BK7238)
	if(os_strcmp(argv[1], "bass_init") == 0) {
		ble_set_notice_cb(profile_notice_cb);
		bk_bass_init();
	}
	if(os_strcmp(argv[1], "bass_ntf") == 0){
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		bk_bass_ntf_send(os_strtoul(argv[2], NULL, 10));
	}
	if(os_strcmp(argv[1], "bass_enable") == 0){
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		bk_bass_enable(os_strtoul(argv[2], NULL, 10));
	}
	#endif
	#if (BLE_HID_DEVICE) && (CFG_SOC_NAME == SOC_BK7238)
	if (os_strcmp(argv[1], "hogpd_init") == 0) {
		ble_set_notice_cb(profile_notice_cb);
		bk_hogpd_init();
	}
	if (os_strcmp(argv[1], "hogpd_enable") == 0) {
		bk_hogpd_enable();
	}
	#endif
	#if (BLE_FINDME_TARGET) && (CFG_SOC_NAME == SOC_BK7238)
	if (os_strcmp(argv[1], "findt_init") == 0) {
		ble_set_notice_cb(profile_notice_cb);
		bk_findt_init();
	}
	#endif
	#if (BLE_DIS_SERVER) && (CFG_SOC_NAME == SOC_BK7238)
	if (os_strcmp(argv[1], "diss_init") == 0) {
		ble_set_notice_cb(profile_notice_cb);
		bk_diss_init();
	}
	if (os_strcmp(argv[1], "diss_set") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		if(os_strtoul(argv[2], NULL, 10) == 0){
			uint8_t set_data[5];
			set_data[0]=0x62;//b
			set_data[1]=0x65;//e
			set_data[2]=0x6b;//k
			set_data[3]=0x65;//e
			set_data[4]=0x6e;//n
			bk_diss_set(os_strtoul(argv[2], NULL, 10), 5, set_data);
		}else if(os_strtoul(argv[2], NULL, 10) == 1){
			uint8_t set_data[6];
			set_data[0]=0x62;//b
			set_data[1]=0x6b;//k
			set_data[2]=0x37;//7
			set_data[3]=0x32;//2
			set_data[4]=0x33;//3
			set_data[5]=0x38;//8
			bk_diss_set(os_strtoul(argv[2], NULL, 10), 6, set_data);
		}else if(os_strtoul(argv[2], NULL, 10) == 6){
			uint8_t set_data[8];
			set_data[0]=0xdd;//mac
			set_data[1]=0xa3;
			set_data[2]=0xca;
			set_data[3]=0x05;//beken
			set_data[4]=0xf0;
			set_data[5]=0x8c;
			set_data[6]=0x47;
			set_data[7]=0xc7;
			bk_diss_set(os_strtoul(argv[2], NULL, 10),8, set_data);
		}else if(os_strtoul(argv[2], NULL, 10) == 7){
			uint8_t set_data[6];
			set_data[0]=0x01;
			set_data[1]=0x02;
			set_data[2]=0x03;
			set_data[3]=0x04;
			set_data[4]=0x05;
			set_data[5]=0x06;
			bk_diss_set(os_strtoul(argv[2], NULL, 10), 6, set_data);
		}else if(os_strtoul(argv[2], NULL, 10) == 8){
			uint8_t set_data[7];
			set_data[0]=0x01;//Blutooth SIG company
			set_data[1]=0xf0;//beken
			set_data[2]=0x05;
			set_data[3]=0x01;//product id
			set_data[4]=0x00;
			set_data[5]=0x02;//product version
			set_data[6]=0x00;
			bk_diss_set(os_strtoul(argv[2], NULL, 10), 7, set_data);
		}else{
			uint8_t set_data[3];
			set_data[0]=0x31;
			set_data[1]=0x32;
			set_data[2]=0x33;
			bk_diss_set(os_strtoul(argv[2], NULL, 10), 3, set_data);
		}
	}
	#endif
	if (os_strcmp(argv[1], "create_adv") == 0) {
		actv_idx = app_ble_get_idle_actv_idx_handle();
		bk_ble_create_advertising(actv_idx, 7, 160, 160, ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "create_ext_adv") == 0) {
		if (argc < 4){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		actv_idx = app_ble_get_idle_actv_idx_handle();
		bk_ble_create_extended_advertising(actv_idx, 7, 160, 160, /*scannable*/os_strtoul(argv[2], NULL, 10), /*connectable*/os_strtoul(argv[3], NULL, 10), ble_cmd_cb);
	}
	#if (CFG_SOC_NAME == SOC_BK7231N)
	if (os_strcmp(argv[1], "set_adv_data") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		adv_data[0] = 0x02;
		adv_data[1] = 0x01;
		adv_data[2] = 0x06;
		adv_data[3] = 0x0B;
		adv_data[4] = 0x09;
		memcpy(&adv_data[5], "7231N_BLE", 10);
		bk_ble_set_adv_data(os_strtoul(argv[2], NULL, 10), adv_data, 0xF, ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "set_rsp_data") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		adv_data[0] = 0x07;
		adv_data[1] = 0x08;
		memcpy(&adv_data[2], "7231N", 6);
		bk_ble_set_scan_rsp_data(os_strtoul(argv[2], NULL, 10), adv_data, 0x8, ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "set_ext_adv_data") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		uint16_t data_len;
	#if (CFG_BLE_AUX_CHAIN)
		data_len = 140;
	#else
		data_len = 22;
	#endif
		//data_len = os_strtoul(argv[3], NULL, 10);
		uint8_t ext_adv_data[data_len];
		uint16_t i;

		ext_adv_data[0] = 0x02;
		ext_adv_data[1] = 0x01;
		ext_adv_data[2] = 0x06;
		ext_adv_data[3] = 0x0B;
		ext_adv_data[4] = 0x09;
		memcpy(&ext_adv_data[5], "7231N_EXT", 10);
		ext_adv_data[15] = data_len - 16;
		ext_adv_data[16] = 0xFF;
		ext_adv_data[17] = 0xF0;
		ext_adv_data[18] = 0x05;
		for (i = 0; i < data_len - 19; i++) {
			ext_adv_data[19 + i] = i;
		}
		bk_ble_set_ext_adv_data(os_strtoul(argv[2], NULL, 10), ext_adv_data, data_len, ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "set_ext_rsp_data") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		uint16_t data_len;
	#if (CFG_BLE_AUX_CHAIN)
		data_len = 260;
	#else
		data_len = 22;
	#endif
		//data_len = os_strtoul(argv[3], NULL, 10);
		uint8_t ext_adv_data[data_len];
		uint16_t i;

		ext_adv_data[0] = 0x0B;
		ext_adv_data[1] = 0x09;
		memcpy(&ext_adv_data[2], "7231N_EXT", 10);
		ext_adv_data[12] = data_len - 13;
		ext_adv_data[13] = 0xFF;
		ext_adv_data[14] = 0xF0;
		ext_adv_data[15] = 0x05;
		for (i = 0; i < data_len - 16; i++) {
			ext_adv_data[16 + i] = i;
		}
		bk_ble_set_ext_scan_rsp_data(os_strtoul(argv[2], NULL, 10), ext_adv_data, data_len, ble_cmd_cb);
	}

	#elif (CFG_SOC_NAME == SOC_BK7238)
	/*note:AD type flags already added to adv data,not be set by application*/
	if (os_strcmp(argv[1], "set_adv_data") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		adv_data[0] = 0x0A;
		adv_data[1] = 0x09;
		memcpy(&adv_data[2], "7238_BLE", 9);
		bk_ble_set_adv_data(os_strtoul(argv[2], NULL, 10), adv_data, 0xB, ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "set_rsp_data") == 0) {
		uint8_t adv_data_len;
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		adv_data[0] = 0x06;
		adv_data[1] = 0x08;
		memcpy(&adv_data[2], "7238", 5);
		#if (BLE_HID_DEVICE)
		adv_data[7] = 0x03;
		adv_data[8] = 0x03;
		adv_data[9] = 0x12;
		adv_data[10] = 0x18;
		adv_data_len = 0xB;
		#else
		adv_data_len = 0x7;
		#endif
		bk_ble_set_scan_rsp_data(os_strtoul(argv[2], NULL, 10), adv_data, adv_data_len, ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "set_ext_adv_data") == 0) {
		uint16_t data_len;
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		#if (CFG_BLE_AUX_CHAIN)
		data_len = 140;		//189frag
		#else
		data_len = 18;
		#endif

		//data_len = os_strtoul(argv[3], NULL, 10);
		uint8_t ext_adv_data[data_len];
		uint16_t i;

		ext_adv_data[0] = 0x0B;
		ext_adv_data[1] = 0x09;
		memcpy(&ext_adv_data[2], "BK7238EXT", 10);
		ext_adv_data[12] = data_len - 13;
		ext_adv_data[13] = 0xFF;
		ext_adv_data[14] = 0xF0;
		ext_adv_data[15] = 0x05;
		for (i = 0; i < data_len - 16; i++) {
			ext_adv_data[16 + i] = i;
		}
		bk_ble_set_ext_adv_data(os_strtoul(argv[2], NULL, 10), ext_adv_data, data_len, ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "set_ext_rsp_data") == 0) {
		uint16_t data_len;
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		#if (CFG_BLE_AUX_CHAIN)
		data_len = 260;
		#else
		data_len = 25;
		#endif

		//data_len = os_strtoul(argv[3], NULL, 10);
		uint8_t ext_adv_data[data_len];
		uint16_t i;

		ext_adv_data[0] = 0x11;
		ext_adv_data[1] = 0x09;
		memcpy(&ext_adv_data[2], "BK7238-SCAN-EXT", 16);
		ext_adv_data[18] = data_len - 19;
		ext_adv_data[19] = 0xFF;
		ext_adv_data[20] = 0xF0;
		ext_adv_data[21] = 0x05;
		for (i = 0; i < data_len - 22; i++) {
			ext_adv_data[22 + i] = i;
		}
		bk_ble_set_ext_scan_rsp_data(os_strtoul(argv[2], NULL, 10), ext_adv_data, data_len, ble_cmd_cb);
	}

	if (os_strcmp(argv[1], "rssi") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		app_ble_get_con_rssi(os_strtoul(argv[2], NULL, 10));
	}
	#endif
	if (os_strcmp(argv[1], "start_adv") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		bk_ble_start_advertising(os_strtoul(argv[2], NULL, 10), 0, ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "stop_adv") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		bk_ble_stop_advertising(os_strtoul(argv[2], NULL, 10), ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "delete_adv") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		bk_ble_delete_advertising(os_strtoul(argv[2], NULL, 10), ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "create_scan") == 0) {
		actv_idx = app_ble_get_idle_actv_idx_handle();
		bk_ble_create_scaning(actv_idx, ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "start_scan") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		bk_ble_start_scaning(os_strtoul(argv[2], NULL, 10), 100, 30, ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "stop_scan") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		bk_ble_stop_scaning(os_strtoul(argv[2], NULL, 10), ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "delete_scan") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		bk_ble_delete_scaning(os_strtoul(argv[2], NULL, 10), ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "update_conn") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		bk_ble_update_param(os_strtoul(argv[2], NULL, 10), 50, 50, 0, 800, ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "dis_conn") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		bk_ble_disconnect(os_strtoul(argv[2], NULL, 10), ble_cmd_cb);
	}
	#if (CFG_SOC_NAME == SOC_BK7231N)
	/*note:bk7238 auto mtu exchange by gatt_cli,not be call it by application*/
	if (os_strcmp(argv[1], "mtu_change") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		bk_ble_gatt_mtu_change(os_strtoul(argv[2], NULL, 10), ble_cmd_cb);
	}
	#endif // (CFG_SOC_NAME == SOC_BK7231N)
	if (os_strcmp(argv[1], "init_adv") == 0) {
		#if (CFG_SOC_NAME == SOC_BK7231N)
		struct adv_param adv_info;
		adv_info.channel_map = 7;
		adv_info.duration = 0;
		adv_info.prop = (1 << ADV_PROP_CONNECTABLE_POS) | (1 << ADV_PROP_SCANNABLE_POS);
		adv_info.interval_min = 160;
		adv_info.interval_max = 160;
		adv_info.advData[0] = 0x02;
		adv_info.advData[1] = 0x01;
		adv_info.advData[2] = 0x06;
		adv_info.advData[3] = 0x0B;
		adv_info.advData[4] = 0x09;
		memcpy(&adv_info.advData[5], "7231N_BLE", 10);
		adv_info.advDataLen = 0xF;
		adv_info.respData[0] = 0x07;
		adv_info.respData[1] = 0x08;
		memcpy(&adv_info.respData[2], "7231N", 6);
		adv_info.respDataLen = 0x8;
		#elif (CFG_SOC_NAME == SOC_BK7238)
		struct adv_param adv_info;
		adv_info.channel_map = 7;
		adv_info.duration = 0;
		adv_info.prop = (1 << ADV_PROP_CONNECTABLE_POS) | (1 << ADV_PROP_SCANNABLE_POS);
		adv_info.interval_min = 160;
		adv_info.interval_max = 160;
		adv_info.advData[0] = 0x09;
		adv_info.advData[1] = 0x09;
		memcpy(&adv_info.advData[2], "7238_BLE", 8);
		adv_info.advDataLen = 10;
		adv_info.respData[0] = 0x05;
		adv_info.respData[1] = 0x08;
		memcpy(&adv_info.respData[2], "7238", 4);
		adv_info.respDataLen = 6;
		#endif
		actv_idx = app_ble_get_idle_actv_idx_handle();
		bk_ble_adv_start(actv_idx, &adv_info, ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "deinit_adv") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		bk_ble_adv_stop(os_strtoul(argv[2], NULL, 10), ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "init_scan") == 0) {
		struct scan_param scan_info;
		scan_info.channel_map = 7;
		scan_info.interval = 100;
		scan_info.window = 30;
		actv_idx = app_ble_get_idle_actv_idx_handle();
		bk_ble_scan_start(actv_idx, &scan_info, ble_cmd_cb);
	}
	if (os_strcmp(argv[1], "deinit_scan") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		bk_ble_scan_stop(os_strtoul(argv[2], NULL, 10), ble_cmd_cb);
	}
	#if BLE_APP_SEC
	if (os_strcmp(argv[1], "smp_init") == 0) {
		struct app_pairing_cfg par;
		par.iocap = GAP_IO_CAP_DISPLAY_ONLY;
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		if (os_strtoul(argv[2], NULL, 10) == 0) {
			//legacy pairing
			par.sec_req = GAP_SEC1_AUTH_PAIR_ENC;
			par.auth = GAP_AUTH_REQ_MITM_BOND;
			par.ikey_dist = GAP_KDIST_ENCKEY | GAP_KDIST_LINKKEY | GAP_KDIST_IDKEY;
			par.rkey_dist = GAP_KDIST_ENCKEY | GAP_KDIST_LINKKEY;
		#if BLE_APP_SEC_CON
		} else if (os_strtoul(argv[2], NULL, 10) == 1) {
			// secure connection pairiing
			par.sec_req = GAP_SEC1_SEC_CON_PAIR_ENC;
			par.auth = GAP_AUTH_REQ_SEC_CON_BOND;
			par.ikey_dist = GAP_KDIST_IDKEY;
			par.rkey_dist = GAP_KDIST_NONE;
		#endif
		}
		app_sec_config(&par, security_notice_cb);
	}
	if (os_strcmp(argv[1], "sec_req") == 0) {
		if (argc < 3){
			bk_printf("\nThe number of param is wrong!\n");
			return;
		}
		app_sec_send_security_req(os_strtoul(argv[2], NULL, 10));
	}
	#endif
#if CFG_BLE_INIT_NUM
	if (os_strcmp(argv[1], "con_create") == 0)
	{
		ble_set_notice_cb(ble_notice_cb);
		#if BLE_SDP_CLIENT && (CFG_SOC_NAME == SOC_BK7231N)
		register_app_sdp_service_tab(sizeof(service_tab)/sizeof(app_sdp_service_uuid),(app_sdp_service_uuid *)service_tab);
		app_sdp_service_filtration(0);
		register_app_sdp_characteristic_callback(ble_app_sdp_characteristic_cb);
		register_app_sdp_charac_callback(app_sdp_charac_cb);
		#elif (CFG_SOC_NAME == SOC_BK7238)
		sdp_set_notice_cb(sdp_event_cb);
		#endif
		actv_idx = app_ble_get_idle_conn_idx_handle();
		bk_printf("------------->conn_idx:%d\r\n",actv_idx);
		bk_ble_create_init(actv_idx, 0, 0, 0,ble_cmd_cb);
	}
	else if ((os_strcmp(argv[1], "con_start") == 0) && (argc >= 5))
	{
		struct bd_addr bdaddr;
		unsigned char addr_type = ADDR_PUBLIC;
		int addr_type_str = atoi(argv[3]);
		int actv_idx_str = atoi(argv[4]);
		bk_printf("idx:%d,addr_type:%d\r\n",actv_idx_str,addr_type_str);
		if((addr_type_str > ADDR_RPA_OR_RAND)||(actv_idx_str >= 0xFF)){
			return;
		}
		actv_idx = actv_idx_str;
		hexstr2bin(argv[2], bdaddr.addr, GAP_BD_ADDR_LEN);
		addr_type = addr_type_str;
		bk_ble_init_set_connect_dev_addr(actv_idx,&bdaddr,addr_type);
		bk_ble_init_start_conn(actv_idx,ble_cmd_cb);
	}
	else if ((os_strcmp(argv[1], "con_stop") == 0) && (argc >= 3))
	{
		int actv_idx_str = atoi(argv[2]);
		bk_printf("idx:%d\r\n",actv_idx_str);
		if(actv_idx_str >= 0xFF){
			return;
		}
		actv_idx = actv_idx_str;
		bk_ble_init_stop_conn(actv_idx,ble_cmd_cb);
	}
	else if ((os_strcmp(argv[1], "con_dis") == 0) && (argc >= 3))
	{
		int actv_idx_str = atoi(argv[2]);
		bk_printf("idx:%d\r\n",actv_idx_str);
		if(actv_idx_str >= 0xFF){
			return;
		}
		actv_idx = actv_idx_str;
		app_ble_master_appm_disconnect(actv_idx);
	}
#if BLE_CENTRAL
	else if (os_strcmp(argv[1], "con_read") == 0)
	{
		if(argc < 4){
			bk_printf("param error\r\n");
			return;
		}
		int actv_idx_str = atoi(argv[3]);
		bk_printf("idx:%d\r\n",actv_idx_str);
		if(actv_idx_str >= 0xFF){
			return;
		}
		actv_idx = actv_idx_str;
		int handle = atoi(argv[2]);
		if(handle >=0 && handle <= 0xFFFF){
			bk_ble_read_service_data_by_handle_req(actv_idx,handle,ble_cmd_cb);
		}
		else{
			bk_printf("handle(%x) error\r\n",handle);
		}
	}
	else if (os_strcmp(argv[1], "con_write") == 0)
	{
		//cmd:ble con_write 24 0
		if(argc < 4){
			bk_printf("param error\r\n");
			return;
		}
		int handle = atoi(argv[2]);
		int actv_idx_str = atoi(argv[3]);
		bk_printf("idx:%d\r\n",actv_idx_str);
		if(actv_idx_str >= 0xFF){
			return;
		}
		actv_idx = actv_idx_str;
		unsigned char test_buf[4] = {0x01,0x02,0x22,0x32};
		if(handle >=0 && handle <= 0xFFFF){
			bk_ble_write_service_data_req(actv_idx,handle,4,test_buf,ble_cmd_cb);
		}else{
			bk_printf("handle(%x) error\r\n",handle);
		}
	}
	#if (CFG_SOC_NAME == SOC_BK7231N)
	else if (os_strcmp(argv[1], "con_rd_sv_ntf_int_cfg") == 0)
	{
		if(argc < 4){
			bk_printf("param error\r\n");
			return;
		}
		int actv_idx_str = atoi(argv[3]);
		bk_printf("idx:%d\r\n",actv_idx_str);
		if(actv_idx_str >= 0xFF){
			return;
		}
		actv_idx = actv_idx_str;
		int handle = atoi(argv[2]);
		if(handle >=0 && handle <= 0xFFFF){
			appm_read_service_ntf_ind_cfg_by_handle_req(actv_idx,handle);
		}else{
			bk_printf("handle(%x) error\r\n",handle);
		}
	}
	else if (os_strcmp(argv[1], "con_rd_sv_ud_cfg") == 0)
	{
		if(argc < 4){
			bk_printf("param error\r\n");
			return;
		}
		int actv_idx_str = atoi(argv[3]);
		bk_printf("idx:%d\r\n",actv_idx_str);
		if(actv_idx_str >= 0xFF){
			return;
		}
		actv_idx = actv_idx_str;
		int handle = atoi(argv[2]);
		if(handle >=0 && handle <= 0xFFFF){
			appm_read_service_userDesc_by_handle_req(actv_idx,handle);
		}else{
			bk_printf("handle(%x) error\r\n",handle);
		}
	}
	#endif //(CFG_SOC_NAME == SOC_BK7231N)
	else if(os_strcmp(argv[1], "svc_filt") == 0)
	{
		if(argc < 3){
			bk_printf("param error\r\n");
			return;
		}
		int en = atoi(argv[2]);
		bk_printf("svc_filt en:%d\r\n",en);
		app_sdp_service_filtration(en);
	}
#endif
#endif ///CFG_BLE_INIT_NUM
}
#endif
#endif

#if CFG_WIFI_SENSOR
#include "sensor.h"
void wifi_sensor_callback(int status)
{
    if(status)
        bk_printf("detected movement\r\n");
    else
        bk_printf("nothing detected\r\n");
}

static void wifi_sensor_command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    if ( argc != 2 )
    {
        bk_printf("start / stop\r\n");
    }

    if (os_strcmp(argv[1], "start") == 0)
    {
        if(bk_wifi_detect_movement_start(wifi_sensor_callback) != BK_WSD_OK)
            bk_printf("bk wifi sensor start failed\r\n");
    }
    else if (os_strcmp(argv[1], "stop") == 0)
    {
        bk_wifi_detect_movement_stop();
        bk_printf("bk wifi sensor stop\r\n");
    }
    else
    {
        bk_printf("not support\r\n");
    }
}
#endif

extern void cmd_rfcali_cfg_mode(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
extern void cmd_rfcali_cfg_rate_dist(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
extern void cmd_rfcali_cfg_tssi_g(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
extern void cmd_rfcali_cfg_tssi_b(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
extern void cmd_rfcali_show_data(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
extern void lowvol_Sleep_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);

static void wifi_mgmt_filter_cb(uint8_t *data, int len, void *info)
{
	if (!data) {
		bk_printf("null data\n");
		return;
	}

	uint16_t framectrl = co_read16(data);
	uint16_t frame_type_subtype = framectrl & MAC_FCTRL_TYPESUBTYPE_MASK;

	bk_printf("filter type=%x info=%u\n", frame_type_subtype, (uint32_t)info);
}

static void wifi_mgmt_filter_help(void)
{
	bk_printf("wifi_mgmt_filter 0(all)/1(probe req)/-1(stop)\n");
}

static void cmd_wifi_mgmt_filter(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	OSStatus ret = kGeneralErr;
	uint32_t filter = 0;

	if (argc != 2){
		wifi_mgmt_filter_help();
		return;
	}

	filter = os_strtoul(argv[1], NULL, 0);
	if (filter == -1) {
		ret = bk_wlan_reg_rx_mgmt_cb(NULL, 0);
	} else {
		ret = bk_wlan_reg_rx_mgmt_cb(wifi_mgmt_filter_cb, filter);
	}

	bk_printf("set filter ret=%x\n", ret);
}

#if CFG_WIFI_RAW_TX_CMD

typedef struct {
	uint32_t interval;
	uint32_t counter;
} wifi_raw_tx_param_t;

static void wifi_raw_tx_thread(void *arg)
{
	char frame[] = {
		0xB0, //version, type, subtype
		0x00, //frame control
		0x3A, 0x01, //duration
		0xC8, 0x47, 0x8C, 0x42, 0x00, 0x48, //Address1 - destination
		0x4C, 0xD1, 0xA1, 0xC5, 0x38, 0xE4, //Address2 - source
		0x4C, 0xD1, 0xA1, 0xC5, 0x38, 0xE4, //Address3 - bssid
		0x20, 0xC0, //sequence

		//Auth Response
		0x00, 0x00, //Auth algorithm - open system
		0x02, 0x00, //Auth seq num
		0x00, 0x00, //Status code
	};
	wifi_raw_tx_param_t *tx_param;
	int ret;

	tx_param = (wifi_raw_tx_param_t *)arg;
	os_printf("wifi raw tx begin, interval=%u counter=%d\n", tx_param->interval,
			tx_param->counter);

	for (uint32_t i = 0; i < tx_param->counter; i++) {
		ret = bk_wlan_send_80211_raw_frame((unsigned char*)frame, sizeof(frame));
		if (ret != kNoErr) {
			os_printf("raw tx error, ret=%d\n", ret);
		}

		rtos_delay_milliseconds(tx_param->interval);
	}

	os_free(arg);
	os_printf("wifi raw tx end\n");
	rtos_delete_thread(NULL);
}

static void wifi_raw_tx_command(char *pcWriteBuffer, int xWriteBufferLen,
				int argc, char **argv)
{
	OSStatus ret;

	if (argc != 3) {
		bk_printf("param error");
		bk_printf("usage: wifi_raw_tx interval counter");
		return;
	}

	wifi_raw_tx_param_t *tx_param;
	tx_param = (wifi_raw_tx_param_t *)os_malloc(sizeof(wifi_raw_tx_param_t));
	if (!tx_param) {
		bk_printf("out of memory\n");
		return;
	}

	tx_param->interval = os_strtoul(argv[1], NULL, 10);
	tx_param->counter = os_strtoul(argv[2], NULL, 10);
	ret = rtos_create_thread(NULL, THD_CORE_PRIORITY, "raw_tx",
				(beken_thread_function_t)wifi_raw_tx_thread,
				2048, tx_param);
	if (kNoErr != ret) {
		os_free(tx_param);
		os_printf("Create raw tx thread failed, ret=%d\r\n", ret);
		return;
	}
}
#endif

#if CFG_BK_AWARE
extern void bk_aware_demo_main(void);
extern void bk_aware_demo_stop(void);

static void bk_wifi_aware_command(char *pcWriteBuffer, int xWriteBufferLen,
				int argc, char **argv)
{
	if (argc == 2 && os_strcmp(argv[1], "start") == 0) {
		bk_aware_demo_main();
	} else if (argc == 2 && os_strcmp(argv[1], "stop") == 0) {
		bk_aware_demo_stop();
	} else {
		os_printf("Usage: bk_aware <start>|<stop>\n");
	}
}
#endif

#if CFG_WIFI_FTM
static void ftm_command(char *pcWriteBuffer, int xWriteBufferLen,
				int argc, char **argv)
{
	if (argc == 2 && os_strcmp(argv[1], "start") == 0) {
		rw_msg_send_ftm_start(0, 3, 1);
	} else if (argc == 2 && os_strcmp(argv[1], "stop") == 0) {
		//bk_aware_demo_stop();
	} else {
		os_printf("Usage: ftm <start>|<stop>\n");
	}
}
#endif

#if CFG_BK_NX_GET_WIFI_SNR
extern uint8_t rwnx_get_system_snr(void);
static void wifi_snr_timer_handler(void *data)
{
    os_printf("wifi snr:%d\r\n", rwnx_get_system_snr());
}

beken_timer_t wifi_snr_timer;
static void get_wifi_snr_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    rtos_init_timer(&wifi_snr_timer,
                          1 * 1000,
                          wifi_snr_timer_handler,
                          (void *)0);
    rtos_start_timer(&wifi_snr_timer);
}
#endif

#if (CFG_SUPPORT_MATTER == 0)
static const struct cli_command built_ins[] =
{
    {"help", NULL, help_command},
    {"version", NULL, get_version},
    {"echo", NULL, echo_cmd_handler},
    {"exit", "CLI exit", cli_exit_handler},

    /// WIFI
    {"scan", "scan ap", wifiscan_Command},
    {"advscan", "scan ap", wifiadvscan_Command},
    {"softap", "softap ssid key", softap_Command},
    {"stopintf", "stopintf intfacename", stop_wlan_intface_Command},
    {"sta", "sta ap_ssid key", sta_Command},
#if CFG_WPA2_ENTERPRISE
	{"sta_eap", "sta_eap ssid password [identity] [client_cert] [private_key]", sta_eap_Command},
#endif
#if CFG_WFA_CERT
    {"net", "wifi net config", net_Command},           // 8k rom size
#endif
    {"adv", "adv", sta_adv_Command},
    {"mtr", "mtr channel", mtr_Command},
    {"addif", "addif param", add_virtual_intface},
    {"delif", "delif role", del_virtual_intface},
    {"showif", "show", show_virtual_intface},
    {"psk", "show psk", show_sta_psk},

    {"wifistate", "Show wifi state", wifistate_Command},
    {"blacklist", "Set ssid blacklist", blacklist_Command},

    // network
    {"ifconfig", "Show IP address", ifconfig_Command},
    {"ping", "ping <ip>", ping_Command},
    {"dns", "show/clean/<domain>", dns_Command},
    {"sockshow", "Show all sockets", socket_show_Command},
    // os
    {"tasklist", "list all thread name status", task_Command},
#if CFG_MEM_DEBUG
    {"memleak", "show memleak", memleak_Command},
#endif

    // others
    {"memshow", "print memory information", memory_show_Command},
    {"memdump", "<addr> <length>", memory_dump_Command},
    {"os_memset", "<addr> <value 1> [<value 2> ... <value n>]", memory_set_Command},
    {"memp", "print memp list", memp_dump_Command},

    {"reboot", "reboot system", reboot},

    {"time",     "system time",                 uptime_Command},
    {"partition",    "Flash partition map",            partShow_Command},

    {"GPIO", "GPIO <cmd> <arg1> <arg2>", Gpio_op_Command},
    {"GPIO_INT", "GPIO_INT <cmd> <arg1> <arg2>", Gpio_int_Command},
    {"flash", "flash <cmd(R/W/E/N/idle_read_start/idle_read_stop)>", flash_command_test},
    {"UART", "UART I <index>", Uart_command_test},

#if CFG_TX_EVM_TEST
    {"txevm", "txevm [-m] [-c] [-l] [-r] [-w]", tx_evm_cmd_test},
#endif
#if CFG_RX_SENSITIVITY_TEST
    {"rxsens", "rxsens [-m] [-d] [-c] [-l]", rx_sens_cmd_test},
#endif
    #if (CFG_SOC_NAME != SOC_BK7231)
    {"efuse",       "efuse [-r addr] [-w addr data]", efuse_cmd_test},
    {"efusemac",    "efusemac [-r] [-w] [mac]",       efuse_mac_cmd_test},
    #endif // (CFG_SOC_NAME != SOC_BK7231)

#if CFG_SARADC_CALIBRATE
    {"adc", "adc [func] [param]", adc_command},
#endif

    {"easylink", "start easylink", easylink_Command},
#if CFG_AIRKISS_TEST
    {"airkiss", "start airkiss", airkiss_Command},

#endif
#if CFG_SUPPORT_OTA_TFTP
	{"tftpota", "tftpota [ip] [file]", tftp_ota_get_Command},
#endif

#if CFG_USE_SDCARD_HOST
    {"sdtest", "sdtest <cmd>", sd_operate},
#endif

#if CFG_USE_TEMPERATURE_DETECT
    {"tmpdetect", "tmpdetect <cmd>", temp_detect_Command},
#endif
#if CFG_SUPPORT_OTA_HTTP
    {"http_ota", "http_ota url", http_ota_Command},
#endif
    {"regshow", "regshow -w/r addr [value]", reg_write_read_test},

	{"cca", "cca open\\close\\show", phy_cca_test},
#if ((CFG_SOC_NAME != SOC_BK7231) && (CFG_SUPPORT_BLE == 1))
    {"ble", "ble arg1 arg2",  ble_command},
#endif

#if (CFG_SOC_NAME != SOC_BK7231)
    {"rfcali_cfg_mode",      "1:manual, 0:auto, others: clear flash mode",      cmd_rfcali_cfg_mode},
    {"rfcali_cfg_tssi_g",    "0-255",                 cmd_rfcali_cfg_tssi_g},
    {"rfcali_cfg_tssi_b",    "0-255",                 cmd_rfcali_cfg_tssi_b},
    {"rfcali_cfg_tssi_n20",  "0-255",                 cmd_rfcali_cfg_tssi_n20},
    {"rfcali_cfg_tssi_n40",  "0-255",                 cmd_rfcali_cfg_tssi_n40},
    {"rfcali_show_data",     "",                      cmd_rfcali_show_data},
    {"rfcali_cfg_rate_dist", "b g n40 ble (0-31)",    cmd_rfcali_cfg_rate_dist},
#endif
#if CFG_WIFI_SENSOR
	{"wifisensor", "wifi sensor", wifi_sensor_command},
#endif
    {"wifi_mgmt_filter", "wifi_mgmt_filter <0/1/-1>", cmd_wifi_mgmt_filter},
#if CFG_WIFI_RAW_TX_CMD
	{"wifi_raw_tx", "wifi_raw_tx", wifi_raw_tx_command},
#endif
#if CFG_BK_AWARE
	{"bk_aware", "bk_aware", bk_wifi_aware_command},
#endif
#if CFG_QUICK_TRACK
    {"app", "quicktrack controlappc", controlappc_start},
#endif
#if CFG_WIFI_FTM
    {"ftm", "802.11mc FTM", ftm_command},
#endif
#if CFG_BK_NX_GET_WIFI_SNR
    {"get_wifi_snr", "get", get_wifi_snr_Command},
#endif
#if (CFG_SUPPORT_MATTER)
    {"bk_write_data_test", "bk_write_data_test", bk_write_data_test},
    {"bk_read_data_test", "bk_read_data_test argv[]", bk_read_data_test},
    {"bk_erase_namespace_test", "bk_erase_namespace_test argv[]", bk_erase_namespace_test},
    {"bk_erase_name_test", "bk_erase_name_test argv[]", bk_erase_name_test},
    {"bk_name_data_exits", "bk_name_data_exits argv[]", bk_name_data_exits},
    {"bk_erase_all_test", "bk_erase_all_test ", bk_erase_all_test},
    {"BkQueryImageCmdHandler", "BkQueryImageCmdHandler [nodeid]  [fabricid]", BkQueryImageCmdHandler},
    {"BkApplyUpdateCmdHandler", "BkApplyUpdateCmdHandler", BkApplyUpdateCmdHandler},
    {"BkNotifyUpdateApplied", "BkNotifyUpdateApplied [version]", BkNotifyUpdateApplied},
#endif
};
#else
static const struct cli_command built_ins[] =
{
    {"help", NULL, help_command},
    {"version", NULL, get_version},
    {"echo", NULL, echo_cmd_handler},
    {"exit", "CLI exit", cli_exit_handler},
    {"wifistate", "Show wifi state", wifistate_Command},
    {"blacklist", "Set ssid blacklist", blacklist_Command},
    // network
    {"ifconfig", "Show IP address", ifconfig_Command},
    {"ping", "ping <ip>", ping_Command},
    {"dns", "show/clean/<domain>", dns_Command},
    {"sockshow", "Show all sockets", socket_show_Command},
    // os
    {"tasklist", "list all thread name status", task_Command},
#if CFG_MEM_DEBUG
    {"memleak", "show memleak", memleak_Command},
#endif
    // others
    {"memshow", "print memory information", memory_show_Command},
    {"memdump", "<addr> <length>", memory_dump_Command},
    {"os_memset", "<addr> <value 1> [<value 2> ... <value n>]", memory_set_Command},
    {"memp", "print memp list", memp_dump_Command},
    {"reboot", "reboot system", reboot},
    {"time",     "system time",                 uptime_Command},
    {"partition",    "Flash partition map",            partShow_Command},
#if CFG_SARADC_CALIBRATE
    {"adc", "adc [func] [param]", adc_command},
#endif
#if CFG_AIRKISS_TEST
    {"airkiss", "start airkiss", airkiss_Command},
#endif
#if CFG_SUPPORT_OTA_TFTP
    {"tftpota", "tftpota [ip] [file]", tftp_ota_get_Command},
#endif
#if CFG_USE_SDCARD_HOST
    {"sdtest", "sdtest <cmd>", sd_operate},
#endif
#if CFG_WIFI_SENSOR
    {"wifisensor", "wifi sensor", wifi_sensor_command},
#endif
#if CFG_WIFI_RAW_TX_CMD
    {"wifi_raw_tx", "wifi_raw_tx", wifi_raw_tx_command},
#endif
#if CFG_BK_AWARE
    {"bk_aware", "bk_aware", bk_wifi_aware_command},
#endif
#if CFG_QUICK_TRACK
    {"app", "quicktrack controlappc", controlappc_start},
#endif
#if CFG_WIFI_FTM
    {"ftm", "802.11mc FTM", ftm_command},
#endif
#if CFG_BK_NX_GET_WIFI_SNR
    {"get_wifi_snr", "get", get_wifi_snr_Command},
#endif
    {"bk_erase_all_test", "bk_erase_all_test ", bk_erase_all_test},
};
#endif

/* Built-in "help" command: prints all registered commands and their help
* text string, if any. */
void help_command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    int i, n;
    uint32_t build_in_count = sizeof(built_ins) / sizeof(struct cli_command);

#if (DEBUG)
    build_in_count++; //For command: micodebug
#endif

    os_printf( "====Build-in Commands====\r\n" );
    for (i = 0, n = 0; i < MAX_CMD_COUNT && n < pCli->num_commands; i++)
    {
        if (pCli->commands[i]->name)
        {
            os_printf("%s: %s\r\n", pCli->commands[i]->name,
                      pCli->commands[i]->help ?
                      pCli->commands[i]->help : "");
            n++;
            if( n == build_in_count )
            {
                os_printf("\r\n====User Commands====\r\n");
            }
        }
    }
}


int cli_register_command(const struct cli_command *command)
{
    int i;
    if (!command->name || !command->function)
        return 1;

    if (pCli->num_commands < MAX_CMD_COUNT)
    {
        /* Check if the command has already been registered.
        * Return 0, if it has been registered.
        */
        for (i = 0; i < pCli->num_commands; i++)
        {
            if (pCli->commands[i] == command)
                return 0;
        }
        pCli->commands[pCli->num_commands++] = command;
        return 0;
    }

    return 1;
}

int cli_unregister_command(const struct cli_command *command)
{
    int i;
    if (!command->name || !command->function)
        return 1;

    for (i = 0; i < pCli->num_commands; i++)
    {
        if (pCli->commands[i] == command)
        {
            pCli->num_commands--;
            int remaining_cmds = pCli->num_commands - i;
            if (remaining_cmds > 0)
            {
                os_memmove(&pCli->commands[i], &pCli->commands[i + 1],
                           (remaining_cmds *
                            sizeof(struct cli_command *)));
            }
            pCli->commands[pCli->num_commands] = NULL;
            return 0;
        }
    }

    return 1;
}


int cli_register_commands(const struct cli_command *commands, int num_commands)
{
    int i;
    for (i = 0; i < num_commands; i++)
        if (cli_register_command(commands++))
            return 1;
    return 0;
}

int cli_unregister_commands(const struct cli_command *commands,
                            int num_commands)
{
    int i;
    for (i = 0; i < num_commands; i++)
        if (cli_unregister_command(commands++))
            return 1;

    return 0;
}

static void micodebug_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    if (argc == 1)
    {
        os_printf("Usage: micodebug on/off. _BK_ debug is currently %s\r\n",
                  mico_debug_enabled ? "Enabled" : "Disabled");
        return;
    }

    if (!os_strcasecmp(argv[1], "on"))
    {
        os_printf("Enable _BK_ debug\r\n");
        mico_debug_enabled = 1;
    }
    else if (!os_strcasecmp(argv[1], "off"))
    {
        os_printf("Disable _BK_ debug\r\n");
        mico_debug_enabled = 0;
    }
}

void monitor(uint8_t *data, int len, wifi_link_info_t *info)
{
    int i;

    monitor_dbg("[%d]: ", len);
    for(i = 0; i < len; i++)
    {
        monitor_dbg("%02x ", data[i]);
    }
    monitor_dbg("\r\n");
}

static void monitor_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    if (argc == 1)
    {
        os_printf("Usage: monitor on/off.");
        return;
    }

    if (!os_strcasecmp(argv[1], "on"))
    {
        cmd_printf("start monitor\r\n");
        // start monitor, need set callback
        bk_wlan_register_monitor_cb(monitor);
        // then start hal mac
        bk_wlan_start_monitor();
    }
    else if (!os_strcasecmp(argv[1], "off"))
    {
        cmd_printf("stop monitor\r\n");
        mico_debug_enabled = 0;

        // stop monitor mode, need stop hal mac first
        bk_wlan_stop_monitor();
        // then set monitor callback
        bk_wlan_register_monitor_cb(NULL);
    }
}

#if CFG_AP_MONITOR_COEXIST_TBTT
static void monitor_tbtt_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    if (argc == 1)
    {
        os_printf("Usage: monitor tbtt on/off.");
        return;
    }

    if (!os_strcasecmp(argv[1], "on"))
    {
        cmd_printf("monitor tbtt on\r\n");
        bk_wlan_ap_monitor_coexist_tbtt_enable();
    }
    else if (!os_strcasecmp(argv[1], "off"))
    {
        cmd_printf("monitor tbtt off\r\n");
        bk_wlan_ap_monitor_coexist_tbtt_disable();
    }
}

void monitor_tbtt_dur_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    int tbtt_duration = 0;

    if(argc != 2)
    {
        os_printf("Usage: monitor_tbtt_dur [0~50].");
        return;
    }

    tbtt_duration = atoi(argv[1]);

    bk_wlan_ap_monitor_coexist_tbtt_duration(tbtt_duration);
}
#endif

#if CFG_AP_MONITOR_COEXIST_DEMO
void monitor_all_chan_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    u8 start = 0;

    if(argc != 2)
    {
        os_printf("need 2 parameters: monitor all channel 1(start), 0(stop)\r\n");
        return;
    }

    start = strtoul(argv[1], NULL, 0);

    monitor_process(start);
}
#endif

static void channel_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    int channel, i = 0;

    if (argc == 1)
    {
        os_printf("Usage: channel [1~13].");
        return;
    }

    while(argv[1][i])
    {
        if((argv[1][i] < '0') || (argv[1][i] > '9'))
        {
            os_printf("parameter should be a number\r\n");
            return ;
        }
        i++;
    }

    channel = atoi(argv[1]);

    if((channel < 1) || (channel > 13))
    {
        os_printf("Invalid channel number \r\n");
        return ;
    }
    cmd_printf("monitor mode :set to channel %d\r\n", channel);
    bk_wlan_set_channel_sync(channel);
}

void pwr_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    int pwr = 0;

    if(argc != 2)
    {
        os_printf("Usage: pwr [hex:5~15].");
        return;
    }

    pwr = os_strtoul(argv[1], NULL, 16);

    rw_msg_set_power(0,pwr);
}

static void Deep_Sleep_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	PS_DEEP_CTRL_PARAM deep_sleep_param;

	deep_sleep_param.wake_up_way			= 0;

	deep_sleep_param.gpio_index_map      	= os_strtoul(argv[1], NULL, 16);
	deep_sleep_param.gpio_edge_map       	= os_strtoul(argv[2], NULL, 16);
	deep_sleep_param.gpio_last_index_map 	= os_strtoul(argv[3], NULL, 16);
	deep_sleep_param.gpio_last_edge_map  	= os_strtoul(argv[4], NULL, 16);
	deep_sleep_param.sleep_time     		= os_strtoul(argv[5], NULL, 16);
	deep_sleep_param.wake_up_way     		= os_strtoul(argv[6], NULL, 16);
	deep_sleep_param.gpio_stay_lo_map 	    = os_strtoul(argv[7], NULL, 16);
	deep_sleep_param.gpio_stay_hi_map  	    = os_strtoul(argv[8], NULL, 16);

	if(argc == 9)
	{
		os_printf("---deep sleep test param : 0x%0X 0x%0X 0x%0X 0x%0X %d %d\r\n",
					deep_sleep_param.gpio_index_map,
					deep_sleep_param.gpio_edge_map,
					deep_sleep_param.gpio_last_index_map,
					deep_sleep_param.gpio_last_edge_map,
					deep_sleep_param.sleep_time,
					deep_sleep_param.wake_up_way);

	#if (CFG_SOC_NAME != SOC_BK7271) && CFG_USE_DEEP_PS
		bk_enter_deep_sleep_mode(&deep_sleep_param);
	#endif
	}
	else
	{
		os_printf("---argc error!!! \r\n");
	}
}

static void Ps_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    UINT32 dtim = 0;

    dtim = dtim;

	#if PS_SUPPORT_MANUAL_SLEEP
    UINT32 standby_time = 0;
    UINT32 dtim_wait_time = 0;
	#endif

    if(argc < 3)
    {
        goto IDLE_CMD_ERR;
    }
#if  CFG_USE_DEEP_PS

    if(0 == os_strcmp(argv[1], "deepps"))
    {
    	PS_DEEP_CTRL_PARAM deep_sleep_param;
        if(argc != 11)
        {
            goto IDLE_CMD_ERR;
        }

    	deep_sleep_param.gpio_index_map      	= os_strtoul(argv[2], NULL, 16);
    	deep_sleep_param.gpio_edge_map       	= os_strtoul(argv[3], NULL, 16);
    	deep_sleep_param.gpio_stay_lo_map 	    = os_strtoul(argv[4], NULL, 16);

    	deep_sleep_param.gpio_last_index_map 	= os_strtoul(argv[5], NULL, 16);
    	deep_sleep_param.gpio_last_edge_map  	= os_strtoul(argv[6], NULL, 16);
        deep_sleep_param.gpio_stay_hi_map  	    = os_strtoul(argv[7], NULL, 16);

    	deep_sleep_param.sleep_time     		= os_strtoul(argv[8], NULL, 16);
    	deep_sleep_param.lpo_32k_src     		= os_strtoul(argv[9], NULL, 16);

    	deep_sleep_param.wake_up_way     		= os_strtoul(argv[10], NULL, 16);

		os_printf("---deep sleep test param : 0x%0X 0x%0X 0x%0X 0x%0X %d %d\r\n",
					deep_sleep_param.gpio_index_map,
					deep_sleep_param.gpio_edge_map,
					deep_sleep_param.gpio_last_index_map,
					deep_sleep_param.gpio_last_edge_map,
					deep_sleep_param.sleep_time,
					deep_sleep_param.wake_up_way);
		os_printf("--- 0x%0X 0x%0X 0x%0X \r\n",
        			deep_sleep_param.gpio_stay_lo_map,
        			deep_sleep_param.gpio_stay_hi_map,
        			deep_sleep_param.lpo_32k_src);

		bk_enter_deep_sleep_mode(&deep_sleep_param);
    }
#endif

#if CFG_USE_FAKERTC_PS
    else if (0 == os_strcmp(argv[1], "idleps"))
    {
        GLOBAL_INT_DECLARATION();
        int count = 0;
        bk_printf("[ARF]rwnxl_reset_evt\r\n");
        HAL_FATAL_ERROR_RECOVER(0);	// rwnxl_reset_evt(0);

        rtos_delay_milliseconds(10);

        while (1) {
            GLOBAL_INT_DISABLE();
            evt_field_t field = ke_evt_get();
            GLOBAL_INT_RESTORE();

            if (!(field & KE_EVT_RESET_BIT))
                break;

            rtos_delay_milliseconds(10);
            if (++count > 10000) {
                bk_printf("%s: failed\r\n", __func__);
                break;
            }
        }

        //bk_enable_unconditional_sleep();
        count = 100;
        while((1 == bk_unconditional_normal_sleep(0xFFFFFFFF,1))) {
            rtos_delay_milliseconds(2);
			count--;
			if(count == 0){
				bk_printf("IDLE_SLEEP timeout\r\n");
				break;
        	}
        }

        bk_printf("idle Sleep out\r\n");
    }
#endif

#if CFG_USE_MCU_PS
    else if(0 == os_strcmp(argv[1], "mcudtim"))
    {
        if(argc != 3)
        {
            goto IDLE_CMD_ERR;
        }

        dtim = os_strtoul(argv[2], NULL, 10);
        if(dtim == 1)
        {
            bk_wlan_mcu_ps_mode_enable();
        }
        else if(dtim == 0)
        {
            bk_wlan_mcu_ps_mode_disable();
        }
        else
        {
            goto IDLE_CMD_ERR;
        }
    }
#if (1 == CFG_LOW_VOLTAGE_PS)
    else if(0 == os_strcmp(argv[1], "gpio"))
    {
        if(argc < 4)
        {
            goto IDLE_CMD_ERR;
        }
        sctrl_set_gpio_wakeup_index(os_strtoul(argv[2], NULL, 10),os_strtoul(argv[3], NULL, 10));
    }
#endif
#endif
#if CFG_USE_STA_PS
    else if(0 == os_strcmp(argv[1], "rfdtim"))
    {
        if(argc < 3)
        {
            goto IDLE_CMD_ERR;
        }

        dtim = os_strtoul(argv[2], NULL, 10);
        if(dtim == 1)
        {
            if(argc == 4)
            {
                dtim = os_strtoul(argv[3], NULL, 10);
                power_save_set_listen_int(dtim);
            }
            if (bk_wlan_dtim_rf_ps_mode_enable())
                os_printf("dtim enable failed\r\n");

            os_printf("listen interval = %d\r\n", power_save_get_listen_int());
        }
        else if(dtim == 0)
        {
            if (bk_wlan_dtim_rf_ps_mode_disable())
                os_printf("dtim disable failed\r\n");
        }
        else
        {
            goto IDLE_CMD_ERR;
        }
    }
    else if(0 == os_strcmp(argv[1], "rfwkup"))
    {
        if(argc != 3)
        {
            goto IDLE_CMD_ERR;
        }

        power_save_rf_dtim_manual_do_wakeup();
    }
    else if(0 == os_strcmp(argv[1], "setwktm"))
    {
        if(argc != 3)
        {
            os_printf("beacon len:%d\r\n", power_save_beacon_len_get());
            os_printf("wktm:%d\r\n", power_save_radio_wkup_get());
            goto IDLE_CMD_ERR;
        }

        dtim = os_strtoul(argv[2], NULL, 10);

        if(dtim)
        {
            power_save_radio_wkup_set(dtim);
            os_printf("set ridio wkup:%d\r\n", dtim);
        }
        else
        {
            goto IDLE_CMD_ERR;
        }
    }
    else if(0 == os_strcmp(argv[1], "bcmc"))
    {
        if(argc != 3)
        {
            goto IDLE_CMD_ERR;
        }

        dtim = os_strtoul(argv[2], NULL, 10);

        if(dtim == 0 || dtim == 1)
        {
            power_save_sm_set_all_bcmc(dtim);
        }
        else
        {
            goto IDLE_CMD_ERR;
        }
    }
	#if PS_USE_KEEP_TIMER
    else if(0 == os_strcmp(argv[1], "rf_timer"))
    {
        if(argc != 3)
        {
            goto IDLE_CMD_ERR;
        }

        dtim = os_strtoul(argv[2], NULL, 10);

        if(dtim == 1)
        {
        	extern int bk_wlan_dtim_rf_ps_timer_start(void);
            bk_wlan_dtim_rf_ps_timer_start();
        }
        else  if(dtim == 0)
        {
        	extern int bk_wlan_dtim_rf_ps_timer_pause(void);
            bk_wlan_dtim_rf_ps_timer_pause();
        }
        else
        {
            goto IDLE_CMD_ERR;
        }
    }
	#endif
    else if(0 == os_strcmp(argv[1], "listen"))
    {
        if(argc != 4)
        {
            goto IDLE_CMD_ERR;
        }

        if(0 == os_strcmp(argv[2], "dtim"))
        {
            dtim = os_strtoul(argv[3], NULL, 10);
            power_save_set_dtim_multi(dtim);

        }
        else
        {
            goto IDLE_CMD_ERR;
        }

    }
    else if(0 == os_strcmp(argv[1], "dump"))
    {
        #if CFG_USE_MCU_PS
        mcu_ps_dump();
        #endif
        power_save_dump();
    }
#if ((1 == CFG_LOW_VOLTAGE_PS) && (1 == CFG_LOW_VOLTAGE_PS_TEST))
    else if(0 == os_strcmp(argv[1], "info"))
    {
        if(argc == 3)
        {
            bool print_flag = os_strtoul(argv[2], NULL, 10);
            lv_ps_info_print_switch(print_flag, 10*60);
            os_printf("enable ps info print!\r\n");
            os_printf("ps info print period: 600 s!\r\n");
        }
        else if(argc == 4)
        {
            bool print_flag = os_strtoul(argv[2], NULL, 10);
            uint32_t print_period = os_strtoul(argv[3], NULL, 10);
            lv_ps_info_print_switch(print_flag, print_period);
            os_printf("enable ps info print!\r\n");
            os_printf("ps info print period: %d s!\r\n", print_period);

        }
        else
        {
            goto IDLE_CMD_ERR;
        }
    }
#endif
#if(CFG_HW_PARSER_TIM_ELEMENT == 1)
    else if(0 == os_strcmp(argv[1], "tim"))
    {
        uint32_t cnt = os_strtoul(argv[2], NULL, 10);
        power_save_set_hw_tim_cnt_limit(cnt);
    }
#endif
#if ( 1 == CFG_LOW_VOLTAGE_PS_32K_DIV)
    else if(0 == os_strcmp(argv[1], "test"))
    {
        extern void sctrl_sta_test_sleep(uint32 sleep_ms);
        dtim = os_strtoul(argv[2], NULL, 10);
        sctrl_sta_test_sleep(dtim);
    }
#endif
#endif
    else
    {
        goto IDLE_CMD_ERR;
    }


    return;
IDLE_CMD_ERR:
    os_printf("Usage:ps [func] [param] []\r\n");
    os_printf("standby:ps deepps [gpio_map] ");
    os_printf("gpio_map is hex and every bits is map to gpio0-gpio31\r\n");
    os_printf("ieee:ps rfdtim [],[1/0] is open or close \r\n\r\n");
    os_printf("ieee:ps mcudtim [],[1/0] is open or close \r\n\r\n");
#if ((1 == CFG_LOW_VOLTAGE_PS) && (1 == CFG_LOW_VOLTAGE_PS_TEST))
    os_printf("info:ps info [1/0] [period_s], default period = 600 s \r\n");
#endif

}

static void mac_command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    uint8_t mac[6];

    if (argc == 1)
    {
        wifi_get_mac_address((char *)mac, CONFIG_ROLE_STA);
        os_printf("MAC address: %02x-%02x-%02x-%02x-%02x-%02x\r\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    else if(argc == 2)
    {
        hexstr2bin(argv[1], mac, 6);
        wifi_set_mac_address((char *)mac);
        os_printf("Set MAC address: %02x-%02x-%02x-%02x-%02x-%02x\r\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    else
    {
        os_printf("invalid cmd\r\n");
    }

}

static void rc_command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	uint8_t sta_idx = 0;
	uint16_t rate_cfg = 0;

	if (argc <= 2) {
		os_printf("invalid RC command\n");
		return;
	}

	if(os_strcmp(argv[1], "set_fixrate") == 0) {
		sta_idx = os_strtoul(argv[2], NULL, 10) & 0xFFFF;
		rate_cfg = os_strtoul(argv[3], NULL, 10) & 0xFFFF;
		bk_wifi_rc_config(sta_idx, rate_cfg);
	}
	else {
		os_printf("invalid RC paramter\n");
	}
}

#if CFG_SARADC_CALIBRATE
/****channel 1 - 7***/
static ADC_OBJ test_adc;
extern uint8_t step_flag ;
extern uint8_t adctest_flag ;
extern uint8_t adc_accuracy;
int adc_offfset, adc_value_2v;

static void adc_detect_callback(int new_mv, void *user_data)
{
	static int cnt = 0;
	new_mv = (new_mv << (adc_accuracy - 1));
	test_adc.user_data = (void *)new_mv;

	if (cnt++ >= 100)
	{
		cnt = 0;
		os_printf("adc channel%d voltage:%d,%x\r\n", test_adc.channel, new_mv, test_adc.user_data);
	}
}

static void adc_detect_callback1(int new_mv, void *user_data)
{
	static int total = 0;
	static int cnt = 0;
	int low_adc;
	static int temp = 0;
	if (temp++ < 100)
		return;
	new_mv = (new_mv << (adc_accuracy - 1));
	test_adc.user_data = (void *)new_mv;

	total += new_mv;
	cnt++;
	if (cnt >= 100)
	{
		low_adc = total / cnt;
		saradc_val.low = low_adc;
		cnt = 0;
		temp = 0;
		adc_offfset = low_adc - 2048;
		os_printf("step1: adc channel:%d adc_offfset:%d,low_adc:%d\r\n", test_adc.channel, adc_offfset, low_adc);
		os_printf("adc_low:%x,adc_high,%x\r\n", saradc_val.low, saradc_val.high);
		total = 0;
	}
}

static void adc_detect_callback2(int new_mv, void *user_data)
{
	static int total = 0;
	static int cnt = 0;
	static int temp = 0;
	if (temp++ < 100)
		return;
	int high_adc;

	new_mv = (new_mv << (adc_accuracy - 1));
	test_adc.user_data = (void *)new_mv;

	total += new_mv;
	cnt++;
	if (cnt >= 100)
	{
		high_adc = total / cnt;
		saradc_val.high = high_adc;
		cnt = 0;
		temp = 0;
		adc_value_2v = high_adc;
		os_printf("step2: adc channel:%d adc_value_2v:%d\r\n", test_adc.channel, high_adc);
		total = 0;
		os_printf("adc_low:%x,adc_high,%x\r\n", saradc_val.low, saradc_val.high);

	}
}

static void adc_command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	UINT32 status;
	DD_HANDLE saradc_handle;
	saradc_cal_val_t p_ADC_cal;
	saradc_desc_t *p_ADC_drv_desc = NULL;
	int channel;
	GLOBAL_INT_DECLARATION();

	if (argc < 2)
		goto IDLE_CMD_ERR;

	if (0 == os_strcmp(argv[1], "read"))
	{
		status = manual_cal_load_adc_cali_flash();
		if (status != 0) {
			os_printf("Can't read cali value, use default!\r\n");
			os_printf("calibrate low value:[%x]\r\n", saradc_val.low);
			os_printf("calibrate high value:[%x]\r\n", saradc_val.high);
		}
	} else if (0 == os_strcmp(argv[1], "set"))
	{
		p_ADC_drv_desc = (saradc_desc_t *)os_malloc(sizeof(saradc_desc_t));
		if (p_ADC_drv_desc == NULL) {
			os_printf("malloc1 failed!\r\n");
			return;
		}

		saradc_config_param_init(p_ADC_drv_desc);

		p_ADC_drv_desc->data_buff_size = ADC_TEMP_BUFFER_SIZE;
		p_ADC_drv_desc->pData = (UINT16 *)os_malloc(p_ADC_drv_desc->data_buff_size * sizeof(UINT16));
		os_memset(p_ADC_drv_desc->pData, 0x00, p_ADC_drv_desc->data_buff_size * sizeof(UINT16));

		if (p_ADC_drv_desc->pData == NULL) {
			os_printf("malloc1 failed!\r\n");
			os_free(p_ADC_drv_desc);
			return;
		}

		UINT32 ret = 0;
		do {
			GLOBAL_INT_DISABLE();
			if (saradc_check_busy() == 0) {
				saradc_handle = ddev_open(SARADC_DEV_NAME, &status, (UINT32)p_ADC_drv_desc);
				if (DD_HANDLE_UNVALID != saradc_handle) {
					GLOBAL_INT_RESTORE();
					break;
				}
			}
			GLOBAL_INT_RESTORE();

			rtos_delay_milliseconds(5);
			ret++;
		} while (ret < 5);

		if (ret == 5) {
			os_printf("saradc open failed!\r\n");
			os_free(p_ADC_drv_desc->pData);
			os_free(p_ADC_drv_desc);
			return;
		}

		while (1) {
			if (p_ADC_drv_desc->current_sample_data_cnt >=
				p_ADC_drv_desc->data_buff_size) {
				ddev_close(saradc_handle);
				saradc_ensure_close();
				break;
			}
		}

		{
			UINT32 sum = 0, sum1, sum2;
			UINT16 *pData = p_ADC_drv_desc->pData;
			sum1 = pData[1] + pData[2];
			sum2 = pData[3] + pData[4];
			sum = sum1 / 2  + sum2 / 2;
			sum = sum / 2;
			sum = sum / 4;
			p_ADC_drv_desc->pData[0] = sum;
		}

		if (0 == os_strcmp(argv[2], "low"))
			p_ADC_cal.mode = SARADC_CALIBRATE_LOW;
		else if (0 == os_strcmp(argv[2], "high"))
			p_ADC_cal.mode = SARADC_CALIBRATE_HIGH;
		else {
			os_printf("invalid parameter\r\n");
			return;
		}
		p_ADC_cal.val = p_ADC_drv_desc->pData[4];
		if (SARADC_FAILURE == ddev_control(saradc_handle, SARADC_CMD_SET_CAL_VAL, (VOID *)&p_ADC_cal)) {
			os_printf("set calibrate value failture\r\n");
			os_free(p_ADC_drv_desc->pData);
			os_free(p_ADC_drv_desc);
			return;
		}
		os_printf("set calibrate success\r\n");
		os_printf("type:[%s] value:[0x%x]\r\n", (p_ADC_cal.mode ? "high" : "low"), p_ADC_cal.val);
		os_free(p_ADC_drv_desc->pData);
		os_free(p_ADC_drv_desc);
	} else if (0 == os_strcmp(argv[1], "write"))
	{
		manual_cal_save_chipinfo_tab_to_flash();
		os_printf("calibrate low value:[%x]\r\n", saradc_val.low);
		os_printf("calibrate high value:[%x]\r\n", saradc_val.high);
	} else if (0 == os_strcmp(argv[1], "cal_low"))
	{
		/*adc calibration when first usd adc channel ,
		 *cal_low: caculate offset when voltage is 0v.
		 *cal_high:caculate voltage is 2v .
		 *start: get current adc value.
		 */
		channel = os_strtoul(argv[2], NULL, 10);
		os_printf("offset adc channel:%d \r\n", channel);
		step_flag = 0;
		adctest_flag = 0;
		saradc_work_create();
		adc_obj_init(&test_adc, adc_detect_callback1, channel, &test_adc);
		adc_obj_start(&test_adc);
	} else if (0 == os_strcmp(argv[1], "cal_high"))
	{
		channel = os_strtoul(argv[2], NULL, 10);
		os_printf("2v: adc channel:%d \r\n", channel);
		step_flag = 1;
		adctest_flag = 0;
		saradc_work_create();
		adc_obj_init(&test_adc, adc_detect_callback2, channel, &test_adc);
		adc_obj_start(&test_adc);
	} else if (0 == os_strcmp(argv[1], "start"))
	{
		channel = os_strtoul(argv[2], NULL, 10);
		os_printf("---adc channel:%d---\r\n", channel);
		adctest_flag = 1;
		saradc_work_create();
		adc_obj_init(&test_adc, adc_detect_callback, channel, &test_adc);
		adc_obj_start(&test_adc);
	} else if (0 == os_strcmp(argv[1], "stop"))
		adc_obj_stop(&test_adc);
    else if(0 == os_strcmp(argv[1], "get"))
    {
        INT32 channel = atoi(argv[2]);
        UINT32 sum = 0;
        UINT32 index;
		typedef UINT16 heap_t;
		size_t MinHeapInsert(heap_t *heap, size_t heap_size, heap_t x);
		heap_t MinHeapReplace(heap_t *heap, size_t heap_size, heap_t x);

        p_ADC_drv_desc = (saradc_desc_t *)os_malloc(sizeof(saradc_desc_t));
        if (p_ADC_drv_desc == NULL)
        {
            os_printf("malloc1 failed!\r\n");
            return;
        }

        os_memset(p_ADC_drv_desc, 0x00, sizeof(saradc_desc_t));
        p_ADC_drv_desc->channel = channel;
        p_ADC_drv_desc->data_buff_size = 100;
        p_ADC_drv_desc->mode = (ADC_CONFIG_MODE_CONTINUE << 0)
                               | (ADC_CONFIG_MODE_4CLK_DELAY << 2);
        p_ADC_drv_desc->has_data                = 0;
        p_ADC_drv_desc->current_read_data_cnt   = 0;
        p_ADC_drv_desc->current_sample_data_cnt = 0;
        p_ADC_drv_desc->pre_div = 0x10;
        p_ADC_drv_desc->samp_rate = 0x20;
        p_ADC_drv_desc->pData = (UINT16 *)os_malloc(p_ADC_drv_desc->data_buff_size * sizeof(UINT16));
        os_memset(p_ADC_drv_desc->pData, 0x00, p_ADC_drv_desc->data_buff_size * sizeof(UINT16));

        if(p_ADC_drv_desc->pData == NULL)
        {
            os_printf("malloc1 failed!\r\n");
            os_free(p_ADC_drv_desc);

            return;
        }

        saradc_handle = ddev_open(SARADC_DEV_NAME, &status, (UINT32)p_ADC_drv_desc);
        while (1)
        {
            if (p_ADC_drv_desc->current_sample_data_cnt == p_ADC_drv_desc->data_buff_size)
            {
                if ((argc > 3) && atoi(argv[3]))
                {
                }
                else
                {
                ddev_close(saradc_handle);
                }
                break;
            }
        }

		heap_t heap[(100 + 1) / 2];
		int count = 0;

		for (index = 0; index < sizeof(heap) / sizeof(heap[0]); index++) {
			MinHeapInsert(heap, index, (heap_t)p_ADC_drv_desc->pData[index]);

		}

		for (index = sizeof(heap) / sizeof(heap[0]); index < 100; index++) {
			if (heap[0] < (heap_t)p_ADC_drv_desc->pData[index])
			{
				MinHeapReplace(heap, sizeof(heap) / sizeof(heap[0]), (heap_t)p_ADC_drv_desc->pData[index]);
			}
		}

		for (index = 0; index < 100; index++)
		{
			//error [-0.5%, 0.5%] ==> [-5, 5]
			if ((p_ADC_drv_desc->pData[index] > heap[0] + 0x10) || (heap[0] > p_ADC_drv_desc->pData[index] + 0x10))
			{
				continue;
			}

			count++;
			sum += p_ADC_drv_desc->pData[index];
		}

        p_ADC_drv_desc->pData[0] = (UINT16)(sum / count);
        float voltage = saradc_calculate(p_ADC_drv_desc->pData[0]);

        os_printf("voltage is [%f] orignal %d count=%d\r\n", voltage, p_ADC_drv_desc->pData[0], count);

        os_free(p_ADC_drv_desc->pData);
        os_free(p_ADC_drv_desc);
        return;
    }
	else
		goto IDLE_CMD_ERR;

	return;

IDLE_CMD_ERR:
	os_printf("Usage:ps [func] [param]\r\n");
}
#endif
#if !AT_SERVICE_CFG
static void cli_rx_callback(int uport, void *param)
{
	if(log_rx_interrupt_sema)
    	rtos_set_semaphore(&log_rx_interrupt_sema);
}
#endif
/* ========= CLI input&output APIs ============ */
int cli_printf(const char *msg, ...)
{
    va_list ap;
    char *pos, message[256];
    int sz;
    int nMessageLen = 0;

    os_memset(message, 0, 256);
    pos = message;

    sz = 0;
    va_start(ap, msg);
    nMessageLen = vsnprintf(pos, 256 - sz, msg, ap);
    va_end(ap);

    if( nMessageLen <= 0 ) return 0;

    cli_putstr((const char *)message);
    return 0;
}


int cli_putstr(const char *msg)
{
    if (msg[0] != 0)
        bk_uart_send(cli_uart, (const char *)msg, os_strlen(msg) );

    return 0;
}

int cli_getchar(char *inbuf)
{
    if (bk_uart_recv(cli_uart, inbuf, 1, BEKEN_WAIT_FOREVER) == 0)
        return 1;
    else
        return 0;
}

int cli_getchars(char *inbuf, int len)
{
    if(bk_uart_recv(cli_uart, inbuf, len, BEKEN_WAIT_FOREVER) == 0)
        return 1;
    else
        return 0;
}

int cli_getchars_prefetch(char *inbuf, int len)
{
    if(bk_uart_recv_prefetch(cli_uart, inbuf, len, BEKEN_WAIT_FOREVER) == 0)
        return 1;
    else
        return 0;
}

int cli_get_all_chars_len(void)
{
    return bk_uart_get_length_in_buffer(cli_uart);
}

extern void iperf(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
extern void cmd_la(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
#endif

static const struct cli_command user_clis[] =
{
    {"micodebug", "micodebug on/off", micodebug_Command},
    {"monitor", "monitor on/off", monitor_Command},
    {"channel", "channel []", channel_Command},
    {"mac", "mac <mac>, Get mac/Set mac. <mac>: c89346000001", mac_command},
    {"rc", "wifi rate control config", rc_command},
    {"ps", "ps [func] [param]", Ps_Command},
    {"deep_sleep", "deep_sleep [param]", Deep_Sleep_Command},
#if (1 == CFG_USE_FORCE_LOWVOL_PS)
    {"lowvol_sleep", "lowvol_sleep [param]", lowvol_Sleep_Command},
#endif
#ifdef TCP_CLIENT_DEMO
    {"tcp_cont", "tcp_cont [ip] [port]", tcp_make_connect_server_command},
#endif

#if CFG_TCP_SERVER_TEST
    {"tcp_server", "tcp_server [ip] [port]",make_tcp_server_command },
#endif

#if (CFG_IPERF_TEST == IPERF_OPEN_WITH_ACCEL)
    {"iperf", "iperf help", iperf },
#endif // (CFG_IPERF_TEST == IPERF_OPEN_WITH_ACCEL)

#if CFG_SUPPORT_TPC_PA_MAP
    {"pwr", "pwr help", pwr_Command },
#endif

#if (CFG_SOC_NAME == SOC_BK7221U)
    {"sec", "sec help", sec_Command },
#endif
#if CFG_AP_MONITOR_COEXIST_TBTT
    {"monitor_tbtt", "monitor tbtt on/off", monitor_tbtt_Command },
    {"monitor_tbtt_dur", "monitor tbtt dur (0-50)", monitor_tbtt_dur_Command },
#endif
#if CFG_AP_MONITOR_COEXIST_DEMO
    {"monitor_all_chan", "start monitor all channel", monitor_all_chan_Command},
#endif
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
    {"la", "la [i] to dump ADC data", cmd_la },
#endif
};

static const struct cli_command ate_clis[] =
{
#if (CFG_IPERF_TEST == IPERF_OPEN_ONLY)
    {"iperf", "iperf help", iperf },
#endif // (CFG_IPERF_TEST == IPERF_OPEN_ONLY)
};

extern int video_demo_register_cmd(void);

#if CFG_PERIPHERAL_TEST
void bk_peripheral_cli_init();
#endif


#if AT_SERVICE_CFG
int atsvr_app_init(void);
#endif

int cli_init(void)
{
  //  int ret;

    if (UART1_PORT == uart_print_port)
    {
        cli_uart = BK_UART_1;
    }
    else
    {
        cli_uart = BK_UART_2;
    }

    pCli = (struct cli_st *)os_malloc(sizeof(struct cli_st));
    if (pCli == NULL)
        return kNoMemoryErr;

    os_memset((void *)pCli, 0, sizeof(struct cli_st));
    rtos_init_semaphore(&log_rx_interrupt_sema, 10);

    if (cli_register_commands(&built_ins[0],
                              sizeof(built_ins) / sizeof(struct cli_command)))
    {
        goto init_general_err;
    }

    if(cli_register_commands(user_clis, sizeof(user_clis) / sizeof(struct cli_command))) {
        goto init_general_err;
    }

    if (get_ate_mode_state()) {
         if(cli_register_commands(ate_clis, sizeof(ate_clis) / sizeof(struct cli_command))) {
             goto init_general_err;
         }
    }

    if (video_demo_register_cmd()) {
        goto init_general_err;
    }

#if CFG_PERIPHERAL_TEST
    bk_peripheral_cli_init();
#endif

#if AT_SERVICE_CFG
    atsvr_app_init();
#else
    int ret;
    ret = rtos_create_thread(&cli_thread_handle,
                             BEKEN_DEFAULT_WORKER_PRIORITY,
                             "cli",
                             (beken_thread_function_t)cli_main,
                             #if (1 == CFG_SUPPORT_MATTER)
                             2048,
                             #else
                             4096,
                             #endif
                             0);
    if (ret != kNoErr)
    {
        os_printf("Error: Failed to create cli thread: %d\r\n",
                  ret);
        goto init_general_err;
    }
#endif

    pCli->initialized = 1;
    pCli->echo_disabled = 0;

    return kNoErr;

init_general_err:
    if(pCli)
    {
        os_free(pCli);
        pCli = NULL;
    }

    return kGeneralErr;
}
// eof

