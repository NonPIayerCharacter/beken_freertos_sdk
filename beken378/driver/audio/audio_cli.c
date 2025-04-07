#include<stdlib.h>
#include "wlan_cli_pub.h"
#include "mem_pub.h"
#include "str_pub.h"
#include "uart_pub.h"
#include "audio_pub.h"
#include "audio_intf_pub.h"

static void cli_aud_intf_test_cmd(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	if (os_strcmp(argv[1], "start") == 0) {
		audio_intf_init();
	} else if (os_strcmp(argv[1], "play") == 0) {
		if (os_strcmp(argv[2], "dac") == 0) {
			audio_intf_dac_play();
		} else if (os_strcmp(argv[2], "adc") == 0) {
			audio_intf_adc_play();
		}
	} else if (os_strcmp(argv[1], "vol") == 0) {
		audio_intf_dac_set_volume();
	} else if (os_strcmp(argv[1], "rate") == 0) {
		audio_intf_set_sample_rate();
	} else if (os_strcmp(argv[1], "pause") == 0) {
		if (os_strcmp(argv[2], "dac") == 0) {
			audio_intf_dac_pause();
		} else if (os_strcmp(argv[2], "adc") == 0) {
			audio_intf_adc_pause();
		}
	}
}

static const struct cli_command aud_ins[] =
{
	{"aud_intf_test", "aud_intf_test", cli_aud_intf_test_cmd},
};

int cli_aud_test_init(void)
{
	int ret;

	ret = cli_register_commands(&aud_ins[0], sizeof(aud_ins) / sizeof(struct cli_command));
	if (ret) {
		bk_printf("register audio cli fail\r\n");
		return -1;
	}

	return 0;
}
