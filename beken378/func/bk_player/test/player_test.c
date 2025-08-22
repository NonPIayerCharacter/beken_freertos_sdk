#include "include.h"

#if CFG_USE_BK_PLAYER_TEST

#if (!CFG_USE_BK_PLAYER)
#error "CFG_USE_BK_PLAYER_TEST need CFG_USE_BK_PLAYER set 1"
#endif

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "bk_player_api.h"
#include "plugin_manager.h"
#include "rtos_pub.h"
#include "mem_pub.h"
#include "str_pub.h"
#include "wlan_cli_pub.h"


beken_thread_t bk_player_test_handle = NULL;

static void app_event_handler(int event, void *extra_info)
{
    if (event == EVENT_SONG_TICK)
    {
        int cur_seconds;
        cur_seconds = *(int *)extra_info;
        bk_printf("===== %d =====\n", cur_seconds);
    }
}

static void player_test_main( beken_thread_arg_t data )
{
    audio_source_t *source = NULL;
    audio_codec_t *codec = NULL;
    audio_sink_t *sink = NULL;
    audio_info_t info;
    char *url = "http://192.168.200.43/daiwoqushanding.mp3";

    if(data)
        url = (char *)data;
    bk_printf("%p, %p\r\n", data, url);
    int codec_type;
    int ret;
    char *buffer = NULL;
    int len;
    int len2;
    int chunk_size;

    ret = bk_player_init(app_event_handler);
    if (ret)
    {
        bk_printf("bk_player_init failed\r\n");
        ret = -1;
        goto out;
    }

#if 0//add other source
    audio_source_ops_t other_source_ops =
    {
        .open = NULL,
        .get_codec_type = NULL,
        .get_total_bytes = NULL,
        .read = NULL,
        .seek = NULL,
        .close = NULL,
    };

    extern audio_source_ops_t other_source_ops;
    audio_source_register(&other_source_ops);
#endif


    source = audio_source_open_url(url);
    if (!source)
    {
        ret = -1;
        goto out;
    }

    if(data)
        os_free(data);

    codec_type = audio_source_get_codec_type(source);   //
    if (codec_type == AUDIO_CODEC_UNKNOWN)
    {
        ret = -4;
        goto out;
    }

    codec = audio_codec_open(codec_type, NULL, source);
    if (!codec)
    {
        ret = -2;
        goto out;
    }

    ret = audio_codec_get_info(codec, &info);   //
    if (ret)
    {
        ret = -5;
        goto out;
    }
    bk_printf("t1 channel_number=%d, sample_rate=%d, sample_bits=%d, total_bytes=%d\n",
              info.channel_number, info.sample_rate, info.sample_bits, info.total_bytes);

    chunk_size = audio_codec_get_chunk_size(codec); //
    bk_printf("chunk_size is %d\n", chunk_size);

    buffer = malloc(chunk_size);
    if (!buffer)
    {
        goto out;
    }

    sink = audio_sink_open(AUDIO_SINK_DEVICE, &info);
    bk_printf("audio_sink_open %p\r\n", sink);
    if (!sink)
    {
        ret = -3;
        goto out;
    }

    while (1)
    {
        len = audio_codec_get_data(codec, buffer, chunk_size);
        if (len <= 0)
        {
            bk_printf("codec : ret = %d\n", len);
            break;
        }
        len2 = audio_sink_write_data(sink, buffer, len);
        if (len2 <= 0)
        {
            bk_printf("sink : ret = %d\n", len2);
            break;
        }
    }
    ret = 0;

out:
    if (codec)
    {
        audio_codec_close(codec);
    }
    if (source)
    {
        audio_source_close(source);
    }
    if (sink)
    {
        audio_sink_close(sink);
    }

    if (buffer)
    {
        free(buffer);
    }

    bk_printf("exit code:%d\r\n", ret);

    bk_player_test_handle = NULL;
    rtos_delete_thread(NULL);
}

static void bk_player_test_url(char *url)
{
    if(bk_player_test_handle)
    {
        return;
    }
    bk_printf("%p\r\n", url);
    rtos_create_thread(&bk_player_test_handle,
                       BEKEN_APPLICATION_PRIORITY,
                       "bk_player_test",
                       (beken_thread_function_t)player_test_main,
                       2046,
                       (beken_thread_arg_t)url);
}

static void bk_player_test_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    int ret;
    char *ptr;

    ptr = argv[1];

    if (ptr && strlen(ptr) > 0)
    {
        if (strncmp(ptr, "init", 4) == 0)
        {
            bk_printf("player version : %s\n", BK_PLAYER_API_VERSION);

            ret = bk_player_init(app_event_handler);
            if (ret)
            {
                bk_printf("bk_player_init failed\r\n");
            }
        }
        else if(strncmp(ptr, "deinit", 6) == 0)
        {
            bk_player_deinit();
            bk_printf("bk_player_deinit\r\n");
        }
        else if (strncmp(ptr, "url", 3) == 0)
        {
            char *url = NULL;
            if(argc >= 3)
            {
                int len = os_strlen(argv[2]) + 1;
                if(len) {
                    url = os_malloc(len);
                    if(url)
                        os_strcpy(url, argv[2]);
                }
            }
            bk_player_test_url(url);
        }
        else if (strncmp(ptr, "add", 3) == 0)
        {
            if(argc >= 4)
            {
                char *name = argv[2];
                char *url = argv[3];
                ret = bk_player_add_music(name, url);
                bk_printf("add name=%s url=%s, ret=%d\n", name, url, ret);
            }
            else
            {
                bk_printf("usage : add NAME URL\n");
            }
        }
        else if (strncmp(ptr, "rm", 2) == 0)
        {
            if(argc >= 3)
            {
                char *name = argv[2];
                ret = bk_player_rm_music_by_name(name);
                bk_printf("rm name=%s, ret=%d\n", name, ret);
            }
            else
            {
                bk_printf("usage : rm NAME\n");
            }
        }
        else if (strncmp(ptr, "rm2", 3) == 0)
        {
            if(argc >= 3)
            {
                char *url = argv[2];
                ret = bk_player_rm_music_by_url(url);
                bk_printf("rm2 url=%s, ret=%d\n", url, ret);
            }
            else
            {
                bk_printf("usage : rm2 NAME\n");
            }
        }
        else if (strncmp(ptr, "clear", 5) == 0)
        {
            ret = bk_player_clear_music_list();
            bk_printf("clear ret=%d\n", ret);
        }
        else if (strncmp(ptr, "dump", 4) == 0)
        {
            bk_player_dump_music_list();
        }
        else if (strncmp(ptr, "play", 4) == 0)
        {
            ret = bk_player_play();
            bk_printf("play ret=%d\n", ret);
        }
        else if (strncmp(ptr, "stop", 4) == 0)
        {
            ret = bk_player_stop();
            bk_printf("stop ret=%d\n", ret);
        }
        else if (strncmp(ptr, "pause", 5) == 0)
        {
            ret = bk_player_pause();
            bk_printf("pause ret=%d\n", ret);
        }
        else if (strncmp(ptr, "resume", 6) == 0)
        {
            ret = bk_player_resume();
            bk_printf("resume ret=%d\n", ret);
        }
        else if (strncmp(ptr, "prev", 4) == 0)
        {
            ret = bk_player_prev();
            bk_printf("prev ret=%d\n", ret);
        }
        else if (strncmp(ptr, "next", 4) == 0)
        {
            ret = bk_player_next();
            bk_printf("next ret=%d\n", ret);
        }
        else if (strncmp(ptr, "mode", 4) == 0)
        {
            int mode;
            if(argc >= 3)
            {
                mode = os_strtoul(argv[2], NULL, 10);
                ret = bk_player_set_play_mode(mode);
                bk_printf("set mode to %d, ret=%d\n", mode, ret);
            }
            else
            {
                bk_printf("usage : mode MODE(0~4), 0: PLAY_ONE_SONG, 1:PLAY_SEQUENCE, 2:PLAY_ONE_SONG_LOOP, 3:PLAY_SEQUENCE_LOOP, 4:PLAY_RANDOM \n");
            }
        }
        else if(strncmp(ptr, "volume", 6) == 0)
        {
            int volume;
            if(argc >= 3)
            {
                volume = os_strtoul(argv[2], NULL, 10);
                ret = bk_player_set_spk_gain(volume);
                printf("volume ret=%d\n", ret);
            }
            else
            {
                printf("usage : volume INDEX\n");
            }
        }
        else
        {
            bk_printf("usage : add, rm, play, stop, pause, resume, seek\n");
        }
    }
}

const struct cli_command bk_player_test_clis[] = {
    {"bk_player",      "bk player ",      bk_player_test_Command},
};

void bk_player_cli_init(void)
{
    int ret;

    bk_printf("bk player cli int \r\n");
    ret = cli_register_commands(bk_player_test_clis, sizeof(bk_player_test_clis) / sizeof(struct cli_command));
    if (ret)
        bk_printf("ret: %d bk player fail.\r\n",ret);
}

#endif // CFG_USE_BK_PLAYER_TEST