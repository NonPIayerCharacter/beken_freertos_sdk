set(incs . include plugins/sources/hls)
set(srcs "")
set(plugins "")

list(APPEND plugins
	"plugins/sources/hls/source_hls.c"
	"plugins/sources/hls/m3u8_work.c"
	"plugins/sources/source_file.c"
	"plugins/sources/source_net.c"
	"plugins/sinks/sink_file.c"
	"plugins/sinks/sink_device.c"
	"plugins/codecs/codec_wav.c"
	"plugins/codecs/codec_aac.c"
	"plugins/codecs/codec_ts.c"
	"plugins/codecs/codec_amr.c"
	"plugins/codecs/codec_mp3.c"
	"plugins/codecs/codec_flac.c"
	"plugins/codecs/codec_ogg.c"
	"plugins/codecs/frame_info_list.c"
	)

if(CONFIG_PLAYER)
	list(APPEND incs .)
	list(APPEND srcs
		"core/play_manager.c"
		"core/play_sm.c"
		"core/plugin_manager.c"
		"core/music_list.c"
		"core/ring_buffer.c"
		"core/linked_list.c"
		"core/player_osal.c"
		"core/source.c"
		"core/codec.c"
		"core/sink.c"
		"core/player_mem.c"
		${plugins})
endif()

armino_component_register(SRCS "${srcs}"
		INCLUDE_DIRS "${incs}"
		PRIV_REQUIRES bk_common bk_cli avdk_libs bk_vfs webclient lwip_intf_v2_1 avdk mbedtls
		)
