set(incs . ../../../../include/avdk)

set(srcs
	bitstream.c
	buffers.c
	dct32.c
	dequant.c
	dqchan.c
	huffman.c
	hufftabs.c
	imdct.c
	mp3dec.c
	mp3tabs.c
	polyphase.c
	scalfact.c
	stproc.c
	subband.c
	trigtabs.c
	)

armino_build_get_property(armino_soc ARMINO_SOC)
set(out_dir $ENV{ARMINO_PATH}/../components/avdk_libs/${armino_soc}/libs)
get_filename_component(config_component_name ${CMAKE_CURRENT_LIST_DIR} NAME)

armino_component_register(INTERNAL_LIB SRCS "${srcs}" INCLUDE_DIRS "${incs}" PRIV_REQUIRES audio_osi)

armino_component_get_target(target "${config_component_name}")
set_target_properties(${target} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${out_dir})