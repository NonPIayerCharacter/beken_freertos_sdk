

set(incs .)
set(srcs)


if (CONFIG_WEBCLIENT)
if (CONFIG_DYNAMIC_LOAD_WEBCLIENT)
	list(APPEND srcs
		webclient.c
		)
endif()
endif()

set(DYN_OPTS)
set(DYN_OPTS ${DYN_OPTS} -fPIE)
set(DYN_OPTS ${DYN_OPTS} -msingle-pic-base)
set(DYN_OPTS ${DYN_OPTS} -mpic-register=r9)
set(DYN_OPTS ${DYN_OPTS} -fomit-frame-pointer)
set(DYN_OPTS ${DYN_OPTS} -mno-pic-data-is-text-relative )
set(DYN_OPTS ${DYN_OPTS} -mthumb)
set(DYN_OPTS ${DYN_OPTS} -mlong-calls)
set(DYN_OPTS ${DYN_OPTS} -fno-stack-protector)


###############For Cpp Module################
# set(DYN_OPTS ${DYN_OPTS} -fno-use-cxa-atexit)
# set(DYN_OPTS ${DYN_OPTS} -fno-unwind-tables)
# set(DYN_OPTS ${DYN_OPTS} -fno-asynchronous-unwind-tables)
###############For Cpp################

set(priv_incs lwip_intf_v2_1 mbedtls udynlink bk_vfs)

get_filename_component(config_component_name ${CMAKE_CURRENT_LIST_DIR} NAME)
set(config_component_name ${config_component_name})

armino_component_register_ex(INTERNAL_LIB SRCS "${srcs}" INCLUDE_DIRS "${incs}" PRIV_REQUIRES "${priv_incs}" CONFIG_COMPONENT_NAME "${config_component_name}")
armino_component_get_target(target "${config_component_name}")



target_compile_options(${target} PRIVATE ${DYN_OPTS})


######################################################################################
set(current_dir ${COMPONENT_DIR}/webclient_mod)
set(base_dir $ENV{ARMINO_PATH})
set(output_dir ${CMAKE_BINARY_DIR}/armino/webclient/webclient_mod)
set(build_dir ${CMAKE_BINARY_DIR})
set(lib_dir ${output_dir})
set(toolchain ${CONFIG_TOOLCHAIN_PATH})
if(NOT IS_ABSOLUTE ${toolchain})
	set(toolchain ${armino_path}/${toolchain})
endif()

set(keep_api_file ${current_dir}/webclient_mod_keep_api.c)

MESSAGE(STATUS "current_dir is ${current_dir}")
MESSAGE(STATUS "base_dir is ${base_dir}")
MESSAGE(STATUS "build_dir is ${build_dir}")
MESSAGE(STATUS "lib_dir is ${lib_dir}")
MESSAGE(STATUS "toolchain is ${toolchain}")
MESSAGE(STATUS "keep_api_file is ${keep_api_file}")

set(mod_build_cmd ${current_dir}/build_webclient_module.sh  ${base_dir} ${build_dir} ${lib_dir} ${current_dir} ${toolchain} ${keep_api_file})
MESSAGE(STATUS "${mod_build_cmd}")
MESSAGE(STATUS "${dyn_srcs}")


add_custom_target(mod_build DEPENDS ${output_dir}/webclient_mod.bin)
add_custom_command(
				TARGET ${target}
				POST_BUILD
				COMMAND ${mod_build_cmd}
				OUTPUT_VARIABLE output
				RESULT_VARIABLE result)

MESSAGE(STATUS "${output}")
MESSAGE(STATUS "result: ${result}")

add_dependencies(mod_build ${target})
