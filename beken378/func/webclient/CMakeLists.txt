set(incs .)
set(srcs)


if (CONFIG_WEBCLIENT)
	list(APPEND srcs
		webclient_test.c
		)

	if (CONFIG_DYNAMIC_LOAD_WEBCLIENT)
		list(APPEND srcs
			webclient_mod/webclient_mod.c
			)
	else()
		list(APPEND srcs
			webclient.c
			)
	endif()
endif()


armino_component_register(SRCS "${srcs}"
	INCLUDE_DIRS "${incs}"
	PRIV_REQUIRES lwip_intf_v2_1 mbedtls udynlink bk_vfs
	)

if (CONFIG_DYNAMIC_LOAD_WEBCLIENT)


add_subdirectory(webclient_mod)

endif()

