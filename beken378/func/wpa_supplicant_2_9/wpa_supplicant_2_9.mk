NAME := wpa_supplicant_2_9

$(NAME)_TYPE := kernel

-include $(SOURCE_ROOT)/platform/mcu/$(HOST_MCU_FAMILY)/.config

$(NAME)_INCLUDES += ../../ip/lmac/src/mm\
                    ../../ip/mac\
                    ../../ip/lmac/src/hal \
                    ../../ip/lmac/src/ps \
                    ../../ip/lmac/src/rd \
                    ../../ip/lmac/src/rwnx \
                    ../../ip/lmac/src/rx \
                    ../../ip/lmac/src/scan \
                    ../../ip/lmac/src/sta \
                    ../../ip/lmac/src/tx \
                    ../../ip/lmac/src/vif \
                    ../../ip/lmac/src/rx/rxl \
                    ../../ip/lmac/src/tx/txl \
                    ../../ip/lmac/src/tpc \
                    ../../ip/lmac/src/tdls \
                    ../../ip/lmac/src/p2p \
                    ../../ip/lmac/src/chan \
                    ../../ip/lmac/src/td \
                    ../../ip/umac/src/bam \
                    ../../ip/umac/src/llc \
                    ../../ip/umac/src/me \
                    ../../ip/umac/src/rxu \
                    ../../ip/umac/src/scanu \
                    ../../ip/umac/src/sm \
                    ../../ip/umac/src/txu \
                    ../../ip/umac/src/apm \
                    ../../ip/umac/src/mesh \
                    ../../ip/umac/src/rc\
                    ../../ip/ke

$(NAME)_INCLUDES += ../../driver/dma\
                    ../../driver/common/reg\
                    ../../driver/phy\
                    ../../driver/uart\
                    ../../func/user_driver\
                    ../../func/temp_detect\
                    ../../func/rwnx_intf\
                    ../../app/standalone-station\
                    ../../app/standalone-ap\
                    ../../demo


$(NAME)_INCLUDES += src \
                    src/ap \
                    src/utils \
                    src/common \
                    src/drivers \
                    bk_patch \
                    hostapd \
                    wpa_supplicant

$(NAME)_SOURCES +=  bk_patch/ddrv.c \
                    bk_patch/signal.c \
                    bk_patch/sk_intf.c \
                    bk_patch/fake_socket.c \
                    src/common/hw_features_common.c \
                    src/common/ieee802_11_common.c \
                    src/common/wpa_common.c \
                    src/common/notifier.c \
                    src/common/wpa_psk_cache.c \
                    src/common/ocv.c \
                    src/crypto/aes-unwrap.c \
                    src/crypto/rc4.c \
                    src/crypto/sha1-pbkdf2.c \
                    src/crypto/sha1-prf.c \
                    src/crypto/tls_none.c \
                    src/crypto/aes-wrap.c \
                    src/drivers/driver_beken.c \
                    src/drivers/driver_common.c \
                    src/drivers/drivers.c \
                    src/l2_packet/l2_packet_none.c \
                    src/rsn_supp/pmksa_cache.c \
                    src/rsn_supp/wpa.c \
                    src/rsn_supp/wpa_ie.c \
                    src/utils/common.c \
                    src/utils/eloop.c \
                    src/utils/os_none.c \
                    src/utils/wpabuf.c \
                    src/utils/wpa_debug.c \
                    src/ap/ap_config.c \
                    src/ap/ap_drv_ops.c \
                    src/ap/ap_list.c \
                    src/ap/ap_mlme.c \
                    src/ap/beacon.c \
                    src/ap/drv_callbacks.c \
                    src/ap/hostapd.c \
                    src/ap/hw_features.c \
                    src/ap/ieee802_11.c \
                    src/ap/ieee802_11_auth.c \
                    src/ap/ieee802_11_ht.c \
                    src/ap/ieee802_11_shared.c \
                    src/ap/ieee802_1x.c \
                    src/ap/pmksa_cache_auth.c \
                    src/ap/sta_info.c \
                    src/ap/tkip_countermeasures.c \
                    src/ap/utils.c \
                    src/ap/wmm.c \
                    src/ap/wpa_auth.c \
                    src/ap/wpa_auth_glue.c \
                    src/ap/wpa_auth_ie.c \
                    hostapd/main_none.c \
                    wpa_supplicant/blacklist.c \
                    wpa_supplicant/bss.c \
                    wpa_supplicant/config.c \
                    wpa_supplicant/config_none.c \
                    wpa_supplicant/events.c \
                    wpa_supplicant/main_supplicant.c \
                    wpa_supplicant/notify.c \
                    wpa_supplicant/wmm_ac.c \
                    wpa_supplicant/wpa_scan.c \
                    wpa_supplicant/wpa_supplicant.c \
                    wpa_supplicant/wpas_glue.c \
                    wpa_supplicant/ctrl_iface.c \
                    wpa_supplicant/wlan.c \
                    wpa_supplicant/sme.c\
                    wpa_supplicant/op_classes.c\
                    src/crypto/crypto_ali.c

ifeq ($(CFG_WPA3),1)
$(NAME)_INCLUDES += ../wolfssl \

$(NAME)_SOURCES +=  src/crypto/crypto_wolfssl.c \
                    src/common/sae.c \
                    src/common/dragonfly.c \
                    src/crypto/aes-ctr.c \
                    src/crypto/aes-omac1.c \
                    src/crypto/aes-siv.c \
                    src/crypto/dh_group5.c \
                    src/crypto/dh_groups.c \
                    src/crypto/sha256-prf.c \
		    src/rsn_supp/pmksa_cache.c \
		    wpa_supplicant/sme.c \
		    ../wolfssl/wolfcrypt/src/wolfmath.c \
		    ../wolfssl/wolfcrypt/src/memory.c \
		    ../wolfssl/wolfcrypt/src/tfm.c \
		    ../wolfssl/wolfcrypt/src/ecc.c \
		    ../wolfssl/wolfcrypt/src/random.c \
		    ../wolfssl/wolfcrypt/src/sha256.c

#ifeq ($(CFG_OWE),1)
$(NAME)_SOURCES +=  src/crypto/sha256-kdf.c \
                    src/crypto/sha384-kdf.c \
                    src/crypto/sha512-kdf.c \
		    ../wolfssl/wolfcrypt/src/sha512.c \
		    ../wolfssl/wolfcrypt/src/md5.c \
		    ../wolfssl/wolfcrypt/src/hmac.c \
		    ../wolfssl/wolfcrypt/src/sha.c
#endif
endif

ifeq ($(CFG_WIFI_P2P_GO),1)
$(NAME)_SOURCES += src/ap/wps_hostapd.c \
                    src/ap/bss_load.c \
                    src/ap/eap_user_db.c \
                    src/eap_server/eap_server.c \
                    src/eap_server/eap_server_methods.c \
                    src/eap_server/eap_server_wsc.c \
                    src/eap_server/eap_server_identity.c \
                    src/eapol_auth/eapol_auth_sm.c
endif
