NAME := ble

$(NAME)_TYPE := kernel

-include $(SOURCE_ROOT)/platform/mcu/$(HOST_MCU_FAMILY)/.config

WPA_VERSION := wpa_supplicant-2.9

$(NAME)_INCLUDES := ../../../app/standalone-ap \
					../../../app/standalone-station \
					../../../driver/sdio \
					../../../driver/uart \
					../../../driver/sys_ctrl \
					../../../driver/gpio \
					../../../driver/common/reg \
					../../../driver/dma \
					../../../driver/intc \
					../../../driver/phy \
					../../../driver/rc_beken \
					../../../driver/general_dma \
					../../../driver/spidma \
					../../../driver/rw_pub \
					../../../driver/wdt \
					../../../func/sdio_intf \
					../../../func/power_save \
					../../../func/temp_detect \
					../../../func/saradc_intf \
					../../../func/spidma_intf \
					../../../func/ethernet_intf \
					../../../func/rwnx_intf \
					../../../func/rf_test \
					../../../func/joint_up

$(NAME)_INCLUDES += ../../../ip/ke \
					../../../ip/mac \
					../../../ip/lmac/src/hal \
					../../../ip/lmac/src/mm \
					../../../ip/lmac/src/ps \
					../../../ip/lmac/src/rd \
					../../../ip/lmac/src/rwnx \
					../../../ip/lmac/src/rx \
					../../../ip/lmac/src/scan \
					../../../ip/lmac/src/sta \
					../../../ip/lmac/src/tx \
					../../../ip/lmac/src/vif \
					../../../ip/lmac/src/rx/rxl \
					../../../ip/lmac/src/tx/txl \
					../../../ip/lmac/src/tpc \
					../../../ip/lmac/src/tdls \
					../../../ip/lmac/src/p2p \
					../../../ip/lmac/src/chan \
					../../../ip/lmac/src/td \
					../../../ip/umac/src/bam \
					../../../ip/umac/src/llc \
					../../../ip/umac/src/me \
					../../../ip/umac/src/rxu \
					../../../ip/umac/src/scanu \
					../../../ip/umac/src/sm \
					../../../ip/umac/src/txu \
					../../../ip/umac/src/apm \
					../../../ip/umac/src/mesh \
					../../../ip/umac/src/rc

$(NAME)_INCLUDES += ./ble_lib/ip/ble/hl/api \
					./ble_lib/ip/ble/hl/inc \
					./ble_lib/ip/ble/hl/src/gap/gapc \
					./ble_lib/ip/ble/hl/src/gap/gapm \
					./ble_lib/ip/ble/hl/src/gatt \
					./ble_lib/ip/ble/hl/src/gatt/attc \
					./ble_lib/ip/ble/hl/src/gatt/attm \
					./ble_lib/ip/ble/hl/src/gatt/atts \
					./ble_lib/ip/ble/hl/src/gatt/gattc \
					./ble_lib/ip/ble/hl/src/gatt/gattm \
					./ble_lib/ip/ble/hl/src/l2c/l2cc \
					./ble_lib/ip/ble/hl/src/l2c/l2cm \
					./ble_lib/ip/ble/ll/api \
					./ble_lib/ip/ble/ll/import/reg \
					./ble_lib/ip/ble/ll/src \
					./ble_lib/ip/ble/ll/src/llc \
					./ble_lib/ip/ble/ll/src/lld \
					./ble_lib/ip/ble/ll/src/llm \
					./ble_lib/ip/em/api \
					./ble_lib/ip/hci/api \
					./ble_lib/ip/hci/src \
					./ble_lib/ip/sch/api \
					./ble_lib/ip/sch/import \
					./ble_lib/modules/aes/api \
					./ble_lib/modules/aes/src \
					./ble_lib/modules/common/api \
					./ble_lib/modules/dbg/api \
					./ble_lib/modules/dbg/src \
					./ble_lib/modules/ecc_p256/api \
					./ble_lib/modules/h4tl/api \
					./ble_lib/modules/ke/api \
					./ble_lib/modules/ke/src \
					./ble_pub/prf \
					./platform/7231n/rwip/api \
					./platform/7231n/rwip/import/reg \
					./platform/7231n/nvds/api \
					./platform/7231n/config \
					./platform/7231n/driver/reg \
					./platform/7231n/driver/rf \
					./platform/7231n/driver/uart \
					./platform/7231n/entry \
					./arch/armv5 \
					./arch/armv5/ll \
					./arch/armv5/compiler \
					./ble_pub/profiles/comm/api \
					./ble_pub/profiles/sdp/api \
					./ble_pub/app/api \
					./ble_pub/ui

$(NAME)_INCLUDES += ../../../func/$(WPA_VERSION)/src \
					../../../func/$(WPA_VERSION)/src/ap \
					../../../func/$(WPA_VERSION)/src/utils \
					../../../func/$(WPA_VERSION)/src/common \
					../../../func/$(WPA_VERSION)/src/drivers \
					../../../func/$(WPA_VERSION)/bk_patch \
					../../../func/$(WPA_VERSION)/hostapd \
					../../../func/$(WPA_VERSION)/wpa_supplicant

$(NAME)_SOURCES :=  ./ble_lib/ip/ble/hl/src/gap/gapc/gapc.c \
					./ble_lib/ip/ble/hl/src/gap/gapc/gapc_cte.c \
					./ble_lib/ip/ble/hl/src/gap/gapc/gapc_hci.c \
					./ble_lib/ip/ble/hl/src/gap/gapc/gapc_past.c \
					./ble_lib/ip/ble/hl/src/gap/gapc/gapc_sig.c \
					./ble_lib/ip/ble/hl/src/gap/gapc/gapc_smp.c \
					./ble_lib/ip/ble/hl/src/gap/gapc/gapc_task.c \
					./ble_lib/ip/ble/hl/src/gap/gapm/gapm.c \
					./ble_lib/ip/ble/hl/src/gap/gapm/gapm_actv.c \
					./ble_lib/ip/ble/hl/src/gap/gapm/gapm_addr.c \
					./ble_lib/ip/ble/hl/src/gap/gapm/gapm_adv.c \
					./ble_lib/ip/ble/hl/src/gap/gapm/gapm_cfg.c \
					./ble_lib/ip/ble/hl/src/gap/gapm/gapm_init.c \
					./ble_lib/ip/ble/hl/src/gap/gapm/gapm_list.c \
					./ble_lib/ip/ble/hl/src/gap/gapm/gapm_per_sync.c \
					./ble_lib/ip/ble/hl/src/gap/gapm/gapm_scan.c \
					./ble_lib/ip/ble/hl/src/gap/gapm/gapm_smp.c \
					./ble_lib/ip/ble/hl/src/gap/gapm/gapm_task.c \
					./ble_lib/ip/ble/hl/src/gatt/attc/attc.c \
					./ble_lib/ip/ble/hl/src/gatt/attm/attm.c \
					./ble_lib/ip/ble/hl/src/gatt/attm/attm_db.c \
					./ble_lib/ip/ble/hl/src/gatt/atts/atts.c \
					./ble_lib/ip/ble/hl/src/gatt/gattc/gattc.c \
					./ble_lib/ip/ble/hl/src/gatt/gattc/gattc_rc.c \
					./ble_lib/ip/ble/hl/src/gatt/gattc/gattc_sdp.c \
					./ble_lib/ip/ble/hl/src/gatt/gattc/gattc_svc.c \
					./ble_lib/ip/ble/hl/src/gatt/gattc/gattc_task.c \
					./ble_lib/ip/ble/hl/src/gatt/gattm/gattm.c \
					./ble_lib/ip/ble/hl/src/gatt/gattm/gattm_task.c \
					./ble_lib/ip/ble/hl/src/l2c/l2cc/l2cc.c \
					./ble_lib/ip/ble/hl/src/l2c/l2cc/l2cc_lecb.c \
					./ble_lib/ip/ble/hl/src/l2c/l2cc/l2cc_pdu.c \
					./ble_lib/ip/ble/hl/src/l2c/l2cc/l2cc_sig.c \
					./ble_lib/ip/ble/hl/src/l2c/l2cc/l2cc_task.c \
					./ble_lib/ip/ble/hl/src/l2c/l2cm/l2cm.c \
					./ble_lib/ip/ble/hl/src/rwble_hl/rwble_hl.c \
					./ble_lib/ip/ble/ll/src/co/ble_util_buf.c \
					./ble_lib/ip/ble/ll/src/llc/llc.c \
					./ble_lib/ip/ble/ll/src/llc/llc_chmap_upd.c \
					./ble_lib/ip/ble/ll/src/llc/llc_clk_acc.c \
					./ble_lib/ip/ble/ll/src/llc/llc_con_upd.c \
					./ble_lib/ip/ble/ll/src/llc/llc_cte.c \
					./ble_lib/ip/ble/ll/src/llc/llc_dbg.c \
					./ble_lib/ip/ble/ll/src/llc/llc_disconnect.c \
					./ble_lib/ip/ble/ll/src/llc/llc_dl_upd.c \
					./ble_lib/ip/ble/ll/src/llc/llc_encrypt.c \
					./ble_lib/ip/ble/ll/src/llc/llc_feat_exch.c \
					./ble_lib/ip/ble/ll/src/llc/llc_hci.c \
					./ble_lib/ip/ble/ll/src/llc/llc_le_ping.c \
					./ble_lib/ip/ble/ll/src/llc/llc_llcp.c \
					./ble_lib/ip/ble/ll/src/llc/llc_past.c \
					./ble_lib/ip/ble/ll/src/llc/llc_phy_upd.c \
					./ble_lib/ip/ble/ll/src/llc/llc_task.c \
					./ble_lib/ip/ble/ll/src/llc/llc_ver_exch.c \
					./ble_lib/ip/ble/ll/src/lld/lld.c \
					./ble_lib/ip/ble/ll/src/lld/lld_adv.c \
					./ble_lib/ip/ble/ll/src/lld/lld_con.c \
					./ble_lib/ip/ble/ll/src/lld/lld_init.c \
					./ble_lib/ip/ble/ll/src/lld/lld_per_adv.c \
					./ble_lib/ip/ble/ll/src/lld/lld_scan.c \
					./ble_lib/ip/ble/ll/src/lld/lld_sync.c \
					./ble_lib/ip/ble/ll/src/lld/lld_test.c \
					./ble_lib/ip/ble/ll/src/llm/llm.c \
					./ble_lib/ip/ble/ll/src/llm/llm_adv.c \
					./ble_lib/ip/ble/ll/src/llm/llm_hci.c \
					./ble_lib/ip/ble/ll/src/llm/llm_init.c \
					./ble_lib/ip/ble/ll/src/llm/llm_scan.c \
					./ble_lib/ip/ble/ll/src/llm/llm_task.c \
					./ble_lib/ip/ble/ll/src/llm/llm_test.c \
					./ble_lib/ip/hci/src/hci.c \
					./ble_lib/ip/hci/src/hci_fc.c \
					./ble_lib/ip/hci/src/hci_msg.c \
					./ble_lib/ip/hci/src/hci_tl.c \
					./ble_lib/ip/sch/src/sch_alarm.c \
					./ble_lib/ip/sch/src/sch_arb.c \
					./ble_lib/ip/sch/src/sch_plan.c \
					./ble_lib/ip/sch/src/sch_prog.c \
					./ble_lib/ip/sch/src/sch_slice.c \
					./ble_lib/modules/aes/src/ble_aes.c \
					./ble_lib/modules/aes/src/ble_aes_c1.c \
					./ble_lib/modules/aes/src/ble_aes_ccm.c \
					./ble_lib/modules/aes/src/ble_aes_cmac.c \
					./ble_lib/modules/aes/src/ble_aes_f4.c \
					./ble_lib/modules/aes/src/ble_aes_f5.c \
					./ble_lib/modules/aes/src/ble_aes_f6.c \
					./ble_lib/modules/aes/src/ble_aes_g2.c \
					./ble_lib/modules/aes/src/ble_aes_k1.c \
					./ble_lib/modules/aes/src/ble_aes_k2.c \
					./ble_lib/modules/aes/src/ble_aes_k3.c \
					./ble_lib/modules/aes/src/ble_aes_k4.c \
					./ble_lib/modules/aes/src/ble_aes_rpa.c \
					./ble_lib/modules/aes/src/ble_aes_s1.c \
					./ble_lib/modules/common/src/common_list.c \
					./ble_lib/modules/common/src/common_utils.c \
					./ble_lib/modules/dbg/src/dbg.c \
					./ble_lib/modules/dbg/src/dbg_iqgen.c \
					./ble_lib/modules/dbg/src/dbg_mwsgen.c \
					./ble_lib/modules/dbg/src/dbg_swdiag.c \
					./ble_lib/modules/dbg/src/dbg_task.c \
					./ble_lib/modules/dbg/src/dbg_trc.c \
					./ble_lib/modules/dbg/src/dbg_trc_mem.c \
					./ble_lib/modules/dbg/src/dbg_trc_tl.c \
					./ble_lib/modules/ecc_p256/src/ecc_p256.c \
					./ble_lib/modules/h4tl/src/h4tl.c \
					./ble_lib/modules/ke/src/kernel.c \
					./ble_lib/modules/ke/src/kernel_event.c \
					./ble_lib/modules/ke/src/kernel_mem.c \
					./ble_lib/modules/ke/src/kernel_msg.c \
					./ble_lib/modules/ke/src/kernel_queue.c \
					./ble_lib/modules/ke/src/kernel_task.c \
					./ble_lib/modules/ke/src/kernel_timer.c \
					./ble_lib/modules/rwip/rwip_driver.c \
					./ble_pub/prf/prf.c \
					./ble_pub/prf/prf_utils.c \
					./ble_pub/profiles/comm/src/comm.c \
					./ble_pub/profiles/comm/src/comm_task.c \
					./ble_pub/profiles/sdp/src/sdp_common.c \
					./ble_pub/profiles/sdp/src/sdp_comm_task.c \
					./ble_pub/app/src/app_comm.c \
					./ble_pub/app/src/app_ble.c \
					./ble_pub/app/src/app_task.c \
					./ble_pub/app/src/app_ble_init.c \
					./ble_pub/app/src/app_sdp.c \
					./ble_pub/ui/ble_ui.c \
					./platform/7231n/rwip/src/rwip.c \
					./platform/7231n/rwip/src/rwble.c \
					./platform/7231n/entry/ble_main.c \
					./platform/7231n/driver/rf/rf_xvr.c \
					./platform/7231n/driver/rf/ble_rf_port.c \
					./platform/7231n/driver/uart/uart_ble.c
