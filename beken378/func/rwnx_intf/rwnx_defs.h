/**
 ******************************************************************************
 *
 * @file rwnx_defs.h
 *
 * @brief Main driver structure declarations for fullmac driver
 *
 * Copyright (C) RivieraWaves 2012-2019
 *
 ******************************************************************************
 */

#ifndef _RWNX_DEFS_H_
#define _RWNX_DEFS_H_

#include "mm_task.h"

#define RWNX_TX_LIFETIME_MS             100

struct rwnx_sec_phy_chan {
	u16 prim20_freq;
	u16 center_freq1;
	u16 center_freq2;
	//enum nl80211_band band;
	u8 type;
};

#define RWNX_CH_NOT_SET 0xFF
#define RWNX_INVALID_VIF 0xFF
#define RWNX_INVALID_STA 0xFF

struct rwnx_hw;

/**
 * rwnx_phy_info - Phy information
 *
 * @phy_cnt: Number of phy interface
 * @cfg: Configuration send to firmware
 * @sec_chan: Channel configuration of the second phy interface (if phy_cnt > 1)
 * @limit_bw: Set to true to limit BW on requested channel. Only set to use
 * VHT with old radio that don't support 80MHz (deprecated)
 */
struct rwnx_phy_info {
	u8 cnt;
	struct phy_cfg_tag cfg;
	struct rwnx_sec_phy_chan sec_chan;
	bool limit_bw;
};

/* Structure that will contains all RoC information received from cfg80211 */
struct rwnx_roc_elem {
	unsigned int freq;
    unsigned int duration;
    /* Used to avoid call of CFG80211 callback upon expiration of RoC */
    //bool mgmt_roc;
    /* Indicate if we have switch on the RoC channel */
    bool on_chan;
};

struct rwnx_hw {
	struct wiphy *wiphy;
	unsigned long drv_flags;
	u8 monitor_vif; 		/* FW id of the monitor interface, RWNX_INVALID_VIF if no monitor vif at fw level */

	struct mm_version_cfm version_cfm;          /* Lower layers versions - obtained via MM_VERSION_REQ */

	u8 avail_idx_map;
	u8 vif_started;
	bool adding_sta;

    /* RoC Management */
    struct rwnx_roc_elem *roc_elem;             /* Information provided by cfg80211 in its remain on channel request */
};

extern struct rwnx_hw g_rwnx_hw;
#endif /* _RWNX_DEFS_H_ */


