#ifndef __BK_CAL_PUB_H__
#define __BK_CAL_PUB_H__

#include "bk7011_cal_pub.h"

#define TMP_PWR_TAB_LEN             39

/* bit[7]: TXPWR flag
 *          0:  invalid
 *          1:  used
 * bit[4:0]: 5bit TXPWR pwr_gain;
 */
#define FLAG_MASK                       (0x1u)
#define FLAG_POSI                       (7)
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
#define GAIN_MASK                       (0x7fu)
#else
#define GAIN_MASK                       (0x1fu)
#endif
#define GET_TXPWR_GAIN(p)               ((p)->value & GAIN_MASK)
#define SET_TXPWR_GAIN(p, gain)         {(p)->value &= (~GAIN_MASK); \
                                        (p)->value |= ((gain)&GAIN_MASK);}

#define GET_TXPWR_FLAG(p)               (((p)->value>>FLAG_POSI)&FLAG_MASK)
#define SET_TXPWR_FLAG(p, flag)         {(p)->value &= (~(FLAG_MASK<<FLAG_POSI)); \
                                        (p)->value |= ((flag&FLAG_MASK)<<FLAG_POSI);}
#define INIT_TXPWR_VALUE(gain, flag)    {(((flag&FLAG_MASK)<<FLAG_POSI)|(gain&GAIN_MASK))}

#define TXPWR_ELEM_INUSED               (0)
#define TXPWR_ELEM_UNUSED               (1)

typedef struct txpwr_st
{
    UINT8 value;
} TXPWR_ST, *TXPWR_PTR;

#define WLAN_2_4_G_CHANNEL_NUM          (14)
#define BLE_2_4_G_CHANNEL_NUM           (40)

extern const TXPWR_ST gtxpwr_tab_def_b[WLAN_2_4_G_CHANNEL_NUM];
extern const TXPWR_ST gtxpwr_tab_def_g[WLAN_2_4_G_CHANNEL_NUM];
extern const TXPWR_ST gtxpwr_tab_def_n_40[WLAN_2_4_G_CHANNEL_NUM];
extern const TXPWR_ST gtxpwr_tab_def_ble[BLE_2_4_G_CHANNEL_NUM];
extern const TMP_PWR_ST tmp_pwr_tab[TMP_PWR_TAB_LEN];
extern const UINT16 shift_tab_b[4];   // 11M base,5.5M,2M,1M
extern const UINT16 shift_tab_g[8];   // 54M base,48M,36M,24M,18M,12M,9M,6M
extern const UINT16 shift_tab_n20[8]; // n20 mcs7base -  mcs0
extern const UINT16 shift_tab_n40[8]; // n40 mcs7base -  mcs0

#endif
