#ifndef __BK_CAL_PUB_H__
#define __BK_CAL_PUB_H__

#include "bk7011_cal_pub.h"

#define TMP_PWR_TAB_LEN             39

#if (CFG_SOC_NAME == SOC_BK7238)
/* bit[15]: TXPWR flag
 *          0:  invalid
 *          1:  used
 * bit[7:0]: 8bit TXPWR pwr_gain;
 */
#define FLAG_MASK                       (0x1u)
#define FLAG_POSI                       (15)
#define GAIN_MASK                       (0xffu)
#elif (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7252N)
/* bit[7]: TXPWR flag
 *          0:  invalid
 *          1:  used
 * bit[6:0]: 7bit TXPWR pwr_gain;
 */
#define FLAG_MASK                       (0x1u)
#define FLAG_POSI                       (7)
#define GAIN_MASK                       (0x7fu)
#else
/* bit[7]: TXPWR flag
 *          0:  invalid
 *          1:  used
 * bit[4:0]: 5bit TXPWR pwr_gain;
 */
#define FLAG_MASK                       (0x1u)
#define FLAG_POSI                       (7)
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

typedef struct txpwr_st_v2
{
    UINT16 value;
} TXPWR_ST_V2, *TXPWR_PTR_V2;

#define WLAN_2_4_G_CHANNEL_NUM          (14)
#define BLE_2_4_G_CHANNEL_NUM           (40)

#if (CFG_SOC_NAME == SOC_BK7238)
extern const TXPWR_ST_V2 gtxpwr_tab_def_b[WLAN_2_4_G_CHANNEL_NUM];
extern const TXPWR_ST_V2 gtxpwr_tab_def_g[WLAN_2_4_G_CHANNEL_NUM];
extern const TXPWR_ST_V2 gtxpwr_tab_def_n_40[WLAN_2_4_G_CHANNEL_NUM];
extern const TXPWR_ST_V2 gtxpwr_tab_def_ble[BLE_2_4_G_CHANNEL_NUM];
#else
extern const TXPWR_ST gtxpwr_tab_def_b[WLAN_2_4_G_CHANNEL_NUM];
extern const TXPWR_ST gtxpwr_tab_def_g[WLAN_2_4_G_CHANNEL_NUM];
extern const TXPWR_ST gtxpwr_tab_def_n_40[WLAN_2_4_G_CHANNEL_NUM];
extern const TXPWR_ST gtxpwr_tab_def_ble[BLE_2_4_G_CHANNEL_NUM];
#endif
extern const TMP_PWR_ST tmp_pwr_tab[TMP_PWR_TAB_LEN];
extern const UINT16 shift_tab_b[4];   // 11M base,5.5M,2M,1M
extern const UINT16 shift_tab_g[8];   // 54M base,48M,36M,24M,18M,12M,9M,6M
extern const UINT16 shift_tab_n20[8]; // n20 mcs7base -  mcs0
extern const UINT16 shift_tab_n40[8]; // n40 mcs7base -  mcs0
extern const INT16 fcc_shift_tab_b_ch13[4];   // 11M base,5.5M,2M,1M
extern const INT16 fcc_shift_tab_g_ch13[8];   // 54M base,48M,36M,24M,18M,12M,9M,6M
extern const INT16 fcc_shift_tab_n20_ch13[8]; // n20 mcs7base -  mcs0
extern const INT16 fcc_shift_tab_n40_ch13[8]; // n40 mcs7base -  mcs0

extern float target_pwr_ble; //2G4 BLE target power
extern float target_pwr_11b; //2G4 11B target power
extern float target_pwr_11g; //2G4 11G target power
extern float target_pwr_n20; //2G4 N20 target power
extern float target_pwr_n40; //2G4 N40 target power
#endif
