#include "include.h"

#if (CFG_SOC_NAME != SOC_BK7231)
#include "bk7011_cal_pub.h"
#include "bk_cal_pub.h"

/********************************  power table ********************************/
#if (CFG_SOC_NAME == SOC_BK7231U) || (CFG_SOC_NAME == SOC_BK7221U)
const TXPWR_ST gtxpwr_tab_def_b[WLAN_2_4_G_CHANNEL_NUM] =
{
    INIT_TXPWR_VALUE(20, TXPWR_ELEM_INUSED),  // ch1  inused
    INIT_TXPWR_VALUE(20, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(20, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(19, TXPWR_ELEM_UNUSED),  // ch4
    INIT_TXPWR_VALUE(19, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(19, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(18, TXPWR_ELEM_UNUSED),  // ch7
    INIT_TXPWR_VALUE(18, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(18, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(17, TXPWR_ELEM_UNUSED),  // ch10
    INIT_TXPWR_VALUE(17, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(17, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_INUSED),  // ch13  inused
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
};
#elif (CFG_SOC_NAME == SOC_BK7231N)
const TXPWR_ST gtxpwr_tab_def_b[WLAN_2_4_G_CHANNEL_NUM] =
{
    INIT_TXPWR_VALUE(19, TXPWR_ELEM_INUSED),  // ch1  inused
    INIT_TXPWR_VALUE(20, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(20, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(20, TXPWR_ELEM_UNUSED),  // ch4
    INIT_TXPWR_VALUE(20, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(20, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(20, TXPWR_ELEM_UNUSED),  // ch7
    INIT_TXPWR_VALUE(22, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(23, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(23, TXPWR_ELEM_UNUSED),  // ch10
    INIT_TXPWR_VALUE(24, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_INUSED),  // ch13  inused
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
};
#elif (CFG_SOC_NAME == SOC_BK7238)
const TXPWR_ST gtxpwr_tab_def_b[WLAN_2_4_G_CHANNEL_NUM] = {
    INIT_TXPWR_VALUE(75, TXPWR_ELEM_INUSED),  // ch1  inused
    INIT_TXPWR_VALUE(75, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(75, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(75, TXPWR_ELEM_UNUSED),  // ch4
    INIT_TXPWR_VALUE(75, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(75, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(75, TXPWR_ELEM_UNUSED),  // ch7
    INIT_TXPWR_VALUE(75, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(75, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(75, TXPWR_ELEM_UNUSED),  // ch10
    INIT_TXPWR_VALUE(75, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(75, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(75, TXPWR_ELEM_INUSED),  // ch13  inused
    INIT_TXPWR_VALUE(75, TXPWR_ELEM_UNUSED),
};
#endif // (CFG_SOC_NAME == SOC_BK7231U)

#if (CFG_SOC_NAME == SOC_BK7231U) || (CFG_SOC_NAME == SOC_BK7221U)
const TXPWR_ST gtxpwr_tab_def_g[WLAN_2_4_G_CHANNEL_NUM] =
{
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_INUSED),  // ch1  inused
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(15, TXPWR_ELEM_UNUSED),  // ch4
    INIT_TXPWR_VALUE(15, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(15, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(14, TXPWR_ELEM_UNUSED),  // ch7
    INIT_TXPWR_VALUE(14, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(14, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(13, TXPWR_ELEM_UNUSED),  // ch10
    INIT_TXPWR_VALUE(13, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(13, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(12, TXPWR_ELEM_INUSED),  // ch13  inused
    INIT_TXPWR_VALUE(12, TXPWR_ELEM_UNUSED),
};

const TXPWR_ST gtxpwr_tab_def_n_40[WLAN_2_4_G_CHANNEL_NUM] =
{
    INIT_TXPWR_VALUE(8, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(8, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(8, TXPWR_ELEM_UNUSED),  // ch3
    INIT_TXPWR_VALUE(8, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(8, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(8, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(7, TXPWR_ELEM_UNUSED),  // ch7
    INIT_TXPWR_VALUE(7, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(7, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(6, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(5, TXPWR_ELEM_UNUSED),  // ch11
    INIT_TXPWR_VALUE(5, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(5, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(5, TXPWR_ELEM_UNUSED),
};

#elif (CFG_SOC_NAME == SOC_BK7231N)
const TXPWR_ST gtxpwr_tab_def_g[WLAN_2_4_G_CHANNEL_NUM] =
{
    INIT_TXPWR_VALUE(38, TXPWR_ELEM_INUSED),  // ch1  inused
    INIT_TXPWR_VALUE(39, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(39, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(39, TXPWR_ELEM_UNUSED),  // ch4
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),  // ch7
    INIT_TXPWR_VALUE(43, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(43, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(45, TXPWR_ELEM_UNUSED),  // ch10
    INIT_TXPWR_VALUE(47, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(48, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(49, TXPWR_ELEM_INUSED),  // ch13  inused
    INIT_TXPWR_VALUE(49, TXPWR_ELEM_UNUSED),
};

const TXPWR_ST gtxpwr_tab_def_n_40[WLAN_2_4_G_CHANNEL_NUM] =
{
    INIT_TXPWR_VALUE(35, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(35, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(35, TXPWR_ELEM_UNUSED),  // ch3
    INIT_TXPWR_VALUE(37, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(38, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(38, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(38, TXPWR_ELEM_UNUSED),  // ch7
    INIT_TXPWR_VALUE(39, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(42, TXPWR_ELEM_UNUSED),  // ch11
    INIT_TXPWR_VALUE(42, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(42, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(42, TXPWR_ELEM_UNUSED),
};

#elif (CFG_SOC_NAME == SOC_BK7238)
const TXPWR_ST gtxpwr_tab_def_g[WLAN_2_4_G_CHANNEL_NUM] = {
    INIT_TXPWR_VALUE(92, TXPWR_ELEM_INUSED),  // ch1  inused
    INIT_TXPWR_VALUE(92, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(92, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(92, TXPWR_ELEM_UNUSED),  // ch4
    INIT_TXPWR_VALUE(92, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(92, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(92, TXPWR_ELEM_UNUSED),  // ch7
    INIT_TXPWR_VALUE(92, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(92, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(92, TXPWR_ELEM_UNUSED),  // ch10
    INIT_TXPWR_VALUE(92, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(92, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(92, TXPWR_ELEM_INUSED),  // ch13  inused
    INIT_TXPWR_VALUE(92, TXPWR_ELEM_UNUSED),
};
#endif // (CFG_SOC_NAME == SOC_BK7231)
#if (CFG_SOC_NAME == SOC_BK7231N)
const TXPWR_ST gtxpwr_tab_def_ble[BLE_2_4_G_CHANNEL_NUM] =
{
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),  // ch0 2402  inused
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),  // ch1 2404
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),  // ch4 2410
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),  // ch9 2420
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),  // ch14 2430
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_INUSED),  // ch19 2440 inused
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),  // ch24 2450
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),  // ch29 2460
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),  // ch34 2470
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(25, TXPWR_ELEM_UNUSED),  // ch39 2480 inused
};
#elif (CFG_SOC_NAME == SOC_BK7238)
const TXPWR_ST gtxpwr_tab_def_ble[BLE_2_4_G_CHANNEL_NUM] = {
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),  // ch0 2402  inused
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),  // ch1 2404
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),  // ch4 2410
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),  // ch9 2420
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),  // ch14 2430
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_INUSED),  // ch19 2440 inused
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),  // ch24 2450
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),  // ch29 2460
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),  // ch34 2470
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(40, TXPWR_ELEM_UNUSED),  // ch39 2480 inused
};
#else
const TXPWR_ST gtxpwr_tab_def_ble[BLE_2_4_G_CHANNEL_NUM] =
{
    INIT_TXPWR_VALUE(20, TXPWR_ELEM_UNUSED),  // ch0 2402  inused
    INIT_TXPWR_VALUE(20, TXPWR_ELEM_UNUSED),  // ch1 2404
    INIT_TXPWR_VALUE(20, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(20, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(20, TXPWR_ELEM_UNUSED),  // ch4 2410
    INIT_TXPWR_VALUE(19, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(19, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(19, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(19, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(19, TXPWR_ELEM_UNUSED),  // ch9 2420
    INIT_TXPWR_VALUE(18, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(18, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(18, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(18, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(18, TXPWR_ELEM_UNUSED),  // ch14 2430
    INIT_TXPWR_VALUE(17, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(17, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(17, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(17, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_INUSED),  // ch19 2440 inused
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),  // ch24 2450
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),  // ch29 2460
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),  // ch34 2470
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),
    INIT_TXPWR_VALUE(16, TXPWR_ELEM_UNUSED),  // ch39 2480 inused
};
#endif

/****************************** temperature table  ****************************/
#if CFG_USE_TEMPERATURE_DETECT
#if (CFG_SOC_NAME == SOC_BK7231N)
const TMP_PWR_ST tmp_pwr_tab[TMP_PWR_TAB_LEN] =
{
    //trx0x0c[12:15], shift_b, shift_g, shift_ble, xtal_c_delta
#if (CFG_XTAL_85DEGREE)
    {  0x00,       -6,      -9,        0,       -18},   // 0     ,-40    -35
    {  0x00,       -6,      -9,        0,       -10},   // 1     ,-35    -30
    {  0x00,       -5,      -9,        0,        -4},   // 2     ,-30    -25
    {  0x00,       -5,      -8,        0,         0},   // 3     ,-25    -20
    {  0x00,       -5,      -8,        0,         3},   // 4     ,-20    -15
    {  0x00,       -4,      -6,        0,         4},   // 5     ,-15    -10
    {  0x00,       -4,      -6,        0,         5},   // 6     ,-10    -5
    {  0x00,       -3,      -5,        0,         5},   // 7     ,-5     0
    {  0x00,       -3,      -4,        0,         5},   // 8     ,0      5
    {  0x00,       -2,      -4,        0,         5},   // 9     ,5      10
    {  0x00,       -2,      -3,        0,         4},   // 10    ,10     15
    {  0x00,       -1,      -3,        0,         3},   // 11    ,15     20
    {  0x00,       -1,      -1,        0,         1},   // 12    ,20     25
    {  0x00,        0,       0,        0,         0},   // 13    ,25     30
    {  0x00,        0,       0,        0,        -3},   // 14    ,30     35
    {  0x00,        0,       1,        0,        -5},   // 15    ,35     40
    {  0x00,        1,       1,        0,        -6},   // 16    ,40     45
    {  0x00,        1,       2,        0,        -7},   // 17    ,45     50
    {  0x00,        2,       2,        0,        -8},   // 18    ,50     55
    {  0x00,        2,       2,        0,        -7},   // 19    ,55     60
    {  0x00,        3,       3,        0,        -6},   // 20    ,60     65
    {  0x00,        3,       3,        0,        -4},   // 21    ,65     70
    {  0x00,        4,       4,        0,         0},   // 22    ,70     75
    {  0x00,        4,       5,        0,         7},   // 23    ,75     80
    {  0x00,        4,       5,        0,        14},   // 24    ,80     85
    {  0x00,        5,       5,        0,        22},   // 25    ,85     90
    {  0x00,        5,       7,        0,        33},   // 26    ,90     95
    {  0x00,        6,       8,        1,        47},   // 27    ,95     100
    {  0x00,        6,       9,        1,        63},   // 28    ,100    105
    {  0x00,        7,       9,        2,        84},   // 29    ,105    110
    {  0x00,        7,      10,        2,       106},   // 30    ,110    115
    {  0x00,        8,      10,        3,       127},   // 31    ,115
    {  0x00,        8,      11,        3,       127},   // 32    ,120
    {  0x00,        9,      12,        3,       127},   // 33    ,125
    {  0x00,        9,      12,        3,       127},   // 34    ,130
    {  0x00,        9,      13,        3,       127},   // 35    ,135
    {  0x00,       10,      13,        3,       127},   // 36    ,140
    {  0x00,       10,      13,        3,       127},   // 37    ,145
    {  0x00,       10,      13,        3,       127},   // 38    ,150
#else
    {  0x00,        -6,      -9,       0,        -1},   // 0     ,-40    -35
    {  0x00,        -6,      -9,       0,         1},   // 1     ,-35    -30
    {  0x00,        -5,      -9,       0,         3},   // 2     ,-30    -25
    {  0x00,        -5,      -8,       0,         4},   // 3     ,-25    -20
    {  0x00,        -5,      -8,       0,         5},   // 4     ,-20    -15
    {  0x00,        -4,      -6,       0,         6},   // 5     ,-15    -10
    {  0x00,        -4,      -6,       0,         7},   // 6     ,-10    -5
    {  0x00,        -3,      -5,       0,         7},   // 7     ,-5     0
    {  0x00,        -3,      -4,       0,         7},   // 8     ,0      5
    {  0x00,        -2,      -4,       0,         6},   // 9     ,5      10
    {  0x00,        -2,      -3,       0,         5},   // 10    ,10     15
    {  0x00,        -1,      -3,       0,         4},   // 11    ,15     20
    {  0x00,        -1,      -1,       0,         3},   // 12    ,20     25
    {  0x00,        0,       0,        0,         0},   // 13    ,25     30
    {  0x00,        0,       0,        0,         0},   // 14    ,30     35
    {  0x00,        0,       1,        0,         -1},   // 15    ,35    40
    {  0x00,        1,       1,        0,         -2},   // 16    ,40    45
    {  0x00,        1,       2,        0,         -2},   // 17    ,45    50
    {  0x00,        2,       2,        0,         -2},   // 18    ,50    55
    {  0x00,        2,       2,        0,         -2},   // 19    ,55    60
    {  0x00,        3,       3,        0,         -2},   // 20    ,60    65
    {  0x00,        3,       3,        0,         -1},   // 21    ,65    70
    {  0x00,        4,       4,        0,         0},   // 22    ,70   75
    {  0x00,        4,       5,        0,         2},   // 23    ,75   80
    {  0x00,        4,       5,        0,         5},   // 24    ,80   85
    {  0x00,        5,       5,        0,         7},   // 25    ,85  90
    {  0x00,        5,       7,        0,         11},   // 26    ,90  95
    {  0x00,        6,       8,        1,        17},   // 27    ,95  100
    {  0x00,        6,       9,        1,        24},   // 28    ,100  105
    {  0x00,        7,       9,        2,        32},   // 29    ,105  110
    {  0x00,        7,      10,        2,        43},   // 30    ,110  115
    {  0x00,        8,      10,        3,        58},   // 31    ,115
    {  0x00,        8,      11,        3,        77},   // 32    ,120
    {  0x00,        9,      12,        3,        99},   // 33    ,125
    {  0x00,        9,      12,        3,        117},   // 34    ,130
    {  0x00,        9,      13,        3,        117},   // 35    ,135
    {  0x00,        10,      13,        3,        117},   // 36    ,140
    {  0x00,        10,      13,        3,        117},   // 37    ,145
    {  0x00,        10,      13,        3,        117},   // 38    ,150
#endif
};
#elif (CFG_SOC_NAME == SOC_BK7238)
const TMP_PWR_ST tmp_pwr_tab[TMP_PWR_TAB_LEN] = {
//trx0x0c[12:15], shift_b, shift_g, shift_ble, xtal_c_delta
#if (CFG_XTAL_85DEGREE)
    {  0x00,       -6,      -9,        0,       -18},   // 0     ,-40    -35
    {  0x00,       -6,      -9,        0,       -10},   // 1     ,-35    -30
    {  0x00,       -5,      -9,        0,        -4},   // 2     ,-30    -25
    {  0x00,       -5,      -8,        0,         0},   // 3     ,-25    -20
    {  0x00,       -5,      -8,        0,         3},   // 4     ,-20     -15
    {  0x00,       -4,      -6,        0,         4},   // 5     ,-15  -10
    {  0x00,       -4,      -6,        0,         5},   // 6     ,-10   -5
    {  0x00,       -3,      -5,        0,         5},   // 7     ,-5     0
    {  0x00,       -3,      -4,        0,         5},   // 8     ,0       5
    {  0x00,       -2,      -4,        0,         5},   // 9     ,5      10
    {  0x00,       -2,      -3,        0,         4},   // 10    ,10     15
    {  0x00,       -1,      -3,        0,         3},   // 11    ,15    20
    {  0x00,       -1,      -1,        0,         1},   // 12    ,20    25
    {  0x00,        0,       0,        0,         0},   // 13    ,25    30
    {  0x00,        0,       0,        0,        -3},   // 14    ,30   35
    {  0x00,        0,       1,        0,        -5},   // 15    ,35   40
    {  0x00,        1,       1,        0,        -6},   // 16    ,40  45
    {  0x00,        1,       2,        0,        -7},   // 17    ,45    50
    {  0x00,        2,       2,        0,        -8},   // 18    ,50    55
    {  0x00,        2,       2,        0,        -7},   // 19    ,55   60
    {  0x00,        3,       3,        0,        -6},   // 20    ,60   65
    {  0x00,        3,       3,        0,        -4},   // 21    ,65   70
    {  0x00,        4,       4,        0,         0},   // 22    ,70   75
    {  0x00,        4,       5,        0,         7},   // 23    ,75   80
    {  0x00,        4,       5,        0,        14},   // 24    ,80   85
    {  0x00,        5,       5,        0,        22},   // 25    ,85  90
    {  0x00,        5,       7,        0,        33},   // 26    ,90  95
    {  0x00,        6,       8,        1,        47},   // 27    ,95  100
    {  0x00,        6,       9,        1,        63},   // 28    ,100  105
    {  0x00,        7,       9,        2,        84},   // 29    ,105  110
    {  0x00,        7,      10,        2,       106},   // 30    ,110  115
    {  0x00,        8,      10,        3,       127},   // 31    ,115
    {  0x00,        8,      11,        3,       127},   // 32    ,120
    {  0x00,        9,      12,        3,       127},   // 33    ,125
    {  0x00,        9,      12,        3,       127},   // 34    ,130
    {  0x00,        9,      13,        3,       127},   // 35    ,135
    {  0x00,       10,      13,        3,       127},   // 36    ,140
    {  0x00,       10,      13,        3,       127},   // 37    ,145
    {  0x00,       10,      13,        3,       127},   // 38    ,150
#else
    {  0x00,        0,      0,       0,        -5},   // 0     ,-40
    {  0x00,        0,      0,       0,        -2},   // 1     ,-35
    {  0x00,        0,      0,       0,         1},   // 2     ,-30
    {  0x00,        0,      0,       0,         2},   // 3     ,-25
    {  0x00,        0,      0,       0,         3},   // 4     ,-20
    {  0x00,        0,      0,       0,         4},   // 5     ,-15
    {  0x00,        0,      0,       0,         4},   // 6     ,-10
    {  0x00,        0,      0,       0,         4},   // 7     ,-5
    {  0x00,        0,      0,       0,         4},   // 8     ,0
    {  0x00,        0,      0,       0,         4},   // 9     ,5
    {  0x00,        0,      0,       0,         3},   // 10    ,10
    {  0x00,        0,      0,       0,         2},   // 11    ,15
    {  0x00,        0,      0,       0,         1},   // 12    ,20
    {  0x00,        0,      0,       0,         0},   // 13    ,25
    {  0x00,        0,      0,       0,        -1},   // 14    ,30
    {  0x00,        0,      0,       0,        -2},   // 15    ,35
    {  0x00,        0,      0,       0,        -3},   // 16    ,40
    {  0x00,        0,      0,       0,        -3},   // 17    ,45
    {  0x00,        0,      0,       0,        -4},   // 18    ,50
    {  0x00,        0,      0,       0,        -5},   // 19    ,55
    {  0x00,        0,      0,       0,        -5},   // 20    ,60
    {  0x00,        0,      0,       0,        -5},   // 21    ,65
    {  0x00,        0,      0,       0,        -4},   // 22    ,70
    {  0x00,        0,      0,       0,        -3},   // 23    ,75
    {  0x00,        0,      0,       0,        -3},   // 24    ,80
    {  0x00,        0,      0,       0,        -1},   // 25    ,85
    {  0x00,        0,      0,       0,         2},   // 26    ,90
    {  0x00,        0,      0,       0,         5},   // 27    ,95
    {  0x00,        0,      0,       0,         9},   // 28    ,100
    {  0x00,        0,      0,       0,        14},   // 29    ,105
    {  0x00,        0,      0,       0,        21},   // 30    ,110
    {  0x00,        0,      0,       0,        29},   // 31    ,115
    {  0x00,        0,      0,       0,        39},   // 32    ,120
    {  0x00,        0,      0,       0,        53},   // 33    ,125
    {  0x00,        0,      0,       0,        69},   // 34    ,130
    {  0x00,        0,      0,       0,        92},   // 35    ,135
    {  0x00,        0,      0,       0,       122},   // 36    ,140
    {  0x00,        0,      0,       0,       166},   // 37    ,145
    {  0x00,        0,      0,       0,       166},   // 38    ,150
#endif
};
#else
const TMP_PWR_ST tmp_pwr_tab[TMP_PWR_TAB_LEN] =
{
    //trx0x0c[12:15], shift_b, shift_g, shift_ble, xtal_c_delta
    {  0x08,        -4,      -4,       0,         5},   // 0     ,-40
    {  0x08,        -4,      -4,       0,         5},   // 1     ,-35
    {  0x08,        -4,      -4,       0,         5},   // 2     ,-30
    {  0x08,        -4,      -4,       0,         5},   // 3     ,-25
    {  0x08,        -4,      -4,       0,         5},   // 4     ,-20
    {  0x08,        -3,      -3,       0,         5},   // 5     ,-15
    {  0x08,        -3,      -3,       0,         5},   // 6     ,-10
    {  0x08,        -3,      -3,       0,         5},   // 7     ,-5
    {  0x08,        -2,      -2,       0,         5},   // 8     ,0
    {  0x08,        -2,      -2,       0,         5},   // 9     ,5
    {  0x08,        -2,      -2,       0,         5},   // 10    ,10
    {  0x08,        -1,      -1,       0,         5},   // 11    ,15
    {  0x08,        -1,      -1,       0,         4},   // 12    ,20
    {  0x08,        -1,      -1,       0,         3},   // 13    ,25
    {  0x08,        0,       0,        0,         2},   // 14    ,30
    {  0x08,        0,       0,        0,         1},   // 15    ,35
    {  0x08,        0,       0,        0,         0},   // 16    ,40
    {  0x08,        0,       0,        0,         0},   // 17    ,45
    {  0x08,        0,       0,        0,         0},   // 18    ,50
    {  0x08,        0,       0,        0,         0},   // 19    ,55
    {  0x08,        0,       0,        0,         0},   // 20    ,60
    {  0x08,        0,       0,        0,         0},   // 21    ,65
    {  0x08,        0,       0,        0,         0},   // 22    ,70
    {  0x08,        0,       0,        0,         0},   // 23    ,75
    {  0x08,        0,       0,        0,         0},   // 24    ,80
    {  0x08,        0,       0,        0,         5},   // 25    ,85
    {  0x08,        0,       0,        0,         8},   // 26    ,90
    {  0x08,        1,       1,        1,        11},   // 27    ,95
    {  0x08,        1,       1,        1,        16},   // 28    ,100
    {  0x08,        2,       2,        2,        22},   // 29    ,105
    {  0x08,        2,       2,        2,        30},   // 30    ,110
    {  0x08,        3,       3,        3,        40},   // 31    ,115
    {  0x08,        3,       3,        3,        52},   // 32    ,120
    {  0x08,        3,       3,        3,        63},   // 33    ,125
    {  0x08,        3,       3,        3,        63},   // 34    ,130
    {  0x08,        3,       3,        3,        63},   // 35    ,135
    {  0x08,        3,       3,        3,        63},   // 36    ,140
    {  0x08,        3,       3,        3,        63},   // 37    ,145
    {  0x08,        3,       3,        3,        63},   // 38    ,150
};
#endif
#endif  // CFG_USE_TEMPERATURE_DETECT

/****************************** power shift table  ****************************/
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
const UINT16 shift_tab_b[4] = {0, 0, 0, 0}; // 11M base,5.5M,2M,1M
// 54M base -                 54M,48M,36M,24M,18M,12M,9M,6M
const UINT16 shift_tab_g[8] = {0,  2,  2,  2,  3,  3,  4, 4/*4*/}; // 54M base -  12M,9M,6M//do
const UINT16 shift_tab_n20[8] = {0,  2,  2,  2,  3,  3,  4, 4/*4*/};; // n20 mcs7base -  mcs0
const UINT16 shift_tab_n40[8] = {0,  2,  2,  2,  3,  3,  4, 4/*4*/}; // n40 mcs7base -  mcs0
#else
const UINT16 shift_tab_b[4] = {0, 0, 0, 0}; // 11M base,5.5M,2M,1M
// 54M base -                 54M,48M,36M,24M,18M,12M,9M,6M
const UINT16 shift_tab_g[8] = {0,  1,  1,  1,  2,  2,  4, 6/*4*/}; // 54M base -  12M,9M,6M
const UINT16 shift_tab_n20[8] = {0, 1, 1, 2, 2, 4, 4, 6/*4*/};; // n20 mcs7base -  mcs0
const UINT16 shift_tab_n40[8] = {0, 1, 1, 2, 2, 4, 4, 6/*4*/}; // n40 mcs7base -  mcs0
#endif
#endif // (CFG_SOC_NAME != SOC_BK7231)

/******************************* user define api  *****************************/
UINT32 rwnx_cal_load_user_rfcali_mode(int *rfcali_mode)
{
    // set to CALI_MODE_AUTO / CALI_MODE_MANUAL
    *rfcali_mode = CALI_MODE_AUTO;

    // return 0 means no used,  return 1 means used
    return 0;
}

UINT32 rwnx_cal_load_user_g_tssi_threshold(int *tssi_g)
{
    *tssi_g = 0;

    // return 0 means no used,  return 1 means used
    return 0;
}

UINT32 rwnx_cal_load_user_b_tssi_threshold(int *tssi_b)
{
    *tssi_b = 0;

    // return 0 means no used,  return 1 means used
    return 0;
}

UINT32 rwnx_cal_load_user_n20_tssi_threshold(int *tssi_g)
{
    *tssi_g = 0;

    // return 0 means no used,  return 1 means used
    return 0;
}

UINT32 rwnx_cal_load_user_n40_tssi_threshold(int *tssi_b)
{
    *tssi_b = 0;

    // return 0 means no used,  return 1 means used
    return 0;
}

UINT32 rwnx_cal_load_user_xtal(UINT32 *xtal)
{
    *xtal = 0;

    // return 0 means no used,  return 1 means used
    return 0;
}

