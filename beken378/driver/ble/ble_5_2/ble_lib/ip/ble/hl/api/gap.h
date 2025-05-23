/**
 ****************************************************************************************
 *
 * @file gap.h
 *
 * @brief Header file - GAP.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */
#ifndef GAP_H_
#define GAP_H_
/**
 ****************************************************************************************
 * @addtogroup HOST
 * @ingroup ROOT
 * @brief Bluetooth Low Energy Host
 *
 * The HOST layer of the stack contains the higher layer protocols (@ref ATT "ATT",
 * @ref SMP "SMP") and transport module (@ref L2C "L2C"). It also includes the Generic
 * Access Profile (@ref GAP "GAP"), used for scanning/connection operations.
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @addtogroup GAP Generic Access Profile
 * @ingroup HOST
 * @brief Generic Access Profile.
 *
 * The GAP module is responsible for providing an API to the application in order to
 * configure the device in the desired mode (discoverable, connectable, etc.) and perform
 * required actions (scanning, connection, pairing, etc.). To achieve this, the GAP
 * interfaces with both the @ref SMP "SMP", @ref L2C "L2C" and the @ref CONTROLLER "CONTROLLER"
 *
 * @{
 ****************************************************************************************
 */

#include <stdint.h>
#include <stdbool.h>
#include "compiler.h"

/*
 * DEFINES
 ****************************************************************************************
 */
/// Invalid Procedure Token
#define GAP_INVALID_TOKEN     (0x0000)

/// BD address length
#define GAP_BD_ADDR_LEN       (6)
/// LE Channel map length
#define GAP_LE_CHNL_MAP_LEN   (0x05)
/// LE Feature Flags Length
#define GAP_LE_FEATS_LEN      (0x08)
/// ADV Data and Scan Response length
#define GAP_ADV_DATA_LEN      (0x1F)
#define GAP_SCAN_RSP_DATA_LEN (0x1F)
/// Random number length
#define GAP_RAND_NB_LEN       (0x08)
/// Key length
#define GAP_KEY_LEN           (16)
/// P256 Key Len
#define GAP_P256_KEY_LEN      (0x20)


///***** AD Type Flag - Bit set *******/
enum gap_ad_type_flag
{
    /// Limited discovery flag - AD Flag
    GAP_LE_LIM_DISCOVERABLE_FLG_BIT     = 0x01,
    GAP_LE_LIM_DISCOVERABLE_FLG_POS     = 0,
    /// General discovery flag - AD Flag
    GAP_LE_GEN_DISCOVERABLE_FLG_BIT     = 0x02,
    GAP_LE_GEN_DISCOVERABLE_FLG_POS     = 1,
    /// Legacy BT not supported - AD Flag
    GAP_BR_EDR_NOT_SUPPORTED_BIT        = 0x04,
    GAP_BR_EDR_NOT_SUPPORTED_POS        =  2,
    /// Dual mode for controller supported (BR/EDR/LE) - AD Flag
    GAP_SIMUL_BR_EDR_LE_CONTROLLER_BIT  = 0x08,
    GAP_SIMUL_BR_EDR_LE_CONTROLLER_POS  = 3,
    /// Dual mode for host supported (BR/EDR/LE) - AD Flag
    GAP_SIMUL_BR_EDR_LE_HOST_BIT        = 0x10,
    GAP_SIMUL_BR_EDR_LE_HOST_POS        = 4,
};

/*********** GAP Miscellaneous Defines *************/
/// Invalid activity index
#define GAP_INVALID_ACTV_IDX                    0xFF
/// Invalid connection index
#define GAP_INVALID_CONIDX                      0xFF

/// Invalid connection handle
#define GAP_INVALID_CONHDL                      0xFFFF

/// Connection interval min (N*1.250ms)
#define GAP_CNX_INTERVAL_MIN            6       //(0x06)
/// Connection interval Max (N*1.250ms)
#define GAP_CNX_INTERVAL_MAX            3200    //(0xC80)
/// Connection latency min (N*cnx evt)
#define GAP_CNX_LATENCY_MIN             0       //(0x00)
/// Connection latency Max (N*cnx evt
#define GAP_CNX_LATENCY_MAX             499     //(0x1F3)
/// Supervision TO min (N*10ms)
#define GAP_CNX_SUP_TO_MIN              10      //(0x0A)
/// Supervision TO Max (N*10ms)
#define GAP_CNX_SUP_TO_MAX              3200    //(0xC80)



/// Length of resolvable random address prand part
#define GAP_ADDR_PRAND_LEN            (3)
/// Length of resolvable random address hash part
#define GAP_ADDR_HASH_LEN             (3)

/// Number of bytes needed for a bit field indicated presence of a given Advertising Flag value
/// in the Advertising or the Scan Response data
/// Advertising Flags is a 8-bit value, hence 256 value are possible
/// -> 256 / 8 bytes = 32 bytes are needed
#define GAP_AD_TYPE_BITFIELD_BYTES                          (32)

/*
 * DEFINES - Optional for BLE application usage
 ****************************************************************************************
 */

/// Central idle timer
/// TGAP(conn_pause_central)
/// recommended value: 1 s: (100 for ke timer)
#define GAP_TMR_CONN_PAUSE_CT                               0x0064

/// Minimum time upon connection establishment before the peripheral
/// starts a connection update procedure: TGAP(conn_pause_peripheral)
/// recommended value: 5 s: (500 for ke timer)
#define GAP_TMR_CONN_PAUSE_PH                               0x01F4

/// Minimum time to perform scanning when user initiated
/// TGAP(scan_fast_period)
/// recommended value: 30.72 s: (3072 for ke timer)
#define GAP_TMR_SCAN_FAST_PERIOD                            0x0C00

/// Minimum time to perform advertising when user initiated
/// TGAP(adv_fast_period)
/// recommended value: 30 s: (3000 for ke timer)
#define GAP_TMR_ADV_FAST_PERIOD                             0x0BB8

/// Scan interval used during Link Layer Scanning State when
/// performing the Limited Discovery procedure
/// TGAP(lim_disc_scan_int)
/// recommended value: 11.25ms; (18 decimal)
#define GAP_LIM_DISC_SCAN_INT                               0x0012

/// Scan interval in any discovery or connection establishment
/// procedure when user initiated: TGAP(scan_fast_interval)
/// recommended value: 30 to 60 ms; N * 0.625
#define GAP_SCAN_FAST_INTV                                  0x0030

/// Scan window in any discovery or connection establishment
/// procedure when user initiated: TGAP(scan_fast_window)
/// recommended value: 30 ms; N * 0.625
#define GAP_SCAN_FAST_WIND                                  0x0030

/// Scan interval in any discovery or connection establishment
/// procedure when background scanning: TGAP(scan_slow_interval1)
/// recommended value: 1.28 s : 0x00CD (205); N * 0.625
#define GAP_SCAN_SLOW_INTV1                                 0x00CD

/// Scan interval in any discovery or connection establishment
/// procedure when background scanning: TGAP(scan_slow_interval2)
/// recommended value: 2.56 s : 0x019A (410); N * 0.625
#define GAP_SCAN_SLOW_INTV2                                 0x019A

/// Scan window in any discovery or connection establishment
/// procedure when background scanning: TGAP(scan_slow_window1)
/// recommended value: 11.25 ms : 0x0012 (18); N * 0.625
#define GAP_SCAN_SLOW_WIND1                                 0x0012

/// Scan window in any discovery or connection establishment
/// procedure when background scanning: TGAP(scan_slow_window2)
/// recommended value: 22.5 ms : 0x0024 (36); N * 0.625
#define GAP_SCAN_SLOW_WIND2                                 0x0024

/// Minimum to maximum advertisement interval in any discoverable
/// or connectable mode when user initiated: TGAP(adv_fast_interval1)
/// recommended value: 30 to 60 ms; N * 0.625
#define GAP_ADV_FAST_INTV1                                  0x0030

/// Minimum to maximum advertisement interval in any discoverable
/// or connectable mode when user initiated: TGAP(adv_fast_interval2)
/// recommended value: 100 to 150 ms; N * 0.625
#define GAP_ADV_FAST_INTV2                                  0x0064

/// Minimum to maximum advertisement interval in any discoverable or
/// connectable mode when background advertising: TGAP(adv_slow_interval)
/// recommended value: 1 to 1.2 s : 0x00B0 (176); N * 0.625
#define GAP_ADV_SLOW_INTV                                   0x00B0

/// Minimum to maximum connection interval upon any connection
/// establishment: TGAP(initial_conn_interval)
/// recommended value: 30 to 50 ms ; N * 1.25 ms
#define GAP_INIT_CONN_MIN_INTV                              0x0018
#define GAP_INIT_CONN_MAX_INTV                              0x0028

/// RW Defines
#define GAP_INQ_SCAN_INTV                                   0x0012
#define GAP_INQ_SCAN_WIND                                   0x0012

/// Connection supervision timeout
/// recommended value: 20s
#define GAP_CONN_SUPERV_TIMEOUT                             0x07D0

/// Minimum connection event
/// default value: 0x0000
#define GAP_CONN_MIN_CE                                     0x0000

/// Maximum connection event
/// default value: 0xFFFF
#define GAP_CONN_MAX_CE                                     0xFFFF

/// Connection latency
/// default value: 0x0000
#define GAP_CONN_LATENCY                                    0x0000

/// GAP Device name Characteristic
/// Default device name
#define GAP_DEV_NAME                                        "RIVIERAWAVES-BLE"

/// GAP Appearance or Icon Characteristic - 2 octets
/// Current appearance value is 0x0000 (unknown appearance)
/// Description:
/// http://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.gap.appearance.xml
#define GAP_APPEARANCE                                      0x0000

///GAP Peripheral Preferred Connection Parameter - 8 octets
#define GAP_PPCP_CONN_INTV_MAX                              0x0064
#define GAP_PPCP_CONN_INTV_MIN                              0x00C8
#define GAP_PPCP_SLAVE_LATENCY                              0x0000
#define GAP_PPCP_STO_MULT                                   0x07D0


/// Acceptable encryption key size - strict access
#define GAP_SEC_ENC_KEY_SIZE                               (0x10)

/*
 * Macros
 ****************************************************************************************
 */

#define GAP_AD_TYPE_SET_BIT(bitfield, adv_flag)                             \
                bitfield[adv_flag / 8] |= COMMON_BIT(adv_flag % 8)

#define GAP_AD_TYPE_CHECK_BIT(bitfield, adv_flag)                           \
                (bitfield[adv_flag / 8] & COMMON_BIT(adv_flag % 8))

/// size of advertising data counting advertising type header
#define GAP_AD_TOTAL_LEN(data_size)      ((data_size) + 1)

/*
 * Enumerations
 ****************************************************************************************
 */


/// List of supported BLE Features
enum gap_le_feature
{
    //byte 0
    GAP_LE_FEAT_ENC                       = (0),
    GAP_LE_FEAT_CON_PARAM_REQ_PROC        = (1),
    GAP_LE_FEAT_EXT_REJ_IND               = (2),
    GAP_LE_FEAT_SLAVE_INIT_FEAT_EXCHG     = (3),
    GAP_LE_FEAT_PING                      = (4),
    GAP_LE_FEAT_DATA_PKT_LEN_EXT          = (5),
    GAP_LE_FEAT_LL_PRIVACY                = (6),
    GAP_LE_FEAT_EXT_SCAN_FILT_POLICY      = (7),
    //byte 1
    GAP_LE_FEAT_2M_PHY                    = (8),
    GAP_LE_FEAT_STABLE_MOD_IDX_TX         = (9),
    GAP_LE_FEAT_STABLE_MOD_IDX_RX         = (10),
    GAP_LE_FEAT_CODED_PHY                 = (11),
    GAP_LE_FEAT_EXT_ADV                   = (12),
    GAP_LE_FEAT_PER_ADV                   = (13),
    GAP_LE_FEAT_CHAN_SEL_ALGO_2           = (14),
    GAP_LE_FEAT_PWR_CLASS_1               = (15),
    //byte 2
    GAP_LE_FEAT_MIN_NUM_USED_CHAN_PROC    = (16),
    GAP_LE_FEAT_CON_CTE_REQ               = (17),
    GAP_LE_FEAT_CON_CTE_RSP               = (18),
    GAP_LE_FEAT_CONLESS_CTE_TX            = (19),
    GAP_LE_FEAT_CONLESS_CTE_RX            = (20),
    GAP_LE_FEAT_AOD                       = (21),
    GAP_LE_FEAT_AOA                       = (22),
    GAP_LE_FEAT_CTE_RX                    = (23),
    //byte 3
    GAP_LE_FEAT_PER_ADV_SYNC_TRANSF_TX    = (24),
    GAP_LE_FEAT_PER_ADV_SYNC_TRANSF_RX    = (25),
    GAP_LE_FEAT_SLEEP_CLK_ACC_UPD         = (26),
    GAP_LE_FEAT_PUB_KEY_VALID             = (27),
    GAP_LE_FEAT_CON_ISO_STREAM_MASTER     = (28),
    GAP_LE_FEAT_CON_ISO_STREAM_SLAVE      = (29),
    GAP_LE_FEAT_ISO_BROADCASTER           = (30),
    GAP_LE_FEAT_SYNCED_RECEIVER           = (31),
    //byte 4
    GAP_LE_FEAT_ISO_CHANNELS_HOST_SUPPORT = (32),
    GAP_LE_FEAT_POWER_CONTROL_REQ         = (33),
    GAP_LE_FEAT_POWER_CHANGE_IND          = (34),
    GAP_LE_FEAT_PATH_LOSS_MONITORING      = (35),

    GAP_LE_FEAT_MAX,
};

/// GAP Advertising Flags
enum gap_ad_type
{
    /// Flag
    GAP_AD_TYPE_FLAGS                      = 0x01,
    /// Use of more than 16 bits UUID
    GAP_AD_TYPE_MORE_16_BIT_UUID           = 0x02,
    /// Complete list of 16 bit UUID
    GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID  = 0x03,
    /// Use of more than 32 bit UUD
    GAP_AD_TYPE_MORE_32_BIT_UUID           = 0x04,
    /// Complete list of 32 bit UUID
    GAP_AD_TYPE_COMPLETE_LIST_32_BIT_UUID  = 0x05,
    /// Use of more than 128 bit UUID
    GAP_AD_TYPE_MORE_128_BIT_UUID          = 0x06,
    /// Complete list of 128 bit UUID
    GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID = 0x07,
    /// Shortened device name
    GAP_AD_TYPE_SHORTENED_NAME             = 0x08,
    /// Complete device name
    GAP_AD_TYPE_COMPLETE_NAME              = 0x09,
    /// Transmit power
    GAP_AD_TYPE_TRANSMIT_POWER             = 0x0A,
    /// Class of device
    GAP_AD_TYPE_CLASS_OF_DEVICE            = 0x0D,
    /// Simple Pairing Hash C
    GAP_AD_TYPE_SP_HASH_C                  = 0x0E,
    /// Simple Pairing Randomizer
    GAP_AD_TYPE_SP_RANDOMIZER_R            = 0x0F,
    /// Temporary key value
    GAP_AD_TYPE_TK_VALUE                   = 0x10,
    /// Out of Band Flag
    GAP_AD_TYPE_OOB_FLAGS                  = 0x11,
    /// Slave connection interval range
    GAP_AD_TYPE_SLAVE_CONN_INT_RANGE       = 0x12,
    /// Require 16 bit service UUID
    GAP_AD_TYPE_RQRD_16_BIT_SVC_UUID       = 0x14,
    /// Require 32 bit service UUID
    GAP_AD_TYPE_RQRD_32_BIT_SVC_UUID       = 0x1F,
    /// Require 128 bit service UUID
    GAP_AD_TYPE_RQRD_128_BIT_SVC_UUID      = 0x15,
    /// Service data 16-bit UUID
    GAP_AD_TYPE_SERVICE_16_BIT_DATA        = 0x16,
    /// Service data 32-bit UUID
    GAP_AD_TYPE_SERVICE_32_BIT_DATA        = 0x20,
    /// Service data 128-bit UUID
    GAP_AD_TYPE_SERVICE_128_BIT_DATA       = 0x21,
    /// Public Target Address
    GAP_AD_TYPE_PUB_TGT_ADDR               = 0x17,
    /// Random Target Address
    GAP_AD_TYPE_RAND_TGT_ADDR              = 0x18,
    /// Appearance
    GAP_AD_TYPE_APPEARANCE                 = 0x19,
    /// Advertising Interval
    GAP_AD_TYPE_ADV_INTV                   = 0x1A,
    /// LE Bluetooth Device Address
    GAP_AD_TYPE_LE_BT_ADDR                 = 0x1B,
    /// LE Role
    GAP_AD_TYPE_LE_ROLE                    = 0x1C,
    /// Simple Pairing Hash C-256
    GAP_AD_TYPE_SPAIR_HASH                 = 0x1D,
    /// Simple Pairing Randomizer R-256
    GAP_AD_TYPE_SPAIR_RAND                 = 0x1E,
    /// 3D Information Data
    GAP_AD_TYPE_3D_INFO                    = 0x3D,

    /// Manufacturer specific data
    GAP_AD_TYPE_MANU_SPECIFIC_DATA         = 0xFF,
};

/// Boolean value set
enum
{
    /// Disable
    GAP_DISABLE = 0x00,
    /// Enable
    GAP_ENABLE
};

/****************** GAP Role **********************/
enum gap_role
{
    /// No role set yet
    GAP_ROLE_NONE        = 0x00,
    /// Observer role
    GAP_ROLE_OBSERVER    = 0x01,
    /// Broadcaster role
    GAP_ROLE_BROADCASTER = 0x02,
    /// Master/Central role
    GAP_ROLE_CENTRAL     = (0x04 | GAP_ROLE_OBSERVER),
    /// Peripheral/Slave role
    GAP_ROLE_PERIPHERAL  = (0x08 | GAP_ROLE_BROADCASTER),
    /// Device has all role, both peripheral and central
    GAP_ROLE_ALL         = (GAP_ROLE_CENTRAL | GAP_ROLE_PERIPHERAL),
};

/// IO Capability Values
enum gap_io_cap
{
    /// Display Only
    GAP_IO_CAP_DISPLAY_ONLY = 0x00,
    /// Display Yes No
    GAP_IO_CAP_DISPLAY_YES_NO,
    /// Keyboard Only
    GAP_IO_CAP_KB_ONLY,
    /// No Input No Output
    GAP_IO_CAP_NO_INPUT_NO_OUTPUT,
    /// Keyboard Display
    GAP_IO_CAP_KB_DISPLAY,
    GAP_IO_CAP_LAST
};

/// TK Type
enum gap_tk_type
{
    ///  TK get from out of band method
    GAP_TK_OOB         = 0x00,
    /// TK generated and shall be displayed by local device
    GAP_TK_DISPLAY,
    /// TK shall be entered by user using device keyboard
    GAP_TK_KEY_ENTRY
};

/// OOB Data Present Flag Values
enum gap_oob
{
    /// OOB Data not present
    GAP_OOB_AUTH_DATA_NOT_PRESENT = 0x00,
    /// OOB data present
    GAP_OOB_AUTH_DATA_PRESENT,
    GAP_OOB_AUTH_DATA_LAST
};

/// Authentication mask
enum gap_auth_mask
{
    /// No Flag set
    GAP_AUTH_NONE    = 0,
    /// Bond authentication
    GAP_AUTH_BOND    = (1 << 0),
    /// Man In the middle protection
    GAP_AUTH_MITM    = (1 << 2),
    /// Secure Connection
    GAP_AUTH_SEC_CON = (1 << 3),
    /// Key Notification
    GAP_AUTH_KEY_NOTIF = (1 << 4)
};

/// Security Link Level
enum gap_sec_lvl
{
    /// Service accessible through an un-encrypted link
    GAP_SEC_NOT_ENC             = 0,
    /// Service require an unauthenticated pairing (just work pairing)
    GAP_SEC_UNAUTH,
    /// Service require an authenticated pairing (Legacy pairing with pin code or OOB)
    GAP_SEC_AUTH,
    /// Service require a secure connection pairing
    GAP_SEC_SECURE_CON,
};

/// Authentication Requirements
enum gap_auth
{
    /// No MITM No Bonding
    GAP_AUTH_REQ_NO_MITM_NO_BOND  = (GAP_AUTH_NONE),
    /// No MITM Bonding
    GAP_AUTH_REQ_NO_MITM_BOND     = (GAP_AUTH_BOND),
    /// MITM No Bonding
    GAP_AUTH_REQ_MITM_NO_BOND     = (GAP_AUTH_MITM),
    /// MITM and Bonding
    GAP_AUTH_REQ_MITM_BOND        = (GAP_AUTH_MITM | GAP_AUTH_BOND),
    /// SEC_CON and No Bonding
    GAP_AUTH_REQ_SEC_CON_NO_BOND  = (GAP_AUTH_SEC_CON | GAP_AUTH_MITM),
    /// SEC_CON and Bonding
    GAP_AUTH_REQ_SEC_CON_BOND     = (GAP_AUTH_SEC_CON | GAP_AUTH_MITM | GAP_AUTH_BOND),

    GAP_AUTH_REQ_LAST,

    /// Mask of  authentication features without reserved flag
    GAP_AUTH_REQ_MASK             = 0x1F,
};

/// Pairing level achieved
enum gap_pairing_lvl
{
    /// Unauthenticated pairing achieved but without bond data
    /// (meaning-less for connection confirmation)
    GAP_PAIRING_UNAUTH           = 0x00,
    /// Authenticated pairing achieved but without bond data
    /// (meaning-less for connection confirmation)
    GAP_PAIRING_AUTH             = 0x04,
    /// Secure connection pairing achieved but without bond data
    /// (meaning-less for connection confirmation)
    GAP_PAIRING_SECURE_CON       = 0x0C,

    /// No pairing performed with peer device
    /// (meaning-less for connection confirmation)
    GAP_PAIRING_NO_BOND          = 0x00,
    /// Peer device bonded through an unauthenticated pairing.
    GAP_PAIRING_BOND_UNAUTH      = 0x01,
    /// Peer device bonded through an authenticated pairing.
    GAP_PAIRING_BOND_AUTH        = 0x05,
    /// Peer device bonded through a secure connection pairing pairing.
    GAP_PAIRING_BOND_SECURE_CON  = 0x0D,

    /// Pairing with bond data present Bit
    GAP_PAIRING_BOND_PRESENT_BIT = 0x01,
    GAP_PAIRING_BOND_PRESENT_POS = 0x00,
};

/// Key Distribution Flags
enum gap_kdist
{
    /// No Keys to distribute
    GAP_KDIST_NONE = 0x00,
    /// Encryption key in distribution
    GAP_KDIST_ENCKEY = (1 << 0),
    /// IRK (ID key)in distribution
    GAP_KDIST_IDKEY  = (1 << 1),
    /// CSRK(Signature key) in distribution
    GAP_KDIST_SIGNKEY= (1 << 2),
    /// LTK in distribution
    GAP_KDIST_LINKKEY=   (1 << 3),

    GAP_KDIST_LAST =   (1 << 4)
};

/// Security Defines
enum gap_sec_req
{
    /// No security (no authentication and encryption)
    GAP_NO_SEC = 0x00,
    /// Unauthenticated pairing with encryption
    GAP_SEC1_NOAUTH_PAIR_ENC,
    /// Authenticated pairing with encryption
    GAP_SEC1_AUTH_PAIR_ENC,
    /// Unauthenticated pairing with data signing
    GAP_SEC2_NOAUTH_DATA_SGN,
    /// Authentication pairing with data signing
    GAP_SEC2_AUTH_DATA_SGN,
    /// Secure Connection pairing with encryption
    GAP_SEC1_SEC_CON_PAIR_ENC,
};

/// Bit field use to select the preferred TX or RX LE PHY. 0 means no preferences
enum gap_phy
{
    /// No preferred PHY
    GAP_PHY_ANY               = 0x00,
    /// LE 1M PHY preferred for an active link
    GAP_PHY_LE_1MBPS          = (1 << 0),
    /// LE 2M PHY preferred for an active link
    GAP_PHY_LE_2MBPS          = (1 << 1),
    /// LE Coded PHY preferred for an active link
    GAP_PHY_LE_CODED          = (1 << 2),
};

/// Enumeration of TX/RX PHY values
enum gap_phy_val
{
    /// LE 1M PHY (TX or RX)
    GAP_PHY_1MBPS        = 1,
    /// LE 2M PHY (TX or RX)
    GAP_PHY_2MBPS        = 2,
    /// LE Coded PHY (RX Only)
    GAP_PHY_CODED        = 3,
    /// LE Coded PHY with S=8 data coding (TX Only)
    GAP_PHY_125KBPS      = 3,
    /// LE Coded PHY with S=2 data coding (TX Only)
    GAP_PHY_500KBPS      = 4,
};

/// Modulation index
enum gap_modulation_idx
{
    /// Assume transmitter will have a standard modulation index
    GAP_MODULATION_STANDARD,
    /// Assume transmitter will have a stable modulation index
    GAP_MODULATION_STABLE,
};

/// Packet Payload type for test mode
enum gap_pkt_pld_type
{
    /// PRBS9 sequence "11111111100000111101..." (in transmission order)
    GAP_PKT_PLD_PRBS9,
    /// Repeated "11110000" (in transmission order)
    GAP_PKT_PLD_REPEATED_11110000,
    /// Repeated "10101010" (in transmission order)
    GAP_PKT_PLD_REPEATED_10101010,
    /// PRBS15 sequence
    GAP_PKT_PLD_PRBS15,
    /// Repeated "11111111" (in transmission order) sequence
    GAP_PKT_PLD_REPEATED_11111111,
    /// Repeated "00000000" (in transmission order) sequence
    GAP_PKT_PLD_REPEATED_00000000,
    /// Repeated "00001111" (in transmission order) sequence
    GAP_PKT_PLD_REPEATED_00001111,
    /// Repeated "01010101" (in transmission order) sequence
    GAP_PKT_PLD_REPEATED_01010101,
};

/// Constant Tone Extension type
enum gap_cte_type
{
    /// Allow AoA Constant Tone Extension Response
    GAP_CTE_AOA          = (1 << 0),
    /// Allow AoD Constant Tone Extension Response with 1 us slots
    GAP_CTE_AOD_1US_SLOT = (1 << 1),
    /// Allow AoD Constant Tone Extension Response with 2 us slots
    GAP_CTE_AOD_2US_SLOT = (1 << 2),
};

/*************** GAP Structures ********************/

/// Device name
/*@TRACE*/
struct gap_dev_name
{
    /// Length of provided value
    uint16_t value_length;
    /// name value starting from offset to maximum length
    uint8_t  value[__ARRAY_EMPTY];
};

/// Slave preferred connection parameters
/*@TRACE*/
typedef struct gap_slv_pref
{
    /// Connection interval minimum
    uint16_t con_intv_min;
    /// Connection interval maximum
    uint16_t con_intv_max;
    /// Slave latency
    uint16_t slave_latency;
    /// Connection supervision timeout multiplier
    uint16_t conn_timeout;
} gap_slv_pref_t;

///Channel map structure
/*@TRACE*/
typedef struct
{
    ///5-byte channel map array
    uint8_t map[GAP_LE_CHNL_MAP_LEN];
} le_chnl_map_t;


///Random number structure
/*@TRACE*/
typedef struct
{
    ///8-byte array for random number
    uint8_t     nb[GAP_RAND_NB_LEN];
} rand_nb_t;

/// P256 Public key data format
typedef struct
{
    /// X Coordinate of the key
    uint8_t x[GAP_P256_KEY_LEN];
    /// X Coordinate of the key
    uint8_t y[GAP_P256_KEY_LEN];
} public_key_t;

/// Bluetooth address
/*@TRACE*/
typedef struct gap_addr
{
    /// BD Address of device
    uint8_t addr[GAP_BD_ADDR_LEN];
} gap_addr_t;

/// Address information about a device address
/*@TRACE*/
typedef struct gap_bdaddr
{
    /// BD Address of device
    uint8_t addr[GAP_BD_ADDR_LEN];
    /// Address type of the device 0=public/1=private random
    uint8_t addr_type;
} gap_bdaddr_t;

/// Connection parameters information
/*@TRACE*/
typedef struct gap_con_param
{
    /// Connection interval in 1.25ms unit
    uint16_t            con_interval;
    /// Connection latency value (in number of connection events)
    uint16_t            con_latency;
    /// Supervision timeout in 10ms unit
    uint16_t            sup_to;
} gap_con_param_t;


/// Periodic advertising address information
/*@TRACE*/
typedef struct gap_per_adv_bdaddr
{
    /// BD Address of device
    uint8_t addr[GAP_BD_ADDR_LEN];
    /// Address type of the device 0=public/1=private random
    uint8_t addr_type;
    /// Advertising SID
    uint8_t adv_sid;
} gap_per_adv_bdaddr_t;

/// Resolving list device information
/*@TRACE*/
struct gap_ral_dev_info
{
    /// Device identity
    gap_bdaddr_t addr;
    /// Privacy Mode
    uint8_t      priv_mode;
    /// Peer IRK
    uint8_t      peer_irk[GAP_KEY_LEN];
    /// Local IRK
    uint8_t      local_irk[GAP_KEY_LEN];
};

/// Generic Security key structure
/*@TRACE*/
typedef struct gap_sec_key
{
    /// Key value MSB -> LSB
    uint8_t key[GAP_KEY_LEN];
} gap_sec_key_t;

/// I/Q sample
/*@TRACE*/
struct gap_iq_sample
{
    /// I sample
    uint8_t i;
    /// Q sample
    uint8_t q;
};

/// BIG Info Report
/*@TRACE*/
typedef struct gap_big_info
{
    /// Value of the SDU interval in microseconds (Range 0x0000FF-0x0FFFFF)
    uint32_t  sdu_interval;
    /// Value of the ISO Interval (1.25 ms unit)
    uint16_t  iso_interval;
    /// Value of the maximum PDU size (Range 0x0000-0x00FB)
    uint16_t  max_pdu;
    /// VValue of the maximum SDU size (Range 0x0000-0x0FFF)
    uint16_t  max_sdu;
    /// Number of BIS present in the group (Range 0x01-0x1F)
    uint8_t   num_bis;
    /// Number of sub-events (Range 0x01-0x1F)
    uint8_t   nse;
    /// Burst number (Range 0x01-0x07)
    uint8_t   bn;
    /// Pre-transmit offset (Range 0x00-0x0F)
    uint8_t   pto;
    /// Initial retransmission count (Range 0x01-0x0F)
    uint8_t   irc;
    /// PHY used for transmission (0x01: 1M, 0x02: 2M, 0x03: Coded, All other values: RFU)
    uint8_t   phy;
    /// Framing mode (0x00: Unframed, 0x01: Framed, All other values: RFU)
    uint8_t   framing;
    /// True if broadcast isochronous group is encrypted, False otherwise
    bool      encrypted;
} gap_big_info_t;
/// @} GAP
#endif // GAP_H_
