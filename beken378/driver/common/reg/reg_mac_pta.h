/**
 * @file reg_mac_pta.h
 * @brief Definitions of the NXMAC HW block registers and register access functions.
 *
 * @defgroup REG_MAC_PTA REG_MAC_PTA
 * @ingroup REG
 * @{
 *
 * @brief Definitions of the NXMAC HW block registers and register access functions.
 */
#ifndef _REG_MAC_PTA_H_
#define _REG_MAC_PTA_H_

#include "co_int.h"
#include "_reg_mac_pta.h"
#include "compiler.h"
#include "arch.h"
#include "reg_access.h"

/** @brief Number of registers in the REG_MAC_PTA peripheral.
 */
#define REG_MAC_PTA_COUNT 10

/** @brief Decoding mask of the REG_MAC_PTA peripheral registers from the CPU point of view.
 */
#define REG_MAC_PTA_DECODING_MASK 0x0000003F

/**
 * @name CONFIG register definitions
 * <table>
 * <caption id="CONFIG_BF">CONFIG bitfields</caption>
 * <tr><th>Bits <th>Field Name <th>HW Access <th>SW Access <th>Reset Value
 * <tr><td>19 <td>  sw_wlan_rx_abort <td>R <td>R/W <td>0
 * <tr><td>18 <td>  sw_wlan_tx_abort <td>R <td>R/W <td>0
 * <tr><td>17 <td>    sw_bt_rx_abort <td>R <td>R/W <td>0
 * <tr><td>16 <td>    sw_bt_tx_abort <td>R <td>R/W <td>0
 * <tr><td>13:08 <td>       chan_margin <td>R <td>R/W <td>0x0
 * <tr><td>07 <td>     wlan_pti_mode <td>R <td>R/W <td>0
 * <tr><td>06 <td>     bt_event_mask <td>R <td>R/W <td>0
 * <tr><td>05 <td>       chan_enable <td>R <td>R/W <td>0
 * <tr><td>04 <td>        pti_enable <td>R <td>R/W <td>0
 * <tr><td>03 <td>         no_sim_rx <td>R <td>R/W <td>0
 * <tr><td>02 <td>         no_sim_tx <td>R <td>R/W <td>0
 * <tr><td>01 <td>    basic_priority <td>R <td>R/W <td>0
 * <tr><td>00 <td>        pta_enable <td>R <td>R/W <td>0
 * </table>
 *
 * @{
 */

/// Address of the CONFIG register
#define NXMAC_CONFIG_ADDR   0xC0010004
/// Offset of the CONFIG register from the base address
#define NXMAC_CONFIG_OFFSET 0x00000004
/// Index of the CONFIG register
#define NXMAC_CONFIG_INDEX  0x00000001
/// Reset value of the CONFIG register
#define NXMAC_CONFIG_RESET  0x00000000

/**
 * @brief Returns the current value of the CONFIG register.
 * The CONFIG register will be read and its value returned.
 * @return The current value of the CONFIG register.
 */
__INLINE uint32_t nxmac_config_get(void)
{
    return REG_PL_RD(NXMAC_CONFIG_ADDR);
}

/**
 * @brief Sets the CONFIG register to a value.
 * The CONFIG register will be written.
 * @param value - The value to write.
 */
__INLINE void nxmac_config_set(uint32_t value)
{
    REG_PL_WR(NXMAC_CONFIG_ADDR, value);
}

// field definitions
/// SW_WLAN_RX_ABORT field bit
#define NXMAC_SW_WLAN_RX_ABORT_BIT    ((uint32_t)0x00080000)
/// SW_WLAN_RX_ABORT field position
#define NXMAC_SW_WLAN_RX_ABORT_POS    19
/// SW_WLAN_TX_ABORT field bit
#define NXMAC_SW_WLAN_TX_ABORT_BIT    ((uint32_t)0x00040000)
/// SW_WLAN_TX_ABORT field position
#define NXMAC_SW_WLAN_TX_ABORT_POS    18
/// SW_BT_RX_ABORT field bit
#define NXMAC_SW_BT_RX_ABORT_BIT      ((uint32_t)0x00020000)
/// SW_BT_RX_ABORT field position
#define NXMAC_SW_BT_RX_ABORT_POS      17
/// SW_BT_TX_ABORT field bit
#define NXMAC_SW_BT_TX_ABORT_BIT      ((uint32_t)0x00010000)
/// SW_BT_TX_ABORT field position
#define NXMAC_SW_BT_TX_ABORT_POS      16
/// CHAN_MARGIN field mask
#define NXMAC_CHAN_MARGIN_MASK        ((uint32_t)0x00003F00)
/// CHAN_MARGIN field LSB position
#define NXMAC_CHAN_MARGIN_LSB         8
/// CHAN_MARGIN field width
#define NXMAC_CHAN_MARGIN_WIDTH       ((uint32_t)0x00000006)
/// WLAN_PTI_MODE field bit
#define NXMAC_WLAN_PTI_MODE_BIT       ((uint32_t)0x00000080)
/// WLAN_PTI_MODE field position
#define NXMAC_WLAN_PTI_MODE_POS       7
/// BT_EVENT_MASK field bit
#define NXMAC_BT_EVENT_MASK_BIT       ((uint32_t)0x00000040)
/// BT_EVENT_MASK field position
#define NXMAC_BT_EVENT_MASK_POS       6
/// CHAN_ENABLE field bit
#define NXMAC_CHAN_ENABLE_BIT         ((uint32_t)0x00000020)
/// CHAN_ENABLE field position
#define NXMAC_CHAN_ENABLE_POS         5
/// PTI_ENABLE field bit
#define NXMAC_PTI_ENABLE_BIT          ((uint32_t)0x00000010)
/// PTI_ENABLE field position
#define NXMAC_PTI_ENABLE_POS          4
/// NO_SIM_RX field bit
#define NXMAC_NO_SIM_RX_BIT           ((uint32_t)0x00000008)
/// NO_SIM_RX field position
#define NXMAC_NO_SIM_RX_POS           3
/// NO_SIM_TX field bit
#define NXMAC_NO_SIM_TX_BIT           ((uint32_t)0x00000004)
/// NO_SIM_TX field position
#define NXMAC_NO_SIM_TX_POS           2
/// BASIC_PRIORITY field bit
#define NXMAC_BASIC_PRIORITY_BIT      ((uint32_t)0x00000002)
/// BASIC_PRIORITY field position
#define NXMAC_BASIC_PRIORITY_POS      1
/// PTA_ENABLE field bit
#define NXMAC_PTA_ENABLE_BIT          ((uint32_t)0x00000001)
/// PTA_ENABLE field position
#define NXMAC_PTA_ENABLE_POS          0

/// SW_WLAN_RX_ABORT field reset value
#define NXMAC_SW_WLAN_RX_ABORT_RST    0x0
/// SW_WLAN_TX_ABORT field reset value
#define NXMAC_SW_WLAN_TX_ABORT_RST    0x0
/// SW_BT_RX_ABORT field reset value
#define NXMAC_SW_BT_RX_ABORT_RST      0x0
/// SW_BT_TX_ABORT field reset value
#define NXMAC_SW_BT_TX_ABORT_RST      0x0
/// CHAN_MARGIN field reset value
#define NXMAC_CHAN_MARGIN_RST         0x0
/// WLAN_PTI_MODE field reset value
#define NXMAC_WLAN_PTI_MODE_RST       0x0
/// BT_EVENT_MASK field reset value
#define NXMAC_BT_EVENT_MASK_RST       0x0
/// CHAN_ENABLE field reset value
#define NXMAC_CHAN_ENABLE_RST         0x0
/// PTI_ENABLE field reset value
#define NXMAC_PTI_ENABLE_RST          0x0
/// NO_SIM_RX field reset value
#define NXMAC_NO_SIM_RX_RST           0x0
/// NO_SIM_TX field reset value
#define NXMAC_NO_SIM_TX_RST           0x0
/// BASIC_PRIORITY field reset value
#define NXMAC_BASIC_PRIORITY_RST      0x0
/// PTA_ENABLE field reset value
#define NXMAC_PTA_ENABLE_RST          0x0

/**
 * @brief Constructs a value for the CONFIG register given values for its fields
 * and writes the value to the register.
 *
 * @param[in] swwlanrxabort - The value to use for the sw_wlan_rx_abort field.
 * @param[in] swwlantxabort - The value to use for the sw_wlan_tx_abort field.
 * @param[in] swbtrxabort - The value to use for the sw_bt_rx_abort field.
 * @param[in] swbttxabort - The value to use for the sw_bt_tx_abort field.
 * @param[in] chanmargin - The value to use for the chan_margin field.
 * @param[in] wlanptimode - The value to use for the wlan_pti_mode field.
 * @param[in] bteventmask - The value to use for the bt_event_mask field.
 * @param[in] chanenable - The value to use for the chan_enable field.
 * @param[in] ptienable - The value to use for the pti_enable field.
 * @param[in] nosimrx - The value to use for the no_sim_rx field.
 * @param[in] nosimtx - The value to use for the no_sim_tx field.
 * @param[in] basicpriority - The value to use for the basic_priority field.
 * @param[in] ptaenable - The value to use for the pta_enable field.
 */
__INLINE void nxmac_config_pack(uint8_t swwlanrxabort, uint8_t swwlantxabort, uint8_t swbtrxabort, uint8_t swbttxabort, uint8_t chanmargin, uint8_t wlanptimode, uint8_t bteventmask, uint8_t chanenable, uint8_t ptienable, uint8_t nosimrx, uint8_t nosimtx, uint8_t basicpriority, uint8_t ptaenable)
{
    ASSERT_ERR((((uint32_t)swwlanrxabort << 19) & ~((uint32_t)0x00080000)) == 0);
    ASSERT_ERR((((uint32_t)swwlantxabort << 18) & ~((uint32_t)0x00040000)) == 0);
    ASSERT_ERR((((uint32_t)swbtrxabort << 17) & ~((uint32_t)0x00020000)) == 0);
    ASSERT_ERR((((uint32_t)swbttxabort << 16) & ~((uint32_t)0x00010000)) == 0);
    ASSERT_ERR((((uint32_t)chanmargin << 8) & ~((uint32_t)0x00003F00)) == 0);
    ASSERT_ERR((((uint32_t)wlanptimode << 7) & ~((uint32_t)0x00000080)) == 0);
    ASSERT_ERR((((uint32_t)bteventmask << 6) & ~((uint32_t)0x00000040)) == 0);
    ASSERT_ERR((((uint32_t)chanenable << 5) & ~((uint32_t)0x00000020)) == 0);
    ASSERT_ERR((((uint32_t)ptienable << 4) & ~((uint32_t)0x00000010)) == 0);
    ASSERT_ERR((((uint32_t)nosimrx << 3) & ~((uint32_t)0x00000008)) == 0);
    ASSERT_ERR((((uint32_t)nosimtx << 2) & ~((uint32_t)0x00000004)) == 0);
    ASSERT_ERR((((uint32_t)basicpriority << 1) & ~((uint32_t)0x00000002)) == 0);
    ASSERT_ERR((((uint32_t)ptaenable << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(NXMAC_CONFIG_ADDR,  ((uint32_t)swwlanrxabort << 19) | ((uint32_t)swwlantxabort << 18) | ((uint32_t)swbtrxabort << 17) | ((uint32_t)swbttxabort << 16) | ((uint32_t)chanmargin << 8) | ((uint32_t)wlanptimode << 7) | ((uint32_t)bteventmask << 6) | ((uint32_t)chanenable << 5) | ((uint32_t)ptienable << 4) | ((uint32_t)nosimrx << 3) | ((uint32_t)nosimtx << 2) | ((uint32_t)basicpriority << 1) | ((uint32_t)ptaenable << 0));
}

/**
 * @brief Unpacks CONFIG's fields from current value of the CONFIG register.
 *
 * Reads the CONFIG register and populates all the _field variables with the corresponding
 * values from the register.
 *
 * @param[out] swwlanrxabort - Will be populated with the current value of this field from the register.
 * @param[out] swwlantxabort - Will be populated with the current value of this field from the register.
 * @param[out] swbtrxabort - Will be populated with the current value of this field from the register.
 * @param[out] swbttxabort - Will be populated with the current value of this field from the register.
 * @param[out] chanmargin - Will be populated with the current value of this field from the register.
 * @param[out] wlanptimode - Will be populated with the current value of this field from the register.
 * @param[out] bteventmask - Will be populated with the current value of this field from the register.
 * @param[out] chanenable - Will be populated with the current value of this field from the register.
 * @param[out] ptienable - Will be populated with the current value of this field from the register.
 * @param[out] nosimrx - Will be populated with the current value of this field from the register.
 * @param[out] nosimtx - Will be populated with the current value of this field from the register.
 * @param[out] basicpriority - Will be populated with the current value of this field from the register.
 * @param[out] ptaenable - Will be populated with the current value of this field from the register.
 */
__INLINE void nxmac_config_unpack(uint8_t* swwlanrxabort, uint8_t* swwlantxabort, uint8_t* swbtrxabort, uint8_t* swbttxabort, uint8_t* chanmargin, uint8_t* wlanptimode, uint8_t* bteventmask, uint8_t* chanenable, uint8_t* ptienable, uint8_t* nosimrx, uint8_t* nosimtx, uint8_t* basicpriority, uint8_t* ptaenable)
{
    uint32_t localVal = REG_PL_RD(NXMAC_CONFIG_ADDR);

    *swwlanrxabort = (localVal & ((uint32_t)0x00080000)) >> 19;
    *swwlantxabort = (localVal & ((uint32_t)0x00040000)) >> 18;
    *swbtrxabort = (localVal & ((uint32_t)0x00020000)) >> 17;
    *swbttxabort = (localVal & ((uint32_t)0x00010000)) >> 16;
    *chanmargin = (localVal & ((uint32_t)0x00003F00)) >> 8;
    *wlanptimode = (localVal & ((uint32_t)0x00000080)) >> 7;
    *bteventmask = (localVal & ((uint32_t)0x00000040)) >> 6;
    *chanenable = (localVal & ((uint32_t)0x00000020)) >> 5;
    *ptienable = (localVal & ((uint32_t)0x00000010)) >> 4;
    *nosimrx = (localVal & ((uint32_t)0x00000008)) >> 3;
    *nosimtx = (localVal & ((uint32_t)0x00000004)) >> 2;
    *basicpriority = (localVal & ((uint32_t)0x00000002)) >> 1;
    *ptaenable = (localVal & ((uint32_t)0x00000001)) >> 0;
}

/**
 * @brief Returns the current value of the sw_wlan_rx_abort field in the CONFIG register.
 *
 * The CONFIG register will be read and the sw_wlan_rx_abort field's value will be returned.
 *
 * @return The current value of the sw_wlan_rx_abort field in the CONFIG register.
 */
__INLINE uint8_t nxmac_sw_wlan_rx_abort_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00080000)) >> 19);
}

/**
 * @brief Sets the sw_wlan_rx_abort field of the CONFIG register.
 *
 * The CONFIG register will be read, modified to contain the new field value, and written.
 *
 * @param[in] swwlanrxabort - The value to set the field to.
 */
__INLINE void nxmac_sw_wlan_rx_abort_setf(uint8_t swwlanrxabort)
{
    ASSERT_ERR((((uint32_t)swwlanrxabort << 19) & ~((uint32_t)0x00080000)) == 0);
    REG_PL_WR(NXMAC_CONFIG_ADDR, (REG_PL_RD(NXMAC_CONFIG_ADDR) & ~((uint32_t)0x00080000)) | ((uint32_t)swwlanrxabort << 19));
}

/**
 * @brief Returns the current value of the sw_wlan_tx_abort field in the CONFIG register.
 *
 * The CONFIG register will be read and the sw_wlan_tx_abort field's value will be returned.
 *
 * @return The current value of the sw_wlan_tx_abort field in the CONFIG register.
 */
__INLINE uint8_t nxmac_sw_wlan_tx_abort_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

/**
 * @brief Sets the sw_wlan_tx_abort field of the CONFIG register.
 *
 * The CONFIG register will be read, modified to contain the new field value, and written.
 *
 * @param[in] swwlantxabort - The value to set the field to.
 */
__INLINE void nxmac_sw_wlan_tx_abort_setf(uint8_t swwlantxabort)
{
    ASSERT_ERR((((uint32_t)swwlantxabort << 18) & ~((uint32_t)0x00040000)) == 0);
    REG_PL_WR(NXMAC_CONFIG_ADDR, (REG_PL_RD(NXMAC_CONFIG_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)swwlantxabort << 18));
}

/**
 * @brief Returns the current value of the sw_bt_rx_abort field in the CONFIG register.
 *
 * The CONFIG register will be read and the sw_bt_rx_abort field's value will be returned.
 *
 * @return The current value of the sw_bt_rx_abort field in the CONFIG register.
 */
__INLINE uint8_t nxmac_sw_bt_rx_abort_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

/**
 * @brief Sets the sw_bt_rx_abort field of the CONFIG register.
 *
 * The CONFIG register will be read, modified to contain the new field value, and written.
 *
 * @param[in] swbtrxabort - The value to set the field to.
 */
__INLINE void nxmac_sw_bt_rx_abort_setf(uint8_t swbtrxabort)
{
    ASSERT_ERR((((uint32_t)swbtrxabort << 17) & ~((uint32_t)0x00020000)) == 0);
    REG_PL_WR(NXMAC_CONFIG_ADDR, (REG_PL_RD(NXMAC_CONFIG_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)swbtrxabort << 17));
}

/**
 * @brief Returns the current value of the sw_bt_tx_abort field in the CONFIG register.
 *
 * The CONFIG register will be read and the sw_bt_tx_abort field's value will be returned.
 *
 * @return The current value of the sw_bt_tx_abort field in the CONFIG register.
 */
__INLINE uint8_t nxmac_sw_bt_tx_abort_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

/**
 * @brief Sets the sw_bt_tx_abort field of the CONFIG register.
 *
 * The CONFIG register will be read, modified to contain the new field value, and written.
 *
 * @param[in] swbttxabort - The value to set the field to.
 */
__INLINE void nxmac_sw_bt_tx_abort_setf(uint8_t swbttxabort)
{
    ASSERT_ERR((((uint32_t)swbttxabort << 16) & ~((uint32_t)0x00010000)) == 0);
    REG_PL_WR(NXMAC_CONFIG_ADDR, (REG_PL_RD(NXMAC_CONFIG_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)swbttxabort << 16));
}

/**
 * @brief Returns the current value of the chan_margin field in the CONFIG register.
 *
 * The CONFIG register will be read and the chan_margin field's value will be returned.
 *
 * @return The current value of the chan_margin field in the CONFIG register.
 */
__INLINE uint8_t nxmac_chan_margin_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00003F00)) >> 8);
}

/**
 * @brief Sets the chan_margin field of the CONFIG register.
 *
 * The CONFIG register will be read, modified to contain the new field value, and written.
 *
 * @param[in] chanmargin - The value to set the field to.
 */
__INLINE void nxmac_chan_margin_setf(uint8_t chanmargin)
{
    ASSERT_ERR((((uint32_t)chanmargin << 8) & ~((uint32_t)0x00003F00)) == 0);
    REG_PL_WR(NXMAC_CONFIG_ADDR, (REG_PL_RD(NXMAC_CONFIG_ADDR) & ~((uint32_t)0x00003F00)) | ((uint32_t)chanmargin << 8));
}

/**
 * @brief Returns the current value of the wlan_pti_mode field in the CONFIG register.
 *
 * The CONFIG register will be read and the wlan_pti_mode field's value will be returned.
 *
 * @return The current value of the wlan_pti_mode field in the CONFIG register.
 */
__INLINE uint8_t nxmac_wlan_pti_mode_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

/**
 * @brief Sets the wlan_pti_mode field of the CONFIG register.
 *
 * The CONFIG register will be read, modified to contain the new field value, and written.
 *
 * @param[in] wlanptimode - The value to set the field to.
 */
__INLINE void nxmac_wlan_pti_mode_setf(uint8_t wlanptimode)
{
    ASSERT_ERR((((uint32_t)wlanptimode << 7) & ~((uint32_t)0x00000080)) == 0);
    REG_PL_WR(NXMAC_CONFIG_ADDR, (REG_PL_RD(NXMAC_CONFIG_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)wlanptimode << 7));
}

/**
 * @brief Returns the current value of the bt_event_mask field in the CONFIG register.
 *
 * The CONFIG register will be read and the bt_event_mask field's value will be returned.
 *
 * @return The current value of the bt_event_mask field in the CONFIG register.
 */
__INLINE uint8_t nxmac_bt_event_mask_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

/**
 * @brief Sets the bt_event_mask field of the CONFIG register.
 *
 * The CONFIG register will be read, modified to contain the new field value, and written.
 *
 * @param[in] bteventmask - The value to set the field to.
 */
__INLINE void nxmac_bt_event_mask_setf(uint8_t bteventmask)
{
    ASSERT_ERR((((uint32_t)bteventmask << 6) & ~((uint32_t)0x00000040)) == 0);
    REG_PL_WR(NXMAC_CONFIG_ADDR, (REG_PL_RD(NXMAC_CONFIG_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)bteventmask << 6));
}

/**
 * @brief Returns the current value of the chan_enable field in the CONFIG register.
 *
 * The CONFIG register will be read and the chan_enable field's value will be returned.
 *
 * @return The current value of the chan_enable field in the CONFIG register.
 */
__INLINE uint8_t nxmac_chan_enable_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

/**
 * @brief Sets the chan_enable field of the CONFIG register.
 *
 * The CONFIG register will be read, modified to contain the new field value, and written.
 *
 * @param[in] chanenable - The value to set the field to.
 */
__INLINE void nxmac_chan_enable_setf(uint8_t chanenable)
{
    ASSERT_ERR((((uint32_t)chanenable << 5) & ~((uint32_t)0x00000020)) == 0);
    REG_PL_WR(NXMAC_CONFIG_ADDR, (REG_PL_RD(NXMAC_CONFIG_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)chanenable << 5));
}

/**
 * @brief Returns the current value of the pti_enable field in the CONFIG register.
 *
 * The CONFIG register will be read and the pti_enable field's value will be returned.
 *
 * @return The current value of the pti_enable field in the CONFIG register.
 */
__INLINE uint8_t nxmac_pti_enable_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

/**
 * @brief Sets the pti_enable field of the CONFIG register.
 *
 * The CONFIG register will be read, modified to contain the new field value, and written.
 *
 * @param[in] ptienable - The value to set the field to.
 */
__INLINE void nxmac_pti_enable_setf(uint8_t ptienable)
{
    ASSERT_ERR((((uint32_t)ptienable << 4) & ~((uint32_t)0x00000010)) == 0);
    REG_PL_WR(NXMAC_CONFIG_ADDR, (REG_PL_RD(NXMAC_CONFIG_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)ptienable << 4));
}

/**
 * @brief Returns the current value of the no_sim_rx field in the CONFIG register.
 *
 * The CONFIG register will be read and the no_sim_rx field's value will be returned.
 *
 * @return The current value of the no_sim_rx field in the CONFIG register.
 */
__INLINE uint8_t nxmac_no_sim_rx_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

/**
 * @brief Sets the no_sim_rx field of the CONFIG register.
 *
 * The CONFIG register will be read, modified to contain the new field value, and written.
 *
 * @param[in] nosimrx - The value to set the field to.
 */
__INLINE void nxmac_no_sim_rx_setf(uint8_t nosimrx)
{
    ASSERT_ERR((((uint32_t)nosimrx << 3) & ~((uint32_t)0x00000008)) == 0);
    REG_PL_WR(NXMAC_CONFIG_ADDR, (REG_PL_RD(NXMAC_CONFIG_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)nosimrx << 3));
}

/**
 * @brief Returns the current value of the no_sim_tx field in the CONFIG register.
 *
 * The CONFIG register will be read and the no_sim_tx field's value will be returned.
 *
 * @return The current value of the no_sim_tx field in the CONFIG register.
 */
__INLINE uint8_t nxmac_no_sim_tx_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

/**
 * @brief Sets the no_sim_tx field of the CONFIG register.
 *
 * The CONFIG register will be read, modified to contain the new field value, and written.
 *
 * @param[in] nosimtx - The value to set the field to.
 */
__INLINE void nxmac_no_sim_tx_setf(uint8_t nosimtx)
{
    ASSERT_ERR((((uint32_t)nosimtx << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_PL_WR(NXMAC_CONFIG_ADDR, (REG_PL_RD(NXMAC_CONFIG_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)nosimtx << 2));
}

/**
 * @brief Returns the current value of the basic_priority field in the CONFIG register.
 *
 * The CONFIG register will be read and the basic_priority field's value will be returned.
 *
 * @return The current value of the basic_priority field in the CONFIG register.
 */
__INLINE uint8_t nxmac_basic_priority_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

/**
 * @brief Sets the basic_priority field of the CONFIG register.
 *
 * The CONFIG register will be read, modified to contain the new field value, and written.
 *
 * @param[in] basicpriority - The value to set the field to.
 */
__INLINE void nxmac_basic_priority_setf(uint8_t basicpriority)
{
    ASSERT_ERR((((uint32_t)basicpriority << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_PL_WR(NXMAC_CONFIG_ADDR, (REG_PL_RD(NXMAC_CONFIG_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)basicpriority << 1));
}

/**
 * @brief Returns the current value of the pta_enable field in the CONFIG register.
 *
 * The CONFIG register will be read and the pta_enable field's value will be returned.
 *
 * @return The current value of the pta_enable field in the CONFIG register.
 */
__INLINE uint8_t nxmac_pta_enable_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_CONFIG_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

/**
 * @brief Sets the pta_enable field of the CONFIG register.
 *
 * The CONFIG register will be read, modified to contain the new field value, and written.
 *
 * @param[in] ptaenable - The value to set the field to.
 */
__INLINE void nxmac_pta_enable_setf(uint8_t ptaenable)
{
    ASSERT_ERR((((uint32_t)ptaenable << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_PL_WR(NXMAC_CONFIG_ADDR, (REG_PL_RD(NXMAC_CONFIG_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)ptaenable << 0));
}

/// @}

/**
 * @name STAT_BT_TX register definitions
 * <table>
 * <caption id="STAT_BT_TX_BF">STAT_BT_TX bitfields</caption>
 * <tr><th>Bits <th>Field Name <th>HW Access <th>SW Access <th>Reset Value
 * <tr><td>31:00 <td>          statBtTx <td>R <td>R/W <td>0x0
 * </table>
 *
 * @{
 */

/// Address of the STAT_BT_TX register
#define NXMAC_STAT_BT_TX_ADDR   0xC0010008
/// Offset of the STAT_BT_TX register from the base address
#define NXMAC_STAT_BT_TX_OFFSET 0x00000008
/// Index of the STAT_BT_TX register
#define NXMAC_STAT_BT_TX_INDEX  0x00000002
/// Reset value of the STAT_BT_TX register
#define NXMAC_STAT_BT_TX_RESET  0x00000000

/**
 * @brief Returns the current value of the STAT_BT_TX register.
 * The STAT_BT_TX register will be read and its value returned.
 * @return The current value of the STAT_BT_TX register.
 */
__INLINE uint32_t nxmac_stat_bt_tx_get(void)
{
    return REG_PL_RD(NXMAC_STAT_BT_TX_ADDR);
}

/**
 * @brief Sets the STAT_BT_TX register to a value.
 * The STAT_BT_TX register will be written.
 * @param value - The value to write.
 */
__INLINE void nxmac_stat_bt_tx_set(uint32_t value)
{
    REG_PL_WR(NXMAC_STAT_BT_TX_ADDR, value);
}

// field definitions
/// STAT_BT_TX field mask
#define NXMAC_STAT_BT_TX_MASK   ((uint32_t)0xFFFFFFFF)
/// STAT_BT_TX field LSB position
#define NXMAC_STAT_BT_TX_LSB    0
/// STAT_BT_TX field width
#define NXMAC_STAT_BT_TX_WIDTH  ((uint32_t)0x00000020)

/// STAT_BT_TX field reset value
#define NXMAC_STAT_BT_TX_RST    0x0

/**
 * @brief Returns the current value of the statBtTx field in the STAT_BT_TX register.
 *
 * The STAT_BT_TX register will be read and the statBtTx field's value will be returned.
 *
 * @return The current value of the statBtTx field in the STAT_BT_TX register.
 */
__INLINE uint32_t nxmac_stat_bt_tx_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_STAT_BT_TX_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief Sets the statBtTx field of the STAT_BT_TX register.
 *
 * The STAT_BT_TX register will be read, modified to contain the new field value, and written.
 *
 * @param[in] statbttx - The value to set the field to.
 */
__INLINE void nxmac_stat_bt_tx_setf(uint32_t statbttx)
{
    ASSERT_ERR((((uint32_t)statbttx << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(NXMAC_STAT_BT_TX_ADDR, (uint32_t)statbttx << 0);
}

/// @}

/**
 * @name STAT_BT_ABORT_TX register definitions
 * <table>
 * <caption id="STAT_BT_ABORT_TX_BF">STAT_BT_ABORT_TX bitfields</caption>
 * <tr><th>Bits <th>Field Name <th>HW Access <th>SW Access <th>Reset Value
 * <tr><td>31:00 <td>     statBtTxAbort <td>R <td>R/W <td>0x0
 * </table>
 *
 * @{
 */

/// Address of the STAT_BT_ABORT_TX register
#define NXMAC_STAT_BT_ABORT_TX_ADDR   0xC001000C
/// Offset of the STAT_BT_ABORT_TX register from the base address
#define NXMAC_STAT_BT_ABORT_TX_OFFSET 0x0000000C
/// Index of the STAT_BT_ABORT_TX register
#define NXMAC_STAT_BT_ABORT_TX_INDEX  0x00000003
/// Reset value of the STAT_BT_ABORT_TX register
#define NXMAC_STAT_BT_ABORT_TX_RESET  0x00000000

/**
 * @brief Returns the current value of the STAT_BT_ABORT_TX register.
 * The STAT_BT_ABORT_TX register will be read and its value returned.
 * @return The current value of the STAT_BT_ABORT_TX register.
 */
__INLINE uint32_t nxmac_stat_bt_abort_tx_get(void)
{
    return REG_PL_RD(NXMAC_STAT_BT_ABORT_TX_ADDR);
}

/**
 * @brief Sets the STAT_BT_ABORT_TX register to a value.
 * The STAT_BT_ABORT_TX register will be written.
 * @param value - The value to write.
 */
__INLINE void nxmac_stat_bt_abort_tx_set(uint32_t value)
{
    REG_PL_WR(NXMAC_STAT_BT_ABORT_TX_ADDR, value);
}

// field definitions
/// STAT_BT_TX_ABORT field mask
#define NXMAC_STAT_BT_TX_ABORT_MASK   ((uint32_t)0xFFFFFFFF)
/// STAT_BT_TX_ABORT field LSB position
#define NXMAC_STAT_BT_TX_ABORT_LSB    0
/// STAT_BT_TX_ABORT field width
#define NXMAC_STAT_BT_TX_ABORT_WIDTH  ((uint32_t)0x00000020)

/// STAT_BT_TX_ABORT field reset value
#define NXMAC_STAT_BT_TX_ABORT_RST    0x0

/**
 * @brief Returns the current value of the statBtTxAbort field in the STAT_BT_ABORT_TX register.
 *
 * The STAT_BT_ABORT_TX register will be read and the statBtTxAbort field's value will be returned.
 *
 * @return The current value of the statBtTxAbort field in the STAT_BT_ABORT_TX register.
 */
__INLINE uint32_t nxmac_stat_bt_tx_abort_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_STAT_BT_ABORT_TX_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief Sets the statBtTxAbort field of the STAT_BT_ABORT_TX register.
 *
 * The STAT_BT_ABORT_TX register will be read, modified to contain the new field value, and written.
 *
 * @param[in] statbttxabort - The value to set the field to.
 */
__INLINE void nxmac_stat_bt_tx_abort_setf(uint32_t statbttxabort)
{
    ASSERT_ERR((((uint32_t)statbttxabort << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(NXMAC_STAT_BT_ABORT_TX_ADDR, (uint32_t)statbttxabort << 0);
}

/// @}

/**
 * @name STAT_BT_RX register definitions
 * <table>
 * <caption id="STAT_BT_RX_BF">STAT_BT_RX bitfields</caption>
 * <tr><th>Bits <th>Field Name <th>HW Access <th>SW Access <th>Reset Value
 * <tr><td>31:00 <td>          statBtRx <td>R <td>R/W <td>0x0
 * </table>
 *
 * @{
 */

/// Address of the STAT_BT_RX register
#define NXMAC_STAT_BT_RX_ADDR   0xC0010010
/// Offset of the STAT_BT_RX register from the base address
#define NXMAC_STAT_BT_RX_OFFSET 0x00000010
/// Index of the STAT_BT_RX register
#define NXMAC_STAT_BT_RX_INDEX  0x00000004
/// Reset value of the STAT_BT_RX register
#define NXMAC_STAT_BT_RX_RESET  0x00000000

/**
 * @brief Returns the current value of the STAT_BT_RX register.
 * The STAT_BT_RX register will be read and its value returned.
 * @return The current value of the STAT_BT_RX register.
 */
__INLINE uint32_t nxmac_stat_bt_rx_get(void)
{
    return REG_PL_RD(NXMAC_STAT_BT_RX_ADDR);
}

/**
 * @brief Sets the STAT_BT_RX register to a value.
 * The STAT_BT_RX register will be written.
 * @param value - The value to write.
 */
__INLINE void nxmac_stat_bt_rx_set(uint32_t value)
{
    REG_PL_WR(NXMAC_STAT_BT_RX_ADDR, value);
}

// field definitions
/// STAT_BT_RX field mask
#define NXMAC_STAT_BT_RX_MASK   ((uint32_t)0xFFFFFFFF)
/// STAT_BT_RX field LSB position
#define NXMAC_STAT_BT_RX_LSB    0
/// STAT_BT_RX field width
#define NXMAC_STAT_BT_RX_WIDTH  ((uint32_t)0x00000020)

/// STAT_BT_RX field reset value
#define NXMAC_STAT_BT_RX_RST    0x0

/**
 * @brief Returns the current value of the statBtRx field in the STAT_BT_RX register.
 *
 * The STAT_BT_RX register will be read and the statBtRx field's value will be returned.
 *
 * @return The current value of the statBtRx field in the STAT_BT_RX register.
 */
__INLINE uint32_t nxmac_stat_bt_rx_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_STAT_BT_RX_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief Sets the statBtRx field of the STAT_BT_RX register.
 *
 * The STAT_BT_RX register will be read, modified to contain the new field value, and written.
 *
 * @param[in] statbtrx - The value to set the field to.
 */
__INLINE void nxmac_stat_bt_rx_setf(uint32_t statbtrx)
{
    ASSERT_ERR((((uint32_t)statbtrx << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(NXMAC_STAT_BT_RX_ADDR, (uint32_t)statbtrx << 0);
}

/// @}

/**
 * @name STAT_BT_RX_ABORT register definitions
 * <table>
 * <caption id="STAT_BT_RX_ABORT_BF">STAT_BT_RX_ABORT bitfields</caption>
 * <tr><th>Bits <th>Field Name <th>HW Access <th>SW Access <th>Reset Value
 * <tr><td>31:00 <td>     statBtRxAbort <td>R <td>R/W <td>0x0
 * </table>
 *
 * @{
 */

/// Address of the STAT_BT_RX_ABORT register
#define NXMAC_STAT_BT_RX_ABORT_ADDR   0xC0010014
/// Offset of the STAT_BT_RX_ABORT register from the base address
#define NXMAC_STAT_BT_RX_ABORT_OFFSET 0x00000014
/// Index of the STAT_BT_RX_ABORT register
#define NXMAC_STAT_BT_RX_ABORT_INDEX  0x00000005
/// Reset value of the STAT_BT_RX_ABORT register
#define NXMAC_STAT_BT_RX_ABORT_RESET  0x00000000

/**
 * @brief Returns the current value of the STAT_BT_RX_ABORT register.
 * The STAT_BT_RX_ABORT register will be read and its value returned.
 * @return The current value of the STAT_BT_RX_ABORT register.
 */
__INLINE uint32_t nxmac_stat_bt_rx_abort_get(void)
{
    return REG_PL_RD(NXMAC_STAT_BT_RX_ABORT_ADDR);
}

/**
 * @brief Sets the STAT_BT_RX_ABORT register to a value.
 * The STAT_BT_RX_ABORT register will be written.
 * @param value - The value to write.
 */
__INLINE void nxmac_stat_bt_rx_abort_set(uint32_t value)
{
    REG_PL_WR(NXMAC_STAT_BT_RX_ABORT_ADDR, value);
}

// field definitions
/// STAT_BT_RX_ABORT field mask
#define NXMAC_STAT_BT_RX_ABORT_MASK   ((uint32_t)0xFFFFFFFF)
/// STAT_BT_RX_ABORT field LSB position
#define NXMAC_STAT_BT_RX_ABORT_LSB    0
/// STAT_BT_RX_ABORT field width
#define NXMAC_STAT_BT_RX_ABORT_WIDTH  ((uint32_t)0x00000020)

/// STAT_BT_RX_ABORT field reset value
#define NXMAC_STAT_BT_RX_ABORT_RST    0x0

/**
 * @brief Returns the current value of the statBtRxAbort field in the STAT_BT_RX_ABORT register.
 *
 * The STAT_BT_RX_ABORT register will be read and the statBtRxAbort field's value will be returned.
 *
 * @return The current value of the statBtRxAbort field in the STAT_BT_RX_ABORT register.
 */
__INLINE uint32_t nxmac_stat_bt_rx_abort_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_STAT_BT_RX_ABORT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief Sets the statBtRxAbort field of the STAT_BT_RX_ABORT register.
 *
 * The STAT_BT_RX_ABORT register will be read, modified to contain the new field value, and written.
 *
 * @param[in] statbtrxabort - The value to set the field to.
 */
__INLINE void nxmac_stat_bt_rx_abort_setf(uint32_t statbtrxabort)
{
    ASSERT_ERR((((uint32_t)statbtrxabort << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(NXMAC_STAT_BT_RX_ABORT_ADDR, (uint32_t)statbtrxabort << 0);
}

/// @}

/**
 * @name STAT_WLAN_TX register definitions
 * <table>
 * <caption id="STAT_WLAN_TX_BF">STAT_WLAN_TX bitfields</caption>
 * <tr><th>Bits <th>Field Name <th>HW Access <th>SW Access <th>Reset Value
 * <tr><td>31:00 <td>        statWlanTx <td>R <td>R/W <td>0x0
 * </table>
 *
 * @{
 */

/// Address of the STAT_WLAN_TX register
#define NXMAC_STAT_WLAN_TX_ADDR   0xC0010018
/// Offset of the STAT_WLAN_TX register from the base address
#define NXMAC_STAT_WLAN_TX_OFFSET 0x00000018
/// Index of the STAT_WLAN_TX register
#define NXMAC_STAT_WLAN_TX_INDEX  0x00000006
/// Reset value of the STAT_WLAN_TX register
#define NXMAC_STAT_WLAN_TX_RESET  0x00000000

/**
 * @brief Returns the current value of the STAT_WLAN_TX register.
 * The STAT_WLAN_TX register will be read and its value returned.
 * @return The current value of the STAT_WLAN_TX register.
 */
__INLINE uint32_t nxmac_stat_wlan_tx_get(void)
{
    return REG_PL_RD(NXMAC_STAT_WLAN_TX_ADDR);
}

/**
 * @brief Sets the STAT_WLAN_TX register to a value.
 * The STAT_WLAN_TX register will be written.
 * @param value - The value to write.
 */
__INLINE void nxmac_stat_wlan_tx_set(uint32_t value)
{
    REG_PL_WR(NXMAC_STAT_WLAN_TX_ADDR, value);
}

// field definitions
/// STAT_WLAN_TX field mask
#define NXMAC_STAT_WLAN_TX_MASK   ((uint32_t)0xFFFFFFFF)
/// STAT_WLAN_TX field LSB position
#define NXMAC_STAT_WLAN_TX_LSB    0
/// STAT_WLAN_TX field width
#define NXMAC_STAT_WLAN_TX_WIDTH  ((uint32_t)0x00000020)

/// STAT_WLAN_TX field reset value
#define NXMAC_STAT_WLAN_TX_RST    0x0

/**
 * @brief Returns the current value of the statWlanTx field in the STAT_WLAN_TX register.
 *
 * The STAT_WLAN_TX register will be read and the statWlanTx field's value will be returned.
 *
 * @return The current value of the statWlanTx field in the STAT_WLAN_TX register.
 */
__INLINE uint32_t nxmac_stat_wlan_tx_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_STAT_WLAN_TX_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief Sets the statWlanTx field of the STAT_WLAN_TX register.
 *
 * The STAT_WLAN_TX register will be read, modified to contain the new field value, and written.
 *
 * @param[in] statwlantx - The value to set the field to.
 */
__INLINE void nxmac_stat_wlan_tx_setf(uint32_t statwlantx)
{
    ASSERT_ERR((((uint32_t)statwlantx << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(NXMAC_STAT_WLAN_TX_ADDR, (uint32_t)statwlantx << 0);
}

/// @}

/**
 * @name STAT_WLAN_ABORT_TX register definitions
 * <table>
 * <caption id="STAT_WLAN_ABORT_TX_BF">STAT_WLAN_ABORT_TX bitfields</caption>
 * <tr><th>Bits <th>Field Name <th>HW Access <th>SW Access <th>Reset Value
 * <tr><td>31:00 <td>   statWlanTxAbort <td>R <td>R/W <td>0x0
 * </table>
 *
 * @{
 */

/// Address of the STAT_WLAN_ABORT_TX register
#define NXMAC_STAT_WLAN_ABORT_TX_ADDR   0xC001001C
/// Offset of the STAT_WLAN_ABORT_TX register from the base address
#define NXMAC_STAT_WLAN_ABORT_TX_OFFSET 0x0000001C
/// Index of the STAT_WLAN_ABORT_TX register
#define NXMAC_STAT_WLAN_ABORT_TX_INDEX  0x00000007
/// Reset value of the STAT_WLAN_ABORT_TX register
#define NXMAC_STAT_WLAN_ABORT_TX_RESET  0x00000000

/**
 * @brief Returns the current value of the STAT_WLAN_ABORT_TX register.
 * The STAT_WLAN_ABORT_TX register will be read and its value returned.
 * @return The current value of the STAT_WLAN_ABORT_TX register.
 */
__INLINE uint32_t nxmac_stat_wlan_abort_tx_get(void)
{
    return REG_PL_RD(NXMAC_STAT_WLAN_ABORT_TX_ADDR);
}

/**
 * @brief Sets the STAT_WLAN_ABORT_TX register to a value.
 * The STAT_WLAN_ABORT_TX register will be written.
 * @param value - The value to write.
 */
__INLINE void nxmac_stat_wlan_abort_tx_set(uint32_t value)
{
    REG_PL_WR(NXMAC_STAT_WLAN_ABORT_TX_ADDR, value);
}

// field definitions
/// STAT_WLAN_TX_ABORT field mask
#define NXMAC_STAT_WLAN_TX_ABORT_MASK   ((uint32_t)0xFFFFFFFF)
/// STAT_WLAN_TX_ABORT field LSB position
#define NXMAC_STAT_WLAN_TX_ABORT_LSB    0
/// STAT_WLAN_TX_ABORT field width
#define NXMAC_STAT_WLAN_TX_ABORT_WIDTH  ((uint32_t)0x00000020)

/// STAT_WLAN_TX_ABORT field reset value
#define NXMAC_STAT_WLAN_TX_ABORT_RST    0x0

/**
 * @brief Returns the current value of the statWlanTxAbort field in the STAT_WLAN_ABORT_TX register.
 *
 * The STAT_WLAN_ABORT_TX register will be read and the statWlanTxAbort field's value will be returned.
 *
 * @return The current value of the statWlanTxAbort field in the STAT_WLAN_ABORT_TX register.
 */
__INLINE uint32_t nxmac_stat_wlan_tx_abort_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_STAT_WLAN_ABORT_TX_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief Sets the statWlanTxAbort field of the STAT_WLAN_ABORT_TX register.
 *
 * The STAT_WLAN_ABORT_TX register will be read, modified to contain the new field value, and written.
 *
 * @param[in] statwlantxabort - The value to set the field to.
 */
__INLINE void nxmac_stat_wlan_tx_abort_setf(uint32_t statwlantxabort)
{
    ASSERT_ERR((((uint32_t)statwlantxabort << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(NXMAC_STAT_WLAN_ABORT_TX_ADDR, (uint32_t)statwlantxabort << 0);
}

/// @}

/**
 * @name STAT_WLAN_RX register definitions
 * <table>
 * <caption id="STAT_WLAN_RX_BF">STAT_WLAN_RX bitfields</caption>
 * <tr><th>Bits <th>Field Name <th>HW Access <th>SW Access <th>Reset Value
 * <tr><td>31:00 <td>        statWlanRx <td>R <td>R/W <td>0x0
 * </table>
 *
 * @{
 */

/// Address of the STAT_WLAN_RX register
#define NXMAC_STAT_WLAN_RX_ADDR   0xC0010020
/// Offset of the STAT_WLAN_RX register from the base address
#define NXMAC_STAT_WLAN_RX_OFFSET 0x00000020
/// Index of the STAT_WLAN_RX register
#define NXMAC_STAT_WLAN_RX_INDEX  0x00000008
/// Reset value of the STAT_WLAN_RX register
#define NXMAC_STAT_WLAN_RX_RESET  0x00000000

/**
 * @brief Returns the current value of the STAT_WLAN_RX register.
 * The STAT_WLAN_RX register will be read and its value returned.
 * @return The current value of the STAT_WLAN_RX register.
 */
__INLINE uint32_t nxmac_stat_wlan_rx_get(void)
{
    return REG_PL_RD(NXMAC_STAT_WLAN_RX_ADDR);
}

/**
 * @brief Sets the STAT_WLAN_RX register to a value.
 * The STAT_WLAN_RX register will be written.
 * @param value - The value to write.
 */
__INLINE void nxmac_stat_wlan_rx_set(uint32_t value)
{
    REG_PL_WR(NXMAC_STAT_WLAN_RX_ADDR, value);
}

// field definitions
/// STAT_WLAN_RX field mask
#define NXMAC_STAT_WLAN_RX_MASK   ((uint32_t)0xFFFFFFFF)
/// STAT_WLAN_RX field LSB position
#define NXMAC_STAT_WLAN_RX_LSB    0
/// STAT_WLAN_RX field width
#define NXMAC_STAT_WLAN_RX_WIDTH  ((uint32_t)0x00000020)

/// STAT_WLAN_RX field reset value
#define NXMAC_STAT_WLAN_RX_RST    0x0

/**
 * @brief Returns the current value of the statWlanRx field in the STAT_WLAN_RX register.
 *
 * The STAT_WLAN_RX register will be read and the statWlanRx field's value will be returned.
 *
 * @return The current value of the statWlanRx field in the STAT_WLAN_RX register.
 */
__INLINE uint32_t nxmac_stat_wlan_rx_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_STAT_WLAN_RX_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief Sets the statWlanRx field of the STAT_WLAN_RX register.
 *
 * The STAT_WLAN_RX register will be read, modified to contain the new field value, and written.
 *
 * @param[in] statwlanrx - The value to set the field to.
 */
__INLINE void nxmac_stat_wlan_rx_setf(uint32_t statwlanrx)
{
    ASSERT_ERR((((uint32_t)statwlanrx << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(NXMAC_STAT_WLAN_RX_ADDR, (uint32_t)statwlanrx << 0);
}

/// @}

/**
 * @name STAT_WLAN_RX_ABORT register definitions
 * <table>
 * <caption id="STAT_WLAN_RX_ABORT_BF">STAT_WLAN_RX_ABORT bitfields</caption>
 * <tr><th>Bits <th>Field Name <th>HW Access <th>SW Access <th>Reset Value
 * <tr><td>31:00 <td>   statWlanRxAbort <td>R <td>R/W <td>0x0
 * </table>
 *
 * @{
 */

/// Address of the STAT_WLAN_RX_ABORT register
#define NXMAC_STAT_WLAN_RX_ABORT_ADDR   0xC0010024
/// Offset of the STAT_WLAN_RX_ABORT register from the base address
#define NXMAC_STAT_WLAN_RX_ABORT_OFFSET 0x00000024
/// Index of the STAT_WLAN_RX_ABORT register
#define NXMAC_STAT_WLAN_RX_ABORT_INDEX  0x00000009
/// Reset value of the STAT_WLAN_RX_ABORT register
#define NXMAC_STAT_WLAN_RX_ABORT_RESET  0x00000000

/**
 * @brief Returns the current value of the STAT_WLAN_RX_ABORT register.
 * The STAT_WLAN_RX_ABORT register will be read and its value returned.
 * @return The current value of the STAT_WLAN_RX_ABORT register.
 */
__INLINE uint32_t nxmac_stat_wlan_rx_abort_get(void)
{
    return REG_PL_RD(NXMAC_STAT_WLAN_RX_ABORT_ADDR);
}

/**
 * @brief Sets the STAT_WLAN_RX_ABORT register to a value.
 * The STAT_WLAN_RX_ABORT register will be written.
 * @param value - The value to write.
 */
__INLINE void nxmac_stat_wlan_rx_abort_set(uint32_t value)
{
    REG_PL_WR(NXMAC_STAT_WLAN_RX_ABORT_ADDR, value);
}

// field definitions
/// STAT_WLAN_RX_ABORT field mask
#define NXMAC_STAT_WLAN_RX_ABORT_MASK   ((uint32_t)0xFFFFFFFF)
/// STAT_WLAN_RX_ABORT field LSB position
#define NXMAC_STAT_WLAN_RX_ABORT_LSB    0
/// STAT_WLAN_RX_ABORT field width
#define NXMAC_STAT_WLAN_RX_ABORT_WIDTH  ((uint32_t)0x00000020)

/// STAT_WLAN_RX_ABORT field reset value
#define NXMAC_STAT_WLAN_RX_ABORT_RST    0x0

/**
 * @brief Returns the current value of the statWlanRxAbort field in the STAT_WLAN_RX_ABORT register.
 *
 * The STAT_WLAN_RX_ABORT register will be read and the statWlanRxAbort field's value will be returned.
 *
 * @return The current value of the statWlanRxAbort field in the STAT_WLAN_RX_ABORT register.
 */
__INLINE uint32_t nxmac_stat_wlan_rx_abort_getf(void)
{
    uint32_t localVal = REG_PL_RD(NXMAC_STAT_WLAN_RX_ABORT_ADDR);
    ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief Sets the statWlanRxAbort field of the STAT_WLAN_RX_ABORT register.
 *
 * The STAT_WLAN_RX_ABORT register will be read, modified to contain the new field value, and written.
 *
 * @param[in] statwlanrxabort - The value to set the field to.
 */
__INLINE void nxmac_stat_wlan_rx_abort_setf(uint32_t statwlanrxabort)
{
    ASSERT_ERR((((uint32_t)statwlanrxabort << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_PL_WR(NXMAC_STAT_WLAN_RX_ABORT_ADDR, (uint32_t)statwlanrxabort << 0);
}

/// @}


#endif // _REG_MAC_PTA_H_

/// @}

