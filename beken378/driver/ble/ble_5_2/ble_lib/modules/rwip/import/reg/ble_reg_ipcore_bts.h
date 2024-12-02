#ifndef _REG_IPCORE_BTS_H_
#define _REG_IPCORE_BTS_H_

#include <stdint.h>
#include "_reg_ipcore_bts.h"
#include "compiler.h"
#include "architect.h"
#include "ble_reg_access.h"

#define REG_IPCORE_BTS_COUNT 125

#define REG_IPCORE_BTS_DECODING_MASK 0x000001FF

/**
 * @brief ISOCNTCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31              ISOSAMP   0
 *     30              ISO_UPD   0
 *     02    ISO_CLKSHIFT_MODE   0
 *     01   ISO_PHASE_SHIFT_MODE   0
 *     00          ISOCORRMODE   0
 * </pre>
 */
#define BLE_ISOCNTCNTL_ADDR   0x009001C0
#define BLE_ISOCNTCNTL_OFFSET 0x000001C0
#define BLE_ISOCNTCNTL_INDEX  0x00000070
#define BLE_ISOCNTCNTL_RESET  0x00000000

__INLINE uint32_t ble_isocntcntl_get(void)
{
    return REG_BLE_RD(BLE_ISOCNTCNTL_ADDR);
}

__INLINE void ble_isocntcntl_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOCNTCNTL_ADDR, value);
}

// field definitions
#define BLE_ISOSAMP_BIT                 ((uint32_t)0x80000000)
#define BLE_ISOSAMP_POS                 31
#define BLE_ISO_UPD_BIT                 ((uint32_t)0x40000000)
#define BLE_ISO_UPD_POS                 30
#define BLE_ISO_CLKSHIFT_MODE_BIT       ((uint32_t)0x00000004)
#define BLE_ISO_CLKSHIFT_MODE_POS       2
#define BLE_ISO_PHASE_SHIFT_MODE_BIT    ((uint32_t)0x00000002)
#define BLE_ISO_PHASE_SHIFT_MODE_POS    1
#define BLE_ISOCORRMODE_BIT             ((uint32_t)0x00000001)
#define BLE_ISOCORRMODE_POS             0

#define BLE_ISOSAMP_RST                 0x0
#define BLE_ISO_UPD_RST                 0x0
#define BLE_ISO_CLKSHIFT_MODE_RST       0x0
#define BLE_ISO_PHASE_SHIFT_MODE_RST    0x0
#define BLE_ISOCORRMODE_RST             0x0

__INLINE void ble_isocntcntl_pack(uint8_t isosamp, uint8_t isoupd, uint8_t isoclkshiftmode, uint8_t isophaseshiftmode, uint8_t isocorrmode)
{
    BLE_ASSERT_ERR((((uint32_t)isosamp << 31) & ~((uint32_t)0x80000000)) == 0);
    BLE_ASSERT_ERR((((uint32_t)isoupd << 30) & ~((uint32_t)0x40000000)) == 0);
    BLE_ASSERT_ERR((((uint32_t)isoclkshiftmode << 2) & ~((uint32_t)0x00000004)) == 0);
    BLE_ASSERT_ERR((((uint32_t)isophaseshiftmode << 1) & ~((uint32_t)0x00000002)) == 0);
    BLE_ASSERT_ERR((((uint32_t)isocorrmode << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BLE_WR(BLE_ISOCNTCNTL_ADDR,  ((uint32_t)isosamp << 31) | ((uint32_t)isoupd << 30) | ((uint32_t)isoclkshiftmode << 2) | ((uint32_t)isophaseshiftmode << 1) | ((uint32_t)isocorrmode << 0));
}

__INLINE void ble_isocntcntl_unpack(uint8_t* isosamp, uint8_t* isoupd, uint8_t* isoclkshiftmode, uint8_t* isophaseshiftmode, uint8_t* isocorrmode)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCNTCNTL_ADDR);

    *isosamp = (localVal & ((uint32_t)0x80000000)) >> 31;
    *isoupd = (localVal & ((uint32_t)0x40000000)) >> 30;
    *isoclkshiftmode = (localVal & ((uint32_t)0x00000004)) >> 2;
    *isophaseshiftmode = (localVal & ((uint32_t)0x00000002)) >> 1;
    *isocorrmode = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ble_isocntcntl_isosamp_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCNTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ble_isocntcntl_isosamp_setf(uint8_t isosamp)
{
    BLE_ASSERT_ERR((((uint32_t)isosamp << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BLE_WR(BLE_ISOCNTCNTL_ADDR, (REG_BLE_RD(BLE_ISOCNTCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)isosamp << 31));
}

__INLINE uint8_t ble_isocntcntl_iso_upd_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCNTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void ble_isocntcntl_iso_upd_setf(uint8_t isoupd)
{
    BLE_ASSERT_ERR((((uint32_t)isoupd << 30) & ~((uint32_t)0x40000000)) == 0);
    REG_BLE_WR(BLE_ISOCNTCNTL_ADDR, (REG_BLE_RD(BLE_ISOCNTCNTL_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)isoupd << 30));
}

__INLINE uint8_t ble_isocntcntl_iso_clkshift_mode_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCNTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ble_isocntcntl_iso_clkshift_mode_setf(uint8_t isoclkshiftmode)
{
    BLE_ASSERT_ERR((((uint32_t)isoclkshiftmode << 2) & ~((uint32_t)0x00000004)) == 0);
    REG_BLE_WR(BLE_ISOCNTCNTL_ADDR, (REG_BLE_RD(BLE_ISOCNTCNTL_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)isoclkshiftmode << 2));
}

__INLINE uint8_t ble_isocntcntl_iso_phase_shift_mode_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCNTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ble_isocntcntl_iso_phase_shift_mode_setf(uint8_t isophaseshiftmode)
{
    BLE_ASSERT_ERR((((uint32_t)isophaseshiftmode << 1) & ~((uint32_t)0x00000002)) == 0);
    REG_BLE_WR(BLE_ISOCNTCNTL_ADDR, (REG_BLE_RD(BLE_ISOCNTCNTL_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)isophaseshiftmode << 1));
}

__INLINE uint8_t ble_isocntcntl_isocorrmode_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCNTCNTL_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ble_isocntcntl_isocorrmode_setf(uint8_t isocorrmode)
{
    BLE_ASSERT_ERR((((uint32_t)isocorrmode << 0) & ~((uint32_t)0x00000001)) == 0);
    REG_BLE_WR(BLE_ISOCNTCNTL_ADDR, (REG_BLE_RD(BLE_ISOCNTCNTL_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)isocorrmode << 0));
}

/**
 * @brief ISOCNTSAMP register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           ISOCNTSAMP   0x0
 * </pre>
 */
#define BLE_ISOCNTSAMP_ADDR   0x009001C4
#define BLE_ISOCNTSAMP_OFFSET 0x000001C4
#define BLE_ISOCNTSAMP_INDEX  0x00000071
#define BLE_ISOCNTSAMP_RESET  0x00000000

__INLINE uint32_t ble_isocntsamp_get(void)
{
    return REG_BLE_RD(BLE_ISOCNTSAMP_ADDR);
}

__INLINE void ble_isocntsamp_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOCNTSAMP_ADDR, value);
}

// field definitions
#define BLE_ISOCNTSAMP_MASK   ((uint32_t)0xFFFFFFFF)
#define BLE_ISOCNTSAMP_LSB    0
#define BLE_ISOCNTSAMP_WIDTH  ((uint32_t)0x00000020)

#define BLE_ISOCNTSAMP_RST    0x0

__INLINE uint32_t ble_isocntsamp_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCNTSAMP_ADDR);
    BLE_ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_isocntsamp_setf(uint32_t isocntsamp)
{
    BLE_ASSERT_ERR((((uint32_t)isocntsamp << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_BLE_WR(BLE_ISOCNTSAMP_ADDR, (uint32_t)isocntsamp << 0);
}

/**
 * @brief ISOCNTCORR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           ISOCNTCORR   0x0
 * </pre>
 */
#define BLE_ISOCNTCORR_ADDR   0x009001C8
#define BLE_ISOCNTCORR_OFFSET 0x000001C8
#define BLE_ISOCNTCORR_INDEX  0x00000072
#define BLE_ISOCNTCORR_RESET  0x00000000

__INLINE uint32_t ble_isocntcorr_get(void)
{
    return REG_BLE_RD(BLE_ISOCNTCORR_ADDR);
}

__INLINE void ble_isocntcorr_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOCNTCORR_ADDR, value);
}

// field definitions
#define BLE_ISOCNTCORR_MASK   ((uint32_t)0xFFFFFFFF)
#define BLE_ISOCNTCORR_LSB    0
#define BLE_ISOCNTCORR_WIDTH  ((uint32_t)0x00000020)

#define BLE_ISOCNTCORR_RST    0x0

__INLINE uint32_t ble_isocntcorr_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOCNTCORR_ADDR);
    BLE_ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_isocntcorr_setf(uint32_t isocntcorr)
{
    BLE_ASSERT_ERR((((uint32_t)isocntcorr << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_BLE_WR(BLE_ISOCNTCORR_ADDR, (uint32_t)isocntcorr << 0);
}

/**
 * @brief ISOINTCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00            ISOINTMSK   0x0
 * </pre>
 */
#define BLE_ISOINTCNTL_ADDR   0x009001D0
#define BLE_ISOINTCNTL_OFFSET 0x000001D0
#define BLE_ISOINTCNTL_INDEX  0x00000074
#define BLE_ISOINTCNTL_RESET  0x00000000

__INLINE uint32_t ble_isointcntl_get(void)
{
    return REG_BLE_RD(BLE_ISOINTCNTL_ADDR);
}

__INLINE void ble_isointcntl_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOINTCNTL_ADDR, value);
}

// field definitions
#define BLE_ISOINTMSK_MASK   ((uint32_t)0x000000FF)
#define BLE_ISOINTMSK_LSB    0
#define BLE_ISOINTMSK_WIDTH  ((uint32_t)0x00000008)

#define BLE_ISOINTMSK_RST    0x0

__INLINE uint8_t ble_isointcntl_isointmsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOINTCNTL_ADDR);
    BLE_ASSERT_ERR((localVal & ~((uint32_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_isointcntl_isointmsk_setf(uint8_t isointmsk)
{
    BLE_ASSERT_ERR((((uint32_t)isointmsk << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_ISOINTCNTL_ADDR, (uint32_t)isointmsk << 0);
}

/**
 * @brief ISOINTSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00           ISOINTSTAT   0x0
 * </pre>
 */
#define BLE_ISOINTSTAT_ADDR   0x009001D4
#define BLE_ISOINTSTAT_OFFSET 0x000001D4
#define BLE_ISOINTSTAT_INDEX  0x00000075
#define BLE_ISOINTSTAT_RESET  0x00000000

__INLINE uint32_t ble_isointstat_get(void)
{
    return REG_BLE_RD(BLE_ISOINTSTAT_ADDR);
}

// field definitions
#define BLE_ISOINTSTAT_MASK   ((uint32_t)0x000000FF)
#define BLE_ISOINTSTAT_LSB    0
#define BLE_ISOINTSTAT_WIDTH  ((uint32_t)0x00000008)

#define BLE_ISOINTSTAT_RST    0x0

__INLINE uint8_t ble_isointstat_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOINTSTAT_ADDR);
    BLE_ASSERT_ERR((localVal & ~((uint32_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief ISOINTACK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00            ISOINTACK   0x0
 * </pre>
 */
#define BLE_ISOINTACK_ADDR   0x009001D8
#define BLE_ISOINTACK_OFFSET 0x000001D8
#define BLE_ISOINTACK_INDEX  0x00000076
#define BLE_ISOINTACK_RESET  0x00000000

__INLINE uint32_t ble_isointack_get(void)
{
    return REG_BLE_RD(BLE_ISOINTACK_ADDR);
}

__INLINE void ble_isointack_clear(uint32_t value)
{
    REG_BLE_WR(BLE_ISOINTACK_ADDR, value);
}

// field definitions
#define BLE_ISOINTACK_MASK   ((uint32_t)0x000000FF)
#define BLE_ISOINTACK_LSB    0
#define BLE_ISOINTACK_WIDTH  ((uint32_t)0x00000008)

#define BLE_ISOINTACK_RST    0x0

__INLINE uint8_t ble_isointack_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOINTACK_ADDR);
    BLE_ASSERT_ERR((localVal & ~((uint32_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_isointack_clearf(uint8_t isointack)
{
    BLE_ASSERT_ERR((((uint32_t)isointack << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_ISOINTACK_ADDR, (uint32_t)isointack << 0);
}

/**
 * @brief ISOGPIOCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31           ISOGPIOBEH   0
 *  07:00           ISOGPIOMSK   0x0
 * </pre>
 */
#define BLE_ISOGPIOCNTL_ADDR   0x009001E0
#define BLE_ISOGPIOCNTL_OFFSET 0x000001E0
#define BLE_ISOGPIOCNTL_INDEX  0x00000078
#define BLE_ISOGPIOCNTL_RESET  0x00000000

__INLINE uint32_t ble_isogpiocntl_get(void)
{
    return REG_BLE_RD(BLE_ISOGPIOCNTL_ADDR);
}

__INLINE void ble_isogpiocntl_set(uint32_t value)
{
    REG_BLE_WR(BLE_ISOGPIOCNTL_ADDR, value);
}

// field definitions
#define BLE_ISOGPIOBEH_BIT    ((uint32_t)0x80000000)
#define BLE_ISOGPIOBEH_POS    31
#define BLE_ISOGPIOMSK_MASK   ((uint32_t)0x000000FF)
#define BLE_ISOGPIOMSK_LSB    0
#define BLE_ISOGPIOMSK_WIDTH  ((uint32_t)0x00000008)

#define BLE_ISOGPIOBEH_RST    0x0
#define BLE_ISOGPIOMSK_RST    0x0

__INLINE void ble_isogpiocntl_pack(uint8_t isogpiobeh, uint8_t isogpiomsk)
{
    BLE_ASSERT_ERR((((uint32_t)isogpiobeh << 31) & ~((uint32_t)0x80000000)) == 0);
    BLE_ASSERT_ERR((((uint32_t)isogpiomsk << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_ISOGPIOCNTL_ADDR,  ((uint32_t)isogpiobeh << 31) | ((uint32_t)isogpiomsk << 0));
}

__INLINE void ble_isogpiocntl_unpack(uint8_t* isogpiobeh, uint8_t* isogpiomsk)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOGPIOCNTL_ADDR);

    *isogpiobeh = (localVal & ((uint32_t)0x80000000)) >> 31;
    *isogpiomsk = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ble_isogpiocntl_isogpiobeh_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOGPIOCNTL_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void ble_isogpiocntl_isogpiobeh_setf(uint8_t isogpiobeh)
{
    BLE_ASSERT_ERR((((uint32_t)isogpiobeh << 31) & ~((uint32_t)0x80000000)) == 0);
    REG_BLE_WR(BLE_ISOGPIOCNTL_ADDR, (REG_BLE_RD(BLE_ISOGPIOCNTL_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)isogpiobeh << 31));
}

__INLINE uint8_t ble_isogpiocntl_isogpiomsk_getf(void)
{
    uint32_t localVal = REG_BLE_RD(BLE_ISOGPIOCNTL_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void ble_isogpiocntl_isogpiomsk_setf(uint8_t isogpiomsk)
{
    BLE_ASSERT_ERR((((uint32_t)isogpiomsk << 0) & ~((uint32_t)0x000000FF)) == 0);
    REG_BLE_WR(BLE_ISOGPIOCNTL_ADDR, (REG_BLE_RD(BLE_ISOGPIOCNTL_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)isogpiomsk << 0));
}

/**
 * @brief ISOTIMERTGT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00          ISOTIMERTGT   0x0
 * </pre>
 */
#define BLE_ISOTIMERTGT_ADDR   0x009001F0
#define BLE_ISOTIMERTGT_OFFSET 0x000001F0
#define BLE_ISOTIMERTGT_INDEX  0x0000007C
#define BLE_ISOTIMERTGT_RESET  0x00000000
#define BLE_ISOTIMERTGT_COUNT  8

__INLINE uint32_t ble_isotimertgt_get(int reg_idx)
{
    BLE_ASSERT_ERR(reg_idx <= 7);
    return REG_BLE_RD(BLE_ISOTIMERTGT_ADDR + reg_idx * 4);
}

__INLINE void ble_isotimertgt_set(int reg_idx, uint32_t value)
{
    BLE_ASSERT_ERR(reg_idx <= 7);
    REG_BLE_WR(BLE_ISOTIMERTGT_ADDR + reg_idx * 4, value);
}

// field definitions
#define BLE_ISOTIMERTGT_MASK   ((uint32_t)0xFFFFFFFF)
#define BLE_ISOTIMERTGT_LSB    0
#define BLE_ISOTIMERTGT_WIDTH  ((uint32_t)0x00000020)

#define BLE_ISOTIMERTGT_RST    0x0

__INLINE uint32_t ble_isotimertgt_getf(int reg_idx)
{
    BLE_ASSERT_ERR(reg_idx <= 7);
    uint32_t localVal = REG_BLE_RD(BLE_ISOTIMERTGT_ADDR + reg_idx * 4);
    BLE_ASSERT_ERR((localVal & ~((uint32_t)0xFFFFFFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void ble_isotimertgt_setf(int reg_idx, uint32_t isotimertgt)
{
    BLE_ASSERT_ERR(reg_idx <= 7);
    BLE_ASSERT_ERR((((uint32_t)isotimertgt << 0) & ~((uint32_t)0xFFFFFFFF)) == 0);
    REG_BLE_WR(BLE_ISOTIMERTGT_ADDR + reg_idx * 4, (uint32_t)isotimertgt << 0);
}


#endif // _REG_IPCORE_BTS_H_

