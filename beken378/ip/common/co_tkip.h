#ifndef __TKIP_H__
#define __TKIP_H__

//#define TKIP_DEBUG
#if defined(TKIP_DEBUG)
#define TKIP_TX_PRINT os_printf
#define TKIP_RX_PRINT os_printf
#else
#define TKIP_TX_PRINT(...)
#define TKIP_RX_PRINT(...)
#endif

#include "hal_desc.h"

#define le_to_host16(n) (n)

static inline void WPA_PUT_LE16(uint8_t *a, uint16_t val)
{
	a[1] = val >> 8;
	a[0] = val & 0xff;
}

static inline uint32_t WPA_GET_LE32(const uint8_t *a)
{
	return ((uint32_t) a[3] << 24) | (a[2] << 16) | (a[1] << 8) | a[0];
}

static inline void WPA_PUT_LE32(uint8_t *a, uint32_t val)
{
	a[3] = (val >> 24) & 0xff;
	a[2] = (val >> 16) & 0xff;
	a[1] = (val >> 8) & 0xff;
	a[0] = val & 0xff;
}

static inline uint32_t WPA_GET_BE32(const uint8_t *a)
{
	return ((uint32_t) a[0] << 24) | (a[1] << 16) | (a[2] << 8) | a[3];
}

static inline uint16_t WPA_GET_BE16(const uint8_t *a)
{
	return (a[0] << 8) | a[1];
}

void tkip_mixing_phase1(uint16_t *TTAK, const uint8_t *TK, const uint8_t *TA, uint32_t IV32);
void tkip_mixing_phase2(uint8_t *WEPSeed, const uint8_t *TK, const uint16_t *TTAK, uint16_t IV16);
void wep_crypt(uint8_t *key, struct rx_pbd *buf, int plen, bool wep40);
uint32_t tkip_crc32(const uint8_t *frame, int frame_len);
void wep_crypt_bug(uint8_t *key, struct rx_pbd *buf, int plen, bool wep40);
void wep_crypt_bugfixed(uint8_t *key, struct rx_pbd *buf, int plen, bool wep40);

#endif

