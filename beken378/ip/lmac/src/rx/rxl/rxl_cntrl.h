/**
 ****************************************************************************************
 * @file rxl_cntrl.h
 *
 * @brief API declarations for the Rx path.
 *
 * Copyright (C) RivieraWaves 2011-2016
 ****************************************************************************************
 */

#ifndef _RXL_CNTRL_H_
#define _RXL_CNTRL_H_

/**
 ****************************************************************************************
 * @defgroup RX_CNTRL RX_CNTRL
 * @ingroup RX
 * @brief Initialization and control of LMAC SW Rx path.
 * @{
 * It is responsible to initialize the data structures required for the RX path. These include:
 * -Tuple initialization
 * -TID Info table initialization
 * -LMAC SW Rx descriptor initialization
 * -Defragmentation table initialization
 *
 * It initializes the Rx context that includes the lists maintained at SW to host the received
 * frames and the HW descriptor details.
 * It configures the MAC HW registers for the Rx operation.
 *
 * Controls the operations to be performed on the received frame.
 *
 * It is responsible to validate the frame and control the sequence of operations to be
 * performed in the receive and the deferred context.
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "co_int.h"
#include "co_list.h"
#include "co_ring.h"
#include "dma.h"
#include "vif_mgmt.h"

//#define RXL_CNTRL_DEBUG

#ifdef RXL_CNTRL_DEBUG
#define RXL_CNTRL_PRT       os_printf
#define RXL_CNTRL_WPRT      warning_prf
#else
#define RXL_CNTRL_PRT       os_null_printf
#define RXL_CNTRL_WPRT      warning_prf
#endif

/*
 * MACROS
 ****************************************************************************************
 */
/// Pattern indicating to the host that the buffer is ready for him
#define DMA_HD_RXPATTERN                 0xAAAAAA00

/// Reception timeout
#define RX_TIMEOUT                       200  ///< 200us

/// Frame threshold above which we assert an interrupt to the host
#define RX_FRAME_THD                     (1)

/// Threshold of frame upload preparation before which we handle the DMA interrupts
#if CFG_IPERF_TEST_ACCEL
#define RX_FRAME_PREP_THD                16
#else
#define RX_FRAME_PREP_THD                4
#endif

/// Set bit in frame_info value (struct rxl_cntrl_rx_status)
#define RXL_CNTRL_FRAME_INFO_SET(bit)                           \
    (rxl_cntrl_env.rx_status.frame_info |= RXL_CNTRL_ ## bit)

/// Get if bit set to 1 in frame_info value (struct rxl_cntrl_rx_status)
#define RXL_CNTRL_FRAME_INFO_GET(bit)                           \
    (rxl_cntrl_env.rx_status.frame_info & RXL_CNTRL_ ## bit)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/*
 * TYPES DECLARATION
 ****************************************************************************************
 */
// forward declarations
struct rx_swdesc;
struct rx_dmadesc;
struct rx_hd;
struct rx_pbd;

/// RX control environment declaration.
struct rxl_cntrl_env_tag
{
    /// Frames waiting for a available host buffer
    struct co_list ready;
    /// First DMA descriptor the SW has to process.
    struct rx_dmadesc *first;
    /// The last DMA descriptor in the HW list.
    struct rx_dmadesc *last;
    /// First DMA descriptor the SW has to process.
    struct rx_dmadesc *free;
    /// The bridge DMA count
    uint16_t bridgedmacnt;
    /// The number of packets pending for signalling to host
    uint8_t packet_cnt;
};

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */
/// RXL module environment variable
extern struct rxl_cntrl_env_tag rxl_cntrl_env;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief This function initializes the RX path.
 ****************************************************************************************
 */
void rxl_init(void);

/**
 ****************************************************************************************
 * @brief RX path reset function.
 * This function is part of the recovery mechanism invoked upon an error detection in the
 * LMAC. It flushes all the packets currently in the RX path and exits when all of them
 * have been indicated to the driver
 ****************************************************************************************
 */
void rxl_reset(void);

/**
 ****************************************************************************************
 * @brief RX path background packet handler.
 * This function is a kernel event handler triggered upon packet reception. It performs
 * the platform DMA programming for the upload of the packet to the host memory.
 *
 * @param[in] dummy Parameter not used but required to follow the kernel event callback
 * format
 ****************************************************************************************
 */
void rxl_cntrl_evt(int dummy);

/**
 ****************************************************************************************
 * @brief RX TIMER interrupt handler.
 *
 * This function clears the source of the interrupt and invokes the RX receive path
 * operations by setting an event that is scheduled by the kernel.
 ****************************************************************************************
 */
void rxl_timer_int_handler(void);

/**
 ****************************************************************************************
 * @brief Handling of the SW RX timeout.
 *
 * This function is called when the SW RX timeout expires, that indicates that a pending
 * A-MPDU reception is over. In this handler we warn the host about the packets that are
 * available.
 *
 ****************************************************************************************
 */
void rxl_timeout_int_handler(void);

/**
 ****************************************************************************************
 * @brief RX IPC DMA interrupt handler.
 *
 * This function is called upon interrupt from the IPC DMA machine.  It sets an event
 * to schedule the processing of the DMA'd packets and disables the interrupts until they
 * have all been successfully handled in the event handler.
 ****************************************************************************************
 */
void rxl_dma_int_handler(void);

/**
 ****************************************************************************************
 * @brief This function cleans up the complete frame data structures.
 *
 * @param[in] swdesc Pointer to the structure that is holding the frame details.
 ****************************************************************************************
 */
void rxl_frame_release(struct rx_swdesc* swdesc);

/**
 ****************************************************************************************
 * @brief Free the HW descriptors attached to the SW descriptor passed as parameter.
 *
 *This function releases both the RHD and RBD and requests chaining them to the HW.
 *
 * @param[in] swdesc   Pointer to the rx SW descriptor for which the MPDU descriptors are freed
 ****************************************************************************************
 */
void rxl_mpdu_free(struct rx_swdesc *swdesc);

/**
 ****************************************************************************************
 * @brief This function returns the pointer to the first RX Header and Buffer
 * descriptors chained to the MAC HW.
 * It is used for error logging when an issue is detected in the LMAC.
 *
 * @param[out] rhd Pointer to the first RX Header descriptor chained to the MAC HW
 * @param[out] rbd Pointer to the first RX Buffer descriptor chained to the MAC HW
 *
 ****************************************************************************************
 */
void rxl_current_desc_get(struct rx_hd **rhd, struct rx_pbd **rbd);

/**
 ****************************************************************************************
 * @brief This function programs the DMA transfer of the received payload to the host
 * memory.
 *
 * @param[in] swdesc Pointer to SW descriptor attached to the received payload
 *
 ****************************************************************************************
 */
void rxl_mpdu_transfer(struct rx_swdesc *swdesc);

int8_t rxl_get_beacon_rssi(void);


#ifdef CONFIG_AOS_MESH
void rxl_mpdu_transfer_mesh(struct rx_swdesc *swdesc);
#endif

#if (NX_P2P)
void rxl_packet_trace(uint8_t *frame, struct vif_info_tag *vif);
#endif
#if CFG_AP_MONITOR_COEXIST
uint32_t rxu_cntrl_patch_get_compensation_len(uint8_t *p_frame);
int rxl_data_monitor(struct rx_swdesc *swdesc, uint8_t *payload, uint16_t length);
#endif
#endif // _RXL_CNTRL_H_
// eof

