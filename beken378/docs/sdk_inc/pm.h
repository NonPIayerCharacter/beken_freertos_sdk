#ifndef _PM__H_
#define _PM__H_

/** @brief  Request deep sleep,and wakeup by gpio/rtc/usb.
 *
 *  @param  deep_param param of deeps leep
 *          @arg wake_up_way wakeup mode gpio/rtc/usb
 *          @arg gpio_index_map:The gpio bitmap which set 1 enable wakeup deep sleep.
 *              gpio_index_map is hex and every bits is map to gpio0-gpio31.
 *          @arg gpio_edge_map:The gpio edge bitmap for wakeup gpios,
 *              gpio_edge_map is hex and every bits is map to gpio0-gpio31.
 *              0:rising,1:falling.
 *          @arg gpio_stay_lo_map:The gpio bitmap which need stay ,not change in deep sleep.
 *               gpio_index_lo_map is hex and every bits is map to gpio0-gpio31.
 *          @arg gpio_last_index_map:The gpio bitmap which set 1 enable wakeup deep sleep.
 *              gpio_index_map is hex and every bits is map to gpio32-gpio39.
 *          @arg gpio_last_edge_map:The gpio edge bitmap for wakeup gpios,
 *              gpio_edge_map is hex and every bits is map to gpio32-gpio39.
 *              0:rising,1:falling.
 *          @arg gpio_stay_hi_map:The gpio bitmap which need stay ,not change in deep sleep.
 *           gpio_index_lo_map is hex and every bits is map to gpio32-gpio39.
 *          @arg sleep_time:the time secound when use PS_DEEP_WAKEUP_RTC wakeup.
 *          @arg lpo_32k_src:the RTC wakeup source.LPO_SELECT_ROSC or LPO_SELECT_32K_XTAL.
 *          @arg sleep_mode  MCU_LOW_VOLTAGE_SLEEP or other
 * @return
 *    - void
 */
void bk_enter_deep_sleep_mode ( PS_DEEP_CTRL_PARAM *deep_param );


/** @brief  Request low vol sleep,and wakeup by gpio/rtc/usb.
 *
 *  @param  lowvol_param param of low lov leep
 *          @arg wake_up_way wakeup mode gpio/rtc/usb
 *          @arg gpio_index_map:The gpio bitmap which set 1 enable wakeup deep sleep.
 *              gpio_index_map is hex and every bits is map to gpio0-gpio31.
 *          @arg gpio_edge_map:The gpio edge bitmap for wakeup gpios,
 *              gpio_edge_map is hex and every bits is map to gpio0-gpio31.
 *              0:rising,1:falling.
 *          @arg gpio_stay_lo_map:The gpio bitmap which need stay ,not change in deep sleep.
 *               gpio_index_lo_map is hex and every bits is map to gpio0-gpio31.
 *          @arg gpio_last_index_map:The gpio bitmap which set 1 enable wakeup deep sleep.
 *              gpio_index_map is hex and every bits is map to gpio32-gpio39.
 *          @arg gpio_last_edge_map:The gpio edge bitmap for wakeup gpios,
 *              gpio_edge_map is hex and every bits is map to gpio32-gpio39.
 *              0:rising,1:falling.
 *          @arg gpio_stay_hi_map:The gpio bitmap which need stay ,not change in deep sleep.
 *           gpio_index_lo_map is hex and every bits is map to gpio32-gpio39.
 *          @arg sleep_time:the time secound when use PS_DEEP_WAKEUP_RTC wakeup.
 *          @arg lpo_32k_src:the RTC wakeup source.LPO_SELECT_ROSC or LPO_SELECT_32K_XTAL.
 *          @arg sleep_mode  MCU_LOW_VOLTAGE_SLEEP or other
 *  @return
 *      - kNoErr  : success
 *      - othesr  : failed
 */
UINT32 bk_wlan_instant_lowvol_sleep( PS_DEEP_CTRL_PARAM *lowvol_param );


/** @brief  Enable dtim power save,close rf,and wakeup by ieee dtim dynamical
 *  @param void
 *  @return
 *      - kNoErr  : success
 *      - others  : failed
 */
int bk_wlan_dtim_rf_ps_mode_enable(void);


/** @brief  Request exit power save by ieee dtim
 *  @param
 *      - void
 *
 *  @return
 *      - kNoErr  : success.
 *      - othesr   : failed
 */
int bk_wlan_dtim_rf_ps_mode_disable(void);


/** @brief  set wifi listen interval in dtim sleep mode
 *  @param
 *      - listen_int  dtim listen interval
 */
void power_save_set_listen_int(UINT16 listen_int);


/** @brief  set wifi hw_ tim interval in dtim sleep mode. 
 *
 *  @attention 
 *          Set the cnt_limit for hw_tim, once hw_tim_cnt is over cnt_limit, hw_tim will be disabled until next beacon received (hw_tim_cnt reset to 0).
 *  @param 
 *     - cnt : The cnt limit you want to pass, pass 0 if you want to disable hw_tim.
 *
 *  @return
 *     - void
 */
void power_save_set_hw_tim_cnt_limit(uint32_t cnt);


/** @brief  Enable mcu power save,close mcu ,and wakeup by irq
 *
*  @param
 *      - void
 *
 *  @return
 *      - kNoErr  : success.
 *      - others   : failed
 */
int bk_wlan_mcu_ps_mode_enable(void);


/** @brief  Disenable mcu power save
 *
*  @param
 *      - void
 *
 *  @return
 *      - kNoErr  : success.
 *      - others   : failed
 */
int bk_wlan_mcu_ps_mode_disable(void);


/** @brief set GPIO wakeup in wifi keepalive
 *
 *  @param gpio_index: Pin x used to wakeup low voltage sleep
 *  @param gpio_type:
 *          @arg  0 - low level
 *          @arg  1 - high level
 *          @arg  2 - posedge
 *          @arg  3 - negedge
 *
 *  @return
 *      - true  : success.
 *      - false : failed
 */
bool sctrl_set_gpio_wakeup_index(UINT8 gpio_index, WAKEUP_GPIO_TYPE gpio_type);

#endif
