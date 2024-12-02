#ifndef _WIFI_SENSOR_H_
#define _WIFI_SENSOR_H_

#define BK_WSD_OK               0
#define BK_WSD_FAIL            -1


/** @brief  bk wifi senser detecting movement callback.
 *
 *  @return   1        : something move.
 *  @return   0        : no movement in there
 */
typedef void (*bk_wsd_cb)(int status);

int bk_wifi_detect_movement_start(bk_wsd_cb callback);
void bk_wifi_detect_movement_stop(void);

#endif

