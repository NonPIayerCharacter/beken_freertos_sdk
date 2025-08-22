// Copyright 2015-2024 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __YUV_BUF_PUB_H__
#define __YUV_BUF_PUB_H__

#include "rtos_pub.h"
#include "media_types.h"

#define YUV_BUF_FAILURE                 (1)
#define YUV_BUF_SUCCESS                 (0)

#define YUV_BUF_DEV_NAME                "yuv_buf"

enum
{
    YUV_BUF_CTRL_INIT,
    YUV_BUF_SOFT_MODE,
    YUV_BUF_RENC_START,
    YUV_BUF_EMADDR_SET,
    YUV_BUF_EMADDR_GET,
    YUV_BUF_CTRL_DEINIT,
    YUV_BUF_CMD_RESET,
};

typedef enum
{
    YUV_BUF_INT_VSYNC_NEGE,
    YUV_BUF_INT_YUV_ARV,
    YUV_BUF_INT_SM0_WR,
    YUV_BUF_INT_SM1_WR,
    YUV_BUF_INT_SEN_FULL,
    YUV_BUF_INT_ENC_LINE,
    YUV_BUF_INT_SEN_RESL,
    YUV_BUF_INT_H264_ERR,
    YUV_BUF_INT_ENC_SLOW
} YUV_BUF_INT_STATUS;

typedef enum
{
    UNKNOW_MODE = 0,
    YUV_MODE,
    GRAY_MODE,
    JPEG_MODE,
    H264_MODE,
    H265_MODE,
    JPEG_YUV_MODE,
    H264_YUV_MODE,
} yuv_mode_t;

typedef enum {
    YUV_FORMAT_YUYV = 0,
    YUV_FORMAT_UYVY,
    YUV_FORMAT_YYUV,
    YUV_FORMAT_UVYY,
} yuv_format_t;

typedef enum {
    YUV_MCLK_DIV_4 = 0,
    YUV_MCLK_DIV_6 = 1,
    YUV_MCLK_DIV_2 = 2,
    YUV_MCLK_DIV_3 = 3,
} mclk_div_t;

typedef struct {
    yuv_mode_t work_mode;
    yuv_format_t yuv_format;
    mclk_div_t mclk_div;    /**< div yuv_buf module clock for MCLK and PCLK */
    uint32_t x_pixel;    /**< sensor data's resolution for width */
    uint32_t y_pixel;    /**< sensor data's resolution for height */
    uint8_t *base_addr;  /**< used for save source yuv data base addr */

    void (*sensor_full_handler)(void);
    void (*resol_err_handler)(void);
    void (*enc_slow_handler)(void);
    void (*vsync_handler)(void);
} yuv_buf_config_t;

#define YUV_ENCODE_UINT_LINES				(16)
#define YUV_PIXEL_SIZE					(2)

void yuv_buf_init(void);
void yuv_buf_exit(void);

#endif

