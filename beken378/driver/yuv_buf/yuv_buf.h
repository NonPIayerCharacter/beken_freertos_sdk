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

#ifndef __YUV_BUF_H__
#define __YUV_BUF_H__

#if (CFG_SOC_NAME == SOC_BK7252N)
#define YUV_BUF_BASE                        (0x00A04000)

#define YUV_BUF_DEV_ID_ADDR                 (YUV_BUF_BASE + 0*4)
#define YUV_BUF_VER_ID_ADDR                 (YUV_BUF_BASE + 1*4)

#define YUV_BUF_CLK_CTL_ADDR                (YUV_BUF_BASE + 2*4)
#define YUV_BUF_SOFT_RESET                  (1 << 0)
#define YUV_BUF_CLK_GATE                    (1 << 1)

#define YUV_BUF_REG0X4_ADDR                 (YUV_BUF_BASE + 4*4)
#define YUV_BUF_YUV_MODE                    (1 << 0)
#define YUV_BUF_JPEG_WORD_REVERSE           (1 << 1)
#define YUV_BUF_YUV_FMT_SEL_POSI            (2)
#define YUV_BUF_YUV_FMT_SEL_MASK            (0x3)
#define YUV_BUF_VCK_EDGE                    (1 << 5)
#define YUV_BUF_HSYNC_REV                   (1 << 6)
#define YUV_BUF_VSYNC_REV                   (1 << 7)
#define YUV_BUF_PARTIAL_DISPLAY_ENA         (1 << 8)
#define YUV_BUF_SUNC_EDGE_DECT_ENA          (1 << 9)
#define YUV_BUF_MCLK_DIV_POSI               (10)
#define YUV_BUF_MCLK_DIV_MASK               (0x3)
#define YUV_BUF_H264_MODE                   (1 << 12)
#define YUV_BUF_BPS_CIS                     (1 << 13)
#define YUV_BUF_MCLK_HD                     (1 << 15)
#define YUV_BUF_SEN_MEM_CLR                 (1 << 16)
#define YUV_BUF_SEN_MEM_ENA_HD              (1 << 17)
#define YUV_BUF_BUS_DAT_BYTE_REVE           (1 << 18)
#define YUV_BUF_SOI_HSYNC                   (1 << 19)

#define YUV_BUF_REG0X5_ADDR                 (YUV_BUF_BASE + 5*4)
#define YUV_BUF_X_PIXEL_POSI                (0)
#define YUV_BUF_X_PIXEL_MASK                (0xFF)
#define YUV_BUF_Y_PIXEL_POSI                (8)
#define YUV_BUF_Y_PIXEL_MASK                (0xFF)
#define YUV_BUF_FRAME_BLK_POSI              (16)
#define YUV_BUF_FRAME_BLK_MASK              (0xFFFF)

#define YUV_BUF_REG0X6_ADDR                 (YUV_BUF_BASE + 6*4)
#define YUV_BUF_X_PARTIAL_OFFSET_I_POSI     (0)
#define YUV_BUF_X_PARTIAL_OFFSET_I_MASK     (0x7FF)
#define YUV_BUF_X_PARTIAL_OFFSET_R_POSI     (16)
#define YUV_BUF_X_PARTIAL_OFFSET_R_MASK     (0x7FF)

#define YUV_BUF_REG0X7_ADDR                 (YUV_BUF_BASE + 7*4)
#define YUV_BUF_Y_PARTIAL_OFFSET_I_POSI     (0)
#define YUV_BUF_Y_PARTIAL_OFFSET_I_MASK     (0x7FF)
#define YUV_BUF_Y_PARTIAL_OFFSET_R_POSI     (16)
#define YUV_BUF_Y_PARTIAL_OFFSET_R_MASK     (0x7FF)

#define YUV_BUF_EM_BASE_ADDR                (YUV_BUF_BASE + 8*4)

#define YUV_BUF_REG0X9_ADDR                 (YUV_BUF_BASE + 9*4)
#define YUV_BUF_YSYNC_NEGE_INT_ENA          (1 << 0)
#define YUV_BUF_YUV_ARV_INT_ENA             (1 << 1)
#define YUV_BUF_SM0_WR_INT_ENA              (1 << 2)
#define YUV_BUF_SM1_WR_INT_ENA              (1 << 3)
#define YUV_BUF_SEN_FULL_INT_ENA            (1 << 4)
#define YUV_BUF_ENC_LINE_INT_ENA            (1 << 5)
#define YUV_BUF_SEN_RESL_INT_ENA            (1 << 6)
#define YUV_BUF_H264_ERR_INT_ENA            (1 << 7)
#define YUV_BUF_ENC_SLOW_INT_ENA            (1 << 8)
#define YUV_INT_MASK                        (0x1FF)

#define YUV_BUF_REG0XA_ADDR                 (YUV_BUF_BASE + 10*4)
#define YUV_BUF_YSYNC_NEGE_INT              (1 << 0)
#define YUV_BUF_YUV_ARV_INT                 (1 << 1)
#define YUV_BUF_SM0_WR_INT                  (1 << 2)
#define YUV_BUF_SM1_WR_INT                  (1 << 3)
#define YUV_BUF_SEN_FULL_INT                (1 << 4)
#define YUV_BUF_ENC_LINE_INT                (1 << 5)
#define YUV_BUF_SEN_RESL_INT                (1 << 6)
#define YUV_BUF_H264_ERR_INT                (1 << 7)
#define YUV_BUF_ENC_SLOW_INT                (1 << 8)

#define YUV_BUF_REG0XB_ADDR                 (YUV_BUF_BASE + 11*4)
#define YUV_BUF_INT_VSY_NEGE_MASK           (0x3)
#define YUV_BUF_INT_RESL_ERR_MASK           (1 << 2)
#define YUV_BUF_INT_SENS_FULL_MASK          (1 << 3)
#define YUV_BUF_INT_ENC_SLOW_MASK           (1 << 4)
#define YUV_BUF_INT_H264_ERR_MASK           (1 << 5)

#define YUV_BUF_EMR_BASE_ADDR               (YUV_BUF_BASE + 12*4)

#define YUV_BUF_REG0XD_ADDR                 (YUV_BUF_BASE + 13*4)
#define YUV_BUF_BPS_PINGPONG                (1 << 0)
#define YUV_BUF_X_PIXEL_RESIZE_POSI         (1)
#define YUV_BUF_X_PIXEL_RESIZE_MASK         (0xFF)
#define YUV_BUF_Y_PIXEL_RESIZE_POSI         (9)
#define YUV_BUF_Y_PIXEL_RESIZE_MASK         (0xFF)

#define YUV_BUF_RENC_START_ADDR             (YUV_BUF_BASE + 15*4)

#endif

#endif // __YUV_BUF_H__

