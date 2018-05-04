/*
The MIT License (MIT)

Copyright (c) 2016 Vasil Kalchev

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once
#include <stdint.h>
#include "I2Chelper.hpp"


#define MPU925x_ADDRESS_AD0_LOW         0x68 // address pin low (GND)
#define MPU925x_ADDRESS_AD0_HIGH        0x69 // address pin high (VCC)

#define MPU9250_ID                      0x71
#define MPU9255_ID                      0x73

// Register addresses
#define MPU925x_RA_SELF_TEST_X_GYRO     0x00
#define MPU925x_RA_SELF_TEST_Y_GYRO     0x01
#define MPU925x_RA_SELF_TEST_Z_GYRO     0x02
#define MPU925x_RA_SELF_TEST_X_ACCEL    0x0D
#define MPU925x_RA_SELF_TEST_Y_ACCEL    0x0E
#define MPU925x_RA_SELF_TEST_Z_ACCEL    0x0F
#define MPU925x_RA_XG_OFFSET_H          0x13
#define MPU925x_RA_XG_OFFSET_L          0x14
#define MPU925x_RA_YG_OFFSET_H          0x15
#define MPU925x_RA_YG_OFFSET_L          0x16
#define MPU925x_RA_ZG_OFFSET_H          0x17
#define MPU925x_RA_ZG_OFFSET_L          0x18
#define MPU925x_RA_SMPLRT_DIV           0x19
#define MPU925x_RA_CONFIG               0x1A
#define MPU925x_RA_GYRO_CONFIG          0x1B
#define MPU925x_RA_ACCEL_CONFIG         0x1C
#define MPU925x_RA_ACCEL_CONFIG_2       0x1D
#define MPU925x_RA_LP_ACCEL_ODR         0x1E
#define MPU925x_RA_WOM_THR              0x1F
#define MPU925x_RA_FIFO_EN              0x23
#define MPU925x_RA_I2C_MST_CTRL         0x24
#define MPU925x_RA_I2C_SLV0_ADDR        0x25
#define MPU925x_RA_I2C_SLV0_REG         0x26
#define MPU925x_RA_I2C_SLV0_CTRL        0x27
#define MPU925x_RA_I2C_SLV1_ADDR        0x28
#define MPU925x_RA_I2C_SLV1_REG         0x29
#define MPU925x_RA_I2C_SLV1_CTRL        0x2A
#define MPU925x_RA_I2C_SLV2_ADDR        0x2B
#define MPU925x_RA_I2C_SLV2_REG         0x2C
#define MPU925x_RA_I2C_SLV2_CTRL        0x2D
#define MPU925x_RA_I2C_SLV3_ADDR        0x2E
#define MPU925x_RA_I2C_SLV3_REG         0x2F
#define MPU925x_RA_I2C_SLV3_CTRL        0x30
#define MPU925x_RA_I2C_SLV4_ADDR        0x31
#define MPU925x_RA_I2C_SLV4_REG         0x32
#define MPU925x_RA_I2C_SLV4_DO          0x33
#define MPU925x_RA_I2C_SLV4_CTRL        0x34
#define MPU925x_RA_I2C_SLV4_DI          0x35
#define MPU925x_RA_I2C_MST_STATUS       0x36
#define MPU925x_RA_INT_PIN_CFG          0x37
#define MPU925x_RA_INT_ENABLE           0x38
#define MPU925x_RA_INT_STATUS           0x3A
#define MPU925x_RA_ACCEL_XOUT_H         0x3B
#define MPU925x_RA_ACCEL_XOUT_L         0x3C
#define MPU925x_RA_ACCEL_YOUT_H         0x3D
#define MPU925x_RA_ACCEL_YOUT_L         0x3E
#define MPU925x_RA_ACCEL_ZOUT_H         0x3F
#define MPU925x_RA_ACCEL_ZOUT_L         0x40
#define MPU925x_RA_TEMP_OUT_H           0x41
#define MPU925x_RA_TEMP_OUT_L           0x42
#define MPU925x_RA_GYRO_XOUT_H          0x43
#define MPU925x_RA_GYRO_XOUT_L          0x44
#define MPU925x_RA_GYRO_YOUT_H          0x45
#define MPU925x_RA_GYRO_YOUT_L          0x46
#define MPU925x_RA_GYRO_ZOUT_H          0x47
#define MPU925x_RA_GYRO_ZOUT_L          0x48
#define MPU925x_RA_EXT_SENS_DATA_00     0x49
#define MPU925x_RA_EXT_SENS_DATA_01     0x4A
#define MPU925x_RA_EXT_SENS_DATA_02     0x4B
#define MPU925x_RA_EXT_SENS_DATA_03     0x4C
#define MPU925x_RA_EXT_SENS_DATA_04     0x4D
#define MPU925x_RA_EXT_SENS_DATA_05     0x4E
#define MPU925x_RA_EXT_SENS_DATA_06     0x4F
#define MPU925x_RA_EXT_SENS_DATA_07     0x50
#define MPU925x_RA_EXT_SENS_DATA_08     0x51
#define MPU925x_RA_EXT_SENS_DATA_09     0x52
#define MPU925x_RA_EXT_SENS_DATA_10     0x53
#define MPU925x_RA_EXT_SENS_DATA_11     0x54
#define MPU925x_RA_EXT_SENS_DATA_12     0x55
#define MPU925x_RA_EXT_SENS_DATA_13     0x56
#define MPU925x_RA_EXT_SENS_DATA_14     0x57
#define MPU925x_RA_EXT_SENS_DATA_15     0x58
#define MPU925x_RA_EXT_SENS_DATA_16     0x59
#define MPU925x_RA_EXT_SENS_DATA_17     0x5A
#define MPU925x_RA_EXT_SENS_DATA_18     0x5B
#define MPU925x_RA_EXT_SENS_DATA_19     0x5C
#define MPU925x_RA_EXT_SENS_DATA_20     0x5D
#define MPU925x_RA_EXT_SENS_DATA_21     0x5E
#define MPU925x_RA_EXT_SENS_DATA_22     0x5F
#define MPU925x_RA_EXT_SENS_DATA_23     0x60
#define MPU925x_RA_I2C_SLV0_DO          0x63
#define MPU925x_RA_I2C_SLV1_DO          0x64
#define MPU925x_RA_I2C_SLV2_DO          0x65
#define MPU925x_RA_I2C_SLV3_DO          0x66
#define MPU925x_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU925x_RA_SIGNAL_PATH_RESET    0x68
#define MPU925x_RA_MOT_DETECT_CTRL      0x69
#define MPU925x_RA_USER_CTRL            0x6A
#define MPU925x_RA_PWR_MGMT_1           0x6B // non-zero reset
#define MPU925x_RA_PWR_MGMT_2           0x6C
#define MPU925x_RA_FIFO_COUNT_H         0x72
#define MPU925x_RA_FIFO_COUNT_L         0x73
#define MPU925x_RA_FIFO_R_W             0x74
#define MPU925x_RA_WHO_AM_I             0x75 // reset 0x73
#define MPU925x_RA_XA_OFFSET_H          0x77
#define MPU925x_RA_XA_OFFSET_L          0x78
#define MPU925x_RA_YA_OFFSET_H          0x7A
#define MPU925x_RA_YA_OFFSET_L          0x7B
#define MPU925x_RA_ZA_OFFSET_H          0x7D
#define MPU925x_RA_ZA_OFFSET_L          0x7E

// SELF_TEST_n_GYRO
#define MPU925x_XG_ST_DATA_BIT        0x07
#define MPU925x_XG_ST_DATA_LENGTH        8
#define MPU925x_YG_ST_DATA_BIT        0x07
#define MPU925x_YG_ST_DATA_LENGTH        8
#define MPU925x_ZG_ST_DATA_BIT        0x07
#define MPU925x_ZG_ST_DATA_LENGTH        8

// SELF_TEST_n_ACCEL
#define MPU925x_XA_ST_DATA_BIT        0x07
#define MPU925x_XA_ST_DATA_LENGTH        8
#define MPU925x_YA_ST_DATA_BIT        0x07
#define MPU925x_YA_ST_DATA_LENGTH        8
#define MPU925x_ZA_ST_DATA_BIT        0x07
#define MPU925x_ZA_ST_DATA_LENGTH        8

// nG_OFFSET
#define MPU925x_X_OFFS_USR_BIT        0x07
#define MPU925x_X_OFFS_USR_LENGTH        8
#define MPU925x_Y_OFFS_USR_BIT        0x07
#define MPU925x_Y_OFFS_USR_LENGTH        8
#define MPU925x_Z_OFFS_USR_BIT        0x07
#define MPU925x_Z_OFFS_USR_LENGTH        8

// SMPLRT_DIV
#define MPU925x_SMPLRT_DIV_BIT        0x07
#define MPU925x_SMPLRT_DIV_LENGTH        8

// CONFIG
#define MPU925x_FIFO_MODE_BIT         0x06
#define MPU925x_FIFO_MODE_LENGTH         1
#define MPU925x_EXT_SYNC_SET_BIT      0x05
#define MPU925x_EXT_SYNC_SET_LENGTH      3
#define MPU925x_DLPF_CFG_SET_BIT      0x02
#define MPU925x_DLPF_CFG_SET_LENGTH      3

// GYRO_CONFIG
#define MPU925x_XGYRO_CT_EN_BIT       0x07
#define MPU925x_XGYRO_CT_EN_LENGTH       1
#define MPU925x_YGYRO_CT_EN_BIT       0x06
#define MPU925x_YGYRO_CT_EN_LENGTH       1
#define MPU925x_ZGYRO_CT_EN_BIT       0x05
#define MPU925x_ZGYRO_CT_EN_LENGTH       1
#define MPU925x_GYRO_FS_SEL_BIT       0x04
#define MPU925x_GYRO_FS_SEL_LENGTH       2
#define MPU925x_FCHOICE_B_BIT         0x01
#define MPU925x_FCHOICE_B_LENGTH         2

// ACCEL_CONFIG
#define MPU925x_AX_ST_EN_BIT            0x07
#define MPU925x_AX_ST_EN_LENGTH            1
#define MPU925x_AY_ST_EN_BIT            0x06
#define MPU925x_AY_ST_EN_LENGTH            1
#define MPU925x_AZ_ST_EN_BIT            0x05
#define MPU925x_AZ_ST_EN_LENGTH            1
#define MPU925x_ACCEL_FS_SEL_BIT        0x04
#define MPU925x_ACCEL_FS_SEL_LENGTH        2

// ACCEL_CONFIG 2
#define MPU925x_ACCEL_FCHOICE_B_BIT       0x03
#define MPU925x_ACCEL_FCHOICE_B_LENGTH       1
#define MPU925x_A_DLPF_CFG_BIT            0x02
#define MPU925x_A_DLPF_CFG_LENGTH            3

// LP_ACCEL_ODR
#define MPU925x_LPOSC_CLKSEL_BIT        0x03
#define MPU925x_LPOSC_CLKSEL_LENGTH        4

// WOM_THR
#define MPU925x_WOM_THRESHOLD_BIT       0x07
#define MPU925x_WOM_THRESHOLD_LENGTH       8

// FIFO_EN
#define MPU925x_TEMP_FIFO_EN_BIT      0x07
#define MPU925x_TEMP_FIFO_EN_LENGTH      1
#define MPU925x_GYRO_XO_UT_BIT        0x06
#define MPU925x_GYRO_XO_UT_LENGTH        1
#define MPU925x_GYRO_YO_UT_BIT        0x05
#define MPU925x_GYRO_YO_UT_LENGTH        1
#define MPU925x_GYRO_ZO_UT_BIT        0x04
#define MPU925x_GYRO_ZO_UT_LENGTH        1
#define MPU925x_ACCEL_BIT             0x03
#define MPU925x_ACCEL_LENGTH             1
#define MPU925x_SLV2_BIT              0x02
#define MPU925x_SLV2_LENGTH              1
#define MPU925x_SLV1_BIT              0x01
#define MPU925x_SLV1_LENGTH              1
#define MPU925x_SLV0_BIT              0x00
#define MPU925x_SLV0_LENGTH              1

// I2C_MST_CTRL
#define MPU925x_MULT_MST_EN_BIT         0x07
#define MPU925x_MULT_MST_EN_LENGTH         1
#define MPU925x_WAIT_FOR_ES_BIT         0x06
#define MPU925x_WAIT_FOR_ES_LENGTH         1
#define MPU925x_SLV_3_FIFO_EN_BIT       0x05
#define MPU925x_SLV_3_FIFO_EN_LENGTH       1
#define MPU925x_I2C_MST_P_NSR_BIT       0x04
#define MPU925x_I2C_MST_P_NSR_LENGTH       1
#define MPU925x_I2C_MST_CLK_BIT         0x03
#define MPU925x_I2C_MST_CLK_LENGTH         4

// I2C_SLV0_ADDR
#define MPU925x_I2C_SLV0_RNW_BIT        0x07
#define MPU925x_I2C_SLV0_RNW_LENGTH        1
#define MPU925x_I2C_ID_0_BIT            0x06
#define MPU925x_I2C_ID_0_LENGTH            7

// I2C_SLV0_REG
#define MPU925x_I2C_SLV0_REG_BIT        0x07
#define MPU925x_I2C_SLV0_REG_LENGTH        8

// I2C_SLV0_CTRL
#define MPU925x_I2C_SLV0_EN_BIT           0x07
#define MPU925x_I2C_SLV0_EN_LENGTH           1
#define MPU925x_I2C_SLV0_BYTE_SW_BIT      0x06
#define MPU925x_I2C_SLV0_BYTE_SW_LENGTH      1
#define MPU925x_I2C_SLV0_REG_DIS_BIT      0x05
#define MPU925x_I2C_SLV0_REG_DIS_LENGTH      1
#define MPU925x_I2C_SLV0_GRP_BIT          0x04
#define MPU925x_I2C_SLV0_GRP_LENGTH          1
#define MPU925x_I2C_SLV0_LENG_BIT         0x03
#define MPU925x_I2C_SLV0_LENG_LENGTH         4

// I2C_SLV1_ADDR
#define MPU925x_I2C_SLV1_RNW_BIT        0x07
#define MPU925x_I2C_SLV1_RNW_LENGTH        1
#define MPU925x_I2C_ID_1_BIT            0x06
#define MPU925x_I2C_ID_1_LENGTH            7

// I2C_SLV1_REG
#define MPU925x_I2C_SLV1_REG_BIT        0x07
#define MPU925x_I2C_SLV1_REG_LENGTH        8

// I2C_SLV1_CTRL
#define MPU925x_I2C_SLV1_EN_BIT           0x07
#define MPU925x_I2C_SLV1_EN_LENGTH           1
#define MPU925x_I2C_SLV1_BYTE_SW_BIT      0x06
#define MPU925x_I2C_SLV1_BYTE_SW_LENGTH      1
#define MPU925x_I2C_SLV1_REG_DIS_BIT      0x05
#define MPU925x_I2C_SLV1_REG_DIS_LENGTH      1
#define MPU925x_I2C_SLV1_GRP_BIT          0x04
#define MPU925x_I2C_SLV1_GRP_LENGTH          1
#define MPU925x_I2C_SLV1_LENG_BIT         0x03
#define MPU925x_I2C_SLV1_LENG_LENGTH         4

// I2C_SLV2_ADDR
#define MPU925x_I2C_SLV2_RNW_BIT        0x07
#define MPU925x_I2C_SLV2_RNW_LENGTH        1
#define MPU925x_I2C_ID_2_BIT            0x06
#define MPU925x_I2C_ID_2_LENGTH            7

// I2C_SLV2_REG
#define MPU925x_I2C_SLV2_REG_BIT        0x07
#define MPU925x_I2C_SLV2_REG_LENGTH        8

// I2C_SLV2_CTRL
#define MPU925x_I2C_SLV2_EN_BIT           0x07
#define MPU925x_I2C_SLV2_EN_LENGTH           1
#define MPU925x_I2C_SLV2_BYTE_SW_BIT      0x06
#define MPU925x_I2C_SLV2_BYTE_SW_LENGTH      1
#define MPU925x_I2C_SLV2_REG_DIS_BIT      0x05
#define MPU925x_I2C_SLV2_REG_DIS_LENGTH      1
#define MPU925x_I2C_SLV2_GRP_BIT          0x04
#define MPU925x_I2C_SLV2_GRP_LENGTH          1
#define MPU925x_I2C_SLV2_LENG_BIT         0x03
#define MPU925x_I2C_SLV2_LENG_LENGTH         4

// I2C_SLV3_ADDR
#define MPU925x_I2C_SLV3_RNW_BIT        0x07
#define MPU925x_I2C_SLV3_RNW_LENGTH        1
#define MPU925x_I2C_ID_3_BIT            0x06
#define MPU925x_I2C_ID_3_LENGTH            7

// I2C_SLV3_REG
#define MPU925x_I2C_SLV3_REG_BIT        0x07
#define MPU925x_I2C_SLV3_REG_LENGTH        8

// I2C_SLV3_CTRL
#define MPU925x_I2C_SLV3_EN_BIT           0x07
#define MPU925x_I2C_SLV3_EN_LENGTH           1
#define MPU925x_I2C_SLV3_BYTE_SW_BIT      0x06
#define MPU925x_I2C_SLV3_BYTE_SW_LENGTH      1
#define MPU925x_I2C_SLV3_REG_DIS_BIT      0x05
#define MPU925x_I2C_SLV3_REG_DIS_LENGTH      1
#define MPU925x_I2C_SLV3_GRP_BIT          0x04
#define MPU925x_I2C_SLV3_GRP_LENGTH          1
#define MPU925x_I2C_SLV3_LENG_BIT         0x03
#define MPU925x_I2C_SLV3_LENG_LENGTH         4

// I2C_SLV4_ADDR
#define MPU925x_I2C_SLV4_RNW_BIT        0x07
#define MPU925x_I2C_SLV4_RNW_LENGTH        1
#define MPU925x_I2C_ID_4_BIT            0x06
#define MPU925x_I2C_ID_4_LENGTH         0x07

// I2C_SLV4_REG
#define MPU925x_I2C_SLV4_REG_BIT        0x07
#define MPU925x_I2C_SLV4_REG_LENGTH        8

// I2C_SLV4_DO
#define MPU925x_I2C_SLV4_DO_BIT         0x07
#define MPU925x_I2C_SLV4_DO_LENGTH         8

// I2C_SLV4_CTRL
#define MPU925x_I2C_SLV4_EN_BIT               0x07
#define MPU925x_I2C_SLV4_EN_LENGTH               1
#define MPU925x_I2C_SLV4_DONE_INT_EN_BIT      0x06
#define MPU925x_I2C_SLV4_DONE_INT_EN_LENGTH      1
#define MPU925x_I2C_SLV4_REG_DIS_BIT          0x05
#define MPU925x_I2C_SLV4_REG_DIS_LENGTH          1
#define MPU925x_I2C_MST_DLY_BIT               0x04
#define MPU925x_I2C_MST_DLY_LENGTH            0x05

// I2C_SLV4_DI
#define MPU925x_I2C_SLV4_DI_BIT         0x07
#define MPU925x_I2C_SLV4_DI_LENGTH         8

// I2C_MST_STATUS
#define MPU925x_PASS_THROUGH_BIT        0x07
#define MPU925x_PASS_THROUGH_LENGTH        1
#define MPU925x_I2C_SLV4_DONE_BIT       0x06
#define MPU925x_I2C_SLV4_DONE_LENGTH       1
#define MPU925x_I2C_LOST_ARB_BIT        0x05
#define MPU925x_I2C_LOST_ARB_LENGTH        1
#define MPU925x_I2C_SLV4_NACK_BIT       0x04
#define MPU925x_I2C_SLV4_NACK_LENGTH       1
#define MPU925x_SLV3_NACK_BIT           0x03
#define MPU925x_SLV3_NACK_LENGTH           1
#define MPU925x_SLV2_NACK_BIT           0x02
#define MPU925x_SLV2_NACK_LENGTH           1
#define MPU925x_SLV1_NACK_BIT           0x01
#define MPU925x_SLV1_NACK_LENGTH           1
#define MPU925x_SLV0_NACK_BIT           0x00
#define MPU925x_SLV0_NACK_LENGTH           1

// INT_PIN_CFG
#define MPU925x_ACTL_BIT                  0x07
#define MPU925x_ACTL_LENGTH                  1
#define MPU925x_OPEN_BIT                  0x06
#define MPU925x_OPEN_LENGTH                  1
#define MPU925x_LATCH_INT_EN_BIT          0x05
#define MPU925x_LATCH_INT_EN_LENGTH          1
#define MPU925x_INT_ANYRD_2CLEAR_BIT      0x04
#define MPU925x_INT_ANYRD_2CLEAR_LENGTH      1
#define MPU925x_ACTL_FSYNC_BIT            0x03
#define MPU925x_ACTL_FSYNC_LENGTH            1
#define MPU925x_FSYNC_INT_MODE_EN_BIT     0x02
#define MPU925x_FSYNC_INT_MODE_EN_LENGTH     1
#define MPU925x_BYPASS_EN_BIT             0x01
#define MPU925x_BYPASS_EN_LENGTH             1

// INT_ENABLE
#define MPU925x_WOM_EN_BIT              0x06
#define MPU925x_WOM_EN_LENGTH              1
#define MPU925x_FIFO_OFLOW_EN_BIT       0x04
#define MPU925x_FIFO_OFLOW_EN_LENGTH       1
#define MPU925x_FSYNC_INT_EN_BIT        0x03
#define MPU925x_FSYNC_INT_EN_LENGTH        1
#define MPU925x_RAW_RDY_EN_BIT          0x00
#define MPU925x_RAW_RDY_EN_LENGTH          1

// INT_STATUS
#define MPU925x_WOM_INT_BIT               0x06
#define MPU925x_WOM_INT_LENGTH               1
#define MPU925x_FIFO_OFLOW_INT_BIT        0x04
#define MPU925x_FIFO_OFLOW_INT_LENGTH        1
#define MPU925x_FSYNC_INT_INT_BIT         0x03
#define MPU925x_FSYNC_INT_INT_LENGTH         1
#define MPU925x_RAW_DATA_RDY_INT_BIT      0x00
#define MPU925x_RAW_DATA_RDY_INT_LENGTH      1

// ACCEL_XOUT_H
#define MPU925x_ACCEL_XOUT_H_BIT          0x07
#define MPU925x_ACCEL_XOUT_H_LENGTH          8
// ACCEL_XOUT_L
#define MPU925x_ACCEL_XOUT_L_BIT          0x07
#define MPU925x_ACCEL_XOUT_L_LENGTH          8
// ACCEL_YOUT_H
#define MPU925x_ACCEL_YOUT_H_BIT          0x07
#define MPU925x_ACCEL_YOUT_H_LENGTH          8
// ACCEL_YOUT_L
#define MPU925x_ACCEL_YOUT_L_BIT          0x07
#define MPU925x_ACCEL_YOUT_L_LENGTH          8
// ACCEL_ZOUT_H
#define MPU925x_ACCEL_ZOUT_H_BIT          0x07
#define MPU925x_ACCEL_ZOUT_H_LENGTH          8
// ACCEL_ZOUT_L
#define MPU925x_ACCEL_ZOUT_L_BIT          0x07
#define MPU925x_ACCEL_ZOUT_L_LENGTH          8
// TEMP_OUT_H
#define MPU925x_TEMP_OUT_H_BIT            0x07
#define MPU925x_TEMP_OUT_H_LENGTH            8
// TEMP_OUT_L
#define MPU925x_TEMP_OUT_L_BIT            0x07
#define MPU925x_TEMP_OUT_L_LENGTH            8
// GYRO_XOUT_H
#define MPU925x_GYRO_XOUT_H_BIT           0x07
#define MPU925x_GYRO_XOUT_H_LENGTH           8
// GYRO_XOUT_L
#define MPU925x_GYRO_XOUT_L_BIT           0x07
#define MPU925x_GYRO_XOUT_L_LENGTH           8
// GYRO_YOUT_H
#define MPU925x_GYRO_YOUT_H_BIT           0x07
#define MPU925x_GYRO_YOUT_H_LENGTH           8
// GYRO_YOUT_L
#define MPU925x_GYRO_YOUT_L_BIT           0x07
#define MPU925x_GYRO_YOUT_L_LENGTH           8
// GYRO_ZOUT_H
#define MPU925x_GYRO_ZOUT_H_BIT           0x07
#define MPU925x_GYRO_ZOUT_H_LENGTH           8
// GYRO_ZOUT_L
#define MPU925x_GYRO_ZOUT_L_BIT           0x07
#define MPU925x_GYRO_ZOUT_L_LENGTH           8
// EXT_SENS_DATA_00
#define MPU925x_EXT_SENS_DATA_00_BIT      0x07
#define MPU925x_EXT_SENS_DATA_00_LENGTH      8
// EXT_SENS_DATA_02
#define MPU925x_EXT_SENS_DATA_02_BIT      0x07
#define MPU925x_EXT_SENS_DATA_02_LENGTH      8
// EXT_SENS_DATA_03
#define MPU925x_EXT_SENS_DATA_03_BIT      0x07
#define MPU925x_EXT_SENS_DATA_03_LENGTH      8
// EXT_SENS_DATA_04
#define MPU925x_EXT_SENS_DATA_04_BIT      0x07
#define MPU925x_EXT_SENS_DATA_04_LENGTH      8
// EXT_SENS_DATA_05
#define MPU925x_EXT_SENS_DATA_05_BIT      0x07
#define MPU925x_EXT_SENS_DATA_05_LENGTH      8
// EXT_SENS_DATA_06
#define MPU925x_EXT_SENS_DATA_06_BIT      0x07
#define MPU925x_EXT_SENS_DATA_06_LENGTH      8
// EXT_SENS_DATA_07
#define MPU925x_EXT_SENS_DATA_07_BIT      0x07
#define MPU925x_EXT_SENS_DATA_07_LENGTH      8
// EXT_SENS_DATA_08
#define MPU925x_EXT_SENS_DATA_08_BIT      0x07
#define MPU925x_EXT_SENS_DATA_08_LENGTH      8
// EXT_SENS_DATA_09
#define MPU925x_EXT_SENS_DATA_09_BIT      0x07
#define MPU925x_EXT_SENS_DATA_09_LENGTH      8
// EXT_SENS_DATA_10
#define MPU925x_EXT_SENS_DATA_10_BIT      0x07
#define MPU925x_EXT_SENS_DATA_10_LENGTH      8
// EXT_SENS_DATA_11
#define MPU925x_EXT_SENS_DATA_11_BIT      0x07
#define MPU925x_EXT_SENS_DATA_11_LENGTH      8
// EXT_SENS_DATA_12
#define MPU925x_EXT_SENS_DATA_12_BIT      0x07
#define MPU925x_EXT_SENS_DATA_12_LENGTH      8
// EXT_SENS_DATA_13
#define MPU925x_EXT_SENS_DATA_13_BIT      0x07
#define MPU925x_EXT_SENS_DATA_13_LENGTH      8
// EXT_SENS_DATA_14
#define MPU925x_EXT_SENS_DATA_14_BIT      0x07
#define MPU925x_EXT_SENS_DATA_14_LENGTH      8
// EXT_SENS_DATA_15
#define MPU925x_EXT_SENS_DATA_15_BIT      0x07
#define MPU925x_EXT_SENS_DATA_15_LENGTH      8
// EXT_SENS_DATA_16
#define MPU925x_EXT_SENS_DATA_16_BIT      0x07
#define MPU925x_EXT_SENS_DATA_16_LENGTH      8
// EXT_SENS_DATA_17
#define MPU925x_EXT_SENS_DATA_17_BIT      0x07
#define MPU925x_EXT_SENS_DATA_17_LENGTH      8
// EXT_SENS_DATA_18
#define MPU925x_EXT_SENS_DATA_18_BIT      0x07
#define MPU925x_EXT_SENS_DATA_18_LENGTH      8
// EXT_SENS_DATA_19
#define MPU925x_EXT_SENS_DATA_19_BIT      0x07
#define MPU925x_EXT_SENS_DATA_19_LENGTH      8
// EXT_SENS_DATA_20
#define MPU925x_EXT_SENS_DATA_20_BIT      0x07
#define MPU925x_EXT_SENS_DATA_20_LENGTH      8
// EXT_SENS_DATA_21
#define MPU925x_EXT_SENS_DATA_21_BIT      0x07
#define MPU925x_EXT_SENS_DATA_21_LENGTH      8
// EXT_SENS_DATA_22
#define MPU925x_EXT_SENS_DATA_22_BIT      0x07
#define MPU925x_EXT_SENS_DATA_22_LENGTH      8
// EXT_SENS_DATA_23
#define MPU925x_EXT_SENS_DATA_23_BIT      0x07
#define MPU925x_EXT_SENS_DATA_23_LENGTH      8
// I2C_SLV0_DO
#define MPU925x_I2C_SLV0_DO_BIT           0x07
#define MPU925x_I2C_SLV0_DO_LENGTH           8
// I2C_SLV1_DO
#define MPU925x_I2C_SLV1_DO_BIT           0x07
#define MPU925x_I2C_SLV1_DO_LENGTH           8
// I2C_SLV2_DO
#define MPU925x_I2C_SLV2_DO_BIT           0x07
#define MPU925x_I2C_SLV2_DO_LENGTH           8
// I2C_SLV3_DO
#define MPU925x_I2C_SLV3_DO_BIT           0x07
#define MPU925x_I2C_SLV3_DO_LENGTH           8

// I2C_MST_DELAY_CTRL
#define MPU925x_DELAY_ES_SHADOW_BIT       0x07
#define MPU925x_DELAY_ES_SHADOW_LENGTH       1
#define MPU925x_I2C_SLV4_DLY_EN_BIT       0x04
#define MPU925x_I2C_SLV4_DLY_EN_LENGTH       1
#define MPU925x_I2C_SLV3_DLY_EN_BIT       0x03
#define MPU925x_I2C_SLV3_DLY_EN_LENGTH       1
#define MPU925x_I2C_SLV2_DLY_EN_BIT       0x02
#define MPU925x_I2C_SLV2_DLY_EN_LENGTH       1
#define MPU925x_I2C_SLV1_DLY_EN_BIT       0x01
#define MPU925x_I2C_SLV1_DLY_EN_LENGTH       1
#define MPU925x_I2C_SLV0_DLY_EN_BIT       0x00
#define MPU925x_I2C_SLV0_DLY_EN_LENGTH       1

// SIGNAL_PATH_RESET
#define MPU925x_GYRO_RST_BIT          0x02
#define MPU925x_GYRO_RST_LENGTH          1
#define MPU925x_ACCEL_RST_BIT         0x01
#define MPU925x_ACCEL_RST_LENGTH         1
#define MPU925x_TEMP_RST_BIT          0x00
#define MPU925x_TEMP_RST_LENGTH          1

// MOT_DETECT_CTRL
#define MPU925x_ACCEL_INTEL_EN_BIT        0x07
#define MPU925x_ACCEL_INTEL_EN_LENGTH        1
#define MPU925x_ACCEL_INTEL_MODE_BIT      0x06
#define MPU925x_ACCEL_INTEL_MODE_LENGTH      1

// USER_CTRL
#define MPU925x_FIFO_EN_BIT           0x06
#define MPU925x_FIFO_EN_LENGTH           1
#define MPU925x_I2C_MST_EN_BIT        0x05
#define MPU925x_I2C_MST_EN_LENGTH        1
#define MPU925x_I2C_IF_DIS_BIT        0x04
#define MPU925x_I2C_IF_DIS_LENGTH        1
#define MPU925x_FIFO_RST_BIT          0x02
#define MPU925x_FIFO_RST_LENGTH          1
#define MPU925x_I2C_MST_RST_BIT       0x01
#define MPU925x_I2C_MST_RST_LENGTH       1
#define MPU925x_SIG_COND_RST_BIT      0x00
#define MPU925x_SIG_COND_RST_LENGTH      1

// PWR_MGMT_1
#define MPU925x_H_RESET_BIT           0x07
#define MPU925x_H_RESET_LENGTH           1
#define MPU925x_SLEEP_BIT             0x06
#define MPU925x_SLEEP_LENGTH             1
#define MPU925x_CYCLE_BIT             0x05
#define MPU925x_CYCLE_LENGTH             1
#define MPU925x_GYRO_STANDBY_BIT      0x04
#define MPU925x_GYRO_STANDBY_LENGTH      1
#define MPU925x_PD_PTAT_BIT           0x03
#define MPU925x_PD_PTAT_LENGTH           1
#define MPU925x_CLKSEL_BIT            0x02
#define MPU925x_CLKSEL_LENGTH            3

// PWR_MGMT_2
#define MPU925x_DIS_XA_BIT          0x05
#define MPU925x_DIS_XA_LENGTH          1
#define MPU925x_DIS_YA_BIT          0x04
#define MPU925x_DIS_YA_LENGTH          1
#define MPU925x_DIS_ZA_BIT          0x03
#define MPU925x_DIS_ZA_LENGTH          1
#define MPU925x_DIS_XG_BIT          0x02
#define MPU925x_DIS_XG_LENGTH          1
#define MPU925x_DIS_YG_BIT          0x01
#define MPU925x_DIS_YG_LENGTH          1
#define MPU925x_DIS_ZG_BIT          0x00
#define MPU925x_DIS_ZG_LENGTH          1

// FIFO_COUNTH
#define MPU925x_FIFO_CNT_BIT          0x04
#define MPU925x_FIFO_CNT_LENGTH         19

// FIFO_R_W
#define MPU925x_D_BIT                 0x07
#define MPU925x_D_LENGTH                 8

// WHO_AM_I
#define MPU925x_WHOAMI_BIT            0x07
#define MPU925x_WHOAMI_LENGTH            8

// XA_OFFSET_H
#define MPU925x_XA_OFFS_BIT           0x07
#define MPU925x_XA_OFFS_LENGTH          15
// YA_OFFSET_H
#define MPU925x_YA_OFFS_BIT           0x07
#define MPU925x_YA_OFFS_LENGTH          15
// ZA_OFFSET_H
#define MPU925x_ZA_OFFS_BIT           0x07
#define MPU925x_ZA_OFFS_LENGTH          15


class MPU925x_I2C {
public:
  MPU925x_I2C(uint8_t address = MPU925x_ADDRESS_AD0_LOW);
  void initialize();

  uint8_t getDeviceId() const;
  bool testConnection() const;

  int16_t getXGyroOffset();
  bool setXGyroOffset(int16_t offset);
  int16_t getYGyroOffset();
  bool setYGyroOffset(int16_t offset);
  int16_t getZGyroOffset();
  bool setZGyroOffset(int16_t offset);
  int16_t getXAccelOffset();
  bool setXAccelOffset(int16_t offset);
  int16_t getYAccelOffset();
  bool setYAccelOffset(int16_t offset);
  int16_t getZAccelOffset();
  bool setZAccelOffset(int16_t offset);

  //bool calibrate() calibrate and return values
  // so they can be saved in eeprom

  bool setGyroDLPF(int8_t dlpf);
  int8_t getGyroDLPF();
  bool setAccelDLPF(int8_t dlpf);
  int8_t getAccelDLPF();
  bool setAccelFullScaleRange(uint8_t scale);
  uint8_t getAccelFullScaleRange();
  bool setGyroFullScaleRange(uint8_t scale);
  uint8_t getGyroFullScaleRange();

  uint8_t getRate();
  bool setRate(uint8_t rate);

  uint8_t getClockSource();
  bool setClockSource(uint8_t clockSource);

  bool getSleepMode();
  bool setSleepMode(bool enabled = true);

  bool getAccelRaw(int16_t* x, int16_t* y, int16_t* z);
  bool getAccel(float* x, float* y, float* z);
  bool getAccelXRaw(int16_t* x);
  bool getAccelYRaw(int16_t* y);
  bool getAccelZRaw(int16_t* z);
  bool getGyroRaw(int16_t* x, int16_t* y, int16_t* z);
  bool getGyro(float* x, float* y, float* z);
  bool getTemperatureRaw(uint16_t* temperature);
  bool getTemperature(float* temperature);
  bool getAccelAndGyroRaw(int16_t* ax, int16_t* yx, int16_t* zx,
                          int16_t* gx, int16_t* gy, int16_t* gz);

  void getFIFOBytes(uint8_t *data, uint8_t length);

  float accelRawToG(const int16_t accelRaw) const;
  float accelRawToMs2(const int16_t accelRaw) const;
  float gyroRawToDegS(const int16_t gyroRaw) const;
  float gyroRawToRadS(const int16_t gyroRaw) const;

  // free function
  // void setRegister(address...);
  // void getRegister(address...);

private:
  uint8_t _i2cAddress = MPU925x_ADDRESS_AD0_LOW;
  uint8_t _buffer[14];
  float _accelerometerSensitivity = 131.0f;
  int16_t _gyroscopeSensitivity = 16384;
};


/*
Gyroscope DLPF
==============
| Fchoice 1:0 | DLPF_CFG | Bandwidth, Hz | Delay, ms | Fs, kHz |
|-------------|----------|---------------|-----------|---------|
|     x 0     |    x     |     8800      |   0.064   |   32    |
|     0 1     |    x     |     3600      |    0.11   |   32    |
|     1 1     |    0     |      250      |    0.97   |    8    |
|     1 1     |    1     |      184      |     2.9   |    1    |
|     1 1     |    2     |       92      |     3.9   |    1    |
|     1 1     |    3     |       41      |     5.9   |    1    |
|     1 1     |    4     |       20      |     9.9   |    1    |
|     1 1     |    5     |       10      |   17.85   |    1    |
|     1 1     |    6     |        5      |   33.48   |    1    |
|     1 1     |    7     |     3600      |    0.17   |    8    |


Accelerometer DLPF for MPU9250
==============================
| Fchoice | DLPF_CFG | Bandwidth, Hz | Delay, ms | Fs, kHz |
|---------|----------|---------------|-----------|---------|
|    0    |     x    |      1046     |   0.503   |    4    |
|    1    |     0    |     218.1     |    1.88   |    1    |
|    1    |     1    |     218.1     |    1.88   |    1    |
|    1    |     2    |        99     |    2.88   |    1    |
|    1    |     3    |      44.8     |    4.88   |    1    |
|    1    |     4    |      21.2     |    8.87   |    1    |
|    1    |     5    |      10.2     |   16.83   |    1    |
|    1    |     6    |      5.05     |   32.48   |    1    |
|    1    |     7    |       420     |    1.38   |    1    |


Accelerometer DLPF for MPU9255
==============================
| Fchoice | DLPF_CFG | Bandwidth, Hz | Delay, ms | Fs, kHz |
|---------|----------|---------------|-----------|---------|
|    0    |     x    |     1.13k     |    0.75   |    4    |
|    1    |     0    |       460     |    1.94   |    1    |
|    1    |     1    |       184     |     5.8   |    1    |
|    1    |     2    |        92     |     7.8   |    1    |
|    1    |     3    |        41     |    11.8   |    1    |
|    1    |     4    |        20     |    19.8   |    1    |
|    1    |     5    |        10     |    35.7   |    1    |
|    1    |     6    |         5     |   66.96   |    1    |
|    1    |     7    |       420     |    1.94   |    1    |
*/
