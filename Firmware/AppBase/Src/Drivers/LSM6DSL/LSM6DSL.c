/*
 *    ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 *   (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *    ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \
 *   (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2018 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO WARRANTY
 * of ANY KIND is provided. This heading must NOT be removed from the file.
 *
 * Date:          23/05/2018 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Driver of LSM6DSL 3 axis accelerometer and 3 axis gyroscope.
 *
 */

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>

/* Self include */
#include "LSM6DSL.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
/****************/
/* Register MAP */
/****************/
/* Embedded functions configuration register */
#define FUNC_CFG_ACCESS_REG            (uint8_t)0x01

/* Sensor sync configuration register */
#define SENSOR_SYNC_TIME_FRAME_REG     (uint8_t)0x04
#define SENSOR_SYNC_RES_RATIO_REG      (uint8_t)0x05
/* FIFO configuration registers */
#define FIFO_CTRL1_REG                 (uint8_t)0x06
#define FIFO_CTRL2_REG                 (uint8_t)0x07
#define FIFO_CTRL3_REG                 (uint8_t)0x08
#define FIFO_CTRL4_REG                 (uint8_t)0x09
#define FIFO_CTRL5_REG                 (uint8_t)0x0A
/* DataReady configuration register */
#define DRDY_PULSE_CFG_G_REG           (uint8_t)0x0B

/* INT1/2 pin control */
#define INT1_CTRL_REG                  (uint8_t)0x0D
#define INT2_CTRL_REG                  (uint8_t)0x0E
/* Who I am ID */
#define WHO_AM_I_REG                   (uint8_t)0x0F
/* Accelerometer and gyroscope control registers */
#define CTRL1_XL_REG                   (uint8_t)0x10
#define CTRL2_G_REG                    (uint8_t)0x11
#define CTRL3_C_REG                    (uint8_t)0x12
#define CTRL4_C_REG                    (uint8_t)0x13
#define CTRL5_C_REG                    (uint8_t)0x14
#define CTRL6_C_REG                    (uint8_t)0x15
#define CTRL7_G_REG                    (uint8_t)0x16
#define CTRL8_XL_REG                   (uint8_t)0x17
#define CTRL9_XL_REG                   (uint8_t)0x18
#define CTRL10_C_REG                   (uint8_t)0x19
/* I2C master configuration register */
#define MASTER_CONFIG_REG              (uint8_t)0x1A
/* Interrupt registers */
#define WAKE_UP_SRC_REG                (uint8_t)0x1B
#define TAP_SRC_REG                    (uint8_t)0x1C
#define D6D_SRC_REG                    (uint8_t)0x1D
/* Status data register for user interface */
#define STATUS_REG                     (uint8_t)0x1E

/* Temperature output data registers */
#define OUT_TEMP_L_REG                 (uint8_t)0x20
#define OUT_TEMP_H_REG                 (uint8_t)0x21
/* Gyroscope output registers for user interface */
#define OUTX_L_G_REG                   (uint8_t)0x22
#define OUTX_H_G_REG                   (uint8_t)0x23
#define OUTY_L_G_REG                   (uint8_t)0x24
#define OUTY_H_G_REG                   (uint8_t)0x25
#define OUTZ_L_G_REG                   (uint8_t)0x26
#define OUTZ_H_G_REG                   (uint8_t)0x27
/* Accelerometer output registers */
#define OUTX_L_XL_REG                  (uint8_t)0x28
#define OUTX_H_XL_REG                  (uint8_t)0x29
#define OUTY_L_XL_REG                  (uint8_t)0x2A
#define OUTY_H_XL_REG                  (uint8_t)0x2B
#define OUTZ_L_XL_REG                  (uint8_t)0x2C
#define OUTZ_H_XL_REG                  (uint8_t)0x2D
/* Sensor hub output registers */
#define SENSORHUB1_REG                 (uint8_t)0x2E
#define SENSORHUB2_REG                 (uint8_t)0x2F
#define SENSORHUB3_REG                 (uint8_t)0x30
#define SENSORHUB4_REG                 (uint8_t)0x31
#define SENSORHUB5_REG                 (uint8_t)0x32
#define SENSORHUB6_REG                 (uint8_t)0x33
#define SENSORHUB7_REG                 (uint8_t)0x34
#define SENSORHUB8_REG                 (uint8_t)0x35
#define SENSORHUB9_REG                 (uint8_t)0x36
#define SENSORHUB10_REG                (uint8_t)0x37
#define SENSORHUB11_REG                (uint8_t)0x38
#define SENSORHUB12_REG                (uint8_t)0x39
/* FIFO status registers */
#define FIFO_STATUS1_REG               (uint8_t)0x3A
#define FIFO_STATUS2_REG               (uint8_t)0x3B
#define FIFO_STATUS3_REG               (uint8_t)0x3C
#define FIFO_STATUS4_REG               (uint8_t)0x3D
/* FIFO data output registers */
#define FIFO_DATA_OUT_L_REG            (uint8_t)0x3E
#define FIFO_DATA_OUT_H_REG            (uint8_t)0x3F
/* Timestamp output registers */
#define TIMESTAMP0_REG                 (uint8_t)0x40
#define TIMESTAMP1_REG                 (uint8_t)0x41
#define TIMESTAMP2_REG                 (uint8_t)0x42

/* Step counter timestamp registers */
#define STEP_TIMESTAMP_L_REG           (uint8_t)0x49
#define STEP_TIMESTAMP_H_REG           (uint8_t)0x4A
/* Step counter output registers */
#define STEP_COUNTER_L_REG             (uint8_t)0x4B
#define STEP_COUNTER_H_REG             (uint8_t)0x4C
/* Sensor hub output registers */
#define SENSORHUB13_REG                (uint8_t)0x4D
#define SENSORHUB14_REG                (uint8_t)0x4E
#define SENSORHUB15_REG                (uint8_t)0x4F
#define SENSORHUB16_REG                (uint8_t)0x50
#define SENSORHUB17_REG                (uint8_t)0x51
#define SENSORHUB18_REG                (uint8_t)0x52
/* Interrupt registers */
#define FUNC_SRC1_REG                  (uint8_t)0x53
#define FUNC_SRC2_REG                  (uint8_t)0x54
#define WRIST_TILT_IA_REG              (uint8_t)0x55

/* Interrupt registers */
#define TAP_CFG_REG                    (uint8_t)0x58
#define TAP_THS_6D_REG                 (uint8_t)0x59
#define INT_DUR2_REG                   (uint8_t)0x5A
#define WAKE_UP_THS_REG                (uint8_t)0x5B
#define WAKE_UP_DUR_REG                (uint8_t)0x5C
#define FREE_FALL_REG                  (uint8_t)0x5D
#define MD1_CFG_REG                    (uint8_t)0x5E
#define MD2_CFG_REG                    (uint8_t)0x5F

#define MASTER_CMD_CODE_REG            (uint8_t)0x60
#define SENS_SYNC_SPI_ERROR_CODE_REG   (uint8_t)0x61

/* External magnetometer raw data output registers */
#define OUT_MAG_RAW_X_L_REG            (uint8_t)0x66
#define OUT_MAG_RAW_X_H_REG            (uint8_t)0x67
#define OUT_MAG_RAW_Y_L_REG            (uint8_t)0x68
#define OUT_MAG_RAW_Y_H_REG            (uint8_t)0x69
#define OUT_MAG_RAW_Z_L_REG            (uint8_t)0x6A
#define OUT_MAG_RAW_Z_H_REG            (uint8_t)0x6B

/* Accelerometer user offset correction */
#define X_OFS_USR_REG                  (uint8_t)0x73
#define Y_OFS_USR_REG                  (uint8_t)0x74
#define Z_OFS_USR_REG                  (uint8_t)0x75

#define LAST_REGISTER                  (uint8_t)(Z_OFS_USR_REG + 1u)

/*********************/
/* Mask and Position */
/*********************/
/* FUNC_CFG_ACCESS Msk Pos */
#define FUNC_CFG_ACCESS_EN_B_POS       (uint8_t)5
#define FUNC_CFG_ACCESS_EN_B_MSK       (uint8_t)0x20
#define FUNC_CFG_ACCESS_EN_A_POS       (uint8_t)7
#define FUNC_CFG_ACCESS_EN_A_MSK       (uint8_t)0x80

/* INT1 CTRL Msk Pos */
#define INT1_CTRL_DRDY_XL_POS          (uint8_t)0
#define INT1_CTRL_DRDY_XL_MSK          (uint8_t)0x01
#define INT1_CTRL_DRDY_G_POS           (uint8_t)1
#define INT1_CTRL_DRDY_G_MSK           (uint8_t)0x02
#define INT1_CTRL_BOOT_POS             (uint8_t)2
#define INT1_CTRL_BOOT_MSK             (uint8_t)0x04
#define INT1_CTRL_FTH_POS              (uint8_t)3
#define INT1_CTRL_FTH_MSK              (uint8_t)0x08
#define INT1_CTRL_FIFO_OVR_POS         (uint8_t)4
#define INT1_CTRL_FIFO_OVR_MSK         (uint8_t)0x10
#define INT1_CTRL_FULL_FLAG_POS        (uint8_t)5
#define INT1_CTRL_FULL_FLAG_MSK        (uint8_t)0x20
#define INT1_CTRL_SIGN_MOT_POS         (uint8_t)6
#define INT1_CTRL_SIGN_MOT_MSK         (uint8_t)0x40
#define INT1_CTRL_STEP_DETEC_POS       (uint8_t)7
#define INT1_CTRL_STEP_DETEC_MSK       (uint8_t)0x80

/* INT2 CTRL Msk Pos */
#define INT2_CTRL_DRDY_XL_POS          (uint8_t)0
#define INT2_CTRL_DRDY_XL_MSK          (uint8_t)0x01
#define INT2_CTRL_DRDY_G_POS           (uint8_t)1
#define INT2_CTRL_DRDY_G_MSK           (uint8_t)0x02
#define INT2_CTRL_TEMP_POS             (uint8_t)2
#define INT2_CTRL_TEMP_MSK             (uint8_t)0x04
#define INT2_CTRL_FTH_POS              (uint8_t)3
#define INT2_CTRL_FTH_MSK              (uint8_t)0x08
#define INT2_CTRL_FIFO_OVR_POS         (uint8_t)4
#define INT2_CTRL_FIFO_OVR_MSK         (uint8_t)0x10
#define INT2_CTRL_FULL_FLAG_POS        (uint8_t)5
#define INT2_CTRL_FULL_FLAG_MSK        (uint8_t)0x20
#define INT2_CTRL_STEP_CNT_OV_POS      (uint8_t)6
#define INT2_CTRL_STEP_CNT_OV_MSK      (uint8_t)0x40
#define INT2_CTRL_STEP_DELTA_POS       (uint8_t)7
#define INT2_CTRL_STEP_DELTA_MSK       (uint8_t)0x80

/* CTRL1_XL Msk Pos */
#define CTRL1_XL_BW0_POS               (uint8_t)0
#define CTRL1_XL_BW0_MSK               (uint8_t)0x01
#define CTRL1_XL_LPF1_BW_POS           (uint8_t)1
#define CTRL1_XL_LPF1_BW_MSK           (uint8_t)0x02
#define CTRL1_XL_FS_POS                (uint8_t)2
#define CTRL1_XL_FS_MSK                (uint8_t)0x0C
#define CTRL1_XL_ODR_POS               (uint8_t)4
#define CTRL1_XL_ODR_MSK               (uint8_t)0xF0

/* CTRL2_G Msk Pos */
#define CTRL2_G_FS125_POS              (uint8_t)1
#define CTRL2_G_FS125_MSK              (uint8_t)0x02
#define CTRL2_G_FS_POS                 (uint8_t)2
#define CTRL2_G_FS_MSK                 (uint8_t)0x0C
#define CTRL2_G_ODR_POS                (uint8_t)4
#define CTRL2_G_ODR_MSK                (uint8_t)0xF0

/* CTRL3_C Msk Pos */
#define CTRL3_C_SW_RESET_POS           (uint8_t)0
#define CTRL3_C_SW_RESET_MSK           (uint8_t)0x01
#define CTRL3_C_BLE_POS                (uint8_t)1
#define CTRL3_C_BLE_MSK                (uint8_t)0x02
#define CTRL3_C_IF_INC_POS             (uint8_t)2
#define CTRL3_C_IF_INC_MSK             (uint8_t)0x04
#define CTRL3_C_SIM_POS                (uint8_t)3
#define CTRL3_C_SIM_MSK                (uint8_t)0x08
#define CTRL3_C_PP_OP_POS              (uint8_t)4
#define CTRL3_C_PP_OP_MSK              (uint8_t)0x10
#define CTRL3_C_H_LACTIVE_POS          (uint8_t)5
#define CTRL3_C_H_LACTIVE_MSK          (uint8_t)0x20
#define CTRL3_C_BDU_POS                (uint8_t)6
#define CTRL3_C_BDU_MSK                (uint8_t)0x40
#define CTRL3_C_BOOT_POS               (uint8_t)7
#define CTRL3_C_BOOT_MSK               (uint8_t)0x80

/* CTRL4_C Msk Pos */
#define CTRL4_C_LPF1_SEL_G_POS         (uint8_t)1
#define CTRL4_C_LPF1_SEL_G_MSK         (uint8_t)0x02
#define CTRL4_C_I2C_DIS_POS            (uint8_t)2
#define CTRL4_C_I2C_DIS_MSK            (uint8_t)0x04
#define CTRL4_C_DRDY_MASK_POS          (uint8_t)3
#define CTRL4_C_DRDY_MASK_MSK          (uint8_t)0x08
#define CTRL4_C_DEN_DRDY_INT1_POS      (uint8_t)4
#define CTRL4_C_DEN_DRDY_INT1_MSK      (uint8_t)0x10
#define CTRL4_C_INT2_ON_INT1_POS       (uint8_t)5
#define CTRL4_C_INT2_ON_INT1_MSK       (uint8_t)0x20
#define CTRL4_C_SLEEP_POS              (uint8_t)6
#define CTRL4_C_SLEEP_MSK              (uint8_t)0x40
#define CTRL4_C_DEN_XL_EN_POS          (uint8_t)7
#define CTRL4_C_DEN_XL_EN_MSK          (uint8_t)0x80

/* CTRL5_C Msk Pos */
#define CTRL5_C_ST_XL_POS              (uint8_t)0
#define CTRL5_C_ST_XL_MSK              (uint8_t)0x03
#define CTRL5_C_ST_G_POS               (uint8_t)2
#define CTRL5_C_ST_G_MSK               (uint8_t)0x0C
#define CTRL5_C_DEN_LH_POS             (uint8_t)4
#define CTRL5_C_DEN_LH_MSK             (uint8_t)0x10
#define CTRL5_C_ROUNDING_POS           (uint8_t)5
#define CTRL5_C_ROUNDING_MSK           (uint8_t)0xE0

/* CTRL6_C Msk Pos */
#define CTRL6_C_FTYPE_POS              (uint8_t)0
#define CTRL6_C_FTYPE_MSK              (uint8_t)0x03
#define CTRL6_C_USR_OFF_W_POS          (uint8_t)3
#define CTRL6_C_USR_OFF_W_MSK          (uint8_t)0x08
#define CTRL6_C_XL_HM_MODE_POS         (uint8_t)4
#define CTRL6_C_XL_HM_MODE_MSK         (uint8_t)0x10
#define CTRL6_C_TRIG_MODE_POS          (uint8_t)5
#define CTRL6_C_TRIG_MODE_MSK          (uint8_t)0xE0

/* CTRL7_C Msk Pos */
#define CTRL7_G_ROUNDING_STATUS_POS    (uint8_t)2
#define CTRL7_G_ROUNDING_STATUS_MSK    (uint8_t)0x04
#define CTRL7_G_HPM_G_POS              (uint8_t)4
#define CTRL7_G_HPM_G_MSK              (uint8_t)0x30
#define CTRL7_G_HP_EN_G_POS            (uint8_t)6
#define CTRL7_G_HP_EN_G_MSK            (uint8_t)0x40
#define CTRL7_G_G_HM_MODE_POS          (uint8_t)7
#define CTRL7_G_G_HM_MODE_MSK          (uint8_t)0x80

/* CTRL8_C Msk Pos */
#define CTRL8_XL_LOW_PASS_6D_POS       (uint8_t)0
#define CTRL8_XL_LOW_PASS_6D_MSK       (uint8_t)0x01
#define CTRL8_XL_HP_SLOPE_X_L_EN_POS   (uint8_t)2
#define CTRL8_XL_HP_SLOPE_X_L_EN_MSK   (uint8_t)0x04
#define CTRL8_XL_INPUT_COMBO_SIET_POS  (uint8_t)3
#define CTRL8_XL_INPUT_COMBO_SIET_MSK  (uint8_t)0x08
#define CTRL8_XL_HP_REF_MODE_POS       (uint8_t)4
#define CTRL8_XL_HP_REF_MODE_MSK       (uint8_t)0x80
#define CTRL8_XL_HPCF_XL_POS           (uint8_t)5
#define CTRL8_XL_HPCF_XL_MSK           (uint8_t)0x60
#define CTRL8_XL_LPF2_XL_EN_POS        (uint8_t)7
#define CTRL8_XL_LPF2_XL_EN_MSK        (uint8_t)0x80

/* CTRL9_XL Msk Pos */
#define CTRL9_XL_SOFT_EN_POS           (uint8_t)3
#define CTRL9_XL_SOFT_EN_MSK           (uint8_t)0x04
#define CTRL9_XL_DEN_XL_G_POS          (uint8_t)5
#define CTRL9_XL_DEN_XL_G_MSK          (uint8_t)0x10
#define CTRL9_XL_DENXYZ_POS            (uint8_t)5
#define CTRL9_XL_DENXYZ_MSK            (uint8_t)0xE0

/* CTRL10_C Msk Pos */
#define CTRL10_C_SIGN_MOTION_EN_POS    (uint8_t)0
#define CTRL10_C_SIGN_MOTION_EN_MSK    (uint8_t)0x01
#define CTRL10_C_PEDO_RST_STEP_POS     (uint8_t)1
#define CTRL10_C_PEDO_RST_STEP_MSK     (uint8_t)0x02
#define CTRL10_C_FUNC_EN_POS           (uint8_t)2
#define CTRL10_C_FUNC_EN_MSK           (uint8_t)0x04
#define CTRL10_C_TILT_EN_POS           (uint8_t)3
#define CTRL10_C_TILT_EN_MSK           (uint8_t)0x08
#define CTRL10_C_PEDO_EN_POS           (uint8_t)4
#define CTRL10_C_PEDO_EN_MSK           (uint8_t)0x10
#define CTRL10_C_TIMER_EN_POS          (uint8_t)5
#define CTRL10_C_TIMER_EN_MSK          (uint8_t)0x20
#define CTRL10_C_WRIST_TILT_EN_POS     (uint8_t)7
#define CTRL10_C_WRIST_TILT_EN_MSK     (uint8_t)0x80

/* MASTER_CONFIG Msk Pos */
#define MASTER_CFG_MASTER_ON_POS             (uint8_t)0
#define MASTER_CFG_MASTER_ON_MSK             (uint8_t)0x01
#define MASTER_CFG_IRON_EN_POS               (uint8_t)1
#define MASTER_CFG_IRON_EN_MSK               (uint8_t)0x02
#define MASTER_CFG_PASS_THROUGH_MODE_POS     (uint8_t)2
#define MASTER_CFG_PASS_THROUGH_MODE_MSK     (uint8_t)0x04
#define MASTER_CFG_PULL_UP_EN_POS            (uint8_t)3
#define MASTER_CFG_PULL_UP_EN_MSK            (uint8_t)0x08
#define MASTER_CFG_START_CONFIG_POS          (uint8_t)4
#define MASTER_CFG_START_CONFIG_MSK          (uint8_t)0x10
#define MASTER_CFG_DATA_VALID_SEL_FIFO_POS   (uint8_t)6
#define MASTER_CFG_DATA_VALID_SEL_FIFO_MSK   (uint8_t)0x40
#define MASTER_CFG_DRDY_ON_INT1_POS          (uint8_t)7
#define MASTER_CFG_DRDY_ON_INT1_MSK          (uint8_t)0x80

/* WAKE_UP_SRC Msk Pos */
#define WAKE_UP_SRC_XYZ_WU_POS            (uint8_t)0
#define WAKE_UP_SRC_XYZ_WU_MSK            (uint8_t)0x07
#define WAKE_UP_SRC_WU_IA_POS             (uint8_t)3
#define WAKE_UP_SRC_WU_IA_MSK             (uint8_t)0x08
#define WAKE_UP_SRC_SLEEP_STATE_IA_POS    (uint8_t)4
#define WAKE_UP_SRC_SLEEP_STATE_IA_MSK    (uint8_t)0x10

/* TAP_SRC Msk Pos */
#define TAP_SRC_XYZ_TAP_POS               (uint8_t)0
#define TAP_SRC_XYZ_TAP_MSK               (uint8_t)0x07
#define TAP_SRC_TAP_SIGN_POS              (uint8_t)3
#define TAP_SRC_TAP_SIGN_MSK              (uint8_t)0x08
#define TAP_SRC_DOUBLE_TAP_POS            (uint8_t)4
#define TAP_SRC_DOUBLE_TAP_MSK            (uint8_t)0x10
#define TAP_SRC_SINGLE_TAP_POS            (uint8_t)5
#define TAP_SRC_SINGLE_TAP_MSK            (uint8_t)0x20
#define TAP_SRC_TAP_IA_POS                (uint8_t)6
#define TAP_SRC_TAP_IA_MSK                (uint8_t)0x40

/* D6D_SRC Msk Pos */
#define D6D_SRC_XL_POS                    (uint8_t)0
#define D6D_SRC_XL_MSK                    (uint8_t)0x01
#define D6D_SRC_XH_POS                    (uint8_t)1
#define D6D_SRC_XH_MSK                    (uint8_t)0x02
#define D6D_SRC_YL_POS                    (uint8_t)2
#define D6D_SRC_YL_MSK                    (uint8_t)0x04
#define D6D_SRC_YH_POS                    (uint8_t)3
#define D6D_SRC_YH_MSK                    (uint8_t)0x08
#define D6D_SRC_ZL_POS                    (uint8_t)4
#define D6D_SRC_ZL_MSK                    (uint8_t)0x10
#define D6D_SRC_ZH_POS                    (uint8_t)5
#define D6D_SRC_ZH_MSK                    (uint8_t)0x20
#define D6D_SRC_D6D_IA_POS                (uint8_t)6
#define D6D_SRC_D6D_IA_MSK                (uint8_t)0x40
#define D6D_SRC_D6D_DRDY_POS              (uint8_t)7
#define D6D_SRC_D6D_DRDY_MSK              (uint8_t)0x80

/* STATUS_REG Msk Pos */
#define STATUS_REG_XLDA_POS               (uint8_t)0
#define STATUS_REG_XLDA_MSK               (uint8_t)0x01
#define STATUS_REG_GDA_POS                (uint8_t)1
#define STATUS_REG_GDA_MSK                (uint8_t)0x02
#define STATUS_REG_TDA_POS                (uint8_t)2
#define STATUS_REG_TDA_MSK                (uint8_t)0x04

/* FUNC_SRC1 Msk Pos */
#define FUNC_SRC1_SENSOR_HUB_END_OP_POS   (uint8_t)0
#define FUNC_SRC1_SENSOR_HUB_END_OP_MSK   (uint8_t)0x01
#define FUNC_SRC1_SI_EN_OP_POS            (uint8_t)1
#define FUNC_SRC1_SI_EN_OP_MSK            (uint8_t)0x02
#define FUNC_SRC1_HI_FAIL_POS             (uint8_t)2
#define FUNC_SRC1_HI_FAIL_MSK             (uint8_t)0x04
#define FUNC_SRC1_STEP_OVERFLOW_POS       (uint8_t)3
#define FUNC_SRC1_STEP_OVERFLOW_MSK       (uint8_t)0x08
#define FUNC_SRC1_STEP_DETECTED_POS       (uint8_t)4
#define FUNC_SRC1_STEP_DETECTED_MSK       (uint8_t)0x10
#define FUNC_SRC1_TILT_IA_POS             (uint8_t)5
#define FUNC_SRC1_TILT_IA_MSK             (uint8_t)0x20
#define FUNC_SRC1_SIGN_MOTION_IA_POS      (uint8_t)6
#define FUNC_SRC1_SIGN_MOTION_IA_MSK      (uint8_t)0x40
#define FUNC_SRC1_STEP_COUNT_DELTA_IA_POS (uint8_t)7
#define FUNC_SRC1_STEP_COUNT_DELTA_IA_MSK (uint8_t)0x80

/* FUNC_SRC2 Msk Pos */
#define FUNC_SRC2_WRIST_TILT_IA_POS    (uint8_t)0
#define FUNC_SRC2_WRIST_TILT_IA_MSK    (uint8_t)0x01

/* TAP_CFG Msk Pos */
#define TAP_CFG_LIR_POS                (uint8_t)0
#define TAP_CFG_LIR_MSK                (uint8_t)0x01
#define TAP_CFG_TAPXYZ_POS             (uint8_t)1
#define TAP_CFG_TAPXYZ_MSK             (uint8_t)0x0E
#define TAP_CFG_SLOPE_FDS_POS          (uint8_t)4
#define TAP_CFG_SLOPE_FDS_MSK          (uint8_t)0x08
#define TAP_CFG_INACT_EN_POS           (uint8_t)5
#define TAP_CFG_INACT_EN_MSK           (uint8_t)0x60
#define TAP_CFG_INTERRUPTS_EN_POS      (uint8_t)7
#define TAP_CFG_INTERRUPTS_EN_MSK      (uint8_t)0x80

/* TAP_THS_6D Msk Pos */
#define TAP_THS_6D_TAP_THS_POS         (uint8_t)0
#define TAP_THS_6D_TAP_THS_MSK         (uint8_t)0x1F
#define TAP_THS_6D_SIXD_THS_POS        (uint8_t)5
#define TAP_THS_6D_SIXD_THS_MSK        (uint8_t)0x60
#define TAP_THS_6D_D4D_EN_POS          (uint8_t)7
#define TAP_THS_6D_D4D_EN_MSK          (uint8_t)0x80

/* INT_DUR2 Msk Pos */
#define INT_DUR2_SHOCK_POS             (uint8_t)0
#define INT_DUR2_SHOCK_MSK             (uint8_t)0x03
#define INT_DUR2_QUIET_POS             (uint8_t)2
#define INT_DUR2_QUIET_MSK             (uint8_t)0x0C
#define INT_DUR2_DUR_POS               (uint8_t)4
#define INT_DUR2_DUR_MSK               (uint8_t)0xF0

/* WAKE_UP_THS Msk Pos */
#define WAKE_UP_THS_WK_THS_POS            (uint8_t)0
#define WAKE_UP_THS_WK_THS_MSK            (uint8_t)0x3F
#define WAKE_UP_THS_SINGLE_DOUBLE_TAP_POS (uint8_t)7
#define WAKE_UP_THS_SINGLE_DOUBLE_TAP_MSK (uint8_t)0x80

/* WAKE_UP_DUR Msk Pos */
#define WAKE_UP_DUR_SLEEP_DUR_POS      (uint8_t)0
#define WAKE_UP_DUR_SLEEP_DUR_MSK      (uint8_t)0x0F
#define WAKE_UP_DUR_TIMER_HR_POS       (uint8_t)4
#define WAKE_UP_DUR_TIMER_HR_MSK       (uint8_t)0x10
#define WAKE_UP_DUR_WAKE_DUR_POS       (uint8_t)5
#define WAKE_UP_DUR_WAKE_DUR_MSK       (uint8_t)0x60
#define WAKE_UP_DUR_FF_DUR_POS         (uint8_t)7
#define WAKE_UP_DUR_FF_DUR_MSK         (uint8_t)0x80

/* FREE_FALL Msk Pos */
#define FREE_FALL_FF_THS_POS           (uint8_t)0
#define FREE_FALL_FF_THS_MSK           (uint8_t)0x07
#define FREE_FALL_FF_DUR_POS           (uint8_t)3
#define FREE_FALL_FF_DUR_MSK           (uint8_t)0xF8

/* MD1_CFG Msk Pos */
#define MD1_CFG_INT1_TIMER_POS         (uint8_t)0
#define MD1_CFG_INT1_TIMER_MSK         (uint8_t)0x01
#define MD1_CFG_INT1_TILT_POS          (uint8_t)1
#define MD1_CFG_INT1_TILT_MSK          (uint8_t)0x02
#define MD1_CFG_INT1_6D_POS            (uint8_t)2
#define MD1_CFG_INT1_6D_MSK            (uint8_t)0x04
#define MD1_CFG_INT1_DOUBLE_TAP_POS    (uint8_t)3
#define MD1_CFG_INT1_DOUBLE_TAP_MSK    (uint8_t)0x08
#define MD1_CFG_INT1_FF_POS            (uint8_t)4
#define MD1_CFG_INT1_FF_MSK            (uint8_t)0x10
#define MD1_CFG_INT1_WU_POS            (uint8_t)5
#define MD1_CFG_INT1_WU_MSK            (uint8_t)0x20
#define MD1_CFG_INT1_SINGLE_TAP_POS    (uint8_t)6
#define MD1_CFG_INT1_SINGLE_TAP_MSK    (uint8_t)0x40
#define MD1_CFG_INT1_INACT_STATE_POS   (uint8_t)7
#define MD1_CFG_INT1_INACT_STATE_MSK   (uint8_t)0x80

/* MD2_CFG Msk Pos */
#define MD2_CFG_INT2_IRON_POS          (uint8_t)0
#define MD2_CFG_INT2_IRON_MSK          (uint8_t)0x01
#define MD2_CFG_INT2_TILT_POS          (uint8_t)1
#define MD2_CFG_INT2_TILT_MSK          (uint8_t)0x02
#define MD2_CFG_INT2_6D_POS            (uint8_t)2
#define MD2_CFG_INT2_6D_MSK            (uint8_t)0x04
#define MD2_CFG_INT2_DOUBLE_TAP_POS    (uint8_t)3
#define MD2_CFG_INT2_DOUBLE_TAP_MSK    (uint8_t)0x08
#define MD2_CFG_INT2_FF_POS            (uint8_t)4
#define MD2_CFG_INT2_FF_MSK            (uint8_t)0x10
#define MD2_CFG_INT2_WU_POS            (uint8_t)5
#define MD2_CFG_INT2_WU_MSK            (uint8_t)0x20
#define MD2_CFG_INT2_SINGLE_TAP_POS    (uint8_t)6
#define MD2_CFG_INT2_SINGLE_TAP_MSK    (uint8_t)0x40
#define MD2_CFG_INT2_INACT_STATE_POS   (uint8_t)7
#define MD2_CFG_INT2_INACT_STATE_MSK   (uint8_t)0x80

#define REBOOT_TIMEOUT                 (uint32_t)20

#define EXIT_ERROR_CHECK(error)  do {     \
      if((error != LSM6DSL_ERROR_NONE))   \
      {                                   \
         return error;                    \
      }                                   \
   }while(0);

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/

/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static e_LSM6DSL_Error_t eReadRegister(uint8_t p_u8Register, uint8_t * p_pu8Value, uint8_t p_u8RegNumber);
static e_LSM6DSL_Error_t eWriteRegister(uint8_t p_u8Register, uint8_t p_u8Data);
static e_LSM6DSL_Error_t eWriteBitsReg(uint8_t p_u8Register, uint8_t p_u8Value, uint8_t p_u8Pos, uint8_t p_u8Mask);

//static void vDirectAccessData(void);
//static void vLowPowerModeAccel(void);
//static void vHighPerfModeAccel(void);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
static s_LSM6DSL_Context_t g_sLSM6DSLContext;
static uint8_t g_u8LSM6CommFailure = 0u;
static uint8_t g_u8LSM6Initialized = 0u;

s_LSM6DSL_3AxisRawData_t g_sRawDataAccel;
s_LSM6DSL_3AxisRawData_t g_sRawDataGyro;

struct {
   e_LSM6DSL_ODR_t eODR;
   e_LSM6DSL_AccelFullScale_t eFS;
   e_LSM6DSL_Mode_t eMode;
}g_sAccelCfg;

struct {
   e_LSM6DSL_ODR_t eODR;
   e_LSM6DSL_GyroFullScale_t eFS;
   e_LSM6DSL_Mode_t eMode;
}g_sGyroCfg;

/****************************************************************************************
 * Public functions
 ****************************************************************************************/
/**@brief  Initialize the LSM6DSL module.
 * @param[in]  Context.
 * @return Error Code.
 */
e_LSM6DSL_Error_t eLSM6DSL_ContextSet(s_LSM6DSL_Context_t p_sContext)
{
   e_LSM6DSL_Error_t l_eErrCode = LSM6DSL_ERROR_PARAM;
   uint8_t l_u8Data = 0u;

   if(   (p_sContext.eInterface == LSM6DSL_ITF_I2C)
      && (p_sContext.sI2CCfg.fp_u32I2C_Write != NULL)
      && (p_sContext.sI2CCfg.fp_u32I2C_Read != NULL)
      && (p_sContext.fp_vDelay_ms != NULL)
      && (  (p_sContext.sI2CCfg.eI2CAddress == LSM6DSL_SLAVE_ADDR_VCC)
         || (p_sContext.sI2CCfg.eI2CAddress == LSM6DSL_SLAVE_ADDR_GND) ) )
   {
      /* Set new context */
      g_sLSM6DSLContext.sI2CCfg.eI2CAddress = p_sContext.sI2CCfg.eI2CAddress;
      g_sLSM6DSLContext.sI2CCfg.fp_u32I2C_Write = p_sContext.sI2CCfg.fp_u32I2C_Write;
      g_sLSM6DSLContext.sI2CCfg.fp_u32I2C_Read = p_sContext.sI2CCfg.fp_u32I2C_Read;
      g_sLSM6DSLContext.fp_vDelay_ms = p_sContext.fp_vDelay_ms;

      /* Check Sensor ID */
      if(eReadRegister(WHO_AM_I_REG, &l_u8Data, 1u) == LSM6DSL_ERROR_NONE)
      {
         if(l_u8Data == LSM6DSL_WHO_I_AM_ID)
         {
//            vHighPerfModeAccel();
//            vLSM6DSL_GyroEnable(0u);
            /* Reboot Sensor to start from 0 */
//            vLSM6DSL_Reboot();

            /* Low power mode */
//            vLowPowerModeAccel();
            /* Configure ODR and Scale */
//            eLSM6DSL_AccelCfgSet(g_sLSM6DSLContext.eAccelODR, g_sLSM6DSLContext.eAccelFullScale);
//            eLSM6DSL_GyroCfgSet(g_sLSM6DSLContext.eGyroODR, g_sLSM6DSLContext.eGyroFullScale);

            //vLSM6DSL_AccelEnable(0u);
            /* Set continuous mode and direct access */
//            vDirectAccessData();

//            uint16_t l_u16Temp = 0u;
//            vLSM6DSL_TemperatureRead(&l_u16Temp);

            g_u8LSM6Initialized = 1u;
            l_eErrCode = LSM6DSL_ERROR_NONE;
         }
         else
         {
            l_eErrCode = LSM6DSL_ERROR_NOT_FOUND;
         }
      }
   }
   else if( (p_sContext.eInterface == LSM6DSL_ITF_SPI)
         && (p_sContext.sSPICfg.fp_vPinSet != NULL)
         && (p_sContext.sSPICfg.fp_vPinClear != NULL)
         && (p_sContext.sSPICfg.fp_u32SPI_Transfer != NULL)
         && (p_sContext.fp_vDelay_ms != NULL) )
   {
      /* Set new context */
      g_sLSM6DSLContext.sSPICfg.fp_vPinSet = p_sContext.sSPICfg.fp_vPinSet;
      g_sLSM6DSLContext.sSPICfg.fp_vPinClear = p_sContext.sSPICfg.fp_vPinClear;
      g_sLSM6DSLContext.sSPICfg.fp_u32SPI_Transfer = p_sContext.sSPICfg.fp_u32SPI_Transfer;
      g_sLSM6DSLContext.sSPICfg.u32ChipSelect = p_sContext.sSPICfg.u32ChipSelect;
      g_sLSM6DSLContext.fp_vDelay_ms = p_sContext.fp_vDelay_ms;

      g_u8LSM6Initialized = 1u;
      l_eErrCode = LSM6DSL_ERROR_NONE;
   }
   else
   {
      g_u8LSM6Initialized = 0u;
      l_eErrCode = LSM6DSL_ERROR_PARAM;
   }

   /* Set Default value */
   g_sAccelCfg.eODR = LSM6DSL_ODR_POWER_DOWN;
   g_sGyroCfg.eODR = LSM6DSL_ODR_POWER_DOWN;
   g_sAccelCfg.eFS = LSM6DSL_ACCEL_RANGE_2G;
   g_sGyroCfg.eFS = LSM6DSL_GYRO_RANGE_250DPS;

   return l_eErrCode;
}

/**@brief Function to get Chip ID sensor.
 * @param[out] p_pu8WhoAmI
 * @return Error code
 */
e_LSM6DSL_Error_t eLSM6DSL_WhoAmIGet(uint8_t * p_pu8WhoAmI)
{
   e_LSM6DSL_Error_t l_eErrCode = LSM6DSL_ERROR_CONTEXT;
   
   if(p_pu8WhoAmI != NULL)
   {
      l_eErrCode = eReadRegister(WHO_AM_I_REG, p_pu8WhoAmI, 1u);
   }
   else
   {
      l_eErrCode = LSM6DSL_ERROR_PARAM;
   }
   
   return l_eErrCode;
}

/**@brief Function to Soft reset sensor.
 * @return Error code
 */
e_LSM6DSL_Error_t eLSM6DSL_SoftReset(void)
{
   e_LSM6DSL_Error_t l_eErrCode = LSM6DSL_ERROR_CONTEXT;

   l_eErrCode = eWriteBitsReg(CTRL3_C_REG, 0x01, CTRL3_C_SW_RESET_POS, CTRL3_C_SW_RESET_MSK);
   EXIT_ERROR_CHECK(l_eErrCode);

   (*g_sLSM6DSLContext.fp_vDelay_ms)(REBOOT_TIMEOUT);

   return l_eErrCode;
}

/**@brief Set Block Data Update.
 * @param[in]  p_u8Enable : 1 to active else 0.
 * @return Error Code.
 */
e_LSM6DSL_Error_t eLSM6DSL_BlockDataUpdateSet(uint8_t p_u8Enable)
{
   return eWriteBitsReg(CTRL3_C_REG, (uint8_t)p_u8Enable, CTRL3_C_BDU_POS, CTRL3_C_BDU_MSK);
}
/**@brief Function to Configure Accelerometer.
 * @param[in] p_eODR
 * @param[in] p_eFullScale
 * @param[in] p_eMode
 * @return Error code
 */
e_LSM6DSL_Error_t eLSM6DSL_AccelCfgSet(e_LSM6DSL_ODR_t p_eODR, e_LSM6DSL_AccelFullScale_t p_eFullScale, e_LSM6DSL_Mode_t p_eMode)
{
   e_LSM6DSL_Error_t l_eErrCode = LSM6DSL_ERROR_CONTEXT;

   if(g_u8LSM6Initialized == 1u)
   {
      /* Write FS */
      l_eErrCode = eWriteBitsReg(CTRL1_XL_REG, p_eFullScale, CTRL1_XL_FS_POS, CTRL1_XL_FS_MSK);
      EXIT_ERROR_CHECK(l_eErrCode);
      g_sAccelCfg.eFS = p_eFullScale;
      /* Write ODR */
      l_eErrCode = eWriteBitsReg(CTRL1_XL_REG, p_eODR, CTRL1_XL_ODR_POS, CTRL1_XL_ODR_MSK);
      EXIT_ERROR_CHECK(l_eErrCode);
      g_sAccelCfg.eODR = p_eODR;
      /* Write Accel Mode */
      l_eErrCode = eWriteBitsReg(CTRL6_C_REG, p_eMode, CTRL6_C_XL_HM_MODE_POS, CTRL6_C_XL_HM_MODE_MSK);
      EXIT_ERROR_CHECK(l_eErrCode);
      g_sGyroCfg.eMode = p_eMode;
   }

   return l_eErrCode;
}
/**@brief Function to Configure Gyroscope.
 * @param[in] p_eODR
 * @param[in] p_eFullScale
 * @param[in] p_eMode
 * @return Error code
 */
 e_LSM6DSL_Error_t eLSM6DSL_GyroCfgSet(e_LSM6DSL_ODR_t p_eODR, e_LSM6DSL_GyroFullScale_t p_eFullScale, e_LSM6DSL_Mode_t p_eMode)
{
   e_LSM6DSL_Error_t l_eErrCode = LSM6DSL_ERROR_CONTEXT;

   if(g_u8LSM6Initialized == 1u)
   {
      /* Write FS */
      l_eErrCode = eWriteBitsReg(CTRL2_G_REG, p_eFullScale, CTRL2_G_FS_POS, CTRL2_G_FS_MSK);
      EXIT_ERROR_CHECK(l_eErrCode);
      g_sGyroCfg.eFS = p_eFullScale;
      /* Write ODR */
      l_eErrCode = eWriteBitsReg(CTRL2_G_REG, p_eODR, CTRL2_G_ODR_POS, CTRL2_G_ODR_MSK);
      EXIT_ERROR_CHECK(l_eErrCode);
      g_sGyroCfg.eODR = p_eODR;
      /* Write Gyro Mode */
      l_eErrCode = eWriteBitsReg(CTRL7_G_REG, p_eMode, CTRL7_G_HPM_G_POS, CTRL7_G_HPM_G_MSK);
      EXIT_ERROR_CHECK(l_eErrCode);
      g_sGyroCfg.eMode = p_eMode;
   }

   return l_eErrCode;
}
/**@brief Function to Read Accelerometer sensor data.
 * @return Error code
 */
e_LSM6DSL_Error_t eLSM6DSL_AccelRead(void)
{
   uint8_t l_au8Data[6u] = { 0u };
   uint8_t l_u8DataRdy = 0u;
   uint8_t l_u8Retry = 3u;
   e_LSM6DSL_Error_t l_eErrCode = LSM6DSL_ERROR_CONTEXT;

   if(g_u8LSM6Initialized == 1u)
   {
      l_eErrCode = eReadRegister(STATUS_REG, &l_au8Data[0u], 1u);
      EXIT_ERROR_CHECK(l_eErrCode);
      l_u8DataRdy = (l_au8Data[0u] & 0x01);
      while((l_u8DataRdy == 0u) && (l_u8Retry > 0u))
      {
         l_eErrCode = eReadRegister(STATUS_REG, &l_au8Data[0u], 1u);
         EXIT_ERROR_CHECK(l_eErrCode);
         l_u8DataRdy = (l_au8Data[0u] & 0x01);
         (*g_sLSM6DSLContext.fp_vDelay_ms)(10u);
         l_u8Retry--;
      }

      l_eErrCode = eReadRegister(OUTX_L_XL_REG, l_au8Data, 6u);
      EXIT_ERROR_CHECK(l_eErrCode);

      g_sRawDataAccel.RawUnsigned.u16DataX = (uint16_t)l_au8Data[0u] + ((uint16_t)l_au8Data[1u] << 8u);
      g_sRawDataAccel.RawUnsigned.u16DataY = (uint16_t)l_au8Data[2u] + ((uint16_t)l_au8Data[3u] << 8u);
      g_sRawDataAccel.RawUnsigned.u16DataZ = (uint16_t)l_au8Data[4u] + ((uint16_t)l_au8Data[5u] << 8u);
   }

   return l_eErrCode;
}
/**@brief Function to Get Accelerometer computed data according to Full Scale parameter.
 * @param[out] p_ps16X
 * @param[out] p_ps16Y
 * @param[out] p_ps16Z
 * @return Error code
 */
e_LSM6DSL_Error_t eLSM6DSL_AccelGet(int16_t * p_ps16X, int16_t * p_ps16Y, int16_t * p_ps16Z)
{
   e_LSM6DSL_Error_t l_eErrCode = LSM6DSL_ERROR_CONTEXT;

   static const float l_afAccelSensitivity[4u] = {
      0.061f, 0.122f, 0.244f, 0.488f  };

   int32_t l_s32DataX = (int32_t)g_sRawDataAccel.RawSigned.s16DataX;
   int32_t l_s32DataY = (int32_t)g_sRawDataAccel.RawSigned.s16DataY;
   int32_t l_s32DataZ = (int32_t)g_sRawDataAccel.RawSigned.s16DataZ;

   uint8_t l_u8Sensitivity = 0u;
   float l_fSensitivity = 1.0f;

   if(g_u8LSM6Initialized == 1u)
   {
      if( (p_ps16X != NULL) && (p_ps16Y != NULL) && (p_ps16Z != NULL) )
      {
         switch(g_sAccelCfg.eFS)
         {
            case LSM6DSL_ACCEL_RANGE_4G:
               l_u8Sensitivity = 1u;
               break;
            case LSM6DSL_ACCEL_RANGE_8G:
               l_u8Sensitivity = 2u;
               break;
            case LSM6DSL_ACCEL_RANGE_16G:
               l_u8Sensitivity = 3u;
               break;
            case LSM6DSL_ACCEL_RANGE_2G:
            default:
               l_u8Sensitivity = 0u;
               break;
         }

         l_fSensitivity = l_afAccelSensitivity[l_u8Sensitivity];

         (*p_ps16X) = (int16_t)((float)l_s32DataX * l_fSensitivity);
         (*p_ps16Y) = (int16_t)((float)l_s32DataY * l_fSensitivity);
         (*p_ps16Z) = (int16_t)((float)l_s32DataZ * l_fSensitivity);

         l_eErrCode = LSM6DSL_ERROR_NONE;
      }
      else
      {
         l_eErrCode = LSM6DSL_ERROR_PARAM;
      }
   }

   return l_eErrCode;

}
/**@brief Function to Get Accelerometer raw data.
 * @param[out] p_pu16X
 * @param[out] p_pu16Y
 * @param[out] p_pu16Z
 * @return Error code
 */
 e_LSM6DSL_Error_t eLSM6DSL_AccelRawGet(uint16_t * p_pu16X, uint16_t * p_pu16Y, uint16_t * p_pu16Z)
{
   e_LSM6DSL_Error_t l_eErrCode = LSM6DSL_ERROR_CONTEXT;

   if(g_u8LSM6Initialized == 1u)
   {
      if( (p_pu16X != NULL) && (p_pu16Y != NULL) && (p_pu16Z != NULL) )
      {
         (*p_pu16X) = g_sRawDataAccel.RawUnsigned.u16DataX;
         (*p_pu16Y) = g_sRawDataAccel.RawUnsigned.u16DataY;
         (*p_pu16Z) = g_sRawDataAccel.RawUnsigned.u16DataZ;

         l_eErrCode = LSM6DSL_ERROR_NONE;
      }
      else
      {
         l_eErrCode = LSM6DSL_ERROR_PARAM;
      }

   }

   return l_eErrCode;
}
/**@brief Function to Read Gyroscope sensor data.
 * @return Error code
 */
 e_LSM6DSL_Error_t eLSM6DSL_GyroRead(void)
{
   uint8_t l_au8Data[6u] = { 0u };
   uint8_t l_u8DataRdy = 0u;
   uint8_t l_u8Retry = 3u;
   e_LSM6DSL_Error_t l_eErrCode = LSM6DSL_ERROR_CONTEXT;

   if(g_u8LSM6Initialized == 1u)
   {
      l_eErrCode = eReadRegister(STATUS_REG, &l_au8Data[0u], 1u);
      EXIT_ERROR_CHECK(l_eErrCode);
      l_u8DataRdy = (l_au8Data[0u] & 0x02);
      while((l_u8DataRdy == 0u) && (l_u8Retry > 0u))
      {
         l_eErrCode = eReadRegister(STATUS_REG, &l_au8Data[0u], 1u);
         EXIT_ERROR_CHECK(l_eErrCode);
         l_u8DataRdy = (l_au8Data[0u] & 0x02);
         (*g_sLSM6DSLContext.fp_vDelay_ms)(10u);
         l_u8Retry--;
      }

      l_eErrCode = eReadRegister(OUTX_L_G_REG, l_au8Data, 6u);
      EXIT_ERROR_CHECK(l_eErrCode);

      g_sRawDataGyro.RawUnsigned.u16DataX = (uint16_t)l_au8Data[0u] + ((uint16_t)l_au8Data[1u] << 8u);
      g_sRawDataGyro.RawUnsigned.u16DataY = (uint16_t)l_au8Data[2u] + ((uint16_t)l_au8Data[3u] << 8u);
      g_sRawDataGyro.RawUnsigned.u16DataZ = (uint16_t)l_au8Data[4u] + ((uint16_t)l_au8Data[5u] << 8u);
   }

   return l_eErrCode;
}
/**@brief Function to Get Gyroscope computed data according to Full Scale parameter.
 * @param[out] p_ps16X
 * @param[out] p_ps16Y
 * @param[out] p_ps16Z
 * @return Error code
 */
e_LSM6DSL_Error_t eLSM6DSL_GyroGet(int16_t * p_ps16X, int16_t * p_ps16Y, int16_t * p_ps16Z)
{
   e_LSM6DSL_Error_t l_eErrCode = LSM6DSL_ERROR_CONTEXT;
   static const float l_afGyroSensitivity[5u] = {
      0.004375f, 0.00875f, 0.0175f, 0.0350f, 0.0700f  };

   int32_t l_s32DataX = (int32_t)g_sRawDataGyro.RawSigned.s16DataX;
   int32_t l_s32DataY = (int32_t)g_sRawDataGyro.RawSigned.s16DataY;
   int32_t l_s32DataZ = (int32_t)g_sRawDataGyro.RawSigned.s16DataZ;

   uint8_t l_u8Sensitivity = 0u;
   float l_fSensitivity = 1.0f;

   if(g_u8LSM6Initialized == 1u)
   {
      if( (p_ps16X != NULL) && (p_ps16Y != NULL) && (p_ps16Z != NULL) )
      {
         switch(g_sGyroCfg.eFS)
         {
            case LSM6DSL_GYRO_RANGE_125DPS:
               l_u8Sensitivity = 0u;
               break;
            case LSM6DSL_GYRO_RANGE_500DPS:
               l_u8Sensitivity = 2u;
               break;
            case LSM6DSL_GYRO_RANGE_1000DPS:
               l_u8Sensitivity = 3u;
               break;
            case LSM6DSL_GYRO_RANGE_2000DPS:
               l_u8Sensitivity = 4u;
               break;
            case LSM6DSL_GYRO_RANGE_250DPS:
            default:
               l_u8Sensitivity = 1u;
               break;
         }
         l_fSensitivity = l_afGyroSensitivity[l_u8Sensitivity];

         (*p_ps16X) = (int16_t)((float)l_s32DataX * l_fSensitivity);
         (*p_ps16Y) = (int16_t)((float)l_s32DataY * l_fSensitivity);
         (*p_ps16Z) = (int16_t)((float)l_s32DataZ * l_fSensitivity);

         l_eErrCode = LSM6DSL_ERROR_NONE;
      }
      else
      {
         l_eErrCode = LSM6DSL_ERROR_PARAM;
      }
   }

   return l_eErrCode;
}
/**@brief Function to Get Gyroscope raw data.
 * @param[out] p_pu16X
 * @param[out] p_pu16Y
 * @param[out] p_pu16Z
 * @return Error code
 */
 e_LSM6DSL_Error_t eLSM6DSL_GyroRawGet(uint16_t * p_pu16X, uint16_t * p_pu16Y, uint16_t * p_pu16Z)
{
   e_LSM6DSL_Error_t l_eErrCode = LSM6DSL_ERROR_CONTEXT;

   if(g_u8LSM6Initialized == 1u)
   {
      if( (p_pu16X != NULL) && (p_pu16Y != NULL) && (p_pu16Z != NULL) )
      {
         (*p_pu16X) = g_sRawDataGyro.RawUnsigned.u16DataX;
         (*p_pu16Y) = g_sRawDataGyro.RawUnsigned.u16DataY;
         (*p_pu16Z) = g_sRawDataGyro.RawUnsigned.u16DataZ;

         l_eErrCode = LSM6DSL_ERROR_NONE;
      }
      else
      {
         l_eErrCode = LSM6DSL_ERROR_PARAM;
      }

   }

   return l_eErrCode;
}
/**@brief Function to check if device is available.
 * @return 1 if sensor is available, else 0
 */
uint8_t u8LSM6DSL_IsAvailable(void)
{
   return ((g_u8LSM6Initialized == 1u) && (g_u8LSM6CommFailure == 0u))?1u:0u;
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/
/**@brief Function to read on registers.
 * @param[in] p_u8Register
 * @param[out] p_pu8Value
 * @param[in] p_u8RegNumber
 * @return Error code
 */
static e_LSM6DSL_Error_t eReadRegister(uint8_t p_u8Register, uint8_t * p_pu8Value, uint8_t p_u8RegNumber)
{
   e_LSM6DSL_Error_t l_eErrCode = LSM6DSL_ERROR_COMM;

   if(   (p_pu8Value != NULL)
      || ((p_u8Register + p_u8RegNumber) <= LAST_REGISTER) )
   {
      if(g_sLSM6DSLContext.eInterface == LSM6DSL_ITF_I2C)
      {
         if((*g_sLSM6DSLContext.sI2CCfg.fp_u32I2C_Read)((uint8_t)g_sLSM6DSLContext.sI2CCfg.eI2CAddress, &p_u8Register, 1u, p_pu8Value, p_u8RegNumber) == 0u)
         {
            l_eErrCode = LSM6DSL_ERROR_NONE;
            g_u8LSM6CommFailure = 0u;
         }
         else
         {
            g_u8LSM6CommFailure = 1u;
         }
      }
      else if(g_sLSM6DSLContext.eInterface == LSM6DSL_ITF_SPI)
      {
         (*g_sLSM6DSLContext.sSPICfg.fp_vPinClear)(g_sLSM6DSLContext.sSPICfg.u32ChipSelect);

         if((*g_sLSM6DSLContext.sSPICfg.fp_u32SPI_Transfer)(&p_u8Register, 1u, p_pu8Value, p_u8RegNumber) == 0u)
         {
            l_eErrCode = LSM6DSL_ERROR_NONE;
            g_u8LSM6CommFailure = 0u;
         }
         else
         {
            g_u8LSM6CommFailure = 1u;
         }

         (*g_sLSM6DSLContext.sSPICfg.fp_vPinSet)(g_sLSM6DSLContext.sSPICfg.u32ChipSelect);
      }
      else
      {
         l_eErrCode = LSM6DSL_ERROR_CONTEXT;
      }
   }
   else
   {
      l_eErrCode = LSM6DSL_ERROR_PARAM;
   }

   return l_eErrCode;
}
/**@brief Function to write on register.
 * @param[in] p_u8Register
 * @param[in] p_u8Data
 * @return Error code
 */
static e_LSM6DSL_Error_t eWriteRegister(uint8_t p_u8Register, uint8_t p_u8Data)
{
   e_LSM6DSL_Error_t l_eErrCode = LSM6DSL_ERROR_COMM;
   uint8_t l_au8Data[2u] = { 0u };

   if(p_u8Register < LAST_REGISTER)
   {
      l_au8Data[0u] = p_u8Register;
      l_au8Data[1u] = p_u8Data;

      if(g_sLSM6DSLContext.eInterface == LSM6DSL_ITF_I2C)
      {
         if((*g_sLSM6DSLContext.sI2CCfg.fp_u32I2C_Write)((uint8_t)g_sLSM6DSLContext.sI2CCfg.eI2CAddress, l_au8Data, 2u) == 0u)
         {
            l_eErrCode = LSM6DSL_ERROR_NONE;
            g_u8LSM6CommFailure = 0u;
         }
         else
         {
            g_u8LSM6CommFailure = 1u;
         }
      }
      else if(g_sLSM6DSLContext.eInterface == LSM6DSL_ITF_SPI)
      {
         uint8_t l_u8Ack = 0u;
         (*g_sLSM6DSLContext.sSPICfg.fp_vPinClear)(g_sLSM6DSLContext.sSPICfg.u32ChipSelect);

         if((*g_sLSM6DSLContext.sSPICfg.fp_u32SPI_Transfer)(l_au8Data, 2u, &l_u8Ack, 1u) == 0u)
         {
            (void)l_u8Ack;
            l_eErrCode = LSM6DSL_ERROR_NONE;
            g_u8LSM6CommFailure = 0u;
         }
         else
         {
            g_u8LSM6CommFailure = 1u;
         }

         (*g_sLSM6DSLContext.sSPICfg.fp_vPinSet)(g_sLSM6DSLContext.sSPICfg.u32ChipSelect);
      }
      else
      {
         l_eErrCode = LSM6DSL_ERROR_CONTEXT;
      }
   }
   else
   {
      l_eErrCode = LSM6DSL_ERROR_PARAM;
   }

   return l_eErrCode;
}
/**@brief Function to write specific bits of one register.
 * @param[in] p_u8Register
 * @param[in] p_u8Data
 * @param[in] p_u8Pos
 * @param[in] p_u8Mask
 * @return Error code
 */
static e_LSM6DSL_Error_t eWriteBitsReg(uint8_t p_u8Register, uint8_t p_u8Value, uint8_t p_u8Pos, uint8_t p_u8Mask)
{
   uint8_t l_u8RegVal = 0u;
   e_LSM6DSL_Error_t l_eErrCode = LSM6DSL_ERROR_PARAM;

   l_eErrCode = eReadRegister(p_u8Register, &l_u8RegVal, 1u);

   if(l_eErrCode == LSM6DSL_ERROR_NONE)
   {
      l_u8RegVal &= ~p_u8Mask;

      l_u8RegVal |= ((uint8_t)p_u8Value << p_u8Pos);

      l_eErrCode = eWriteRegister(p_u8Register, l_u8RegVal);
   }

   return l_eErrCode;
}

/*
static void vDirectAccessData(void)
{
   uint8_t l_u8Reg = 0u;

   if(eReadRegister(CTRL9_XL_REG, &l_u8Reg, 1u) == LSM6DSL_ERROR_NONE)
   {
      l_u8Reg &= ~CTRL9_XL_DENXYZ_MSK;

      l_u8Reg |= (0x0F << CTRL9_XL_DENXYZ_POS);

      (void)eWriteRegister(CTRL9_XL_REG, l_u8Reg);
   }
}

static void vLowPowerModeAccel(void)
{
   uint8_t l_u8RegVal = 0u;

   if(eReadRegister(CTRL6_C_REG, &l_u8RegVal, 1u) == LSM6DSL_ERROR_NONE)
   {
      l_u8RegVal &= ~CTRL6_X_HP_MODE_MSK;

      l_u8RegVal |= (1 << CTRL6_X_HP_MODE_POS);

      (void)eWriteRegister(CTRL6_C_REG, l_u8RegVal);
   }
}

static void vHighPerfModeAccel(void)
{
   uint8_t l_u8RegVal = 0u;

   if(eReadRegister(CTRL6_C_REG, &l_u8RegVal, 1u) == LSM6DSL_ERROR_NONE)
   {
      l_u8RegVal &= ~CTRL6_X_HP_MODE_MSK;

      l_u8RegVal |= (0 << CTRL6_X_HP_MODE_POS);

      (void)eWriteRegister(CTRL6_C_REG, l_u8RegVal);
   }
}
*/
/****************************************************************************************
 * End Of File
 ****************************************************************************************/


