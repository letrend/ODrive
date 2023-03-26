/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#ifndef _DRV8311_H_
#define _DRV8311_H_

//! \file   drivers/drvic/DRV8311/src/32b/f28x/f2805x/DRV8311.h
//! \brief  Contains public interface to various functions related
//!         to the DRV8311 object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes
#include <math.h>
#include <stdbool.h>
// drivers
#include "spi.h"
#include "gpio.h"
//#include "sw/drivers/spi/src/32b/f28x/f2805x/spi.h"
//#include "sw/drivers/gpio/src/32b/f28x/f2805x/gpio.h"

// modules

// solutions


//!
//! \defgroup DRVIC

//!
//! \ingroup DRVIC
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// DRV8311S/P defines

// TODO
#define DRV8311_ADDR_MASK               (0x7E00)
#define DRV8311_DATA_MASK               (0x7FFF)
#define DRV8311_RW_MASK                 (0x8000)
#define DRV8311_FAULT_TYPE_MASK         (0x00FF)


//DEV_STS1 masks
#define DRV8311_DEV_STS1_FAULT_BITS          (0 << 0)
#define DRV8311_DEV_STS1_OT_BITS             (0 << 1)
#define DRV8311_DEV_STS1_UVP_BITS            (0 << 2)
#define DRV8311_DEV_STS1_RESERVED1_BITS      (0 << 3)
#define DRV8311_DEV_STS1_RESERVED2_BITS      (0 << 4)
#define DRV8311_DEV_STS1_OCP_BITS            (0 << 5)
#define DRV8311_DEV_STS1_SPI_FLT_BITS        (0 << 6)
#define DRV8311_DEV_STS1_RESET_BITS          (0 << 7)
#define DRV8311_DEV_STS1_OTP_FLT_BITS        (0 << 8)
#define DRV8311_DEV_STS1_RESERVED3_BITS      (0 << 9)
#define DRV8311_DEV_STS1_RESERVED4_BITS      (0 << 10)
#define DRV8311_DEV_STS1_RESERVED5_BITS      (0 << 11)
#define DRV8311_DEV_STS1_RESERVED6_BITS      (0 << 12)
#define DRV8311_DEV_STS1_RESERVED7_BITS      (0 << 13)
#define DRV8311_DEV_STS1_RESERVED8_BITS      (0 << 14)

//OT_STS masks
#define DRV8311_OT_STS_OTSD_BITS             (1 << 0)
#define DRV8311_OT_STS_OTW_BITS              (1 << 1)
#define DRV8311_OT_STS_OTS_AVDD_BITS         (1 << 2)
#define DRV8311_OT_STS_RESERVED1_BITS        (1 << 3)
#define DRV8311_OT_STS_RESERVED2_BITS        (1 << 4)
#define DRV8311_OT_STS_RESERVED3_BITS        (1 << 5)
#define DRV8311_OT_STS_RESERVED4_BITS        (1 << 6)
#define DRV8311_OT_STS_RESERVED5_BITS        (1 << 7)
#define DRV8311_OT_STS_RESERVED6_BITS        (1 << 8)
#define DRV8311_OT_STS_RESERVED7_BITS        (1 << 9)
#define DRV8311_OT_STS_RESERVED8_BITS        (1 << 10)
#define DRV8311_OT_STS_RESERVED9_BITS        (1 << 11)
#define DRV8311_OT_STS_RESERVED10_BITS       (1 << 12)
#define DRV8311_OT_STS_RESERVED11_BITS       (1 << 13)
#define DRV8311_OT_STS_RESERVED12_BITS       (1 << 14)

//SUP_STS_masks
#define DRV8311_SUP_STS_VINAVDD_UV_BITS      (1 << 0)
#define DRV8311_SUP_STS_RESERVED1_BITS       (1 << 1)
#define DRV8311_SUP_STS_AVDD_UV_BITS         (1 << 2)
#define DRV8311_SUP_STS_RESERVED2_BITS       (1 << 3)
#define DRV8311_SUP_STS_CP_UV_BITS           (1 << 4)
#define DRV8311_SUP_STS_CSAREF_UV_BITS       (1 << 5)
#define DRV8311_SUP_STS_RESERVED3_BITS       (1 << 6)
#define DRV8311_SUP_STS_RESERVED4_BITS       (1 << 7)
#define DRV8311_SUP_STS_RESERVED5_BITS       (1 << 8)
#define DRV8311_SUP_STS_RESERVED6_BITS       (1 << 9)
#define DRV8311_SUP_STS_RESERVED7_BITS       (1 << 10)
#define DRV8311_SUP_STS_RESERVED8_BITS       (1 << 11)
#define DRV8311_SUP_STS_RESERVED9_BITS       (1 << 12)
#define DRV8311_SUP_STS_RESERVED10_BITS      (1 << 13)
#define DRV8311_SUP_STS_RESERVED11_BITS      (1 << 14)

//DRV_STS masks
#define DRV8311_DRV_STS_OCPA_LS_BITS         (1 << 0)
#define DRV8311_DRV_STS_OCPB_LS_BITS         (1 << 1)
#define DRV8311_DRV_STS_OCPC_LS_BITS         (1 << 2)
#define DRV8311_DRV_STS_RESERVED1_BITS       (1 << 3)
#define DRV8311_DRV_STS_OCPA_HS_BITS         (1 << 4)
#define DRV8311_DRV_STS_OCPB_HS_BITS         (1 << 5)
#define DRV8311_DRV_STS_OCPC_HS_BITS         (1 << 6)
#define DRV8311_DRV_STS_RESERVED2_BITS       (1 << 7)
#define DRV8311_DRV_STS_RESERVED3_BITS       (1 << 8)
#define DRV8311_DRV_STS_RESERVED4_BITS       (1 << 9)
#define DRV8311_DRV_STS_RESERVED5_BITS       (1 << 10)
#define DRV8311_DRV_STS_RESERVED6_BITS       (1 << 11)
#define DRV8311_DRV_STS_RESERVED7_BITS       (1 << 12)
#define DRV8311_DRV_STS_RESERVED8_BITS       (1 << 13)
#define DRV8311_DRV_STS_RESERVED9_BITS       (1 << 14)

//SYS_STS masks
#define DRV8311_SYS_STS_FRM_ERR_BITS         (1 << 0)
#define DRV8311_SYS_STS_BUS_CNT_BITS         (1 << 1)
#define DRV8311_SYS_STS_SPI_PARITY           (1 << 2)
#define DRV8311_SYS_STS_RESERVED1_BITS       (1 << 3)
#define DRV8311_SYS_STS_RESERVED2_BITS       (1 << 4)
#define DRV8311_SYS_STS_RESERVED3_BITS       (1 << 5)
#define DRV8311_SYS_STS_RESERVED4_BITS       (1 << 6)
#define DRV8311_SYS_STS_RESERVED5_BITS       (1 << 7)
#define DRV8311_SYS_STS_RESERVED6_BITS       (1 << 8)
#define DRV8311_SYS_STS_RESERVED7_BITS       (1 << 9)
#define DRV8311_SYS_STS_RESERVED8_BITS       (1 << 10)
#define DRV8311_SYS_STS_RESERVED9_BITS       (1 << 11)
#define DRV8311_SYS_STS_RESERVED10_BITS      (1 << 12)
#define DRV8311_SYS_STS_RESERVED11_BITS      (1 << 13)
#define DRV8311_SYS_STS_RESERVED12_BITS      (1 << 14)


//PWM_SYNC_PRD masks
#define DRV8311_PWM_SYNC_PRD_PWM_SYNC_PRD_BITS   (4095 << 0) //2^12-1=4095
#define DRV8311_PWM_SYNC_PRD_RESERVED1_BITS      (1 << 12)
#define DRV8311_PWM_SYNC_PRD_RESERVED2_BITS      (1 << 13)
#define DRV8311_PWM_SYNC_PRD_RESERVED3_BITS      (1 << 14)
// should customer be able to read device name? or is that unnecessary? Question
//FLT_MODE masks
#define DRV8311_FLT_MODE_OTSD_MODE_BITS      (3 << 0)
#define DRV8311_FLT_MODE_UVP_MODE_BITS       (3 << 2)
#define DRV8311_FLT_MODE_OCP_MODE_BITS       (7 << 4)
#define DRV8311_FLT_MODE_SPIFLT_MODE_BITS    (1 << 7)
#define DRV8311_FLT_MODE_OTPFLT_MODE_BITS    (1 << 8)
#define DRV8311_FLT_MODE_RESERVED1_BITS      (1 << 9)
#define DRV8311_FLT_MODE_RESERVED2_BITS      (1 << 10)
#define DRV8311_FLT_MODE_RESERVED3_BITS      (1 << 11)
#define DRV8311_FLT_MODE_RESERVED4_BITS      (1 << 12)
#define DRV8311_FLT_MODE_RESERVED5_BITS      (1 << 13)
#define DRV8311_FLT_MODE_RESERVED6_BITS      (1 << 14)

//SYSF_CTRL masks
#define DRV8311_SYSF_CTRL_RESERVED1_BITS        (1 << 0)
#define DRV8311_SYSF_CTRL_RESERVED2_BITS         (1 << 1)
#define DRV8311_SYSF_CTRL_RESERVED3_BITS         (1 << 2)
#define DRV8311_SYSF_CTRL_RESERVED4_BITS         (1 << 3)
#define DRV8311_SYSF_CTRL_RESERVED5_BITS         (1 << 4)
#define DRV8311_SYSF_CTRL_CSAVREFUV_EN_BITS      (1 << 5)
#define DRV8311_SYSF_CTRL_RESERVED6_BITS         (1 << 6)
#define DRV8311_SYSF_CTRL_RESERVED7_BITS         (1 << 7)
#define DRV8311_SYSF_CTRL_RESERVED8_BITS         (1 << 8)
#define DRV8311_SYSF_CTRL_OTW_EN_BITS            (1 << 9)
#define DRV8311_SYSF_CTRL_OTAVDD_EN_BITS         (1 << 10)
#define DRV8311_SYSF_CTRL_RESERVED9_BITS         (1 << 11)
#define DRV8311_SYSF_CTRL_RESERVED10_BITS        (1 << 12)
#define DRV8311_SYSF_CTRL_RESERVED11_BITS        (1 << 13)
#define DRV8311_SYSF_CTRL_RESERVED12_BITS        (1 << 14)


//DRVF_CTRL masks
#define DRV8311_DRVF_CTRL_OCP_LVL_BITS        (1 << 0)
#define DRV8311_DRVF_CTRL_RESERVED1_BITS      (1 << 1)
#define DRV8311_DRVF_CTRL_OCP_TBLANK_BITS     (3 << 2)
#define DRV8311_DRVF_CTRL_OCP_DEG_BITS        (3 << 4)
#define DRV8311_DRVF_CTRL_RESERVED2_BITS      (1 << 6)
#define DRV8311_DRVF_CTRL_RESERVED3_BITS      (1 << 7)
#define DRV8311_DRVF_CTRL_RESERVED4_BITS      (1 << 8)
#define DRV8311_DRVF_CTRL_RESERVED5_BITS      (1 << 9)
#define DRV8311_DRVF_CTRL_RESERVED6_BITS      (1 << 10)
#define DRV8311_DRVF_CTRL_RESERVED7_BITS      (1 << 11)
#define DRV8311_DRVF_CTRL_RESERVED8_BITS      (1 << 12)
#define DRV8311_DRVF_CTRL_RESERVED9_BITS      (1 << 13)
#define DRV8311_DRVF_CTRL_RESERVED10_BITS     (1 << 14)


//FLT_TCTRL masks
#define DRV8311_FLT_TCTRL_FAST_TRETRY_BITS    (3 << 0)
#define DRV8311_FLT_TCTRL_SLOW_TRETRY_BITS    (3 << 2)
#define DRV8311_FLT_TCTRL_RESERVED1_BITS      (1 << 4)
#define DRV8311_FLT_TCTRL_RESERVED2_BITS      (1 << 5)
#define DRV8311_FLT_TCTRL_RESERVED3_BITS      (1 << 6)
#define DRV8311_FLT_TCTRL_RESERVED4_BITS      (1 << 7)
#define DRV8311_FLT_TCTRL_RESERVED5_BITS      (1 << 8)
#define DRV8311_FLT_TCTRL_RESERVED6_BITS      (1 << 9)
#define DRV8311_FLT_TCTRL_RESERVED7_BITS      (1 << 10)
#define DRV8311_FLT_TCTRL_RESERVED8_BITS      (1 << 11)
#define DRV8311_FLT_TCTRL_RESERVED9_BITS      (1 << 12)
#define DRV8311_FLT_TCTRL_RESERVED10_BITS     (1 << 13)
#define DRV8311_FLT_TCTRL_RESERVED11_BITS     (1 << 14)


//FLT_CLR masks
#define DRV8311_FLT_CLR_FLT_CLR_BITS        (1 << 0)
#define DRV8311_FLT_CLR_RESERVED1_BITS      (1 << 1)
#define DRV8311_FLT_CLR_RESERVED2_BITS      (1 << 2)
#define DRV8311_FLT_CLR_RESERVED3_BITS      (1 << 3)
#define DRV8311_FLT_CLR_RESERVED4_BITS      (1 << 4)
#define DRV8311_FLT_CLR_RESERVED5_BITS      (1 << 5)
#define DRV8311_FLT_CLR_RESERVED6_BITS      (1 << 6)
#define DRV8311_FLT_CLR_RESERVED7_BITS      (1 << 7)
#define DRV8311_FLT_CLR_RESERVED8_BITS      (1 << 8)
#define DRV8311_FLT_CLR_RESERVED9_BITS      (1 << 9)
#define DRV8311_FLT_CLR_RESERVED10_BITS     (1 << 10)
#define DRV8311_FLT_CLR_RESERVED11_BITS     (1 << 11)
#define DRV8311_FLT_CLR_RESERVED12_BITS     (1 << 12)
#define DRV8311_FLT_CLR_RESERVED13_BITS     (1 << 13)
#define DRV8311_FLT_CLR_RESERVED14_BITS     (1 << 14)

//PWMG_PERIOD masks
#define DRV8311_PWMG_PERIOD_PWM_PRD_OUT_BITS      (4095 << 0)
#define DRV8311_PWMG_PERIOD_RESERVED1_BITS      (1 << 12)
#define DRV8311_PWMG_PERIOD_RESERVED2_BITS      (1 << 13)
#define DRV8311_PWMG_PERIOD_RESERVED3_BITS      (1 << 14)


//PWMG_A_DUTY masks
#define DRV8311_PWMG_A_DUTY_PWM_DUTY_OUTA_BITS      (4095 << 0)
#define DRV8311_PWMG_A_DUTY_RESERVED1_BITS      (1 << 12)
#define DRV8311_PWMG_A_DUTY_RESERVED2_BITS      (1 << 13)
#define DRV8311_PWMG_A_DUTY_RESERVED3_BITS      (1 << 14)


//PWMG_B_DUTY masks
#define DRV8311_PWMG_B_DUTY_PWM_DUTY_OUTB_BITS      (4095 << 0)
#define DRV8311_PWMG_B_DUTY_RESERVED1_BITS      (1 << 12)
#define DRV8311_PWMG_B_DUTY_RESERVED2_BITS      (1 << 13)
#define DRV8311_PWMG_B_DUTY_RESERVED3_BITS      (1 << 14)

//PWMG_C_DUTY masks
#define DRV8311_PWMG_C_DUTY_PWM_DUTY_OUTC_BITS      (4095 << 0)
#define DRV8311_PWMG_C_DUTY_RESERVED1_BITS      (1 << 12)
#define DRV8311_PWMG_C_DUTY_RESERVED2_BITS      (1 << 13)
#define DRV8311_PWMG_C_DUTY_RESERVED3_BITS      (1 << 14)


//PWM_STATE masks
#define DRV8311_PWM_STATE_PWMA_STATE_BITS      (7 << 0)
#define DRV8311_PWM_STATE_RESERVED1_BITS       (1 << 3)
#define DRV8311_PWM_STATE_PWMB_STATE_BITS      (7 << 4)
#define DRV8311_PWM_STATE_RESERVED2_BITS       (1 << 7)
#define DRV8311_PWM_STATE_PWMC_STATE_BITS      (7 << 8)
#define DRV8311_PWM_STATE_RESERVED3_BITS       (1 << 11)
#define DRV8311_PWM_STATE_RESERVED4_BITS       (1 << 12)
#define DRV8311_PWM_STATE_RESERVED5_BITS       (1 << 13)
#define DRV8311_PWM_STATE_RESERVED6_BITS       (1 << 14)

//PWMG_CTRL masks
#define DRV8311_PWMG_CTRL_SPISYNC_ACRCY_BITS       (3 << 0)
#define DRV8311_PWMG_CTRL_SPICLK_FREQ_SYNC_BITS    (7 << 2)
#define DRV8311_PWMG_CTRL_PWM_OSC_SYNC_BITS        (7 << 5)
#define DRV8311_PWMG_CTRL_PWMCNTR_MODE_BITS        (3 << 8)
#define DRV8311_PWMG_CTRL_PWM_EN_BITS              (1 << 10)
#define DRV8311_PWMG_CTRL_RESERVED1_BITS           (1 << 11)
#define DRV8311_PWMG_CTRL_RESERVED2_BITS           (1 << 12)
#define DRV8311_PWMG_CTRL_RESERVED3_BITS           (1 << 13)
#define DRV8311_PWMG_CTRL_RESERVED4_BITS           (1 << 14)


//PWM_CTRL1 masks
#define DRV8311_PWM_CTRL1_PWM_MODE_BITS        (3 << 0)
#define DRV8311_PWM_CTRL1_SSC_DIS_BITS         (1 << 2)
#define DRV8311_PWM_CTRL1_RESERVED1_BITS       (1 << 3)
#define DRV8311_PWM_CTRL1_RESERVED2_BITS       (1 << 4)
#define DRV8311_PWM_CTRL1_RESERVED3_BITS       (1 << 5)
#define DRV8311_PWM_CTRL1_RESERVED4_BITS       (1 << 6)
#define DRV8311_PWM_CTRL1_RESERVED5_BITS       (1 << 7)
#define DRV8311_PWM_CTRL1_RESERVED6_BITS       (1 << 8)
#define DRV8311_PWM_CTRL1_RESERVED7_BITS       (1 << 9)
#define DRV8311_PWM_CTRL1_RESERVED8_BITS       (1 << 10)
#define DRV8311_PWM_CTRL1_RESERVED9_BITS       (1 << 11)
#define DRV8311_PWM_CTRL1_RESERVED10_BITS      (1 << 12)
#define DRV8311_PWM_CTRL1_RESERVED11_BITS      (1 << 13)
#define DRV8311_PWM_CTRL1_RESERVED12_BITS      (1 << 14)

//DRV_CTRL masks
#define DRV8311_DRV_CTRL_SLEW_RATE_BITS       (3 << 0)
#define DRV8311_DRV_CTRL_RESERVED1_BITS       (1 << 2)
#define DRV8311_DRV_CTRL_RESERVED2_BITS       (1 << 3)
#define DRV8311_DRV_CTRL_TDEAD_CTRL_BITS      (7 << 4)
#define DRV8311_DRV_CTRL_DLYCMP_EN_BITS       (1 << 7)
#define DRV8311_DRV_CTRL_DLY_TARGET_BITS      (15 << 8)
#define DRV8311_DRV_CTRL_RESERVED3_BITS       (1 << 12)
#define DRV8311_DRV_CTRL_RESERVED4_BITS       (1 << 13)
#define DRV8311_DRV_CTRL_RESERVED5_BITS       (1 << 14)

//CSA_CTRL masks
#define DRV8311_CSA_CTRL_CSA_GAIN_BITS        (3 << 0)
#define DRV8311_CSA_CTRL_RESERVED1_BITS       (1 << 2)
#define DRV8311_CSA_CTRL_CSA_EN_BITS          (1 << 3)
#define DRV8311_CSA_CTRL_RESERVED2_BITS       (1 << 4)
#define DRV8311_CSA_CTRL_RESERVED3_BITS       (1 << 5)
#define DRV8311_CSA_CTRL_RESERVED4_BITS       (1 << 6)
#define DRV8311_CSA_CTRL_RESERVED5_BITS       (1 << 7)
#define DRV8311_CSA_CTRL_RESERVED6_BITS       (1 << 8)
#define DRV8311_CSA_CTRL_RESERVED7_BITS       (1 << 9)
#define DRV8311_CSA_CTRL_RESERVED8_BITS       (1 << 10)
#define DRV8311_CSA_CTRL_RESERVED9_BITS       (1 << 11)
#define DRV8311_CSA_CTRL_RESERVED10_BITS      (1 << 12)
#define DRV8311_CSA_CTRL_RESERVED11_BITS      (1 << 13)
#define DRV8311_CSA_CTRL_RESERVED12_BITS      (1 << 14)

//SYS_CTRL masks
#define DRV8311_SYS_CTRL_RESERVED1_BITS       (1 << 0)
#define DRV8311_SYS_CTRL_RESERVED2_BITS       (1 << 1)
#define DRV8311_SYS_CTRL_RESERVED3_BITS       (1 << 2)
#define DRV8311_SYS_CTRL_RESERVED4_BITS       (1 << 3)
#define DRV8311_SYS_CTRL_RESERVED5_BITS       (1 << 4)
#define DRV8311_SYS_CTRL_RESERVED6_BITS       (1 << 5)
#define DRV8311_SYS_CTRL_SPI_PEN_BITS         (1 << 6)
#define DRV8311_SYS_CTRL_REG_LOCK_BITS        (1 << 7)
#define DRV8311_SYS_CTRL_RESERVED7_BITS       (1 << 8)
#define DRV8311_SYS_CTRL_RESERVED8_BITS       (1 << 9)
#define DRV8311_SYS_CTRL_RESERVED9_BITS       (1 << 10)
#define DRV8311_SYS_CTRL_RESERVED10_BITS      (1 << 11)
#define DRV8311_SYS_CTRL_WRITE_KEY_BITS       (7 << 12)


// **************************************************************************
// the typedefs


//! \brief Enumeration for the R/W modes
//!
typedef enum
{
    DRV8311_CTRLMODE_READ  = 1 << 7,   //!< Read Mode
    DRV8311_CTRLMODE_WRITE = 0 << 7,   //!< Write Mode
} DRV8311_CtrlMode_e;


//DEV_STS1 enumeration
typedef enum
{
  FAULT            = (1 << 0),
  OT               = (1 << 1),
  UVP              = (1 << 2),
  DEV_STS1_RSV1    = (1 << 3),
  DEV_STS1_RSV2    = (1 << 4),
  OCP              = (1 << 5),
  SPI_FLT          = (1 << 6),
  NPOR             = (1 << 7),
  OTP_FLT          = (1 << 8),
  DEV_STS1_RSV3    = (1 << 9),
  DEV_STS1_RSV4    = (1 << 10),
  DEV_STS1_RSV5    = (1 << 11),
  DEV_STS1_RSV6    = (1 << 12),
  DEV_STS1_RSV7    = (1 << 13),
  DEV_STS1_RSV8    = (1 << 14),
} DRV8311_DEV_STS1_e;


//OT_STS enumeration
typedef enum
{
  OTSD           = (1 << 0),
  OTW            = (1 << 1),
  OTS_AVDD       = (1 << 2),
  OT_STS_RSV1    = (1 << 3),
  OT_STS_RSV2    = (1 << 4),
  OT_STS_RSV3    = (1 << 5),
  OT_STS_RSV4    = (1 << 6),
  OT_STS_RSV5    = (1 << 7),
  OT_STS_RSV6    = (1 << 8),
  OT_STS_RSV7    = (1 << 9),
  OT_STS_RSV8    = (1 << 10),
  OT_STS_RSV9    = (1 << 11),
  OT_STS_RSV10   = (1 << 12),
  OT_STS_RSV11   = (1 << 13),
  OT_STS_RSV12   = (1 << 14),
} DRV8311_OT_STS_e;

//SUP_STS enumeration
typedef enum
{
  VINAVDD_UV        = (1 << 0),
  SUP_STS_RSV1      = (1 << 1),
  AVDD_UV           = (1 << 2),
  SUP_STS_RSV2      = (1 << 3),
  CP_UV             = (1 << 4),
  CSA_REF_UV        = (1 << 5),
  SUP_STS_RSV3      = (1 << 6),
  SUP_STS_RSV4      = (1 << 7),
  SUP_STS_RSV5      = (1 << 8),
  SUP_STS_RSV6      = (1 << 9),
  SUP_STS_RSV7      = (1 << 10),
  SUP_STS_RSV8      = (1 << 11),
  SUP_STS_RSV9      = (1 << 12),
  SUP_STS_RSV10     = (1 << 13),
  SUP_STS_RSV11     = (1 << 14),
} DRV8311_SUP_STS_e;


//DRV_STS enumeration
typedef enum
{
  OCPA_LS            = (1 << 0),
  OCPB_LS            = (1 << 1),
  OCPC_LS            = (1 << 2),
  DRV_STS_RSV1       = (1 << 3),
  OCPA_HS            = (1 << 4),
  OCPB_HS            = (1 << 5),
  OCPC_HS            = (1 << 6),
  DRV_STS_RSV2       = (1 << 7),
  DRV_STS_RSV3       = (1 << 8),
  DRV_STS_RSV4       = (1 << 9),
  DRV_STS_RSV5       = (1 << 10),
  DRV_STS_RSV6       = (1 << 11),
  DRV_STS_RSV7       = (1 << 12),
  DRV_STS_RSV8       = (1 << 13),
  DRV_STS_RSV9       = (1 << 14),
} DRV8311_DRV_STS_e;

//SYS_STS enumeration
typedef enum
{
    FRAME_ERR            = (1 << 0),
    BUS_CONTENTION       = (1 << 1),
    SPI_PARITY_ERR       = (1 << 2),
    SYS_STS_RSV1         = (1 << 3),
    SYS_STS_RSV2         = (1 << 4),
    SYS_STS_RSV3         = (1 << 5),
    SYS_STS_RSV4         = (1 << 6),
    SYS_STS_RSV5         = (1 << 7),
    SYS_STS_RSV6         = (1 << 8),
    SYS_STS_RSV7         = (1 << 9),
    SYS_STS_RSV8         = (1 << 10),
    SYS_STS_RSV9         = (1 << 11),
    SYS_STS_RSV10        = (1 << 12),
    SYS_STS_RSV11        = (1 << 13),
    SYS_STS_RSV12        = (1 << 14),
} DRV8311_SYS_STS_e;

//Overtemp shutdown mode enumeration
typedef enum
{
  OTSD_RTRY_Slow       = (0 << 0),
  OTSD_RTRY_Fast       = (1 << 0),
} DRV8311_FLT_MODE_OTSD_MODE_e;

//Undervoltage protection mode enumeration
typedef enum
{
  UVP_RTRY_Slow       = (0 << 2),
  UVP_RTRY_Fast       = (1 << 2),
} DRV8311_FLT_MODE_UVP_MODE_e;


//Overcurrent protection enumeration
typedef enum
{
    OCP_RTRY_Slow    = (0 << 4),
    OCP_RTRY_Fast    = (1 << 4),
    OCP_Latched      = (2 << 4),
    OCP_NoAction     = (3 << 4),
    OCP_Disabled     = (7 << 4),
} DRV8311_FLT_MODE_OCP_MODE_e;

//SPI fault mode enumeration
typedef enum
{
    SPI_FLT_Enabled    = (0 << 7),
    SPI_FLT_Disabled   = (1 << 7),
} DRV8311_FLT_MODE_SPIFLT_MODE_e;


//OTP fault mode enumeration
typedef enum
{
    OTP_FLT_Enabled   = (0 << 8),
    OTP_FLT_Disabled  = (1 << 8),
} DRV8311_FLT_MODE_OTPFLT_MODE_e;

////VIN_AVDD undervoltage fault enable enumeration
//typedef enum
//{
//    VINAVDDUV_Enabled   = (0 << 0),
//    VINAVDDUV_Disabled   = (1 << 0),
//} DRV8311_SYSF_CTRL_VINAVDDUV_EN_e;
//
////AVDD undervoltage fault enable enumeration
//typedef enum
//{
//    AVDDUV_Enabled   = (0 << 2),
//    AVDDUV_Disabled   = (1 << 2),
//} DRV8311_SYSF_CTRL_AVDDUV_EN_e;
//
////Charge pump undervoltage fault enable enumeration
//typedef enum
//{
//    CPUV_Enabled   = (0 << 4),
//    CPUV_Disabled   = (1 << 4),
//} DRV8311_SYSF_CTRL_CPUV_EN_e;

//CSA VREF undervotlage fault enable enumeration
typedef enum
{
    CSA_VREF_UVLO_Disabled   = (0 << 5),
    CSA_VREF_UVLO_Enabled    = (1 << 5),
} DRV8311_SYSF_CTRL_CSAVREFUV_EN_e;

//Overtemperature warning fault enable enumeration
typedef enum
{
    OTW_Disabled   = (0 << 9),
    OTW_Enabled    = (1 << 9),
} DRV8311_SYSF_CTRL_OTW_EN_e;

//Overtemperature AVDD fault enable enumeration
typedef enum
{
    OTW_AVDD_Disabled   = (0 << 10),
    OTW_AVDD_Enabled    = (1 << 10),
} DRV8311_SYSF_CTRL_OTAVDD_EN_e;

//OCP level settings enumeration
typedef enum
{
    OCP_LVL_8A   = (0 << 0),
    OCP_LVL_5A   = (1 << 0),
} DRV8311_DRVF_CTRL_OCP_LVL_e;

//OCP blanking times enumeration
typedef enum
{
    OCP_TBLANK_200ns   = (0 << 2),
    OCP_TBLANK_500ns   = (1 << 2),
    OCP_TBLANK_800ns   = (2 << 2),
    OCP_TBLANK_1000ns  = (3 << 2),
} DRV8311_DRVF_CTRL_OCP_TBLANK_e;

//OCP deglitch time enumeration
typedef enum
{
    OCP_DEG_200ns   = (0 << 4),
    OCP_DEG_500ns   = (1 << 4),
    OCP_DEG_800ns   = (2 << 4),
    OCP_DEG_1000ns  = (3 << 4),
} DRV8311_DRVF_CTRL_OCP_DEG_e;

//Fast recovery retry time settings enumeration
typedef enum
{
    FAST_RETRY_TIME_500us   = (0 << 0),
    FAST_RETRY_TIME_1ms     = (1 << 0),
    FAST_RETRY_TIME_2ms     = (2 << 0),
    FAST_RETRY_TIME_5ms     = (3 << 0),
} DRV8311_FLT_TCTRL_FAST_TRETRY_e;

//Slow recovery retry time settings enumeration
typedef enum
{
    SLOW_RETRY_TIME_500ms   = (0 << 2),
    SLOW_RETRY_TIME_1s      = (1 << 2),
    SLOW_RETRY_TIME_2s      = (2 << 2),
    SLOW_RETRY_TIME_5s      = (3 << 2),
} DRV8311_FLT_TCTRL_SLOW_TRETRY_e;

//FLT_CLR register enum
typedef enum
{
    No_Clear_Fault = (0 << 0),
    Clear_Fault    = (1 << 0),
} DRV8311_FLT_CLR_e;

//PWMA driver output control enumeration
typedef enum
{
    OUTA_HS_Off_LS_Off   = (0 << 0),
    OUTA_HS_Off_LS_On    = (1 << 0),
    OUTA_HS_On_LS_Off    = (2 << 0),
    OUTA_HS_Off_LS_PWM   = (5 << 0),
    OUTA_HS_PWM_LS_Off   = (6 << 0),
    OUTA_HS_PWM_LS_nPWM  = (7 << 0),
} DRV8311_PWM_STATE_PWMA_STATE_e;

//PWMB driver output control enumeration
typedef enum
{
    OUTB_HS_Off_LS_Off   = (0 << 4),
    OUTB_HS_Off_LS_On    = (1 << 4),
    OUTB_HS_On_LS_Off    = (2 << 4),
    OUTB_HS_Off_LS_PWM   = (5 << 4),
    OUTB_HS_PWM_LS_Off   = (6 << 4),
    OUTB_HS_PWM_LS_nPWM  = (7 << 4),
} DRV8311_PWM_STATE_PWMB_STATE_e;

//PWMC driver output control enumeration
typedef enum
{
    OUTC_HS_Off_LS_Off   = (0 << 8),
    OUTC_HS_Off_LS_On    = (1 << 8),
    OUTC_HS_On_LS_Off    = (2 << 8),
    OUTC_HS_Off_LS_PWM   = (5 << 8),
    OUTC_HS_PWM_LS_Off   = (6 << 8),
    OUTC_HS_PWM_LS_nPWM  = (7 << 8),
} DRV8311_PWM_STATE_PWMC_STATE_e;

//SPI synchronization accuracy enumeration
typedef enum
{
    Clock_Cycles_512   = (0 << 0),
    Clock_Cycles_256   = (1 << 0),
    Clock_Cycles_128   = (2 << 0),
    Clock_Cycles_64    = (3 << 0),
} DRV8311_PWMG_CTRL_SPISYNC_ACRCY_e;

//SPICLK frequency for syncronization enumeration
typedef enum
{
    SPI_Clock_Freq_1_MHz     = (0 << 2),
    SPI_Clock_Freq_1p25_MHz  = (1 << 2),
    SPI_Clock_Freq_2_MHz     = (2 << 2),
    SPI_Clock_Freq_2p5_MHz   = (3 << 2),
    SPI_Clock_Freq_4_MHz     = (4 << 2),
    SPI_Clock_Freq_5_MHz     = (5 << 2),
    SPI_Clock_Freq_8_MHz     = (6 << 2),
    SPI_Clock_Freq_10_MHz    = (7 << 2),
} DRV8311_PWMG_CTRL_SPICLK_FREQ_SYNC_e;

//oscillator syncronization and PWM_SYNC control enumeration
typedef enum
{
    OSC_SYNC_Disable1           = (0 << 5),
    PWM_SYNC_PRD_Period_and_Cal = (1 << 5),
    PWM_SYNC_Pwm_Period         = (2 << 5),
    OSC_SYNC_Disable2           = (3 << 5),
    OSC_SYNC_Disable3           = (4 << 5),
    PWM_SYNC_Osc_Sync           = (5 << 5),
    PWM_SYNC_Osc_Sync_Pwm_Per   = (6 << 5),
    SCLC_Pin_Osc_Sync           = (7 << 5),
} DRV8311_PWMG_CTRL_PWM_OSC_SYNC_e;

//PWM generation counter mode enumeration
typedef enum
{
    PWM_GEN_COUNTER_Up_Down  = (0 << 8),
    PWM_GEN_COUNTER_Up       = (1 << 8),
    PWM_GEN_COUNTER_Down     = (2 << 8),
    PWM_GEN_COUNTER_No_Act   = (3 << 8),
} DRV8311_PWMG_CTRL_PWM_CNTR_MODE_e;

//Enable 3X internal mode PWM generation enumeration
typedef enum
{
    PWM_GEN_Disabled     = (0 << 10),
    PWM_GEN_Enabled      = (1 << 10),
} DRV8311_PWMG_CTRL_PWM_EN_e;

//PWM mode selection enumeration
typedef enum
{
    PWM_6x_Mode              = (0 << 0),
   // PWM_6x_Mode             = (1 << 0),
    PWM_3x_Mode              = (2 << 0),
    PWM_Generation_Mode      = (3 << 0),
} DRV8311_PWM_CTRL1_PWM_MODE_e;

//Disable Spread Spectrum Modulation for internal Oscillator enumeration
typedef enum
{
    Spread_Spectrum_Mod_En   = (0 << 2),
    Spread_Spectrum_Mod_Dis  = (1 << 2),
} DRV8311_PWM_CTRL1_SSC_DIS_e;

// Slew rate settings enumeration
typedef enum
{
    SLEW_RATE_25v_us         = (0 << 0),
    SLEW_RATE_50v_us         = (1 << 0),
    SLEW_RATE_150v_us        = (2 << 0),
    SLEW_RATE_200v_us        = (3 << 0),
} DRV8311_DRV_CTRL_SLEW_RATE_e;

//Deadtime insertion control enumeration
typedef enum
{
    No_Deadtime              = (0 << 4),
    DEADTIME_200ns           = (1 << 4),
    DEADTIME_400ns           = (2 << 4),
    DEADTIME_600ns           = (3 << 4),
    DEADTIME_800ns           = (4 << 4),
    DEADTIME_1us             = (5 << 4),
    DEADTIME_1p2us           = (6 << 4),
    DEADTIME_1p4us           = (7 << 4),
} DRV8311_DRV_CTRL_TDEAD_CTRL_e;

//Driver delay compensation mode enumeration
typedef enum
{
    DRV_DELAY_COMP_Dis         = (0 << 7),
    DRV_DELAY_COMP_En          = (1 << 7),
} DRV8311_DRV_CTRL_DLYCMP_EN_e;

//Todo
//Delay target enumeration


//Current Sense Amplifier Gain settings enumeration
typedef enum
{
    CSA_GAIN_0p25           = (0 << 0),
    CSA_GAIN_0p5            = (1 << 0),
    CSA_GAIN_1              = (2 << 0),
    CSA_GAIN_2              = (3 << 0),
} DRV8311_CSA_CTRL_CSA_GAIN_e;

//Current Sense Amplifier enable enumeration
typedef enum
{
    CSA_Dis                 = (0 << 3),
    CSA_En                  = (1 << 3),
} DRV8311_CSA_CTRL_CSA_EN_e;

//Parity enable for both SPI and tSPI enumeration
typedef enum
{
    PARITY_Dis                 = (0 << 6),
    PARITY_En                  = (1 << 6),
} DRV8311_SYS_CTRL_SPI_PEN_e;

//register lock bit enumeration
typedef enum
{
    REGISTERS_Unlocked         = (0 << 7),
    REGISTERS_Locked           = (1 << 7),
} DRV8311_SYS_CTRL_REG_LOCK_e;

//write key to access SYS_CTRL register enumeration
typedef enum
{
    Access_To_SYS_CTRL_Reg         = (5 << 12),

} DRV8311_SYS_CTRL_WRITE_KEY_e;

//STOP

//! \brief Enumeration for the register addresses   //what should the bit field position be?
//!
typedef enum
{
    DRV8311_ADDRESS_DEV_STS1      = (0 << 1),   //!< Device Status 1 Register
    DRV8311_ADDRESS_OT_STS        = (4 << 1),   //!< Over Temperature Status Register
    DRV8311_ADDRESS_SUP_STS       = (5 << 1),   //!< Supply Status Register
    DRV8311_ADDRESS_DRV_STS       = (6 << 1),   //!< Driver Status Register
    DRV8311_ADDRESS_SYS_STS       = (7 << 1),   //!< System Status Register
    DRV8311_ADDRESS_PWM_SYNC_PRD  = (12 << 1),  //!< PWM Sync Period Register
    DRV8311_ADDRESS_FLT_MODE      = (16 << 1),  //!< Fault Mode Register
    DRV8311_ADDRESS_SYSF_CTRL     = (18 << 1),  //!< System Fault Control Register
    DRV8311_ADDRESS_DRVF_CTRL     = (19 << 1),  //!< Driver Fault Control Register
    DRV8311_ADDRESS_FLT_TCTRL     = (22 << 1),  //!< Fault Timing Control Register
    DRV8311_ADDRESS_FLT_CLR       = (23 << 1),  //!< Fault Clear Register
    DRV8311_ADDRESS_PWMG_PERIOD   = (24 << 1),  //!< PWM_GEN Period Register
    DRV8311_ADDRESS_PWMG_A_DUTY    = (25 << 1), //!< PWM_GEN A Duty Register
    DRV8311_ADDRESS_PWMG_B_DUTY    = (26 << 1), //!< PWM_GEN B Duty Register
    DRV8311_ADDRESS_PWMG_C_DUTY    = (27 << 1), //!< PWM_GEN C Duty Register
    DRV8311_ADDRESS_PWM_STATE      = (28 << 1), //!< PWM State Register
    DRV8311_ADDRESS_PWMG_CTRL      = (29 << 1), //!< PWM_GEN Control Register
    DRV8311_ADDRESS_PWM_CTRL1      = (32 << 1), //!< PWM Control Register 1
    DRV8311_ADDRESS_DRV_CTRL       = (34 << 1), //!< Predriver control Register
    DRV8311_ADDRESS_CSA_CTRL       = (35 << 1), //!< CSA Control Register
    DRV8311_ADDRESS_SYS_CTRL       = (63 << 1), //!< System Control Register
} DRV8311_Address_e;


//-----------------------OBJECTS-----------------------------


//! \brief Object for the DRV8311 DEV_STS1 register
//!
typedef struct _DRV8311_DEV_STS1_00_t
{
  bool                  FAULT;          // Bits 0
  bool                  OT;             // Bits 1
  bool                  UVP;            // Bits 2
  bool                  DEV_STS1_RSV1;  // Bits 3
  bool                  DEV_STS1_RSV2;  // Bits 4
  bool                  OCP;            // Bits 5
  bool                  SPI_FLT;        // Bits 6
  bool                  NPOR;           // Bits 7
  bool                  OTP_FLT;        // Bits 8
  bool                  DEV_STS1_RSV3;  // Bits 9
  bool                  DEV_STS1_RSV4;  // Bits 10
  bool                  DEV_STS1_RSV5;  // Bits 11
  bool                  DEV_STS1_RSV6;  // Bits 12
  bool                  DEV_STS1_RSV7;  // Bits 13
  bool                  DEV_STS1_RSV8;  // Bits 14
}DRV8311_DEV_STS1_00_t;

//! \brief Object for the DRV8311 OT_STS register
//!
typedef struct _DRV8311_OT_STS_04_t_
{
  bool                  OTSD;           // Bits 0
  bool                  OTW;            // Bits 1
  bool                  OTS_AVDD;       // Bits 2
  bool                  OT_STS_RSV1;    // Bits 3
  bool                  OT_STS_RSV2;    // Bits 4
  bool                  OT_STS_RSV3;    // Bits 5
  bool                  OT_STS_RSV4;    // Bits 6
  bool                  OT_STS_RSV5;    // Bits 7
  bool                  OT_STS_RSV6;    // Bits 8
  bool                  OT_STS_RSV7;    // Bits 9
  bool                  OT_STS_RSV8;    // Bits 10
  bool                  OT_STS_RSV9;    // Bits 11
  bool                  OT_STS_RSV10;   // Bits 12
  bool                  OT_STS_RSV11;   // Bits 13
  bool                  OT_STS_RSV12;   // Bits 14
}DRV8311_OT_STS_04_t;


//! \brief Object for the DRV8311 SUP_STS register
//!
typedef struct _DRV8311_SUP_STS_05_t_
{
  bool                  VINAVDD_UV;     // Bits 0
  bool                  SUP_STS_RSV1;   // Bits 1
  bool                  AVDD_UV;        // Bits 2
  bool                  SUP_STS_RSV2;   // Bits 3
  bool                  CP_UV;          // Bits 4
  bool                  CSA_REF_UV;     // Bits 5
  bool                  SUP_STS_RSV3;   // Bits 6
  bool                  SUP_STS_RSV4;   // Bits 7
  bool                  SUP_STS_RSV5;   // Bits 8
  bool                  SUP_STS_RSV6;   // Bits 9
  bool                  SUP_STS_RSV7;   // Bits 10
  bool                  SUP_STS_RSV8;   // Bits 11
  bool                  SUP_STS_RSV9;   // Bits 12
  bool                  SUP_STS_RSV10;  // Bits 13
  bool                  SUP_STS_RSV11;  // Bits 14
}DRV8311_SUP_STS_05_t;


//! \brief Object for the DRV8311 DRV_STS register
//!
typedef struct _DRV8311_DRV_STS_06_t_
{
  bool                   OCPA_LS;      // Bits 0
  bool                   OCPB_LS;      // Bits 1
  bool                   OCPC_LS;      // Bits 2
  bool                   DRV_STS_RSV1; // Bits 3
  bool                   OCPA_HS;      // Bits 4
  bool                   OCPB_HS;      // Bits 5
  bool                   OCPC_HS;      // Bits 6
  bool                   DRV_STS_RSV2; // Bits 7
  bool                   DRV_STS_RSV3; // Bits 8
  bool                   DRV_STS_RSV4; // Bits 9
  bool                   DRV_STS_RSV5; // Bits 10
  bool                   DRV_STS_RSV6; // Bits 11
  bool                   DRV_STS_RSV7; // Bits 12
  bool                   DRV_STS_RSV8; // Bits 13
  bool                   DRV_STS_RSV9; // Bits 14
}DRV8311_DRV_STS_06_t;


//! \brief Object for the DRV8311 SYS_STS register
//!
typedef struct _DRV8311_SYS_STS_07_t_
{
  bool                   FRAME_ERR;       // Bits 0
  bool                   BUS_CONTENTION;  // Bits 1
  bool                   SPI_PARITY_ERR;  // Bits 2
  bool                   SYS_STS_RSV1;    // Bits 3
  bool                   SYS_STS_RSV2;    // Bits 4
  bool                   SYS_STS_RSV3;    // Bits 5
  bool                   SYS_STS_RSV4;    // Bits 6
  bool                   SYS_STS_RSV5;    // Bits 7
  bool                   SYS_STS_RSV6;    // Bits 8
  bool                   SYS_STS_RSV7;    // Bits 9
  bool                   SYS_STS_RSV8;    // Bits 10
  bool                   SYS_STS_RSV9;    // Bits 11
  bool                   SYS_STS_RSV10;   // Bits 12
  bool                   SYS_STS_RSV11;   // Bits 13
  bool                   SYS_STS_RSV12;   // Bits 14
}DRV8311_SYS_STS_07_t;

//! \brief Object for the DRV8311 PWM_SYNC_PRD register
//!
typedef struct _DRV8311_PWM_SYNC_PRD_0C_t_
{
  uint16_t                      PWM_SYNC_PRD;       // Bits 0-11
  bool                          PWM_SYNC_PRD_RSV1;  // Bits 12
  bool                          PWM_SYNC_PRD_RSV2;  // Bits 13
  bool                          PWM_SYNC_PRD_RSV3;  // Bits 14
}DRV8311_PWM_SYNC_PRD_0C_t;

//! \brief Object for the DRV8311 FLT_MODE register
//!
typedef struct _DRV8311_FLT_MODE_10_t_
{
  DRV8311_FLT_MODE_OTSD_MODE_e   OTSD_MODE;    // Bit 0-1
  DRV8311_FLT_MODE_UVP_MODE_e    UVP_MODE;     // Bits 2-3
  DRV8311_FLT_MODE_OCP_MODE_e    OCP_MODE;     // Bits 4-6
  DRV8311_FLT_MODE_SPIFLT_MODE_e SPI_FLT_MODE; // Bits 7
  DRV8311_FLT_MODE_OTPFLT_MODE_e OTP_FLT_MODE; // Bits 8
  bool                           FLT_MODE_RSV1;// Bits 9
  bool                           FLT_MODE_RSV2;// Bits 10
  bool                           FLT_MODE_RSV3;// Bits 11
  bool                           FLT_MODE_RSV4;// Bits 12
  bool                           FLT_MODE_RSV5;// Bits 13
  bool                           FLT_MODE_RSV6;// Bits 14
}DRV8311_FLT_MODE_10_t;


//! \brief Object for the DRV8311 SYSF_CTRL register
//!
typedef struct _DRV8311_SYSF_CTRL_12_t_
{
  bool                              SYSF_CTRL_RSV1; // Bits 0
  bool                              SYSF_CTRL_RSV2; // Bits 1
  bool                              SYSF_CTRL_RSV3; // Bits 2
  bool                              SYSF_CTRL_RSV4; // Bits 3
  bool                              SYSF_CTRL_RSV5; // Bits 4
  DRV8311_SYSF_CTRL_CSAVREFUV_EN_e  CSAVREFUV_EN;   // Bits 5
  bool                              SYSF_CTRL_RSV6; // Bits 6
  bool                              SYSF_CTRL_RSV7; // Bits 7
  bool                              SYSF_CTRL_RSV8; // Bits 8
  DRV8311_SYSF_CTRL_OTW_EN_e        OTW_EN;         // Bits 9
  DRV8311_SYSF_CTRL_OTAVDD_EN_e     OTAVDD_EN;      // Bits 10
  bool                              SYSF_CTRL_RSV9; // Bits 11
  bool                              SYSF_CTRL_RSV10;// Bits 12
  bool                              SYSF_CTRL_RSV11;// Bits 13
  bool                              SYSF_CTRL_RSV12;// Bits 14
}DRV8311_SYSF_CTRL_12_t;


//! \brief Object for the DRV8311 DRVF_CTRL register
//!
typedef struct _DRV8311_DRVF_CTRL_13_t_
{
  DRV8311_DRVF_CTRL_OCP_LVL_e  OCP_LVL;        // Bits 0
  bool                         DRVF_CTRL_RSV1; // Bits 1
  DRV8311_DRVF_CTRL_OCP_TBLANK_e  OCP_TBLANK;  // Bits 2-3
  DRV8311_DRVF_CTRL_OCP_DEG_e  OCP_DEG;        // Bits 4-5
  bool                         DRVF_CTRL_RSV2; // Bits 6
  bool                         DRVF_CTRL_RSV3; // Bits 7
  bool                         DRVF_CTRL_RSV4; // Bits 8
  bool                         DRVF_CTRL_RSV5; // Bits 9
  bool                         DRVF_CTRL_RSV6; // Bits 10
  bool                         DRVF_CTRL_RSV7; // Bits 11
  bool                         DRVF_CTRL_RSV8; // Bits 12
  bool                         DRVF_CTRL_RSV9; // Bits 13
  bool                         DRVF_CTRL_RSV10;// Bits 14
}DRV8311_DRVF_CTRL_13_t;

//! \brief Object for the DRV8311 FLT_TCTRL register
//!
typedef struct _DRV8311_FLT_TCTRL_16_t_
{
  DRV8311_FLT_TCTRL_FAST_TRETRY_e  FAST_TRETRY;  // Bits 0-1
  DRV8311_FLT_TCTRL_SLOW_TRETRY_e  SLOW_TRETRY;  // Bits 2-3
  bool                          FLT_TCTRL_RSV1;  // Bits 4
  bool                          FLT_TCTRL_RSV2;  // Bits 5
  bool                          FLT_TCTRL_RSV3;  // Bits 6
  bool                          FLT_TCTRL_RSV4;  // Bits 7
  bool                          FLT_TCTRL_RSV5;  // Bits 8
  bool                          FLT_TCTRL_RSV6;  // Bits 9
  bool                          FLT_TCTRL_RSV7;  // Bits 10
  bool                          FLT_TCTRL_RSV8;  // Bits 11
  bool                          FLT_TCTRL_RSV9;  // Bits 12
  bool                          FLT_TCTRL_RSV10; // Bits 13
  bool                          FLT_TCTRL_RSV11; // Bits 14

}DRV8311_FLT_TCTRL_16_t;

//! \brief Object for the DRV8311 FLT_CLR register
//!
typedef struct _DRV8311_FLT_CLR_17_t_
{
  DRV8311_FLT_CLR_e             FLT_CLR;       // Bits 0
  bool                          FLT_CLR_RSV1;  // Bits 1
  bool                          FLT_CLR_RSV2;  // Bits 2
  bool                          FLT_CLR_RSV3;  // Bits 3
  bool                          FLT_CLR_RSV4;  // Bits 4
  bool                          FLT_CLR_RSV5;  // Bits 5
  bool                          FLT_CLR_RSV6;  // Bits 6
  bool                          FLT_CLR_RSV7;  // Bits 7
  bool                          FLT_CLR_RSV8;  // Bits 8
  bool                          FLT_CLR_RSV9;  // Bits 9
  bool                          FLT_CLR_RSV10; // Bits 10
  bool                          FLT_CLR_RSV11; // Bits 11
  bool                          FLT_CLR_RSV12; // Bits 12
  bool                          FLT_CLR_RSV13; // Bits 13
  bool                          FLT_CLR_RSV14; // Bits 14
}DRV8311_FLT_CLR_17_t;

//! \brief Object for the DRV8311 PWMG_PERIOD register
//!
typedef struct _DRV8311_PWMG_PERIOD_18_t_
{
  uint16_t                      PWM_PRD_OUT;       // Bits 0-11
  bool                          PWMG_PERIOD_RSV1;  // Bits 12
  bool                          PWMG_PERIOD_RSV2;  // Bits 13
  bool                          PWMG_PERIOD_RSV3;  // Bits 14
  }DRV8311_PWMG_PERIOD_18_t;

//! \brief Object for the DRV8311 PWMG_A_DUTY register
//!
  typedef struct _DRV8311_PWMG_A_DUTY_19_t_
{
  uint16_t                      PWM_DUTY_OUTA;     // Bits 0-11
  bool                          PWMG_A_DUTY_RSV1;  // Bits 12
  bool                          PWMG_A_DUTY_RSV2;  // Bits 13
  bool                          PWMG_A_DUTY_RSV3;  // Bits 14
}DRV8311_PWMG_A_DUTY_19_t;

//! \brief Object for the DRV8311 PWMG_B_DUTY register
//!
typedef struct _DRV8311_PWMG_B_DUTY_1A_t_
{
  uint16_t                      PWM_DUTY_OUTB;     // Bits 0-11
  bool                          PWMG_B_DUTY_RSV1;  // Bits 12
  bool                          PWMG_B_DUTY_RSV2;  // Bits 13
  bool                          PWMG_B_DUTY_RSV3;  // Bits 14
}DRV8311_PWMG_B_DUTY_1A_t;

//! \brief Object for the DRV8311 PWMG_C_DUTY register
//!
typedef struct _DRV8311_PWMG_C_DUTY_1B_t_
{
  uint16_t                      PWM_DUTY_OUTC;     // Bits 0-11
  bool                          PWMG_C_DUTY_RSV1;  // Bits 12
  bool                          PWMG_C_DUTY_RSV2;  // Bits 13
  bool                          PWMG_C_DUTY_RSV3;  // Bits 14
}DRV8311_PWMG_C_DUTY_1B_t;

//! \brief Object for the DRV8311 PWM_STATE register
//!
typedef struct _DRV8311_PWM_STATE_1C_t_
{
  DRV8311_PWM_STATE_PWMA_STATE_e PWMA_STATE;  // Bits 0-2
  bool                       PWM_STATE_RSV1;  // Bits 3
  DRV8311_PWM_STATE_PWMB_STATE_e PWMB_STATE;  // Bits 4-6
  bool                       PWM_STATE_RSV2;  // Bits 7
  DRV8311_PWM_STATE_PWMC_STATE_e PWMC_STATE;  // Bits 8-10
  bool                       PWM_STATE_RSV3;  // Bits 11
  bool                       PWM_STATE_RSV4;  // Bits 12
  bool                       PWM_STATE_RSV5;  // Bits 13
  bool                       PWM_STATE_RSV6;  // Bits 14
}DRV8311_PWM_STATE_1C_t;

//! \brief Object for the DRV8311 PWMG_CTRL register
//!
typedef struct _DRV8311_PWMG_CTRL_1D_t_
{
  DRV8311_PWMG_CTRL_SPISYNC_ACRCY_e    SPISYNC_ACRCY;       // Bits 0-1
  DRV8311_PWMG_CTRL_SPICLK_FREQ_SYNC_e SPICLK_FREQ_SYNC;    // Bits 2-4
  DRV8311_PWMG_CTRL_PWM_OSC_SYNC_e     PWM_OSC_SYNC;        // Bits 5-7
  DRV8311_PWMG_CTRL_PWM_CNTR_MODE_e    PWM_CNTR_MODE;       // Bits 8-9
  DRV8311_PWMG_CTRL_PWM_EN_e           PWM_EN;              // Bits 10
  bool                                 PWMG_CTRL_RSV1;      // Bits 11
  bool                                 PWMG_CTRL_RSV2;      // Bits 12
  bool                                 PWMG_CTRL_RSV3;      // Bits 13
  bool                                 PWMG_CTRL_RSV4;      // Bits 14
}DRV8311_PWMG_CTRL_1D_t;

//! \brief Object for the DRV8311 PWM_CTRL1 register
//!
typedef struct _DRV8311_PWM_CTRL1_20_t_
{
  DRV8311_PWM_CTRL1_PWM_MODE_e    PWM_MODE;        // Bits 0-1
  DRV8311_PWM_CTRL1_SSC_DIS_e     SSC_DIS;         // Bits 2
  bool                            PWM_CTRL1_RSV1;  // Bits 3
  bool                            PWM_CTRL1_RSV2;  // Bits 4
  bool                            PWM_CTRL1_RSV3;  // Bits 5
  bool                            PWM_CTRL1_RSV4;  // Bits 6
  bool                            PWM_CTRL1_RSV5;  // Bits 7
  bool                            PWM_CTRL1_RSV6;  // Bits 8
  bool                            PWM_CTRL1_RSV7;  // Bits 9
  bool                            PWM_CTRL1_RSV8;  // Bits 10
  bool                            PWM_CTRL1_RSV9;  // Bits 11
  bool                            PWM_CTRL1_RSV10; // Bits 12
  bool                            PWM_CTRL1_RSV11; // Bits 13
  bool                            PWM_CTRL1_RSV12; // Bits 14

}DRV8311_PWM_CTRL1_20_t;

//! \brief Object for the DRV8311 DRV_CTRL register
//!
typedef struct _DRV8311_DRV_CTRL_22_t_
{
  DRV8311_DRV_CTRL_SLEW_RATE_e    SLEW_RATE;       // Bits 0-1
  bool                            DRV_CTRL_RSV1;   // Bits 2
  bool                            DRV_CTRL_RSV2;   // Bits 3
  DRV8311_DRV_CTRL_TDEAD_CTRL_e   TDEAD_CTRL;      // Bits 4-6
  DRV8311_DRV_CTRL_DLYCMP_EN_e    DLYCMP_EN;       // Bits 7
  uint16_t                        DLY_TARGET;      // Bits 8-11
  bool                            DRV_CTRL_RSV3;   // Bits 12
  bool                            DRV_CTRL_RSV4;   // Bits 13
  bool                            DRV_CTRL_RSV5;   // Bits 14
}DRV8311_DRV_CTRL_22_t;

//! \brief Object for the DRV8311 CSA_CTRL register
//!
typedef struct _DRV8311_CSA_CTRL_23_t_
{
  DRV8311_CSA_CTRL_CSA_GAIN_e     CSA_GAIN;        // Bits 0-1
  bool                            CSA_CTRL_RSV1;   // Bits 2
  DRV8311_CSA_CTRL_CSA_EN_e       CSA_EN;          // Bits 3
  bool                            CSA_CTRL_RSV2;   // Bits 4
  bool                            CSA_CTRL_RSV3;   // Bits 5
  bool                            CSA_CTRL_RSV4;   // Bits 6
  bool                            CSA_CTRL_RSV5;   // Bits 7
  bool                            CSA_CTRL_RSV6;   // Bits 8
  bool                            CSA_CTRL_RSV7;   // Bits 9
  bool                            CSA_CTRL_RSV8;   // Bits 10
  bool                            CSA_CTRL_RSV9;   // Bits 11
  bool                            CSA_CTRL_RSV10;  // Bits 12
  bool                            CSA_CTRL_RSV11;  // Bits 13
  bool                            CSA_CTRL_RSV12;  // Bits 14
}DRV8311_CSA_CTRL_23_t;

//! \brief Object for the DRV8311 SYS_CTRL register
//!
typedef struct _DRV8311_SYS_CTRL_3F_t_
{
  bool                            SYS_CTRL_RSV1;      // Bits 0
  bool                            SYS_CTRL_RSV2;      // Bits 1
  bool                            SYS_CTRL_RSV3;      // Bits 2
  bool                            SYS_CTRL_RSV4;      // Bits 3
  bool                            SYS_CTRL_RSV5;      // Bits 4
  bool                            SYS_CTRL_RSV6;      // Bits 5
  DRV8311_SYS_CTRL_SPI_PEN_e      SPI_PEN;            // Bits 6
  DRV8311_SYS_CTRL_REG_LOCK_e     REG_LOCK;           // Bits 7
  bool                            SYS_CTRL_RSV7;      // Bits 8
  bool                            SYS_CTRL_RSV8;      // Bits 9
  bool                            SYS_CTRL_RSV9;      // Bits 10
  bool                            SYS_CTRL_RSV10;     // Bits 11
  DRV8311_SYS_CTRL_WRITE_KEY_e    WRITE_KEY; // Bits 12-14
}DRV8311_SYS_CTRL_3F_t;



//! \brief Object for the DRV8311 registers and commands
//!
//TODO
typedef struct _DRV8311_SPIVars_t_
{
    DRV8311_DEV_STS1_00_t        DEV_STS1_Reg_00;
    DRV8311_OT_STS_04_t          OT_STS_Reg_04;
    DRV8311_SUP_STS_05_t         SUP_STS_Reg_05;
    DRV8311_DRV_STS_06_t         DRV_STS_Reg_06;
    DRV8311_SYS_STS_07_t         SYS_STS_Reg_07;
    DRV8311_PWM_SYNC_PRD_0C_t    PWM_SYNC_PRD_Reg_0C;
    DRV8311_FLT_MODE_10_t        FLT_MODE_Reg_10;
    DRV8311_SYSF_CTRL_12_t       SYSF_CTRL_Reg_12;
    DRV8311_DRVF_CTRL_13_t       DRVF_CTRL_Reg_13;
    DRV8311_FLT_TCTRL_16_t       FLT_TCTRL_Reg_16;
    DRV8311_FLT_CLR_17_t         FLT_CLR_Reg_17;
    DRV8311_PWMG_PERIOD_18_t     PWMG_PERIOD_Reg_18;
    DRV8311_PWMG_A_DUTY_19_t     PWMG_A_DUTY_Reg_19;
    DRV8311_PWMG_B_DUTY_1A_t     PWMG_B_DUTY_Reg_1A;
    DRV8311_PWMG_C_DUTY_1B_t     PWMG_C_DUTY_Reg_1B;
    DRV8311_PWM_STATE_1C_t       PWM_STATE_Reg_1C;
    DRV8311_PWMG_CTRL_1D_t       PWMG_CTRL_Reg_1D;
    DRV8311_PWM_CTRL1_20_t       PWM_CTRL1_Reg_20;
    DRV8311_DRV_CTRL_22_t        DRV_CTRL_Reg_22;
    DRV8311_CSA_CTRL_23_t        CSA_CTRL_Reg_23;
    DRV8311_SYS_CTRL_3F_t        SYS_CTRL_Reg_3F;

    bool                         writeCmd;
    bool                         readCmd;

    uint16_t                     manWriteAddr;
    uint16_t                     manReadAddr;
    uint16_t                     manWriteData;
    uint16_t                     manReadData;
    bool                         manWriteCmd;
    bool                         manReadCmd;
}DRV8311_SPIVars_t;


//! \brief Defines the DRV8311 object
//!
typedef struct _DRV8311_Obj_
{
    uint32_t  spiHandle;     //!< handle for the serial peripheral interface
    uint32_t  gpioNumber_CS; //!< GPIO connected to the DRV8311 CS pin
    uint32_t  gpioNumber_EN; //!< GPIO connected to the DRV8311 enable pin
    bool      rxTimeOut;     //!< timeout flag for the RX FIFO
    bool      enableTimeOut; //!< timeout flag for DRV8311 enable
} DRV8311_Obj;


//! \brief Defines the DRV8311 handle
//!
typedef struct _DRV8311_Obj_ *DRV8311_Handle;


//! \brief Defines the DRV8311 Word type
//!
typedef  uint16_t    DRV8311_Word_t;


// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes

//! \brief     Initializes the DRV8311 object
//! \param[in] pMemory   A pointer to the memory for the DRV8311 object
//! \param[in] numBytes  The number of bytes allocated for the DRV8311
//!                      object, bytes
//! \return    The DRV8311 object handle
extern DRV8311_Handle DRV8311_init(void *pMemory);

//! \brief     Builds the 8-bit header
//! \param[in] ctrlMode  The control mode
//! \param[in] regName   The register name
//! \return    The 8-bit header
static inline DRV8311_Word_t DRV8311_buildCtrlHeader(
                                            const DRV8311_CtrlMode_e ctrlMode,
                                            const DRV8311_Address_e regAddr)
{
    uint16_t parity = 0;
    uint16_t temp = regAddr;

    while(temp)
    {
        parity ^= (temp & 1);
        temp >>= 1;
    }

    DRV8311_Word_t ctrlHeader = ctrlMode | (regAddr) | (parity & 1);

    return(ctrlHeader);
} // end of DRV8311_buildHeader() function

//! \brief     Builds the control word
//! \param[in] ctrlMode  The control mode
//! \param[in] regName   The register name
//! \param[in] data      The data
//! \return    The control word
static inline DRV8311_Word_t DRV8311_buildCtrlWord(
                                            const uint16_t data)
{
    uint16_t parity = 0;
    uint16_t temp = data;

    while(temp)
    {
        parity ^= (temp & 1);
        temp >>= 1;
    }

    parity <<= 15;

    DRV8311_Word_t ctrlWord = parity | (data & DRV8311_DATA_MASK);

    return(ctrlWord);
} // end of DRV8311_buildCtrlWord() function


//! \brief     Enables the DRV8311
//! \param[in] handle     The DRV8311 handle
extern void DRV8311_enable(DRV8311_Handle handle);

//! \brief     Sets the SPI handle in the DRV8311
//! \param[in] handle     The DRV8311 handle
//! \param[in] spiHandle  The SPI handle to use
void DRV8311_setSPIHandle(DRV8311_Handle handle,uint32_t spiHandle);

//! \brief     Sets the GPIO number in the DRV8311
//! \param[in] handle       The DRV8311 handle
//! \param[in] gpioHandle   The GPIO number to use
void DRV8311_setGPIOCSNumber(DRV8311_Handle handle,uint32_t gpioNumber);

//! \brief     Sets the GPIO number in the DRV8311
//! \param[in] handle       The DRV8311 handle
//! \param[in] gpioHandle   The GPIO number to use
void DRV8311_setGPIONumber(DRV8311_Handle handle,uint32_t gpioNumber);

//! \brief     Resets the enable timeout flag
//! \param[in] handle   The DRV8311 handle
static inline void DRV8311_resetEnableTimeout(DRV8311_Handle handle)
{
    DRV8311_Obj *obj = (DRV8311_Obj *)handle;

    obj->enableTimeOut = false;

    return;
} // end of DRV8311_resetEnableTimeout() function

//! \brief     Resets the RX fifo timeout flag
//! \param[in] handle   The DRV8311 handle
static inline void DRV8311_resetRxTimeout(DRV8311_Handle handle)
{
    DRV8311_Obj *obj = (DRV8311_Obj *)handle;

    obj->rxTimeOut = false;

    return;
} // end of DRV8311_resetRxTimeout() function

//! \brief     Initialize the interface to all 8316 SPI variables
//! \param[in] handle  The DRV8311 handle
extern void DRV8311_setupSPI(DRV8311_Handle handle,
                             DRV8311_SPIVars_t *DRV8311SPIVars);

//! \brief     Reads data from the DRV8311 register
//! \param[in] handle   The DRV8311 handle
//! \param[in] regAddr  The register address
//! \return    The data value
extern uint16_t DRV8311_readSPI(DRV8311_Handle handle,
                                const DRV8311_Address_e regAddr);

//! \brief     Writes data to the DRV8311 register
//! \param[in] handle   The DRV8311 handle
//! \param[in] regAddr  The register name
//! \param[in] data     The data value
extern void DRV8311_writeSPI(DRV8311_Handle handle,
                             const DRV8311_Address_e regAddr,
                             const uint16_t data);

//! \brief     Write to the DRV8311 SPI registers
//! \param[in] handle  The DRV8311 handle
//! \param[in] DRV8311SPIVars  The (DRV8311_SPIVars_t) structure that contains
//!                           all DRV8311 Status/Control register options
extern void DRV8311_writeData(DRV8311_Handle handle,
                              DRV8311_SPIVars_t *DRV8311SPIVars);

//! \brief     Read from the DRV8311 SPI registers
//! \param[in] handle  The DRV8311 handle
//! \param[in] DRV8311SPIVars  The (DRV8311_SPIVars_t) structure that contains
//!                           all DRV8311 Status/Control register options
extern void DRV8311_readData(DRV8311_Handle handle,
                             DRV8311_SPIVars_t *DRV8311SPIVars);

#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of DRV8311_H definition
