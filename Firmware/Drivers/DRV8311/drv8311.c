//#############################################################################
// $TI Release: MotorControl SDK v3.00.01.00 $
// $Release Date: Tue May 26 19:13:59 CDT 2020 $
// $Copyright:
// Copyright (C) 2017-2019 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//! \file   solutions/boostxl_drv8311rs/f28004x/drivers/source/drv8311.c
//! \brief  Contains the various functions related to the drv8311 object
//!

// **************************************************************************
// the includes

#include <math.h>

// **************************************************************************
// drivers
#include <drv8311.h>

// **************************************************************************
// modules

// **************************************************************************
// platforms

// **************************************************************************
// the defines

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes
void DRV8311_enable(DRV8311_Handle handle)
{
    DRV8311_Obj *obj = (DRV8311_Obj *)handle;
    volatile uint16_t enableWaitTimeOut;
    uint16_t n = 0;

    // Enable the drv8311
    //GPIO_writePin(obj->gpioNumber_EN, 1); //no DRVOFF pin on DRV8311

    enableWaitTimeOut = 0;

    // Make sure the FAULT bit is not set during startup
    while(((DRV8311_readSPI(handle, DRV8311_ADDRESS_DEV_STS1) &
            DRV8311_DEV_STS1_FAULT_BITS) != 0) && (enableWaitTimeOut < 1000))
    {
        if(++enableWaitTimeOut > 999)
        {
            obj->enableTimeOut = true;
        }
    }

    // Wait for the drv8311 to go through start up sequence
    for(n = 0; n < 0xffff; n++)
    {
        __asm(" NOP");
    }

    return;
} // end of DRV8311_enable() function

DRV8311_Handle DRV8311_init(void *pMemory)
{
    DRV8311_Handle handle;

    // assign the handle
    handle = (DRV8311_Handle)pMemory;

    DRV8311_resetRxTimeout(handle);
    DRV8311_resetEnableTimeout(handle);

    return(handle);
} // end of DRV8311_init() function

DRV8311_PWM_CTRL1_PWM_MODE_e DRV8311_getPWMMode(DRV8311_Handle handle)
{
    uint16_t data;

    // read data
    data = DRV8311_readSPI(handle, DRV8311_ADDRESS_PWM_CTRL1);

    // mask the bits
    data &= DRV8311_PWM_CTRL1_PWM_MODE_BITS;

    return((DRV8311_PWM_CTRL1_PWM_MODE_e)data);
} // end of DRV8311_getPWMMode function

void DRV8311_setSPIHandle(DRV8311_Handle handle, uint32_t spiHandle)
{
    DRV8311_Obj *obj = (DRV8311_Obj *)handle;

    // initialize the serial peripheral interface object
    obj->spiHandle = spiHandle;

    return;
} // end of DRV8311_setSPIHandle() function

void DRV8311_setGPIOCSNumber(DRV8311_Handle handle, uint32_t gpioNumber)
{
    DRV8311_Obj *obj = (DRV8311_Obj *)handle;

    // initialize the gpio interface object
    obj->gpioNumber_CS = gpioNumber;

    return;
} // end of DRV8311_setGPIOCSNumber() function

void DRV8311_setGPIONumber(DRV8311_Handle handle, uint32_t gpioNumber)
{
    DRV8311_Obj *obj = (DRV8311_Obj *)handle;

    // initialize the gpio interface object
    obj->gpioNumber_EN = gpioNumber;

    return;
} // end of DRV8311_setGPIONumber() function

void DRV8311_setupSPI(DRV8311_Handle handle,
                      DRV8311_SPIVars_t *drv8311SPIVars)
{
    DRV8311_Address_e drvRegAddr;
    uint16_t drvDataNew;

    // Set Default Values
    // Manual Read/Write
    drv8311SPIVars->manReadAddr  = 0;
    drv8311SPIVars->manReadData  = 0;
    drv8311SPIVars->manReadCmd = false;
    drv8311SPIVars->manWriteAddr = 0;
    drv8311SPIVars->manWriteData = 0;
    drv8311SPIVars->manWriteCmd = false;

    // Read/Write
    drv8311SPIVars->readCmd  = false;
    drv8311SPIVars->writeCmd = false;

    // Read registers for default values
    // Read Status Register 0
    drvRegAddr = DRV8311_ADDRESS_DEV_STS1;
    drvDataNew = DRV8311_readSPI(handle, drvRegAddr);

    drv8311SPIVars->DEV_STS1_Reg_00.FAULT         = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_FAULT_BITS)?1:0;
    drv8311SPIVars->DEV_STS1_Reg_00.OT            = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_OT_BITS)?1:0;
    drv8311SPIVars->DEV_STS1_Reg_00.UVP           = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_UVP_BITS)?1:0;
    drv8311SPIVars->DEV_STS1_Reg_00.DEV_STS1_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESERVED1_BITS)?1:0;
    drv8311SPIVars->DEV_STS1_Reg_00.DEV_STS1_RSV2 = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESERVED2_BITS)?1:0;
    drv8311SPIVars->DEV_STS1_Reg_00.OCP           = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_OCP_BITS)?1:0;
    drv8311SPIVars->DEV_STS1_Reg_00.SPI_FLT       = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_SPI_FLT_BITS)?1:0;
    drv8311SPIVars->DEV_STS1_Reg_00.NPOR          = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESET_BITS)?1:0;
    drv8311SPIVars->DEV_STS1_Reg_00.OTP_FLT       = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_OTP_FLT_BITS)?1:0;
    drv8311SPIVars->DEV_STS1_Reg_00.DEV_STS1_RSV3 = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESERVED3_BITS)?1:0;
    drv8311SPIVars->DEV_STS1_Reg_00.DEV_STS1_RSV4 = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESERVED4_BITS)?1:0;
    drv8311SPIVars->DEV_STS1_Reg_00.DEV_STS1_RSV5 = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESERVED5_BITS)?1:0;
    drv8311SPIVars->DEV_STS1_Reg_00.DEV_STS1_RSV6 = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESERVED6_BITS)?1:0;
    drv8311SPIVars->DEV_STS1_Reg_00.DEV_STS1_RSV7 = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESERVED7_BITS)?1:0;
    drv8311SPIVars->DEV_STS1_Reg_00.DEV_STS1_RSV8 = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESERVED8_BITS)?1:0;

    // Read OT_STS
    drvRegAddr = DRV8311_ADDRESS_OT_STS;
    drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
    drv8311SPIVars->OT_STS_Reg_04.OTSD           = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_OTSD_BITS)?1:0;
    drv8311SPIVars->OT_STS_Reg_04.OTW            = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_OTW_BITS)?1:0;
    drv8311SPIVars->OT_STS_Reg_04.OTS_AVDD       = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_OTS_AVDD_BITS)?1:0;
    drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV1    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED1_BITS)?1:0;
    drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV2    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED2_BITS)?1:0;
    drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV3    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED3_BITS)?1:0;
    drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV4    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED4_BITS)?1:0;
    drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV5    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED5_BITS)?1:0;
    drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV6    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED6_BITS)?1:0;
    drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV7    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED7_BITS)?1:0;
    drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV8    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED8_BITS)?1:0;
    drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV9    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED9_BITS)?1:0;
    drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV10   = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED10_BITS)?1:0;
    drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV11   = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED11_BITS)?1:0;
    drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV12   = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED12_BITS)?1:0;

    // Read SUP_STS
    drvRegAddr = DRV8311_ADDRESS_SUP_STS;
    drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
    drv8311SPIVars->SUP_STS_Reg_05.VINAVDD_UV    = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_VINAVDD_UV_BITS)?1:0;
    drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED1_BITS)?1:0;
    drv8311SPIVars->SUP_STS_Reg_05.AVDD_UV       = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_AVDD_UV_BITS)?1:0;
    drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED2_BITS)?1:0;
    drv8311SPIVars->SUP_STS_Reg_05.CP_UV         = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_CP_UV_BITS)?1:0;
    drv8311SPIVars->SUP_STS_Reg_05.CSA_REF_UV    = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_CSAREF_UV_BITS)?1:0;
    drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED3_BITS)?1:0;
    drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV4  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED4_BITS)?1:0;
    drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV5  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED5_BITS)?1:0;
    drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV6  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED6_BITS)?1:0;
    drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV7  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED7_BITS)?1:0;
    drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV8  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED8_BITS)?1:0;
    drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV9  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED9_BITS)?1:0;
    drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV10 = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED10_BITS)?1:0;
    drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV11 = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED11_BITS)?1:0;

    // Read DRV_STS
    drvRegAddr = DRV8311_ADDRESS_DRV_STS;
    drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
    drv8311SPIVars->DRV_STS_Reg_06.OCPA_LS      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_OCPA_LS_BITS)?1:0;
    drv8311SPIVars->DRV_STS_Reg_06.OCPB_LS      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_OCPB_LS_BITS)?1:0;
    drv8311SPIVars->DRV_STS_Reg_06.OCPC_LS      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_OCPC_LS_BITS)?1:0;
    drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED1_BITS)?1:0;
    drv8311SPIVars->DRV_STS_Reg_06.OCPA_HS      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_OCPA_HS_BITS)?1:0;
    drv8311SPIVars->DRV_STS_Reg_06.OCPB_HS      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_OCPB_HS_BITS)?1:0;
    drv8311SPIVars->DRV_STS_Reg_06.OCPC_HS      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_OCPC_HS_BITS)?1:0;
    drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV2 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED2_BITS)?1:0;
    drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV3 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED3_BITS)?1:0;
    drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV4 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED4_BITS)?1:0;
    drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV5 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED5_BITS)?1:0;
    drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV6 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED6_BITS)?1:0;
    drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV7 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED7_BITS)?1:0;
    drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV8 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED8_BITS)?1:0;
    drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV9 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED9_BITS)?1:0;

    // Read SYS_STS
    drvRegAddr = DRV8311_ADDRESS_SYS_STS;
    drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
    drv8311SPIVars->SYS_STS_Reg_07.FRAME_ERR      = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_FRM_ERR_BITS)?1:0;
    drv8311SPIVars->SYS_STS_Reg_07.BUS_CONTENTION = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_BUS_CNT_BITS)?1:0;
    drv8311SPIVars->SYS_STS_Reg_07.SPI_PARITY_ERR = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_SPI_PARITY)?1:0;
    drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV1   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED1_BITS)?1:0;
    drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV2   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED2_BITS)?1:0;
    drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV3   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED3_BITS)?1:0;
    drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV4   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED4_BITS)?1:0;
    drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV5   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED5_BITS)?1:0;
    drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV6   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED6_BITS)?1:0;
    drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV7   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED7_BITS)?1:0;
    drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV8   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED8_BITS)?1:0;
    drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV9   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED9_BITS)?1:0;
    drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV10  = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED10_BITS)?1:0;
    drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV11  = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED11_BITS)?1:0;
    drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV12  = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED12_BITS)?1:0;

    // Read PWM_SYNC_PRD
    drvRegAddr = DRV8311_ADDRESS_PWM_SYNC_PRD;
    drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
    drv8311SPIVars->PWM_SYNC_PRD_Reg_0C.PWM_SYNC_PRD       = (uint16_t)(drvDataNew & (uint16_t)DRV8311_PWM_SYNC_PRD_PWM_SYNC_PRD_BITS);
    drv8311SPIVars->PWM_SYNC_PRD_Reg_0C.PWM_SYNC_PRD_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_SYNC_PRD_RESERVED1_BITS)?1:0;
    drv8311SPIVars->PWM_SYNC_PRD_Reg_0C.PWM_SYNC_PRD_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_SYNC_PRD_RESERVED2_BITS)?1:0;
    drv8311SPIVars->PWM_SYNC_PRD_Reg_0C.PWM_SYNC_PRD_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_SYNC_PRD_RESERVED3_BITS)?1:0;

    // Read FLT_MODE
    drvRegAddr = DRV8311_ADDRESS_FLT_MODE;
    drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
    drv8311SPIVars->FLT_MODE_Reg_10.OTSD_MODE        = (DRV8311_FLT_MODE_OTSD_MODE_e)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_OTSD_MODE_BITS);
    drv8311SPIVars->FLT_MODE_Reg_10.UVP_MODE         = (DRV8311_FLT_MODE_UVP_MODE_e)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_UVP_MODE_BITS);
    drv8311SPIVars->FLT_MODE_Reg_10.OCP_MODE         = (DRV8311_FLT_MODE_OCP_MODE_e)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_OCP_MODE_BITS);
    drv8311SPIVars->FLT_MODE_Reg_10.SPI_FLT_MODE     = (DRV8311_FLT_MODE_SPIFLT_MODE_e)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_SPIFLT_MODE_BITS);
    drv8311SPIVars->FLT_MODE_Reg_10.OTP_FLT_MODE     = (DRV8311_FLT_MODE_OTPFLT_MODE_e)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_OTPFLT_MODE_BITS);
    drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV1    = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_RESERVED1_BITS)?1:0;
    drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV2    = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_RESERVED2_BITS)?1:0;
    drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV3    = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_RESERVED3_BITS)?1:0;
    drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV4    = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_RESERVED4_BITS)?1:0;
    drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV5    = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_RESERVED5_BITS)?1:0;
    drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV6    = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_RESERVED6_BITS)?1:0;

   // Read SYSF_CTRL
   drvRegAddr = DRV8311_ADDRESS_SYSF_CTRL;
   drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
   drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV1     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED1_BITS)?1:0;
   drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV2     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED2_BITS)?1:0;
   drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV3     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED3_BITS)?1:0;
   drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV4     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED4_BITS)?1:0;
   drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV5     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED5_BITS)?1:0;
   drv8311SPIVars->SYSF_CTRL_Reg_12.CSAVREFUV_EN       = (DRV8311_SYSF_CTRL_CSAVREFUV_EN_e)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_CSAVREFUV_EN_BITS);
   drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV6     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED6_BITS)?1:0;
   drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV7     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED7_BITS)?1:0;
   drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV8     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED8_BITS)?1:0;
   drv8311SPIVars->SYSF_CTRL_Reg_12.OTW_EN             = (DRV8311_SYSF_CTRL_OTW_EN_e)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_OTW_EN_BITS);
   drv8311SPIVars->SYSF_CTRL_Reg_12.OTAVDD_EN          = (DRV8311_SYSF_CTRL_OTAVDD_EN_e)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_OTAVDD_EN_BITS);
   drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV9     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED9_BITS)?1:0;
   drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV10    = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED10_BITS)?1:0;
   drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV11    = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED11_BITS)?1:0;
   drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV12    = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED12_BITS)?1:0;

   // Read DRVF_CTRL
   drvRegAddr = DRV8311_ADDRESS_DRVF_CTRL;
   drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
   drv8311SPIVars->DRVF_CTRL_Reg_13.OCP_LVL          = (DRV8311_DRVF_CTRL_OCP_LVL_e)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_OCP_LVL_BITS);
   drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV1   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED1_BITS)?1:0;
   drv8311SPIVars->DRVF_CTRL_Reg_13.OCP_TBLANK       = (DRV8311_DRVF_CTRL_OCP_TBLANK_e)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_OCP_TBLANK_BITS);
   drv8311SPIVars->DRVF_CTRL_Reg_13.OCP_DEG          = (DRV8311_DRVF_CTRL_OCP_DEG_e)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_OCP_DEG_BITS);
   drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV2   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED2_BITS)?1:0;
   drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV3   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED3_BITS)?1:0;
   drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV4   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED4_BITS)?1:0;
   drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV5   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED5_BITS)?1:0;
   drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV6   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED6_BITS)?1:0;
   drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV7   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED7_BITS)?1:0;
   drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV8   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED8_BITS)?1:0;
   drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV9   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED9_BITS)?1:0;
   drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV10  = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED10_BITS)?1:0;

   // Read FLT_TCTRL
   drvRegAddr = DRV8311_ADDRESS_FLT_TCTRL;
   drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
   drv8311SPIVars->FLT_TCTRL_Reg_16.FAST_TRETRY      = (DRV8311_FLT_TCTRL_FAST_TRETRY_e)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_FAST_TRETRY_BITS);
   drv8311SPIVars->FLT_TCTRL_Reg_16.SLOW_TRETRY      = (DRV8311_FLT_TCTRL_SLOW_TRETRY_e)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_SLOW_TRETRY_BITS);
   drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV1   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED1_BITS)?1:0;
   drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV2   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED1_BITS)?1:0;
   drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV3   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED2_BITS)?1:0;
   drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV4   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED3_BITS)?1:0;
   drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV5   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED4_BITS)?1:0;
   drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV6   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED5_BITS)?1:0;
   drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV7   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED6_BITS)?1:0;
   drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV8   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED7_BITS)?1:0;
   drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV9   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED8_BITS)?1:0;
   drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV10  = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED9_BITS)?1:0;
   drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV11  = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED10_BITS)?1:0;

   // Read FLT_CLR
   drvRegAddr = DRV8311_ADDRESS_FLT_CLR;
   drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
   drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR        = (DRV8311_FLT_CLR_e)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_FAST_TRETRY_BITS);
   drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV1   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED1_BITS)?1:0;
   drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV2   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED2_BITS)?1:0;
   drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV3   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED3_BITS)?1:0;
   drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV4   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED4_BITS)?1:0;
   drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV5   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED5_BITS)?1:0;
   drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV6   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED6_BITS)?1:0;
   drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV7   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED7_BITS)?1:0;
   drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV8   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED8_BITS)?1:0;
   drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV9   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED9_BITS)?1:0;
   drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV10  = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED10_BITS)?1:0;
   drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV11  = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED11_BITS)?1:0;
   drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV12  = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED12_BITS)?1:0;
   drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV13  = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED13_BITS)?1:0;
   drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV14  = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED14_BITS)?1:0;

   // Read PWMG_PERIOD
   drvRegAddr = DRV8311_ADDRESS_PWMG_PERIOD;
   drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
   drv8311SPIVars->PWMG_PERIOD_Reg_18.PWM_PRD_OUT       = (uint16_t)(drvDataNew & (uint16_t)DRV8311_PWMG_PERIOD_PWM_PRD_OUT_BITS);
   drv8311SPIVars->PWMG_PERIOD_Reg_18.PWMG_PERIOD_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_PERIOD_RESERVED1_BITS)?1:0;
   drv8311SPIVars->PWMG_PERIOD_Reg_18.PWMG_PERIOD_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_PERIOD_RESERVED2_BITS)?1:0;
   drv8311SPIVars->PWMG_PERIOD_Reg_18.PWMG_PERIOD_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_PERIOD_RESERVED3_BITS)?1:0;

   // Read PWMG_A_DUTY
   drvRegAddr = DRV8311_ADDRESS_PWMG_A_DUTY;
   drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
   drv8311SPIVars->PWMG_A_DUTY_Reg_19.PWM_DUTY_OUTA     = (uint16_t)(drvDataNew & (uint16_t)DRV8311_PWMG_A_DUTY_PWM_DUTY_OUTA_BITS);
   drv8311SPIVars->PWMG_A_DUTY_Reg_19.PWMG_A_DUTY_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_A_DUTY_RESERVED1_BITS)?1:0;
   drv8311SPIVars->PWMG_A_DUTY_Reg_19.PWMG_A_DUTY_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_A_DUTY_RESERVED2_BITS)?1:0;
   drv8311SPIVars->PWMG_A_DUTY_Reg_19.PWMG_A_DUTY_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_A_DUTY_RESERVED3_BITS)?1:0;

    // Read PWMG_B_DUTY
    drvRegAddr = DRV8311_ADDRESS_PWMG_B_DUTY;
    drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
    drv8311SPIVars->PWMG_B_DUTY_Reg_1A.PWM_DUTY_OUTB     = (uint16_t)(drvDataNew & (uint16_t)DRV8311_PWMG_B_DUTY_PWM_DUTY_OUTB_BITS);
    drv8311SPIVars->PWMG_B_DUTY_Reg_1A.PWMG_B_DUTY_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_B_DUTY_RESERVED1_BITS)?1:0;
    drv8311SPIVars->PWMG_B_DUTY_Reg_1A.PWMG_B_DUTY_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_B_DUTY_RESERVED2_BITS)?1:0;
    drv8311SPIVars->PWMG_B_DUTY_Reg_1A.PWMG_B_DUTY_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_B_DUTY_RESERVED3_BITS)?1:0;

    // Read PWMG_C_DUTY
    drvRegAddr = DRV8311_ADDRESS_PWMG_C_DUTY;
    drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
    drv8311SPIVars->PWMG_C_DUTY_Reg_1B.PWM_DUTY_OUTC       = (uint16_t)(drvDataNew & (uint16_t)DRV8311_PWMG_C_DUTY_PWM_DUTY_OUTC_BITS);
    drv8311SPIVars->PWMG_C_DUTY_Reg_1B.PWMG_C_DUTY_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_C_DUTY_RESERVED1_BITS)?1:0;
    drv8311SPIVars->PWMG_C_DUTY_Reg_1B.PWMG_C_DUTY_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_C_DUTY_RESERVED2_BITS)?1:0;
    drv8311SPIVars->PWMG_C_DUTY_Reg_1B.PWMG_C_DUTY_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_C_DUTY_RESERVED3_BITS)?1:0;

     // Read PWM_STATE
     drvRegAddr = DRV8311_ADDRESS_PWM_STATE;
     drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
     drv8311SPIVars->PWM_STATE_Reg_1C.PWMA_STATE      = (DRV8311_PWM_STATE_PWMA_STATE_e)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_PWMA_STATE_BITS);
     drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_RESERVED1_BITS)?1:0;
     drv8311SPIVars->PWM_STATE_Reg_1C.PWMB_STATE      = (DRV8311_PWM_STATE_PWMB_STATE_e)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_PWMB_STATE_BITS);
     drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_RESERVED2_BITS)?1:0;
     drv8311SPIVars->PWM_STATE_Reg_1C.PWMC_STATE      = (DRV8311_PWM_STATE_PWMC_STATE_e)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_PWMC_STATE_BITS);
     drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_RESERVED3_BITS)?1:0;
     drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV4  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_RESERVED4_BITS)?1:0;
     drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV5  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_RESERVED5_BITS)?1:0;
     drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV6  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_RESERVED6_BITS)?1:0;

     // Read PWMG_CTRL
     drvRegAddr = DRV8311_ADDRESS_PWMG_CTRL;
     drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
     drv8311SPIVars->PWMG_CTRL_Reg_1D.SPISYNC_ACRCY      = (DRV8311_PWMG_CTRL_SPISYNC_ACRCY_e)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_SPISYNC_ACRCY_BITS);
     drv8311SPIVars->PWMG_CTRL_Reg_1D.SPICLK_FREQ_SYNC   = (DRV8311_PWMG_CTRL_SPICLK_FREQ_SYNC_e)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_SPICLK_FREQ_SYNC_BITS);
     drv8311SPIVars->PWMG_CTRL_Reg_1D.PWM_OSC_SYNC       = (DRV8311_PWMG_CTRL_PWM_OSC_SYNC_e)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_PWM_OSC_SYNC_BITS);
     drv8311SPIVars->PWMG_CTRL_Reg_1D.PWM_CNTR_MODE      = (DRV8311_PWMG_CTRL_PWM_CNTR_MODE_e)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_PWMCNTR_MODE_BITS);
     drv8311SPIVars->PWMG_CTRL_Reg_1D.PWM_EN             = (DRV8311_PWMG_CTRL_PWM_EN_e)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_PWM_EN_BITS);
     drv8311SPIVars->PWMG_CTRL_Reg_1D.PWMG_CTRL_RSV1     = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_RESERVED1_BITS)?1:0;
     drv8311SPIVars->PWMG_CTRL_Reg_1D.PWMG_CTRL_RSV2     = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_RESERVED2_BITS)?1:0;
     drv8311SPIVars->PWMG_CTRL_Reg_1D.PWMG_CTRL_RSV3     = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_RESERVED3_BITS)?1:0;
     drv8311SPIVars->PWMG_CTRL_Reg_1D.PWMG_CTRL_RSV4     = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_RESERVED4_BITS)?1:0;

     // Read PWM_CTRL1
     drvRegAddr = DRV8311_ADDRESS_PWM_CTRL1;
     drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
     drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_MODE          = (DRV8311_PWM_CTRL1_PWM_MODE_e)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_PWM_MODE_BITS);
     drv8311SPIVars->PWM_CTRL1_Reg_20.SSC_DIS           = (DRV8311_PWM_CTRL1_SSC_DIS_e)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_SSC_DIS_BITS);
     drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV1    = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_RESERVED1_BITS)?1:0;
     drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV2    = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_RESERVED2_BITS)?1:0;
     drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV3    = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_RESERVED3_BITS)?1:0;
     drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV4    = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_RESERVED4_BITS)?1:0;
     drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV5    = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_RESERVED5_BITS)?1:0;
     drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV6    = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_RESERVED6_BITS)?1:0;
     drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV7    = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_RESERVED7_BITS)?1:0;

     // Read DRV_CTRL
     drvRegAddr = DRV8311_ADDRESS_DRV_CTRL;
     drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
     drv8311SPIVars->DRV_CTRL_Reg_22.SLEW_RATE          = (DRV8311_DRV_CTRL_SLEW_RATE_e)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_SLEW_RATE_BITS);
     drv8311SPIVars->DRV_CTRL_Reg_22.DRV_CTRL_RSV1      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_RESERVED1_BITS)?1:0;
     drv8311SPIVars->DRV_CTRL_Reg_22.DRV_CTRL_RSV2      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_RESERVED2_BITS)?1:0;
     drv8311SPIVars->DRV_CTRL_Reg_22.TDEAD_CTRL         = (DRV8311_DRV_CTRL_TDEAD_CTRL_e)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_TDEAD_CTRL_BITS);
     drv8311SPIVars->DRV_CTRL_Reg_22.DLYCMP_EN          = (DRV8311_DRV_CTRL_DLYCMP_EN_e)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_DLYCMP_EN_BITS);
     drv8311SPIVars->DRV_CTRL_Reg_22.DLY_TARGET         = (uint16_t)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_DLY_TARGET_BITS);
     drv8311SPIVars->DRV_CTRL_Reg_22.DRV_CTRL_RSV3      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_RESERVED3_BITS)?1:0;
     drv8311SPIVars->DRV_CTRL_Reg_22.DRV_CTRL_RSV4      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_RESERVED4_BITS)?1:0;
     drv8311SPIVars->DRV_CTRL_Reg_22.DRV_CTRL_RSV5      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_RESERVED5_BITS)?1:0;

     // Read CSA_CTRL
     drvRegAddr = DRV8311_ADDRESS_CSA_CTRL;
     drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
     drv8311SPIVars->CSA_CTRL_Reg_23.CSA_GAIN         = (DRV8311_CSA_CTRL_CSA_GAIN_e)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_CSA_GAIN_BITS);
     drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV1    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED1_BITS)?1:0;
     drv8311SPIVars->CSA_CTRL_Reg_23.CSA_EN           = (DRV8311_CSA_CTRL_CSA_EN_e)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_CSA_EN_BITS);
     drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV2    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED2_BITS)?1:0;
     drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV3    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED3_BITS)?1:0;
     drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV4    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED4_BITS)?1:0;
     drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV5    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED5_BITS)?1:0;
     drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV6    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED6_BITS)?1:0;
     drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV7    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED7_BITS)?1:0;
     drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV8    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED8_BITS)?1:0;
     drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV9    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED9_BITS)?1:0;
     drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV10   = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED10_BITS)?1:0;
     drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV11   = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED11_BITS)?1:0;
     drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV12   = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED12_BITS)?1:0;

     // Read SYS_CTRL
     drvRegAddr = DRV8311_ADDRESS_SYS_CTRL;
     drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
     drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV1    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED1_BITS)?1:0;
     drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV2    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED2_BITS)?1:0;
     drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV3    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED3_BITS)?1:0;
     drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV4    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED4_BITS)?1:0;
     drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV5    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED5_BITS)?1:0;
     drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV6    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED6_BITS)?1:0;
     drv8311SPIVars->SYS_CTRL_Reg_3F.SPI_PEN          = (DRV8311_SYS_CTRL_SPI_PEN_e)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_SPI_PEN_BITS);
     drv8311SPIVars->SYS_CTRL_Reg_3F.REG_LOCK         = (DRV8311_SYS_CTRL_REG_LOCK_e)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_REG_LOCK_BITS);
     drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV7    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED7_BITS)?1:0;
     drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV8    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED8_BITS)?1:0;
     drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV9    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED9_BITS)?1:0;
     drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV10   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED10_BITS)?1:0;
     drv8311SPIVars->SYS_CTRL_Reg_3F.WRITE_KEY = (DRV8311_SYS_CTRL_WRITE_KEY_e)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_WRITE_KEY_BITS);

     return;
} // end of DRV8311_setupSPI() function

uint16_t DRV8311_readSPI(DRV8311_Handle handle,
                         const DRV8311_Address_e regAddr)
{
    DRV8311_Obj *obj = (DRV8311_Obj *)handle;
    uint16_t ctrlHeader = 0;
    uint16_t n;

    volatile uint16_t readWord, readWord1, readWord2;
    volatile uint16_t WaitTimeOut = 0;

    // volatile SPI_RxFIFOLevel RxFifoCnt = SPI_FIFO_RXEMPTY;

    // build the control header
    ctrlHeader = (uint16_t)DRV8311_buildCtrlHeader(DRV8311_CTRLMODE_READ, regAddr);

    // // reset the Rx fifo pointer to zero
    // SPI_resetRxFIFO(obj->spiHandle);
    // SPI_enableFIFO(obj->spiHandle);

  GPIO_writePin(obj->gpioNumber_CS, 0);

    // wait for registers to update
    for(n = 0; n < 0x06; n++)
    {
        __asm(" NOP");
    }

    // write the command
//    HWREGH(obj->spiHandle + SPI_O_TXBUF) = ctrlHeader << 8;
//    HWREGH(obj->spiHandle + SPI_O_TXBUF) = 0xFF;
//    HWREGH(obj->spiHandle + SPI_O_TXBUF) = 0xFF;

    SPI_writeDataNonBlocking(obj->spiHandle, ctrlHeader<<8);
    SPI_writeDataNonBlocking(obj->spiHandle, 0xFF);
    SPI_writeDataNonBlocking(obj->spiHandle, 0xFF);

    // wait for three words to populate the RX fifo, or a wait timeout will occur
    while(RxFifoCnt < SPI_FIFO_RX3)
    {
        RxFifoCnt = SPI_getRxFIFOStatus(obj->spiHandle);

        if(++WaitTimeOut > 0xfffe)
        {
            obj->rxTimeOut = true;
        }
    }

    WaitTimeOut = 0xffff;

    GPIO_writePin(obj->gpioNumber_CS, 1);

    // Read the word
    readWord = SPI_readDataNonBlocking(obj->spiHandle);
    readWord1 = SPI_readDataNonBlocking(obj->spiHandle);
    readWord2 = SPI_readDataNonBlocking(obj->spiHandle);
    readWord = ((readWord1 & 0xFF) << 8) | (readWord2 & 0xFF); //concatenate D14:D0

    return(readWord & DRV8311_DATA_MASK);
} // end of DRV8311_readSPI() function


void DRV8311_writeSPI(DRV8311_Handle handle, const DRV8311_Address_e regAddr,
                      const uint16_t data)
{
    DRV8311_Obj *obj = (DRV8311_Obj *)handle;
    uint16_t ctrlHeader = 0, ctrlWord = 0;
    uint16_t n;

    volatile uint16_t WaitTimeOut = 0;

    volatile SPI_RxFIFOLevel RxFifoCnt = SPI_FIFO_RXEMPTY;

    // build the control header
    ctrlHeader = (uint16_t)DRV8311_buildCtrlHeader(DRV8311_CTRLMODE_WRITE, regAddr);
    // build the control word
    ctrlWord   = (uint16_t)DRV8311_buildCtrlWord(data);

    // reset the Rx fifo pointer to zero
    SPI_resetRxFIFO(obj->spiHandle);
    SPI_enableFIFO(obj->spiHandle);

    GPIO_writePin(obj->gpioNumber_CS, 0);

    // wait for GPIO
    for(n = 0; n < 0x06; n++)
    {
        __asm(" NOP");
    }

    // write the command //TODO
//    HWREGH(obj->spiHandle + SPI_O_TXBUF) = ctrlHeader << 8;
//    HWREGH(obj->spiHandle + SPI_O_TXBUF) = ctrlWord;
//    HWREGH(obj->spiHandle + SPI_O_TXBUF) = ctrlWord << 8;

    SPI_writeDataNonBlocking(obj->spiHandle, ctrlHeader<<8);
    SPI_writeDataNonBlocking(obj->spiHandle, ctrlWord);
    SPI_writeDataNonBlocking(obj->spiHandle, ctrlWord<<8);

    while(RxFifoCnt < SPI_FIFO_RX3)
    {
        RxFifoCnt = SPI_getRxFIFOStatus(obj->spiHandle);

        if(++WaitTimeOut > 0xfffe)
        {
            obj->rxTimeOut = true;
        }
    }

    WaitTimeOut = 0xffff;

    GPIO_writePin(obj->gpioNumber_CS, 1);

    return;
}  // end of DRV8311_writeSPI() function


void DRV8311_writeData(DRV8311_Handle handle, DRV8311_SPIVars_t *drv8311SPIVars)
{
    DRV8311_Address_e drvRegAddr;
    uint16_t drvDataNew;

    if(drv8311SPIVars->writeCmd)
    {
        // Write FLT_MODE
        drvRegAddr = DRV8311_ADDRESS_FLT_MODE;
        drvDataNew = (drv8311SPIVars->FLT_MODE_Reg_10.OTSD_MODE       ) | \
                     (drv8311SPIVars->FLT_MODE_Reg_10.UVP_MODE        ) | \
                     (drv8311SPIVars->FLT_MODE_Reg_10.OCP_MODE        ) | \
                     (drv8311SPIVars->FLT_MODE_Reg_10.SPI_FLT_MODE    ) | \
                     (drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV1 << 9) | \
                     (drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV2 << 10) | \
                     (drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV3 << 11) | \
                     (drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV4 << 12) | \
                     (drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV5 << 13) | \
                     (drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV6 << 14);
        DRV8311_writeSPI(handle, drvRegAddr, drvDataNew);

    // Write SYSF_CTRL
              drvRegAddr = DRV8311_ADDRESS_SYSF_CTRL;
              drvDataNew = (drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV1 << 0)  | \
                           (drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV2 << 1)  | \
                           (drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV3 << 2)  | \
                           (drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV4 << 3)  | \
                           (drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV5 << 4)  | \
                           (drv8311SPIVars->SYSF_CTRL_Reg_12.CSAVREFUV_EN       )  | \
                           (drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV6 << 6)  | \
                           (drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV7 << 7)  | \
                           (drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV8 << 8)  | \
                           (drv8311SPIVars->SYSF_CTRL_Reg_12.OTW_EN             )  | \
                           (drv8311SPIVars->SYSF_CTRL_Reg_12.OTAVDD_EN          )  | \
                           (drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV9 << 11)  | \
                           (drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV10 << 12)  | \
                           (drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV11 << 13)  | \
                           (drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV12 << 14);
              DRV8311_writeSPI(handle, drvRegAddr, drvDataNew);

      // Write DRVF_CTRL
                drvRegAddr = DRV8311_ADDRESS_DRVF_CTRL;
                drvDataNew = (drv8311SPIVars->DRVF_CTRL_Reg_13.OCP_LVL            )  | \
                             (drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV1 << 1)  | \
                             (drv8311SPIVars->DRVF_CTRL_Reg_13.OCP_TBLANK         )  | \
                             (drv8311SPIVars->DRVF_CTRL_Reg_13.OCP_DEG            )  | \
                             (drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV2 << 6)  | \
                             (drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV3 << 7)  | \
                             (drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV4 << 8)  | \
                             (drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV5 << 9)  | \
                             (drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV6 << 10)  | \
                             (drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV7 << 11)  | \
                             (drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV8 << 12)  | \
                             (drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV9 << 13)  | \
                             (drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV10 << 14);
                DRV8311_writeSPI(handle, drvRegAddr, drvDataNew);

    // Write FLT_TCTRL
              drvRegAddr = DRV8311_ADDRESS_FLT_TCTRL;
              drvDataNew = (drv8311SPIVars->FLT_TCTRL_Reg_16.FAST_TRETRY            )  | \
                           (drv8311SPIVars->FLT_TCTRL_Reg_16.SLOW_TRETRY            )  | \
                           (drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV1     << 4)  | \
                           (drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV2     << 5)  | \
                           (drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV3     << 6)  | \
                           (drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV4     << 7)  | \
                           (drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV5     << 8)  | \
                           (drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV6     << 9)  | \
                           (drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV7     << 10)  | \
                           (drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV8     << 11)  | \
                           (drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV9     << 12)  | \
                           (drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV10    << 13)  | \
                           (drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV11    << 14);
              DRV8311_writeSPI(handle, drvRegAddr, drvDataNew);

        // Write FLT_CLR
                  drvRegAddr = DRV8311_ADDRESS_FLT_CLR;
                  drvDataNew = (drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR           )  | \
                               (drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV1  << 1)  | \
                               (drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV2  << 2)  | \
                               (drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV3  << 3)  | \
                               (drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV4  << 4)  | \
                               (drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV5  << 5)  | \
                               (drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV6  << 6)  | \
                               (drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV7  << 7)  | \
                               (drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV8  << 8)  | \
                               (drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV9  << 9)  | \
                               (drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV10 << 10)  | \
                               (drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV11  << 11)  | \
                               (drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV12  << 12)  | \
                               (drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV13  << 13)  | \
                               (drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV14 << 14);
                  DRV8311_writeSPI(handle, drvRegAddr, drvDataNew);

      // Write PWMG_PERIOD
                drvRegAddr = DRV8311_ADDRESS_PWMG_PERIOD;
                drvDataNew = (drv8311SPIVars->PWMG_PERIOD_Reg_18.PWM_PRD_OUT           )  | \
                             (drv8311SPIVars->PWMG_PERIOD_Reg_18.PWMG_PERIOD_RSV1 << 12)  | \
                             (drv8311SPIVars->PWMG_PERIOD_Reg_18.PWMG_PERIOD_RSV2 << 13)  | \
                             (drv8311SPIVars->PWMG_PERIOD_Reg_18.PWMG_PERIOD_RSV3 << 14);
                DRV8311_writeSPI(handle, drvRegAddr, drvDataNew);

    // Write PWMG_A_DUTY
              drvRegAddr = DRV8311_ADDRESS_PWMG_A_DUTY;
              drvDataNew = (drv8311SPIVars->PWMG_A_DUTY_Reg_19.PWM_DUTY_OUTA           )  | \
                           (drv8311SPIVars->PWMG_A_DUTY_Reg_19.PWMG_A_DUTY_RSV1 << 12)  | \
                           (drv8311SPIVars->PWMG_A_DUTY_Reg_19.PWMG_A_DUTY_RSV1 << 13)  | \
                           (drv8311SPIVars->PWMG_A_DUTY_Reg_19.PWMG_A_DUTY_RSV1 << 14);
              DRV8311_writeSPI(handle, drvRegAddr, drvDataNew);

    // Write PWMG_B_DUTY
            drvRegAddr = DRV8311_ADDRESS_PWMG_B_DUTY;
            drvDataNew = (drv8311SPIVars->PWMG_B_DUTY_Reg_1A.PWM_DUTY_OUTB           )  | \
                         (drv8311SPIVars->PWMG_B_DUTY_Reg_1A.PWMG_B_DUTY_RSV1 << 12)  | \
                         (drv8311SPIVars->PWMG_B_DUTY_Reg_1A.PWMG_B_DUTY_RSV1 << 13)  | \
                         (drv8311SPIVars->PWMG_B_DUTY_Reg_1A.PWMG_B_DUTY_RSV1 << 14);
            DRV8311_writeSPI(handle, drvRegAddr, drvDataNew);

    // Write PWMG_C_DUTY
              drvRegAddr = DRV8311_ADDRESS_PWMG_C_DUTY;
              drvDataNew = (drv8311SPIVars->PWMG_C_DUTY_Reg_1B.PWM_DUTY_OUTC           )  | \
                           (drv8311SPIVars->PWMG_C_DUTY_Reg_1B.PWMG_C_DUTY_RSV1 << 12)  | \
                           (drv8311SPIVars->PWMG_C_DUTY_Reg_1B.PWMG_C_DUTY_RSV1 << 13)  | \
                           (drv8311SPIVars->PWMG_C_DUTY_Reg_1B.PWMG_C_DUTY_RSV1 << 14);
              DRV8311_writeSPI(handle, drvRegAddr, drvDataNew);


  // Write PWM_STATE
            drvRegAddr = DRV8311_ADDRESS_PWM_STATE;
            drvDataNew = (drv8311SPIVars->PWM_STATE_Reg_1C.PWMA_STATE         )   | \
                         (drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV1 << 3)   | \
                         (drv8311SPIVars->PWM_STATE_Reg_1C.PWMB_STATE         )   | \
                         (drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV2 << 7)   | \
                         (drv8311SPIVars->PWM_STATE_Reg_1C.PWMC_STATE         )   | \
                         (drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV3 << 11)  | \
                         (drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV4 << 12)  | \
                         (drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV5 << 13)  | \
                         (drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV6 << 14);
            DRV8311_writeSPI(handle, drvRegAddr, drvDataNew);

    // Write PWMG_CTRL
              drvRegAddr = DRV8311_ADDRESS_PWMG_CTRL;
              drvDataNew = (drv8311SPIVars->PWMG_CTRL_Reg_1D.SPISYNC_ACRCY        )  | \
                           (drv8311SPIVars->PWMG_CTRL_Reg_1D.SPICLK_FREQ_SYNC     )  | \
                           (drv8311SPIVars->PWMG_CTRL_Reg_1D.PWM_OSC_SYNC         )  | \
                           (drv8311SPIVars->PWMG_CTRL_Reg_1D.PWM_CNTR_MODE        )  | \
                           (drv8311SPIVars->PWMG_CTRL_Reg_1D.PWM_EN               )  | \
                           (drv8311SPIVars->PWMG_CTRL_Reg_1D.PWMG_CTRL_RSV1  << 11)  | \
                           (drv8311SPIVars->PWMG_CTRL_Reg_1D.PWMG_CTRL_RSV2  << 12)  | \
                           (drv8311SPIVars->PWMG_CTRL_Reg_1D.PWMG_CTRL_RSV3  << 13)  | \
                           (drv8311SPIVars->PWMG_CTRL_Reg_1D.PWMG_CTRL_RSV4  << 14);
              DRV8311_writeSPI(handle, drvRegAddr, drvDataNew);

  // Write PWM_CTRL1
            drvRegAddr = DRV8311_ADDRESS_PWM_CTRL1;
            drvDataNew = (drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_MODE           )  | \
                         (drv8311SPIVars->PWM_CTRL1_Reg_20.SSC_DIS            )  | \
                         (drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV1 << 3)  | \
                         (drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV2 << 4)  | \
                         (drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV3 << 5)  | \
                         (drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV4 << 6)  | \
                         (drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV5 << 7)  | \
                         (drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV6 << 8)  | \
                         (drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV7 << 9)  | \
                         (drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV8 << 10)  | \
                         (drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV9 << 11)  | \
                         (drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV10 << 12)  | \
                         (drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV11 << 13)  | \
                         (drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV12 << 14);
            DRV8311_writeSPI(handle, drvRegAddr, drvDataNew);

     // Write DRV_CTRL
              drvRegAddr = DRV8311_ADDRESS_DRV_CTRL;
              drvDataNew = (drv8311SPIVars->DRV_CTRL_Reg_22.SLEW_RATE             )  | \
                           (drv8311SPIVars->DRV_CTRL_Reg_22.DRV_CTRL_RSV1     << 2)  | \
                           (drv8311SPIVars->DRV_CTRL_Reg_22.DRV_CTRL_RSV2     << 3)  | \
                           (drv8311SPIVars->DRV_CTRL_Reg_22.TDEAD_CTRL            )  | \
                           (drv8311SPIVars->DRV_CTRL_Reg_22.DLYCMP_EN             )  | \
                           (drv8311SPIVars->DRV_CTRL_Reg_22.DLY_TARGET            )  | \
                           (drv8311SPIVars->DRV_CTRL_Reg_22.DRV_CTRL_RSV3     << 12) | \
                           (drv8311SPIVars->DRV_CTRL_Reg_22.DRV_CTRL_RSV4     << 13) | \
                           (drv8311SPIVars->DRV_CTRL_Reg_22.DRV_CTRL_RSV5     << 14);
              DRV8311_writeSPI(handle, drvRegAddr, drvDataNew);

  // Write CSA_CTRL
           drvRegAddr = DRV8311_ADDRESS_CSA_CTRL;
           drvDataNew = (drv8311SPIVars->CSA_CTRL_Reg_23.CSA_GAIN          )  | \
                        (drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV1 << 2)  | \
                        (drv8311SPIVars->CSA_CTRL_Reg_23.CSA_EN            )  | \
                        (drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV2 << 4)  | \
                        (drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV3 << 5)  | \
                        (drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV4 << 6)  | \
                        (drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV5 << 7)  | \
                        (drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV6 << 8)  | \
                        (drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV7 << 9)  | \
                        (drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV8 << 10)  | \
                        (drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV9 << 11)  | \
                        (drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV10 << 12)  | \
                        (drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV11 << 13)  | \
                        (drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV12 << 14);
           DRV8311_writeSPI(handle, drvRegAddr, drvDataNew);

   // Write SYS_CTRL
            drvRegAddr = DRV8311_ADDRESS_SYS_CTRL;
            drvDataNew = (drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV1      << 0)  | \
                         (drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV2      << 1)  | \
                         (drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV3      << 2)  | \
                         (drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV4      << 3)  | \
                         (drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV5      << 4)  | \
                         (drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV6      << 5)  | \
                         (drv8311SPIVars->SYS_CTRL_Reg_3F.SPI_PEN                )  | \
                         (drv8311SPIVars->SYS_CTRL_Reg_3F.REG_LOCK               )  | \
                         (drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV7      << 8)  | \
                         (drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV8      << 9)  | \
                         (drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV9      << 10)  | \
                         (drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV10     << 11)  | \
                         (drv8311SPIVars->SYS_CTRL_Reg_3F.WRITE_KEY     );
            DRV8311_writeSPI(handle, drvRegAddr, drvDataNew);

        drv8311SPIVars->writeCmd = false;
    }

    // Manual write to the drv8311
    if(drv8311SPIVars->manWriteCmd)
    {
        // Custom Write
        drvRegAddr = (DRV8311_Address_e)(drv8311SPIVars->manWriteAddr<<1);
        drvDataNew = drv8311SPIVars->manWriteData;

        DRV8311_writeSPI(handle, drvRegAddr, drvDataNew);

        drv8311SPIVars->manWriteCmd = false;
    }

    return;
}  // end of DRV8311_writeData() function

void DRV8311_readData(DRV8311_Handle handle, DRV8311_SPIVars_t *drv8311SPIVars)
{
    DRV8311_Address_e drvRegAddr;
    uint16_t drvDataNew;

    if(drv8311SPIVars->readCmd)
    {
        // Read registers for default values
        // Read Status Register 0
        drvRegAddr = DRV8311_ADDRESS_DEV_STS1;
        drvDataNew = DRV8311_readSPI(handle, drvRegAddr);

        drv8311SPIVars->DEV_STS1_Reg_00.FAULT         = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_FAULT_BITS)?1:0;
        drv8311SPIVars->DEV_STS1_Reg_00.OT            = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_OT_BITS)?1:0;
        drv8311SPIVars->DEV_STS1_Reg_00.UVP           = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_UVP_BITS)?1:0;
        drv8311SPIVars->DEV_STS1_Reg_00.DEV_STS1_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESERVED1_BITS)?1:0;
        drv8311SPIVars->DEV_STS1_Reg_00.DEV_STS1_RSV2 = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESERVED2_BITS)?1:0;
        drv8311SPIVars->DEV_STS1_Reg_00.OCP           = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_OCP_BITS)?1:0;
        drv8311SPIVars->DEV_STS1_Reg_00.SPI_FLT       = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_SPI_FLT_BITS)?1:0;
        drv8311SPIVars->DEV_STS1_Reg_00.NPOR          = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESET_BITS)?1:0;
        drv8311SPIVars->DEV_STS1_Reg_00.OTP_FLT       = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_OTP_FLT_BITS)?1:0;
        drv8311SPIVars->DEV_STS1_Reg_00.DEV_STS1_RSV3 = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESERVED3_BITS)?1:0;
        drv8311SPIVars->DEV_STS1_Reg_00.DEV_STS1_RSV4 = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESERVED4_BITS)?1:0;
        drv8311SPIVars->DEV_STS1_Reg_00.DEV_STS1_RSV5 = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESERVED5_BITS)?1:0;
        drv8311SPIVars->DEV_STS1_Reg_00.DEV_STS1_RSV6 = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESERVED6_BITS)?1:0;
        drv8311SPIVars->DEV_STS1_Reg_00.DEV_STS1_RSV7 = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESERVED7_BITS)?1:0;
        drv8311SPIVars->DEV_STS1_Reg_00.DEV_STS1_RSV8 = (bool)(drvDataNew & (uint16_t)DRV8311_DEV_STS1_RESERVED8_BITS)?1:0;

        // Read OT_STS
        drvRegAddr = DRV8311_ADDRESS_OT_STS;
        drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
        drv8311SPIVars->OT_STS_Reg_04.OTSD           = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_OTSD_BITS)?1:0;
        drv8311SPIVars->OT_STS_Reg_04.OTW            = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_OTW_BITS)?1:0;
        drv8311SPIVars->OT_STS_Reg_04.OTS_AVDD       = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_OTS_AVDD_BITS)?1:0;
        drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV1    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED1_BITS)?1:0;
        drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV2    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED2_BITS)?1:0;
        drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV3    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED3_BITS)?1:0;
        drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV4    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED4_BITS)?1:0;
        drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV5    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED5_BITS)?1:0;
        drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV6    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED6_BITS)?1:0;
        drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV7    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED7_BITS)?1:0;
        drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV8    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED8_BITS)?1:0;
        drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV9    = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED9_BITS)?1:0;
        drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV10   = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED10_BITS)?1:0;
        drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV11   = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED11_BITS)?1:0;
        drv8311SPIVars->OT_STS_Reg_04.OT_STS_RSV12   = (bool)(drvDataNew & (uint16_t)DRV8311_OT_STS_RESERVED12_BITS)?1:0;

        // Read SUP_STS
        drvRegAddr = DRV8311_ADDRESS_SUP_STS;
        drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
        drv8311SPIVars->SUP_STS_Reg_05.VINAVDD_UV    = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_VINAVDD_UV_BITS)?1:0;
        drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED1_BITS)?1:0;
        drv8311SPIVars->SUP_STS_Reg_05.AVDD_UV       = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_AVDD_UV_BITS)?1:0;
        drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED2_BITS)?1:0;
        drv8311SPIVars->SUP_STS_Reg_05.CP_UV         = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_CP_UV_BITS)?1:0;
        drv8311SPIVars->SUP_STS_Reg_05.CSA_REF_UV    = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_CSAREF_UV_BITS)?1:0;
        drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED3_BITS)?1:0;
        drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV4  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED4_BITS)?1:0;
        drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV5  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED5_BITS)?1:0;
        drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV6  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED6_BITS)?1:0;
        drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV7  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED7_BITS)?1:0;
        drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV8  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED8_BITS)?1:0;
        drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV9  = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED9_BITS)?1:0;
        drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV10 = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED10_BITS)?1:0;
        drv8311SPIVars->SUP_STS_Reg_05.SUP_STS_RSV11 = (bool)(drvDataNew & (uint16_t)DRV8311_SUP_STS_RESERVED11_BITS)?1:0;

        // Read DRV_STS
        drvRegAddr = DRV8311_ADDRESS_DRV_STS;
        drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
        drv8311SPIVars->DRV_STS_Reg_06.OCPA_LS      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_OCPA_LS_BITS)?1:0;
        drv8311SPIVars->DRV_STS_Reg_06.OCPB_LS      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_OCPB_LS_BITS)?1:0;
        drv8311SPIVars->DRV_STS_Reg_06.OCPC_LS      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_OCPC_LS_BITS)?1:0;
        drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED1_BITS)?1:0;
        drv8311SPIVars->DRV_STS_Reg_06.OCPA_HS      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_OCPA_HS_BITS)?1:0;
        drv8311SPIVars->DRV_STS_Reg_06.OCPB_HS      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_OCPB_HS_BITS)?1:0;
        drv8311SPIVars->DRV_STS_Reg_06.OCPC_HS      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_OCPC_HS_BITS)?1:0;
        drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV2 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED2_BITS)?1:0;
        drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV3 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED3_BITS)?1:0;
        drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV4 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED4_BITS)?1:0;
        drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV5 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED5_BITS)?1:0;
        drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV6 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED6_BITS)?1:0;
        drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV7 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED7_BITS)?1:0;
        drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV8 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED8_BITS)?1:0;
        drv8311SPIVars->DRV_STS_Reg_06.DRV_STS_RSV9 = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_STS_RESERVED9_BITS)?1:0;

        // Read SYS_STS
        drvRegAddr = DRV8311_ADDRESS_SYS_STS;
        drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
        drv8311SPIVars->SYS_STS_Reg_07.FRAME_ERR      = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_FRM_ERR_BITS)?1:0;
        drv8311SPIVars->SYS_STS_Reg_07.BUS_CONTENTION = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_BUS_CNT_BITS)?1:0;
        drv8311SPIVars->SYS_STS_Reg_07.SPI_PARITY_ERR = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_SPI_PARITY)?1:0;
        drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV1   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED1_BITS)?1:0;
        drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV2   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED2_BITS)?1:0;
        drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV3   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED3_BITS)?1:0;
        drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV4   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED4_BITS)?1:0;
        drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV5   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED5_BITS)?1:0;
        drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV6   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED6_BITS)?1:0;
        drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV7   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED7_BITS)?1:0;
        drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV8   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED8_BITS)?1:0;
        drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV9   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED9_BITS)?1:0;
        drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV10  = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED10_BITS)?1:0;
        drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV11  = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED11_BITS)?1:0;
        drv8311SPIVars->SYS_STS_Reg_07.SYS_STS_RSV12  = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_STS_RESERVED12_BITS)?1:0;

        // Read PWM_SYNC_PRD
        drvRegAddr = DRV8311_ADDRESS_PWM_SYNC_PRD;
        drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
        drv8311SPIVars->PWM_SYNC_PRD_Reg_0C.PWM_SYNC_PRD       = (uint16_t)(drvDataNew & (uint16_t)DRV8311_PWM_SYNC_PRD_PWM_SYNC_PRD_BITS);
        drv8311SPIVars->PWM_SYNC_PRD_Reg_0C.PWM_SYNC_PRD_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_SYNC_PRD_RESERVED1_BITS)?1:0;
        drv8311SPIVars->PWM_SYNC_PRD_Reg_0C.PWM_SYNC_PRD_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_SYNC_PRD_RESERVED2_BITS)?1:0;
        drv8311SPIVars->PWM_SYNC_PRD_Reg_0C.PWM_SYNC_PRD_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_SYNC_PRD_RESERVED3_BITS)?1:0;

        // Read FLT_MODE
        drvRegAddr = DRV8311_ADDRESS_FLT_MODE;
        drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
       drv8311SPIVars->FLT_MODE_Reg_10.OTSD_MODE        = (DRV8311_FLT_MODE_OTSD_MODE_e)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_OTSD_MODE_BITS);
       drv8311SPIVars->FLT_MODE_Reg_10.UVP_MODE         = (DRV8311_FLT_MODE_UVP_MODE_e)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_UVP_MODE_BITS);
       drv8311SPIVars->FLT_MODE_Reg_10.OCP_MODE         = (DRV8311_FLT_MODE_OCP_MODE_e)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_OCP_MODE_BITS);
       drv8311SPIVars->FLT_MODE_Reg_10.SPI_FLT_MODE     = (DRV8311_FLT_MODE_SPIFLT_MODE_e)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_SPIFLT_MODE_BITS);
       drv8311SPIVars->FLT_MODE_Reg_10.OTP_FLT_MODE     = (DRV8311_FLT_MODE_OTPFLT_MODE_e)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_OTPFLT_MODE_BITS);
       drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV1    = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_RESERVED1_BITS)?1:0;
       drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV2    = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_RESERVED2_BITS)?1:0;
       drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV3    = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_RESERVED3_BITS)?1:0;
       drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV4    = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_RESERVED4_BITS)?1:0;
       drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV5    = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_RESERVED5_BITS)?1:0;
       drv8311SPIVars->FLT_MODE_Reg_10.FLT_MODE_RSV6    = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_MODE_RESERVED6_BITS)?1:0;

       // Read SYSF_CTRL
       drvRegAddr = DRV8311_ADDRESS_SYSF_CTRL;
       drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
       drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV1     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED1_BITS);
       drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV2     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED2_BITS)?1:0;
       drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV3     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED3_BITS)?1:0;
       drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV4     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED4_BITS)?1:0;
       drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV5     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED5_BITS)?1:0;
       drv8311SPIVars->SYSF_CTRL_Reg_12.CSAVREFUV_EN       = (DRV8311_SYSF_CTRL_CSAVREFUV_EN_e)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_CSAVREFUV_EN_BITS);
       drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV6     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED6_BITS)?1:0;
       drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV7     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED7_BITS)?1:0;
       drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV8     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED8_BITS)?1:0;
       drv8311SPIVars->SYSF_CTRL_Reg_12.OTW_EN             = (DRV8311_SYSF_CTRL_OTW_EN_e)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_OTW_EN_BITS);
       drv8311SPIVars->SYSF_CTRL_Reg_12.OTAVDD_EN          = (DRV8311_SYSF_CTRL_OTAVDD_EN_e)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_OTAVDD_EN_BITS);
       drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV9     = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED9_BITS)?1:0;
       drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV10    = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED10_BITS)?1:0;
       drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV11    = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED11_BITS)?1:0;
       drv8311SPIVars->SYSF_CTRL_Reg_12.SYSF_CTRL_RSV12    = (bool)(drvDataNew & (uint16_t)DRV8311_SYSF_CTRL_RESERVED12_BITS)?1:0;

       // Read DRVF_CTRL
       drvRegAddr = DRV8311_ADDRESS_DRVF_CTRL;
       drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
       drv8311SPIVars->DRVF_CTRL_Reg_13.OCP_LVL          = (DRV8311_DRVF_CTRL_OCP_LVL_e)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_OCP_LVL_BITS);
       drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV1   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED1_BITS)?1:0;
       drv8311SPIVars->DRVF_CTRL_Reg_13.OCP_TBLANK       = (DRV8311_DRVF_CTRL_OCP_TBLANK_e)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_OCP_TBLANK_BITS);
       drv8311SPIVars->DRVF_CTRL_Reg_13.OCP_DEG          = (DRV8311_DRVF_CTRL_OCP_DEG_e)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_OCP_DEG_BITS);
       drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV2   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED2_BITS)?1:0;
       drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV3   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED3_BITS)?1:0;
       drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV4   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED4_BITS)?1:0;
       drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV5   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED5_BITS)?1:0;
       drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV6   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED6_BITS)?1:0;
       drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV7   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED7_BITS)?1:0;
       drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV8   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED8_BITS)?1:0;
       drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV9   = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED9_BITS)?1:0;
       drv8311SPIVars->DRVF_CTRL_Reg_13.DRVF_CTRL_RSV10  = (bool)(drvDataNew & (uint16_t)DRV8311_DRVF_CTRL_RESERVED10_BITS)?1:0;

       // Read FLT_TCTRL
       drvRegAddr = DRV8311_ADDRESS_FLT_TCTRL;
       drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
         drv8311SPIVars->FLT_TCTRL_Reg_16.FAST_TRETRY      = (DRV8311_FLT_TCTRL_FAST_TRETRY_e)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_FAST_TRETRY_BITS);
         drv8311SPIVars->FLT_TCTRL_Reg_16.SLOW_TRETRY      = (DRV8311_FLT_TCTRL_SLOW_TRETRY_e)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_SLOW_TRETRY_BITS);
         drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV1   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED1_BITS)?1:0;
         drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV2   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED1_BITS)?1:0;
         drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV3   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED2_BITS)?1:0;
         drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV4   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED3_BITS)?1:0;
         drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV5   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED4_BITS)?1:0;
         drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV6   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED5_BITS)?1:0;
         drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV7   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED6_BITS)?1:0;
         drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV8   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED7_BITS)?1:0;
         drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV9   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED8_BITS)?1:0;
         drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV10  = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED9_BITS)?1:0;
         drv8311SPIVars->FLT_TCTRL_Reg_16.FLT_TCTRL_RSV11  = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_RESERVED10_BITS)?1:0;

     // Read FLT_CLR
       drvRegAddr = DRV8311_ADDRESS_FLT_CLR;
       drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
       drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR        = (DRV8311_FLT_CLR_e)(drvDataNew & (uint16_t)DRV8311_FLT_TCTRL_FAST_TRETRY_BITS);
       drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV1   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED1_BITS)?1:0;
       drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV2   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED2_BITS)?1:0;
       drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV3   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED3_BITS)?1:0;
       drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV4   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED4_BITS)?1:0;
       drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV5   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED5_BITS)?1:0;
       drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV6   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED6_BITS)?1:0;
       drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV7   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED7_BITS)?1:0;
       drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV8   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED8_BITS)?1:0;
       drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV9   = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED9_BITS)?1:0;
       drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV10  = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED10_BITS)?1:0;
       drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV11  = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED11_BITS)?1:0;
       drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV12  = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED12_BITS)?1:0;
       drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV13  = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED13_BITS)?1:0;
       drv8311SPIVars->FLT_CLR_Reg_17.FLT_CLR_RSV14  = (bool)(drvDataNew & (uint16_t)DRV8311_FLT_CLR_RESERVED14_BITS)?1:0;

       // Read PWMG_PERIOD
       drvRegAddr = DRV8311_ADDRESS_PWMG_PERIOD;
       drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
       drv8311SPIVars->PWMG_PERIOD_Reg_18.PWM_PRD_OUT       = (uint16_t)(drvDataNew & (uint16_t)DRV8311_PWMG_PERIOD_PWM_PRD_OUT_BITS);
       drv8311SPIVars->PWMG_PERIOD_Reg_18.PWMG_PERIOD_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_PERIOD_RESERVED1_BITS)?1:0;
       drv8311SPIVars->PWMG_PERIOD_Reg_18.PWMG_PERIOD_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_PERIOD_RESERVED2_BITS)?1:0;
       drv8311SPIVars->PWMG_PERIOD_Reg_18.PWMG_PERIOD_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_PERIOD_RESERVED3_BITS)?1:0;

       // Read PWMG_A_DUTY
       drvRegAddr = DRV8311_ADDRESS_PWMG_A_DUTY;
       drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
       drv8311SPIVars->PWMG_A_DUTY_Reg_19.PWM_DUTY_OUTA       = (uint16_t)(drvDataNew & (uint16_t)DRV8311_PWMG_A_DUTY_PWM_DUTY_OUTA_BITS);
       drv8311SPIVars->PWMG_A_DUTY_Reg_19.PWMG_A_DUTY_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_A_DUTY_RESERVED1_BITS)?1:0;
       drv8311SPIVars->PWMG_A_DUTY_Reg_19.PWMG_A_DUTY_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_A_DUTY_RESERVED2_BITS)?1:0;
       drv8311SPIVars->PWMG_A_DUTY_Reg_19.PWMG_A_DUTY_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_A_DUTY_RESERVED3_BITS)?1:0;

       // Read PWMG_B_DUTY
       drvRegAddr = DRV8311_ADDRESS_PWMG_B_DUTY;
       drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
        drv8311SPIVars->PWMG_B_DUTY_Reg_1A.PWM_DUTY_OUTB       = (uint16_t)(drvDataNew & (uint16_t)DRV8311_PWMG_B_DUTY_PWM_DUTY_OUTB_BITS);
        drv8311SPIVars->PWMG_B_DUTY_Reg_1A.PWMG_B_DUTY_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_B_DUTY_RESERVED1_BITS)?1:0;
        drv8311SPIVars->PWMG_B_DUTY_Reg_1A.PWMG_B_DUTY_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_B_DUTY_RESERVED2_BITS)?1:0;
        drv8311SPIVars->PWMG_B_DUTY_Reg_1A.PWMG_B_DUTY_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_B_DUTY_RESERVED3_BITS)?1:0;

        // Read PWMG_C_DUTY
        drvRegAddr = DRV8311_ADDRESS_PWMG_C_DUTY;
        drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
         drv8311SPIVars->PWMG_C_DUTY_Reg_1B.PWM_DUTY_OUTC       = (uint16_t)(drvDataNew & (uint16_t)DRV8311_PWMG_C_DUTY_PWM_DUTY_OUTC_BITS);
         drv8311SPIVars->PWMG_C_DUTY_Reg_1B.PWMG_C_DUTY_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_C_DUTY_RESERVED1_BITS)?1:0;
         drv8311SPIVars->PWMG_C_DUTY_Reg_1B.PWMG_C_DUTY_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_C_DUTY_RESERVED2_BITS)?1:0;
         drv8311SPIVars->PWMG_C_DUTY_Reg_1B.PWMG_C_DUTY_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_C_DUTY_RESERVED3_BITS)?1:0;

         // Read PWM_STATE
         drvRegAddr = DRV8311_ADDRESS_PWM_STATE;
         drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
          drv8311SPIVars->PWM_STATE_Reg_1C.PWMA_STATE      = (DRV8311_PWM_STATE_PWMA_STATE_e)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_PWMA_STATE_BITS);
          drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_RESERVED1_BITS)?1:0;
          drv8311SPIVars->PWM_STATE_Reg_1C.PWMB_STATE      = (DRV8311_PWM_STATE_PWMB_STATE_e)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_PWMB_STATE_BITS);
          drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_RESERVED2_BITS)?1:0;
          drv8311SPIVars->PWM_STATE_Reg_1C.PWMC_STATE      = (DRV8311_PWM_STATE_PWMC_STATE_e)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_PWMC_STATE_BITS);
          drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_RESERVED3_BITS)?1:0;
          drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV4  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_RESERVED4_BITS)?1:0;
          drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV5  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_RESERVED5_BITS)?1:0;
          drv8311SPIVars->PWM_STATE_Reg_1C.PWM_STATE_RSV6  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_STATE_RESERVED6_BITS)?1:0;

         // Read PWMG_CTRL
          drvRegAddr = DRV8311_ADDRESS_PWMG_CTRL;
          drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
         drv8311SPIVars->PWMG_CTRL_Reg_1D.SPISYNC_ACRCY      = (DRV8311_PWMG_CTRL_SPISYNC_ACRCY_e)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_SPISYNC_ACRCY_BITS);
         drv8311SPIVars->PWMG_CTRL_Reg_1D.SPICLK_FREQ_SYNC   = (DRV8311_PWMG_CTRL_SPICLK_FREQ_SYNC_e)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_SPICLK_FREQ_SYNC_BITS);
         drv8311SPIVars->PWMG_CTRL_Reg_1D.PWM_OSC_SYNC       = (DRV8311_PWMG_CTRL_PWM_OSC_SYNC_e)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_PWM_OSC_SYNC_BITS);
         drv8311SPIVars->PWMG_CTRL_Reg_1D.PWM_CNTR_MODE      = (DRV8311_PWMG_CTRL_PWM_CNTR_MODE_e)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_PWMCNTR_MODE_BITS);
         drv8311SPIVars->PWMG_CTRL_Reg_1D.PWM_EN             = (DRV8311_PWMG_CTRL_PWM_EN_e)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_PWM_EN_BITS);
         drv8311SPIVars->PWMG_CTRL_Reg_1D.PWMG_CTRL_RSV1     = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_RESERVED1_BITS)?1:0;
         drv8311SPIVars->PWMG_CTRL_Reg_1D.PWMG_CTRL_RSV2     = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_RESERVED2_BITS)?1:0;
         drv8311SPIVars->PWMG_CTRL_Reg_1D.PWMG_CTRL_RSV3     = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_RESERVED3_BITS)?1:0;
         drv8311SPIVars->PWMG_CTRL_Reg_1D.PWMG_CTRL_RSV4     = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_CTRL_RESERVED4_BITS)?1:0;

         // Read PWM_CTRL1
         drvRegAddr = DRV8311_ADDRESS_PWM_CTRL1;
         drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
         drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_MODE          = (DRV8311_PWM_CTRL1_PWM_MODE_e)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_PWM_MODE_BITS);
         drv8311SPIVars->PWM_CTRL1_Reg_20.SSC_DIS           = (DRV8311_PWM_CTRL1_SSC_DIS_e)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_SSC_DIS_BITS);
         drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV1    = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_RESERVED1_BITS)?1:0;
         drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV2    = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_RESERVED2_BITS)?1:0;
         drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV3    = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_RESERVED3_BITS)?1:0;
         drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV4    = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_RESERVED4_BITS)?1:0;
         drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV5    = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_RESERVED5_BITS)?1:0;
         drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV6    = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_RESERVED6_BITS)?1:0;
         drv8311SPIVars->PWM_CTRL1_Reg_20.PWM_CTRL1_RSV7    = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_CTRL1_RESERVED7_BITS)?1:0;

         // Read DRV_CTRL
         drvRegAddr = DRV8311_ADDRESS_DRV_CTRL;
         drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
         drv8311SPIVars->DRV_CTRL_Reg_22.SLEW_RATE          = (DRV8311_DRV_CTRL_SLEW_RATE_e)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_SLEW_RATE_BITS);
         drv8311SPIVars->DRV_CTRL_Reg_22.DRV_CTRL_RSV1      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_RESERVED1_BITS)?1:0;
         drv8311SPIVars->DRV_CTRL_Reg_22.DRV_CTRL_RSV2      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_RESERVED2_BITS)?1:0;
         drv8311SPIVars->DRV_CTRL_Reg_22.TDEAD_CTRL         = (DRV8311_DRV_CTRL_TDEAD_CTRL_e)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_TDEAD_CTRL_BITS);
         drv8311SPIVars->DRV_CTRL_Reg_22.DLYCMP_EN          = (DRV8311_DRV_CTRL_DLYCMP_EN_e)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_DLYCMP_EN_BITS);
         drv8311SPIVars->DRV_CTRL_Reg_22.DLY_TARGET         = (uint16_t)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_DLY_TARGET_BITS);
         drv8311SPIVars->DRV_CTRL_Reg_22.DRV_CTRL_RSV3      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_RESERVED3_BITS)?1:0;
         drv8311SPIVars->DRV_CTRL_Reg_22.DRV_CTRL_RSV4      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_RESERVED4_BITS)?1:0;
         drv8311SPIVars->DRV_CTRL_Reg_22.DRV_CTRL_RSV5      = (bool)(drvDataNew & (uint16_t)DRV8311_DRV_CTRL_RESERVED5_BITS)?1:0;

         // Read CSA_CTRL
         drvRegAddr = DRV8311_ADDRESS_CSA_CTRL;
         drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
         drv8311SPIVars->CSA_CTRL_Reg_23.CSA_GAIN         = (DRV8311_CSA_CTRL_CSA_GAIN_e)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_CSA_GAIN_BITS);
         drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV1    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED1_BITS)?1:0;
         drv8311SPIVars->CSA_CTRL_Reg_23.CSA_EN           = (DRV8311_CSA_CTRL_CSA_EN_e)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_CSA_EN_BITS);
         drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV2    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED2_BITS)?1:0;
         drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV3    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED3_BITS)?1:0;
         drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV4    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED4_BITS)?1:0;
         drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV5    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED5_BITS)?1:0;
         drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV6    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED6_BITS)?1:0;
         drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV7    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED7_BITS)?1:0;
         drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV8    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED8_BITS)?1:0;
         drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV9    = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED9_BITS)?1:0;
         drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV10   = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED10_BITS)?1:0;
         drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV11   = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED11_BITS)?1:0;
         drv8311SPIVars->CSA_CTRL_Reg_23.CSA_CTRL_RSV12   = (bool)(drvDataNew & (uint16_t)DRV8311_CSA_CTRL_RESERVED12_BITS)?1:0;

         // Read SYS_CTRL
         drvRegAddr = DRV8311_ADDRESS_SYS_CTRL;
         drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
          drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV1    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED1_BITS)?1:0;
          drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV2    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED2_BITS)?1:0;
          drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV3    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED3_BITS)?1:0;
          drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV4    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED4_BITS)?1:0;
          drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV5    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED5_BITS)?1:0;
          drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV6    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED6_BITS)?1:0;
          drv8311SPIVars->SYS_CTRL_Reg_3F.SPI_PEN          = (DRV8311_SYS_CTRL_SPI_PEN_e)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_SPI_PEN_BITS);
          drv8311SPIVars->SYS_CTRL_Reg_3F.REG_LOCK         = (DRV8311_SYS_CTRL_REG_LOCK_e)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_REG_LOCK_BITS);
          drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV7    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED7_BITS)?1:0;
          drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV8    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED8_BITS)?1:0;
          drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV9    = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED9_BITS)?1:0;
          drv8311SPIVars->SYS_CTRL_Reg_3F.SYS_CTRL_RSV10   = (bool)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_RESERVED10_BITS)?1:0;
          drv8311SPIVars->SYS_CTRL_Reg_3F.WRITE_KEY = (DRV8311_SYS_CTRL_WRITE_KEY_e)(drvDataNew & (uint16_t)DRV8311_SYS_CTRL_WRITE_KEY_BITS);

        drv8311SPIVars->readCmd = false;
    }

    // Manual read from the drv8311
    if(drv8311SPIVars->manReadCmd)
    {
        // Custom Read
        drvRegAddr = (DRV8311_Address_e)(drv8311SPIVars->manReadAddr<<1);
        drvDataNew = DRV8311_readSPI(handle, drvRegAddr);
        drv8311SPIVars->manReadData = drvDataNew;

        drv8311SPIVars->manReadCmd = false;
    }

    return;
}  // end of DRV8311_readData() function

// end of file
