
#include "drv8311.hpp"
#include "utils.hpp"
#include "cmsis_os.h"
#include "board.h"

const SPI_InitTypeDef Drv8311::spi_config_ = {
    .Mode = SPI_MODE_MASTER,
    .Direction = SPI_DIRECTION_2LINES,
    .DataSize = SPI_DATASIZE_16BIT,
    .CLKPolarity = SPI_POLARITY_LOW,
    .CLKPhase = SPI_PHASE_2EDGE,
    .NSS = SPI_NSS_SOFT,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256,
    .FirstBit = SPI_FIRSTBIT_MSB,
    .TIMode = SPI_TIMODE_DISABLE,
    .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
    .CRCPolynomial = 10,
};

bool Drv8311::config(float requested_gain, float* actual_gain) {
    // Calculate gain setting: Snap down to have equal or larger range as
    // requested or largest possible range otherwise

    // for reference:
    // 20V/V on 500uOhm gives a range of +/- 150A
    // 40V/V on 500uOhm gives a range of +/- 75A
    // 20V/V on 666uOhm gives a range of +/- 110A
    // 40V/V on 666uOhm gives a range of +/- 55A

    uint16_t gain_setting = 3;
    float gain_choices[] = {10.0f, 20.0f, 40.0f, 80.0f};
    while (gain_setting && (gain_choices[gain_setting] > requested_gain)) {
        gain_setting--;
    }

    if (actual_gain) {
        *actual_gain = gain_choices[gain_setting];
    }

    RegisterFile new_config;

    new_config.control_register_1 =
          (21 << 6) // Overcurrent set to approximately 150A at 100degC. This may need tweaking.
        | (0b01 << 4) // OCP_MODE: latch shut down
        | (0b0 << 3) // 6x PWM mode
        | (0b0 << 2) // don't reset latched faults
        | (0b00 << 0); // gate-drive peak current: 1.7A

    new_config.control_register_2 =
          (0b0 << 6) // OC_TOFF: cycle by cycle
        | (0b00 << 4) // calibration off (normal operation)
        | (gain_setting << 2) // select gain
        | (0b00 << 0); // report both over temperature and over current on nOCTW pin

    bool regs_equal = (regs_.control_register_1 == new_config.control_register_1)
                   && (regs_.control_register_2 == new_config.control_register_2);

    if (!regs_equal) {
        regs_ = new_config;
        state_ = kStateUninitialized;
        enable_gpio_.write(false);
    }

    return true;
}

bool Drv8311::init() {
    uint16_t val;

    if (state_ == kStateReady) {
        return true;
    }

    // Reset DRV chip. The enable pin also controls the SPI interface, not only
    // the driver stages.
    enable_gpio_.write(false);
    delay_us(40); // mimumum pull-down time for full reset: 20us
    state_ = kStateUninitialized; // make is_ready() ignore transient errors before registers are set up
    enable_gpio_.write(true);
    osDelay(20); // t_spi_ready, max = 10ms

    // // Write current configuration
    // bool wrote_regs = write_reg(kRegNameControl1, regs_.control_register_1)
    //                    && write_reg(kRegNameControl1, regs_.control_register_1)
    //                    && write_reg(kRegNameControl1, regs_.control_register_1)
    //                    && write_reg(kRegNameControl1, regs_.control_register_1)
    //                    && write_reg(kRegNameControl1, regs_.control_register_1) // the write operation tends to be ignored if only done once (not sure why)
    //                    && write_reg(kRegNameControl2, regs_.control_register_2);
    // if (!wrote_regs) {
    //     return false;
    // }
    DRV8311_SPIVars_t vars;
    setupSPI(&vars);

    // Wait for configuration to be applied
    delay_us(100);
    state_ = kStateStartupChecks;

    bool is_read_regs = read_reg(kRegNameControl1, &val) && (val == regs_.control_register_1)
                      && read_reg(kRegNameControl2, &val) && (val == regs_.control_register_2);
    if (!is_read_regs) {
        return false;
    }

    if (get_error() != FaultType_NoFault) {
        return false;
    }

    // There could have been an nFAULT edge meanwhile. In this case we shouldn't
    // consider the driver ready.
    CRITICAL_SECTION() {
        if (state_ == kStateStartupChecks) {
            state_ = kStateReady;
        }
    }

    return state_ == kStateReady;
}

void Drv8311::do_checks() {
    if (state_ != kStateUninitialized && !nfault_gpio_.read()) {
        state_ = kStateUninitialized;
    }
}

bool Drv8311::is_ready() {
    return state_ == kStateReady;
}

Drv8311::FaultType_e Drv8311::get_error() {
    uint16_t fault1, fault2;

    if (!read_reg(kRegNameStatus1, &fault1) ||
        !read_reg(kRegNameStatus2, &fault2)) {
        return (FaultType_e)0xffffffff;
    }

    return (FaultType_e)((uint32_t)fault1 | ((uint32_t)(fault2 & 0x0080) << 16));
}

bool Drv8311::read_reg(const RegName_e regName, uint16_t* data) {
    tx_buf_ = build_ctrl_word(DRV8311_CtrlMode_Read, regName, 0);
    if (!spi_arbiter_->transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), nullptr, 1, 1000)) {
        return false;
    }
    
    delay_us(1);

    tx_buf_ = build_ctrl_word(DRV8311_CtrlMode_Read, regName, 0);
    rx_buf_ = 0xffff;
    if (!spi_arbiter_->transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), (uint8_t *)(&rx_buf_), 1, 1000)) {
        return false;
    }

    delay_us(1);

    if (rx_buf_ == 0xbeef) {
        return false;
    }

    if (data) {
        *data = rx_buf_ & 0x07FF;
    }
    
    return true;
}

bool Drv8311::write_reg(const RegName_e regName, const uint16_t data) {
    // Do blocking write
    tx_buf_ = build_ctrl_word(DRV8311_CtrlMode_Write, regName, data);
    if (!spi_arbiter_->transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), nullptr, 1, 1000)) {
        return false;
    }
    delay_us(1);

    return true;
}

void Drv8311::setupSPI(DRV8311_SPIVars_t *drv8311SPIVars){
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
    
    drvDataNew = readSPI(drvRegAddr);

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
    drvDataNew = readSPI(drvRegAddr);
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
    drvDataNew = readSPI(drvRegAddr);
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
    drvDataNew = readSPI(drvRegAddr);
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
    drvDataNew = readSPI(drvRegAddr);
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
    drvDataNew = readSPI(drvRegAddr);
    drv8311SPIVars->PWM_SYNC_PRD_Reg_0C.PWM_SYNC_PRD       = (uint16_t)(drvDataNew & (uint16_t)DRV8311_PWM_SYNC_PRD_PWM_SYNC_PRD_BITS);
    drv8311SPIVars->PWM_SYNC_PRD_Reg_0C.PWM_SYNC_PRD_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_SYNC_PRD_RESERVED1_BITS)?1:0;
    drv8311SPIVars->PWM_SYNC_PRD_Reg_0C.PWM_SYNC_PRD_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_SYNC_PRD_RESERVED2_BITS)?1:0;
    drv8311SPIVars->PWM_SYNC_PRD_Reg_0C.PWM_SYNC_PRD_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_PWM_SYNC_PRD_RESERVED3_BITS)?1:0;

    // Read FLT_MODE
    drvRegAddr = DRV8311_ADDRESS_FLT_MODE;
    drvDataNew = readSPI(drvRegAddr);
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
    drvDataNew = readSPI(drvRegAddr);
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
    drvDataNew = readSPI(drvRegAddr);
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
    drvDataNew = readSPI(drvRegAddr);
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
    drvDataNew = readSPI(drvRegAddr);
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
    drvDataNew = readSPI(drvRegAddr);
    drv8311SPIVars->PWMG_PERIOD_Reg_18.PWM_PRD_OUT       = (uint16_t)(drvDataNew & (uint16_t)DRV8311_PWMG_PERIOD_PWM_PRD_OUT_BITS);
    drv8311SPIVars->PWMG_PERIOD_Reg_18.PWMG_PERIOD_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_PERIOD_RESERVED1_BITS)?1:0;
    drv8311SPIVars->PWMG_PERIOD_Reg_18.PWMG_PERIOD_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_PERIOD_RESERVED2_BITS)?1:0;
    drv8311SPIVars->PWMG_PERIOD_Reg_18.PWMG_PERIOD_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_PERIOD_RESERVED3_BITS)?1:0;

    // Read PWMG_A_DUTY
    drvRegAddr = DRV8311_ADDRESS_PWMG_A_DUTY;
    drvDataNew = readSPI(drvRegAddr);
    drv8311SPIVars->PWMG_A_DUTY_Reg_19.PWM_DUTY_OUTA     = (uint16_t)(drvDataNew & (uint16_t)DRV8311_PWMG_A_DUTY_PWM_DUTY_OUTA_BITS);
    drv8311SPIVars->PWMG_A_DUTY_Reg_19.PWMG_A_DUTY_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_A_DUTY_RESERVED1_BITS)?1:0;
    drv8311SPIVars->PWMG_A_DUTY_Reg_19.PWMG_A_DUTY_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_A_DUTY_RESERVED2_BITS)?1:0;
    drv8311SPIVars->PWMG_A_DUTY_Reg_19.PWMG_A_DUTY_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_A_DUTY_RESERVED3_BITS)?1:0;

    // Read PWMG_B_DUTY
    drvRegAddr = DRV8311_ADDRESS_PWMG_B_DUTY;
    drvDataNew = readSPI(drvRegAddr);
    drv8311SPIVars->PWMG_B_DUTY_Reg_1A.PWM_DUTY_OUTB     = (uint16_t)(drvDataNew & (uint16_t)DRV8311_PWMG_B_DUTY_PWM_DUTY_OUTB_BITS);
    drv8311SPIVars->PWMG_B_DUTY_Reg_1A.PWMG_B_DUTY_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_B_DUTY_RESERVED1_BITS)?1:0;
    drv8311SPIVars->PWMG_B_DUTY_Reg_1A.PWMG_B_DUTY_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_B_DUTY_RESERVED2_BITS)?1:0;
    drv8311SPIVars->PWMG_B_DUTY_Reg_1A.PWMG_B_DUTY_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_B_DUTY_RESERVED3_BITS)?1:0;

    // Read PWMG_C_DUTY
    drvRegAddr = DRV8311_ADDRESS_PWMG_C_DUTY;
    drvDataNew = readSPI(drvRegAddr);
    drv8311SPIVars->PWMG_C_DUTY_Reg_1B.PWM_DUTY_OUTC       = (uint16_t)(drvDataNew & (uint16_t)DRV8311_PWMG_C_DUTY_PWM_DUTY_OUTC_BITS);
    drv8311SPIVars->PWMG_C_DUTY_Reg_1B.PWMG_C_DUTY_RSV1  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_C_DUTY_RESERVED1_BITS)?1:0;
    drv8311SPIVars->PWMG_C_DUTY_Reg_1B.PWMG_C_DUTY_RSV2  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_C_DUTY_RESERVED2_BITS)?1:0;
    drv8311SPIVars->PWMG_C_DUTY_Reg_1B.PWMG_C_DUTY_RSV3  = (bool)(drvDataNew & (uint16_t)DRV8311_PWMG_C_DUTY_RESERVED3_BITS)?1:0;

    // Read PWM_STATE
    drvRegAddr = DRV8311_ADDRESS_PWM_STATE;
    drvDataNew = readSPI(drvRegAddr);
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
    drvDataNew = readSPI(drvRegAddr);
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
    drvDataNew = readSPI(drvRegAddr);
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
    drvDataNew = readSPI(drvRegAddr);
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
    drvDataNew = readSPI(drvRegAddr);
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
    drvDataNew = readSPI(drvRegAddr);
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
}

uint16_t Drv8311::readSPI( const DRV8311_Address_e regAddr){
    volatile uint16_t ctrlHeader = 0;

    // build the control header
    ctrlHeader = (uint16_t)DRV8311_buildCtrlHeader(DRV8311_CTRLMODE_READ, regAddr);
    tx_buf_ = 0x8000|regAddr<<2;//ctrlHeader<<8;

    if (!spi_arbiter_->transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), (uint8_t *)(&rx_buf_), 2, 1000)) {
        return false;
    }

    volatile uint32_t rx = rx_buf_;
    uint16_t readWord = (rx>>16)&0xffff;//(((rx>>24) & 0xFF) << 8) | ((rx>>16) & 0xFF); //concatenate D14:D0

    return(readWord & DRV8311_DATA_MASK);
}

bool Drv8311::writeSPI(const DRV8311_Address_e regAddr, const uint16_t data){
    uint16_t ctrlHeader = 0, ctrlWord = 0;

    // build the control header
    ctrlHeader = (uint16_t)DRV8311_buildCtrlHeader(DRV8311_CTRLMODE_WRITE, regAddr);
    // build the control word
    ctrlWord   = (uint16_t)DRV8311_buildCtrlWord(data);
    tx_buf_ = ctrlHeader<<16|ctrlWord;


    if (!spi_arbiter_->transfer(spi_config_, ncs_gpio_, (uint8_t *)(&tx_buf_), (uint8_t *)(&rx_buf_), 2, 1000)) {
        return false;
    }

    return true;
}

