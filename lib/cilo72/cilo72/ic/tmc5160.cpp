/*
  Copyright (c) 2023 Daniel Zwirner
  SPDX-License-Identifier: MIT-0
*/

#include "cilo72/ic/tmc5160.h"
#include <math.h>
#include <float.h>
#include <tmc5xxx.h>

namespace cilo72
{
    namespace ic
    {
        namespace
        {
            static const int READ = 0x00;
            static const int WRITE = 0x80;

            static const int REGISTER_GCONF = 0x00;
            static const int REGISTER_GSTAT = 0x01;
            static const int REGISTER_IOIN = 0x04;
            static const int REGISTER_GLOBAL_SCALER = 0x0B;
            static const int REGISTER_IHOLD_IRUN = 0x10;
            static const int REGISTER_TSTEP = 0x12;
            static const int REGISTER_TPWMTHRS = 0x13;
            static const int REGISTER_TCOOLTHRS = 0x14;
            static const int REGISTER_RAMPMODE = 0x20;
            static const int REGISTER_XACTUAL = 0x21;
            static const int REGISTER_VACTUAL = 0x22;
            static const int REGISTER_VSTART = 0x23;
            static const int REGISTER_A1 = 0x24;
            static const int REGISTER_V1 = 0x25;
            static const int REGISTER_AMAX = 0x26;
            static const int REGISTER_VMAX = 0x27;
            static const int REGISTER_DMAX = 0x28;
            static const int REGISTER_D1 = 0x2A;
            static const int REGISTER_VSTOP = 0x2B;
            static const int REGISTER_TZEROWAIT = 0x2C;
            static const int REGISTER_XTARGET = 0x2D;
            static const int REGISTER_VDCMIN = 0x33;
            static const int REGISTER_SW_MODE = 0x34;
            static const int REGISTER_RAMP_STAT = 0x35;
            static const int REGISTER_XLATCH = 0x36;
            static const int REGISTER_ENCMODE = 0x38;
            static const int REGISTER_X_ENC = 0x39;
            static const int REGISTER_ENC_CONST = 0x3A;
            static const int REGISTER_ENC_STATUS = 0x3B;
            static const int REGISTER_ENC_LATCH = 0x3C;
            static const int REGISTER_ENC_DEVIATION = 0x3D;
            static const int REGISTER_CHOPCONF = 0x6C;
            static const int REGISTER_COOLCONF = 0x6D;
            static const int REGISTER_DRV_STATUS = 0x6F;
            static const int REGISTER_PWMCONF = 0x70;

        }
        Tmc5160::Tmc5160(const struct device * tmc5xxx, uint32_t rsens, uint32_t fclk)
            : tmc5xxx_(tmc5xxx), rsens_(rsens), fclk_(fclk)
        {
            int retry = 100;

            Tmc5160::IoIn i = {0};

            while (i.version != 0x30 && retry)
            {
                ioin(i);
                retry--;
            }
        }
        Tmc5160::Status Tmc5160::writeRegister(uint8_t addr, uint32_t tx)
        {

            Status ret;
            tmc5xxx_reg_write(tmc5xxx_, addr, tx, &ret.reg);

            return ret;
        }

        Tmc5160::Status Tmc5160::readRegister(uint8_t addr, uint32_t *rx)
        {
            Status ret;

            tmc5xxx_reg_read(tmc5xxx_, addr, rx, &ret.reg);
/*
            if (lastReadAddr_ != addr)
            {
                xfer(addr, READ, 0, 0); // dummy read
            }
            ret = xfer(addr, READ, 0, rx);

            lastReadAddr_ = addr;
*/
            return ret;
        }

        Tmc5160::Status Tmc5160::status()
        {
            Status ret;
            uint32_t dummy;
            ret = readRegister(REGISTER_RAMPMODE, &dummy);

            return ret;
        }

        Tmc5160::Status Tmc5160::ioin(IoIn &value)
        {
            return readRegister(REGISTER_IOIN, &value.reg);
        }

        Tmc5160::Status Tmc5160::setGconf(const Gconf &value)
        {
            return writeRegister(REGISTER_GCONF, value.reg);
        }

        Tmc5160::Status Tmc5160::gconf(Gconf &value)
        {
            return readRegister(REGISTER_GCONF, &value.reg);
        }

        Tmc5160::Status Tmc5160::gstat(Gstat &value)
        {
            return readRegister(REGISTER_GSTAT, &value.reg);
        }

        Tmc5160::Status Tmc5160::setGstat(Gstat value)
        {
            return writeRegister(REGISTER_GSTAT, value.reg);
        }

        Tmc5160::Status Tmc5160::setRampmode(const RampMode value)
        {
            return writeRegister(REGISTER_RAMPMODE, static_cast<uint32_t>(value));
        }

        Tmc5160::Status Tmc5160::rampmode(RampMode &value)
        {
            Status ret;
            uint32_t rx;

            ret = readRegister(REGISTER_RAMPMODE, &rx);

            value = static_cast<RampMode>(rx);

            return ret;
        }

        Tmc5160::Status Tmc5160::setXactual(int32_t value)
        {
            return writeRegister(REGISTER_XACTUAL, value);
        }

        Tmc5160::Status Tmc5160::xactual(int32_t &value)
        {
            return readRegister(REGISTER_XACTUAL, (uint32_t *)&value);
        }

        Tmc5160::Status Tmc5160::vactual(uint32_t &value)
        {
            return readRegister(REGISTER_VACTUAL, &value);
        }

        Tmc5160::Status Tmc5160::setVstart(uint32_t value)
        {
            return writeRegister(REGISTER_VSTART, value);
        }

        Tmc5160::Status Tmc5160::setA1(uint32_t value)
        {
            return writeRegister(REGISTER_A1, value);
        }

        Tmc5160::Status Tmc5160::setV1(uint32_t value)
        {
            return writeRegister(REGISTER_V1, value);
        }

        Tmc5160::Status Tmc5160::setAMax(uint32_t value)
        {
            return writeRegister(REGISTER_AMAX, value);
        }

        Tmc5160::Status Tmc5160::setVMax(uint32_t value)
        {
            return writeRegister(REGISTER_VMAX, value);
        }

        Tmc5160::Status Tmc5160::setDMax(uint32_t value)
        {
            return writeRegister(REGISTER_DMAX, value);
        }

        Tmc5160::Status Tmc5160::setD1(uint32_t value)
        {
            return writeRegister(REGISTER_D1, value);
        }

        Tmc5160::Status Tmc5160::setVStop(uint32_t value)
        {
            return writeRegister(REGISTER_VSTOP, value);
        }

        Tmc5160::Status Tmc5160::setTZeroWait(uint32_t value)
        {
            return writeRegister(REGISTER_TZEROWAIT, value);
        }

        Tmc5160::Status Tmc5160::setXTarget(int32_t value)
        {
            return writeRegister(REGISTER_XTARGET, value);
        }

        Tmc5160::Status Tmc5160::xTarget(int32_t &value)
        {
            return writeRegister(REGISTER_XTARGET, value);
        }

        Tmc5160::Status Tmc5160::setVDcMin(uint32_t value)
        {
            return writeRegister(REGISTER_VDCMIN, value);
        }

        Tmc5160::Status Tmc5160::setSwMode(Sw_Mode value)
        {
            return writeRegister(REGISTER_SW_MODE, value.reg);
        }

        Tmc5160::Status Tmc5160::swMode(Sw_Mode &value)
        {
            return readRegister(REGISTER_SW_MODE, &value.reg);
        }

        Tmc5160::Status Tmc5160::rampStat(RampStat &value)
        {
            return readRegister(REGISTER_RAMP_STAT, &value.reg);
        }

        Tmc5160::Status Tmc5160::xLatch(int32_t &value)
        {
            return readRegister(REGISTER_XLATCH, (uint32_t *)&value);
        }

        Tmc5160::Status Tmc5160::setChopConf(ChopConf value)
        {
            return writeRegister(REGISTER_CHOPCONF, value.reg);
        }

        Tmc5160::Status Tmc5160::chopConf(ChopConf &value)
        {
            return readRegister(REGISTER_CHOPCONF, &value.reg);
        }

        Tmc5160::Status Tmc5160::setIholdIRun(IHold_IRun value)
        {
            return writeRegister(REGISTER_IHOLD_IRUN, value.reg);
        }

        Tmc5160::Status Tmc5160::setTpwmThrs(uint32_t value)
        {
            return writeRegister(REGISTER_TPWMTHRS, value);
        }

        Tmc5160::Status Tmc5160::setTcoolThrs(uint32_t value)
        {
            return writeRegister(REGISTER_TCOOLTHRS, value);
        }

        Tmc5160::Status Tmc5160::tstep(uint32_t &value)
        {
            return readRegister(REGISTER_TSTEP, &value);
        }

        Tmc5160::Status Tmc5160::setPwmConf(PwmConf value)
        {
            return readRegister(REGISTER_PWMCONF, &value.reg);
        }

        Tmc5160::Status Tmc5160::setCoolConf(CoolConf value)
        {
            return writeRegister(REGISTER_COOLCONF, value.reg);
        }

        Tmc5160::Status Tmc5160::drv_Status(Drv_Status &value)
        {
            return readRegister(REGISTER_DRV_STATUS, &value.reg);
        }

        uint8_t Tmc5160::irms2CurrentScale(uint16_t current, bool &valid)
        {
            // See TMC5160 Data sheet, chapter 9 Selecting Sense Resistors
            double vfs = 325.0; // mV See chapter 28.2 DC and Timing Characteristics, Sense resistor voltage levels
            double cs = 0.0;
            double globalscaler = 256.0; // The default value is 0, 0 is the same as 256.
                                         // See chapter 6.1 General Configuration Registers, Register GLOBALSCALER
                                         // And the register is write only ....
            uint8_t csInt = 0;

            cs = ((current * 1.414 * (rsens_) * 32 * 256) / (vfs * 1000 * globalscaler)) - 1;

            if (cs < 0)
            {
                cs = 0;
            }

            csInt = static_cast<uint8_t>(cs + 0.5);

            if (csInt <= 31)
            {
                valid = true;
            }
            else
            {
                valid = false;
            }

            return csInt;
        }

        double Tmc5160::drps2Atmc(double drps, uint16_t uSteps, uint16_t fSteps)
        {
            double atmc;

            atmc = drps * 512.0 * 256.0 * pow(2.0, 24.0) * uSteps * fSteps / pow((double)fclk_, 2);

            return atmc;
        }

        double Tmc5160::rps2Vtmc(double rps, uint16_t uSteps, uint16_t fSteps)
        {
            double vtmc;

            vtmc = rps * fSteps * uSteps * 2.0 * pow(2.0, 23.0) / fclk_;

            return vtmc;
        }

        Tmc5160::Status Tmc5160::setEncMode(EncMode value)
        {
            return writeRegister(REGISTER_ENCMODE, value.reg);
        }

        Tmc5160::Status Tmc5160::encMode(EncMode &value)
        {
            return readRegister(REGISTER_ENCMODE, &value.reg);
        }

        Tmc5160::Status Tmc5160::setXEnc(int32_t value)
        {
            return writeRegister(REGISTER_X_ENC, value);
        }

        Tmc5160::Status Tmc5160::xEnc(int32_t &value)
        {
            return readRegister(REGISTER_X_ENC, reinterpret_cast<uint32_t *>(&value));
        }

        Tmc5160::Status Tmc5160::setEncConst(int32_t &value)
        {
            return writeRegister(REGISTER_ENC_CONST, value);
        }

        Tmc5160::Status Tmc5160::encStatus(Enc_Status &value)
        {
            Tmc5160::Status status = readRegister(REGISTER_ENC_STATUS, &value.reg); // R+WC for TMC5160
            status = writeRegister(REGISTER_ENC_STATUS, value.reg);                 // clear flags set during above read
            return status;
        }

        Tmc5160::Status Tmc5160::encLatch(int32_t &value)
        {
            return readRegister(REGISTER_ENC_LATCH, reinterpret_cast<uint32_t *>(&value));
        }

        Tmc5160::Status Tmc5160::setEncDeviation(uint32_t &value)
        {
            return writeRegister(REGISTER_ENC_DEVIATION, value);
        }

        void Tmc5160::setEncoderFactor(double factor)
        {
            EncMode mode;
            encMode(mode);

            double integral;
            double fraction = modf(factor, &integral);
            double dummy;

            union Factor
            {
                struct
                {
                    uint16_t fraction;
                    int16_t factor;
                };
                int32_t reg;
            };

            Factor f;

            static constexpr double FRACTION_BINARY = 65536.0;
            static constexpr double FRACTION_DECIMAL = 10000.0;

            f.factor = integral;

            double errorBinary = modf(FRACTION_BINARY * fraction, &dummy);
            double errorDecimal = modf(FRACTION_DECIMAL * fraction, &dummy);

            if (errorBinary < errorDecimal)
            {
                f.fraction = (uint16_t)(fraction * FRACTION_BINARY);
                mode.enc_sel_decimal = 0;
            }
            else
            {
                f.fraction = (uint16_t)(fraction * FRACTION_DECIMAL);
                mode.enc_sel_decimal = 1;
            }
            setEncConst(f.reg);
            setEncMode(mode);
        }

        Tmc5160::Status Tmc5160::setGlobalScaler(GlobalScaler value)
        {
            return writeRegister(REGISTER_GLOBAL_SCALER, value.reg);
        }

        Tmc5160::Status Tmc5160::setPwmCoil(PwmCoil value)
        {
            return writeRegister(REGISTER_XTARGET, value.reg);
        }

        void Tmc5160::enable(bool enable)
        {
            tmc5xxx_enable(tmc5xxx_, enable);
        }

    }
}
