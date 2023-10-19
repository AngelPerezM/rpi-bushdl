/*------------------------------------------------------------------------------
--                   HERCCULES On-Board Software Components                   --
--                                                                            --
--                               BUS HANDLERS                                 --
--                                                                            --
--                             I2CHandler Source                              --
--                                                                            --
--            Copyright (C) 2022 Universidad Politécnica de Madrid            --
--                                                                            --
-- HERCCULES was developed by the Real-Time Systems Group at  the Universidad --
-- Politécnica de Madrid.                                                     --
--                                                                            --
-- HERCCULES is free software: you can redistribute it and/or modify it under --
-- the terms of the GNU General Public License as published by the Free Soft- --
-- ware Foundation, either version 3 of the License,  or (at your option) any --
-- later version.                                                             --
--                                                                            --
-- HERCCULES is distributed  in the hope that  it will be useful  but WITHOUT --
-- ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FIT- --
-- NESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more --
-- details. You should have received a copy of the GNU General Public License --
-- along with HERCCULES. If not, see <https://www.gnu.org/licenses/>.         --
--                                                                            --
------------------------------------------------------------------------------*/

#include "I2CHandler.h"
#include <fcntl.h>          // open
#include <unistd.h>         // close
#include <linux/i2c-dev.h>  // i2c_rdwr_ioctl_data, I2C_RDWR
#include <linux/i2c.h>      // i2c_msg, I2C_M_RD
#include <sys/ioctl.h>      // ioctl
#include <cstring>          // memcpy
#include <array>

namespace bh_dt = bus_handlers::data;

namespace bus_handlers {

    //-------------------------
    //-- Auxiliary functions --
    //-------------------------

    struct i2c_msg packOneI2CMessage(uint16_t slaveAddress, int flags, uint16_t nBytes, uint8_t *buffer) {
        struct i2c_msg msg {
                .addr  = slaveAddress,
                .flags = static_cast<uint16_t>(flags),
                .len   = nBytes,
                .buf   = buffer
        };

        return msg;
    }

    std::string deviceFileName(uint8_t busID) {
        return std::string("/dev/i2c-") + std::to_string(static_cast<int>(busID));
    }

    //------------------------
    //-- PIs implementation --
    //------------------------

    I2CHandler::I2CHandler() = default;

    /**
     * @brief Initialize the bus if it was not openned before
     */
    bool I2CHandler::initialize(uint8_t busID) {
        if (!isTheBusOpen()) {
            std::string fileName = deviceFileName(busID);
            deviceFileDescriptor = ::open(fileName.c_str(), O_RDWR);
        }

        return isTheBusOpen();
    }

    void I2CHandler::finalize() {
        if (isTheBusOpen()) {
            if (::close(deviceFileDescriptor) == 0) {
                deviceFileDescriptor = -1;                
            } else {
                perror("[Bus Handlers: I2C] Could not close");
            }
        }
    }

    bool I2CHandler::readByteRegister(uint8_t slaveAddress, uint8_t reg, uint8_t &rxBuffer) const {
        uint8_t rx = 0U;
        bool success = readRegister(slaveAddress, reg, &rx, static_cast<uint16_t>(sizeof(rx)));
        if (success) {
            rxBuffer = rx;
        }

        return success;
    }

    bool I2CHandler::readWordRegister(uint8_t slaveAddress, uint8_t reg, uint16_t &rxBuffer,
                                     bh_dt::ByteOrdering byteOrdering) const {
        uint8_t rx [2];
        bool success = readRegister(slaveAddress, reg, rx, static_cast<uint16_t>(sizeof(rx)));
        if (success) {
            rxBuffer = static_cast<uint16_t>(rx[0]) |
                       static_cast<uint16_t>(rx[1] << 8U);
            rxBuffer = (byteOrdering == bh_dt::ByteOrdering::Little) ?
                       le16toh(rxBuffer):
                       be16toh(rxBuffer);
        }

        return success;
    }

    bool I2CHandler::read3ByteRegister(uint8_t slaveAddress, uint8_t reg, uint32_t &rxBuffer,
                                      bh_dt::ByteOrdering byteOrdering) const {
        uint8_t rx [3];
        bool success = readRegister(slaveAddress, reg, rx, static_cast<uint16_t>(sizeof(rx)));
        if (success) {
            rxBuffer = 0U |
                       static_cast<uint32_t>(rx[0]) |
                       static_cast<uint32_t>(rx[1] << 8U) |
                       static_cast<uint32_t>(rx[2] << 16U);
            rxBuffer = (byteOrdering == bh_dt::ByteOrdering::Little) ?
                       rxBuffer :
                       rxBuffer << 8;
            rxBuffer = (byteOrdering == bh_dt::ByteOrdering::Little) ?
                       le32toh(rxBuffer):
                       be32toh(rxBuffer);
        }
        return success;
    }

    bool I2CHandler::readRegister(uint8_t slaveAddress, uint8_t reg, uint8_t *rxBuffer, uint16_t nBytes) const {
        if (!isTheBusOpen()) {
            return false;
        }

        std::array<struct i2c_msg, 2U> msgs {
                packOneI2CMessage(slaveAddress, 0, 1U, &reg),                // write and
                packOneI2CMessage(slaveAddress, I2C_M_RD, nBytes, rxBuffer)  // read messages.
        };

        struct i2c_rdwr_ioctl_data msgSet = {
                .msgs = msgs.data(),
                .nmsgs = 2U
        };

        int numberOfCompletedMessages;
        numberOfCompletedMessages = ioctl(deviceFileDescriptor,
                                          static_cast<unsigned long>(I2C_RDWR),
                                          &msgSet);
        return (numberOfCompletedMessages == 2);
    }

    int I2CHandler::memRead(uint8_t slaveAddress, uint8_t *rxBuffer, uint16_t nBytes) const {
        if (!isTheBusOpen()) {
            return -1;
        }

        struct i2c_msg msg = packOneI2CMessage(slaveAddress, I2C_M_RD, nBytes, rxBuffer);
        struct i2c_rdwr_ioctl_data msgSet {
                .msgs  = &msg,
                .nmsgs = 1U
        };


        int numberOfCompletedMsgs;
        numberOfCompletedMsgs = ioctl(deviceFileDescriptor,
                                      static_cast<unsigned long>(I2C_RDWR),
                                      &msgSet);
        return numberOfCompletedMsgs;
    }

    bool I2CHandler::writeByteRegister(uint8_t slaveAddress, uint8_t reg, uint8_t txBuffer) const {
        return writeRegister(slaveAddress, reg, &txBuffer, static_cast<uint16_t>(sizeof(txBuffer)));
    }

    bool I2CHandler::writeWordRegister(uint8_t slaveAddress, uint8_t reg, uint16_t txBuffer,
                                      bh_dt::ByteOrdering byteOrdering) const {
        uint8_t tx[2];
        txBuffer = (byteOrdering == bh_dt::ByteOrdering::Little) ?
                   htole16(txBuffer):
                   htobe16(txBuffer);
        tx[0]  = static_cast<uint8_t>(txBuffer & 0x00FFU);
        tx[1] = static_cast<uint8_t>((txBuffer & 0xFF00U) >> 8U);
        return writeRegister(slaveAddress, reg, tx, static_cast<uint16_t>(sizeof(tx)));
    }

    bool I2CHandler::writeRegister(uint8_t slaveAddress, uint8_t reg, const uint8_t *txBuffer, uint16_t nBytes) const {
        if (!isTheBusOpen()) {
            return false;
        }

        //      +---LSB---+----MSB---+
        // buf: | address | txBuffer |
        //      +---dir---+---dir+1--+
        uint16_t bufSize = 1U + nBytes;
        uint8_t buf [bufSize];
        buf[0] = reg;

        if (nBytes > 0U) { // txBuffer is not empty
            (void) memcpy(buf + 1,
                          txBuffer,
                          static_cast<size_t>(nBytes));
        }

        int nOfCompletedMessages;
        nOfCompletedMessages = memWrite(slaveAddress, buf, bufSize);
        return (nOfCompletedMessages == 1);
    }

    bool I2CHandler::writeCommand(uint8_t slaveAddress, uint8_t command) const {
        // +---LSB---+----MSB---+
        // | address | command  |
        // +---dir---+---dir+1--+
        int nOfCompletedMessages = memWrite(slaveAddress, &command, 1U);
        return (nOfCompletedMessages == 1);
    }

    int I2CHandler::memWrite(uint8_t slaveAddress, uint8_t *txBuffer, uint16_t nBytes) const {
        if (!isTheBusOpen()) {
            return -1;
        }

        // `flags = 0` means write data from `txBuffer` to I2C slave.
        struct i2c_msg msg = packOneI2CMessage(slaveAddress, 0, nBytes, txBuffer);
        struct i2c_rdwr_ioctl_data msgSet {
                .msgs  = &msg,
                .nmsgs = 1U
        };

        int numberOfCompletedMsgs;
        numberOfCompletedMsgs = ioctl(deviceFileDescriptor,
                                      static_cast<unsigned long>(I2C_RDWR),
                                      &msgSet);
        return numberOfCompletedMsgs;
    }

    bool I2CHandler::isTheBusOpen() const{
        return deviceFileDescriptor > 0;
    }
}
