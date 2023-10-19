/*------------------------------------------------------------------------------
--                   HERCCULES On-Board Software Components                   --
--                                                                            --
--                               BUS HANDLERS                                 --
--                                                                            --
--                        ProtectedI2CHandler Source                          --
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

#include "ProtectedI2CHandler.h"

namespace bh_dt = bus_handlers::data;

namespace bus_handlers {

    ProtectedI2CHandler::ProtectedI2CHandler() = default;

    bool ProtectedI2CHandler::initialize(uint8_t busID) {
        return i2CHandler.initialize(busID);
    }

    void ProtectedI2CHandler::finalize() {
        i2CHandler.finalize();
    }

    int ProtectedI2CHandler::memRead(uint8_t slaveAddress, uint8_t *rxBuffer, uint16_t nBytes) const {
        std::lock_guard<std::mutex> lock (mutex);
        return i2CHandler.memRead(slaveAddress, rxBuffer, nBytes);
    }

    bool ProtectedI2CHandler::readRegister(uint8_t slaveAddress, uint8_t reg, uint8_t *rxBuffer, uint16_t nBytes) const {
        std::lock_guard<std::mutex> lock (mutex);
        return i2CHandler.readRegister(slaveAddress, reg, rxBuffer, nBytes);
    }

    bool ProtectedI2CHandler::readByteRegister(uint8_t slaveAddress, uint8_t reg, uint8_t &rxBuffer) const {
        std::lock_guard<std::mutex> lock (mutex);
        return i2CHandler.readByteRegister(slaveAddress, reg, rxBuffer);
    }

    bool ProtectedI2CHandler::readWordRegister(uint8_t slaveAddress, uint8_t reg, uint16_t &rxBuffer,
                                              bh_dt::ByteOrdering byteOrdering) const {
        std::lock_guard<std::mutex> lock (mutex);
        return i2CHandler.readWordRegister(slaveAddress, reg, rxBuffer, byteOrdering);
    }

    int ProtectedI2CHandler::memWrite(uint8_t slaveAddress, uint8_t *txBuffer, uint16_t nBytes) const {
        std::lock_guard<std::mutex> lock (mutex);
        return i2CHandler.memWrite(slaveAddress, txBuffer, nBytes);
    }

    bool ProtectedI2CHandler::writeRegister(uint8_t slaveAddress, uint8_t reg, const uint8_t *txBuffer, uint16_t nBytes) const {
        std::lock_guard<std::mutex> lock (mutex);
        return i2CHandler.writeRegister(slaveAddress, reg, txBuffer, nBytes);
    }

    bool ProtectedI2CHandler::writeCommand(uint8_t slaveAddress, uint8_t command) const {
        std::lock_guard<std::mutex> lock (mutex);
        return i2CHandler.writeCommand(slaveAddress, command);
    }

    bool ProtectedI2CHandler::writeByteRegister(uint8_t slaveAddress, uint8_t reg, uint8_t txBuffer) const {
        std::lock_guard<std::mutex> lock (mutex);
        return i2CHandler.writeByteRegister(slaveAddress, reg, txBuffer);
    }

    bool ProtectedI2CHandler::writeWordRegister(uint8_t slaveAddress, uint8_t reg, uint16_t txBuffer,
                                               bh_dt::ByteOrdering byteOrdering) const {
        std::lock_guard<std::mutex> lock (mutex);
        return i2CHandler.writeWordRegister(slaveAddress, reg, txBuffer, byteOrdering);
    }

    bool ProtectedI2CHandler::isTheBusOpen() const {
        std::lock_guard<std::mutex> lock (mutex);
        return i2CHandler.isTheBusOpen();
    }

    bool ProtectedI2CHandler::read3ByteRegister
            (uint8_t slaveAddress,
             uint8_t reg,
             uint32_t &rxBuffer,
             bh_dt::ByteOrdering byteOrdering) const
    {
        std::lock_guard<std::mutex> lock (mutex);
        return i2CHandler.read3ByteRegister(slaveAddress, reg, rxBuffer, byteOrdering);
    }
}
