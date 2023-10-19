/*------------------------------------------------------------------------------
--                   HERCCULES On-Board Software Components                   --
--                                                                            --
--                               BUS HANDLERS                                 --
--                                                                            --
--                                  Source                                    --
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

#include "BusHandlers.h"
#include "ProtectedI2CHandler.h"

#include <cstdint>
#include <cstdio>  // perror
#include <cstdlib> // atexit
#include <array>

#include <pigpio.h>

namespace bh_dt = bus_handlers::data;

using bh_dt::I2CBusID;

/// Private members
namespace {
    bool initialized = false;

    std::array<bus_handlers::ProtectedI2CHandler, 4U> i2c_buses;
}

namespace bus_handlers {

    bool initialize() {
        if (atexit(finalize)) {
            perror("[Bus Handlers] Could not register Finalize atexit");
        }

        if (!initialized) {
            bool i2c0Success = i2c_buses.at(I2CBusID::BUS0).initialize(0U);
            bool i2c1Success = i2c_buses.at(I2CBusID::BUS1).initialize(1U);
            bool i2c3Success = i2c_buses.at(I2CBusID::BUS3).initialize(3U);
            bool i2c4Success = i2c_buses.at(I2CBusID::BUS4).initialize(4U);

            initialized = i2c0Success && i2c1Success && i2c3Success && i2c4Success;
        }

        gpioSetMode(0, PI_ALT0);
        gpioSetMode(1, PI_ALT0);

        return initialized;
    }

    void finalize() {
        i2c_buses.at(I2CBusID::BUS0).finalize();
        i2c_buses.at(I2CBusID::BUS1).finalize();
        i2c_buses.at(I2CBusID::BUS3).finalize();
        i2c_buses.at(I2CBusID::BUS4).finalize();

        initialized = false;
    }

    int memRead(bh_dt::I2CBusID busID, uint8_t slaveAddress, uint8_t *rxBuffer, uint16_t nBytes) {
        return i2c_buses.at(busID).memRead(slaveAddress, rxBuffer, nBytes);
    }

    bool readRegister(bh_dt::I2CBusID busID, uint8_t slaveAddress, uint8_t reg, uint8_t *rxBuffer, uint16_t nBytes) {
        return i2c_buses.at(busID).readRegister(slaveAddress, reg, rxBuffer, nBytes);
    }

    bool readByteRegister(bh_dt::I2CBusID busID, uint8_t slaveAddress, uint8_t reg, uint8_t &rxBuffer) {
        return i2c_buses.at(busID).readByteRegister(slaveAddress, reg, rxBuffer);
    }

    bool readWordRegister(bh_dt::I2CBusID busID, uint8_t slaveAddress, uint8_t reg, uint16_t &rxBuffer, bh_dt::ByteOrdering byteOrdering) {
        return i2c_buses.at(busID).readWordRegister(slaveAddress, reg, rxBuffer, byteOrdering);
    }

    bool read3ByteRegister(bh_dt::I2CBusID busID, uint8_t slaveAddress, uint8_t reg, uint32_t &rxBuffer, bh_dt::ByteOrdering byteOrdering) {
        return i2c_buses.at(busID).read3ByteRegister(slaveAddress, reg, rxBuffer, byteOrdering);
    }

    int memWrite(bh_dt::I2CBusID busID, uint8_t slaveAddress, uint8_t *txBuffer, uint16_t nBytes) {
        return i2c_buses.at(busID).memWrite(slaveAddress, txBuffer, nBytes);
    }

    bool writeRegister(bh_dt::I2CBusID busID, uint8_t slaveAddress, uint8_t reg, const uint8_t *txBuffer, uint16_t nBytes) {
        return i2c_buses.at(busID).writeRegister(slaveAddress, reg, txBuffer, nBytes);
    }

    bool writeByteRegister(bh_dt::I2CBusID busID, uint8_t slaveAddress, uint8_t reg, uint8_t txBuffer) {
        return i2c_buses.at(busID).writeByteRegister(slaveAddress, reg, txBuffer);
    }

    bool writeWordRegister(bh_dt::I2CBusID busID, uint8_t slaveAddress, uint8_t reg, uint16_t txBuffer, bh_dt::ByteOrdering byteOrdering) {
        return i2c_buses.at(busID).writeWordRegister(slaveAddress, reg, txBuffer, byteOrdering);
    }

    bool writeCommand(bh_dt::I2CBusID busID, uint8_t slaveAddress, uint8_t command) {
        return i2c_buses.at(busID).writeCommand(slaveAddress, command);
    }

    bool isTheBusOpen(bh_dt::I2CBusID busID) {
        return i2c_buses.at(busID).isTheBusOpen();
    }

}
