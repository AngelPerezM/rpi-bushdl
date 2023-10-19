/*------------------------------------------------------------------------------
--                   HERCCULES On-Board Software Components                   --
--                                                                            --
--                               BUS HANDLERS                                 --
--                                                                            --
--                             I2CHandler Header                              --
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

#ifndef HAL_I2CHANDLER_H
#define HAL_I2CHANDLER_H

#include "BusHandlers_Data.h" // I2CBusID, ByteOrdering

#include <cstdint>
#include <string>


namespace bus_handlers {

    // Brief
    // -----
    // This class is a facade that provides hardware abstraction for
    // the I2C bus.
    //
    // Notes
    // -----
    // This class access the i2c device driver from userspace through the /dev
    // interface.
    //
    // From https://www.kernel.org/doc/Documentation/i2c/dev-interface.rst:
    //
    // - The following Linux command must be executed in order to load the dev
    // module into the Linux kernel:
    // > sudo modprobe i2c-dev
    //
    // - Run `i2cdetect -l` to obtain a formatted list of all I2C adapters present
    // on your system at a given time. `i2cdetect` is part of the `i2c-tools`
    // package.
    //
    // We are using ioctl calls because:
    //
    // > ... only a subset of the I2C and SMBus protocols can be achieved by
    // > the means of read() and write() calls. In particular, so-called combined
    // > transactions (mixing read and write messages in the same transaction)
    // > aren't supported.
    class I2CHandler {

    public:

        explicit I2CHandler();

        /// \brief initializes the I2C Bus with the busID identifier
        bool initialize(uint8_t busID);

        void finalize();

        int memRead
            (uint8_t slaveAddress,
             uint8_t *rxBuffer,
             uint16_t nBytes) const;

        bool readRegister
            (uint8_t slaveAddress,
             uint8_t reg,
             uint8_t *rxBuffer,
             uint16_t nBytes) const;

        bool readByteRegister
            (uint8_t slaveAddress,
             uint8_t reg,
             uint8_t &rxBuffer) const;

        bool readWordRegister
            (uint8_t slaveAddress,
             uint8_t reg,
             uint16_t &rxBuffer,
             bus_handlers::data::ByteOrdering byteOrdering) const;

        bool read3ByteRegister
            (uint8_t slaveAddress,
             uint8_t reg,
             uint32_t &rxBuffer,
             bus_handlers::data::ByteOrdering byteOrdering) const;

        int memWrite
            (uint8_t slaveAddress,
             uint8_t *txBuffer,
             uint16_t nBytes) const;

        bool writeCommand
            (uint8_t slaveAddress,
             uint8_t command) const;

        bool writeRegister
            (uint8_t slaveAddress,
             uint8_t reg,
             const uint8_t *txBuffer,
             uint16_t nBytes) const;

        bool writeByteRegister
            (uint8_t slaveAddress,
             uint8_t reg,
             uint8_t txBuffer) const;

        bool writeWordRegister
             (uint8_t slaveAddress,
              uint8_t reg,
              uint16_t txBuffer,
              bus_handlers::data::ByteOrdering byteOrdering) const;

        bool isTheBusOpen() const;

    private:
        int deviceFileDescriptor = -1;

    };

}

#endif //HAL_I2CHANDLER_H
