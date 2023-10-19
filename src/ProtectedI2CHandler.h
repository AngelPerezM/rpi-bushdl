/*------------------------------------------------------------------------------
--                   HERCCULES On-Board Software Components                   --
--                                                                            --
--                               BUS HANDLERS                                 --
--                                                                            --
--                        ProtectedI2CHandler Header                          --
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

#ifndef HAL_PROTECTED_I2CHANDLER_H
#define HAL_PROTECTED_I2CHANDLER_H

#include "BusHandlers_Data.h" // I2CBusID, ByteOrdering
#include "I2CHandler.h"       // HAS-A relationship with this module.

#include <mutex>

namespace bus_handlers {

    /// @brief This class is a wrapper for the I2CHandler that handles mutual exclusion
    /// in the protected operations. Similar to the protected object concept from Ada.
    class ProtectedI2CHandler {

    public:

        explicit ProtectedI2CHandler();

        /// \brief initializes the I2C Bus with the busID identifier.
        /// This function does not need to be protected
        /// since it only initializes the bus and MUST be invoked during all threads initialization.
        bool initialize(uint8_t busID);

        void finalize();

        /// Protected operation
        int memRead
                (uint8_t slaveAddress,
                 uint8_t *rxBuffer,
                 uint16_t nBytes) const;

        /// Protected operation
        bool readRegister
                (uint8_t slaveAddress,
                 uint8_t reg,
                 uint8_t *rxBuffer,
                 uint16_t nBytes) const;

        /// Protected operation
        bool readByteRegister
                (uint8_t slaveAddress,
                 uint8_t reg,
                 uint8_t &rxBuffer) const;

        /// Protected operation
        bool readWordRegister
                (uint8_t slaveAddress,
                 uint8_t reg,
                 uint16_t &rxBuffer,
                 bus_handlers::data::ByteOrdering byteOrdering) const;

        /// Protected operation
        bool read3ByteRegister
                (uint8_t slaveAddress,
                 uint8_t reg,
                 uint32_t &rxBuffer,
                 bus_handlers::data::ByteOrdering byteOrdering) const;

        /// Protected operation
        int memWrite
                (uint8_t slaveAddress,
                 uint8_t *txBuffer,
                 uint16_t nBytes) const;

        /// Protected operation
        bool writeRegister
                (uint8_t slaveAddress,
                 uint8_t reg,
                 const uint8_t *txBuffer,
                 uint16_t nBytes) const;

        /// Protected PI
        bool writeCommand
                (uint8_t slaveAddress,
                 uint8_t command) const;

        /// Protected operation
        bool writeByteRegister
                (uint8_t slaveAddress,
                 uint8_t reg,
                 uint8_t txBuffer) const;

        /// Protected operation
        bool writeWordRegister
                (uint8_t slaveAddress,
                 uint8_t reg,
                 uint16_t txBuffer,
                 bus_handlers::data::ByteOrdering byteOrdering) const;

        /// Protected operation
        bool isTheBusOpen() const;

    private:
        mutable std::mutex mutex;
        I2CHandler i2CHandler;
    };

}
#endif //HAL_PROTECTED_I2CHANDLER_H
