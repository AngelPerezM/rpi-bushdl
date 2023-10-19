/*------------------------------------------------------------------------------
--                   HERCCULES On-Board Software Components                   --
--                                                                            --
--                               BUS HANDLERS                                 --
--                                                                            --
--                             Data Types Header                              --
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


#ifndef HAL_BUS_HANDLERS_DATA_H
#define HAL_BUS_HANDLERS_DATA_H

namespace bus_handlers {
    /// @brief Data types for the Bus Handlers subsystem.
    namespace data {

        /// @brief Enumeration type for the allowed I2C bus identifiers. This
        /// way, we offer only the available buses.
        /// @note: BUS2 is not available in HERCCULES
        enum I2CBusID : int {
            BUS0 = 0,
            BUS1 = 1,
            BUS3 = 2,
            BUS4 = 3
        };

        enum class ByteOrdering {
            Big,
            Little
        };

    }
}

#endif //HAL_BUS_HANDLERS_DATA_H
