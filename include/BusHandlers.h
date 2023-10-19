/*------------------------------------------------------------------------------
--                   HERCCULES On-Board Software Components                   --
--                                                                            --
--                               BUS HANDLERS                                 --
--                                                                            --
--                                  Header                                    --
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

#ifndef HAL_HL_INTERFACE_H
#define HAL_HL_INTERFACE_H

#include "BusHandlers_Data.h" // I2CBusID, ByteOrdering
#include <cstdint>

/// @brief High level interface for the Bus Handlers layer.
/// @details Clients that need this class do not have to know the
/// internals of this component and should not access to all of them.
/// Bus_Handlers class represents one subsystem, thereby, there must exist
/// only one instance of Bus_Handlers in the whole system.
namespace bus_handlers {

    /// \brief Initializes the I2C buses used in the HERCCULES mission.
    /// This function is tolerant to multiple invocations,
    /// so if you invoke it twice it will be initialized only the first time.
    /// \return A boolean that represents the initialization result.
    bool initialize();

    /// \brief Closes all the I2C buses.
    /// This function is tolerant to multiple invocations,
    /// so if you invoke it twice the bus will be closed only once.
    void finalize();

    /// \brief Reads nBytes from the I2C slave. The reading is stored in rxBuffer.
    /// \param[in]  busID I2C bus identifier from which the data is read.
    /// \param[in]  slaveAddress Address from the I2C slave.
    /// \param[out] rxBuffer Reception buffer initialized by the user, where the reading is returned.
    /// \param[in]  nBytes Number of bytes to be read from the slave.
    int memRead
        (bus_handlers::data::I2CBusID busID,
         uint8_t slaveAddress,
         uint8_t *rxBuffer,
         uint16_t nBytes);

    /// \brief Creates an I2C read transaction to read the reg register.
    /// The number of bytes read is determined by the nBytes parameter. The
    /// data is read without intermediate STOP conditions between bytes.
    /// \param[in]  busID I2C bus identifier from which the data is read.
    /// \param[in]  slaveAddress Address from the I2C slave.
    /// \param[in]  reg Register to be read from the slave.
    /// \param[out] rxBuffer Reception buffer initialized by the user, where the reading is returned.
    /// \param[in]  nBytes Number of bytes to be read from the slave.
    /// \returns True on success.
    bool readRegister
        (bus_handlers::data::I2CBusID busID,
         uint8_t slaveAddress,
         uint8_t reg,
         uint8_t *rxBuffer,
         uint16_t nBytes);

    /// \brief Same as readRegister but the reception buffer length is set to 1 byte.
    /// \returns True on success.
    bool readByteRegister
        (bus_handlers::data::I2CBusID busID,
         uint8_t slaveAddress,
         uint8_t reg,
         uint8_t &rxBuffer);

    /// \brief Same as readRegister but the reception buffer length is set to 2 bytes.
    /// \param[in] byteOrdering Determines the endian-ess of the I2C slave (if
    /// the I2C slave gives the LSB first, then it is Little Endian slave).
    /// \returns True on success.
    bool readWordRegister
        (bus_handlers::data::I2CBusID busID,
         uint8_t slaveAddress,
         uint8_t reg,
         uint16_t &rxBuffer,
         bus_handlers::data::ByteOrdering byteOrdering = bus_handlers::data::ByteOrdering::Big);

    /// \brief Same as readRegister but only the 3 bytes are read. This 3 bytes are then
    /// stored in a 4 byte reception buffer (rxBuffer). There are no 3-byte data types in C/C++
    /// \param[in] byteOrdering Determines the endian-ess of the I2C slave (if
    /// the I2C slave gives the LSB first, then it is Little Endian slave).
    /// \returns True on success.
    bool read3ByteRegister
        (bus_handlers::data::I2CBusID busID,
         uint8_t slaveAddress,
         uint8_t reg,
         uint32_t &rxBuffer,
         bus_handlers::data::ByteOrdering byteOrdering);

    /// \brief Writes txBuffer to the I2C slave. The length of the transmission buffer
    /// is determined by nBytes.
    /// \param[in]  busID I2C bus identifier from which the data is read.
    /// \param[in]  slaveAddress Address from the I2C slave.
    /// \param[out] txBuffer Transmission buffer, it is the data to be written in the I2C Slave.
    /// \param[in]  nBytes Number of bytes to be written to the slave.
    int memWrite
        (bus_handlers::data::I2CBusID busID,
         uint8_t slaveAddress,
         uint8_t *txBuffer,
         uint16_t nBytes);

    /// \brief Creates an I2C write transaction to write to the reg register.
    /// The number of bytes written is determined by the nBytes parameter, each byte
    /// is written without intermediate STOP conditions between bytes.
    /// \param[in]  busID I2C bus identifier from which the data is read.
    /// \param[in]  slaveAddress Address from the I2C slave.
    /// \param[in]  reg Register to be read from the slave.
    /// \param[out] rxBuffer Reception buffer initialized by the user, where the reading is returned.
    /// \param[in]  nBytes Number of bytes to be read from the slave.
    bool writeRegister
        (bus_handlers::data::I2CBusID busID,
         uint8_t slaveAddress,
         uint8_t reg,
         const uint8_t *txBuffer,
         uint16_t nBytes);

    /// \brief Same as writeRegister but the transmission buffer length is set to 1 byte.
    bool writeByteRegister
        (bus_handlers::data::I2CBusID busID,
         uint8_t slaveAddress,
         uint8_t reg,
         uint8_t txBuffer);

    /// \brief Same as writeRegister but the reception buffer length is set to 2 bytes.
    /// \param[in] byteOrdering Determines the endian-ess of the I2C slave (if
    /// the I2C slave expects the LSB first, then it is Little Endian slave).
    /// \returns True on success.
    bool writeWordRegister
        (bus_handlers::data::I2CBusID busID,
         uint8_t slaveAddress,
         uint8_t reg,
         uint16_t txBuffer,
         bus_handlers::data::ByteOrdering byteOrdering = bus_handlers::data::ByteOrdering::Big);

    /// \brief This does not perform a transaction operation. This function only writes
    /// the command byte in the I2C slave.
    bool writeCommand
        (bus_handlers::data::I2CBusID busID,
         uint8_t slaveAddress,
         uint8_t command);

    /// \brief If the I2C bus is open, the user can perform read and write operations.
    /// \returns True if the bus is available (open), otherwise false.
    bool isTheBusOpen
        (bus_handlers::data::I2CBusID busID);

}

#endif //HAL_HL_INTERFACE_H