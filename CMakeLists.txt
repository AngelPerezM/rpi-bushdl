#######################
# Bus Handlers Module #
#######################

project(Bus_Handlers VERSION 1.0 LANGUAGES CXX)

# 1) Declare the module Bus_Handlers.
add_library(${PROJECT_NAME} STATIC
        src/I2CHandler.cpp
        src/ProtectedI2CHandler.cpp
        src/BusHandlers.cpp)

# 2) Declare header (include) dependencies (See note 1 from CMAKE_Notes.md)
target_include_directories(${PROJECT_NAME}
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src)

# 3) Declare module's dependencies.
target_link_libraries(${PROJECT_NAME} PRIVATE pigpio)

# 4) Install configuration:
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CMAKE_BINARY_DIR}/lib/${PROJECT_NAME})

install(DIRECTORY   ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION ${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h*")
