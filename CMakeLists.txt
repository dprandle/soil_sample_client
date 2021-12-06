cmake_minimum_required(VERSION 3.10.0)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/MSP430.cmake)

project(TemplateProj)

set(VERSION "1.0.0")
set(REQUIRED_EXT ".elf")
set(MSP430_DEVICE_NAME "msp430fr4133")

set(TARGET_NAME "${PROJECT_NAME}-v${VERSION}${REQUIRED_EXT}")
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)

# Set the src files for the project
file(GLOB SRC_FILES
  "${SRC_DIR}/*.cpp"
  "${SRC_DIR}/*.c"
  )

# Uncomment the line below to allow manual commands and command feedback to show on BC Uart
# This will affect the TX frame timing so only do it to debug radio
# add_definitions(-DRADIO_DEBUG_SPI)

# Uncomment the line below to send packet RX to backchannel uart
# This takes quite a bit of memory to do so just use it when needed
#add_definitions(-DRADIO_DEBUG_RX_PACKET)

# Uncomment this to add more general debug backchannel uart info
# add_definitions(-DBC_UART_DEBUG)

msp430_add_executable(${TARGET_NAME} ${SRC_FILES})
target_link_libraries(${TARGET_NAME} c gcc)