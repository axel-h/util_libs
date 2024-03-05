#
# Copyright 2018, Data61, CSIRO (ABN 41 687 119 230)
#
# SPDX-License-Identifier: BSD-2-Clause
#

cmake_minimum_required(VERSION 3.13)

project(libethdrivers C)

set(configure_string "")

config_string(
    LibEthdriverRXDescCount LIB_ETHDRIVER_RX_DESC_COUNT
    "Number of RX descriptors in the descriptor ring for the driver."
    DEFAULT 128
    UNQUOTE
)

config_string(
    LibEthdriverTXDescCount LIB_ETHDRIVER_TX_DESC_COUNT
    "Number of TX descriptors in the descriptor ring for the driver."
    DEFAULT 128
    UNQUOTE
)

config_string(
    LibEthdriverNumPreallocatedBuffers LIB_ETHDRIVER_NUM_PREALLOCATED_BUFFERS
    "Number of preallocated DMA buffers
    To avoid allocating and freeing buffers continuously the driver
    can preallocate a base amount internally."
    DEFAULT 512
    UNQUOTE
)

config_string(
    LibEthdriverPreallocatedBufSize LIB_ETHDRIVER_PREALLOCATED_BUF_SIZE
    "Size of preallocated DMA buffers
    The size of each preallocated buffer that will be used for RX and
    TX allocation requests. This needs to be the maximum of the RX buffer
    size and the MTU. Currently the largest RX buffer of any of the
    implemented drivers is 2048, and the MTU is 1500"
    DEFAULT 2048
    UNQUOTE
)

config_option(
    LibEthdriverPicoTCBAsyncDriver LIB_PICOTCP_ASYNC_DRIVER
    "Async driver for PicoTcp
    Use an async instead of a polling driver for PicoTCP."
    DEFAULT ON
)

mark_as_advanced(
    LibEthdriverRXDescCount
    LibEthdriverTXDescCount
    LibEthdriverNumPreallocatedBuffers
    LibEthdriverPreallocatedBufSize
    LibEthdriverPicoTCBAsyncDriver
)

# Create ethdrivers_Config
add_config_library(ethdrivers "${configure_string}")

add_library(
    ethdrivers
    STATIC
    EXCLUDE_FROM_ALL
    "src/helpers.c"
    "src/virtio_pci.c"
)

if(KernelPlatformImx8mq-evk)
    # Re-use the imx6 sources
    set(PlatPrefix "imx6")
else()
    set(PlatPrefix "${KernelPlatform}")
endif()

file(
    GLOB
        sources
        CONFIGURE_DEPENDS
        src/plat/${PlatPrefix}/*.c
        src/plat/${PlatPrefix}/uboot/*.c
        src/plat/${PlatPrefix}/cpsw/*.c
)
target_sources(ethdrivers PRIVATE ${sources})

target_include_directories(
    ethdrivers
    PUBLIC "include" "plat_include/${PlatPrefix}" "arch_include/${KernelArch}"
    PRIVATE "src/plat/${PlatPrefix}/cpsw" "$<TARGET_PROPERTY:virtio,INTERFACE_INCLUDE_DIRECTORIES>"
)

# Provide the driver modules by forcing the linker to put symbols into the
# library, instead of discarding them as unused. Using target_link_libraries()
# is a bit of a hack, target_link_options() would be the cleaner way, but this
# requires CMake v3.13.
foreach(module IN ITEMS "tx2_ether_qos" "zynq7000_gem" "zynqmp_gem" "imx_fec" "odroidc2_ethernet")
    target_link_options(ethdrivers PUBLIC "LINKER:--undefined=${module}_ptr")
endforeach()

target_link_libraries(ethdrivers muslc platsupport ethdrivers_Config sel4_autoconf)

if(LibLwip)
    target_sources(ethdrivers "src/lwip.c")
    target_link_libraries(ethdrivers lwip lwip_Config)
endif()

if(LibPicotcp)
    target_sources(ethdrivers "src/pico_dev_eth.c")
    target_link_libraries(ethdrivers picotcp picotcp_Config)
endif()
