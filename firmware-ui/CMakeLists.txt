cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(zx_pico_if1_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

if (TARGET tinyusb_device)

  add_executable(zx_pico_if1_ui_fw
    zx_pico_if1_ui_fw.c
    ssd1306.c
    hw_config.c
  )

  add_subdirectory(no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI)

  target_link_libraries(zx_pico_if1_ui_fw pico_stdlib
                                          pico_mem_ops	
                                          pico_multicore
                                          hardware_spi
                                          hardware_flash
                                          hardware_dma
                                          hardware_i2c
                                          FatFs_SPI)

  pico_enable_stdio_usb(zx_pico_if1_ui_fw 0)
  pico_enable_stdio_uart(zx_pico_if1_ui_fw 0)

  pico_add_extra_outputs(zx_pico_if1_ui_fw)

elseif(PICO_ON_DEVICE)
   message(WARNING "not building because TinyUSB submodule is not initialized in the SDK")
endif()
