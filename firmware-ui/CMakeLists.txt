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
    sd_card.c
    oled_display.c
    hw_config.c
    work_queue.c
    fsm.c
    gui/gui.c
    gui/gui_fsm.c
  )

  # Run the script which generates the FSM code based on the FSM definition
  #
  execute_process( WORKING_DIRECTORY ../../tools/ COMMAND ./process_fsm.pl ../firmware-ui/gui/gui_fsm.def )

  target_include_directories(zx_pico_if1_ui_fw PRIVATE ../firmware-common)
  target_include_directories(zx_pico_if1_ui_fw PRIVATE ./ gui/)

  add_subdirectory(no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI)

  target_link_libraries(zx_pico_if1_ui_fw pico_stdlib
                                          pico_mem_ops	
                                          pico_multicore
                                          hardware_spi
                                          hardware_i2c
					  hardware_uart
                                          FatFs_SPI)

  pico_enable_stdio_usb(zx_pico_if1_ui_fw 0)
  pico_enable_stdio_uart(zx_pico_if1_ui_fw 0)

  pico_add_extra_outputs(zx_pico_if1_ui_fw)

elseif(PICO_ON_DEVICE)
   message(WARNING "not building because TinyUSB submodule is not initialized in the SDK")
endif()
