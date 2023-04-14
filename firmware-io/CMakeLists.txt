cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(zx_pico_if1_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Run from RAM. If the program runs from flash, the first time it goes
# down the IORQ code path it runs slowly and the IO is missed. This
# fetches the code into fast RAM from the start
#
# The issue with this is that the test MDR image, all 100KB of it,
# gets copied into RAM at startup.
#
# set(PICO_COPY_TO_RAM 1)

pico_sdk_init()

if (TARGET tinyusb_device)

  add_executable(zx_pico_if1_io_fw
    zx_pico_if1_io_fw.c
    if1.c
  )

  # I prefer to catch my own OOM conditions
  # 
  target_compile_definitions(zx_pico_if1_io_fw PUBLIC -D PICO_MALLOC_PANIC=0)

  pico_generate_pio_header(zx_pico_if1_io_fw ${CMAKE_CURRENT_LIST_DIR}/mreq_dir.pio)

  target_link_libraries(zx_pico_if1_io_fw pico_stdlib pico_mem_ops pico_multicore hardware_pio hardware_spi hardware_flash hardware_dma)

  pico_enable_stdio_usb(zx_pico_if1_io_fw 0)
  pico_enable_stdio_uart(zx_pico_if1_io_fw 0)

  pico_add_extra_outputs(zx_pico_if1_io_fw)

elseif(PICO_ON_DEVICE)
   message(WARNING "not building because TinyUSB submodule is not initialized in the SDK")
endif()
