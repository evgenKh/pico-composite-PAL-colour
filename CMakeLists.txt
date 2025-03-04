# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

# these need to be early to be honored on the first cmake
#set(PICO_BOARD "pimoroni_tiny2040" CACHE STRING "" FORCE)
set(PICO_BOARD pico CACHE STRING "Board type")

#set(PICO_COPY_TO_RAM 1  CACHE STRING "" FORCE)
set(USE_U8G2 FALSE)
set(USE_UBS_STDIO FALSE)


include(pico_sdk_import.cmake)


project(pico-composite-PAL-colour C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fstack-usage")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-usage")

add_executable(pico-composite-PAL-colour
    SyncGlobals.cpp
    #EepromStorage.cpp
    OptionsRegistry.cpp
    pico-composite-PAL-colour.cpp
    #I2cDisplayDevice.cpp
    #I2cDevice.cpp
    #UiMenu.cpp
    #SettingsAndControls.cpp
)

pico_generate_pio_header(pico-composite-PAL-colour ${CMAKE_CURRENT_LIST_DIR}/dac.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

#target_include_directories(pico-composite-PAL-colour PRIVATE includes includes/images demos)
include_directories(includes includes/images demos)

target_include_directories(pico-composite-PAL-colour PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(pico-composite-PAL-colour
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_dma
    pico_malloc
	pico_mem_ops
    pico_float
    #pico_sync

    pico_stdio
    hardware_i2c
    hardware_flash
)
target_compile_definitions(pico-composite-PAL-colour PRIVATE PLATFORM_PICO=1)

if(USE_U8G2)
    #I wasn't able to make it work with add_subdirectory() or include() or find_package()    
    file(GLOB COMPONENT_SRCS 
        "${CMAKE_CURRENT_LIST_DIR}/libs/u8g2/csrc/*.c" 
        "${CMAKE_CURRENT_LIST_DIR}/libs/u8g2/cppsrc/*.cpp")   
    add_library(u8g2 INTERFACE)
    target_sources(u8g2 INTERFACE ${COMPONENT_SRCS})
    target_include_directories(u8g2 INTERFACE
        "${CMAKE_CURRENT_LIST_DIR}/libs/u8g2/csrc"
        "${CMAKE_CURRENT_LIST_DIR}/libs/u8g2/cppsrc")
    target_compile_definitions(u8g2 INTERFACE U8X8_WITH_USER_PTR)

    target_compile_definitions(pico-composite-PAL-colour PRIVATE USE_U8G2=1)
    target_link_libraries(pico-composite-PAL-colour
        u8g2
    )
endif()
pico_add_extra_outputs(pico-composite-PAL-colour)

if(USE_UBS_STDIO)
pico_enable_stdio_usb(pico-composite-PAL-colour 1)
pico_enable_stdio_uart(pico-composite-PAL-colour 0)
target_compile_definitions(pico-composite-PAL-colour PRIVATE USE_UBS_STDIO=1)
else()
pico_enable_stdio_usb(pico-composite-PAL-colour 0)
pico_enable_stdio_uart(pico-composite-PAL-colour 0)
endif()

# for CPU clocks above 266 MHz, slow down access to the flash
pico_define_boot_stage2(slower_boot2 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
target_compile_definitions(slower_boot2 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
pico_set_boot_stage2(pico-composite-PAL-colour slower_boot2)
