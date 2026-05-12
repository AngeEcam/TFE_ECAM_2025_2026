cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf_monitoring)

target_sources(app PRIVATE
    src/main.c
    src/modbus/modbus.c
    src/sensors/sthp01a.c
    src/sensors/xymd04.c
    src/sensors/pt100_pta8c04.c
    src/storage/fifo_storage.c
)

target_include_directories(app PRIVATE src)