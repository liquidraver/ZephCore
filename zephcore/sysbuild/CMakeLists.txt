# Sysbuild resolves the board BEFORE the application CMakeLists.txt runs,
# so the BOARD_ROOT set there never fires. Set it here, before find_package(Sysbuild).
set(BOARD_ROOT "${APP_DIR}" CACHE STRING "" FORCE)

# Auto-propagate flash size from board.conf to MCUboot.
#
# MCUboot is built as a separate image and does NOT inherit the app's board.conf
# or esp32_common.conf. Boards with non-default flash sizes set
# CONFIG_ESPTOOLPY_FLASHSIZE_<N>MB=y in their board.conf, but without this
# block MCUboot builds with whatever its defconfig/Kconfig default is.
# A mismatch between MCUboot's image header flash size and the actual hardware
# causes the ROM bootloader to misconfigure the SPI flash controller, making
# MCUboot hang at "spi_flash: detected chip:".
#
# We set mcuboot_EXTRA_CONF_FILE BEFORE find_package(Sysbuild) so it takes
# priority — sysbuild_extensions.cmake uses CACHE INTERNAL (no FORCE) which
# only writes if the variable is not already set.
#
# Static conf files (mcuboot_flashsize_*.conf) are used instead of generating
# a temp file to avoid Windows line-ending issues with cmake file(WRITE).
string(REPLACE "/" ";" _board_parts "${BOARD}")
list(GET _board_parts 0 _board_base)

file(GLOB_RECURSE _board_conf_candidates
    "${APP_DIR}/boards/*/${_board_base}/board.conf")
if(NOT _board_conf_candidates AND
   EXISTS "${APP_DIR}/boards/${_board_base}/board.conf")
    set(_board_conf_candidates "${APP_DIR}/boards/${_board_base}/board.conf")
endif()

if(_board_conf_candidates)
    list(GET _board_conf_candidates 0 _board_conf)
    file(READ "${_board_conf}" _board_conf_content)

    set(_flashsize_fragment "")
    foreach(_size 16MB 8MB 32MB 64MB 128MB)
        if(_board_conf_content MATCHES "CONFIG_ESPTOOLPY_FLASHSIZE_${_size}=y")
            string(TOLOWER "${_size}" _size_lower)
            set(_flashsize_fragment
                "${CMAKE_CURRENT_SOURCE_DIR}/mcuboot_flashsize_${_size_lower}.conf")
            break()
        endif()
    endforeach()

    if(_flashsize_fragment AND EXISTS "${_flashsize_fragment}")
        set(mcuboot_EXTRA_CONF_FILE
            "${CMAKE_CURRENT_SOURCE_DIR}/mcuboot.conf;${_flashsize_fragment}"
            CACHE INTERNAL "MCUboot conf: global + flash size auto-propagated from board.conf")
    endif()
endif()

# Share the app's flash partition layout with MCUboot.
#
# The app's board.overlay is applied to the app image only. Without this, MCUboot
# falls back to the upstream Espressif base partition table, whose slot0/slot1
# differ from the app's customised layout. The two images then agree only on
# slot0's *start* (so the device boots), but their image-1 (OTA) slots land at
# different addresses: the app stages an update where MCUboot never looks, so OTA
# uploads report success and then silently boot the old image on reset.
#
# Feeding the same partitions.overlay to MCUboot keeps both flash maps identical.
file(GLOB_RECURSE _board_partitions
    "${APP_DIR}/boards/*/${_board_base}/partitions.overlay")
if(NOT _board_partitions AND
   EXISTS "${APP_DIR}/boards/${_board_base}/partitions.overlay")
    set(_board_partitions "${APP_DIR}/boards/${_board_base}/partitions.overlay")
endif()
if(_board_partitions)
    list(GET _board_partitions 0 _board_partitions_file)
    set(mcuboot_EXTRA_DTC_OVERLAY_FILE "${_board_partitions_file}"
        CACHE INTERNAL "MCUboot partitions overlay: shared with the app image")
endif()

find_package(Sysbuild REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sysbuild LANGUAGES)
