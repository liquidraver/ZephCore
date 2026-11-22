# Sysbuild resolves the board BEFORE the application CMakeLists.txt runs,
# so the BOARD_ROOT set there never fires. Set it here, before find_package(Sysbuild).
set(BOARD_ROOT "${APP_DIR}" CACHE STRING "" FORCE)

# Same story for the Zephyr patches and the files ZephCore adds to the Zephyr
# tree: the app applies/copies them in its own CMakeLists, but MCUboot is
# configured first and runs a devicetree pass over the same board DTS. A board
# whose radio node uses a ZephCore-only binding (the LR11xx / LR20xx LoRa
# drivers) or a property only a patch adds (lna-bypass-gpios, patch 0003) needs
# it present before MCUboot's DTS pass, not after. See cmake/zephyr_patches.cmake
# and cmake/zephyr_new_files.cmake.
set(ZEPHCORE_SOURCE_DIR "${APP_DIR}")
set(ZEPHCORE_ZEPHYR_DIR "$ENV{ZEPHYR_BASE}")
get_filename_component(ZEPHCORE_MODULES_DIR "${APP_DIR}/../modules" ABSOLUTE)
include("${APP_DIR}/cmake/zephyr_patches.cmake")
include("${APP_DIR}/cmake/zephyr_new_files.cmake")

include("${APP_DIR}/cmake/zephcore_board.cmake")

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
zephcore_board_file(_board_partitions_file partitions.overlay)
if(_board_partitions_file)
    set(mcuboot_EXTRA_DTC_OVERLAY_FILE "${_board_partitions_file}"
        CACHE INTERNAL "MCUboot partitions overlay: shared with the app image")
endif()

find_package(Sysbuild REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sysbuild LANGUAGES)
