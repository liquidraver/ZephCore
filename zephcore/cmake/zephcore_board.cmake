# Resolve the ZephCore board directory, the platform, and the build class.
#
# Shared by zephcore/CMakeLists.txt and zephcore/sysbuild/CMakeLists.txt so the
# app and the MCUboot image always agree on which board directory they read.
# Runs before find_package(Zephyr): nothing here may depend on Kconfig or DT.
#
# Inputs:  BOARD                name[@revision][/soc[/cpucluster]]
#          ZEPHCORE_SOURCE_DIR  the application source dir
# Outputs: ZEPHCORE_BOARD_BASE  board name without revision or qualifiers
#          ZEPHCORE_BOARD_DIR   boards/<platform-dir>/<board>, or "" if none
#          ZEPHCORE_PLATFORM    nrf52 | nrf54l | esp32 | mg24 | stm32wl | linux | ""
#          ZEPHCORE_BOARD_LIGHT_SLEEP  TRUE if the board manifest declares it
#          zephcore_board_file(<out-var> <file-name>)
#              absolute path of <file-name> inside the board dir, or "".

string(REGEX REPLACE "[@/].*$" "" ZEPHCORE_BOARD_BASE "${BOARD}")

# ---- Board directory -------------------------------------------------------
# boards/<platform-dir>/<board>/. Exactly one may exist: two would make every
# per-board file (conf, overlay, partitions, battery curve) ambiguous, and the
# old "take the first glob match" behaviour picked one silently.
set(ZEPHCORE_BOARD_DIR "")
if(ZEPHCORE_BOARD_BASE)
    file(GLOB _zb_candidates LIST_DIRECTORIES true
         "${ZEPHCORE_SOURCE_DIR}/boards/*/${ZEPHCORE_BOARD_BASE}")
    set(_zb_dirs "")
    foreach(_c IN LISTS _zb_candidates)
        if(IS_DIRECTORY "${_c}")
            list(APPEND _zb_dirs "${_c}")
        endif()
    endforeach()
    list(LENGTH _zb_dirs _zb_count)
    if(_zb_count GREATER 1)
        message(FATAL_ERROR
            "ZephCore: board '${ZEPHCORE_BOARD_BASE}' has ${_zb_count} directories:\n"
            "  ${_zb_dirs}\nKeep exactly one under boards/<platform>/.")
    elseif(_zb_count EQUAL 1)
        set(ZEPHCORE_BOARD_DIR "${_zb_dirs}")
    endif()
endif()

function(zephcore_board_file OUT NAME)
    if(ZEPHCORE_BOARD_DIR AND EXISTS "${ZEPHCORE_BOARD_DIR}/${NAME}")
        set(${OUT} "${ZEPHCORE_BOARD_DIR}/${NAME}" PARENT_SCOPE)
    else()
        set(${OUT} "" PARENT_SCOPE)
    endif()
endfunction()

# ---- Platform --------------------------------------------------------------
# The board's directory is its platform. Boards without a ZephCore directory
# (a plain upstream board built for experiments) fall back to the SoC named in
# the board qualifier; native_sim is the Linux port, whose presets are
# boards/linux_native/<preset>.conf rather than board directories.
set(ZEPHCORE_PLATFORM "")
if(ZEPHCORE_BOARD_DIR)
    get_filename_component(_zb_parent "${ZEPHCORE_BOARD_DIR}" DIRECTORY)
    get_filename_component(_zb_platdir "${_zb_parent}" NAME)
    if(_zb_platdir STREQUAL "nrf52840")
        set(ZEPHCORE_PLATFORM nrf52)
    elseif(_zb_platdir MATCHES "^(nrf54l|esp32|mg24|stm32wl)$")
        set(ZEPHCORE_PLATFORM "${_zb_platdir}")
    else()
        message(FATAL_ERROR "ZephCore: unknown platform directory boards/${_zb_platdir}/")
    endif()
elseif(BOARD MATCHES "nrf52")
    set(ZEPHCORE_PLATFORM nrf52)
elseif(BOARD MATCHES "esp32")
    set(ZEPHCORE_PLATFORM esp32)
elseif(BOARD MATCHES "nrf54l")
    set(ZEPHCORE_PLATFORM nrf54l)
elseif(BOARD MATCHES "mg24|efr32")
    set(ZEPHCORE_PLATFORM mg24)
elseif(BOARD MATCHES "stm32wl|lora_e5")
    set(ZEPHCORE_PLATFORM stm32wl)
elseif(BOARD MATCHES "^native_(sim|posix)")
    set(ZEPHCORE_PLATFORM linux)
endif()

# ---- Capabilities from the board manifest ----------------------------------
# zephcore.yml (schema: scripts/board_manifest.py). Read by line match because
# this runs before Zephyr has set up Python; `board_manifest.py check` enforces
# the exact two-space-indented `  <capability>: true|false` form this relies on.
set(ZEPHCORE_BOARD_LIGHT_SLEEP FALSE)
zephcore_board_file(_zb_manifest zephcore.yml)
if(_zb_manifest)
    file(STRINGS "${_zb_manifest}" _zb_ls REGEX "^  light_sleep: true[ \t]*$")
    if(_zb_ls)
        set(ZEPHCORE_BOARD_LIGHT_SLEEP TRUE)
    endif()
endif()
