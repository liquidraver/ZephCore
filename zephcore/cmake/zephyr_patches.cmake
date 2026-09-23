# Apply patches/zephyr/*.patch and patches/modules/*/*.patch to the west tree.
#
# Unified diffs applied via `git apply` at configure time.  Idempotent: a stamp
# file in each target tree tracks (patch hashes + target HEAD), and a matching
# stamp is re-verified against the working tree.  Conflicts are fatal.
#
# This lives in its own module because it has to run from TWO places, and the
# order matters -- the same constraint as cmake/zephyr_new_files.cmake:
#
#   - zephcore/CMakeLists.txt, for the application image.
#   - zephcore/sysbuild/CMakeLists.txt, BEFORE find_package(Sysbuild).
#
# Under --sysbuild, sysbuild configures the MCUboot image first, and MCUboot
# runs a full devicetree pass over the same board DTS the app uses.  A board
# whose DTS uses a property that only a patch adds to an upstream binding
# (lna-bypass-gpios, from 0003's semtech,sx126x-base.yaml hunk, in the board
# DTS of heltec_wifi_lora32_v43 / _v4_r8 / heltec_wireless_tracker_v2) then
# fails MCUboot's pass on a fresh tree -- "'lna-bypass-gpios' ... is not
# declared in 'properties:'" -- because the app's CMakeLists, the only place
# that used to apply the patches, has not run yet.  CI never saw it: build.sh
# builds the non-sysbuild boards first, which patches the tree before any
# ESP32 sysbuild runs.  A fresh checkout building one of those boards first
# did.
#
# Running twice per configure costs nothing: the second run finds the stamp
# matching and the patched paths dirty, and skips.
#
# Expects ZEPHCORE_SOURCE_DIR, ZEPHCORE_ZEPHYR_DIR and ZEPHCORE_MODULES_DIR to
# be set by the caller.

function(zephcore_apply_patches PATCH_DIR TARGET_DIR LABEL)
    file(GLOB PATCH_FILES "${PATCH_DIR}/*.patch")
    list(SORT PATCH_FILES)
    if(NOT PATCH_FILES)
        return()
    endif()
    # Stamp = hash(patch contents + target HEAD). Skip if unchanged.
    execute_process(
        COMMAND git rev-parse HEAD
        WORKING_DIRECTORY "${TARGET_DIR}"
        OUTPUT_VARIABLE _target_head
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    set(_hash_input "${_target_head}")
    foreach(_pf ${PATCH_FILES})
        file(MD5 "${_pf}" _h)
        string(APPEND _hash_input "${_h}")
    endforeach()
    string(MD5 _stamp_hash "${_hash_input}")
    set(_stamp "${TARGET_DIR}/.zephcore_${LABEL}.stamp")
    if(EXISTS "${_stamp}")
        file(READ "${_stamp}" _existing_hash)
        string(STRIP "${_existing_hash}" _existing_hash)
        if(_existing_hash STREQUAL _stamp_hash)
            # The stamp hashes HEAD + patch contents and says nothing about the
            # working tree.  A `git reset --hard` / `git checkout -- .` in the
            # target tree reverts our patched files but leaves this stamp behind
            # (it is untracked), and `west update` at an already-current
            # revision does not move HEAD -- so the hash still matches and the
            # build would skip patching and compile a pristine tree.  Depending
            # on which patch was lost that is either a confusing configure-time
            # failure or, worse, a silently unpatched binary.
            #
            # Verify instead of trusting: every patch here modifies tracked
            # files only (none create or delete), so a patch that is applied
            # always leaves its own paths dirty against HEAD.  Paths clean
            # against HEAD therefore means "not applied" exactly.  Note this
            # deliberately does NOT compare tree content to the patch: a dirty
            # path is left alone, so the edit-the-driver-then-regenerate-the-
            # patch workflow still builds your in-progress edits untouched.
            set(_stamp_stale FALSE)
            foreach(_pf ${PATCH_FILES})
                execute_process(
                    COMMAND git apply --numstat "${_pf}"
                    WORKING_DIRECTORY "${TARGET_DIR}"
                    OUTPUT_VARIABLE _vfy_numstat
                    ERROR_QUIET
                )
                string(REGEX MATCHALL "[^\t\n]+\t[^\t\n]+\t[^\t\n]+"
                       _vfy_lines "${_vfy_numstat}")
                set(_vfy_paths "")
                foreach(_line ${_vfy_lines})
                    string(REGEX REPLACE "^[^\t]+\t[^\t]+\t" "" _p "${_line}")
                    list(APPEND _vfy_paths "${_p}")
                endforeach()
                if(_vfy_paths)
                    execute_process(
                        COMMAND git diff --quiet -- ${_vfy_paths}
                        WORKING_DIRECTORY "${TARGET_DIR}"
                        RESULT_VARIABLE _vfy_clean
                        OUTPUT_QUIET ERROR_QUIET
                    )
                    if(_vfy_clean EQUAL 0)
                        get_filename_component(_vfy_name "${_pf}" NAME)
                        message(STATUS
                            "  [${LABEL}] Stamp says applied but ${_vfy_name} "
                            "is missing from the tree — re-applying all.")
                        set(_stamp_stale TRUE)
                        break()
                    endif()
                endif()
            endforeach()
            if(NOT _stamp_stale)
                message(STATUS "  [${LABEL}] Patches already applied, skipping.")
                return()
            endif()
        else()
            message(STATUS "  [${LABEL}] Patch set changed, re-applying...")
        endif()
    endif()
    foreach(PATCH_FILE ${PATCH_FILES})
        get_filename_component(PATCH_NAME ${PATCH_FILE} NAME)
        set(PATCH_FILE_TO_APPLY "${PATCH_FILE}")
        # Prepare LF/CRLF patch variants for robust matching across clones.
        file(READ "${PATCH_FILE}" _patch_content)
        string(REPLACE "\r\n" "\n" _patch_lf "${_patch_content}")
        set(_patch_crlf "${_patch_lf}")
        string(REPLACE "\n" "\r\n" _patch_crlf "${_patch_crlf}")
        set(_tmp_patch_dir "${CMAKE_BINARY_DIR}/zephcore_patch_tmp/${LABEL}")
        file(MAKE_DIRECTORY "${_tmp_patch_dir}")
        set(_tmp_patch_lf "${_tmp_patch_dir}/${PATCH_NAME}.lf")
        set(_tmp_patch_crlf "${_tmp_patch_dir}/${PATCH_NAME}.crlf")
        file(WRITE "${_tmp_patch_lf}" "${_patch_lf}")
        file(WRITE "${_tmp_patch_crlf}" "${_patch_crlf}")
        # Dry-run check
        execute_process(
            COMMAND git apply --check "${PATCH_FILE_TO_APPLY}"
            WORKING_DIRECTORY "${TARGET_DIR}"
            RESULT_VARIABLE PATCH_CHECK
            ERROR_VARIABLE PATCH_ERR
        )
        # Retry check with explicit LF/CRLF variants to avoid EOL drift issues.
        if(NOT PATCH_CHECK EQUAL 0)
            execute_process(
                COMMAND git apply --check "${_tmp_patch_lf}"
                WORKING_DIRECTORY "${TARGET_DIR}"
                RESULT_VARIABLE PATCH_CHECK_LF
                ERROR_VARIABLE PATCH_ERR_LF
            )
            if(PATCH_CHECK_LF EQUAL 0)
                set(PATCH_FILE_TO_APPLY "${_tmp_patch_lf}")
                set(PATCH_CHECK 0)
            else()
                execute_process(
                    COMMAND git apply --check "${_tmp_patch_crlf}"
                    WORKING_DIRECTORY "${TARGET_DIR}"
                    RESULT_VARIABLE PATCH_CHECK_CRLF
                    ERROR_VARIABLE PATCH_ERR_CRLF
                )
                if(PATCH_CHECK_CRLF EQUAL 0)
                    set(PATCH_FILE_TO_APPLY "${_tmp_patch_crlf}")
                    set(PATCH_CHECK 0)
                else()
                    # Keep the most relevant error from the last attempted variant.
                    set(PATCH_ERR "${PATCH_ERR_CRLF}")
                endif()
            endif()
        endif()
        # Stale patches from previous build: reset affected files, retry
        if(NOT PATCH_CHECK EQUAL 0)
            # Extract affected file paths from numstat
            execute_process(
                COMMAND git apply --numstat "${PATCH_FILE_TO_APPLY}"
                WORKING_DIRECTORY "${TARGET_DIR}"
                OUTPUT_VARIABLE PATCH_NUMSTAT
                ERROR_QUIET
            )
            # numstat format: "adds\tdels\tpath"
            string(REGEX MATCHALL "[^\t\n]+\t[^\t\n]+\t[^\t\n]+" NUMSTAT_LINES "${PATCH_NUMSTAT}")
            set(PATCH_PATHS "")
            foreach(_line ${NUMSTAT_LINES})
                string(REGEX REPLACE "^[^\t]+\t[^\t]+\t" "" _path "${_line}")
                list(APPEND PATCH_PATHS "${_path}")
            endforeach()
            if(PATCH_PATHS)
                message(STATUS "  [${LABEL}] Resetting stale files for: ${PATCH_NAME}")
                execute_process(
                    COMMAND git checkout -- ${PATCH_PATHS}
                    WORKING_DIRECTORY "${TARGET_DIR}"
                    ERROR_QUIET
                )
                # Retry dry-run
                execute_process(
                    COMMAND git apply --check "${PATCH_FILE_TO_APPLY}"
                    WORKING_DIRECTORY "${TARGET_DIR}"
                    RESULT_VARIABLE PATCH_CHECK
                    ERROR_VARIABLE PATCH_ERR
                )
            endif()
        endif()
        if(NOT PATCH_CHECK EQUAL 0)
            message(FATAL_ERROR
                "ZephCore patch FAILED to apply: ${PATCH_NAME}\n"
                "Target tree: ${TARGET_DIR}\n"
                "Error:\n${PATCH_ERR}\n"
                "Upstream likely changed — rebase the patch:\n"
                "  cd ${TARGET_DIR}\n"
                "  git diff -- <file>   # inspect current upstream\n"
                "  # Regenerate: git diff -- <file> > ${PATCH_FILE}\n"
            )
        endif()
        # Apply
        execute_process(
            COMMAND git apply "${PATCH_FILE_TO_APPLY}"
            WORKING_DIRECTORY "${TARGET_DIR}"
            RESULT_VARIABLE APPLY_RESULT
            ERROR_VARIABLE APPLY_ERR
        )
        if(NOT APPLY_RESULT EQUAL 0)
            message(FATAL_ERROR "git apply failed unexpectedly: ${PATCH_NAME}\n${APPLY_ERR}")
        endif()
        message(STATUS "  [${LABEL}] Applied: ${PATCH_NAME}")
    endforeach()
    file(WRITE "${_stamp}" "${_stamp_hash}")
endfunction()

if(EXISTS ${ZEPHCORE_SOURCE_DIR}/patches/zephyr)
    message(STATUS "Applying ZephCore patches to Zephyr...")
    zephcore_apply_patches(
        "${ZEPHCORE_SOURCE_DIR}/patches/zephyr"
        "${ZEPHCORE_ZEPHYR_DIR}"
        "zephyr"
    )
endif()

# Apply patches to loramac-node module
if(EXISTS ${ZEPHCORE_SOURCE_DIR}/patches/modules/loramac-node)
    message(STATUS "Applying ZephCore patches to loramac-node...")
    zephcore_apply_patches(
        "${ZEPHCORE_SOURCE_DIR}/patches/modules/loramac-node"
        "${ZEPHCORE_MODULES_DIR}/lib/loramac-node"
        "loramac-node"
    )
endif()

# Apply patches to hal_espressif module (ESP32 BLE controller glue)
if(EXISTS ${ZEPHCORE_SOURCE_DIR}/patches/modules/hal-espressif)
    message(STATUS "Applying ZephCore patches to hal_espressif...")
    zephcore_apply_patches(
        "${ZEPHCORE_SOURCE_DIR}/patches/modules/hal-espressif"
        "${ZEPHCORE_MODULES_DIR}/hal/espressif"
        "hal-espressif"
    )
endif()
