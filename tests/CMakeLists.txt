cmake_minimum_required(VERSION 3.20)
project(zephcore_host_tests LANGUAGES C CXX)

option(ZEPHCORE_TEST_SANITIZERS "Enable host address/undefined behavior sanitizers" OFF)
set(PRODUCTION "${CMAKE_CURRENT_LIST_DIR}/../zephcore")
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(SX126X_PATCH "${PRODUCTION}/patches/zephyr/0003-lora-sx126x-native.patch")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${SX126X_PATCH}" "${CMAKE_CURRENT_LIST_DIR}/support/extract_sx126x_helpers.py")
execute_process(COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_LIST_DIR}/support/extract_sx126x_helpers.py"
  "${SX126X_PATCH}" "${CMAKE_CURRENT_BINARY_DIR}/sx126x_helpers.inc"
  COMMAND_ERROR_IS_FATAL ANY)
add_executable(zephcore_unit
  unit/main.cpp
  unit/packet_tests.cpp
  unit/policy_tests.cpp
  unit/filter_tests.cpp
  ${PRODUCTION}/helpers/FwdFilter.cpp
  unit/pool_tests.cpp
  unit/contention_tests.cpp
  unit/advert_tests.cpp
  unit/dispatcher_tests.cpp
  unit/identity_tests.cpp
  unit/timesync_tests.cpp
  unit/helper_tests.cpp
  unit/radio_tests.cpp
  unit/sx126x_heltec_tests.cpp
  unit/lora_timing_tests.cpp
  unit/prefs_codec_tests.cpp
  unit/prefs_json_tests.cpp
  unit/lpp_tests.cpp
  unit/telemetry_text_tests.cpp
  unit/gps_cli_text_tests.cpp
  unit/gps_guard_tests.cpp
  unit/kiss_modem_tests.cpp
  unit/chat_contacts_tests.cpp
  support/hash_capture.cpp
  support/identity_utils_boundary.cpp
  ${PRODUCTION}/src/Dispatcher.cpp
  ${PRODUCTION}/src/Identity.cpp
  ${PRODUCTION}/lib/monocypher/monocypher.c
  ${PRODUCTION}/lib/monocypher/monocypher-ed25519.c
  ${PRODUCTION}/src/Packet.cpp
  ${PRODUCTION}/src/StaticPoolPacketManager.cpp
  ${PRODUCTION}/src/Mesh.cpp
  ${PRODUCTION}/helpers/BaseChatMesh.cpp
  ${PRODUCTION}/src/ContentionTracker.cpp
  ${PRODUCTION}/helpers/AdvertDataHelpers.cpp
  ${PRODUCTION}/helpers/MeshTimeSync.cpp
  ${PRODUCTION}/helpers/battery_curve.c
  ${PRODUCTION}/adapters/radio/CadController.cpp
  ${PRODUCTION}/helpers/PrefsCodec.cpp
  ${PRODUCTION}/helpers/PrefsJson.cpp
  ${PRODUCTION}/helpers/ConfigSerializer.cpp
  ${PRODUCTION}/helpers/ui/telemetry_text.cpp
  ${PRODUCTION}/helpers/gps_cli_text.cpp
  ${PRODUCTION}/helpers/gps_guard.cpp
  ${PRODUCTION}/app/KissModem.cpp
)
target_compile_features(zephcore_unit PRIVATE cxx_std_17)
target_compile_definitions(zephcore_unit PRIVATE CONFIG_ZEPHCORE_PACKET_LOGGING=0
  CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK=1 CONFIG_ZEPHCORE_UI_DESIGN_BUTTON=0)
target_include_directories(zephcore_unit PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}" support unit ${PRODUCTION} ${PRODUCTION}/include ${PRODUCTION}/helpers ${PRODUCTION}/helpers/compat
  ${PRODUCTION}/lib/monocypher
  ${PRODUCTION}/adapters/radio
  ${PRODUCTION}/patches/zephyr-new/include ${PRODUCTION}/patches/zephyr-new/drivers/lora/lr11xx)
if(MSVC)
  target_compile_options(zephcore_unit PRIVATE /W4)
else()
  target_compile_options(zephcore_unit PRIVATE -Wall -Wextra -Wpedantic)
endif()
if(ZEPHCORE_TEST_SANITIZERS)
  if(WIN32 OR MSVC)
    message(FATAL_ERROR "Use the Linux host target for address/undefined sanitizers")
  endif()
  target_compile_options(zephcore_unit PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -fno-pie)
  target_link_options(zephcore_unit PRIVATE -fsanitize=address,undefined -no-pie)
endif()
