cmake_minimum_required(VERSION 3.20)
project(zephcore_host_tests LANGUAGES C CXX)

option(ZEPHCORE_TEST_SANITIZERS "Enable host address/undefined behavior sanitizers" OFF)
set(PRODUCTION "${CMAKE_CURRENT_LIST_DIR}/../zephcore")
add_executable(zephcore_unit
  unit/main.cpp
  unit/packet_tests.cpp
  unit/policy_tests.cpp
  unit/pool_tests.cpp
  unit/contention_tests.cpp
  unit/advert_tests.cpp
  unit/dispatcher_tests.cpp
  unit/identity_tests.cpp
  unit/timesync_tests.cpp
  unit/helper_tests.cpp
  unit/radio_tests.cpp
  unit/lora_timing_tests.cpp
  unit/prefs_codec_tests.cpp
  unit/prefs_json_tests.cpp
  unit/lpp_tests.cpp
  unit/telemetry_text_tests.cpp
  support/hash_capture.cpp
  support/identity_utils_boundary.cpp
  ${PRODUCTION}/src/Dispatcher.cpp
  ${PRODUCTION}/src/Identity.cpp
  ${PRODUCTION}/lib/monocypher/monocypher.c
  ${PRODUCTION}/lib/monocypher/monocypher-ed25519.c
  ${PRODUCTION}/src/Packet.cpp
  ${PRODUCTION}/src/StaticPoolPacketManager.cpp
  ${PRODUCTION}/src/ContentionTracker.cpp
  ${PRODUCTION}/helpers/AdvertDataHelpers.cpp
  ${PRODUCTION}/helpers/MeshTimeSync.cpp
  ${PRODUCTION}/helpers/battery_curve.c
  ${PRODUCTION}/adapters/radio/CadController.cpp
  ${PRODUCTION}/helpers/PrefsCodec.cpp
  ${PRODUCTION}/helpers/PrefsJson.cpp
  ${PRODUCTION}/helpers/ConfigSerializer.cpp
  ${PRODUCTION}/helpers/ui/telemetry_text.cpp
)
target_compile_features(zephcore_unit PRIVATE cxx_std_17)
target_compile_definitions(zephcore_unit PRIVATE CONFIG_ZEPHCORE_PACKET_LOGGING=0
  CONFIG_ZEPHCORE_UI_DESIGN_JOYSTICK=1 CONFIG_ZEPHCORE_UI_DESIGN_BUTTON=0)
target_include_directories(zephcore_unit PRIVATE
  support unit ${PRODUCTION} ${PRODUCTION}/include ${PRODUCTION}/helpers ${PRODUCTION}/helpers/compat
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
