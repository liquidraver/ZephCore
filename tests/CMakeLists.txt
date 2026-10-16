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
  unit/ascon_tests.cpp
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
  unit/kiss_modem_tests.cpp
  unit/zcpfs_tests.cpp
  unit/zcpfs_peer_tests.cpp
  unit/zcpfs_glue_tests.cpp
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
  ${PRODUCTION}/app/KissModem.cpp
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
# ascon and zcpfs are portable C99 and build as their own libraries with their own,
# stricter flags. ascon depends on nothing; zcpfs on ascon and on the Monocypher
# objects of the executable.
set(STRICT_C -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror)
add_library(ascon STATIC ${PRODUCTION}/lib/ascon/ascon.c)
target_include_directories(ascon PUBLIC ${PRODUCTION}/lib/ascon)
set(ZCPFS "${PRODUCTION}/lib/zcpfs")
add_library(zcpfs STATIC
  ${ZCPFS}/zcpfs_prim.c
  ${ZCPFS}/zcpfs_curve.c
  ${ZCPFS}/zcpfs_drbg.c
  ${ZCPFS}/zcpfs_handshake.c
  ${ZCPFS}/zcpfs_session.c
  ${ZCPFS}/zcpfs_stateless.c
  ${ZCPFS}/zcpfs_login.c
  ${ZCPFS}/zcpfs_peer.c
)
set_target_properties(ascon zcpfs PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
target_include_directories(zcpfs PUBLIC ${ZCPFS}/include ${ZCPFS} PRIVATE ${PRODUCTION}/lib/monocypher)
if(NOT MSVC)
  target_compile_options(ascon PRIVATE ${STRICT_C})
  target_compile_options(zcpfs PRIVATE ${STRICT_C})
endif()
target_link_libraries(zcpfs PUBLIC ascon)
target_link_libraries(zephcore_unit PRIVATE zcpfs)
if(ZEPHCORE_TEST_SANITIZERS)
  if(WIN32 OR MSVC)
    message(FATAL_ERROR "Use the Linux host target for address/undefined sanitizers")
  endif()
  target_compile_options(zephcore_unit PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -fno-pie)
  target_link_options(zephcore_unit PRIVATE -fsanitize=address,undefined -no-pie)
  target_compile_options(ascon PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -fno-pie)
  target_compile_options(zcpfs PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -fno-pie)
endif()

# zcpfs inside the firmware (unit/zcpfs_mesh_tests.cpp): whole companions with the
# switch on. A separate executable because mesh::Mesh has more virtual methods with
# it; and the firmware's own src/Utils.cpp, over OpenSSL (support/psa/crypto.h), so
# that legacy encryption is the real one.
find_package(OpenSSL REQUIRED COMPONENTS Crypto)
add_executable(zephcore_zcpfs_mesh
  unit/main.cpp
  unit/zcpfs_mesh_tests.cpp
  ${PRODUCTION}/src/Mesh.cpp
  ${PRODUCTION}/src/Dispatcher.cpp
  ${PRODUCTION}/src/Packet.cpp
  ${PRODUCTION}/src/Identity.cpp
  ${PRODUCTION}/src/Utils.cpp
  ${PRODUCTION}/src/ContentionTracker.cpp
  ${PRODUCTION}/lib/monocypher/monocypher.c
  ${PRODUCTION}/lib/monocypher/monocypher-ed25519.c
  ${PRODUCTION}/helpers/BaseChatMesh.cpp
  ${PRODUCTION}/helpers/AdvertDataHelpers.cpp
  ${PRODUCTION}/helpers/zcpfs/ZcpfsChat.cpp
  ${PRODUCTION}/helpers/zcpfs/ZcpfsSeed.cpp
  ${PRODUCTION}/helpers/zcpfs/ZcpfsServer.cpp
  ${PRODUCTION}/helpers/ClientACL.cpp
  support/zephyr_fs_none.cpp
)
target_compile_features(zephcore_zcpfs_mesh PRIVATE cxx_std_17)
target_compile_definitions(zephcore_zcpfs_mesh PRIVATE CONFIG_ZEPHCORE_ZCPFS=1 CONFIG_ZEPHCORE_ZCPFS_PEERS=4
  CONFIG_ZEPHCORE_PACKET_LOGGING=0 CONFIG_ZEPHCORE_MAIN_LOG_LEVEL=0)
target_include_directories(zephcore_zcpfs_mesh PRIVATE
  support unit ${PRODUCTION} ${PRODUCTION}/include ${PRODUCTION}/helpers ${PRODUCTION}/helpers/compat
  ${PRODUCTION}/lib/monocypher ${PRODUCTION}/adapters/radio)
if(NOT MSVC)
  target_compile_options(zephcore_zcpfs_mesh PRIVATE -Wall -Wextra -Wpedantic)
endif()
target_link_libraries(zephcore_zcpfs_mesh PRIVATE zcpfs OpenSSL::Crypto)
if(ZEPHCORE_TEST_SANITIZERS)
  target_compile_options(zephcore_zcpfs_mesh PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -fno-pie)
  target_link_options(zephcore_zcpfs_mesh PRIVATE -fsanitize=address,undefined -no-pie)
endif()
