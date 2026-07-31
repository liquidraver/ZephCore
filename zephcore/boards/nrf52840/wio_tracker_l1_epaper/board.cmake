# Copyright (c) 2025 MeshCore
# SPDX-License-Identifier: Apache-2.0

board_runner_args(nrfjprog "--softreset")
include(${ZEPHYR_BASE}/boards/common/nrfjprog.board.cmake)
include(${ZEPHYR_BASE}/boards/common/nrfutil.board.cmake)
