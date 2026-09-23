#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Print the flash size of a built image's SoC flash node as '<N>MB'.

Used by build.sh to pass esptool merge-bin a --flash-size that matches the
board, read from the final devicetree rather than duplicated per board.

    dts_flash_size.py [path/to/zephyr.dts]   (default: build/zephcore/zephyr/zephyr.dts)
"""
import re
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "build/zephcore/zephyr/zephyr.dts"
dts = open(path, encoding="utf-8", errors="replace").read()


def parse_cell(tok: str) -> int:
    t = tok.strip()
    return int(t, 16) if t.lower().startswith("0x") else int(t)


m = re.search(
    r"flash0:\s*flash@[^{]*\{[^}]*?reg\s*=\s*<\s*(?:0x[0-9a-fA-F]+|[0-9]+)\s+"
    r"((?:0x)?[0-9a-fA-F]+)\s*>",
    dts,
    re.DOTALL,
)
if not m:
    m = re.search(
        r"compatible\s*=\s*\"soc-nv-flash\"\s*;\s*[\s\S]*?"
        r"reg\s*=\s*<\s*(?:0x[0-9a-fA-F]+|[0-9]+)\s+((?:0x)?[0-9a-fA-F]+)\s*>",
        dts,
    )
if not m:
    raise SystemExit("flash reg not found in " + path)

size = parse_cell(m.group(1))
print(str(size // 1048576) + "MB")
