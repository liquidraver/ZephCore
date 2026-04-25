#!/usr/bin/env python3
"""
Compress ota_page.html → ota_page.h (gzip + C array).

The HTTP server serves the page with `Content-Encoding: gzip`, so the
on-flash representation is the gzip stream directly. Re-run this any time
ota_page.html changes:

    python3 compress_html.py
"""

import gzip
import pathlib

HERE = pathlib.Path(__file__).parent
HTML = HERE / "ota_page.html"
OUT = HERE / "ota_page.h"

raw = HTML.read_bytes()
# mtime=0 so the gzip output is deterministic across builds.
gz = gzip.compress(raw, compresslevel=9, mtime=0)

lines = []
for i in range(0, len(gz), 16):
    chunk = gz[i:i + 16]
    hexes = ", ".join(f"0x{b:02x}" for b in chunk)
    lines.append(f"    {hexes},")

body = "\n".join(lines)

OUT.write_text(
    "/*\n"
    " * Auto-generated from ota_page.html — do not edit manually.\n"
    " * Regenerate: python3 compress_html.py\n"
    " */\n"
    "\n"
    "#pragma once\n"
    "\n"
    "#include <stdint.h>\n"
    "\n"
    f"#define OTA_PAGE_GZ_SIZE {len(gz)}\n"
    "\n"
    f"static const uint8_t ota_page_gz[OTA_PAGE_GZ_SIZE] = {{\n"
    f"{body}\n"
    "};\n",
    encoding="utf-8",
)

print(f"Wrote {OUT.name}: {len(raw)} B HTML -> {len(gz)} B gzip")
