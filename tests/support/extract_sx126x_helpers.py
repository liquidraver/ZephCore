"""Compile the actual added driver helpers without a Zephyr/kernel substitute."""
import re
import sys
from pathlib import Path


def extract(patch):
    section = patch.split("diff --git a/drivers/lora/native/sx126x/sx126x.c ", 1)[1]
    section = section.split("\ndiff --git ", 1)[0]
    source = "\n".join(line[1:] for line in section.splitlines()
                       if line.startswith(("+", " ")) and not line.startswith("+++"))
    functions = []
    for name in ("sx126x_retention_debug_check", "sx126x_apply_heltec_rx_patch",
                 "sx126x_install_retention_list"):
        start = re.search(r"^static (?:void|int) " + name + r"\(", source, re.M)
        if start is None:
            raise ValueError("Driver helper missing: " + name)
        opening = source.index("{", start.start())
        depth = 0
        for end in range(opening, len(source)):
            depth += (source[end] == "{") - (source[end] == "}")
            if depth == 0:
                functions.append(source[start.start():end + 1])
                break
        else:
            raise ValueError("Incomplete driver helper: " + name)
    return "\n\n".join(functions) + "\n"


if __name__ == "__main__":
    Path(sys.argv[2]).write_text(extract(Path(sys.argv[1]).read_text()), encoding="utf-8")
