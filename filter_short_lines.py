#!/usr/bin/env python3

import os
import sys
import tempfile


def filter_file_in_place(path: str, minimum_length: int = 10) -> None:
    file_mode = os.stat(path).st_mode
    directory = os.path.dirname(os.path.abspath(path)) or "."

    with open(path, "r", encoding="utf-8") as source, tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=directory, delete=False
    ) as destination:
        temporary_path = destination.name
        for line in source:
            if len(line.rstrip("\r\n")) >= minimum_length:
                destination.write(line)

    try:
        os.chmod(temporary_path, file_mode)
        os.replace(temporary_path, path)
    except Exception:
        os.unlink(temporary_path)
        raise


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(f"Usage: {sys.argv[0]} FILE")

    filter_file_in_place(sys.argv[1])
