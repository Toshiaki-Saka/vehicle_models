#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Archlink Systems Lab
"""Launch the vehicle model simulation GUI.

    python run_gui.py

Works from a checkout without installing anything: the package directory next
to this file is put on the path first.
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main() -> int:
    try:
        import tkinter  # noqa: F401
    except ImportError:
        print("tkinter is missing. On Debian/Ubuntu: sudo apt install "
              "python3-tk", file=sys.stderr)
        return 1
    try:
        import matplotlib  # noqa: F401
        import numpy  # noqa: F401
    except ImportError as exc:
        print("missing dependency: %s\nInstall with: pip install -r "
              "requirements.txt" % exc, file=sys.stderr)
        return 1

    from vehicle_models_py.gui import main as gui_main
    gui_main()
    return 0


if __name__ == "__main__":
    sys.exit(main())
