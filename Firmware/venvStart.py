#!/usr/bin/env python3
"""
bootstrap.py

Single-file bootstrap script to:
- Create a virtual environment in ./venv
- Install packages from requirements.txt
- Re-run the current script using the venv's Python

Usage:
    python3 bootstrap.py

Notes:
- Safe for macOS/Homebrew (PEP 668 compliant)
- No system-wide installs
"""

import os
import sys
import subprocess
import venv
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
VENV_DIR = PROJECT_ROOT / "venv"
REQ_FILE = PROJECT_ROOT / "requirements.txt"


def is_running_in_venv() -> bool:
    return sys.prefix != sys.base_prefix


def venv_python_path() -> Path:
    if os.name == "nt":
        return VENV_DIR / "Scripts" / "python.exe"
    else:
        return VENV_DIR / "bin" / "python"


def ensure_requirements_file():
    if not REQ_FILE.exists():
        REQ_FILE.write_text(
            "# Add your project dependencies here\n"
            "cantools\n"
        )
        print("Created default requirements.txt")


def create_venv():
    if VENV_DIR.exists():
        print("Virtual environment already exists")
        return
    print("Creating virtual environment...")
    venv.EnvBuilder(with_pip=True).create(VENV_DIR)


def install_requirements(python_exe: Path):
    print("Installing dependencies from requirements.txt...")
    subprocess.check_call([
        str(python_exe),
        "-m", "pip",
        "install", "--upgrade", "pip"
    ])
    subprocess.check_call([
        str(python_exe),
        "-m", "pip",
        "install", "-r", str(REQ_FILE)
    ])


def rerun_in_venv(python_exe: Path):
    print("Re-running script inside virtual environment...")
    os.execv(str(python_exe), [str(python_exe), *sys.argv])


def main():
    ensure_requirements_file()

    if not is_running_in_venv():
        create_venv()
        python_exe = venv_python_path()
        install_requirements(python_exe)
        rerun_in_venv(python_exe)

    # --- Your actual project code starts here ---
    print("Virtual environment active ✔")
    print("Python executable:", sys.executable)

    # Example sanity check
    try:
        import cantools
        print("cantools OK:", cantools.__version__)
    except Exception as e:
        print("cantools import failed:", e)


if __name__ == "__main__":
    main()
