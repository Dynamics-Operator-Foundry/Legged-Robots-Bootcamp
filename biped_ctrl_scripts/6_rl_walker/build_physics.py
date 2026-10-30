#!/usr/bin/env python3
"""Build the checked-in generated C kernels without requiring Cython.

The symbolic derivation in ``5_walker_3D_control`` already contains both the
generated wrapper C and numerical C sources.  Compiling those files directly
is faster and more reproducible than re-running SymPy/autowrap.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys
import sysconfig

import numpy as np


HERE = Path(__file__).resolve().parent
KERNEL_ROOT = (
    HERE.parent
    / "5_walker_3D_control"
    / "f_walker_3D"
    / "dynamics"
    / "compiled_funcs"
)
KERNELS = {
    "B_matrix": 0,
    "collision": 1,
    "J_l_ss": 2,
    "J_r_ss": 3,
    "Jdot_l_ss": 4,
    "Jdot_r_ss": 5,
    "Mass_matrix": 6,
    "P_L": 7,
    "P_R": 8,
}


def build(force: bool = False) -> list[Path]:
    suffix = sysconfig.get_config_var("EXT_SUFFIX") or ".so"
    include = sysconfig.get_paths()["include"]
    compiler = os.environ.get("CC", "cc")
    outputs: list[Path] = []

    for folder, number in KERNELS.items():
        source_dir = KERNEL_ROOT / folder
        module = f"wrapper_module_{number}"
        output = source_dir / f"{module}{suffix}"
        outputs.append(output)
        if output.exists() and not force:
            print(f"ready: {output.relative_to(HERE.parent.parent)}")
            continue

        command = [
            compiler,
            "-O3",
            "-shared",
            "-fPIC",
            "-std=c99",
            f"-I{include}",
            f"-I{np.get_include()}",
            str(source_dir / f"{module}.c"),
            str(source_dir / f"wrapped_code_{number}.c"),
            "-lm",
            "-o",
            str(output),
        ]
        print(f"building {folder}/{module}")
        subprocess.run(command, check=True)

    return outputs


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true", help="rebuild existing kernels")
    args = parser.parse_args()
    build(force=args.force)


if __name__ == "__main__":
    main()
