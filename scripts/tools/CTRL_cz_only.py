#!/usr/bin/env python3

import fileinput
import sys
import os
import subprocess
from dataclasses import dataclass

@dataclass
class MPCConfig:
    qv_baseline: float = 0.0
    qv_yb: float = 1.8

    cz_add: str = "F_aero_b_frd[2] =   - 1.0*(cz_state) * cs.dot(v,v)"
    cz_no: str = "F_aero_b_frd[2] =   0.0"

    @property
    def config_path(self):
        return os.path.join(root_dir(), 'src', 'nmpc', 'config.py')
    @property
    def model_path(self):
        return os.path.join(root_dir(), 'src', 'nmpc', 'model.py')

def root_dir():
    return os.environ.get('AP_PNC_DIR')

def qv_modify():
    config = MPCConfig()
    config_path = config.config_path

    with fileinput.FileInput(config_path, inplace=True, backup='.bak') as file:
        for line in file:
            if "qv =" in line.strip():
                print(f"    qv = {config.qv_baseline}")
            else:
                print(line, end='')
    print("[CTRL-CZ-ONLY] yb: qv modified")

def cz_modify():
    config = MPCConfig()
    model_path = config.model_path

    with fileinput.FileInput(model_path, inplace=True, backup='.bak') as file:
        skip_next = False
        for line in file:
            if skip_next:
                skip_next = False
                continue
            print(line, end='')
            if "# MARKER" in line:
                skip_next = True
                print(f"    {config.cz_add}")
    print("[CTRL-CZ-ONLY] model: cz modified")
def compile_acados():
    subprocess.run(['cocp'], check=True)
    print("[CTRL-CZ-ONLY] : acados compiled")
if __name__ == "__main__":
    qv_modify()
    cz_modify()
    compile_acados()
