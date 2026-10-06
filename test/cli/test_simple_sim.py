from pathlib import Path


def installed_sim(root: Path) -> Path:
    executable = root / ".artifacts/colcon/install/lib/simple_sim/simple_sim_run"
    executable.parent.mkdir(parents=True)
    executable.write_text("test executable placeholder")
    return executable
