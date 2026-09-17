# Why We Used Wrapped Python (Historical)

> **⚠️ DEPRECATED**: The `pyp` wrapper has been replaced by `uv run`. All Python execution now uses `uv` for virtual environment management and project isolation. See `docs/why-we-migrated-to-uv.md` for the migration rationale. This document remains as a historical reference for the design philosophy behind wrapped Python execution.

> A deep dive into how our `pyp` wrapper exemplifies modern software architecture principles and improves developer experience

---

## The Problem

As projects grow in complexity, managing Python environments across different development scenarios becomes increasingly difficult. Traditional approaches often lead to:

- **Environment inconsistencies** between developers and CI/CD pipelines
- **Complex setup procedures** for new contributors
- **Fragile tooling** that breaks in git worktrees or different directory structures
- **Poor developer experience** with manual environment activation and management

Have you ever encountered any of these issues?

```bash
# ❌ Traditional approach - fragile and error-prone
source .acados_venv/bin/activate
python script.py
# Wait, which python am I using?
# Does this work in CI/CD?
# What if I'm in a git worktree?

# ❌ Another common approach
.acados_venv/bin/python script.py
# But what about ROS2 environment?
# What if the venv is in a different location?
```

---

## The Solution (Historical): Wrapped Python with Modern Design

The legacy `pyp` implementation demonstrated a **wrapped Python approach** that combined modern software design principles with practical benefits. It has since been replaced by `uv run`, which provides equivalent functionality out of the box.

### Usage (Legacy)

```bash
# ✅ Simple and consistent (historical)
./pyp script.py           # Run a Python script (now: uv run python script.py)
./pyp -m module_name      # Run a module (now: uv run python -m module_name)
./pyp                     # Interactive shell (now: uv run python)
./pyp -c "print('hello')" # Execute code (now: uv run python -c "print('hello')")
```

### What Makes It Modern?

#### 1. **Abstraction & Encapsulation**

> "A good interface hides complexity behind simplicity."

```python
# Users don't need to know (historical - now handled by `uv run`):
# - Where the venv is located
# - How to detect git worktrees
# - How to setup ROS2 environment
# - How to handle different Python versions

uv run python script.py  # Just works™ (replaces ./pyp script.py)
```

**Benefits**:
- Cognitive load reduction for developers
- Single point of maintenance for environment logic
- Easy to update without changing user code

#### 2. **Git-Aware Architecture**

The wrapper (and now `uv`) intelligently handles different repository configurations:

```bash
# ✅ Normal repositories (uv handles this automatically)
$ cd /path/to/repo
$ uv run python script.py

# ✅ Git worktrees (uv detects pyproject.toml at repo root)
$ cd /path/to/worktree
$ uv run python script.py
# → Uses .venv from the main worktree / shared project environment

# ✅ Custom paths (set AP_PNC_DIR for env.py resolution)
$ AP_PNC_DIR=/custom/path uv run python script.py
# → Uses /custom/path for ACADOS library path etc.
```

**Implementation Highlight** (now in `ap_pnc/env.py`):

```python
def get_git_root() -> Path:
    """Find the git repository root, handling worktrees via commondir."""
    ap_pnc_dir = os.environ.get("AP_PNC_DIR", "")
    if ap_pnc_dir:
        return Path(ap_pnc_dir).resolve()

    try:
        result = subprocess.run(
            ["git", "rev-parse", "--git-dir"],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            return Path.cwd()

        git_dir = Path(result.stdout.strip())
        if git_dir.is_absolute():
            git_file = git_dir
        else:
            git_file = Path.cwd() / git_dir

        if "/worktrees/" in str(git_file):
            commondir_file = git_file / "commondir"
            if commondir_file.exists():
                commondir = commondir_file.read_text().strip()
                original_git_dir = (git_file / commondir).resolve()
                return original_git_dir.parent

        return git_file.parent.resolve()
    except (subprocess.SubprocessError, FileNotFoundError):
        return Path.cwd()
```

#### 3. **Integration with Modern Tooling**

**uv Integration** (current approach):

```bash
# No venv activation needed!
uv run python scripts/simulate.py
uv run pytest tests/
uv run ruff check src/
```

**CI/CD Pipelines**:

```yaml
# .github/workflows/test.yml
name: Tests
on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Install uv
        run: curl -LsSf https://astral.sh/uv/install.sh | sh
      - name: Install dependencies
        run: uv sync --group dev
      - name: Run tests
        run: uv run pytest
      - name: Lint
        run: uv run ruff check
```

**Pre-commit Hooks**:

```yaml
# .pre-commit-config.yaml
repos:
  - repo: local
    hooks:
      - id: ruff-lint
        name: Run ruff linter
        entry: uv run ruff check --fix
        language: system

      - id: run-tests
        name: Run pytest
        entry: uv run pytest
        language: system
```

#### 4. **Automatic Environment Management**

The wrapper (and now `uv`) handles environment management. The ACADOS and ROS2 setup logic lives in `ap_pnc/env.py`:

```python
def setup_env(ros: bool = False) -> None:
    """Configure os.environ with acados paths and optionally ROS2."""
    git_root = get_git_root()
    os.environ.setdefault("AP_PNC_DIR", str(git_root))
    os.environ.setdefault("ACADOS_SOURCE_DIR", str(git_root / ".artifacts" / "acados"))

    lib_path = str(git_root / ".artifacts" / "acados" / "lib")
    existing_ld = os.environ.get("LD_LIBRARY_PATH", "")
    if lib_path not in existing_ld:
        os.environ["LD_LIBRARY_PATH"] = f"{lib_path}:{existing_ld}"

    pythonpath = os.environ.get("PYTHONPATH", "")
    git_root_str = str(git_root)
    if git_root_str not in pythonpath:
        os.environ["PYTHONPATH"] = f"{git_root_str}:{pythonpath}"

    if ros:
        ros_env = source_ros2_env()
        os.environ.update(ros_env)
```

**What happens automatically**:
- ✅ Detects git root (including worktrees)
- ✅ Sets `LD_LIBRARY_PATH` for ACADOS
- ✅ Sets `PYTHONPATH` to project root
- ✅ Sources ROS2 setup.bash when requested

#### 5. **Type Safety & Modern Python**

```python
# uv handles Python interpreter discovery automatically.
# No wrapper code needed — `uv run` finds .venv at project root.
```

**Modern Python Features (in `ap_pnc/env.py`)**:
- ✅ **Type hints** (`Path | None`, `-> int`)
- ✅ **Type checking** (compatible with mypy/pyright)
- ✅ **Docstrings** with Google style
- ✅ **Error handling** with helpful user feedback
- ✅ **Pathlib** instead of string manipulation

#### 6. Developer Experience Comparison

| Legacy Approach | Wrapped Python (`pyp`) | Current (`uv run`) |
|-----------------|------------------------|---------------------|
| `source .acados_venv/bin/activate` | `./pyp` | `uv run` |
| `which python? which pip?` | Automatic detection | Auto-detect via `uv` |
| Breaks in worktrees | Worktree-aware | Worktree-compatible |
| Manual ROS2 setup | Automatic sourcing | `ap_pnc.env.setup_env(ros=True)` |
| Unclear errors | Colored, helpful messages | Standard `uv` output |

**Real output (current)**:

```
$ uv run python script.py

[Output from uv's environment detection...]
[Script output here...]
```

**Error handling (current)**:

```
$ uv run python script.py
error: Failed to run `python script.py`
  Caused by: No virtual environment found; run `uv sync` first
```

#### 7. Scalability & Maintainability

**Modular Design** - API surface now lives in `ap_pnc/env.py`:

```python
# Git logic
env.get_git_root() -> Path

# ROS2 integration
env.source_ros2_env() -> dict[str, str]

# Combined setup
env.setup_env(ros: bool = False) -> None
```

**Easy to extend**:

```python
# Future enhancements are straightforward:
def setup_cuda_environment(env: dict) -> None:
    """Setup CUDA environment variables."""
    cuda_paths = ["/usr/local/cuda", "/opt/cuda"]
    for cuda_path in cuda_paths:
        if Path(cuda_path).exists():
            env["CUDA_HOME"] = cuda_path
            env["PATH"] = f"{cuda_path}/bin:{env.get('PATH', '')}"
            env["LD_LIBRARY_PATH"] = f"{cuda_path}/lib64:{env.get('LD_LIBRARY_PATH', '')}"
            return

def detect_gpu_requirements() -> bool:
    """Check if GPU is needed based on script imports."""
    # Parse script AST to check for torch/cupy/etc.
    return False

def show_version() -> None:
    """Display Python and environment versions."""
    import sys
    print(f"Python: {sys.version}")
    print(f"Virtual environment: {python_bin.parent.parent}")
```

#### 8. Real-World Benefits

**For New Contributors**:
- ⚡ Faster onboarding (run `uv run ap-pnc install` instead of reading docs)
- 📚 Self-documenting (standard `uv` CLI)
- 🎯 Clear expectations (colored output shows what's happening)

**For Maintainers**:
- 🔧 Single point of maintenance for environment logic
- 🐛 Fewer support requests (environment issues caught automatically)
- ✅ Consistent behavior across all environments

**For CI/CD**:
- 🚀 No flaky tests from environment mismatches
- 📦 Reproducible builds (same commands locally and in CI)
- 🔄 Easy debugging (same output format everywhere)

**For Development**:
- 💡 Less cognitive load (don't think about environments)
- ⌨️ Less typing (shorter commands)
- 😊 Happier developers (less friction, more productivity)

---

## Software Design Principles Demonstrated

### SOLID Principles

1. **Single Responsibility Principle**
   - Each function does one thing well
   - `find_git_root()` only finds git root
   - `find_python_interpreter()` only finds Python

2. **Open/Closed Principle**
   - Open for extension (add new environment setups)
   - Closed for modification (core logic remains stable)

3. **Liskov Substitution Principle**
   - Can swap virtual environment implementations
   - Consistent interface for all venv types

4. **Interface Segregation Principle**
   - Simple, focused interface (`uv run python [args]` or `uv run ap-pnc <cmd>`)
   - No forced dependencies on unused features

5. **Dependency Inversion Principle**
   - Depends on abstractions (Path, subprocess)
   - Not concrete implementations

### Other Principles

- **DRY (Don't Repeat Yourself)**: Environment logic in one place
- **KISS (Keep It Simple, Stupid)**: Simple interface, complex implementation hidden
- **Principle of Least Astonishment**: Behaves as users expect
- **Separation of Concerns**: Git, venv, and ROS2 logic separated
- **Fail Fast**: Clear errors immediately when something is wrong

---

## Performance & Benchmarks

**Startup Overhead**:

```bash
$ time uv run python -c "print('hello')"
hello
uv run python -c "print('hello')"  0.01s user 0.02s system 12% cpu 0.250 total
```

The overhead is comparable to `pyp` (~250ms) for the benefits gained.

---

## Migration Guide

### From Legacy to uv

**Before (legacy)**:
```bash
source .acados_venv/bin/activate
python scripts/simulate.py
```

**After (current)**:
```bash
uv run python scripts/simulate.py
uv pip install -r requirements.txt
```

### In Scripts

**Before**:
```python
#!/usr/bin/env python3
import sys
sys.path.insert(0, '/path/to/venv/lib/python3.10/site-packages')
```

**After**:
```python
#!/usr/bin/env python3
# uv run handles the environment
import your_module
```

### In CI/CD

**Before**:
```yaml
- name: Setup Python
  run: |
    source .acados_venv/bin/activate
    pip install -r requirements.txt
- name: Run tests
  run: |
    source .acados_venv/bin/activate
    pytest
```

**After**:
```yaml
- name: Install uv and dependencies
  run: |
    curl -LsSf https://astral.sh/uv/install.sh | sh
    uv sync --group dev
- name: Run tests
  run: uv run pytest
```

---

## Best Practices

### ✅ DO

- Use `uv run` for all Python execution
- Use `uv run python -m module` for running Python modules
- Check for venv existence with `uv sync`
- Use `ap_pnc.env.setup_env()` for ACADOS/ROS2 paths
- Use type hints in Python code

### ❌ DON'T

- Hardcode venv paths in scripts
- Assume venv is activated
- Use `python` directly without `uv run`
- Skip error handling
- Ignore git worktree scenarios

---

## Future Enhancements (with uv)

- [x] `uv run` provides consistent Python execution
- [x] `uv sync` manages dependencies
- [x] `uv pip install` replaces pip without activation
- [ ] Integrate ROS2 sourcing into `ap_pnc.env.setup_env(ros=True)`
- [ ] Add `uv run ap-pnc env` command for environment diagnostics
- [ ] Support for GPU/TPU configs via environment flags

---

## Conclusion

The wrapped Python approach (whether `pyp` historically or `uv` currently) represents **modern software design** at its best:

- ✅ **Simple interface** for complex operations
- ✅ **Works everywhere** (repos, worktrees, CI/CD)
- ✅ **Self-documenting** with clear error messages
- ✅ **Maintainable** with modular, typed code
- ✅ **Extensible** for future requirements
- ✅ **Developer-focused** respecting time and mental energy

This isn't just about convenience—it's about **building software that empowers developers** to be productive and happy while providing robust, reliable behavior across all development environments.

### The Bottom Line

> **"Good software design is not about following rules—it's about creating tools that make people's lives better."**

The `pyp` wrapper was a small tool that made a big difference, and `uv` continues this philosophy with a more standardized and widely-supported approach.

---

## Resources

- **CLI**: `ap_pnc/cli.py` - CLI entry points
- **Environment**: `ap_pnc/env.py` - Git root, ACADOS, ROS2 setup
- **uv**: [uv documentation](https://docs.astral.sh/uv/) - Python package and project manager

---

*Last updated: June 2026 — migrated from `pyp` to `uv`*
*Contributors: AP-PnC Team*
