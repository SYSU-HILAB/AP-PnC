# Why We Use Wrapped Python with Modern Software Design

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

## The Solution: Wrapped Python with Modern Design

Our `pyp` implementation demonstrates a **wrapped Python approach** that combines modern software design principles with practical benefits.

### Usage

```bash
# ✅ Simple and consistent
./pyp script.py           # Run a Python script
./pyp -m module_name      # Run a module
./pyp                     # Interactive shell
./pyp -c "print('hello')" # Execute code
```

### What Makes It Modern?

#### 1. **Abstraction & Encapsulation**

> "A good interface hides complexity behind simplicity."

```python
# Users don't need to know:
# - Where the venv is located
# - How to detect git worktrees
# - How to setup ROS2 environment
# - How to handle different Python versions

./pyp script.py  # Just works™
```

**Benefits**:
- Cognitive load reduction for developers
- Single point of maintenance for environment logic
- Easy to update without changing user code

#### 2. **Git-Aware Architecture**

The wrapper intelligently handles different repository configurations:

```python
# ✅ Normal repositories
$ cd /path/to/repo
$ ./pyp script.py
# → Detects .git directory

# ✅ Git worktrees
$ cd /path/to/worktree
$ ./pyp script.py
# → Resolves commondir to find original repo

# ✅ Custom paths
$ AP_PNC_DIR=/custom/path ./pyp script.py
# → Uses specified directory
```

**Implementation Highlight**:

```python
def find_git_root() -> Path:
    """Find the git repository root, handling both normal repos and worktrees."""
    # Check for AP_PNC_DIR override
    ap_pnc_dir = os.environ.get("AP_PNC_DIR", "")
    if ap_pnc_dir:
        return Path(ap_pnc_dir).resolve()

    # Use git rev-parse to find git dir
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--git-dir"],
            cwd=cwd,
            capture_output=True,
            text=True,
            check=False
        )
        if result.returncode == 0:
            git_file = Path(result.stdout.strip())

            # Check if we're in a worktree
            if "/worktrees/" in str(git_file):
                # Read commondir to find original repo
                commondir_file = git_file / "commondir"
                if commondir_file.exists():
                    commondir = commondir_file.read_text().strip()
                    return (git_file / commondir).parent
            else:
                # Normal repo
                return git_file.parent
    except (subprocess.SubprocessError, FileNotFoundError):
        pass

    # Fallback to script directory
    return Path(__file__).parent.resolve()
```

#### 3. **Integration with Modern Tooling**

**Justfile Integration**:

```justfile
# No venv activation needed!
run-simulation:
    #!/usr/bin/env bash
    ./pyp scripts/simulate.py

test:
    #!/usr/bin/env bash
    ./pyp -m pytest tests/

type-check:
    #!/usr/bin/env bash
    ./pyp -m mypy src/
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
      - uses: actions/checkout@v3
      - name: Install dependencies
        run: just install-all
      - name: Run tests
        run: ./pyp -m pytest
      - name: Type checking
        run: ./pyp -m mypy src/
```

**Pre-commit Hooks**:

```yaml
# .pre-commit-config.yaml
repos:
  - repo: local
    hooks:
      - id: type-check
        name: Type check with mypy
        entry: ./pyp -m mypy src/
        language: system

      - id: run-tests
        name: Run pytest
        entry: ./pyp -m pytest
        language: system
```

#### 4. **Automatic Environment Management**

The wrapper handles multiple environment aspects automatically:

```python
def find_python_interpreter(git_root: Path) -> Path | None:
    """Find the Python interpreter in the virtual environment."""
    venv_names = [".acados_venv", ".venv", "venv", "env"]

    for venv_name in venv_names:
        python_bin = git_root / venv_name / "bin" / "python"
        if python_bin.exists():
            return python_bin

    return None

def setup_ros2_environment(env: dict) -> None:
    """Source ROS2 setup.bash and update environment variables."""
    ros_paths = [
        "/opt/ros/humble",
        "/opt/ros/iron",
        "/opt/ros/jazzy",
    ]

    for ros_path in ros_paths:
        setup_file = Path(ros_path) / "setup.bash"
        if setup_file.exists():
            # Source and capture environment
            result = subprocess.run(
                ["bash", "-c", f"source {setup_file} && env"],
                capture_output=True,
                text=True,
                check=False
            )
            if result.returncode == 0:
                # Parse and update environment
                for line in result.stdout.splitlines():
                    if "=" in line:
                        key, value = line.split("=", 1)
                        env[key] = value
                return
```

**What happens automatically**:
- ✅ Detects virtual environments (`.acados_venv`, `.venv`, `venv`, `env`)
- ✅ Sources ROS2 setup.bash when available
- ✅ Inherits parent environment variables
- ✅ Provides colored, informative status messages

#### 5. **Type Safety & Modern Python**

```python
from pathlib import Path

def find_python_interpreter(git_root: Path) -> Path | None:
    """
    Find the Python interpreter in the virtual environment.

    Args:
        git_root: The git repository root directory

    Returns:
        Path to the Python interpreter, or None if not found
    """
    venv_names = [".acados_venv", ".venv", "venv", "env"]

    for venv_name in venv_names:
        python_bin = git_root / venv_name / "bin" / "python"
        if python_bin.exists():
            return python_bin

    return None

def main() -> int:
    """Main entry point."""
    git_root = find_git_root()
    python_bin = find_python_interpreter(git_root)

    if python_bin is None:
        color_print(f"Error: Python interpreter not found!", Colors.RED + Colors.BOLD)
        print()
        color_print(f"Git Root detected: {git_root}", Colors.YELLOW)
        color_print(f"Searched for venv at: {git_root}/{{.acados_venv,.venv,venv,env}}/bin/python", Colors.YELLOW)
        return 1

    # ... rest of implementation
```

**Modern Python Features**:
- ✅ **Type hints** (`Path | None`, `-> int`)
- ✅ **Type checking** (compatible with mypy/pyright)
- ✅ **Docstrings** with Google style
- ✅ **Error handling** with helpful user feedback
- ✅ **Pathlib** instead of string manipulation

#### 6. Developer Experience Comparison

| Traditional Approach | Wrapped Python (`pyp`) |
|---------------------|------------------------|
| `source .acados_venv/bin/activate` | `./pyp` |
| `which python? which pip?` | Automatic detection |
| Breaks in worktrees | Worktree-aware |
| Manual ROS2 setup | Automatic sourcing |
| Unclear errors | Colored, helpful messages |
| Different commands per environment | One command everywhere |

**Real output**:

```
$ ./pyp script.py

AP-PnC Environment Loader
  Git Root: /home/user/AP-PnC
  Using: .acados_venv ✓

[Script output here...]
```

**Error handling**:

```
$ ./pyp script.py
Error: Python interpreter not found!

Git Root detected: /home/user/AP-PnC
Searched for venv at: /home/user/AP-PnC/{.acados_venv,.venv,venv,env}/bin/python

To fix this, run one of the following:
  just install-venv    # Create virtual environment
  just install-all     # Full installation with acados

Or set AP_PNC_DIR override:
  AP_PNC_DIR=/custom/path ./pyp
```

#### 7. Scalability & Maintainability

**Modular Design** - Each function has a single responsibility:

```python
# Git logic
find_git_root() -> Path

# Venv detection
find_python_interpreter(git_root: Path) -> Path | None

# ROS2 integration
setup_ros2_environment(env: dict) -> None

# Display
color_print(text: str, color: str) -> None

# Main orchestration
main() -> int
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
- ⚡ Faster onboarding (run `./pyp` instead of reading docs)
- 📚 Self-documenting (helpful error messages guide you)
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
   - Simple, focused interface (`./pyp [args]`)
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
$ time ./pyp -c "print('hello')"
hello
./pyp -c "print('hello')"  0.01s user 0.02s system 12% cpu 0.250 total
```

The overhead is negligible (~250ms) for the benefits gained.

---

## Migration Guide

### From Traditional to Wrapped Python

**Before**:
```bash
source .acados_venv/bin/activate
python scripts/simulate.py
pip install -r requirements.txt
```

**After**:
```bash
./pyp scripts/simulate.py
./pyp -m pip install -r requirements.txt
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
# No path manipulation needed - ./pyp handles it
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
- name: Install and test
  run: |
    ./pyp -m pip install -r requirements.txt
    ./pyp -m pytest
```

---

## Best Practices

### ✅ DO

- Use `./pyp` for all Python execution in justfile
- Use `./pyp -m module` for running Python modules
- Check for venv existence in scripts
- Provide helpful error messages
- Use type hints in wrapper code

### ❌ DON'T

- Hardcode venv paths in scripts
- Assume venv is activated
- Use `python` directly in justfile recipes
- Skip error handling
- Ignore git worktree scenarios

---

## Future Enhancements

We're planning to add:

- [ ] `--version` flag to show Python and environment versions
- [ ] `--which` flag to show interpreter path
- [ ] `--list` flag to show all available Python environments
- [ ] Auto-activation of venv in interactive shells
- [ ] Integration with pyenv for Python version management
- [ ] Support for conda environments
- [ ] Automatic dependency checking
- [ ] Performance profiling mode

---

## Conclusion

The wrapped Python approach embodied by `pyp` represents **modern software design** at its best:

- ✅ **Simple interface** for complex operations
- ✅ **Works everywhere** (repos, worktrees, CI/CD)
- ✅ **Self-documenting** with clear error messages
- ✅ **Maintainable** with modular, typed code
- ✅ **Extensible** for future requirements
- ✅ **Developer-focused** respecting time and mental energy

This isn't just about convenience—it's about **building software that empowers developers** to be productive and happy while providing robust, reliable behavior across all development environments.

### The Bottom Line

> **"Good software design is not about following rules—it's about creating tools that make people's lives better."**

The `pyp` wrapper is a small tool that makes a big difference. It's a perfect example of how thoughtful design, attention to detail, and respect for developers' time can transform a mundane task (environment management) into a pleasant experience.

Let's continue leveraging this pattern and expand it to other tools in our ecosystem.

---

## Resources

- **Implementation**: [`pyp`](https://github.com/WarriorHanamy/AP-PnC/blob/main/pyp) - Full source code
- **Documentation**: [README.md - Python Wrapper](https://github.com/WarriorHanamy/AP-PnC#python-wrapper-pyp)
- **Justfile**: [justfile](https://github.com/WarriorHanamy/AP-PnC/blob/main/justfile) - See integration examples

---

*Last updated: January 2026*
*Contributors: AP-PnC Team*
