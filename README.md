# Installations
## Python Wrapper (`pyp`)

The `pyp` script provides a native Python wrapper that automatically locates and executes the project's Python interpreter from `.acados_venv`. It works both in the main repository and in git worktrees.

### Usage

```shell
# Run Python interactively
./pyp

# Run a Python script
./pyp script.py

# Run a module
./pyp -m module_name

# Pass arguments to Python
./pyp -c "print('hello')"
```

### How It Works

1. **Git-aware**: Automatically detects the git repository root, handling both:
   - Normal repositories (where `.git` is a directory)
   - Git worktrees (where `.git` is a file pointing to the worktree metadata)

2. **Virtual environment detection**: Searches for Python in this order:
   - `.acados_venv/bin/python` (primary)
   - `.venv/bin/python`
   - `venv/bin/python`
   - `env/bin/python`

3. **Environment override**: You can override the git root detection:
   ```shell
   AP_PNC_DIR=/custom/path ./pyp
   ```

### When to Use `pyp`

Use `pyp` when you need to run Python commands outside of the `justfile` recipes:

- In CI/CD pipelines
- From other scripts
- When running Python interactively from the terminal