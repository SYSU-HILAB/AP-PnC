# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Vibe Coding Rules & Project Structure
You are developing a feature for the AP-PnC project.
This feature lies in $AP_PNC_DIR/feature/lu_mpc/

Always run everything using
$AP_PNC_DIR/.acados_venv/bin/python

Always add justfile entry for user usage, remember to use
$AP_PNC_DIR/.acados_venv/bin/python when adding these entries.

When adding justfile entry, you have fully tested the functionality in case of any errors or issues.

## Configuration Design Principle

### Use Structured Configuration Classes
When creating configurable functions or classes, follow this pattern:

1. **Create a `@dataclass` for configuration parameters** - Never use scattered constants or `**kwargs`
2. **Provide a `DEFAULT_CONFIG` instance** for common use cases at the file in the func implementation file.
3. **Accept only the config object in function signatures** - Keep APIs clean and consistent
4. **Allow partial configuration** through dataclass defaults



## Scope
We only focus on the $AP_PNC_DIR/feature/lu_mpc/ folder, everything else is not considered.
