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

## Feature Planning (claude_plans/)

### Store Feature Development Plans
When planning new features or significant changes:
1. **Create plan files** in `$AP_PNC_DIR/feature/lu_mpc/claude_plans/` directory
2. **Use descriptive names**: `[feature_name].md` (e.g., `ros2_debug_viz.md`)
3. **Include in plan**:
   - Overview and requirements
   - Architecture design with loose coupling principles
   - Directory structure for new modules
   - Implementation steps
   - Critical files to create/modify
   - Verification/testing steps

### Plan File Template
```markdown
# [Feature Name]

## Overview
[Brief description of what this feature does]

## Architecture: Loose Coupling Principles
[Diagram or description of layered architecture]

## Directory Structure
```
feature/lu_mpc/
└── [new modules]
```

## Implementation Plan
### Step 1: [Layer/Module Name]
**File**: `path/to/file.py`
[Description and code snippets]

## Critical Files to Create/Modify
1. `path/to/file1.py` - Description
2. `path/to/file2.py` - Description

## Verification
[Steps to verify the implementation works]
```

### Reference Existing Plans
Before starting implementation, check existing plans in `claude_plans/` for:
- Similar patterns and approaches
- Existing infrastructure to reuse
- Consistent architectural decisions
