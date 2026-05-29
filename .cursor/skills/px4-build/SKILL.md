---
name: px4-build
description: Builds and verifies this PX4 workspace. Use whenever code needs to be built, compiled, or validated by a build, especially when the user mentions build, compile, make, 构建, 编译, 固件, 仿真, SITL, hcl_hcl-v1, or px4_sitl.
---

# PX4 Build

## Workflow

When a task requires building or compile validation for this repository:

1. Run all build commands from the workspace root:
   `/home/hitman/PX4/PX4_THRUST_VECTOR_NEW`
2. Build the firmware target:
   `make hcl_hcl-v1`
3. Build the PC-side simulation firmware:
   `make px4_sitl`
4. Treat the build as passed only when both commands exit successfully.

## Reporting

- If either command fails, report that the build did not pass, identify the failing command, and include the most relevant error lines.
- When a command fails, analyze the output as much as possible before responding. Explain the likely cause, point to the affected file or module when visible, and suggest the next concrete fix or investigation step.
- Do not claim build verification passed unless both `make hcl_hcl-v1` and `make px4_sitl` succeeded in the workspace root.
