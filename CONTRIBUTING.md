# Contributing to CANopenSTM32

Thanks for your interest in contributing! This document explains what we check before
accepting a contribution and how to run each check locally before opening a pull request.

For an overview of the repository layout (`CANopenNode`, `CANopenNode_STM32`, `examples`, `legacy`),
see the [README](README.md).

## Prerequisites

Install and make sure the following tools are available in your `PATH`:

- CMake
- Ninja
- ARM GCC (`arm-none-eabi-gcc`)
- STM32CubeProgrammer CLI (for flashing/hardware testing), or other supporting debugger/flasher tool, like Segger J-Link
- `clang-format` for code formatting

You can verify your setup with:

```sh
cmake --version
ninja --version
arm-none-eabi-gcc --version
STM32_Programmer_CLI --version
clang-format --version
```

> The fastest way to get CMake, Ninja and ARM GCC together is the
> [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html) package, as
> described in the README.

## Before opening a pull request

Every PR is expected to satisfy the checklist in the
[PR template](.github/PULL_REQUEST_TEMPLATE.md). The sections below explain how to run each item.

### 1. Validate the implementation for both CAN and FDCAN

`CANopenNode_STM32` auto-detects the controller type at compile time and drives either the
classic `CAN`/`bxCAN` peripheral or `FDCAN`, from the same source files
(`CANopenNode_STM32/CO_driver_STM32.c`, `CO_app_STM32.c`). If your change touches driver or
application code in `CANopenNode_STM32`, verify it builds and behaves correctly on both:

- Classic CAN examples: `examples/stm32f0xx_can`, `examples/stm32f3xx_can`, `examples/stm32f4xx_can`
- FDCAN examples: `examples/stm32g0xx_fdcan`, `examples/stm32g0xx_fdcan_rtos`, `examples/stm32h7xx_fdcan`

Building all of them (step 3 below) is a good first check. If your change is intentionally
specific to only one controller type (e.g. an FDCAN-only errata workaround), say so explicitly
in the PR description, including why the other controller type doesn't need the change.

### 2. New build configuration follows the CMake build system

New examples or boards should follow the pattern of the existing `examples/*/CMakeLists.txt`
projects: a CMake project with a configure preset that `cmake --list-presets` can discover, so
that it's picked up automatically by `scripts/build.py` (see step 3). Reuse existing presets and
toolchain files under [cmake/](cmake/) rather than inventing a parallel setup.

### 3. Code style matches `clang-format`

The repository ships a root [.clang-format](.clang-format) style file. Format the files you
changed before committing:

```sh
# Format specific files in place
clang-format -i path/to/file.c path/to/file.h

# Format everything you've changed relative to master
git diff --name-only master... -- '*.c' '*.h' | xargs -r clang-format -i
```

Re-check `git diff` afterwards, `clang-format` can reformat unrelated lines if a file was never
formatted before — keep unrelated reformatting out of your PR.

> If you develop in VSCode, the C/C++ extension ships clang-format natively. You can configure it to format on save which will ease your activity.

### 4. The build script compiles successfully

`scripts/build.py` configures and builds every CMake preset for every example under `examples/`
(each example directory containing a `CMakeLists.txt`):

```sh
python3 scripts/build.py
```

To iterate faster while working on a single example, scope it with `--path`:

```sh
python3 scripts/build.py --path examples/stm32g0xx_fdcan
```

Add `--clean` to force a clean rebuild of each preset instead of an incremental build. The script
prints a pass/fail summary per project/preset at the end and exits non-zero if anything failed —
make sure you get a clean, all-passing run before opening the PR.

### 5. New features / breaking changes work on real hardware

For anything beyond a documentation or comment change, flash and run your change on real
hardware and confirm normal operation (boot-up message on the CAN bus, status LEDs where
applicable, no HAL errors). See the "Supported boards and MCUs" section in the
[README](README.md#supported-boards-and-mcus) for the boards this project targets, and mention
in your PR which board(s) you tested on.

## Commit messages

Recent history follows a `type: short description` convention, e.g. `fix: ...`, `feat: ...`,
`build: ...`, `docs: ...`. Please follow the same style so the log stays easy to scan.

## Opening the pull request

Push your branch and open a PR against `master`. The PR description will be pre-filled from the
[PR template](.github/PULL_REQUEST_TEMPLATE.md) — fill in the checklist honestly; if an item
doesn't apply (e.g. no hardware available for a doc-only change), say so instead of leaving it
unchecked with no explanation.
