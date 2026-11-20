# Validation {#validate_page}

CMSIS validates with Arm Compiler 6 and GCC and publishes the scope on each component's Validation page. This page is that scope for the Ensemble pack, taken from the repository rather than from a lab log.

## What the repository builds

`.github/workflows/Test-Examples.yml` builds examples with `cbuild --packs` for GCC and AC6 (Arm Compiler 6). It runs on pull requests that change `Boards/`, on pushes to `main`, and every Saturday. `fail-fast` is off, so one broken example does not hide the others.

The workflow checks the compile and link. It does not program a board, so a green run does not by itself prove the LED or the UART output.

| Project | GCC | AC6 | On-target result |
|---|---|---|---|
| AppKit-E7 Blinky, M55_HE | In the CI matrix | In the CI matrix | Not recorded in the repo |
| AppKit-E7 Blinky, M55_HP | In the CI matrix | In the CI matrix | Not recorded in the repo |
| AppKit-E7 DualCore | In the CI matrix | In the CI matrix | Not recorded in the repo |
| DevKit-E7 Blinky_HE | In the CI matrix | In the CI matrix | Not recorded in the repo |
| DevKit-E7 Blinky_HP | In the CI matrix | In the CI matrix | Not recorded in the repo |
| DevKit-E7 Dual_Core | In the CI matrix | In the CI matrix | Not recorded in the repo |
| DevKit-E8 Blinky_HE | In the CI matrix | In the CI matrix | Not recorded in the repo |
| DevKit-E8 Blinky_HP | In the CI matrix | In the CI matrix | Not recorded in the repo |
| DevKit-E8 DualCore | In the CI matrix | In the CI matrix | Not recorded in the repo |

## Compilers the pack claims

| Compiler | Evidence in the pack |
|---|---|
| Arm Compiler 6 | Named in `docs/Overview.md`. The CI tool id is `AC6`. Release 2.3.0 adds `-funwind-tables` so GDB can unwind AC6 frames |
| GCC | Named in `docs/Overview.md` and used as `GCC` in the CI matrix |
| Clang | Release 2.0.4 added Clang support. Clang is not in the current CI matrix |

## Projects the CI does not build

Hello_World on the E8 DevKit, the µVision Blinky and Hello World projects on the E7 DevKit, the E8 AppKit layers, and the E1C kits are not in `Test-Examples.yml`. Their build status is not known from this workflow.

## How to record an on-target result

1. Program the ATOC for the matching core configuration.
2. Flash the CI artifact.
3. Confirm the UART4 string and the LED behavior on @ref examples_page.
4. Replace "Not recorded in the repo" in the table above with the date and the kit revision.
