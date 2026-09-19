# Dependencies And Provenance

This package vendors the small set of motor-control dependencies that are required to build and run the chassis package. The old top-level `car2/bxi_motor`, `car2/bxi_pci_drv`, `car2/ISWV`, and `car2/libcanfd` directories are no longer build inputs.

## Current Layout

| Dependency | New location | Purpose |
| --- | --- | --- |
| BXI MIT motor protocol | `src/motor/bxi_motor.cpp`, `include/chassis/motor/bxi_motor.h` | Packing BXI commands, decoding MIT feedback, and the `bxi::Motor` single-motor interface. |
| BXI protocol tests | `tests/test_bxi_motor.cpp`, `tests/test_bxi_motor_commands.cpp` | Offline protocol regression tests moved from the old standalone BXI wrapper. |
| BXI offline example | `examples/bxi_motor_offline.cpp` | FakeTransport example for BXI command encoding without real hardware. |
| BXI PCI vendor library | `vendor/bxi_pci/include/bxi_pci_drv.h`, `vendor/bxi_pci/lib/libbxi_pci_drv.a` | Required binary interface for the PCI CAN board and shared motor power control. |
| ISWV CANopen core | `src/motor/canopen/` | The eight core CANopen / CiA402 implementation files used by ISWV drive and steering control. |
| ISWV public headers | `include/iswv/` | The original public include tree used by `iswv::canopen`, `iswv::Axis`, `FakeTransport`, and related APIs. |
| ISWV tests | `tests/canopen/` | Six original test/support files covering 24 CANopen, CiA402, and ISWV regression cases. |
| ISWV manuals | `docs/iswv/protocol.md`, `docs/iswv/user_manual.pdf` | Protocol notes and the vendor/user manual kept for maintenance reference. |

`libcanfd` is not kept as a source or link dependency. The chassis package links the retained BXI PCI static library directly.

## Build Contract

`chassis` now builds CANopen directly from the files under `src/motor/canopen/`. There is no `ISWV_SOURCE_DIR` option and no external standalone ISWV source tree in the normal build path.

The installed and exported API still keeps the `iswv::canopen` target because public chassis headers expose ISWV types. External users can continue to link the exported target, but they should treat it as part of the `chassis` package rather than a separate repository checkout.

BXI protocol users should include `chassis/motor/bxi_motor.h` and link `chassis::chassis_bxi_motor`. The removed standalone `car2/bxi_motor` wrapper is replaced by the in-package target and tests.

## Source Notes

The BXI motor protocol implementation is derived from `bxirobotics/bxi_car` commit `a02e41f59373b36e38686ed8be83f3421f1959a4`. The retained code keeps the `bxi` namespace, the protocol behavior, and source notes. The old standalone BXI license matched the existing chassis package license; the retained Apache-2.0 license text is in [LICENSE](../LICENSE).

The ISWV CANopen code was copied from the local ISWV work tree based on commit `47b99e40` plus local fixes, including status/fault text corrections, steering layout updates, and current regression tests. It is intentionally copied from the working tree rather than reset to a clean upstream commit.

The BXI PCI library is vendor-supplied binary material. The repository retains the header and static archive needed by the chassis package, but it does not contain rebuildable source for that archive. Do not describe the whole chassis dependency set as Apache licensed: the BXI-derived protocol code has its retained Apache-2.0 notice, the ISWV package metadata still had license TODO status, and the PCI archive is a vendor binary dependency.

## Removed Material

The cleanup removes old standalone build files, examples, duplicated wrappers, unused transport adapters, generated caches, and external-directory configuration paths from the four former dependency directories. Required manuals and source notes are moved into `docs/`; required code is under `src/`, `include/`, `tests/`, `examples/`, and `vendor/`.

Do not reintroduce duplicate dependency trees unless the build contract changes. New code should use the in-package headers and exported CMake targets.
