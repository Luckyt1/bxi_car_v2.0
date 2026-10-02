# Dependencies And Provenance

This package vendors the small set of motor-control dependencies that are required to build and run the chassis package. The old top-level `car2/bxi_motor`, `car2/bxi_pci_drv`, `car2/ISWV`, and `car2/libcanfd` directories are no longer build inputs.

## Current Layout

| Dependency | Location | Purpose |
| --- | --- | --- |
| BXI MIT motor | `src/thirdparty/motor/bxi/communication.{h,cpp}`, `motor.{h,cpp}` | Frame encoding/decoding and single-motor calls. |
| BXI board | `src/thirdparty/bxi/board.{h,cpp}`, `src/thirdparty/bxi/vendor/` | PCI CAN access and shared power, using the supplied static archive. |
| BXI API types | `src/thirdparty/bxi/result.h`, `can_frame.h`, `transport.h` | Independent BXI results, frames and transport interface, without ISWV includes or aliases. |
| ISWV BXI adapter | `src/thirdparty/motor/iswv/bxi_transport.{h,cpp}` | Converts between the separate BXI and ISWV transport APIs when ISWV uses a BXI board. |
| ISWV CANopen | `src/thirdparty/motor/iswv/communication.{h,cpp}`, `protocol/` | Master/node transactions and existing PDO/CiA402/Axis internals. |
| ISWV motor | `src/thirdparty/motor/iswv/motor.{h,cpp}` | Travel RPM API; Axis/Drive also expose supported position/mode/parameter operations. |
| Yiyou EtherCAT | `src/thirdparty/motor/yiyou/communication.{h,cpp}`, `vendor/soem/` | SOEM master and CoE/PDO communications. |
| Yiyou motor | `src/thirdparty/motor/yiyou/motor.{h,cpp}` | PV/PP/CST motor commands and handshake. |
| Offline tests | `test/unit/` | Refactor regression, mock motor transactions and debug-menu routing. |
| Debug tools | `test/debug.py`, `test/tools/` | Numeric dispatcher and internal diagnostic backends. |
| ISWV manuals | `docs/iswv/protocol.md`, `docs/iswv/user_manual.pdf` | Protocol notes and vendor manual. |

`libcanfd` is not kept as a source or link dependency. The chassis package links the retained BXI PCI static library directly.

## Build Contract

`chassis` now builds CANopen directly from the files under `src/thirdparty/motor/iswv/`. There is no `ISWV_SOURCE_DIR` option and no external standalone ISWV source tree in the normal build path.

The installed and exported API still keeps the `iswv::canopen` target because public chassis headers expose ISWV types. External users can continue to link the exported target, but they should treat it as part of the `chassis` package rather than a separate repository checkout.

BXI protocol users should include `motor/bxi/motor.h` and link `chassis::chassis_bxi_motor`. The removed standalone `car2/bxi_motor` wrapper is replaced by the in-package target and tests.

Neither `chassis_bxi_motor` nor `chassis_bxi_transport` links ISWV. Their public APIs use
BXI-owned types. ISWV applications using the board also link `chassis::chassis_iswv_bxi_transport`
and wrap the board in `iswv::BxiTransport` before constructing the CANopen master.

## Source Notes

The BXI motor protocol implementation is derived from `bxirobotics/bxi_car` commit `a02e41f59373b36e38686ed8be83f3421f1959a4`. The retained code keeps the `bxi` namespace, the protocol behavior, and source notes. The old standalone BXI license matched the existing chassis package license; the retained Apache-2.0 license text is in [LICENSE](../LICENSE).

The ISWV CANopen code was copied from the local ISWV work tree based on commit `47b99e40` plus local fixes, including status/fault text corrections, steering layout updates, and current regression tests. It is intentionally copied from the working tree rather than reset to a clean upstream commit.

The BXI PCI library is vendor-supplied binary material. The repository retains the header and static archive needed by the chassis package, but it does not contain rebuildable source for that archive. Do not describe the whole chassis dependency set as Apache licensed: the BXI-derived protocol code has its retained Apache-2.0 notice, the ISWV package metadata still had license TODO status, and the PCI archive is a vendor binary dependency.

## Removed Material

The cleanup removes old standalone build files, examples, duplicated wrappers, unused transport adapters, generated caches, and external-directory configuration paths from the four former dependency directories. Required manuals and source notes are in `docs/`; maintained code is under `src/thirdparty/`, `src/example/` and `test/`. The original test sources were removed before this refactor; new tests cover the changed boundaries, not the full historical suite.

Do not reintroduce duplicate dependency trees unless the build contract changes. New code should use the in-package headers and exported CMake targets.
