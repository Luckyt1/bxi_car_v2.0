# Dependencies and provenance

This package builds and runs its offline tests from the files inside this directory. It does not require the original development tree or an external ISWV checkout.

| Component | Location | Purpose |
| --- | --- | --- |
| BXI PCI | `src/thirdparty/bxi/` | Board access and shared power; vendor header and static archive are required |
| BXI motor | `src/thirdparty/motor/bxi/` | Motor protocol and API |
| ISWV CANopen | `src/thirdparty/motor/iswv/` | CANopen transactions, PDO/CiA402, steering drives |
| Yiyou EtherCAT | `src/thirdparty/motor/yiyou/` | SOEM communication, motor control and fourth-axis position memory |
| Arm/chassis | `src/thirdparty/arm/`, `src/thirdparty/chassis/` | Kinematics and controllers |
| Commissioning | `tools/arm_yiyou_control.cpp`, `tools/yiyou_control.py` | Runtime jogging and explicit zero-reference setup |
| Torque reports | `tools/arm_torque_report.py` | Runtime log analysis and plots; uses matplotlib |
| Offline tests | `test/unit/`, ISWV `FakeTransport` | Simulated CAN/CoE, calibration, ROS interfaces and deployment regression checks |
| App and OTA | `scripts/`, `src/example/app_interface.h` | Gateway entry point, persistent configuration and install bundles |

## Sources and licenses

The BXI motor protocol implementation is derived from `bxirobotics/bxi_car` commit `a02e41f59373b36e38686ed8be83f3421f1959a4`. Its Apache-2.0 license text is retained in [LICENSE](../LICENSE).

ISWV CANopen code comes from the local ISWV work tree based on commit `47b99e40` with local corrections. Its original package metadata had a TODO license entry; no new license is assigned to it by this export.

`src/thirdparty/bxi/vendor/lib/libbxi_pci_drv.a` is a vendor-supplied binary dependency, not a generated project build artifact. Its header is retained alongside it; rebuildable source for the archive is not included. The project's Apache-2.0 declaration does not establish licensing terms for this vendor binary or the ISWV material.

The included Linux SOEM subset is based on version 1.4.0 and retains its GPLv2 license with linking exception and EtherCAT terms. See [SOEM license](../src/thirdparty/motor/yiyou/vendor/soem/LICENSE), [origin](../src/thirdparty/motor/yiyou/vendor/soem/ORIGIN.md), and [local patches](../src/thirdparty/motor/yiyou/vendor/soem/LOCAL_PATCHES.md).

Arm kinematics origin and original hashes are recorded in [arm_kinematics_origin.md](arm_kinematics_origin.md).

## Build and installed API

The CMake build uses the included sources and vendor archive directly. `libcanfd` and the former standalone dependency trees are not required. The installed package exports the runtime control libraries and `iswv::canopen`; `FakeTransport` supports the offline tests. ROS dependencies are declared in `package.xml`. Runtime configuration and fourth-axis position records remain specific to the deployed machine.
