# Arm kinematics provenance

Copied from the sibling `new_car/arm` C++17 module used by
`new_car/test/arm_sim.py`. The implementation and tests are formatted to this
package's style; the algorithm is unchanged. Keeping the core here makes car2
deployable on its own. No additional third-party runtime dependency is introduced.

The calibrated joint zero uses geometric offsets [+150, -150, 0] degrees.
Offsets belong to the kinematic model, not motor position commands.

| Original file | SHA-256 before formatting |
| --- | --- |
| `include/arm/kinematics.h` | `e03618e51a0ba3b69fee4775dfaa674d2f5857147334ccb759994f5e811621ba` |
| `src/kinematics.cpp` | `5af0e6b5401543aca6169055421abdec37166e4b400ef1bd2cfee9a29b5dfbf0` |
| `tests/test_kinematics.cpp` | `380d67c8ae8d386ffba2ff4a7587497b51fdc393ec29c7ccf61f94516dd6e4db` |
| `config/arm.json` | `b6ff39853cca8557d43ae9da1984b8c2d0d7192601fc76fd559950d8ac855e9f` |
