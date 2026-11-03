# Voliro T: 14 cm CG offset and reversible tail protection

The 2026-10-30 measured CG is 0.14 m aft of the front left/right motor line.
The previously measured front-to-tail distance remains 0.78 m. In body FRD,
front motors are at X=+0.14 m, Y=+/-0.45 m and M5 is at X=-0.64 m.
The PX4 front Z remains 0 m; the model's upper/lower Z offsets remain estimates.

## Configuration

Airframe `19003_voliro_t` sets these initial bench defaults:

| Parameter | Value | Meaning |
| --- | --- | --- |
| `CA_CT_X` | 0.14 m | Front motor line ahead of CG |
| `CA_CT_TAIL` | 0.64 m | Tail motor behind CG |
| `CA_R5_SLEW` | 0.40 s | Minimum full -1 to +1 command traversal time |
| `CA_CT_T5_DZ` | 0.03 | Stop demand at/below this absolute command |
| `CA_CT_T5_HYST` | 0.10 | Minimum absolute command to start/reverse |
| `CA_CT_T5_HOLD` | 0.04 s | Continuous opposite-demand confirmation |
| `CA_CT_T5_WAIT` | 0.08 s | Exact-zero command dwell before reversing |

Deadband/hysteresis thresholds are in motor command space **after** square-root
thrust conversion. A running direction can continue between the low/high
thresholds; a stopped motor needs the high threshold to restart. Opposite demand
first ramps the old output down to zero. Reverse rotation starts only after both
continuous confirmation and the complete zero dwell are satisfied. A cancelled
request clears reversal confirmation. The zero dwell measures command time,
not actual rotor standstill, and needs measurement on the installed motor/ESC.

M5 applies `CA_R5_SLEW` inside its final guard using the original allocation
demand, replacing the generic M5 slew result rather than adding a second slew
stage. Other motor/servo slew limits remain in the common allocator. Achieved
wrench is recomputed from the final bounded motor and servo commands, including
the M5 guard, so rate-controller saturation feedback sees the lost pitch torque.
Disarmed updates reset the guard and request zero immediately. Output-driver
disarm/failsafe handling remains authoritative.

The DShot 3D conversion explicitly sends stop for the neutral mixer value 1000,
including when `DSHOT_3D_DEAD_L=DSHOT_3D_DEAD_H=1000` and `DSHOT_MIN` is nonzero.
The guard does not intercept direct `actuator_test` commands. It is not a current,
temperature or RPM controller.

## Existing aircraft parameter stores

Flashing firmware does not replace saved parameter values. After installing the
matching firmware, apply this block in the PX4 console while disarmed:

```sh
param set CA_CT_X 0.14
param set CA_CT_TAIL 0.64
param set CA_R5_SLEW 0.40
param set CA_CT_T5_DZ 0.03
param set CA_CT_T5_HYST 0.10
param set CA_CT_T5_HOLD 0.04
param set CA_CT_T5_WAIT 0.08
param save
```

The matching local JSBSim model is
`/home/qikai820/src/px4_jsbsim/px4-jsbsim-bridge/models/voliro_t/`.
Its CG stays at the origin and every fixed non-CG location shifts by -0.11 m
in structural X, including landing contacts and reference points. Mass, inertia
estimates and existing thrust curves remain unchanged. HIL simulated and real
outputs receive the same protected PX4 commands. The JSBSim XML retains physical
motor lag and does not duplicate the command guard.

## Thrust margin and validation

For vertical static hover at the model mass of 5.04 kg, required M5 thrust is
`5.04 * 9.80665 * 0.14 / 0.78 = 8.87 N`. This is 88.7% of the provisional
10 N tail maximum, or about 94.2% normalized command under the square law.
Increasing a software maximum does not increase installed thrust. Actual thrust,
continuous thermal capability and stopping time have not been calibrated.
The front-model/PX4 thrust-curve mismatch and estimated reverse thrust remain
separate calibration items.

Host regressions cover 14 cm CG hover equilibrium, feasible wrenches, bounds,
M5 slew, Schmitt hysteresis, short/cancelled reverse demand, exact-zero dwell,
invalid inputs, disarm-style reset, variable scheduler intervals and the actual
DShot 3D neutral conversion. The actual effectiveness implementation is also
compiled on the host to check that generic slew cannot block M5 startup, final
motor/servo commands determine force/torque residuals, and disarm resets output.
Run:

```sh
python3 src/modules/control_allocator/VehicleActuatorEffectiveness/test_voliro_t.py build/cuav_7-nano_volirot
make cuav_7-nano_volirot
```

For the external model's native JSBSim force/moment and geometry checks on this host:

```sh
python3 Tools/tests/voliro_t_model/test_model.py \
  /home/qikai820/src/px4_jsbsim/px4-jsbsim-bridge/models/voliro_t \
  /home/qikai820/src/px4_jsbsim/build/jsbsim_install \
  /home/qikai820/src/px4_jsbsim/px4-jsbsim-bridge/systems
```

These checks and a successful firmware build do not establish reduced hardware
temperature or flight stability. Validate the initial guard settings on a propeller-free
bench, then assess pitch tracking, saturation and thrust margin in the matched model.
