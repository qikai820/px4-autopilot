# MC-derived vector flight control

Current Voliro T designs (2026-11-03):

- [Control law and algorithm design](VoliroT_algorithm_design.md)
- [Software design and interface contracts](VoliroT_software_design.md)
- [Implementation and validation history](VoliroT_large_angle_control.md)
- [Voliro T tilt monitoring and multicopter fallback](VoliroT_tilt_monitor.md)

The Voliro new mode now supports an independent pitch target of +/-160 degrees,
task-frame attitude feedback, angular-acceleration rate control, and constrained
allocation with pitch recovery priority at a tilt stop. `VT_CTRL_MODE=0` remains
the default legacy branch; use the design documents for current mode-specific
behavior, parameter matching, and evidence limits. UDP derived attitude fields
for airframe 17 now use bank/independent-pitch/force-plane-heading in radians;
standard uORB quaternions and MAVLink attitude semantics remain unchanged.

The complete position, attitude and rate modules were copied from this project's
MC implementation at `ca010df2d9396582e641aefa7f034649e4b1f759`.
The original MC module source and controller libraries are unchanged.

| Original | Independent copy |
| --- | --- |
| `mc_pos_control` including PositionControl, GotoControl, Takeoff | `vector_pos_control`, `VectorPosition`, `VectorGoto`, `VectorTakeoff` |
| `mc_att_control` including AttitudeControl | `vector_att_control`, `VectorAttitude` |
| `mc_rate_control` including RateControl | `vector_rate_control`, `VectorRateCore` |

Commander, Navigator and FlightModeManager remain the task owners. The copied
position wrapper consumes their `trajectory_setpoint`, `vehicle_constraints`,
`vehicle_control_mode` and estimator state. It retains takeoff ramping, estimator
reset compensation, last-valid-setpoint fallback, failsafe generation and the
project's existing optional Dock hooks. Standard manual stick mapping is retained.
The supported task inputs therefore include position/altitude, mission, takeoff,
land, RTL, hold, Orbit/Follow (when compiled), and Offboard. Stabilized and Acro
retain the copied attitude/rate module entry paths. Actual flight-mode and mission
validation on both vector aircraft remains a HIL acceptance step.

## Aircraft selection

* Ordinary MC airframes (`VEHICLE_TYPE=mc`) start the original `mc_*` modules.
* MorphQuad airframes 19001/19002 start `vector_*`, `CA_AIRFRAME=16`.
* Voliro T airframe 19003 starts `vector_*`, `CA_AIRFRAME=17`.

The default CUAV 7 Nano and Voliro T configurations both compile the complete MC
and vector chains; selecting an airframe takes effect after reboot. The Voliro T
configuration only chooses 19003 as the default for a fresh parameter store.
An existing saved `SYS_AUTOSTART` remains authoritative. The vector startup
rejects an already-running MC controller. Start only one chain; legacy `geo_*`
controllers use the former physical torque interface and must not be started with
the new normalized vector allocation interface.

## Force, attitude and torque

Position and velocity feedback remain the copied MC P/PID algorithms. Only the
force/attitude mapping changes for the two vector geometries. World axes are NED;
body axes are FRD.

* MorphQuad converts requested acceleration to normalized world force, retaining
  independent level/yaw attitude in task-controlled modes. `VEC_AUX_ATT=1` enables
  optional independent roll/pitch on AUX1/AUX2 in manual position/altitude modes.
  Its default is zero and it has no effect in autonomous/Offboard modes.
* Voliro T converts requested lateral force to roll; forward force is
  assigned to the common-tilt mechanism while AUX1 can independently command
  body pitch. Its allocator handles four front motors,
  one reversible tail motor and one common tilt servo. It cannot independently
  produce body-Y force; that residual participates in position anti-windup.
  Its target rotation is `Rz(yaw) * Rx(bank) * Ry(independent_pitch)`.
  Bank comes from lateral/world-vertical force, before independent pitch, so
  a forward-force correction cannot cause a near-90-degree bank jump when
  projected body-Z force crosses zero. `MPC_TILTMAX_AIR` limits bank separately
  from `VT_PITCH_MAX` (up to 160 degrees). At vertical pitch the reference yaw
  describes the force-plane orientation; Euler heading/roll are singular and
  must not be used to judge quaternion continuity. The common-tilt allocator
  already transforms roll/yaw moments into arm-force difference and rotor
  reaction torque. Near 90-degree servo tilt, body-roll authority is limited
  by reaction torque, so simultaneous large moments may remain infeasible.
* `RC_MAP_VECTOR_SW` and `RC_MAP_LEVEL_SW` assign calibrated RC channels (1–18;
  0 leaves each switch unassigned). On Voliro T, a mapped vector switch above
  center selects vector allocation and AUX1 independent pitch when
  `VEC_AUX_ATT=1` and `RC_MAP_AUX1` is mapped. At or below center it selects
  ordinary multicopter position/altitude control: vector allocation first returns
  independent pitch to level at the smaller of `VT_PITCH_RATE` and `CA_CT_RATE`.
  Only after measured/commanded pitch and rotor tilt are within 5 degrees and
  body rates below 10 deg/s for 0.2 s does fixed-tilt allocation center/hold the
  servo. Body attitude then produces horizontal motion.
  Position/velocity feedback and trajectory generation remain active;
  `MPC_TILTMAX_AIR` and the normal position thrust limits apply. This ordinary
  multicopter mode does not require `VEC_AUX_ATT`. The level switch commands
  zero independent pitch in vector mode, without changing the selected mode or
  re-enabling the servo in ordinary multicopter mode. With the vector switch
  unassigned, `VEC_AUX_ATT` and AUX1 retain independent pitch. Invalid/stale RC,
  failsafe or leaving position control releases the manual mode. A detected
  tilt fault still takes precedence and remains latched until disarm. For
  MorphQuad the original independent attitude-switch behavior remains.
  See [fixed-servo operation and validation](VoliroT_fixed_tilt.md).
* The attitude copy uses full shortest-path quaternion feedback and publishes the
  actual body-rate target in `vehicle_rates_setpoint`. Yaw feedforward retains the
  original world-Z convention. Acro/body-rate inputs bypass the attitude loop.
* In legacy mode, the rate copy retains PID, measured-acceleration D, yaw filtering, integral
  handling and allocation saturation feedback. `VEC_GYRO_FF` optionally adds
  `omega cross (J omega)` in consistent units; it defaults to zero until inertia
  and torque scales are calibrated.
  In Voliro dynamics mode (`VT_CTRL_MODE=1`), feedback and integral are angular
  acceleration, `J` and `omega cross (J omega)` produce physical torque, and the
  low-pass filter acts on the rotor-reaction mode in allocation. See the designs
  above for reference feedforward, freshness, and anti-windup details.

`vector_force_setpoint.xyz` is normalized world force. The rate controller rotates
it using the latest attitude at IMU update rate. It only uses this force while
position/altitude control is enabled and the reference is fresh after activation.
Attitude and rate Offboard inputs continue to use their body-frame thrust.

Both standard wrench topics retain normalized semantics. Each vector allocator
converts force using its total installed maximum thrust and torque using
`VEC_TQ_R/P/Y` (N m per normalized unit). Allocation residuals are converted back
to the same normalized units. Legacy rate gains and torque scales must be tuned
as a pair. New-mode `VT_RATE_*` gains have angular-acceleration semantics and use
calibrated inertia before torque normalization. Old physical `GAC_*` gains cannot
be copied numerically into `VC_*` or `VT_RATE_*` gains.

Control parameter groups:

* `VPC_*`: independent copied position and velocity feedback gains.
* `VC_*`: independent copied attitude/rate gains, rate limits and Acro settings.
  New mode replaces attitude/rate feedback gains with `VT_*`, while continuing
  to use applicable `VC_*` rate limits and manual/Acro settings.
* `VT_*`: Voliro dynamics mode, task gains, acceleration control, reference limits
  and actuator prediction settings. Mode changes require reboot.
* `MPC_*`: shared task constraints, throttle calibration, takeoff/land limits.
* `VEC_*`: vector geometry options, inertia, normalized torque reference scales.
* `CA_MQ_*`: MorphQuad geometry/actuators.
* `CA_CT_*`: Voliro T geometry/actuators. The existing parameter keys and airframe
  ID 17 are retained to preserve saved geometry; displayed aircraft/source names
  are Voliro T.

Voliro T's 2026-10-30 CG correction uses front/tail arms of 0.14/0.64 m.
M5 now has command slew, start/reverse hysteresis, sustained-reversal confirmation
and an exact-zero dwell. Allocation residuals include final protected commands.
See [Voliro T tail protection](VoliroTTailProtection.md) for parameters, existing
parameter-store migration and validation limits.

At zero requested force, Voliro T centers its common-tilt servo at `CA_CT_RATE`
even if manual attitude feedback requests pitch torque. This avoids a negative
pitch correction turning `atan2(0, negative)` into a tilt-limit command immediately
after arming at minimum throttle. A nonzero force request still uses normal
allocation. Neutral is `actuator_servos.control[0] = 0`, which is a servo-center
PWM command, not an absence of PWM pulses.

`CA_CT_ANGLE` is the installed mechanism's one-sided maximum angle after
gearing, not the servo shaft angle or total end-to-end travel. The allocator
limits the physical angle and converts it to `Servo1 = angle / CA_CT_ANGLE`.
Use the assigned output channel's existing MIN/CENT/MAX/REV parameters for PWM
calibration; the channel interpolates linearly on each side of zero. The 19003
default uses MAIN7 (`PWM_MAIN_MIN7`, `PWM_MAIN_CENT7`, `PWM_MAIN_MAX7`, with
direction in the `PWM_MAIN_REV` bitmask). No additional gear-ratio parameter is
needed when the mechanism angle is measured directly. See
[angle/PWM calibration](VoliroT_large_angle_control.md#机构角度与-pwm-标定).

Set `CA_CT_RATE` (deg/s, initial default 200) to the measured speed of the installed
tilt mechanism under load. It limits the common-tilt command in both control
modes. Independent body pitch and return to level use
`min(VT_PITCH_RATE, CA_CT_RATE)`; dynamics mode additionally applies reference
acceleration and tracking-authority limits. `vector_pos_control status` reports
the servo speed limit and effective independent pitch rate. Bank stabilization
keeps its existing limits. See [tilt speed calibration](VoliroT_large_angle_control.md#倾转舵机速度标定).

Existing saved `GC_*`, `GPC_*`, `GAC_*` parameters do not tune the new controllers.
Airframe defaults use the new namespace. Selecting a different airframe does not
make existing saved tuning from another aircraft appropriate for it.

## Verification

Build both CUAV variants:

```sh
make cuav_7-nano_default
make cuav_7-nano_volirot
```

A POSIX integration build is also available without requiring the optional DDS
vendor submodule:

```sh
CCACHE_DIR=/tmp/px4-vector-ccache make px4_sitl_vector
python3 src/modules/vector_pos_control/tests/runtime_smoke.py build/px4_sitl_vector
```

The smoke check starts/status-checks/shuts down each of the three controller
chains in isolated instances, without sensors, MAVLink or actuator output drivers.

Run real controller-core regressions using generated message headers:

```sh
python3 src/modules/vector_pos_control/tests/run_tests.py build/cuav_7-nano_default
python3 src/modules/control_allocator/VehicleActuatorEffectiveness/test_voliro_t.py build/cuav_7-nano_default
```

The regressions compare the original and copied position, attitude and rate cores
before geometry adaptation; check quaternion antipodes/large angles and force
frame invariance; close the adapted attitude/rate loops through the actual Voliro T
allocator and ideal rigid-body dynamics; and track a waypoint sequence with both
position mappings in an ideal point-mass model. These are host simulations, not
HIL or flight validation. They do not model measured ESC curves, sensor latency,
gimbal dynamics or structural vibration.

The vertical-pitch regressions sweep quaternion targets through -120 to +120
degrees, reproduce a logged force-projection zero crossing, and recover small
attitude/rate disturbances at +/-80, +/-90 and +/-100 degrees using the actual
allocator, finite tilt slew and allocation feedback in ideal rigid-body dynamics.
They also ramp from level to each +/-90-degree target at 15 deg/s, dwell for
four seconds, return to level and settle, checking quaternion tracking error.
These tests establish mathematical continuity and ideal small-disturbance
recovery, not stability of the deployed JSBSim/ESC/servo system. Match the
deployed model geometry, forward/reverse thrust curves and actuator latency
before validating large-angle transitions in HIL.

Before flight acceptance, validate both vector models with their actual JSBSim
actuator curves, first hover then a QGC mission with pause/resume, manual takeover,
RTL and landing. The previously observed Voliro T simulator/allocator thrust-curve
mismatch remains a separate calibration item. This implementation does not modify
the external JSBSim repository or install firmware on the aircraft.

## Local verification on 2026-10-29

* `cuav_7-nano_default`: complete firmware build passed; Flash 1,743,024 B / 1,920 KiB (88.65%).
* `cuav_7-nano_volirot`: complete firmware build passed; Flash 1,732,524 B / 1,920 KiB (88.12%).
* `px4_sitl_vector`: POSIX build passed. Isolated startup/status checks passed for original MC, MorphQuad and Voliro T.
* Host numerical equivalence, full quaternion, force-frame transport, allocation anti-windup, Voliro T ideal rigid-body loop and ideal point-mass waypoint tests passed.
* Voliro T feasible-wrench tests from -100 to +100 degrees, side-force residual and actuator/tilt/slew limit tests passed.
* All vector airframe default parameter names exist in generated firmware metadata.
* Original MC source snapshot comparison passed (55 files).
* Hardware flashing, real HIL mission execution and flight tuning were not performed.
