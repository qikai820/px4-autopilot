# MC-derived vector flight control

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
* Voliro T converts requested lateral force to roll, while forward force is
  assigned to the common-tilt mechanism. Its allocator handles four front motors,
  one reversible tail motor and one common tilt servo. It cannot independently
  produce body-Y force; that residual participates in position anti-windup.
* The attitude copy uses full shortest-path quaternion feedback and publishes the
  actual body-rate target in `vehicle_rates_setpoint`. Yaw feedforward retains the
  original world-Z convention. Acro/body-rate inputs bypass the attitude loop.
* The rate copy retains PID, measured-acceleration D, yaw filtering, integral
  handling and allocation saturation feedback. `VEC_GYRO_FF` optionally adds
  `omega cross (J omega)` in consistent units; it defaults to zero until inertia
  and torque scales are calibrated.

`vector_force_setpoint.xyz` is normalized world force. The rate controller rotates
it using the latest attitude at IMU update rate. It only uses this force while
position/altitude control is enabled and the reference is fresh after activation.
Attitude and rate Offboard inputs continue to use their body-frame thrust.

Both standard wrench topics retain normalized semantics. Each vector allocator
converts force using its total installed maximum thrust and torque using
`VEC_TQ_R/P/Y` (N m per normalized unit). Allocation residuals are converted back
to the same normalized units. Rate gains and torque scales must be tuned as a
pair. Old physical `GAC_*` gains cannot be copied numerically into `VC_*` gains.

Control parameter groups:

* `VPC_*`: independent copied position and velocity feedback gains.
* `VC_*`: independent copied attitude/rate gains, rate limits and Acro settings.
* `MPC_*`: shared task constraints, throttle calibration, takeoff/land limits.
* `VEC_*`: vector geometry options, inertia, normalized torque reference scales.
* `CA_MQ_*`: MorphQuad geometry/actuators.
* `CA_CT_*`: Voliro T geometry/actuators. The existing parameter keys and airframe
  ID 17 are retained to preserve saved geometry; displayed aircraft/source names
  are Voliro T.

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
