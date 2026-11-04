# UDP attitude display fields

The bridge exports the complete `vehicle_attitude` and
`vehicle_attitude_setpoint` uORB messages, plus derived `roll`, `pitch`, `yaw`
fields. All three derived fields remain in **radians**, with the same JSON keys.

For `CA_AIRFRAME=17` (Voliro T), their definition is:

| Existing suffix | Meaning | Extraction from body-FRD to NED rotation R |
| --- | --- | --- |
| `roll` | Force-plane bank | `asin(R[2][1])` |
| `pitch` | Independent body pitch | `atan2(-R[2][0], R[2][2])` |
| `yaw` | Force-plane heading | `atan2(-R[0][1], R[1][1])` |

These angles reconstruct `Rz(yaw) * Rx(roll) * Ry(pitch)`, matching Voliro's
attitude reference. Thus `vehicle_attitude/pitch` and
`vehicle_attitude_setpoint/pitch` both pass continuously through +/-90 degrees
and can display +/-160 degrees. +160 degrees is approximately +2.7925268 radians;
multiply by `180/pi` to plot degrees. Measured values are not clipped to the
command limit, so an actual overshoot remains visible.

This convention applies whenever the configured airframe is 17, independently
of `VT_CTRL_MODE`, so telemetry also diagnoses the legacy control branch.
The bridge reads the airframe at startup and refreshes it on parameter updates.
`uorb_udp_bridge status` reports the active angle convention.

Other airframes retain the original standard Euler convention. The raw `q0`
through `q3` and `q_d0` through `q_d3` fields are unchanged (Hamilton w,x,y,z).
Rate setpoints and their aliases remain body rates in rad/s.

Receivers using these Voliro scalar fields for 3D rendering must reconstruct
with the stated rotation order or use the original quaternion. Applying the
standard Z-Y-X Euler rotation order to them gives the wrong orientation.
MAVLink attitude messages, QGC's standard horizon, and the underlying uORB
attitude topics retain their definitions; only these UDP derived display
fields change. The convention assumes force-plane bank stays away from +/-90
degrees. Invalid quaternions produce JSON null; a vertical body-Y axis also
produces null for the undefined pitch and heading.

Host conversion regression:

```sh
python3 src/modules/uorb_udp_bridge/test_attitude_display.py build/px4_sitl_vector
```

It covers the entire +/-160-degree sweep, actual/target shared conversion,
quaternion reconstruction/sign equivalence, small normalization errors,
legacy airframes, and invalid/singular inputs. It does not constitute a live
receiver or hardware UDP validation.
