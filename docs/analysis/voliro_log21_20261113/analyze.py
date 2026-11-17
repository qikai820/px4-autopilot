#!/usr/bin/env python3
"""Reproduce the bounded large-pitch hover analysis from a local PX4 ULog.

Requires pyulog, numpy and matplotlib. No simulator or vehicle is modified.
"""
import argparse
import json
import os
from pathlib import Path

import numpy as np
from pyulog import ULog


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    names = ['vehicle_attitude', 'vehicle_attitude_setpoint', 'vehicle_angular_velocity',
             'vector_actuator_status', 'vector_rate_control_status', 'voliro_tilt_status']
    log = ULog(str(args.log), message_name_filter_list=names)
    data = {topic.name: topic.data for topic in log.data_list if topic.multi_id == 0}

    def times(name):
        return (data[name]['timestamp'].astype(float) - log.start_timestamp) * 1e-6

    def columns(name, field, count=3):
        return np.column_stack([data[name][f'{field}[{i}]'] for i in range(count)])

    def sample(name, field, t):
        return np.interp(t, times(name), data[name][field])

    t = times('vehicle_attitude')
    q = columns('vehicle_attitude', 'q', 4).copy()
    target_q = columns('vehicle_attitude_setpoint', 'q_d', 4).copy()
    for i in range(1, len(target_q)):
        if target_q[i] @ target_q[i - 1] < 0:
            target_q[i] *= -1
    target_q = np.column_stack([np.interp(t, times('vehicle_attitude_setpoint'), target_q[:, i])
                                for i in range(4)])
    target_q /= np.linalg.norm(target_q, axis=1)[:, None]
    # q^-1 * q_d: shortest rotation vector, expressed in the current body frame.
    a, v, b, w = q[:, 0], -q[:, 1:], target_q[:, 0], target_q[:, 1:]
    ew = a * b - np.sum(v * w, axis=1)
    ev = a[:, None] * w + b[:, None] * v + np.cross(v, w)
    ev *= np.where(ew < 0, -1, 1)[:, None]
    norm = np.linalg.norm(ev, axis=1)
    error = np.rad2deg(ev * (2 * np.arctan2(norm, np.abs(ew)) / np.maximum(norm, 1e-12))[:, None])

    def pitch(quat):
        qw, qx, qy, qz = quat.T
        return np.rad2deg(np.arctan2(2 * (qw * qy - qx * qz), 1 - 2 * (qx * qx + qy * qy)))

    status = data['vector_actuator_status']
    ts = times('vector_actuator_status')
    tilt = status['tilt_command']
    axis = np.column_stack([np.sin(tilt), tilt * 0, -np.cos(tilt)])
    allocated = np.sum(axis * columns('vector_actuator_status', 'torque_command'), axis=1)
    front = columns('vector_actuator_status', 'thrust_command', 5)[:, :4]
    km = float(log.initial_parameters['CA_CT_KM'])
    tmax = float(log.initial_parameters['CA_CT_TMAX'])
    pairs = np.column_stack([front[:, :2].sum(axis=1), front[:, 2:].sum(axis=1)])
    capacity = km * np.maximum(0, np.minimum(pairs, 2 * tmax - pairs)).sum(axis=1)
    at_zero = np.any(front < .001, axis=1)
    gyro_t = times('vehicle_angular_velocity')
    gyro = columns('vehicle_angular_velocity', 'xyz')
    gyro_deg = np.rad2deg(gyro)
    params = log.initial_parameters
    inertia = np.array([params[name] for name in ['VEC_IXX', 'VEC_IYY', 'VEC_IZZ']])
    windows = []
    for start, end, label in [(43, 75, '-30 deg'), (88, 118, '+30 deg'),
                              (125, 128, '+45 deg'), (135, 142, '-45 deg')]:
        ei, si, gi = (t >= start) & (t <= end), (ts >= start) & (ts <= end), (gyro_t >= start) & (gyro_t <= end)
        uniform = np.arange(start, end, .01)
        request = sample('vector_actuator_status', 'reaction_requested', uniform)
        frequencies = np.fft.rfftfreq(len(uniform), .01)
        spectrum = abs(np.fft.rfft((request - request.mean()) * np.hanning(len(request)))) ** 2
        band = (frequencies > .5) & (frequencies < 3)
        freq = frequencies[band][np.argmax(spectrum[band])]

        def phasor(values):
            return np.sum((values - values.mean()) * np.hanning(len(values))
                          * np.exp(-2j * np.pi * freq * uniform))

        filtered = sample('vector_actuator_status', 'reaction_filtered', uniform)
        command = np.interp(uniform, ts, allocated)
        rates = np.column_stack([sample('vehicle_angular_velocity', f'xyz[{i}]', uniform) for i in range(3)])
        accel = np.column_stack([sample('vehicle_angular_velocity', f'xyz_derivative[{i}]', uniform) for i in range(3)])
        # This is an inertia-dependent reconstruction, NOT a measured torque.
        observed = inertia * accel + np.cross(rates, inertia * rates)
        servo = np.interp(uniform, ts, tilt)
        observed_axial = np.sin(servo) * observed[:, 0] - np.cos(servo) * observed[:, 2]
        feedforward = data['vector_rate_control_status']
        fi = (times('vector_rate_control_status') >= start) & (times('vector_rate_control_status') <= end)
        windows.append({
            'window_s': [start, end], 'target_pitch': label,
            'body_rotation_error_peak_abs_deg': np.max(abs(error[ei]), axis=0).tolist(),
            'body_rotation_error_std_deg': np.std(error[ei], axis=0).tolist(),
            'body_rate_peak_abs_deg_s': np.max(abs(gyro_deg[gi]), axis=0).tolist(),
            'reaction_peak_abs_requested_nm': float(np.max(abs(status['reaction_requested'][si]))),
            'reaction_peak_abs_filtered_nm': float(np.max(abs(status['reaction_filtered'][si]))),
            'reaction_peak_abs_allocated_nm': float(np.max(abs(allocated[si]))),
            'reaction_capacity_median_nm': float(np.median(capacity[si])),
            'any_front_motor_zero_command_sample_pct': float(100 * np.mean(at_zero[si])),
            'tail_thrust_command_min_max_n': [float(np.min(status['thrust_command[4]'][si])), float(np.max(status['thrust_command[4]'][si]))],
            'reference_feedforward_valid_sample_pct': float(100 * np.mean(feedforward['feedforward_valid'][fi])),
            'dominant_reaction_frequency_hz': float(freq),
            'request_to_filtered_phase_deg': float(np.angle(phasor(filtered) / phasor(request), deg=True)),
            'allocated_to_reconstructed_torque_phase_deg': float(np.angle(phasor(observed_axial) / phasor(command), deg=True)),
        })
    relevant = ['VT_CTRL_MODE', 'VT_REACT_FILT', 'VT_MOT_TAU', 'VT_SRV_TAU', 'VT_ATT_SCALE',
                'VC_ROLL_P', 'VC_PITCH_P', 'VC_YAW_P', 'VC_ROLLRATE_P', 'VC_PITCHRATE_P', 'VC_YAWRATE_P',
                'VC_ROLLRATE_I', 'VC_PITCHRATE_I', 'VC_YAWRATE_I', 'CA_CT_KM', 'CA_CT_TMAX',
                'VEC_IXX', 'VEC_IYY', 'VEC_IZZ', 'VEC_TQ_R', 'VEC_TQ_P', 'VEC_TQ_Y']
    result = {'log_file': str(args.log.resolve()), 'software_hash': log.msg_info_dict.get('ver_sw'),
              'hardware': log.msg_info_dict.get('ver_hw'), 'axis_order': ['body x / roll', 'body y / pitch', 'body z / yaw'],
              'notes': ['Errors are shortest body rotation vectors, not Euler-angle differences.',
                        'Capacity and allocated torque use the logged allocator model; neither is actuator measurement.',
                        'Phase estimates describe closed-loop oscillations; they are not open-loop identification.',
                        'The +45 deg window contains only about three cycles.'],
              'parameters': {key: params[key] for key in relevant}, 'windows': windows}
    (args.output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')

    os.environ.setdefault('MPLCONFIGDIR', '/tmp/voliro-log21-matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    fig = plt.figure(figsize=(13, 9.5), layout='constrained')
    grid = fig.add_gridspec(3, 2, height_ratios=[.7, 1, 1.2])
    ax = fig.add_subplot(grid[0, :])
    ax.plot(t, pitch(target_q), color='#222222', linewidth=1.4, label='Pitch reference')
    ax.plot(t, pitch(q), color='#2376b7', linewidth=1, alpha=.85, label='Actual independent pitch')
    ax.set(xlim=(38, 145), ylabel='Pitch (deg)', xlabel='Seconds since log start')
    ax.legend(loc='upper left', ncols=2)
    for col, (start, end, label) in enumerate([(125, 128, '+45 deg hover'), (135, 142, '-45 deg hover')]):
        ix = (t >= start) & (t <= end)
        ax = fig.add_subplot(grid[1, col])
        for i, name, color in [(0, 'Roll', '#d27a18'), (1, 'Pitch', '#269564'), (2, 'Yaw', '#2376b7')]:
            ax.plot(t[ix], error[ix, i], label=name, color=color, linewidth=1)
        ax.set(title=label, xlim=(start, end), ylim=(-4.5, 4.5), ylabel='Body rotation error (deg)')
        ax.legend(ncols=3, loc='upper right', fontsize=9)
        ax = fig.add_subplot(grid[2, col])
        ix = (ts >= start) & (ts <= end)
        ax.fill_between(ts[ix], -capacity[ix], capacity[ix], color='#dcebdc', label='Allocator reaction capacity')
        ax.plot(ts[ix], status['reaction_requested'][ix], color='#777777', linewidth=1, label='Requested')
        ax.plot(ts[ix], status['reaction_filtered'][ix], color='#d27a18', linewidth=1, label='Filtered (2 Hz)')
        ax.plot(ts[ix], allocated[ix], color='#8b4198', linewidth=1.4, label='Allocated command')
        ax.set(xlim=(start, end), ylim=(-1.9, 1.9), ylabel='Reaction torque (N m)', xlabel='Seconds since log start')
        ax.legend(loc='lower left', fontsize=8, ncols=2)
    for ax in fig.axes:
        ax.grid(alpha=.2)
    fig.suptitle('Voliro T HIL log 21: large-pitch hover oscillation and reaction saturation', fontsize=15)
    fig.savefig(args.output / 'evidence.png', dpi=150)
    plt.close(fig)
    print(json.dumps({'output': str(args.output.resolve()), 'windows': len(windows)}))


if __name__ == '__main__':
    main()
