"""Bounded ROS 2 VIO playback; accuracy is evaluated separately against ground truth."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import shutil
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--estimator', choices=('schurvins', 'openvins'), default='schurvins')
    parser.add_argument('--rate', type=float, default=0.5)
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--calibration', type=Path)
    parser.add_argument('--result', type=Path)
    parser.add_argument('--tag-params', type=Path, help='ROS parameter file for the online SchurVINS Tag initializer')
    parser.add_argument('--expect-tag', choices=('ready', 'withheld'))
    parser.add_argument('--openvins-diagnostics', action='store_true',
                        help='Record upstream visual update counts and timestamped timing rows')
    args = parser.parse_args()
    if not (args.bag / 'metadata.yaml').is_file():
        raise SystemExit(f'Not a prepared ROS 2 bag: {args.bag}')
    if args.rate <= 0 or args.timeout <= 0:
        raise SystemExit('rate and timeout must be positive')
    if args.openvins_diagnostics and args.estimator != 'openvins':
        parser.error('--openvins-diagnostics requires OpenVINS')
    if bool(args.tag_params) != bool(args.expect_tag) or (args.tag_params and args.estimator != 'schurvins'):
        parser.error('--tag-params and --expect-tag are paired and require SchurVINS')
    result = args.result or Path('/workspaces/benchmark_ws/results') / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    result.mkdir(parents=True, exist_ok=False)
    ws = Path('/workspaces/schurvins_ws')
    upstream = ws / 'upstream/SchurVINS/svo_ros/param'
    config = args.config or upstream / 'vio_stereo.yaml'
    calibration = args.calibration or upstream / 'calib/euroc_stereo.yaml'
    if args.estimator == 'schurvins':
        shutil.copyfile(config, result / 'vio.yaml')
        shutil.copyfile(calibration, result / 'calibration.yaml')
    else:
        if args.config is None:
            parser.error('OpenVINS requires --config estimator_config.yaml')
        for name in ('estimator_config.yaml', 'kalibr_imu_chain.yaml', 'kalibr_imucam_chain.yaml'):
            shutil.copyfile(config.parent / name, result / name)
    rclpy.init()
    node = rclpy.create_node('vio_bootstrap_observer')
    samples, states = [], {}
    tag_samples, tag_status, alignments = [], [], []

    def receive(msg):
        pose = msg.pose.pose if args.estimator == 'openvins' else msg.pose
        q, p = pose.orientation, pose.position
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        samples.append((stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    def status(msg):
        states[msg.data] = states.get(msg.data, 0) + 1

    def receive_tag(msg):
        p, q = msg.pose.position, msg.pose.orientation
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        tag_samples.append((stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    pose_topic = '/openvins/poseimu' if args.estimator == 'openvins' else '/vio/pose'
    pose_type = PoseWithCovarianceStamped if args.estimator == 'openvins' else PoseStamped
    pose_subscription = node.create_subscription(pose_type, pose_topic, receive, 1000)
    status_subscription = node.create_subscription(String, '/vio/status', status, 1000)
    if args.tag_params:
        shutil.copyfile(args.tag_params, result / 'tag.yaml')
        node.create_subscription(PoseStamped, '/vio/tag_pose', receive_tag, 1000)
        node.create_subscription(PoseWithCovarianceStamped, '/vio/tag_alignment', alignments.append, 10)
        node.create_subscription(String, '/vio/tag_status', lambda msg: tag_status.append(msg.data), 10)
    processes, handles = [], []
    failure = None
    playback_code = None
    vio = None
    try:
        handle = (result / 'vio.log').open('w'); handles.append(handle)
        command = [
            'ros2', 'run', 'schurvins_jazzy', 'stereo_vio', '--ros-args',
            '-p', f'config_file:={result / "vio.yaml"}',
            '-p', f'calibration_file:={result / "calibration.yaml"}',
            '-p', f'trace_dir:={result}', '-p', 'use_sim_time:=true',
            '-p', 'input_reliable:=true',
        ] if args.estimator == 'schurvins' else [
            'ros2', 'run', 'openvins_jazzy', 'stereo_vio', '--ros-args',
            '-p', f'config_path:={result / "estimator_config.yaml"}',
            '-p', 'use_sim_time:=true', '-r', '__ns:=/openvins',
        ]
        if args.openvins_diagnostics:
            command.extend(['-p', 'verbosity:=ALL', '-p', 'record_timing_information:=true',
                            '-p', f'record_timing_filepath:={result / "openvins_timing.csv"}'])
        if args.tag_params:
            command.extend(['--params-file', str(result / 'tag.yaml')])
        vio = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(vio)
        deadline = time.monotonic() + min(30, args.timeout)
        while (node.count_publishers(pose_topic) < 1 or any(
                node.count_subscribers(t) < 1 for t in ('/imu0', '/cam0/image_raw', '/cam1/image_raw'))):
            rclpy.spin_once(node, timeout_sec=0.1)
            if vio.poll() is not None:
                raise RuntimeError(f'VIO exited before playback: {vio.returncode}')
            if time.monotonic() >= deadline:
                raise RuntimeError('VIO discovery timed out')
        handle = (result / 'playback.log').open('w'); handles.append(handle)
        playback = subprocess.Popen([
            'ros2', 'bag', 'play', str(args.bag), '--rate', str(args.rate),
            '--clock', '100', '--delay', '2', '--disable-keyboard-controls',
        ], stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(playback)
        deadline = time.monotonic() + args.timeout
        while playback.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.05)
            if vio.poll() is not None:
                raise RuntimeError(f'VIO exited during playback: {vio.returncode}')
            if time.monotonic() >= deadline:
                raise RuntimeError('Playback timed out')
        playback_code = playback.returncode
        if playback_code:
            raise RuntimeError(f'Playback exited: {playback_code}')
        drain_deadline = time.monotonic() + 3
        while time.monotonic() < drain_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if vio.poll() is not None:
            raise RuntimeError(f'VIO exited while draining: {vio.returncode}')
        if len(samples) < 10:
            raise RuntimeError(f'Only {len(samples)} poses received')
        if not all(math.isfinite(v) for row in samples for v in row):
            raise RuntimeError('Non-finite pose received')
        if not all(a[0] < b[0] for a, b in zip(samples, samples[1:])):
            raise RuntimeError('Pose timestamps are not strictly increasing')
        if not all(abs(sum(v*v for v in row[4:8]) - 1) < 1e-3 for row in samples):
            raise RuntimeError('Invalid quaternion norm')
        if args.expect_tag == 'ready':
            if len(alignments) != 1 or 'ready' not in tag_status or not tag_samples:
                raise RuntimeError(f'Expected one Tag alignment. Messages: {len(alignments)}, states: {tag_status}')
            covariance = alignments[0].pose.covariance
            if not all(math.isfinite(x) for x in covariance) or min(covariance[i * 7] for i in range(6)) <= 0:
                raise RuntimeError('Invalid Tag alignment covariance')
            if not all(math.isfinite(v) for row in tag_samples for v in row):
                raise RuntimeError('Non-finite Tag pose received')
            if tag_samples[-1][0] != samples[-1][0]:
                raise RuntimeError('Tag frame output did not persist to the last VIO pose')
        if args.expect_tag == 'withheld' and (alignments or tag_samples or 'ready' in tag_status):
            raise RuntimeError('Global poses were published despite an unqualified Tag alignment')
    except Exception as error:
        failure = str(error)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                import os
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
        for handle in handles: handle.close()
        node.destroy_node(); rclpy.shutdown()
    vio_exit_code = vio.returncode if vio is not None else None
    if failure is None and vio_exit_code != 0:
        failure = f'VIO shutdown failed: exit code {vio_exit_code}'
    def write_tum(name, rows):
        with (result / name).open('w') as stream:
            for stamp, *pose in rows:
                stream.write(f'{stamp // 10**9}.{stamp % 10**9:09d} '
                             + ' '.join(f'{v:.12g}' for v in pose) + '\n')

    write_tum('estimated.tum', samples)
    if args.tag_params:
        write_tum('tag-aligned.tum', tag_samples)
        report_alignments = [
            {'stamp_ns': value.header.stamp.sec * 10**9 + value.header.stamp.nanosec,
             'frame': value.header.frame_id,
             'covariance': list(value.pose.covariance)} for value in alignments
        ]
    report = {
        'passed': failure is None, 'failure': failure, 'pose_count': len(samples),
        'states': states, 'bag': str(args.bag), 'playback_rate': args.rate,
        'playback_exit_code': playback_code,
        'vio_exit_code': vio_exit_code,
        'estimated_span_seconds': (samples[-1][0] - samples[0][0])*1e-9 if samples else 0,
        'estimator': args.estimator, 'pose_topic': pose_topic,
        'config': str(config), 'calibration': str(calibration) if args.estimator == 'schurvins' else None,
        'scope': 'playback flow and pose validity; accuracy evaluated separately; no hardware or real-time acceptance',
    }
    if args.tag_params:
        report.update(tag_states=tag_status, tag_pose_count=len(tag_samples), tag_alignments=report_alignments)
    (result / 'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({'result': str(result), **report}, indent=2))
    if failure: raise SystemExit(1)


if __name__ == '__main__':
    main()
