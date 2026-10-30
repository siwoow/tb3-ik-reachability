"""Run closed-loop tests in a localhost-only ROS domain and private topics."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]


def main():
    env = dict(os.environ, ROS_DOMAIN_ID='187', ROS_LOCALHOST_ONLY='1')
    reports = []
    for name, mode, flags, expected_code in [
            ('dry_run', 'normal', [], 0),
            ('reachable_stop', 'reachable', [], 0),
            ('timestamped_camera_tf', 'camera_tf', [], 0),
            ('navigation_only_grasp_rejected', 'nav_only', ['--execute-grasp'], 1),
            ('closed_loop_pick', 'normal', ['--execute-motion', '--execute-grasp'], 0),
            ('scan_loss_stop', 'scan_loss', ['--execute-motion'], 1),
            ('odom_loss_stop', 'odom_loss', ['--execute-motion'], 1),
            ('service_error_stop', 'service_error', [], 1)]:
        stats = ROOT/'results'/f'mock_{name}_robot.json'
        events = ROOT/'results'/f'mock_{name}_events.jsonl'
        for p in (stats, events):
            p.unlink(missing_ok=True)
        with (ROOT/'results'/f'mock_{name}_server.log').open('w') as server_log, \
                (ROOT/'results'/f'mock_{name}_bridge.log').open('w') as client_log:
            server = subprocess.Popen([sys.executable, str(ROOT/'test/fake_robot.py'),
                                       '--output', str(stats), '--mode', mode],
                                      stdout=server_log, stderr=subprocess.STDOUT, env=env, cwd=ROOT)
            try:
                deadline = time.monotonic()+8
                while not stats.exists() and time.monotonic() < deadline:
                    if server.poll() is not None:
                        raise RuntimeError('Mock server exited; inspect ' + str(server_log.name))
                    time.sleep(.05)
                command = [sys.executable, '-m', 'tb3_grasp_ros2.bridge', '--topic-prefix', '/bridge_test',
                           '--planner-root', str(ROOT.parent/'tb3_grasp_analysis'),
                           '--native-probe', str(ROOT/'reference/native_probe'),
                           '--executor-source-root', str(ROOT/'reference'),
                           '--log', str(events), *flags]
                completed = subprocess.run(command, stdout=client_log, stderr=subprocess.STDOUT,
                                           env=env, cwd=ROOT, timeout=110)
                data = json.loads(stats.read_text())
                rows = [json.loads(line) for line in events.read_text().splitlines()]
                assert completed.returncode == expected_code, (name, completed.returncode)
                if name in ('dry_run', 'reachable_stop', 'timestamped_camera_tf', 'service_error_stop', 'navigation_only_grasp_rejected'):
                    assert data['nonzero_commands'] == data['zero_commands'] == data['picks'] == 0, data
                if name == 'timestamped_camera_tf':
                    observation = next(row for row in rows if row['event'] == 'OBSERVATION')
                    assert observation['source_frame'] == 'mock_camera_optical'
                    assert max(abs(a-b) for a, b in zip(observation['target_odom'], [.32, 0, .051])) < 1e-6
                if name == 'closed_loop_pick':
                    assert data['nonzero_commands'] > 0 and data['picks'] == 1
                    assert data['linear'] == data['angular'] == 0
                    starts = [row for row in rows if row['event'] == 'STEP_START']
                    assert starts and all(abs(row['step']['amount']) <= .05 for row in starts if row['step']['kind'] == 'translate')
                    # Every move has a completed stop and a fresh perception/IK cycle.
                    names = [row['event'] for row in rows]
                    for i, event in enumerate(names):
                        if event == 'STEP_STOPPED':
                            j = names.index('OBSERVATION', i+1)
                            k = names.index('ACTUAL_IK', j+1)
                            assert k > j > i
                if name in ('scan_loss_stop', 'odom_loss_stop'):
                    assert data['nonzero_commands'] > 0 and data['zero_commands'] > 0 and data['linear'] == data['angular'] == 0
                    assert data['picks'] == 0
                    assert any(row['event'] == 'STOP_ERROR' for row in rows)
                reports.append(dict(case=name, passed=True, exit_code=completed.returncode, **data))
                print(name, 'passed', flush=True)
            finally:
                server.send_signal(signal.SIGINT)
                try:
                    server.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    server.kill()
                    server.wait()
    (ROOT/'results/mock_integration_report.json').write_text(json.dumps(reports, indent=2))


if __name__ == '__main__':
    main()
