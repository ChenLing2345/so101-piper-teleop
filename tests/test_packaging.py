"""Portable entry point/configuration tests; no hardware is opened."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest
from piper_local import bridge as b


def test_portable_calibration_id_and_directory():
    args = b.parse_args(['--leader-id', 'my_arm', '--calibration-dir', '/tmp/calibration',
                         '--leader-port', '/dev/serial/by-id/example'])
    assert args.leader_id == 'my_arm'
    assert args.calibration_dir == Path('/tmp/calibration')
    assert args.leader_port == '/dev/serial/by-id/example'
    assert b.parse_args([]).calibration_dir is None


def test_module_help_never_needs_connected_hardware():
    result = subprocess.run([sys.executable, '-m', 'piper_local', '--help'],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0
    assert '--leader-id' in result.stdout and '--diagnose-can' in result.stdout


def test_installed_console_entry_point():
    from importlib.metadata import distribution
    points = distribution('so101-piper-teleop').entry_points
    point = next(p for p in points if p.name == 'piper-teleop')
    assert point.load() is b.cli


def test_real_lerobot_adapter_loads_synthetic_calibration_without_connecting(tmp_path):
    if importlib.util.find_spec('lerobot') is None:
        pytest.skip('optional LeRobot dependency not installed')
    data = {name: {'id': i, 'drive_mode': 0, 'homing_offset': 0,
                   'range_min': 0, 'range_max': 4095} for i, name in enumerate(b.NAMES, 1)}
    (tmp_path/'test_arm.json').write_text(json.dumps(data))
    reader = b.LeaderReader('/dev/not-a-real-robot', tmp_path, 'test_arm')
    assert reader.leader.id == 'test_arm'
    assert reader.leader.calibration_fpath == tmp_path/'test_arm.json'
    assert not reader.bus.is_connected
    assert reader.wrist_range == pytest.approx((-180., 180.))
    reader.close()
