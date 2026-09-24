"""Offline tests: no serial/CAN hardware is opened."""
import contextlib
import json
from pathlib import Path
import socket
import struct
import sys
import threading
import time
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from piper_local import bridge as b

Q = [10., 30., -30., 15., 5., 10.]
L = [0., 0., 0., 0., 179., 50.]


def feedback(mode=1, teach=0, enabled=True, now=None):
    now = time.monotonic() if now is None else now
    f = b.Feedback()
    f.accept(0x2a1, bytes([mode, 0, 1, teach, 0, 0, 0, 0]), now)
    for cid, data in b.joint_frames(Q):
        f.accept(cid+0x150, data, now)
    for cid in range(0x261, 0x267):
        f.accept(cid, bytes([0, 240, 0, 30, 25, 64 if enabled else 0, 0, 0]), now)
    f.accept(0x2a8, struct.pack('>ihBB', 20000, 0, 64, 0), now)
    return f


def test_anchor_and_held_extra_axis():
    m = b.Mapper(L, Q, 35., gripper=True)
    target, grip = m.step(L, Q, 35., .02)
    assert target == Q and grip == 35
    changed = [10., 10., 10., 10., 189., 60.]
    target, grip = m.step(changed, Q, 35., .02)
    assert target[3] == Q[3]
    assert all(0 < target[j]-Q[j] <= .161 for j in b.AXES)
    assert grip == pytest.approx(36.)


def test_wrist_wrap_gain_and_no_catchup_jump():
    m = b.Mapper(L, Q, 20.)
    changed = list(L); changed[4] = -179.
    target, _ = m.step(changed, Q, 20., 5.)
    assert m.delta[4] == 2
    assert target[5]-Q[5] == pytest.approx(.32)


def test_direction_and_limit():
    q = list(Q); q[0] = 149.99
    m = b.Mapper(L, q, 20., signs=(1, -1, 1, 1, 1))
    changed = list(L); changed[0] += 30; changed[1] += 30
    target, _ = m.step(changed, q, 20., .02)
    assert target[0] == 150
    assert target[1] < q[1]


def test_mapping_does_not_confuse_normal_servo_lag_with_a_stall():
    m = b.Mapper(L, Q, 20.)
    m.previous[0] += 6
    assert m.step(L, Q, 20., .02)  # actual send-time history is checked by PiperCAN


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_never_encoded(value):
    q = list(Q); q[0] = value
    with pytest.raises(ValueError): b.joint_frames(q)
    leader = list(L); leader[2] = value
    with pytest.raises(ValueError): b.Mapper(L, Q, 20.).step(leader, Q, 20., .02)


def test_wire_units_and_default_limits():
    frames = b.joint_frames(Q)
    assert [f[0] for f in frames] == [0x155, 0x156, 0x157]
    assert struct.unpack('>ii', frames[1][1]) == (-30000, 15000)
    q = list(Q); q[5] = 181.
    with pytest.raises(ValueError): b.joint_frames(q)


def test_decode_freshness_fault_and_competing_sender():
    f = feedback(now=100.)
    state = f.snapshot(100.1, True)
    assert state['q'] == Q and state['grip'] == 20 and all(state['enabled'])
    with pytest.raises(TimeoutError): f.snapshot(100.3)
    f.accept(0x151, bytes(8), 100.1)
    with pytest.raises(RuntimeError, match='其他控制端'): f.snapshot(100.1)


@pytest.mark.parametrize('mode,teach,enabled', [(2, 2, True), (1, 1, True), (1, 3, True), (1, 0, False)])
def test_not_ready_rejected(mode, teach, enabled):
    with pytest.raises(RuntimeError): b.check_ready(feedback(mode, teach, enabled).snapshot(time.monotonic()))


def test_motor_fault_rejected():
    f = feedback()
    f.accept(0x261, bytes([0, 240, 0, 30, 25, 65, 0, 0]), time.monotonic())
    with pytest.raises(RuntimeError): b.check_ready(f.snapshot(time.monotonic()))


class DummyReader:
    wrist_range = (-107.07692307692308, 107.07692307692308)
    def connect(self): pass
    def read(self): return L


class DummyRobot:
    stopped = False
    tx_count = 0
    reason = None
    def __init__(self, mode=1): self.mode = mode
    def wait_state(self): return self.state()
    def state(self, *args): return feedback(mode=self.mode).snapshot(time.monotonic())
    def arm(self, *_): raise AssertionError('Preview must never arm')
    def command(self, *_): raise AssertionError('Preview must never send')
    def stop(self, *_): raise AssertionError('Preview cleanup must never stop robot')


def test_end_to_end_preview_no_can_writes(capsys):
    args = b.parse_args(['--seconds', '.05'])
    b.run(args, DummyReader(), DummyRobot(mode=2))
    assert 'PREVIEW' in capsys.readouterr().out


def test_live_requires_operator_arm_before_send(monkeypatch):
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: True)
    monkeypatch.setattr('builtins.input', lambda _: 'cancel')
    b.run(b.parse_args(['--live']), DummyReader(), DummyRobot())


def test_live_pipe_cannot_arm(monkeypatch):
    monkeypatch.setattr(sys.stdin, 'isatty', lambda: False)
    with pytest.raises(RuntimeError, match='现场终端'):
        b.run(b.parse_args(['--live']), DummyReader(), DummyRobot())


def test_prepare_supported_gravity_pose_not_while_teaching():
    state = feedback().snapshot(time.monotonic())
    with pytest.raises(RuntimeError, match='托稳'): b.check_prepare(state)
    state['q'] = [0.] * 6
    b.check_prepare(state)
    state['teach'] = 1
    with pytest.raises(RuntimeError, match='结束录制'): b.check_prepare(state)


def test_watchdog_stops_independently_of_blocked_main_thread():
    # A local Unix socket pair exercises the receiver/watchdog without CAN hardware.
    robot = b.PiperCAN.__new__(b.PiperCAN)
    robot.bus, peer = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
    robot.bus.settimeout(.01); peer.settimeout(.5)
    robot.feedback = feedback()
    robot.lock = threading.RLock(); robot.done = threading.Event()
    robot.live = True; robot.stopped = False; robot.with_gripper = False
    robot.reason = None; robot.tx_count = 0; robot.heartbeat = time.monotonic()-.3
    robot.thread = threading.Thread(target=robot._receive, daemon=True); robot.thread.start()
    try:
        raw = peer.recv(16)
        cid, n, data = struct.unpack('=IB3x8s', raw)
        assert cid == 0x150 and n == 8 and data == bytes([1, 0, 0, 0, 0, 0, 0, 0])
        assert robot.stopped and robot.tx_count == 1
        with pytest.raises(RuntimeError): robot.command(Q, 20., False)
        assert robot.tx_count == 1
    finally:
        robot.close(); peer.close()


def test_prepare_failure_never_enables_motors(monkeypatch):
    robot = b.PiperCAN.__new__(b.PiperCAN)
    robot.lock = threading.RLock(); robot.stopped = False; robot.reason = None; robot.live = False
    state = feedback(mode=2, teach=2).snapshot(time.monotonic()); state['q'] = [0.]*6
    robot.state = lambda *args: dict(state)
    sends = []
    robot._send = lambda cid, data: sends.append((cid, data))
    # Simulate a reset that never produces disabled-motor feedback.
    real_monotonic = time.monotonic
    offset = [0.]
    monkeypatch.setattr(b.time, 'monotonic', lambda: real_monotonic()+offset[0])
    monkeypatch.setattr(b.time, 'sleep', lambda seconds: offset.__setitem__(0, offset[0]+seconds))
    with pytest.raises(TimeoutError): robot.prepare_can()
    assert 0x471 not in [cid for cid, _ in sends]
    assert robot.stopped


@pytest.mark.parametrize('args', [['--gain','nan'], ['--speed','0'], ['--seconds','-1'], ['--signs','1,1']])
def test_bad_cli_rejected(args):
    with pytest.raises(SystemExit): b.parse_args(args)


def test_preview_outside_limit_does_not_invent_a_clipped_start(capsys):
    robot = DummyRobot(mode=2)
    state = robot.state(); state['q'][5] = 186.
    robot.state = lambda *args: state
    b.run(b.parse_args(['--seconds', '.03']), DummyReader(), robot)
    assert '"target_deg": null' in capsys.readouterr().out


def test_preview_near_boundary_uses_live_hold_policy_without_can_writes(capsys):
    robot = DummyRobot()
    state = robot.state(); state['q'][1] = -3.1
    robot.state = lambda *args: state
    b.run(b.parse_args(['--seconds', '.03']), DummyReader(), robot)
    output = capsys.readouterr().out
    assert '"hold_error": null' in output and '"target_deg": null' not in output
    assert '"startup_hold_deg": [10.0, 0.0' in output
    assert state['q'][1] == -3.1


def test_leader_readonly_never_configures_or_disables_on_close():
    reader = b.LeaderReader.__new__(b.LeaderReader)
    calls = []
    reader.bus = SimpleNamespace(
        connect=lambda: calls.append('connect'), is_calibrated=True, is_connected=True,
        sync_read=lambda name, **kw: ({n: 0 for n in b.NAMES} if name == 'Torque_Enable' else dict(zip(b.NAMES, L))),
        disconnect=lambda **kw: calls.append(kw))
    reader.connect()
    assert reader.read() == L
    reader.close()
    assert calls == ['connect', {'disable_torque': False}]


@pytest.mark.parametrize('pose,stop_pose,reset_pose', [
    ([0.]*6, [0.]*6, [0.]*6),
    ([89.34, -.368, -2.353, -.982, 33.204, 91.591],
     [89.34, -.606, -1.913, -1.092, 34.203, 91.591],
     [89.34, -.72, -1.8, -1.1, 32., 91.591]),
    ([89.34, -3.1, -1., 0., 30., 91.],
     [89.34, -3.1, -1., 0., 30., 91.],
     [89.34, -3.1, -1., 0., 30., 91.]),
])
def test_prepare_success_preloads_pose_before_enable(monkeypatch, pose, stop_pose, reset_pose):
    robot = b.PiperCAN.__new__(b.PiperCAN)
    robot.lock = threading.RLock(); robot.stopped = False; robot.reason = None; robot.live = False
    state = feedback(mode=2, teach=2).snapshot(time.monotonic()); state['q'] = list(pose)
    robot.state = lambda *args: dict(state)
    sends = []
    def send(cid, data):
        sends.append((cid, data))
        if cid == 0x150 and data[0] == 1:
            state['q'] = list(stop_pose)
        if cid == 0x150 and data[0] == 2:
            state.update(enabled=[False]*6, teach=0, mode=0)
            state['q'] = list(reset_pose)
        if cid == 0x151:
            state['mode'] = data[0]
        if cid == 0x471:
            state['enabled'] = [True]*6
    robot._send = send
    real_monotonic = time.monotonic
    offset = [0.]
    monkeypatch.setattr(b.time, 'monotonic', lambda: real_monotonic()+offset[0])
    monkeypatch.setattr(b.time, 'sleep', lambda seconds: offset.__setitem__(0, offset[0]+seconds))
    robot.prepare_can()
    ids = [cid for cid, _ in sends]
    enable_index = ids.index(0x471)
    assert ids[enable_index-3:enable_index] == [0x155, 0x156, 0x157]
    assert sends[enable_index-3:enable_index] == b.joint_frames(b.hold_target(reset_pose))
    assert not robot.live and not robot.stopped
    assert all(state['enabled']) and state['mode'] == 1
    assert 0x159 not in ids and 0x121 not in ids


def test_already_ready_prepare_never_resets():
    robot = b.PiperCAN.__new__(b.PiperCAN)
    robot.state = lambda: feedback().snapshot(time.monotonic())
    robot._send = lambda *_: pytest.fail('Already ready must not reset/send')
    robot.prepare_can()


@pytest.mark.parametrize('axis,value', [(0, 150.001), (5, 181.), (1, -3.01), (2, 4.)])
def test_hold_target_clamps_all_boundaries_without_separate_start_gate(axis, value):
    q = list(Q); q[axis] = value
    target = b.hold_target(q)
    assert b.in_limits(target) and q[axis] == value
    assert target[axis] == max(b.LIMITS[axis][0], min(b.LIMITS[axis][1], value))
    with pytest.raises(ValueError): b.joint_frames(q)


def test_zero_boundary_recovery_preserves_feedback_and_strict_commands(capsys):
    state = feedback().snapshot(time.monotonic())
    state['q'][1] = -.351; state['q'][2] = .2
    q = list(state['q'])
    b.check_ready(state)
    target = b.announce_hold(q)
    assert state['q'] == q and q[1] == -.351
    assert target == [q[0], 0., 0., *q[3:]]
    assert '最大差0.351度' in capsys.readouterr().out
    with pytest.raises(ValueError): b.joint_frames(q)
    b.joint_frames(target)


@pytest.mark.parametrize('axis,value', [(1, 11.), (2, -11.), (4, 50.)])
def test_prepare_bad_support_pose_sends_nothing(axis, value):
    state = feedback(mode=2, teach=2).snapshot(time.monotonic())
    state['q'] = [88., 0., -2., 20., 34., 90.]
    state['q'][axis] = value
    robot = b.PiperCAN.__new__(b.PiperCAN)
    robot.state = lambda: state
    robot._send = lambda *_: pytest.fail('Invalid support pose must never send')
    with pytest.raises(RuntimeError, match='托稳'): robot.prepare_can()


def test_support_feedback_is_not_a_hold_command():
    state = feedback(mode=2, teach=2).snapshot(time.monotonic())
    state['q'] = [89., -4., -2., -1., 34., 91.]
    b.check_prepare(state)  # supported reset posture; no target is sent here
    assert b.hold_target(state['q'])[1] == 0.  # ordinary tracking budget, no extra 3 degree gate
    state['q'][1] = -6.
    with pytest.raises(RuntimeError, match='跟随误差'):
        b.hold_target(state['q'])


def test_can_feedback_not_rejected_as_an_outbound_target():
    state = feedback().snapshot(time.monotonic())
    state['q'][1] = -.606
    b.check_ready(state)
    with pytest.raises(ValueError): b.joint_frames(state['q'])
    target = b.hold_target(state['q'])
    assert target[1] == 0 and state['q'][1] == -.606
    mapper = b.Mapper(L, target, 20.)
    mapper.step(L, state['q'], 20., .02)
    tracking = b.TrackingMonitor(60.)
    tracking.sent(target, 100.)
    state['q'][1] = -6.
    with pytest.raises(RuntimeError, match='跟随误差'):
        tracking.check(target, state['q'], 100.01)


@pytest.mark.parametrize('scenario', ['transient_comm', 'persistent_comm', 'angle_fault', 'missing_then_fresh', 'already_reset', 'repeat_enable', 'move_mode_after_enable'])
def test_reset_recovery_and_resume(monkeypatch, scenario):
    clock = [0.]
    monkeypatch.setattr(b.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(b.time, 'sleep', lambda dt: clock.__setitem__(0, clock[0]+dt))
    robot = b.PiperCAN.__new__(b.PiperCAN)
    robot.lock = threading.RLock(); robot.live = False; robot.stopped = False; robot.reason = None
    state = feedback(mode=2, teach=2).snapshot(0.)
    state['q'] = [89.34, -1.877, -1.18, -1.341, 34.398, 90.744]
    if scenario == 'move_mode_after_enable':
        state['move_mode'] = 0
    if scenario == 'already_reset':
        state.update(mode=0, teach=0, enabled=[False]*6)
    sends = []
    reset_at = [None]
    enables = [0]
    def get_state():
        if reset_at[0] is not None:
            elapsed = clock[0]-reset_at[0]
            if scenario == 'missing_then_fresh' and elapsed < .6:
                raise TimeoutError('reset feedback gap')
            if scenario == 'angle_fault':
                return dict(state, status=4, error=0x100)
            if scenario == 'persistent_comm' or (scenario == 'transient_comm' and elapsed < .6):
                return dict(state, status=5, error=52)
        return dict(state)
    def send(cid, data):
        sends.append((clock[0], cid, data))
        if cid == 0x150 and data[0] == 2:
            reset_at[0] = clock[0]
            state.update(mode=0, teach=0, status=0, error=0, enabled=[False]*6)
        if cid == 0x151:
            state['mode'] = data[0]
            if scenario != 'move_mode_after_enable':
                state['move_mode'] = data[1]
        if cid == 0x471:
            enables[0] += 1
            if scenario != 'repeat_enable' or enables[0] == 2:
                state['enabled'] = [True]*6
                state['move_mode'] = 1
    robot.state = get_state
    robot._send = send
    if scenario in ('persistent_comm', 'angle_fault'):
        with pytest.raises(RuntimeError, match='出现故障'):
            robot.prepare_can()
        assert not any(cid in (0x155, 0x156, 0x157, 0x471) for _, cid, _ in sends)
    else:
        robot.prepare_can()
        assert all(state['enabled']) and state['mode'] == 1
        if scenario == 'already_reset':
            assert not any(cid == 0x150 for _, cid, _ in sends)
        else:
            assert all(t-reset_at[0] >= 1. for t, cid, _ in sends if cid in (0x151, 0x155, 0x156, 0x157, 0x471))
        if scenario == 'repeat_enable':
            assert enables[0] == 2


def test_decode_reset_communication_bits():
    f = feedback()
    f.accept(0x2A1, bytes([0, 5, 0, 0, 0, 0, 0, 52]), time.monotonic())
    s = f.snapshot(time.monotonic())
    assert s['communication_fault_joints'] == [3, 5, 6]
    assert s['angle_fault_joints'] == []


def test_full_gain_offset_and_device_j6_range():
    q = list(Q); q[5] = 170.
    b.joint_frames(q)  # actual 0x473 device reply says ±180 degrees
    m = b.Mapper(L, q, 20., gain=1., speed=20., max_offset=90.)
    leader = list(L); leader[0] += 45.; leader[4] += 20.
    target = q
    for _ in range(80):
        target, _ = m.step(leader, target, 20., .04)
    assert target[0] == q[0]+45.  # no hidden 30 degree cap
    assert target[5] == 180. and m.limited_joints == [6]


def test_can_speed_parameter_reaches_wire():
    robot = b.PiperCAN.__new__(b.PiperCAN)
    robot.lock = threading.RLock(); robot.live = False; robot.stopped = False
    robot.state = lambda *args: feedback().snapshot(time.monotonic())
    frames = []
    robot._send = lambda cid, data: frames.append((cid, data))
    robot.arm(False, 50)
    robot.command(Q, 20., False)
    assert frames[0] == (0x151, bytes([1, 1, 50, 0, 0, 0, 0, 0]))


@pytest.mark.parametrize('args', [['--can-speed','0'], ['--can-speed','101'], ['--max-offset','nan'], ['--max-offset','181']])
def test_invalid_motion_tuning_rejected(args):
    with pytest.raises(SystemExit): b.parse_args(args)


@pytest.mark.parametrize('mask,controller,counter,fatal', [
    (4, 0x40, 0, False), (2, 0, 0, False), (0x204, 0x40, 20, False),
    (0x200, 0, 90, False), (0x200, 0, 96, True),
    (4, 8, 0, True), (4, 0x20, 0, True), (4, 0x48, 0, True),
    (0x20, 0, 0, True), (0x40, 0, 0, True), (8, 0, 0, True),
    (0x100, 0, 0, True), (0x400, 0, 0, True), (0, 0, 0, True),
])
def test_can_error_classification_and_raw_evidence(mask, controller, counter, fatal):
    f = feedback(now=100.)
    data = bytes([0, controller, 0, 0, 0, 0, counter, 0])
    f.accept(0x20000000 | mask, data, 100.01)
    event = f.can_events[-1]
    assert event['fatal'] is fatal and event['data_hex'] == data.hex()
    assert event['tx_error_counter'] == (counter if mask & 0x200 else None)
    if fatal:
        with pytest.raises(RuntimeError, match='CAN通信异常'): f.snapshot(100.1)
    else:
        assert f.snapshot(100.1)['q'] == Q


def test_can_recovery_never_clears_latched_fault():
    f = feedback(now=100.)
    f.accept(0x20000040, bytes(8), 100.01)
    f.accept(0x20000004, bytes([0, 0x40, 0, 0, 0, 0, 0, 0]), 100.02)
    with pytest.raises(RuntimeError, match='BUS_OFF'): f.snapshot(100.1)
    assert len(f.can_events) == 2
    assert b.decode_can_error(0x20000004, b'\x00')['fatal']


def command_robot():
    robot = b.PiperCAN.__new__(b.PiperCAN)
    robot.lock = threading.RLock(); robot.live = False; robot.stopped = False
    state = feedback().snapshot(time.monotonic())
    robot.state = lambda *args: dict(state)
    frames = []
    robot._send = lambda cid, data: frames.append((cid, data))
    robot.arm(False, 50)
    return robot, state, frames


def test_pause_hold_detects_drift_at_send_without_mapper():
    robot, state, frames = command_robot()
    robot.command(Q, 20., False)
    frames.clear()
    state['q'] = list(Q); state['q'][0] -= 6.
    with pytest.raises(RuntimeError, match='跟随误差'): robot.command(Q, 20., False)
    assert [cid for cid, _ in frames] == [0x150]
    assert robot.stopped


def test_partial_can_send_failure_stops_and_never_retries_motion():
    robot, state, frames = command_robot()
    def send(cid, data):
        frames.append((cid, data))
        if cid == 0x156:
            raise OSError('No buffer space available')
    robot._send = send
    with pytest.raises(OSError): robot.command(Q, 20., False)
    assert [cid for cid, _ in frames] == [0x151, 0x155, 0x156, 0x150]
    assert robot.stopped
    with pytest.raises(RuntimeError): robot.command(Q, 20., False)
    assert len(frames) == 4


def test_small_boundary_error_can_arm_and_send_legal_first_target():
    robot, state, frames = command_robot()
    state['q'] = [150.01, -3.01, .02, 100.01, 70.01, 180.01]
    b.check_ready(state)  # no pose/zero-angle gate
    robot.command(b.hold_target(state['q']), 20., False)
    assert [cid for cid, _ in frames] == [0x151, 0x155, 0x156, 0x157]
    assert struct.unpack('>ii', frames[1][1]) == (150000, 0)


def test_receive_malformed_packet_latches_stop_instead_of_losing_watchdog():
    robot = b.PiperCAN.__new__(b.PiperCAN)
    robot.bus, peer = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
    robot.bus.settimeout(.01); peer.settimeout(.5)
    robot.feedback = feedback(); robot.lock = threading.RLock(); robot.done = threading.Event()
    robot.live = True; robot.stopped = False; robot.with_gripper = False
    robot.reason = None; robot.tx_count = 0; robot.heartbeat = time.monotonic()
    robot.thread = threading.Thread(target=robot._receive, daemon=True); robot.thread.start()
    try:
        peer.send(b'bad')
        assert struct.unpack('=IB3x8s', peer.recv(16))[0] == 0x150
        assert robot.stopped and 'CAN接收失败' in robot.reason
    finally:
        robot.close(); peer.close()


def test_keyboard_reads_all_buffered_keys_including_quit(monkeypatch):
    import os
    master, slave = os.openpty()
    monkeypatch.setattr(b.sys, 'stdin', SimpleNamespace(isatty=lambda: True, fileno=lambda: slave))
    try:
        with b.keyboard() as read_key:
            os.write(master, b' cq')
            assert b.select.select([slave], [], [], .5)[0]
            assert [read_key(), read_key(), read_key()] == [' ', 'c', 'q']
    finally:
        os.close(master); os.close(slave)


def test_different_initial_poses_use_only_leader_changes():
    leader = [-13.4, -101.8, 92.4, 60.5, 68.4, .54]
    piper = [89.3, 30., -40., 3.5, 30.9, 101.9]
    mapper = b.Mapper(leader, piper, .98, gain=1., speed=20., max_offset=90.)
    assert mapper.step(leader, piper, .98, .02)[0] == piper
    changed = [leader[i]+d for i, d in enumerate([10., -10., -10., 10., 10., 0.])]
    actual = piper
    for _ in range(20):
        actual, _ = mapper.step(changed, actual, .98, .04)
    assert actual == pytest.approx([99.3, 20., -50., 3.5, 40.9, 111.9])


@pytest.mark.parametrize('driver,fatal', [('gs_usb', False), (None, True), ('m_can', True)])
def test_actual_zero_controller_packet_uses_driver_semantics(driver, fatal):
    f = feedback(now=100.)
    f.driver = driver
    f.accept(0x20000004, bytes(8), 100.01)
    event = f.can_events[-1]
    assert event['fatal'] is fatal
    assert event['counters_reported'] is False and event['tx_error_counter'] is None
    if fatal:
        with pytest.raises(RuntimeError): f.snapshot(100.1)
    else:
        assert event['controller'] == ['GS_USB_ERROR_ACTIVE']
        assert f.snapshot(100.1)['q'] == Q


@pytest.mark.parametrize('cid,data', [
    (0x20000024, bytes(8)),  # ACK error combined with controller flag
    (0x20000044, bytes(8)),  # bus-off combined with controller flag
    (0x20000004, bytes([0, 8, 0, 0, 0, 0, 0, 0])),  # TX warning
    (0x20000004, bytes([0, 0x20, 0, 0, 0, 0, 0, 0])),  # TX passive
    (0x20000004, bytes([0, 1, 0, 0, 0, 0, 0, 0])),  # RX overflow
    (0x20000004, bytes([0, 0, 0, 0, 0, 1, 0, 0])),  # nonzero reserved field
    (0x20000004, bytes([0, 0, 0, 0, 0, 0, 96, 0])),  # warning-level TEC
    (0x20000004, bytes([0, 0, 0, 0, 0, 0, 0, 128])),  # passive-level REC
])
def test_gs_usb_notification_does_not_mask_real_or_mixed_errors(cid, data):
    assert b.decode_can_error(cid, data, 'gs_usb')['fatal']


def test_gs_usb_active_does_not_clear_previous_fault():
    f = feedback(now=100.)
    f.driver = 'gs_usb'
    f.accept(0x20000040, bytes(8), 100.01)
    f.accept(0x20000004, bytes(8), 100.02)
    assert f.can_events[-1]['fatal'] is False
    with pytest.raises(RuntimeError, match='BUS_OFF'): f.snapshot(100.1)


def test_gripper_absolute_open_close_independent_of_start_and_reanchor():
    leader = list(L); leader[5] = 100.
    m = b.Mapper(leader, Q, 2., gripper=True, gripper_speed=50.)
    for _ in range(140):
        _, grip = m.step(leader, Q, 2., .01)  # contact/position error does not abort the arm
    assert grip == 70.
    leader[5] = 0.
    for _ in range(140):
        _, grip = m.step(leader, Q, 20., .01)
    assert grip == 0.
    m = b.Mapper(leader, Q, 20., gripper=True, gripper_speed=50.)
    assert m.step(leader, Q, 20., .01)[1] == 19.5  # C must not redefine closed to 20mm


def test_servo_first_target_preloaded_and_gripper_wire_units():
    robot, state, frames = command_robot()
    robot.arm(True, 100, 'servo', .5)
    robot.command(Q, 35., True)
    assert [cid for cid, _ in frames] == [0x155, 0x156, 0x157, 0x151, 0x155, 0x156, 0x157, 0x159]
    assert frames[3][1] == bytes([1, 1, 100, 0xAD, 0, 0, 0, 0])
    assert frames[-1][1] == struct.pack('>iHBB', 35000, 500, 1, 0)
    frames.clear(); robot.command(Q, 0., True)
    assert [cid for cid, _ in frames] == [0x151, 0x155, 0x156, 0x157, 0x159]
    assert robot.live and not robot.stopped  # 20mm actual while gripping is not an arm fault


def test_gripper_hardware_fault_still_stops():
    robot, state, frames = command_robot()
    robot.arm(True)
    state['gripper_fault'] = 4
    with pytest.raises(RuntimeError, match='夹爪'): robot.command(Q, 0., True)
    assert [cid for cid, _ in frames] == [0x150]


def test_smooth_follow_is_faster_than_old_limit_and_still_bounded():
    m = b.Mapper(L, Q, 20., gain=1., speed=60., acceleration=240., max_offset=90.)
    leader = list(L); leader[0] += 30
    actual = list(Q); positions = [actual[0]]
    for _ in range(100):
        actual, _ = m.step(leader, actual, 20., .01)
        positions.append(actual[0])
    assert actual[0] == pytest.approx(Q[0]+30, abs=.01)
    assert positions[25]-positions[0] > 5  # old 10deg/s can move only 2.5deg here
    assert 0 < positions[1]-positions[0] <= .024001
    assert all(0 <= b_-a <= .600001 for a, b_ in zip(positions, positions[1:]))
    assert max(positions) <= Q[0]+30
    # Reversal returns smoothly without crossing mechanical limits.
    leader[0] -= 30
    for _ in range(100):
        actual, _ = m.step(leader, actual, 20., .01)
    assert actual[0] == pytest.approx(Q[0], abs=.01)


def test_updated_defaults_and_explicit_gripper_opt_out():
    args = b.parse_args([])
    assert args.gripper and args.hz == 100 and args.speed == 60 and args.gain == 1
    assert not b.parse_args(['--no-gripper']).gripper
    assert b.parse_args(['--control-mode', 'servo']).control_mode == 'servo'


def test_observed_shoulder_left_right_direction_uses_inverted_j1_only():
    args = b.parse_args([])
    assert args.signs == (-1, 1, 1, 1, 1)
    assert b.parse_args(['--signs=-1,1,1,1,1']).signs == args.signs
    mapper = b.Mapper(L, Q, 35., gain=1, speed=60, signs=args.signs,
                      acceleration=240, gripper=True, max_offset=90)
    leader = [v+10 if i < 5 else v for i, v in enumerate(L)]
    actual = list(Q)
    for _ in range(100):
        actual, grip = mapper.step(leader, actual, 35., .01)
    assert actual == pytest.approx([Q[0]-10, Q[1]+10, Q[2]+10, Q[3], Q[4]+10, Q[5]+10])
    assert grip == 35.


@pytest.mark.parametrize('args', [
    ['--speed','121'], ['--accel','0'], ['--hz','0'], ['--hz','nan'],
    ['--gripper-speed','0'], ['--gripper-open-mm','71'], ['--gripper-effort','6'],
])
def test_bad_responsiveness_settings_rejected(args):
    with pytest.raises(SystemExit): b.parse_args(args)


def test_stop_cancels_remaining_gripper_travel_without_open_or_disable():
    robot, state, frames = command_robot()
    robot.arm(True, 100, 'joint', .5)
    robot.command(Q, 0., True)  # object blocks closure at measured 20mm
    frames.clear()
    robot.stop('Q')
    assert frames == [b.gripper_frame(20., .5), (0x150, bytes([1,0,0,0,0,0,0,0]))]


def test_live_fast_follow_with_150ms_motor_lag_and_reversal(monkeypatch):
    clock = [100.]
    monkeypatch.setattr(b.time, 'monotonic', lambda: clock[0])
    robot, state, frames = command_robot()
    robot.arm(False, 100, 'joint', .5, 60.)
    robot.command(Q, 20., False)
    mapper = b.Mapper(L, Q, 20., gain=1., speed=60., acceleration=240., max_offset=90.)
    delayed = [list(Q) for _ in range(15)]
    max_error = 0.
    for i in range(300):
        clock[0] += .01
        state['q'] = delayed.pop(0)
        leader = list(L); leader[2] -= 30. if i < 150 else 0.
        target, grip = mapper.step(leader, state['q'], 20., .01)
        max_error = max(max_error, abs(target[2]-state['q'][2]))
        robot.command(target, grip, False)
        delayed.append(target)
    assert max_error > 8  # this exact valid trajectory would hit the former 5deg stop
    assert not robot.stopped and state['q'][2] == pytest.approx(Q[2])
    assert len(robot.tracking.history) <= 27


@pytest.mark.parametrize('delay_frames', [None, 60])
def test_stalled_or_excessively_delayed_joint_still_stops(monkeypatch, delay_frames):
    clock = [100.]
    monkeypatch.setattr(b.time, 'monotonic', lambda: clock[0])
    robot, state, frames = command_robot()
    robot.command(Q, 20., False)
    delayed = [list(Q) for _ in range(delay_frames or 1)]
    with pytest.raises(RuntimeError, match='跟随误差'):
        for i in range(1, 150):
            clock[0] += .01
            if delay_frames:
                state['q'] = delayed.pop(0)
            target = list(Q); target[2] -= 60*i*.01
            robot.command(target, 20., False)
            delayed.append(target)
    assert robot.stopped and frames[-1][0] == 0x150
    assert clock[0] < 100.4  # continuing target must not integrate forever on a stuck joint


def test_first_target_and_large_runtime_divergence_remain_bounded():
    robot, state, frames = command_robot()
    target = list(Q); target[2] -= 5.2
    with pytest.raises(RuntimeError, match='跟随误差'):
        robot.command(target, 20., False)
    assert [cid for cid, _ in frames] == [0x150]
    robot, state, frames = command_robot()
    robot.command(Q, 20., False)
    target[2] = Q[2]-20.1  # 5deg + 60deg/s * .25s is the immediate lead bound
    with pytest.raises(RuntimeError, match='跟随误差'):
        robot.command(target, 20., False)
    assert robot.stopped


def test_partial_send_does_not_enter_tracking_history():
    robot, state, frames = command_robot()
    def send(cid, data):
        frames.append((cid, data))
        if cid == 0x156:
            raise OSError('No buffer space available')
    robot._send = send
    with pytest.raises(OSError): robot.command(Q, 20., False)
    assert not robot.tracking.history


def test_default_reach_is_device_range_not_90_degree_startup_cap():
    args = b.parse_args([])
    assert args.max_offset == 0 and args.wrist_mapping == 'range'
    assert b.parse_args(['--wrist-mapping', 'relative']).wrist_mapping == 'relative'
    leader0 = [-13., -101.846, 89.407, 72.967, 80., .5]
    piper0 = [96., 0., 0., 1.587, 20.189, 97.568]
    mapper = b.Mapper(leader0, piper0, 1., gain=1, signs=args.signs, speed=60,
                      acceleration=240, max_offset=args.max_offset,
                      wrist_range=DummyReader.wrist_range)
    actual = piper0
    for i in range(600):
        fraction = min(1., i/300)
        leader = list(leader0)
        leader[1] += 170.462*fraction
        leader[2] -= 154.462*fraction
        actual, _ = mapper.step(leader, actual, 1., .01)
    assert actual[1:3] == pytest.approx([170.462, -154.462])
    assert mapper.offset_limited_joints == [] and b.in_limits(actual)


@pytest.mark.parametrize('sign', [1, -1])
def test_wrist_range_keeps_anchor_and_reaches_both_ends(sign):
    leader0 = list(L); leader0[3] = 72.967
    piper0 = list(Q); piper0[4] = 20.189
    signs = (1, 1, 1, sign, 1)
    mapper = b.Mapper(leader0, piper0, 35., gain=1, speed=60, max_offset=0,
                      acceleration=240, wrist_range=DummyReader.wrist_range, signs=signs)
    actual, _ = mapper.step(leader0, piper0, 35., .01)
    assert actual == piper0  # no move to a new absolute zero at startup
    leader = list(leader0)
    for end in [DummyReader.wrist_range[0], DummyReader.wrist_range[1]]:
        start = leader[3]
        for i in range(500):
            leader[3] = start+(end-start)*min(1., (i+1)/300)
            previous = actual
            actual, _ = mapper.step(leader, actual, 35., .01)
            assert abs(actual[4]-previous[4]) <= .600001
            assert b.in_limits(actual)
        expected = (70. if end > 0 else -70.)*sign
        assert actual[4] == pytest.approx(expected)


def test_wrist_reversal_in_previously_saturated_input_still_changes_target():
    leader0 = list(L); leader0[3] = 72.967
    piper0 = list(Q); piper0[4] = 20.189
    new = b.Mapper(leader0, piper0, 35., gain=1, max_offset=0, wrist_range=DummyReader.wrist_range)
    old = b.Mapper(leader0, piper0, 35., gain=1, max_offset=90)
    targets = []
    for wrist in [40., 0., -40., -80., -100., -90., -80.]:
        leader = list(leader0); leader[3] = wrist
        new.step(leader, piper0, 35., .01); old.step(leader, piper0, 35., .01)
        targets.append((new.desired[4], old.desired[4]))
    assert targets[-3][0] < targets[-2][0] < targets[-1][0]
    assert targets[-3][1] == targets[-2][1] == targets[-1][1]


def test_wrist_range_reanchor_still_holds_and_degenerate_side_does_not_divide_by_zero():
    for start in [-107.07692307692308, 10., 107.3]:
        leader = list(L); leader[3] = start
        mapper = b.Mapper(leader, Q, 35., gain=1, max_offset=0, wrist_range=DummyReader.wrist_range)
        target, _ = mapper.step(leader, Q, 35., .01)
        assert target == Q


def test_wrist_degree_range_comes_from_existing_encoder_calibration():
    reader = b.LeaderReader.__new__(b.LeaderReader)
    reader.leader = SimpleNamespace(calibration={'wrist_flex': SimpleNamespace(range_min=809, range_max=3245)})
    reader.bus = SimpleNamespace(model_resolution_table={'sts3215': 4096}, motors={'wrist_flex':SimpleNamespace(model='sts3215')})
    assert reader.wrist_range == pytest.approx(DummyReader.wrist_range)


def test_explicit_small_offset_and_true_joint_limits_still_apply():
    leader0 = list(L)
    mapper = b.Mapper(leader0, Q, 35., gain=1, max_offset=20)
    leader = list(leader0); leader[0] += 40
    mapper.step(leader, Q, 35., .01)
    assert mapper.desired[0] == Q[0]+20 and mapper.offset_limited_joints == [1]
    mapper = b.Mapper(leader0, [149., *Q[1:]], 35., gain=1, max_offset=0)
    mapper.step(leader, Q, 35., .01)
    assert mapper.desired[0] == 150 and mapper.joint_limited_joints == [1]


def test_observed_no_ack_is_not_a_gs_usb_recovery(capsys):
    f = feedback(now=100.)
    f.driver = 'gs_usb'
    f.accept(0x20000024, bytes.fromhex('0000000000000800'), 100.01)
    f.accept(0x20000004, bytes.fromhex('0000000000005f00'), 100.02)
    with pytest.raises(RuntimeError, match='NO_ACK'):
        f.snapshot(100.03)
    assert f.can_events[0]['fatal']
    assert not f.can_events[0]['counters_reported']


@pytest.mark.parametrize('tx,rx', [(0, 0), (1, 0), (95, 0), (0, 95), (95, 95)])
def test_legacy_gs_usb_recovery_with_counter_bytes_keeps_feedback_available(tx, rx):
    data = bytes([0, 0, 0, 0, 0, 0, tx, rx])
    f = feedback(now=100.); f.driver = 'gs_usb'
    f.accept(0x20000004, data, 100.01)
    assert not f.can_events[-1]['fatal'] and f.snapshot(100.02)['q'] == Q
    assert f.can_events[-1]['data_hex'] == data.hex()
    assert b.decode_can_error(0x20000004, data, 'm_can')['fatal']


@pytest.mark.parametrize('received', [False, True])
def test_failure_report_distinguishes_missing_and_stale_and_preserves_can_evidence(monkeypatch, tmp_path, received):
    monkeypatch.setattr(b, 'FAILURE_DIR', tmp_path)
    monkeypatch.setattr(b.time, 'monotonic', lambda: 101.)
    initial = {'linkinfo': {'info_xstats': {'error_warning': 5, 'error_passive': 5256}}}
    current = {'linkinfo': {'info_data': {'state': 'ERROR-WARNING'},
                            'info_xstats': {'error_warning': 8, 'error_passive': 10435}}}
    monkeypatch.setattr(b, 'can_link_info', lambda _: current)
    robot = SimpleNamespace(lock=threading.RLock(), initial_link=initial, tx_count=0,
                            feedback=feedback(now=100.) if received else b.Feedback())
    b.record_failure('can0', robot, TimeoutError('反馈缺失或超过250ms'))
    report = json.loads(next(tmp_path.glob('failure-*.json')).read_text())
    assert report['can_tx'] == 0 and not report['feedback_complete']
    assert report['required_frame_age_ms']['0x2a1'] == (1000. if received else None)
    assert report['can_counter_delta'] == {'error_warning': 3, 'error_passive': 5179}


def test_diagnose_can_never_opens_leader_or_sends_can(monkeypatch, capsys):
    class PassiveRobot:
        def __init__(self, interface):
            self.lock = threading.RLock(); self.feedback = feedback()
            self.initial_link = {}; self.tx_count = 0
        def wait_state(self, **kwargs): return self.feedback.snapshot(time.monotonic())
        def state(self, *args): return self.feedback.snapshot(time.monotonic())
        def stop(self, reason): pass  # no live session, like real stop()
        def close(self): pass
    @contextlib.contextmanager
    def lock(interface, port):
        assert port is None
        yield
    monkeypatch.setattr(b, 'PiperCAN', PassiveRobot)
    monkeypatch.setattr(b, 'device_locks', lock)
    monkeypatch.setattr(b, 'can_link_info', lambda _: {})
    monkeypatch.setattr(b, 'LeaderReader', lambda *a: pytest.fail('diagnostic opened leader'))
    b.main(['--diagnose-can', '--seconds', '.01'])
    output = capsys.readouterr().out
    assert '[CAN_DIAG]' in output and '本进程CAN发送总数：0' in output


def test_failure_report_disk_error_does_not_replace_original_error(monkeypatch, tmp_path, capsys):
    blocked = tmp_path/'file'; blocked.write_text('not a directory')
    monkeypatch.setattr(b, 'FAILURE_DIR', blocked)
    monkeypatch.setattr(b, 'can_link_info', lambda _: {})
    b.record_failure('can0', None, RuntimeError('original NO_ACK'))
    assert '无法保存诊断' in capsys.readouterr().err


def test_diagnostic_cannot_be_combined_with_motion_or_run_forever():
    for argv in (['--diagnose-can', '--live'], ['--diagnose-can', '--prepare-can'],
                 ['--diagnose-can', '--seconds', '0']):
        with pytest.raises(SystemExit): b.parse_args(argv)


def test_diagnostic_reports_a_gap_after_startup_without_retrying_or_sending(monkeypatch):
    calls = []
    class DroppedLink:
        tx_count = 0
        def __init__(self, _): pass
        def wait_state(self, **kwargs): calls.append('initial_wait')
        def state(self, *args): raise TimeoutError('250ms missing')
        def stop(self, reason): calls.append('stop')
        def close(self): calls.append('close')
    monkeypatch.setattr(b, 'PiperCAN', DroppedLink)
    monkeypatch.setattr(b, 'device_locks', lambda *a: contextlib.nullcontext())
    monkeypatch.setattr(b, 'record_failure', lambda *a: calls.append('record'))
    with pytest.raises(TimeoutError, match='250ms missing'):
        b.main(['--diagnose-can', '--seconds', '1'])
    assert calls == ['initial_wait', 'stop', 'record', 'stop', 'close']
