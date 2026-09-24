# SPDX-License-Identifier: Apache-2.0
"""Local SO-101 -> PiPER bridge. Preview sends ZERO CAN frames.

Wire format follows agilexrobotics/piper_sdk c9e8a281 (protocol_v2).
Leader reads use LeRobot's Feetech bus without configure/calibrate/torque writes.
No light commands, firmware updates, limit writes, or automatic teaching-mode reset.
"""
from __future__ import annotations

import argparse
from collections import deque
import contextlib
import fcntl
import json
import math
import os
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import sys
import termios
import threading
import time
import tty

NAMES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
# SO101 has five rotary axes. Hold PiPER J4; wrist pitch -> J5, roll -> J6.
AXES = (0, 1, 2, 4, 5)
# Read back from this PiPER's 0x473 replies on 2026-09-23. J6 is ±180,
# unlike the generic SDK table's ±120. No firmware limits are written.
LIMITS = ((-150., 150.), (0., 180.), (-170., 0.), (-100., 100.), (-70., 70.), (-180., 180.))
PORT = os.environ.get('PIPER_LEADER_PORT', '/dev/ttyACM0')
REQUIRED = (0x2A1, 0x2A5, 0x2A6, 0x2A7, *range(0x261, 0x267))
MAX_TRACKING_ERROR = 5.  # static position tolerance, not a zero-latency servo requirement
MAX_FOLLOWING_LAG = .25  # bounded command-history window for moving joints (seconds)
RESET_SETTLE_SECONDS = 1.  # manufacturer's playTrajectory_new.py reset settling time
FAILURE_DIR = Path(os.environ.get('XDG_STATE_HOME', Path.home()/'.local/state')) / 'so101-piper-teleop/failures'


def can_link_info(interface):
    """Read kernel counters without changing/resetting the CAN interface."""
    result = subprocess.run(['ip', '-details', '-statistics', '-json', 'link', 'show', interface],
                            capture_output=True, text=True, timeout=2.)
    if result.returncode:
        return {'interface': interface, 'read_error': result.stderr.strip()}
    return json.loads(result.stdout)[0]


def can_diagnostics(interface, robot=None):
    report = {'interface': interface, 'link_now': can_link_info(interface)}
    if robot is None:
        return report
    with robot.lock:
        now = time.monotonic()
        ages = {hex(cid): round((now-frame[1])*1000, 1)
                for cid, frame in robot.feedback.frames.items()}
        required_ages = {hex(cid): ages.get(hex(cid)) for cid in (*REQUIRED, 0x2A8)}
        report.update(can_tx=robot.tx_count, link_at_start=robot.initial_link,
                      required_frame_age_ms=required_ages,
                      received_ids=sorted(ages), can_events=list(robot.feedback.can_events),
                      receiver_error=robot.feedback.error, conflict=robot.feedback.conflict)
    report['feedback_complete'] = all(age is not None and age <= 250 for age in required_ages.values())
    before = robot.initial_link.get('linkinfo', {}).get('info_xstats', {})
    after = report['link_now'].get('linkinfo', {}).get('info_xstats', {})
    report['can_counter_delta'] = {key: value-before[key] for key, value in after.items() if key in before}
    return report


def record_failure(interface, robot, error):
    # Run only AFTER stopping. Disk/ip diagnostics must not delay the stop path.
    try:
        report = can_diagnostics(interface, robot)
        report.update(error=str(error), wall_time=time.strftime('%Y-%m-%d %H:%M:%S%z'))
        FAILURE_DIR.mkdir(parents=True, exist_ok=True)
        path = FAILURE_DIR / f'failure-{time.time_ns()}.json'
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(f'[DIAG] 故障现场已保存：{path}', flush=True)
        if 'NO_ACK' in str(error):
            print('[DIAG] CAN发送未获总线确认。检查机械臂供电及CAN/USB连接；'
                  '恢复后重新运行--prepare-can，不自动续发旧目标。', flush=True)
        elif isinstance(error, TimeoutError):
            ages = report.get('required_frame_age_ms', {})
            detail = ('本进程未收到任何有效数据帧' if not report.get('received_ids') else
                      '部分反馈缺失或已停止更新')
            print(f'[DIAG] {detail}；CAN状态='
                  f'{report["link_now"].get("linkinfo", {}).get("info_data", {}).get("state", "未知")}；'
                  f'各反馈距今毫秒={ages}。可用--diagnose-can独立检查。', flush=True)
    except Exception as exc:
        print(f'[DIAG] 无法保存诊断（原始故障仍保留）：{exc}', file=sys.stderr, flush=True)


def finite(values):
    if not all(math.isfinite(v) for v in values):
        raise ValueError('非有限关节/参数；停止。')


def in_limits(q):
    finite(q)
    return len(q) == 6 and all(lo <= v <= hi for v, (lo, hi) in zip(q, LIMITS))


def check_tracking(target, measured, max_error=MAX_TRACKING_ERROR):
    finite([*target, *measured])
    for i, (want, actual) in enumerate(zip(target, measured)):
        if abs(want-actual) > max_error:
            raise RuntimeError(f'J{i+1}目标{want:.3f}°与实测{actual:.3f}°'
                               f'跟随误差超过{max_error:g}度；停止。')


class TrackingMonitor:
    """Check feedback against recently SENT targets, allowing bounded servo lag."""
    def __init__(self, speed):
        self.speed = speed
        self.history = deque()

    def check(self, target, measured, now):
        if not self.history:
            check_tracking(target, measured)  # first command must hold the actual pose
            return
        # Keep the sample immediately before the window boundary, too. Targets
        # are held between transmissions; dropping it would shorten the window.
        while len(self.history) > 1 and self.history[1][0] <= now-MAX_FOLLOWING_LAG:
            self.history.popleft()
        check_tracking(target, measured, MAX_TRACKING_ERROR+self.speed*MAX_FOLLOWING_LAG)
        for i, actual in enumerate(measured):
            recent = [q[i] for _, q in self.history]
            lo, hi = min(recent), max(recent)
            if not lo-MAX_TRACKING_ERROR <= actual <= hi+MAX_TRACKING_ERROR:
                raise RuntimeError(f'J{i+1}跟随误差：实测{actual:.3f}°已偏离最近'
                                   f'{MAX_FOLLOWING_LAG*1000:g}ms已发送目标范围'
                                   f'[{lo:.3f},{hi:.3f}]°超过{MAX_TRACKING_ERROR:g}°；'
                                   f'当前目标{target[i]:.3f}°；停止。')

    def sent(self, target, now):
        self.history.append((now, tuple(target)))


def gripper_frame(mm, effort=.5):
    finite([mm, effort])
    if not 0 <= mm <= 70 or not 0 < effort <= 5:
        raise ValueError('夹爪指令需0–70mm，力矩需(0,5]N·m')
    return 0x159, struct.pack('>iHBB', round(mm*1000), round(effort*1000), 1, 0)


def smooth_step(position, velocity, goal, speed, acceleration, dt):
    """Velocity ramp with braking near goal; never overshoot a fixed target."""
    if dt <= 0:
        return position, velocity
    error = goal-position
    # Discrete braking speed includes the next integration step.
    braking = math.sqrt(2*acceleration*abs(error) + (acceleration*dt/2)**2) - acceleration*dt/2
    desired_velocity = math.copysign(min(speed, braking), error) if error else 0.
    velocity = max(velocity-acceleration*dt, min(velocity+acceleration*dt, desired_velocity))
    next_position = position+velocity*dt
    if error and (goal-next_position)*error <= 0:
        return goal, 0.
    return next_position, velocity


def joint_frames(q):
    if len(q) != 6 or not in_limits(q):
        raise ValueError(f'待发送关节目标超出本机读回的关节范围：{q}；请核对型号/零点。')
    vals = [round(v * 1000) for v in q]
    return [(0x155 + i, struct.pack('>ii', *vals[2*i:2*i+2])) for i in range(3)]


def hold_target(q):
    """Build a legal hold command; this is NOT a feedback validity check.

    Project measurements onto command limits for all axes, within the static
    position tolerance. No separate zero-angle gate or dynamic lag allowance.
    """
    finite(q)
    if len(q) != 6:
        raise ValueError('PiPER需要六个关节角度')
    target = [max(lo, min(hi, v)) for v, (lo, hi) in zip(q, LIMITS)]
    joint_frames(target)
    check_tracking(target, q)
    return target


def announce_hold(q):
    target = hold_target(q)
    if target != q:
        print(f'保持目标裁剪到指令范围：实测{q}；保持目标{target}。'
              f'目标与实测最大差{max(abs(a-b) for a, b in zip(q, target)):.3f}度；'
              '不修改实测值、标定或固件限位。', flush=True)
    return target


def anchored_range(value, source_anchor, target_anchor, source_limits, target_limits):
    """Map each remaining side of a calibrated stroke without an entry jump."""
    lo, hi = source_limits
    start = max(lo, min(hi, source_anchor))
    value = max(lo, min(hi, value))
    if value > start:
        return target_anchor + (value-start)/(hi-start)*(target_limits[1]-target_anchor)
    if value < start:
        return target_anchor + (value-start)/(start-lo)*(target_anchor-target_limits[0])
    return target_anchor


class Mapper:
    """Relative anchoring, wrap-aware roll and bounded target velocity."""
    def __init__(self, leader, q, grip, gain=.25, signs=(1,)*5, speed=8., gripper=False, max_offset=30.,
                 acceleration=None, gripper_speed=50., gripper_open_mm=70., wrist_range=None):
        finite([*leader, *q, grip, gain, speed, *signs, max_offset, gripper_speed, gripper_open_mm])
        if acceleration is not None:
            finite([acceleration])
        self.anchor = list(q)
        self.leader_anchor = list(leader)
        self.wrist_range = wrist_range
        if wrist_range is not None:
            finite(wrist_range)
            if len(wrist_range) != 2 or wrist_range[0] >= wrist_range[1]:
                raise ValueError('腕部标定范围无效')
        self.previous_leader = list(leader)
        self.delta = [0.] * 6
        self.previous = list(q)
        self.anchor_grip = self.previous_grip = grip
        self.gain, self.signs, self.speed, self.gripper = gain, signs, speed, gripper
        self.max_offset = max_offset
        self.acceleration = acceleration
        self.velocity = [0.] * 6
        self.gripper_speed, self.gripper_open_mm = gripper_speed, gripper_open_mm
        self.limited_joints = []
        self.offset_limited_joints = []
        self.joint_limited_joints = []
        self.rate_limited_joints = []
        self.desired = list(q)
        self.desired_grip = grip

    def step(self, leader, measured, measured_grip, dt):
        finite([*leader, *measured, measured_grip, dt])
        dt = min(max(dt, 0.), .04)  # stalls never produce a catch-up jump
        for i in range(6):
            inc = leader[i] - self.previous_leader[i]
            if i == 4:
                inc = (inc + 180.) % 360. - 180.
            if i < 5 and abs(inc) > 45:
                raise ValueError('Leader单帧跳变超过45度；检查串口/标定，停止跟随。')
            self.delta[i] += inc
        self.previous_leader = list(leader)
        desired = list(self.anchor)
        self.limited_joints = []
        self.offset_limited_joints = []
        self.joint_limited_joints = []
        self.rate_limited_joints = []
        for i, axis in enumerate(AXES):
            raw_offset = self.delta[i] * self.gain * self.signs[i]
            if i == 3 and self.wrist_range is not None:
                # SO101's ~214 degree pitch stroke cannot be copied 1:1 into
                # PiPER J5's 140 degrees. Allocate BOTH remaining strokes, so
                # the startup offset does not waste reach or create a dead zone.
                sign = self.signs[i]
                source_limits = tuple(sorted(v*sign for v in self.wrist_range))
                mapped = anchored_range(leader[i]*sign, self.leader_anchor[i]*sign,
                                        self.anchor[axis], source_limits, LIMITS[axis])
                raw_offset = (mapped-self.anchor[axis])*self.gain
            offset = (max(-self.max_offset, min(self.max_offset, raw_offset))
                      if self.max_offset else raw_offset)
            if offset != raw_offset:
                self.limited_joints.append(axis+1)
                self.offset_limited_joints.append(axis+1)
            desired[axis] += offset
        target = []
        for i, ((lo, hi), want) in enumerate(zip(LIMITS, desired)):
            want = max(lo, min(hi, want))
            if want != desired[i] and i+1 not in self.limited_joints:
                self.limited_joints.append(i+1)
            if want != desired[i]:
                self.joint_limited_joints.append(i+1)
            # Runtime tracking belongs to PiperCAN: it has the history of targets
            # actually sent and also covers pause/re-anchor and fresh feedback.
            self.desired[i] = want
            if self.acceleration is None:
                step = self.speed * dt
                next_q = max(self.previous[i]-step, min(self.previous[i]+step, want))
            else:
                next_q, self.velocity[i] = smooth_step(self.previous[i], self.velocity[i], want,
                                                      self.speed, self.acceleration, dt)
                bounded = max(lo, min(hi, next_q))
                if bounded != next_q:
                    self.velocity[i] = 0.
                next_q = bounded
            target.append(next_q)
            if abs(next_q-want) > .1:
                self.rate_limited_joints.append(i+1)
        self.previous = target
        grip = self.previous_grip
        if self.gripper:
            # A gripper represents aperture, not a relative arm-joint offset.
            # Starting with an open leader must not erase its opening range.
            want = max(0., min(100., leader[5])) * self.gripper_open_mm / 100.
            self.desired_grip = want
            grip = max(grip-self.gripper_speed*dt, min(grip+self.gripper_speed*dt, want))
            self.previous_grip = grip
        return target, grip


def decode_can_error(cid, data, driver=None):
    """Linux SocketCAN error classes; recovery/lost arbitration are notifications.

    A controller restart still stops a live session: commands may have been lost.
    Never infer that the original failure was harmless from a later recovery.
    """
    mask = cid & 0x1FFFFFFF
    classes = {1: 'TX_TIMEOUT', 2: 'LOST_ARBITRATION', 4: 'CONTROLLER',
               8: 'PROTOCOL', 0x10: 'TRANSCEIVER', 0x20: 'NO_ACK',
               0x40: 'BUS_OFF', 0x80: 'BUS_ERROR', 0x100: 'RESTARTED',
               0x200: 'ERROR_COUNTERS'}
    controller = {1: 'RX_OVERFLOW', 2: 'TX_OVERFLOW', 4: 'RX_WARNING',
                  8: 'TX_WARNING', 0x10: 'RX_PASSIVE', 0x20: 'TX_PASSIVE',
                  0x40: 'RECOVERED_ERROR_ACTIVE'}
    valid = len(data) == 8
    flags = data[1] if valid and mask & 4 else 0
    # Linux v6.8 gs_update_state() maps an unflagged controller notification
    # to ERROR_ACTIVE. candleLight v2.0 src/can.c also puts TEC/REC in bytes
    # 6/7 without CAN_ERR_CNT, e.g. our observed 0000000000005f00 on recovery
    # below the warning threshold (96). Requiring eight zero bytes missed it.
    # Do not accept ACK/protocol errors, reserved bytes or warning-level counts.
    gs_usb_active = (driver == 'gs_usb' and cid == 0x20000004 and valid
                     and data[:6] == bytes(6) and max(data[6:8]) < 96)
    # Only pure arbitration loss and recovery-to-active are informational.
    informative = (valid and bool(mask) and not (mask & ~0x206)
                   and (not (mask & 4) or flags == 0x40)
                   and (not (mask & 0x200) or max(data[6:8]) < 96))
    counters_reported = valid and bool(mask & 0x200)
    return dict(can_id=f'0x{cid:08X}', data_hex=data.hex(),
                driver=driver,
                classes=[name for bit, name in classes.items() if mask & bit],
                unknown_mask=f'0x{mask & ~0x3FF:X}',
                controller=(['GS_USB_ERROR_ACTIVE'] if gs_usb_active else
                            [name for bit, name in controller.items() if flags & bit]),
                protocol_type=data[2] if valid and mask & 8 else None,
                protocol_location=data[3] if valid and mask & 8 else None,
                counters_reported=counters_reported,
                tx_error_counter=data[6] if counters_reported else None,
                rx_error_counter=data[7] if counters_reported else None,
                fatal=not (informative or gs_usb_active))


class Feedback:
    def __init__(self, driver=None):
        self.driver = driver
        self.frames = {}
        self.conflict = None
        self.error = None
        self.can_events = deque(maxlen=20)
        self.last_event_kind = None

    def accept(self, cid, data, stamp):
        if cid & 0xE0000000:
            if cid & 0x20000000:
                event = decode_can_error(cid, data, self.driver)
                event['monotonic'] = stamp
                self.can_events.append(event)
                kind = (cid, tuple(event['controller']), event['fatal'])
                if not self.error and kind != self.last_event_kind:
                    print('[CAN_EVENT] ' + json.dumps(event, ensure_ascii=False), flush=True)
                self.last_event_kind = kind
                if event['fatal'] and not self.error:
                    self.error = 'CAN通信异常：' + json.dumps(event, ensure_ascii=False)
            return
        # Same socket does not receive its own transmitted frames (Linux default).
        if 0x150 <= cid <= 0x159 or cid in (0x470, 0x471):
            self.conflict = f'检测到其他控制端发送0x{cid:X}；禁止争抢控制权。'
        if len(data) == 8:
            self.frames[cid] = (data, stamp)

    def snapshot(self, now, with_gripper=False):
        if self.error or self.conflict:
            raise RuntimeError(self.error or self.conflict)
        needed = REQUIRED + ((0x2A8,) if with_gripper else ())
        missing = [hex(i) for i in needed if i not in self.frames or now-self.frames[i][1] > .25]
        if missing:
            raise TimeoutError(f'反馈缺失或超过250ms：{missing}')
        data = self.frames[0x2A1][0]
        q = [v/1000 for cid in (0x2A5, 0x2A6, 0x2A7)
             for v in struct.unpack('>ii', self.frames[cid][0])]
        motor = [self.frames[cid][0][5] for cid in range(0x261, 0x267)]
        grip_data = self.frames.get(0x2A8, (bytes(8), 0))[0]
        return dict(q=q, grip=int.from_bytes(grip_data[:4], 'big', signed=True)/1000,
                    mode=data[0], status=data[1], move_mode=data[2], teach=data[3],
                    error=int.from_bytes(data[6:8], 'big'),
                    communication_fault_joints=[i+1 for i in range(6) if data[7] & (1 << i)],
                    angle_fault_joints=[i+1 for i in range(6) if data[6] & (1 << i)],
                    enabled=[bool(v & 0x40) for v in motor],
                    motor_fault=[v & 0xBF for v in motor],
                    gripper_effort_nm=int.from_bytes(grip_data[4:6], 'big', signed=True)/1000,
                    gripper_enabled=bool(grip_data[6] & 0x40), gripper_fault=grip_data[6] & 0x3F)


def check_ready(state, gripper=False):
    if state['mode'] != 1 or state['move_mode'] != 1 or state['teach'] in (1, 3, 4, 5, 7):
        raise RuntimeError('实控需PiPER处于CAN/MOVE J模式、退出示教。先按文档完成厂家模式准备。')
    if state['status'] != 0 or state['error'] or any(state['motor_fault']):
        raise RuntimeError(f'PiPER异常状态：{state}')
    if not all(state['enabled']):
        raise RuntimeError('PiPER六关节尚未全部使能；本入口不自动上力矩。')
    if gripper and (not state['gripper_enabled'] or state['gripper_fault']):
        raise RuntimeError('夹爪未使能或有故障。')
    finite(state['q'])


def needs_reset(state):
    return not (state['mode'] in (0, 1) and state['status'] == 0 and
                state['teach'] in (0, 2, 6) and not any(state['enabled']))


def check_prepare(state):
    finite(state['q'])
    if state['mode'] not in (0, 1, 2) or state['teach'] in (1, 3, 4, 5, 7):
        raise RuntimeError('先在现场结束录制/回放；不在运行中切换模式。')
    if state['status'] not in (0, 1) or state['error'] or any(state['motor_fault']):
        raise RuntimeError('PiPER有故障，不能用模式准备掩盖故障。')
    q = state['q']
    if not needs_reset(state):
        hold_target(q)  # already disabled: only legal hold-target displacement matters
        return
    # Official playTrajectory_new.py only constrains gravity-loaded J2/J3/J5.
    # Retain the previously supported near-zero posture for those axes too.
    near_zero = all(abs(q[i]) <= 3 for i in (1, 2, 4))
    tutorial_pose = (abs(math.radians(q[1])) < .1745 and
                     abs(math.radians(q[2])) < .1745 and
                     .2094 < math.radians(q[4]) < .7854)
    if not (near_zero or tutorial_pose):
        raise RuntimeError('复位前需托稳：J2/J3/J5均在±3度，或满足厂家示例'
                           ' |J2|、|J3|<约10度且12<J5<45度；J1/J4/J6不要求回零。')


class PiperCAN:
    def __init__(self, interface):
        # Verify the existing configuration; never reconfigure the interface.
        self.interface = interface
        self.initial_link = link = can_link_info(interface)
        info = link.get('linkinfo', {}).get('info_data', {})
        if 'UP' not in link.get('flags', []) or info.get('bittiming', {}).get('bitrate') != 1000000:
            raise RuntimeError(f'{interface}必须已UP且为1000000 bit/s；当前：{link}')
        self.bus = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
        self.bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER, struct.pack('=I', 0x1FFFFFFF))
        self.bus.bind((interface,))
        self.bus.settimeout(.05)
        try:
            driver = (Path('/sys/class/net') / interface / 'device/driver').resolve(strict=True).name
        except OSError:
            driver = None
        self.feedback = Feedback(driver=driver)
        self.lock = threading.RLock()
        self.done = threading.Event()
        self.live = False
        self.stopped = False
        self.reason = None
        self.heartbeat = time.monotonic()
        self.reset_until = 0.
        self.tx_count = 0
        self.thread = threading.Thread(target=self._receive, daemon=True)
        self.thread.start()

    def _receive(self):
        while not self.done.is_set():
            try:
                raw = self.bus.recv(16)
                cid, n, data = struct.unpack('=IB3x8s', raw)
                with self.lock:
                    self.feedback.accept(cid, data[:n], time.monotonic())
            except socket.timeout:
                pass
            except (OSError, struct.error) as exc:
                with self.lock:
                    self.feedback.error = self.feedback.error or f'CAN接收失败：{exc}'
            with self.lock:
                if self.live and not self.stopped:
                    try:
                        try:
                            self.feedback.snapshot(time.monotonic(), self.with_gripper)
                        except TimeoutError:
                            if time.monotonic() >= getattr(self, 'reset_until', 0.):
                                raise
                        if time.monotonic() - self.heartbeat > .25:
                            raise TimeoutError('控制循环/Leader读取超过250ms未更新')
                    except Exception as exc:
                        self.stop(str(exc))

    def state(self, with_gripper=False):
        with self.lock:
            return self.feedback.snapshot(time.monotonic(), with_gripper)

    def wait_state(self, timeout=2.):
        deadline = time.monotonic() + timeout
        while True:
            try:
                return self.state()
            except TimeoutError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(.01)

    def _send(self, cid, data):
        self.bus.send(struct.pack('=IB3x8s', cid, len(data), data))
        self.tx_count += 1

    def arm(self, with_gripper, speed_percent=10, control_mode='joint', gripper_effort=.5,
            target_speed=60.):
        if control_mode not in ('joint', 'servo'):
            raise ValueError('control_mode必须为joint或servo')
        gripper_frame(0., gripper_effort)  # validate before any actuation
        finite([target_speed])
        if not 0 < target_speed <= 120:
            raise ValueError('target_speed须为(0,120]度/秒')
        with self.lock:
            check_ready(self.state(with_gripper), with_gripper)
            self.with_gripper = with_gripper
            self.speed_percent = speed_percent
            self.control_mode = control_mode
            self.gripper_effort = gripper_effort
            self.mode_configured = False
            self.tracking = TrackingMonitor(target_speed)
            self.heartbeat = time.monotonic()
            self.live = True

    def command(self, q, grip, with_gripper):
        frames = joint_frames(q)
        grip_frame = gripper_frame(grip, self.gripper_effort) if with_gripper else None
        with self.lock:
            if not self.live or self.stopped:
                raise RuntimeError(self.reason or '未ARM；禁止CAN发送。')
            try:
                state = self.state(with_gripper)
                check_ready(state, with_gripper)
                # Covers first command, pause and re-anchor too. A fast moving
                # target can legitimately be >5 degrees ahead of the motor.
                self.tracking.check(q, state['q'], time.monotonic())
                # JS = MOVE J + 0xAD (official move_js), not per-motor torque control.
                # Preload the fresh anchor before first entering fast-follow mode.
                if self.control_mode == 'servo' and not self.mode_configured:
                    for cid, payload in frames:
                        self._send(cid, payload)
                mit = 0xAD if self.control_mode == 'servo' else 0
                self._send(0x151, bytes([1, 1, self.speed_percent, mit, 0, 0, 0, 0]))
                for cid, payload in frames:
                    self._send(cid, payload)
                if with_gripper:
                    # Contact while grasping legitimately prevents full closure.
                    # Enforce effort, hardware fault and feedback checks, not an
                    # aperture-error stop that would abort every thick-object grasp.
                    self._send(*grip_frame)
                self.mode_configured = True
                self.heartbeat = time.monotonic()
                self.tracking.sent(q, self.heartbeat)
            except Exception as exc:
                self.stop(str(exc))
                raise

    def prepare_can(self):
        """Supported-pose transition based on the manufacturer's reset sequence.

        Reset releases torque. This is NEVER part of preview or normal live startup.
        Caller must obtain the in-person ARM confirmation first.
        """
        initial = self.state()
        try:
            check_ready(initial)
        except (RuntimeError, ValueError):
            pass
        else:
            print('已处于CAN/MOVE J且六关节已使能，无需复位；CAN零发送。', flush=True)
            return
        check_prepare(initial)
        with self.lock:
            self.with_gripper = False
            self.heartbeat = time.monotonic()
            self.live = True

        def send(cid, data):
            with self.lock:
                if self.stopped:
                    raise RuntimeError(self.reason)
                self._send(cid, data)
                self.heartbeat = time.monotonic()

        def wait_for(predicate, *, before_reset=False, target=None, resetting=False, repeat=None):
            started = time.monotonic()
            stable_since = None
            last_repeat = started
            while time.monotonic() - started < 3.:
                with self.lock:
                    if self.stopped:
                        raise RuntimeError(self.reason)
                    now = time.monotonic()
                    self.heartbeat = now
                    settling = resetting and now < self.reset_until
                    try:
                        state = self.state()
                    except TimeoutError:
                        if not settling:
                            raise
                        state = None
                # Only a documented reset interval can tolerate missing feedback
                # or communication-only status. Never enable/send targets here.
                comm_only = state is not None and state['status'] in (0, 1, 5) and not (state['error'] & ~0x3F) and not any(state['motor_fault'])
                if settling and (state is None or comm_only):
                    stable_since = None
                    time.sleep(.02)
                    continue
                if state['error'] or any(state['motor_fault']) or state['status'] not in (0, 1):
                    raise RuntimeError(f'模式准备期间出现故障：{state}')
                if state['teach'] in (1, 3, 4, 5, 7):
                    raise RuntimeError('模式准备期间检测到录制/回放；停止。')
                if before_reset:
                    check_prepare(state)
                else:
                    finite(state['q'])
                if target is not None:
                    check_tracking(target, state['q'])
                self.heartbeat = time.monotonic()
                if predicate(state):
                    if stable_since is None:
                        stable_since = time.monotonic()
                    if time.monotonic() - stable_since >= .12:
                        return state
                else:
                    stable_since = None
                    if repeat and time.monotonic() - last_repeat >= .1:
                        send(*repeat)
                        last_repeat = time.monotonic()
                time.sleep(.02)
            raise TimeoutError(f'模式准备3秒内未得到预期反馈；最后状态：{state}；停止。')

        phase = '停止并检查支撑姿态（尚未复位）'
        try:
            if needs_reset(initial):
                print(f'[PREPARE] {phase}', flush=True)
                send(0x150, bytes([1, 0, 0, 0, 0, 0, 0, 0]))
                wait_for(lambda s: True, before_reset=True)
                phase = '复位，等待1秒恢复窗口及六关节失能'
                print(f'[PREPARE] {phase}', flush=True)
                with self.lock:
                    self.reset_until = time.monotonic() + RESET_SETTLE_SECONDS
                    send(0x150, bytes([2, 0, 0, 0, 0, 0, 0, 0]))
                state = wait_for(lambda s: s['status'] == 0 and not any(s['enabled']), resetting=True)
            else:
                print('[PREPARE] 已复位且六关节失能，跳过停止/复位。', flush=True)
            phase = '失能状态切换待机与CAN/MOVE J'
            print(f'[PREPARE] {phase}', flush=True)
            send(0x151, bytes([0, 1, 10, 0, 0, 0, 0, 0]))
            wait_for(lambda s: s['mode'] == 0 and not any(s['enabled']),
                     repeat=(0x151, bytes([0, 1, 10, 0, 0, 0, 0, 0])))
            send(0x151, bytes([1, 1, 10, 0, 0, 0, 0, 0]))
            # Official startup waits for CAN ctrl_mode before enabling; move_mode
            # may still reflect the previous executed motion while disabled.
            state = wait_for(lambda s: s['mode'] == 1 and not any(s['enabled']),
                             repeat=(0x151, bytes([1, 1, 10, 0, 0, 0, 0, 0])))
            phase = '按复位后实测姿态预装合法目标并使能'
            print(f'[PREPARE] {phase}', flush=True)
            target = announce_hold(state['q'])
            for cid, data in joint_frames(target):
                send(cid, data)  # preload measured pose while motors are disabled
            send(0x471, bytes([7, 2, 0, 0, 0, 0, 0, 0]))
            state = wait_for(lambda s: all(s['enabled']) and s['status'] == 0, target=target,
                             repeat=(0x471, bytes([7, 2, 0, 0, 0, 0, 0, 0])))
            check_ready(state)
        except BaseException:
            self.stop(f'CAN模式准备失败，阶段：{phase}')
            raise
        with self.lock:
            self.live = False  # hold the preloaded pose; do not disable on close
        print('CAN/MOVE J准备完成，六关节已使能；夹爪未改动。此后启动--live。', flush=True)

    def stop(self, reason):
        with self.lock:
            if not self.live or self.stopped:
                return
            self.stopped, self.reason = True, reason
            # A gripper now participates in normal teleop. Cancel its outstanding
            # open/close target on exit when fresh, healthy feedback is available;
            # keep holding an object rather than disabling/opening the fingers.
            if self.with_gripper:
                try:
                    state = self.state(True)
                    if not state['gripper_fault'] and 0 <= state['grip'] <= 70:
                        self._send(*gripper_frame(state['grip'], self.gripper_effort))
                except (OSError, RuntimeError, ValueError):
                    pass  # still attempt the arm stop below if feedback/send failed
            try:
                # Official fast stop. Never send reset(0x02) or disable torque.
                self._send(0x150, bytes([1, 0, 0, 0, 0, 0, 0, 0]))
            except OSError as exc:
                print(f'停止指令发送失败，请操作实体急停：{exc}', file=sys.stderr, flush=True)
            print(f'[STOP] {reason}；不自动恢复。', flush=True)

    def close(self):
        self.done.set()
        self.thread.join(timeout=.3)
        self.bus.close()


class LeaderReader:
    def __init__(self, port, calibration_dir, leader_id='so101_leader'):
        # PyPI LeRobot 0.4.3 has a processor/teleoperator import cycle; initialize
        # the processor exports first, as its own CLI does. No devices are opened.
        import lerobot.processor  # noqa: F401
        try:
            from lerobot.teleoperators.so_leader.config_so_leader import SO101LeaderConfig
            from lerobot.teleoperators.so_leader.so_leader import SO101Leader
        except ModuleNotFoundError as exc:
            # The deployed XLeRobot checkout predates upstream's so_leader rename.
            # Only fall back for that missing package, never for missing dependencies.
            if exc.name != 'lerobot.teleoperators.so_leader':
                raise
            from lerobot.teleoperators.so101_leader.config_so101_leader import SO101LeaderConfig
            from lerobot.teleoperators.so101_leader.so101_leader import SO101Leader
        self.leader = SO101Leader(SO101LeaderConfig(id=leader_id, port=port,
                               calibration_dir=calibration_dir, use_degrees=True))
        self.bus = self.leader.bus
        if not self.leader.calibration:
            raise FileNotFoundError(f'缺少Leader标定：{self.leader.calibration_fpath}；请先用LeRobot标定，并核对--leader-id/--calibration-dir。')

    @property
    def wrist_range(self):
        # Match MotorsBus._normalize(DEGREES): midpoint of recorded raw limits,
        # scaled by encoder resolution-1. Read existing calibration, never rewrite.
        calibration = self.leader.calibration['wrist_flex']
        resolution = self.bus.model_resolution_table[self.bus.motors['wrist_flex'].model]-1
        half_range = (calibration.range_max-calibration.range_min)*180/resolution
        return (-half_range, half_range)

    def connect(self):
        # Match the project's read-only reader, not SO101Leader.connect() which writes.
        self.bus.connect()
        if not self.bus.is_calibrated:
            raise RuntimeError('Leader已有标定与电机不一致；不自动写标定。')
        torque = self.bus.sync_read('Torque_Enable', normalize=False)
        if any(torque.values()):
            raise RuntimeError('Leader仍有力矩，不要强行拖动；先使用原项目方式释放。')

    def read(self):
        values = self.bus.sync_read('Present_Position', num_retry=0)
        result = [float(values[n]) for n in NAMES]
        finite(result)
        return result

    def close(self):
        if self.bus.is_connected:
                self.bus.disconnect(disable_torque=False)


@contextlib.contextmanager
def device_locks(interface, port):
    """Block other instances and an already open serial owner before connecting."""
    with contextlib.ExitStack() as stack:
        devices = (interface, str(Path(port).resolve())) if port is not None else (interface,)
        for device in devices:
            name = device.replace('/', '_')
            path = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp')) / f'piper-local-{os.getuid()}-{name}.lock'
            handle = stack.enter_context(path.open('a'))
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if port is not None:
            busy = subprocess.run(['fuser', str(Path(port).resolve())], capture_output=True, text=True)
            if busy.returncode == 0:
                raise RuntimeError(f'Leader串口已被其他进程占用：{busy.stdout.strip()}')
            if busy.returncode != 1:
                raise RuntimeError(f'无法检查串口占用：{busy.stderr.strip()}')
        yield


@contextlib.contextmanager
def keyboard():
    if not sys.stdin.isatty():
        raise RuntimeError('实控需要现场交互终端。')
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield lambda: os.read(fd, 1).decode('ascii', errors='ignore') if select.select([fd], [], [], 0)[0] else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def run(args, reader, robot):
    def make_mapper(leader, q, grip):
        return Mapper(leader, q, grip, args.gain, args.signs, args.speed, args.gripper, args.max_offset,
                      acceleration=args.accel, gripper_speed=args.gripper_speed,
                      gripper_open_mm=args.gripper_open_mm,
                      wrist_range=reader.wrist_range if args.wrist_mapping == 'range' else None)
    reader.connect()
    state = robot.wait_state()
    leader = reader.read()
    try:
        preview_anchor = hold_target(state['q'])
        hold_error = None
    except (RuntimeError, ValueError) as exc:
        preview_anchor, hold_error = state['q'], str(exc)
    mapper = make_mapper(leader, preview_anchor, state['grip'])
    print(json.dumps({'mode': 'ARM_WAIT' if args.live else 'PREVIEW', 'piper': state, 'leader_deg_and_gripper_pct': leader,
                      'startup_hold_deg': None if hold_error else preview_anchor, 'hold_error': hold_error,
                      'gripper_control': args.gripper, 'control_mode': args.control_mode,
                      'joint_signs': args.signs,
                      'wrist_mapping': args.wrist_mapping, 'leader_wrist_range_deg': reader.wrist_range,
                      'mapping': f'J1/J2/J3/J6按角度增量；J5腕俯仰={args.wrist_mapping}；J4固定；夹爪按开口百分比'}, ensure_ascii=False), flush=True)
    if not args.live and hold_error:
        print(f'预览不生成目标：{hold_error}；实测值保持原样。', flush=True)
    if args.live:
        check_ready(state, args.gripper)
        hold_target(state['q'])
        if not sys.stdin.isatty():
            raise RuntimeError('需现场终端亲自输入ARM；拒绝管道确认。')
        print(f'跟随模式={args.control_mode}，目标限速={args.speed:g}°/s，'
              f'夹爪={"开启" if args.gripper else "关闭"}，更新={args.hz:g}Hz，'
              f'映射方向={args.signs}。', flush=True)
        if input('确认运动范围和夹爪间隙清空、可实体急停。输入 ARM 开始跟随：').strip() != 'ARM':
            return
        # Fresh readings after the operator prompt: no stale anchor.
        state = robot.state(args.gripper)
        leader = reader.read()
        target = announce_hold(state['q'])
        mapper = make_mapper(leader, target, state['grip'])
        robot.arm(args.gripper, args.can_speed, args.control_mode, args.gripper_effort, args.speed)
        robot.command(target, state['grip'], args.gripper)
        offset_text = f'额外相对范围±{args.max_offset:g}度' if args.max_offset else '按设备关节范围运行'
        print(f'LIVE：{args.control_mode}，增益{args.gain:g}，目标上限{args.speed:g}度/秒，'
              f'加速度{args.accel:g}度/秒²，{args.hz:g}Hz，{offset_text}，腕映射={args.wrist_mapping}。'
              f'夹爪{"开启" if args.gripper else "关闭"}（{args.gripper_speed:g}mm/s，'
              f'{args.gripper_effort:g}N·m）。空格暂停；C重新锚定；Q/Ctrl+C停止。', flush=True)
    start = last = time.monotonic()
    next_print = 0.
    paused = False
    hold = None
    cycle_times = deque(maxlen=100)
    keys = keyboard() if args.live else contextlib.nullcontext(lambda: '')
    try:
        with keys as read_key:
            while args.seconds == 0 or time.monotonic()-start < args.seconds:
                if robot.stopped:
                    raise RuntimeError(robot.reason)
                cycle = time.monotonic()
                leader = reader.read()
                leader_read_ms = (time.monotonic()-cycle)*1000
                state = robot.state(args.gripper)
                sample_time = time.monotonic()
                key = read_key().lower()
                if key == 'q':
                    break
                if key == ' ':
                    paused = True
                    hold = (hold_target(state['q']) if args.live else state['q'], state['grip'])
                    print('已暂停，C重新锚定继续。', flush=True)
                if key == 'c':
                    anchor = hold_target(state['q']) if args.live else state['q']
                    mapper = make_mapper(leader, anchor, state['grip'])
                    paused = False
                if not args.live and not in_limits(mapper.anchor):
                    target, grip = None, state['grip']
                else:
                    target, grip = hold if paused else mapper.step(
                        leader, state['q'], state['grip'], sample_time-last)
                cycle_times.append(sample_time-last)
                last = sample_time
                if args.live:
                    robot.command(target, grip, args.gripper)
                if cycle >= next_print:
                    print(json.dumps({'mode': 'PAUSED' if paused else ('LIVE' if args.live else 'PREVIEW'),
                                      'leader': [round(v, 2) for v in leader],
                                      'actual_deg': state['q'], 'target_deg': None if target is None else [round(v, 3) for v in target],
                                      'tracking_error_deg': None if target is None else [round(a-b, 3) for a, b in zip(target, state['q'])],
                                      'target_gripper_mm': round(grip, 2), 'can_tx': robot.tx_count,
                                      'actual_gripper_mm': state['grip'], 'gripper_control': args.gripper,
                                      'desired_gripper_mm': round(mapper.desired_grip, 2),
                                      'desired_deg': [round(v, 3) for v in mapper.desired],
                                      'limited_joints': mapper.limited_joints,
                                      'offset_limited_joints': mapper.offset_limited_joints,
                                      'joint_limited_joints': mapper.joint_limited_joints,
                                      'rate_limited_joints': mapper.rate_limited_joints,
                                      'loop_hz': round(len(cycle_times)/sum(cycle_times), 1),
                                      'leader_read_ms': round(leader_read_ms, 2),
                                      'control_mode': args.control_mode,
                                      'piper_mode': state['mode'], 'teaching_state': state['teach']}, ensure_ascii=False), flush=True)
                    next_print = cycle + .5
                time.sleep(max(0, 1/args.hz-(time.monotonic()-cycle)))
    finally:
        if args.live:
            # Fast stop even on Q/Ctrl+C; no torque-disable or reset on exit.
            error = sys.exc_info()[0]
            reason = '用户退出或运行时长结束'
            if error is KeyboardInterrupt:
                reason = '用户Ctrl+C退出'
            elif error is not None:
                reason = '控制异常退出'
            robot.stop(reason)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--live', action='store_true', help='实际跟随；必须现场ARM，默认仅预览')
    modes.add_argument('--prepare-can', action='store_true', help='自动模式初始化：检查J2/J3/J5支撑姿态，托稳并ARM后复位、转CAN、预装当前位置再使能；已就绪则零发送')
    modes.add_argument('--diagnose-can', action='store_true', help='独立监听CAN并输出反馈及接口计数；零发送，不连接SO101；默认10秒')
    parser.add_argument('--can', default='can0')
    parser.add_argument('--leader-port', default=PORT, help='SO101串口；推荐/dev/serial/by-id/...；可用PIPER_LEADER_PORT设置默认值')
    parser.add_argument('--leader-id', default='so101_leader', help='已有LeRobot标定的设备ID（JSON文件名去掉.json）')
    parser.add_argument('--calibration-dir', type=Path, default=None, help='已有标定JSON的目录；省略则使用已安装LeRobot的默认目录')
    parser.add_argument('--seconds', type=float, default=10., help='运行秒数；0持续至退出')
    parser.add_argument('--gain', type=float, default=1., help='Leader角度增量比例，(0,1]')
    parser.add_argument('--speed', type=float, default=60., help='各关节目标最大度/秒，(0,120]')
    parser.add_argument('--accel', type=float, default=240., help='目标起停加速度，度/秒²，(0,720]')
    parser.add_argument('--hz', type=float, default=100., help='控制更新频率，10–200Hz')
    parser.add_argument('--control-mode', choices=('joint', 'servo'), default='joint', help='joint位置速度；servo厂家JS连续跟随（软件限速和平滑仍生效）')
    parser.add_argument('--can-speed', type=int, default=100, help='位置速度模式比例1–100%%；servo响应由软件轨迹约束')
    parser.add_argument('--max-offset', type=float, default=0., help='额外相对角度范围；默认0不额外截断，按设备关节限位；可设(0,180]度')
    parser.add_argument('--wrist-mapping', choices=('range', 'relative'), default='range', help='range按已有标定分配腕部两侧剩余行程；relative保留原角度增量映射')
    parser.add_argument('--signs', default='-1,1,1,1,1', help='SO101肩部左右旋转默认反向映射J1；五轴方向逗号分隔±1，如--signs=-1,1,1,1,1')
    parser.add_argument('--gripper', action=argparse.BooleanOptionalAction, default=True, help='默认控制夹爪；--no-gripper关闭')
    parser.add_argument('--gripper-speed', type=float, default=50., help='夹爪目标变化上限，mm/s，(0,100]')
    parser.add_argument('--gripper-open-mm', type=float, default=70., help='Leader全开对应PiPER开口，mm，(0,70]')
    parser.add_argument('--gripper-effort', type=float, default=.5, help='夹爪力矩上限，N·m，(0,5]')
    args = parser.parse_args(argv)
    try:
        args.signs = tuple(int(s) for s in args.signs.split(','))
        finite([args.seconds, args.gain, args.speed, args.max_offset, args.accel, args.hz,
                args.gripper_speed, args.gripper_open_mm, args.gripper_effort])
        if len(args.signs) != 5 or any(s not in (-1, 1) for s in args.signs):
            raise ValueError('signs须为5个±1')
        if args.seconds < 0 or not 0 < args.gain <= 1 or not 0 < args.speed <= 120:
            raise ValueError('seconds>=0, 0<gain<=1, 0<speed<=120')
        if args.diagnose_can and args.seconds == 0:
            raise ValueError('--diagnose-can需--seconds>0，以便输出完整诊断结果')
        if not 0 < args.accel <= 720 or not 10 <= args.hz <= 200:
            raise ValueError('0<accel<=720, 10<=hz<=200')
        if not 0 < args.gripper_speed <= 100 or not 0 < args.gripper_open_mm <= 70 or not 0 < args.gripper_effort <= 5:
            raise ValueError('0<gripper-speed<=100, 0<gripper-open-mm<=70, 0<gripper-effort<=5')
        if not 1 <= args.can_speed <= 100 or not 0 <= args.max_offset <= 180:
            raise ValueError('1<=can-speed<=100, 0<=max-offset<=180')
    except ValueError as exc:
        parser.error(str(exc))
    return args


def main(argv=None):
    args = parse_args(argv)
    with device_locks(args.can, None if args.diagnose_can or args.prepare_can else args.leader_port):
        reader = None
        robot = None
        try:
            robot = PiperCAN(args.can)
            if args.diagnose_can:
                print('CAN独立监听：不连接SO101，不发送查询/控制帧。', flush=True)
                deadline = time.monotonic()+args.seconds
                robot.wait_state(timeout=min(2., args.seconds))
                while time.monotonic() < deadline:
                    robot.state(args.gripper)  # after startup, report gaps instead of waiting them away
                    time.sleep(min(.05, max(0., deadline-time.monotonic())))
                print('[CAN_DIAG] '+json.dumps(can_diagnostics(args.can, robot), ensure_ascii=False), flush=True)
                return
            if args.prepare_can:
                state = robot.wait_state()
                print(json.dumps(state, ensure_ascii=False), flush=True)
                try:
                    check_ready(state)
                except (RuntimeError, ValueError):
                    pass
                else:
                    print('已处于CAN/MOVE J且六关节已使能，无需复位；可启动--live。', flush=True)
                    return
                check_prepare(state)
                if not sys.stdin.isatty():
                    raise RuntimeError('模式准备需要现场交互终端。')
                if needs_reset(state):
                    print('注意：厂家复位会短暂卸力！请托稳，J1/J4/J6无需回零。'
                          '本步骤将重新使能六关节；不操作夹爪。', flush=True)
                else:
                    print('当前已复位且六关节失能，将跳过停止/复位，预装当前位置目标后使能。'
                          '请托稳；不操作夹爪。', flush=True)
                if input('现场确认后输入 ARM；其他输入取消：').strip() == 'ARM':
                    robot.prepare_can()
            else:
                reader = LeaderReader(args.leader_port, args.calibration_dir, args.leader_id)
                run(args, reader, robot)
        except Exception as exc:
            if robot:
                robot.stop(str(exc))
            record_failure(args.can, robot, exc)
            raise
        finally:
            if robot:
                robot.stop('程序退出')
                print(f'本进程CAN发送总数：{robot.tx_count}', flush=True)
                robot.close()
            if reader:
                reader.close()


def cli():
    """Installed command and python -m share signal/cleanup/error handling."""
    def terminate(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    try:
        main()
    except KeyboardInterrupt:
        print('已退出。')
    except Exception as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    cli()
