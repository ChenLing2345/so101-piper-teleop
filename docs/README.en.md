# SO-101 → PiPER local teleoperation

A Linux bridge from an original SO-101 leader to an AgileX PiPER arm and gripper. It reads the leader over USB serial through LeRobot and sends protocol-v2 commands over SocketCAN. ROS, network relays and teaching-mode recording are not required.

The detailed [Chinese README](../README.md) covers installation, calibration, mapping, every CLI option and troubleshooting. [Protocol notes](PROTOCOL.md) and [validation scope](VALIDATION.md) separate measured results from limitations.

## Hardware scope

The deployed motion core was tested on one PiPER reporting firmware S-V1.8-4, an original SO-101 leader and a gs_usb/candleLight CAN adapter at 1 Mbps. Linux SocketCAN is required. There is no collision planning or Cartesian pose retargeting. Verify your unit's limits before porting: the tested J6 range is ±180°, which must not be assumed for every PiPER model.

This bridge controls five arm joints: leader shoulder pan/lift, elbow, wrist pitch/roll map to PiPER J1/J2/J3/J5/J6. J4 stays at the takeover angle. Gripper aperture is controlled separately. The arms do not need identical starting joint angles.

## Installation

Clone the repository and enter its directory:

```bash
git clone https://github.com/ChenLing2345/so101-piper-teleop.git
cd so101-piper-teleop
```

Then:

```bash
sudo apt-get install -y python3-venv iproute2 psmisc can-utils
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e '.[leader]'
piper-teleop --help
```

The optional `leader` extra installs `lerobot[feetech]==0.4.3` and its dependencies. If your environment already has a compatible LeRobot/Feetech installation, `pip install -e .` installs only this bridge. Pure CAN diagnostics and offline tests do not need LeRobot or a GPU.

## Leader calibration

Find the serial port with `ls -l /dev/serial/by-id/`. Prefer a persistent by-id path. The examples below use `/dev/ttyACM0`; replace it with your actual leader port.

```bash
export PIPER_LEADER_PORT=/dev/ttyACM0
```

Use your own existing calibration, or follow the [official SO-101 setup instructions](https://huggingface.co/docs/lerobot/so101). Initial calibration through LeRobot writes leader calibration values:

```bash
lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port="$PIPER_LEADER_PORT" \
  --teleop.id=so101_leader
```

Use the same ID with `--leader-id`. `--calibration-dir` may point to an existing calibration directory; when omitted, LeRobot selects its own default directory. The PyPI release uses the `so_leader` implementation while the deployed checkout used `so101_leader`; both import layouts are supported. The bridge reads and checks calibration, and does not rewrite it or release leader torque automatically.

## CAN and startup

With all controllers stopped, configure the correct CAN interface:

```bash
sudo ip link set can0 up type can bitrate 1000000
piper-teleop --diagnose-can --seconds 10
piper-teleop --seconds 10 --leader-port "$PIPER_LEADER_PORT"
```

Diagnostics and preview transmit zero CAN frames. Diagnostics do not connect to the leader.

Prepare the arm explicitly, then run a short initial test:

```bash
piper-teleop --prepare-can &&
piper-teleop --live --seconds 10 --speed 15 --accel 60 \
  --leader-port "$PIPER_LEADER_PORT"
```

**The manufacturer's reset temporarily releases torque: physically support the arm.** Preparation checks the supported reset posture, preloads a legal target near the measured pose, then enables the joints. It does not move to an arbitrary zero pose or operate the gripper. Preparation and live takeover require an operator to type `ARM` in a real terminal. Keep the workspace clear and have access to the hardware emergency stop.

After checking directions and your workspace, continuous operation is:

```bash
piper-teleop --prepare-can &&
piper-teleop --live --seconds 0 --leader-port "$PIPER_LEADER_PORT" \
  --leader-id so101_leader --control-mode joint \
  --gain 1 --signs=-1,1,1,1,1 \
  --speed 60 --accel 240 --hz 100 --can-speed 100 \
  --max-offset 0 --wrist-mapping range \
  --gripper --gripper-speed 50 --gripper-effort 0.5
```

Space pauses with a hold target; C reanchors and resumes; Q or Ctrl+C stops and exits. A software stop depends on functioning communication and is not a substitute for a physical emergency stop. There is no automatic fault recovery or replay of previous targets.

## Mapping and tuning

- J1/J2/J3/J6 use relative joint increments from the current takeover posture; the default J1 sign is reversed for the tested installation.
- J5 uses the leader's calibrated remaining wrist travel on each side of the takeover point and maps it to PiPER's remaining J5 travel. `--wrist-mapping relative` selects the older angular-increment behavior.
- `--max-offset 0` removes the additional ±90° software window used in early debugging; real joint limits remain enforced.
- Gripper input 0–100% maps to 0–70 mm by default. It is enabled unless `--no-gripper` is passed. `--gripper-open-mm` can reduce the opening. Effort is a protocol torque parameter, not a calibrated fingertip-force guarantee.
- Default target limits are 60°/s and 240°/s² at 100 Hz. These are software limits, not guaranteed physical performance under all loads.
- `desired_deg`, `target_deg` and `actual_deg` distinguish requested motion, smoothed commands and measured feedback. `loop_hz` and limit-reason fields aid diagnosis.

## Communication faults

An actual `NO_ACK`, bus-off, overflow or missing feedback still stops the run. Known gs_usb ERROR-ACTIVE notifications are informational, including the legacy payload `0000000000005f00`; recovery never clears a previously latched fault.

Fault snapshots are saved under `${XDG_STATE_HOME:-$HOME/.local/state}/so101-piper-teleop/failures/`. They include the original error, frame ages and interface counter changes. The earlier intermittent NO_ACK's physical cause remains unresolved; short successful tests do not prove long-duration reliability.

## Development and license

```bash
python -m pip install -e '.[test]'
python -m pytest -q
python -m build
```

Offline tests do not open hardware. The public package adds portable paths and configurable leader IDs to the deployed motion core. See [VALIDATION.md](VALIDATION.md) for the distinction between past physical trials and release packaging tests.

Apache-2.0; see [LICENSE](../LICENSE) and [NOTICE](../NOTICE). LeRobot and manufacturer references retain their own licenses. This is an independent integration, not an official AgileX product. Calibration files, device identifiers and private logs are not bundled.
