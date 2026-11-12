#!/usr/bin/env python3
"""Standalone Piper teleoperation with gripper-only damping.

Run: python piper_gripper_feedback.py --feedback-enable
Without the flag, teleoperation continues but damping is not applied.
All seven axes follow the leader; only leader servo 6 receives damping.
No URDF or global HOLD. Do not run another controller on the same port.
Enable the follower beforehand. Hardware feel remains uncalibrated."""
import argparse
from dataclasses import dataclass
import math
import struct
import sys
import time

import can
import fashionstar_uart_sdk as uservo
from piper_sdk import C_PiperInterface_V2
import serial

# 1. Parameters


SOURCE_AGE_S = .15  # Clear damping when measurements are older than 150 ms.
GAP_S = .25  # Reset motion estimation after an excessive update/sample gap (s).
CAN_TIMEOUT_S = .25  # Reject CAN feedback older than this (s).
LEADER_TIMEOUT_S = .25  # Exit after a continuous leader gap (s).
STARTUP_TIMEOUT_S = 1.  # Maximum startup feedback wait (s).
GRIPPER_RECOVERY_S = .10  # Extra wait for an isolated gripper feedback gap (s).
MAX_TEMPERATURE_C = 80.  # Stop on leader/follower temperatures at or above this (C).
DEFAULT_POWER_MW = 1500  # Closing damping cap (mW), preserved from the current file.
DEFAULT_RATE_HZ = 30.  # Target loop frequency (Hz), not a real-time guarantee.
GRIPPER_PROFILE = 'smooth-directional-v2 / gripper-only'  # Algorithm identifier printed at startup.
GRIPPER_BASE_POWER_MW = 20  # Baseline damping in both directions (mW).
GRIPPER_DEADZONE_NM = .15  # Static load deadzone (Nm).
GRIPPER_FULL_SCALE_NM = .30  # Excess torque for full closing output (Nm).
GRIPPER_RISE_MW_S = 3500.  # Maximum power rise (mW/s).
GRIPPER_FALL_MW_S = 8000.  # Maximum power fall (mW/s).
GRIPPER_OPEN_RATIO = .2  # Opening fraction of the load-dependent increment.
GRIPPER_OPEN_MAX_POWER_MW = 240  # Opening target cap (mW).
GRIPPER_OPEN_STEP_DEG = .2  # Minimum accumulated travel to confirm direction (deg).
GRIPPER_DIRECTION_CONFIRM_S = .06  # Minimum same-direction duration (s).
GRIPPER_DIRECTION_SAMPLES = 3  # Minimum same-direction moving samples.
GRIPPER_MOTION_EPS_DEG = .02  # Ignore smaller per-sample angle changes (deg).
GRIPPER_DIRECTION_GAP_S = .10  # Discard pending direction after this motion gap (s).
GRIPPER_TORQUE_FILTER_S = .03  # Torque low-pass time constant (s).
GRIPPER_CONTACT_CONFIRM_S = .02  # Continuous excess confirmation (s), not collision detection.
GRIPPER_POWER_FILTER_S = .03  # Output power low-pass time constant (s).

JOINT_SIGNS = [-1., 1., 1., -1., 1., -1.]  # J1..J6 leader-to-follower direction mapping.

JOINT_LIMITS_DEG = [(-150., 150.), (0., 180.), (-170., 0.),
                    (-100., 100.), (-70., 70.), (-120., 120.)]


# 2. Gripper feedback calculation
def lowpass(previous, measured, dt, time_constant):
    """Smooth a measurement using dt and a time constant, both in seconds."""
    return previous + dt / (time_constant + dt) * (measured - previous)


def map_power(torque, deadzone, full_scale, max_power):
    """Map torque above the deadzone (Nm) to a bounded damping increment (mW)."""

    ratio = min(1., max(0., abs(torque) - deadzone) / full_scale)
    return max_power * ratio


class GripperFeedback:
    """Track direction, filtered load, and smooth gripper damping."""
    def __init__(self, max_power=DEFAULT_POWER_MW):
        """Initialize damping parameters; max_power is the closing cap in mW."""
        self.max_power = max_power
        self.base_power = GRIPPER_BASE_POWER_MW
        self.base_deadzone = GRIPPER_DEADZONE_NM
        self.full_scale = GRIPPER_FULL_SCALE_NM
        self.rise_rate = GRIPPER_RISE_MW_S
        self.open_ratio = GRIPPER_OPEN_RATIO
        self.open_cap = GRIPPER_OPEN_MAX_POWER_MW
        self.open_step = GRIPPER_OPEN_STEP_DEG
        self.direction_confirm = GRIPPER_DIRECTION_CONFIRM_S
        self.direction_samples = GRIPPER_DIRECTION_SAMPLES


        # Motion deadzone: margin (Nm), speed scale (rad/s), acceleration coefficient.
        self.moving_margin, self.motion_scale, self.acceleration_margin = .08, .5, .01

        # Raw speed/position-step guards: 8 rad/s and 20 deg.
        self.max_speed, self.max_step = 8., math.radians(20.)

        if (not isinstance(max_power, int) or not 1 <= max_power <= 1500
                or not 0 < self.base_power <= self.open_cap <= max_power
                or not 0 < self.open_ratio < 1):
            raise ValueError('Invalid gripper power limits or opening ratio')
        self.reset()

    def reset(self):

        """Clear estimation history and output after startup or invalid data."""

        # Normal direction changes preserve history; invalid data calls this full reset.
        self.last = self.started = self.previous_q = self.previous_time = self.since = None

        self.velocity = self.acceleration = self.residual_nm = self.power = 0.
        self.deadzone_nm = self.base_deadzone

        self.leader_angle = self.leader_time = self.last_motion = None

        self.candidate = self.candidate_count = 0
        self.candidate_start = None
        self.candidate_travel = 0.

        self.opening = self.contact = False

        self.load_power = 0.

        self.output_time = None
        self.target_power = 0.
        self.state = 'WARMUP'

    def clear_candidate(self):
        """Discard pending direction evidence; retain confirmed direction and load."""

        self.candidate = self.candidate_count = 0
        self.candidate_start = None
        self.candidate_travel = 0.

    def note_leader(self, angle, now):
        """Confirm direction from leader angle in degrees and monotonic time in seconds."""
        if angle is None:
            return
        if not all(math.isfinite(x) for x in (angle, now)):
            raise FeedbackError('Non-finite leader gripper angle/time')

        if self.leader_time is not None and now <= self.leader_time:
            return
        previous, previous_time = self.leader_angle, self.leader_time
        self.leader_angle, self.leader_time = angle, now
        if previous is None or now - previous_time > SOURCE_AGE_S:
            self.clear_candidate()
            return
        delta = angle - previous

        if abs(delta) > 20.:
            self.clear_candidate()
            return
        if self.last_motion is not None and now - self.last_motion > GRIPPER_DIRECTION_GAP_S:
            self.clear_candidate()
        if abs(delta) < GRIPPER_MOTION_EPS_DEG:
            return
        direction = 1 if delta > 0 else -1
        self.last_motion = now

        if direction != self.candidate:
            self.candidate = direction
            self.candidate_start = now
            self.candidate_count = 0
            self.candidate_travel = 0.
        self.candidate_count += 1
        self.candidate_travel += abs(delta)

        # Count, duration, and travel must all pass; epsilon handles rounding.
        if (self.candidate_count >= self.direction_samples
                and now - self.candidate_start + 1e-9 >= self.direction_confirm
                and self.candidate_travel + 1e-9 >= self.open_step):
            self.opening = direction > 0

    def desired_power(self):
        """Return the unsmoothed heavy/closing or light/opening target in mW."""
        if self.opening:

            return self.base_power + min(self.open_cap - self.base_power,
                                         self.open_ratio * self.load_power)
        return self.base_power + self.load_power

    def render_power(self, now, dt=None, *, only_reduce=False):

        """Filter and slew-limit output; only_reduce forbids increases from cached load."""

        elapsed = dt if self.output_time is None else now - self.output_time
        if elapsed is None or elapsed <= 0:
            return self.power
        elapsed = min(elapsed, GAP_S)
        target = self.desired_power()
        # A no-op must not consume time needed for the later fresh-CAN update.
        if only_reduce and target >= self.power:

            return self.power
        self.target_power = target

        proposed = lowpass(self.power, target, elapsed, GRIPPER_POWER_FILTER_S)
        rate = self.rise_rate if proposed > self.power else GRIPPER_FALL_MW_S

        # Clamp the change to rate * elapsed, then clamp output to the allowed range.
        self.power = min(self.max_power, max(0., self.power +
                         max(-rate * elapsed, min(rate * elapsed, proposed - self.power))))
        self.output_time = now
        return self.power

    def relax_for_leader(self, now):

        """Allow smooth early reduction on opening while position feedback is fresh."""

        if (self.opening and self.started is not None and now - self.started >= .25
                and self.previous_time is not None and 0 <= now - self.previous_time <= SOURCE_AGE_S):
            self.render_power(now, only_reduce=True)

    def damp(self, residual, now, dt, h):

        """Filter torque (Nm), confirm load, then update smooth damping (mW)."""
        self.residual_nm = lowpass(self.residual_nm, residual, h, GRIPPER_TORQUE_FILTER_S)

        # Raise the deadzone during motion to reduce friction/acceleration artifacts.
        self.deadzone_nm = (self.base_deadzone + self.moving_margin *
                            min(1., abs(self.velocity) / self.motion_scale) +
                            self.acceleration_margin * abs(self.acceleration))
        magnitude = abs(self.residual_nm)

        if magnitude > self.deadzone_nm:
            if self.since is None:
                self.since = now
            if now - self.since >= GRIPPER_CONTACT_CONFIRM_S:
                self.contact = True
        # Hysteresis: clear below 80% of the entry threshold to reduce chatter.
        elif magnitude < .8 * self.deadzone_nm:

            self.since = None
            self.contact = False
        elif not self.contact:
            self.since = None

        self.load_power = (map_power(self.residual_nm, self.deadzone_nm, self.full_scale,
                                     self.max_power - self.base_power) if self.contact else 0.)
        if now - self.started < .25:
            self.state = 'WARMUP'
            return self.power

        self.state = ('LIGHT' if self.opening else 'HEAVY') + (
            '_LOAD' if self.contact else '_CONFIRM' if self.since is not None else '_IDLE')
        return self.render_power(now, dt)

    def update(self, q, torque_nm, now, received_at, *, leader_angle_deg=None):
        """Process equivalent position (rad), torque (Nm), and optional leader angle (deg)."""
        sample = self.sample(q, now, received_at)
        self.note_leader(leader_angle_deg, now)
        if not math.isfinite(torque_nm) or abs(self.velocity) > 6 or abs(self.acceleration) > 60:
            self.reset()
        if self.previous_q is None or self.started is None:
            self.state = 'INVALID'
            return self.power
        if sample is not None:
            self.damp(torque_nm, now, *sample)
        else:
            self.relax_for_leader(now)
        return self.power

    def sample(self, q, now, received_at):
        """Estimate motion from fresh position samples; return (update_dt, sample_dt) or None."""

        if (not all(math.isfinite(x) for x in (q, now, received_at))
                or not -.02 <= now - received_at <= SOURCE_AGE_S):
            self.reset()
            return None
        if self.last is not None and not 0 < now - self.last <= GAP_S:
            self.reset()
        if self.previous_q is None:

            self.previous_q, self.previous_time = q, received_at
            self.last = self.started = now
            return None
        dt = now - self.last
        h = received_at - self.previous_time
        # Repeated cached frames must not increase damping or refresh the sample clock.
        if h == 0:


            return None

        measured = (q - self.previous_q) / h if h > 0 else float('inf')
        if not 0 < h <= GAP_S or abs(measured) > self.max_speed or abs(q - self.previous_q) > self.max_step:
            self.reset()
            return None
        self.last = now
        prior = self.velocity


        self.velocity = lowpass(prior, measured, h, .06)

        self.acceleration = lowpass(self.acceleration, (self.velocity - prior) / h, h, .10)
        self.previous_q, self.previous_time = q, received_at
        return dt, h

    def step(self, snapshot, leader_angle_deg, now):
        """Check all feedback timestamps, then return gripper damping in mW."""

        times = snapshot.position_times + snapshot.torque_times
        if any(not -.02 <= now - stamp <= SOURCE_AGE_S for stamp in times):
            self.reset()
            return 0.
        return self.update(snapshot.q[6], snapshot.torque[6], now,
                           snapshot.position_times[6], leader_angle_deg=leader_angle_deg)

    def update_leader_intent(self, angle, now):
        """Update direction before CAN waits; return whether output decreased."""
        self.note_leader(angle, now)
        previous = self.power
        self.relax_for_leader(now)
        return self.power < previous


# 3. Read leader -> send targets -> read load -> calculate -> apply damping
def main(argv=None):
    """Connect devices, run teleoperation and damping, then release and close on exit."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--can', dest='can_name', default='can0')
    parser.add_argument('--rate', type=float, default=DEFAULT_RATE_HZ)

    parser.add_argument('--feedback-enable', action='store_true')
    args = parser.parse_args(argv)
    if not math.isfinite(args.rate) or not 1 <= args.rate <= 200:
        parser.error('--rate must be within 1..200 Hz')

    feedback, reader = GripperFeedback(), CANFeedback()
    leader = piper = output = bus = checked_tx = None
    gap_since = None
    try:


        bus = can.Bus(channel=args.can_name, interface='socketcan', can_filters=[
            {'can_id': cid, 'can_mask': 0x7FF, 'extended': False}
            for cid in [0x2A1, *range(0x251, 0x257), *range(0x261, 0x267), *range(0x2A5, 0x2A9)]])
        leader = LeaderConnection(args.port)
        unlock_leader(leader.servo, range(7))
        leader.servo.reset_multi_turn_angle(0xFF)
        output = GripperOutput(leader.servo, enabled=args.feedback_enable)
        piper = C_PiperInterface_V2.get_instance(can_name=args.can_name, judge_flag=True, can_auto_init=True)
        checked_tx = CheckedTransmit(piper)
        piper.ConnectPort()

        piper.ModeCtrl(ctrl_mode=0x01, move_mode=0x01, move_spd_rate_ctrl=100, is_mit_mode=0xAD)
        deadline = time.monotonic() + STARTUP_TIMEOUT_S
        while True:
            try:
                reader.checked_snapshot(bus, startup=True)
                break
            except FeedbackStaleError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.005)
        print(f'七轴遥控，仅夹爪阻尼；实际输出={args.feedback_enable}；{GRIPPER_PROFILE}')
        print('不包含HOLD；参数未实机标定；Ctrl+C退出。')
        shown = 0.
        while True:
            begin = time.monotonic()
            if gap_since is not None and begin - gap_since >= LEADER_TIMEOUT_S:
                raise FeedbackError('Leader missing for 0.25 s')
            # 1. Read fresh leader angles; opening may reduce damping before a CAN wait.
            angles = leader.read()
            after_read = time.monotonic()
            if gap_since is not None and after_read - gap_since >= LEADER_TIMEOUT_S:
                raise FeedbackError('Late leader sample rejected')
            if angles is not None and feedback.update_leader_intent(angles[6], after_read):
                output.apply(feedback.power, after_read)
            snapshot = reader.checked_snapshot(bus)
            now = time.monotonic()
            if angles is None:
                if gap_since is None:
                    gap_since = begin
                if now - gap_since >= LEADER_TIMEOUT_S:
                    raise FeedbackError('Leader missing for 0.25 s')
            else:
                gap_since = None
                # 2. Preserve seven-axis teleoperation, including follower opening.
                send_follower(piper, map_leader(angles))
                snapshot = reader.checked_snapshot(bus)
                now = time.monotonic()
            angle = None if angles is None else angles[6]
            # 3. Process fresh load and send damping only to leader servo 6.
            power = feedback.step(snapshot, angle, now)
            output.apply(power, now)
            if now - shown >= .5:
                print(f'{feedback.state} raw_Nm={snapshot.torque[6]:.3f} '
                      f'filtered_Nm={feedback.residual_nm:.3f} deadzone_Nm={feedback.deadzone_nm:.3f} '
                      f'target_mW={feedback.target_power:.0f} output_mW={power:.0f}')
                shown = now

            time.sleep(max(0., 1. / args.rate - (time.monotonic() - begin)))
    except KeyboardInterrupt:
        print('停止遥控和夹爪反馈')
    finally:

        if output:
            output.close()
        try:
            if piper:
                try:
                    piper.DisconnectPort()
                finally:
                    if checked_tx:
                        checked_tx.close()
        finally:
            try:
                if leader:
                    leader.close()
            finally:
                if bus:
                    bus.shutdown()


# 4. Serial/CAN communication and cleanup
class GripperOutput:
    """Write damping only to leader servo 6, with deduplication and explicit unlock."""
    def __init__(self, servo, enabled=False):
        """Reuse the connected servo manager; enabled controls actual damping writes."""
        self.servo, self.enabled = servo, enabled
        self.sent = None

    def apply(self, power, now):
        """Send power in mW; zero explicitly unlocks and now is monotonic seconds."""
        if not self.enabled:
            return
        if not math.isfinite(power) or not 0 <= power <= 1500:
            raise ValueError('Invalid damping power')
        value = round(power)
        mode = 'unlock' if value == 0 else 'damping'

        # 900 belongs to the unlock command; it is not a damping target.
        argument = 900 if value == 0 else value
        old = self.sent

        # Send on a mode change, >=10 mW change, or 0.5 s refresh.
        if old is None or old[0] != mode or abs(old[1]-argument) >= 10 or now-old[2] >= .5:
            if value == 0:
                self.servo.stop_on_control_mode(6, method=0x10, power=900)
            else:
                self.servo.set_damping(6, power=value)

            self.sent = mode, argument, now

    def close(self):
        """Attempt to release servo 6 if this output was enabled."""
        if self.enabled:
            unlock_leader(self.servo, [6], fail_on_error=False)


def unlock_leader(servo, ids, fail_on_error=True):
    """Release each requested leader ID; optionally raise after attempting all IDs."""
    failures = []
    for servo_id in ids:
        try:
            servo.stop_on_control_mode(servo_id, method=0x10, power=900)
        except Exception:
            failures.append(servo_id)
    if failures:
        message = f'Leader unlock failed: {failures}'
        if fail_on_error:
            raise FeedbackError(message)
        print(message, file=sys.stderr)


class FeedbackError(RuntimeError):
    """A control or communication fault requiring loop cleanup."""

class FeedbackStaleError(FeedbackError):
    def __init__(self, channels):
        """Store missing channel names for bounded gripper-only recovery."""
        self.channels = channels
        super().__init__('Piper feedback missing/stale: ' + ', '.join(channels))

@dataclass
class Snapshot:
    """Seven-axis positions (rad), torques (Nm), and monotonic receive timestamps."""


    q: list
    torque: list
    position_times: list
    torque_times: list

class CANFeedback:
    """Receive and validate per-channel CAN feedback before sending new targets."""

    def __init__(self):
        """Initialize caches; negative-infinite timestamps mean no sample received."""
        (self.q, self.torque) = ([0.0] * 6, [0.0] * 6)

        self.position_times = [float('-inf')] * 6
        self.torque_times = [float('-inf')] * 6
        self.gripper_q = self.gripper_torque = 0.0
        self.gripper_time = float('-inf')
        self.gripper_fault = self.arm_fault = False
        self.arm_status_time = float('-inf')
        self.temperature_times = [float('-inf')] * 6
        self.latched_fault = None

        self.last_by_id = {}

    def ingest(self, can_id, data, received_at):
        """Parse one eight-byte CAN payload with its monotonic receive time."""
        if len(data) != 8:
            return
        if not math.isfinite(received_at) or received_at < self.last_by_id.get(can_id, float('-inf')):
            return
        self.last_by_id[can_id] = received_at

        # 0x251..0x256: joint currents; these do not drive gripper damping.
        if 593 <= can_id <= 598:
            i = can_id - 593

            (_, current, _) = struct.unpack('>hhi', data)

            self.torque[i] = current * (1.18125 if i < 3 else 0.95844) / 1000.0
            self.torque_times[i] = received_at

        # 0x2A5..0x2A7: paired joint angles in 0.001 deg.
        elif 677 <= can_id <= 679:
            i = 2 * (can_id - 677)

            self.q[i:i + 2] = [math.radians(x / 1000.0) for x in struct.unpack('>ii', data)]
            self.position_times[i:i + 2] = [received_at] * 2

        # 0x2A8: stroke (0.001 mm), torque (0.001 Nm), status, reserved.
        elif can_id == 680:

            (stroke, torque, status, _) = struct.unpack('>ihBB', data)

            self.gripper_q = stroke / 80000.0 * math.radians(45.0)
            self.gripper_torque = torque / 1000.0

            self.gripper_fault = bool(status & 63)
            self.gripper_time = received_at

            if self.gripper_fault or not status & 64:
                self.latched_fault = f'Piper gripper fault/not enabled: 0x{status:02x}; enable follower before starting'

        # 0x2A1: arm status and error code.
        elif can_id == 673:
            self.arm_fault = bool(data[1] or int.from_bytes(data[6:8], 'big'))
            self.arm_status_time = received_at
            if self.arm_fault:
                self.latched_fault = 'Piper arm status/error code abnormal'

        # 0x261..0x266: joint temperature, enable state, and faults.
        elif 609 <= can_id <= 614:
            i = can_id - 609

            (_, driver_temp, motor_temp, status, _) = struct.unpack('>HhbBH', data)
            self.temperature_times[i] = received_at

            if status & 191 or not status & 64 or max(driver_temp, motor_temp) >= MAX_TEMPERATURE_C:
                self.latched_fault = f'Piper J{i + 1}: status=0x{status:02x}, temperature={max(driver_temp, motor_temp)}C'

    def drain(self, bus):
        """Drain queued CAN frames without blocking; reject stale or unsupported frames."""
        for _ in range(4096):
            msg = bus.recv(timeout=0)
            if msg is None:
                return
            if msg.is_error_frame or msg.is_remote_frame or msg.is_extended_id:
                continue

            # Convert Unix receive time to monotonic time without hiding queue age.
            received_at = time.monotonic() - (time.time() - msg.timestamp)
            if not -0.02 <= time.monotonic() - received_at <= CAN_TIMEOUT_S:
                continue
            self.ingest(msg.arbitration_id, bytes(msg.data), received_at)
        raise RuntimeError('CAN receive backlog: stopping feedback')

    def snapshot(self, now):
        """Return a fresh snapshot or raise for a latched fault or stale channel."""
        if self.latched_fault:
            raise FeedbackError(self.latched_fault)
        channels = {f'J{i + 1} position': stamp for (i, stamp) in enumerate(self.position_times)}
        channels.update({f'J{i + 1} torque': stamp for (i, stamp) in enumerate(self.torque_times)})
        channels.update({f'J{i + 1} temperature/status': stamp for (i, stamp) in enumerate(self.temperature_times)})

        channels.update(gripper=self.gripper_time, arm_status=self.arm_status_time)

        missing = [name for (name, stamp) in channels.items() if not -0.02 <= now - stamp <= CAN_TIMEOUT_S]
        if missing:
            raise FeedbackStaleError(missing)

        return Snapshot(self.q[:] + [self.gripper_q], self.torque[:] + [self.gripper_torque], self.position_times[:] + [self.gripper_time], self.torque_times[:] + [self.gripper_time])

    def checked_snapshot(self, bus, startup=False):
        """Receive and validate; briefly retry only an isolated gripper gap."""
        self.drain(bus)
        try:
            return self.snapshot(time.monotonic())
        except FeedbackStaleError as error:

            if startup or error.channels != ['gripper'] or (not math.isfinite(self.gripper_time)):
                raise
        deadline = time.monotonic() + GRIPPER_RECOVERY_S
        while time.monotonic() < deadline:
            self.drain(bus)
            try:
                return self.snapshot(time.monotonic())
            except FeedbackStaleError as error:
                if error.channels != ['gripper']:
                    raise
            time.sleep(0.005)
        raise FeedbackStaleError(['gripper'])

class CheckedSerial(serial.Serial):
    """Expose serial exceptions and short writes to the control loop."""

    def write(self, data):
        """Write all bytes or raise; return the transmitted byte count."""
        try:
            written = super().write(data)
        except serial.SerialException as error:
            raise ConnectionError('PiPER-Mate serial write failed') from error

        if written != len(data):
            raise ConnectionError('PiPER-Mate short serial write')
        return written

class LeaderConnection:
    """Read fresh monitor responses for all seven leader servos."""

    def __init__(self, port='/dev/ttyUSB0', uart=None):
        """Open the leader port or reuse the supplied UART and create one SDK manager."""

        self.uart = uart if uart is not None else CheckedSerial(port, 1000000, timeout=0, write_timeout=0.1)
        self.servo = uservo.UartServoManager(self.uart)

        self.fresh = {}
        self.pending_zero = {}
        self.last_angles = {}
        self.retry_count = 0
        self.fault = None

    def ingest_monitor(self, payload):
        """Validate a monitor payload and cache its angle in degrees."""
        if len(payload) != 16:
            return

        (servo_id, _, _, _, temp_raw, status, raw_angle, turn) = struct.unpack('<BHHHHBih', payload)

        if servo_id not in range(7) or (raw_angle == -235929599 and turn == 0):
            return

        if status & 254:
            self.fault = f'Leader servo {servo_id}: status=0x{status:02x}'
            return
        if 0 < temp_raw < 4096:

            # NTC conversion: 12-bit ADC, B=3435, 298.15 K reference; result is Celsius.
            temperature = 1 / (math.log(temp_raw / (4096 - temp_raw)) / 3435 + 1 / 298.15) - 273.15
            if temperature >= MAX_TEMPERATURE_C:
                self.fault = f'Leader servo {servo_id}: temperature={temperature:.1f}C'
                return
        angle = raw_angle / 10.0

        # Confirm a sudden nonzero-to-zero angle with three consecutive replies.
        if angle == 0.0 and abs(self.last_angles.get(servo_id, 0.0)) > 1.0:
            self.pending_zero[servo_id] = self.pending_zero.get(servo_id, 0) + 1
            if self.pending_zero[servo_id] < 3:
                return
        else:
            self.pending_zero.pop(servo_id, None)
        self.fresh[servo_id] = angle
        self.last_angles[servo_id] = angle

    def read_once(self, ids, timeout=0.04):
        """Query selected servo IDs and wait up to timeout seconds for fresh replies."""
        self.uart.reset_input_buffer()
        self.servo.pkt_buffer = uservo.PacketBuffer()

        # Query code, sync parameter, ID count, then the requested servo IDs.
        payload = struct.pack('<BBB', self.servo.CODE_QUERY_SERVO_MONITOR, 1, len(ids)) + bytes(ids)
        self.servo.send_request(self.servo.CODE_SYNC_COMMAND, payload)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.uart.in_waiting:
                for byte in self.uart.read(self.uart.in_waiting):
                    self.servo.pkt_buffer.update(bytes([byte]))
                while self.servo.pkt_buffer.has_valid_packet():
                    packet = self.servo.pkt_buffer.get_packet()
                    result = uservo.Packet.unpack(packet)
                    if result is not None and result[0] == self.servo.CODE_QUERY_SERVO_MONITOR:
                        self.ingest_monitor(result[1])
                if self.fault:
                    raise FeedbackError(self.fault)

                if all((i in self.fresh for i in ids)):
                    return
            else:
                time.sleep(0.0005)

    def read(self):
        """Return seven fresh angles in degrees, or None after at most one retry."""
        started = time.monotonic()

        self.fresh = {}
        self.retry_count = 0
        self.read_once(list(range(7)))
        missing = [i for i in range(7) if i not in self.fresh]

        if missing and time.monotonic() - started < LEADER_TIMEOUT_S / 2:
            self.retry_count = 1
            self.read_once(missing)
        if len(self.fresh) != 7:
            return None
        return [self.fresh[i] for i in range(7)]

    def close(self):
        """Close the leader serial port after damping has been released."""
        self.uart.close()

class CheckedTransmit:
    """Temporarily make SDK CAN transmission failures raise exceptions."""

    def __init__(self, piper):
        """Wrap the connected Piper transport without changing SDK source files."""

        self.transport = getattr(piper, '_arm_can', None)
        if self.transport is None:
            self.transport = getattr(piper, '_C_PiperInterface_V2__arm_can', None)
        if self.transport is None or not hasattr(self.transport, 'SendCanMessage'):
            raise FeedbackError('Unsupported Piper SDK CAN transport; cannot verify command transmission')
        self.original = self.transport.SendCanMessage
        success = self.transport.CAN_STATUS.SEND_MESSAGE_SUCCESS

        def checked_send(can_id, data):
            """Forward a CAN frame and raise if the SDK reports failure."""
            result = self.original(can_id, data)
            if result != success:
                raise FeedbackError(f'Piper CAN transmit failed: id=0x{can_id:x}, status={result}')
            return result

        self.transport.SendCanMessage = checked_send

    def close(self):
        """Restore the original SDK CAN sender."""
        self.transport.SendCanMessage = self.original

def map_leader(angles_deg):
    """Reverse selected joints and apply joint limits; preserve the gripper angle."""
    if len(angles_deg) != 7 or not all(math.isfinite(x) for x in angles_deg):
        raise FeedbackError('Seven finite leader angles required')

    joints = [max(lo, min(sign * angle, hi)) for angle, sign, (lo, hi) in
              zip(angles_deg[:6], JOINT_SIGNS, JOINT_LIMITS_DEG)]
    return joints + [angles_deg[6]]

def send_follower(piper, target):
    """Send seven-axis targets; convert gripper degrees to stroke and return quantized targets."""
    joints = [int(x * 1000.) for x in target[:6]]
    stroke_um = int(target[6] / 45. * 80000.)
    piper.JointCtrl(*joints)

    # Stroke in um, torque field 1000 (1 Nm), enable, no zero reset.
    piper.GripperCtrl(stroke_um, 1000, 0x01, 0)

    return [x / 1000. for x in joints] + [stroke_um / 80000. * 45.]


if __name__ == '__main__':
    main()
