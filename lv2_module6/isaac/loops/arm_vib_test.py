"""Real-robot arm vibration test: move the arm to a joint target or a hand pose with one FollowJointTrajectory goal
(straight to /arm_controller — no MoveIt, no collision check), then hold and measure how it shakes.
Run while ./manipulator-launcher.sh is up.

    python3 arm_vib_test.py --joints 0 -20 40 70 0             # joint1..5 [deg]
    python3 arm_vib_test.py --pose 0.22 0.0 0.15                # finger-tip face centre (x, y, z) in base_footprint [m], hand down
    python3 arm_vib_test.py --pose 0.25 0.0 0.15 --pitch 30    # hand tilted 30 deg from straight down, tip forward
    python3 arm_vib_test.py --named stow                        # pose_<name> from skills/params/basket.yaml
    python3 arm_vib_test.py --joints 0 -20 40 70 0 --rounds 5 --back --time 1.0 --tag sag_off

Per leg (start -> target, and target -> start with --back): the move takes --time s (default from --speed), the JTC
interpolates a quintic from the current state (zero end velocity / acceleration), then --hold s are measured.
Printed per joint over the hold: pk-pk [deg] (whole hold), osc [deg] (pk-pk over the last half), settle [s] (last time
|q - final| > --tol after the move ends), freq [Hz] (FFT peak of the hold), offset [deg] (final - target), max |I| [A].
Encoders only see servo motion — link bending under gravity does not show on /joint_states (watch the hand too).
The body IMU (/imu_sensor_broadcaster/imu) gyro RMS over the hold is printed when it publishes.

Change settings between runs with `ros2 param set /rl_arm_correction <name> <value>` (sag_enabled, sag_gain,
gravity_enabled ...); the values at each run are saved next to the result, --tag labels the run.
Output: logs/arm_vib_<ts>.csv (raw samples), logs/arm_vib_<ts>_summary.csv (one row per leg x joint).
--bag: one ./manipulator-record.sh motion bag for the whole run (logs/arm_vib_<ts>_bag/).
Ctrl-C cancels the goal (the arm stops where it is) and saves what was logged.
"""

import argparse
import csv
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from sensor_msgs.msg import Imu, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from arm_model import ARM, Arm

HERE = Path(__file__).resolve().parent
BASKET = HERE.parents[1] / "ros2_ws" / "src" / "skills" / "params" / "basket.yaml"
RL_PARAMS = ["enabled", "sag_enabled", "sag_gain", "sag_max", "still_rate", "gravity_enabled", "twist_state", "max_delta"]
START_DELAY = 0.3  # [s] goal header stamp ahead of now, so the move start is known exactly

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
g = parser.add_mutually_exclusive_group(required=True)
g.add_argument("--joints", type=float, nargs=5, metavar="DEG", help="joint1..5 target [deg]")
g.add_argument("--pose", type=float, nargs=3, metavar=("X", "Y", "Z"), help="finger-tip face centre in base_footprint [m]")
g.add_argument("--named", help="pose_<name> in basket.yaml (stow, carry_left, carry_right)")
parser.add_argument("--pitch", type=float, default=0.0, help="--pose: hand tilt from straight down [deg], + = hand axis tilts forward (away from the robot)")
parser.add_argument("--roll", type=float, default=0.0, help="--pose: joint5 [deg]")
parser.add_argument("--time", type=float, default=0.0, help="move time [s]; 0 = from --speed")
parser.add_argument("--speed", type=float, default=30.0, help="largest joint speed (average) when --time is 0 [deg/s]")
parser.add_argument("--hold", type=float, default=3.0, help="measure this long after the move ends [s]")
parser.add_argument("--tol", type=float, default=0.2, help="settle band [deg]")
parser.add_argument("--rounds", type=int, default=1)
parser.add_argument("--back", action="store_true", help="return to the start pose after each target leg (measured too)")
parser.add_argument("--tag", default="", help="label written to the CSVs (e.g. which setting is on)")
parser.add_argument("--bag", action="store_true", help="record a manipulator-record.sh motion bag of the run")
parser.add_argument("--yes", action="store_true", help="no Enter before the first move")
args = parser.parse_args()

os.environ.setdefault("FASTDDS_DEFAULT_PROFILES_FILE", str(HERE.parents[1] / "ros2_ws" / "fastdds_local_only.xml"))
rclpy.init()
node = rclpy.create_node("arm_vib_test")
js = {"msg": None}
rows = []  # raw samples: [t, leg, pos x5, vel x5, effort x5, gx, gy, gz]
imu = {"g": (math.nan,) * 3}
rec = {"leg": None, "t0": 0.0}


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def on_js(m):
    idx = {n: i for i, n in enumerate(m.name)}
    if not all(n in idx for n in ARM):
        return
    js["msg"] = m
    if rec["leg"] is None:
        return
    get = lambda arr, n: arr[idx[n]] if len(arr) > idx[n] else math.nan
    rows.append([stamp(m.header) - rec["t0"], rec["leg"]] + [get(m.position, n) for n in ARM]
                + [get(m.velocity, n) for n in ARM] + [get(m.effort, n) for n in ARM] + list(imu["g"]))


def on_imu(m):
    imu["g"] = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)


node.create_subscription(JointState, "/joint_states", on_js, 50)
node.create_subscription(Imu, "/imu_sensor_broadcaster/imu", on_imu, 50)
jtc = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def current_q(timeout=5.0):
    end = time.time() + timeout
    while js["msg"] is None and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if js["msg"] is None:
        raise SystemExit("no /joint_states with joint1..5 — is manipulator-launcher.sh up (same ROS_DOMAIN_ID)?")
    m = js["msg"]
    return np.array([m.position[list(m.name).index(n)] for n in ARM])


def pose_ik(arm, x, y, z, pitch, roll, seed):
    """joint1..5 with the tip mark at (x, y, z) and the hand pitched from down (q2 + q3 + q4 = 90 deg - pitch)"""
    from scipy.optimize import least_squares

    def res(v):
        m = arm.mark([v[0], v[1], v[2], v[3], roll])
        return [m[0] - x, m[1] - y, m[2] - z, 2 * (v[1] + v[2] + v[3] - math.pi / 2 + pitch)]
    lims = [arm.joints[n]["lim"] or (-math.pi, math.pi) for n in ARM[:4]]
    lo, hi = [l[0] for l in lims], [l[1] for l in lims]
    best = None
    for s in ([seed[0], *seed[1:4]], [math.atan2(y, x), 0.5, -0.2, 1.2], [math.atan2(y, x), -0.4, 0.8, 1.15],
              [math.atan2(y, x), 0.2, 0.2, 1.2], [math.atan2(y, x), -0.2, 0.4, 1.0]):
        r = least_squares(res, np.clip(s, lo, hi), bounds=(lo, hi))
        if np.max(np.abs(r.fun)) < 1e-4 and (best is None or r.cost < best.cost):
            best = r
    return None if best is None else np.array([*best.x, roll])


def rl_params():
    cli = node.create_client(GetParameters, "/rl_arm_correction/get_parameters")
    if not cli.wait_for_service(timeout_sec=2.0):
        return {}
    fut = cli.call_async(GetParameters.Request(names=RL_PARAMS))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
    if fut.result() is None:
        return {}
    out = {}
    for n, v in zip(RL_PARAMS, fut.result().values):
        out[n] = {1: v.bool_value, 2: v.integer_value, 3: v.double_value, 4: v.string_value}.get(v.type, "")
    return out


goal_handle = {"h": None}


def move(q_target, t_move, leg):
    """one JTC goal start -> q_target in t_move, then hold; samples go to rows under this leg. Returns (ok, error)"""
    g = FollowJointTrajectory.Goal()
    g.trajectory.joint_names = ARM
    t0 = node.get_clock().now() + Duration(seconds=START_DELAY)
    g.trajectory.header.stamp = t0.to_msg()
    g.trajectory.points = [JointTrajectoryPoint(positions=list(map(float, q_target)), velocities=[0.0] * 5,
                                                accelerations=[0.0] * 5, time_from_start=Duration(seconds=t_move).to_msg())]
    rec.update(leg=leg, t0=t0.nanoseconds * 1e-9)
    fut = jtc.send_goal_async(g)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    h = fut.result()
    if h is None or not h.accepted:
        rec["leg"] = None
        return False, "JTC rejected the goal"
    goal_handle["h"] = h
    res = h.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=START_DELAY + t_move + 10.0)
    goal_handle["h"] = None
    err = "" if res.done() and res.result().result.error_code == 0 else (
        res.result().result.error_string if res.done() else "result timeout")
    while node.get_clock().now().nanoseconds * 1e-9 - rec["t0"] < t_move + args.hold:  # JTC holds the last point
        rclpy.spin_once(node, timeout_sec=0.01)
    rec["leg"] = None
    return not err, err


def leg_metrics(leg, q_target, t_move):
    """per joint over the hold window: pk-pk, osc (last half), settle time, FFT peak, offset, max current"""
    a = np.array([r for r in rows if r[1] == leg and r[0] >= 0.0], dtype=object)
    if len(a) == 0:
        return []
    t = a[:, 0].astype(float)
    hold = t >= t_move
    out = []
    for j, n in enumerate(ARM):
        q = a[:, 2 + j].astype(float)
        cur = np.abs(a[:, 12 + j].astype(float))
        th, qh = t[hold], q[hold]
        if len(qh) < 10:
            continue
        final = np.mean(qh[th >= th[-1] - 0.5])
        out_band = th[np.abs(qh - final) > math.radians(args.tol)]
        tu = np.arange(th[0], th[-1], 0.01)  # uniform 100 Hz for the FFT
        qu = np.interp(tu, th, qh)
        qu = qu - np.polyval(np.polyfit(tu, qu, 1), tu)  # drop offset + slow drift (sag integral)
        spec = np.abs(np.fft.rfft(qu * np.hanning(len(qu))))
        f = np.fft.rfftfreq(len(qu), 0.01)
        k = 1 + int(np.argmax(spec[1:])) if len(spec) > 1 else 0
        second = qh[th >= th[0] + (th[-1] - th[0]) / 2]
        out.append(dict(joint=n, target_deg=math.degrees(q_target[j]), pkpk_deg=math.degrees(np.ptp(qh)),
                        osc_deg=math.degrees(np.ptp(second)),
                        settle_s=(out_band[-1] - t_move) if len(out_band) else 0.0,
                        freq_hz=f[k] if np.ptp(qu) > math.radians(0.02) else 0.0,
                        offset_deg=math.degrees(final - q_target[j]),
                        max_abs_current_A=np.nanmax(cur) if np.isfinite(cur).any() else math.nan))
    g = a[hold][:, 17:20].astype(float)
    gyro = math.sqrt(np.nanmean(np.sum(g ** 2, axis=1))) if np.isfinite(g).any() else math.nan
    for m in out:
        m["imu_gyro_rms"] = gyro
    return out


def main():
    arm = Arm()
    if not jtc.wait_for_server(timeout_sec=10.0):
        raise SystemExit("/arm_controller/follow_joint_trajectory not up — is manipulator-launcher.sh running?")
    q0 = current_q()
    if args.joints:
        q1 = np.radians(args.joints)
    elif args.named:
        table = yaml.safe_load(BASKET.read_text())
        if f"pose_{args.named}" not in table:
            raise SystemExit(f"no pose_{args.named} in {BASKET}: {[k[5:] for k in table if k.startswith('pose_')]}")
        q1 = np.array(table[f"pose_{args.named}"], float)
    else:
        q1 = pose_ik(arm, *args.pose, math.radians(args.pitch), math.radians(args.roll), q0)
        if q1 is None:
            raise SystemExit(f"--pose {args.pose} pitch {args.pitch}: no IK solution inside the joint limits")
    for j, n in enumerate(ARM):
        lo, hi = arm.joints[n]["lim"] or (-math.pi, math.pi)
        if not lo <= q1[j] <= hi:
            raise SystemExit(f"{n} {math.degrees(q1[j]):.1f} deg outside the URDF limits [{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]")
    t_move = args.time or max(1.0, math.degrees(np.max(np.abs(q1 - q0))) / args.speed)
    tip0, tip1 = arm.mark(q0), arm.mark(q1)
    print(f"start  [deg] {np.round(np.degrees(q0), 1)}  tip {np.round(tip0 * 1000)} mm")
    print(f"target [deg] {np.round(np.degrees(q1), 1)}  tip {np.round(tip1 * 1000)} mm  (base_footprint)")
    print(f"move {t_move:.2f} s, hold {args.hold:.1f} s, rounds {args.rounds}{', back to start' if args.back else ''}")
    if tip1[2] < 0.08:
        print(f"WARN: tip z {tip1[2] * 1000:.0f} mm — near the floor / box top")
    params = rl_params()
    print(f"rl_arm_correction: {params or 'parameters not readable'}")
    if not args.yes:
        input("Enter = move (direct JTC, no collision check), Ctrl-C = quit ")

    ts = time.strftime("%Y%m%d_%H%M%S")
    (HERE / "logs").mkdir(exist_ok=True)
    raw_path, sum_path = HERE / "logs" / f"arm_vib_{ts}.csv", HERE / "logs" / f"arm_vib_{ts}_summary.csv"
    bag = None
    if args.bag:
        bag = subprocess.Popen([str(HERE / "manipulator-record.sh"), "motion"], start_new_session=True,
                               env=dict(os.environ, BAG_OUT=str(HERE / "logs" / f"arm_vib_{ts}_bag")))
        spin(2.0)
    summary = []
    try:
        legs = []
        for r in range(1, args.rounds + 1):
            legs.append((f"r{r}_go", q1))
            if args.back:
                legs.append((f"r{r}_back", q0))
        for leg, qt in legs:
            ok, err = move(qt, t_move, leg)
            ms = leg_metrics(leg, qt, t_move)
            print(f"\n{leg}: {'ok' if ok else 'JTC: ' + err}")
            print(f"  {'joint':7} {'pk-pk':>7} {'osc':>7} {'settle':>7} {'freq':>6} {'offset':>7} {'max|I|':>7}   [deg, deg, s, Hz, deg, A]")
            for m in ms:
                print(f"  {m['joint']:7} {m['pkpk_deg']:7.3f} {m['osc_deg']:7.3f} {m['settle_s']:7.2f} {m['freq_hz']:6.1f} "
                      f"{m['offset_deg']:7.3f} {m['max_abs_current_A']:7.3f}")
                summary.append(dict(leg=leg, tag=args.tag, move_s=t_move, ok=ok, **m, **{f"rl_{k}": v for k, v in params.items()}))
            if ms and math.isfinite(ms[0]["imu_gyro_rms"]):
                print(f"  body gyro rms {ms[0]['imu_gyro_rms']:.4f} rad/s")
            if not ok:
                break
    except KeyboardInterrupt:
        if goal_handle["h"] is not None:
            fut = goal_handle["h"].cancel_goal_async()
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
            print("\ngoal cancelled — the arm stops where it is")
    finally:
        with open(raw_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "leg"] + [f"{n}_pos" for n in ARM] + [f"{n}_vel" for n in ARM] + [f"{n}_effort_A" for n in ARM]
                       + ["gyro_x", "gyro_y", "gyro_z"])
            w.writerows(rows)
        if summary:
            with open(sum_path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(summary[0]))
                w.writeheader()
                w.writerows(summary)
        print(f"\nsaved {raw_path}" + (f"\n      {sum_path}" if summary else ""))
        if bag is not None:
            os.killpg(bag.pid, signal.SIGINT)
            bag.wait(timeout=10)
            print(f"bag   {HERE / 'logs' / f'arm_vib_{ts}_bag'}")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
