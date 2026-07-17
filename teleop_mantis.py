import os
import yaml
import time
import threading
import numpy as np
import pinocchio as pin
import qpsolvers
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from pink import Configuration, solve_ik
from pink.exceptions import PinkError
from pink.tasks import FrameTask, PostureTask
from pink.limits import ConfigurationLimit, VelocityLimit
from scipy.spatial import ConvexHull
from ament_index_python.packages import get_package_share_directory

path = os.environ.get("teleop_config", os.path.join(os.path.dirname(os.path.abspath(__file__)), "config_teleop_mantis.yaml"))
with open(path) as f:
    CFG = yaml.safe_load(f)

# Barrier keys moved from teleop: to ik: — warn if an old-style config still has them
# under teleop (their values are ignored there; do NOT fall back, the old defaults
# include the known-bad barrier_safe_gain=1.0).
for _k in ("collision_barrier", "d_min", "d_min_self", "d_influence", "barrier_gain", "barrier_safe_gain",
           "self_collision_min_hops", "drop_dist_thresh", "n_collision_pairs"):
    if _k in (CFG.get("teleop") or {}):
        print(f"WARNING: teleop.{_k} has moved to ik.{_k}; the value under teleop is IGNORED "
              f"(effective: {CFG.get('ik', {}).get(_k, 'built-in default')})")

# urdf/srdf may be given relative to the CONFIG FILE's directory, so the same
# config works on the host (~/work/docker_shared) and inside the docker container
# (/home/ros/share) where the shared directory is mounted at a different path.
_CFG_DIR = os.path.dirname(os.path.abspath(path))
def _cfg_path(p):
    return p if os.path.isabs(p) else os.path.join(_CFG_DIR, p)

URDF = _cfg_path(CFG["urdf"])
EE_FRAME = CFG["ee_frame"]
ARM = list(CFG["arm"])
ARM_CMD_JOINTS = list(CFG["arm_cmd_joints"])
SHOULDER_JOINT = CFG.get("shoulder_joint", "left_shoulder_lift_joint")
# Parked pose for every joint NOT in `arm` (right arm, hand, ...). The reduced model
# locks them here, so self-collision checks see the real parked robot, not zeros.
LOCKED_Q = dict(CFG.get("locked_q") or {})

RATE = float(CFG["rate"])
DT = 1.0 / RATE

POSITION_COST = float(CFG["ik"]["position_cost"])
ORIENTATION_COST = float(CFG["ik"]["orientation_cost"])
LM_DAMPING = float(CFG["ik"]["lm_damping"])
TASK_GAIN = float(CFG["ik"]["task_gain"])
POSTURE_COST = float(CFG["ik"]["posture_cost"])
VEL_SCALE = float(CFG["ik"]["vel_scale"])
COLLISION_MARGIN = float(CFG["ik"]["collision_margin"])

# Collision barrier (CBF): keeps the min link-pair gap >= d_min as a hard QP
# inequality, so the arm slides around obstacles instead of freezing.
COLLISION_BARRIER = bool(CFG["ik"].get("collision_barrier", True))
# d_min must stay ABOVE collision_margin: in_collision() honors the margin, so a
# barrier floor below it would count barrier-held poses as collisions.
D_MIN = float(CFG["ik"].get("d_min", 0.015))
# Separate (smaller) floor for arm SELF pairs. Structurally-close self pairs
# (e.g. forearm<->wrist_2 on a UR: convex-hull gap sits on a ~15mm plateau over
# almost the whole wrist range) would otherwise ride the d_min boundary
# permanently — stealing wrist DOF and chattering. Hull inflation makes the
# small floor still conservative vs real contact.
D_MIN_SELF = float(CFG["ik"].get("d_min_self", 0.006))
D_INFLUENCE = float(CFG["ik"].get("d_influence", 0.04))
# NOTE: keep barrier_gain*dt well below 1.0. gain=100 at 100 Hz gives
# gamma*dt=1.0 — a discrete CBF with zero stability margin: on an active
# constraint the QP bangs joints into the velocity limits at the Nyquist
# frequency (the "violent shaking" diagnosed from the 2026-07-10 bag).
BARRIER_GAIN = float(CFG["ik"].get("barrier_gain", 15.0))
# NOTE: keep barrier_safe_gain at 0.0 — pink adds its "safe displacement" pull to the
# QP objective unconditionally (not just near contact), which drags the whole arm,
# leaves a multi-cm steady position error and makes tracking sluggish. The hard CBF
# inequality alone does the dodging.
BARRIER_SAFE_GAIN = float(CFG["ik"].get("barrier_safe_gain", 0.0))
SELF_MIN_HOPS = int(CFG["ik"].get("self_collision_min_hops", 2))
N_COLLISION_PAIRS = int(CFG["ik"].get("n_collision_pairs", 32))
DROP_DIST_THRESH = float(CFG["ik"].get("drop_dist_thresh", 0.03))
ENV_KEYWORDS = tuple(CFG["ik"].get("env_keywords", ["table", "wall", "floor", "ground"]))
# Reachability prune: drop arm-vs-static pairs the arm can never approach closer
# than prune_margin (conservative bounding-sphere bound over sampled arm configs).
# The barrier jacobian is pure Python and linear in pair count — this is THE speed lever.
PRUNE_SAMPLES = int(CFG["ik"].get("prune_samples", 20000))
PRUNE_MARGIN = float(CFG["ik"].get("prune_margin", 0.05))

SCALE = float(CFG["teleop"]["scale"])
AZ_GAIN = float(CFG["teleop"]["az_gain"])
REACH_LO_FRAC = float(CFG["teleop"]["reach_lo_frac"])
REACH_HI_FRAC = float(CFG["teleop"]["reach_hi_frac"])

AXIS_SIGN = np.array(CFG["teleop"]["axis_sign"])
AXIS_MAP = list(CFG["teleop"]["axis_map"])
M = np.zeros((3, 3))
for i in range(3):
    M[i, AXIS_MAP[i]] = AXIS_SIGN[i]

# Per-axis sign for orientation only (robot base frame x,y,z); flip a component to
# invert the sense of rotation about that axis without touching position mapping.
ORI_SIGN = np.array(CFG["teleop"].get("ori_sign", [1.0, 1.0, 1.0]), float)

# Nose within this angle of vertical at engage -> heading falls back to the
# controller top axis (see yaw_frame); stored as the sine = horizontal-projection norm.
YAW_FALLBACK_SIN = np.sin(np.radians(float(CFG["teleop"].get("yaw_fallback_deg", 15.0))))

# Target shaping / latency:
MAX_TARGET_SPEED = float(CFG["teleop"].get("max_target_speed", 1.2))
MAX_LEAD = float(CFG["teleop"].get("max_lead", 0.08))
MAX_ANG_SPEED = float(CFG["teleop"].get("max_ang_speed", 4.0))
MAX_ANG_LEAD = float(CFG["teleop"].get("max_ang_lead", 0.5))
ORI_ALPHA = float(CFG["teleop"].get("ori_alpha", 0.8))
FILTER_MIN_CUTOFF = float(CFG["teleop"].get("filter_min_cutoff", 1.5))
# beta is in Hz per (m/s): cutoff = min_cutoff + beta*|v|. Hand speeds are ~0.1-1 m/s,
# so beta must be O(10) for the cutoff to actually open up in motion (low lag).
FILTER_BETA = float(CFG["teleop"].get("filter_beta", 15.0))
POSE_TIMEOUT = float(CFG["teleop"].get("pose_timeout", 0.2))
JOINT_TIMEOUT = float(CFG["teleop"].get("joint_timeout", 0.3))
BUTTONS_TIMEOUT = float(CFG["teleop"].get("buttons_timeout", 0.5))
PAD_DEBOUNCE = float(CFG["teleop"].get("pad_debounce", 0.25))
BLEND_TICKS = int(CFG["teleop"].get("disengage_blend_ticks", 5))
# Keep this small on a UR5: the clamp allows a STANDING cmd-vs-measured offset while
# the arm is blocked/paused, and the servo snaps through it on release.
MAX_JOINT_LEAD = float(CFG["teleop"].get("max_joint_lead", 0.15))
VR_RATE = float(CFG["teleop"].get("vr_rate", 250.0))
VR_DT = 1.0 / VR_RATE
MAX_TARGET_STEP = MAX_TARGET_SPEED * VR_DT
MAX_ANG_STEP = MAX_ANG_SPEED * DT

# Output command shaping: a dedicated thread republishes the IK result at
# out_rate with per-joint velocity+acceleration limits, so the robot always sees
# a dense stream with continuous velocity — no 100 Hz staircase beating against
# the UR servo cycle (125 Hz on CB3), and any residual tick-level oscillation of
# the IK output is attenuated below perception. out_rate 0 = old direct path.
OUT_RATE = float(CFG["teleop"].get("out_rate", 250.0))
OUT_VEL = float(CFG["teleop"].get("out_vel", 1.5))
OUT_ACCEL = float(CFG["teleop"].get("out_accel", 10.0))
OUT_KP = float(CFG["teleop"].get("out_kp", 20.0))
OUT_STALE = max(3.0 * DT, 0.05)

GRIP_OPEN = float(CFG["gripper"]["grip_open"])
GRIP_CLOSE = float(CFG["gripper"]["grip_close"])
TRIG_HOLD = float(CFG["gripper"]["trig_hold"])
GRIP_ACTION = CFG["gripper"]["action"]
GRIP_EFFORT = float(CFG["gripper"].get("effort", 40.0))

ARM_CMD_TOPIC = CFG["topics"]["arm_cmd"]
JOINT_STATES_TOPIC = CFG["topics"]["joint_states"]
VIVE_POSE_TOPIC = CFG["topics"].get("vive_pose", "/vive/pose")
VIVE_BUTTONS_TOPIC = CFG["topics"].get("vive_buttons", "/vive/buttons")

RLIMITS = {k: tuple(v) for k, v in (CFG.get("rlimits") or {}).items()}

def mesh_pkg_dirs():
    """Package dirs used to resolve mesh paths referenced by the URDF."""
    prefix = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
    return [os.path.join(p, "share") for p in prefix if p]

def srdf_path():
    """Path to the MoveIt SRDF, used to disable allowed collision pairs."""
    return _cfg_path(CFG["srdf"])

class OneEuroFilter:
    """3D one-euro filter: low lag in motion, smooths jitter at rest."""
    def __init__(self, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.x_prev = None
        self.t_prev = None
        self.dx_prev = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * np.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x, t):
        x = np.asarray(x, float)
        if self.t_prev is None:
            self.t_prev = t
            self.x_prev = x
            self.dx_prev = np.zeros_like(x)
            return x
        dt = t - self.t_prev
        if dt <= 0.0:
            return self.x_prev
        dx = (x - self.x_prev) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        dx_hat = a_d * dx + (1.0 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * float(np.linalg.norm(dx_hat))
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1.0 - a) * self.x_prev
        self.t_prev = t
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        return x_hat

class CommandShaper:
    """Robot-facing command stage: tracks the latest IK joint vector with
    per-joint velocity and acceleration limits, publishing at out_rate from its
    own thread. Feed-forward only (no measured feedback): the braking-aware
    profile cannot overshoot the target. While the target is stale (IK held /
    not ready) it publishes nothing and hard-resyncs to the next fresh target,
    which the tick has already re-anchored to the measured robot."""
    def __init__(self, publisher, rate, v_max, a_max, kp):
        self.pub = publisher
        self.rate = float(rate)
        self.v_max = float(v_max)
        self.a_max = float(a_max)
        self.kp = float(kp)
        self._lock = threading.Lock()
        self._target = None
        self._t_target = 0.0
        self._q = None
        self._v = None
        self._dts = []
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop = True
        self._thread.join(timeout=1.0)

    def set_target(self, vec):
        vec = np.asarray(vec, float)
        with self._lock:
            self._target = vec
            self._t_target = time.monotonic()

    def stats(self):
        """(n, p50, p95, max) of actual publish intervals [ms] since last call."""
        with self._lock:
            arr, self._dts = self._dts, []
        if not arr:
            return None
        a = np.array(arr)
        return len(a), float(np.percentile(a, 50)), float(np.percentile(a, 95)), float(a.max())

    def _step(self, tgt, dt):
        if self._q is None or self._q.shape != tgt.shape:
            self._q = tgt.copy()
            self._v = np.zeros_like(tgt)
            return self._q
        err = tgt - self._q
        v_brake = np.sqrt(2.0 * self.a_max * np.abs(err))
        v_des = np.clip(np.clip(self.kp * err, -v_brake, v_brake), -self.v_max, self.v_max)
        self._v = self._v + np.clip(v_des - self._v, -self.a_max * dt, self.a_max * dt)
        self._q = self._q + self._v * dt
        return self._q

    def _run(self):
        period = 1.0 / self.rate
        next_t = time.monotonic()
        t_last = next_t
        while not self._stop:
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()  # fell behind (GIL/load): re-anchor, don't burst
            now = time.monotonic()
            # measured dt (clamped) so velocity stays continuous through scheduling jitter
            dt = min(max(now - t_last, 0.25 * period), 4.0 * period)
            t_last = now
            with self._lock:
                tgt = self._target
                fresh = tgt is not None and (now - self._t_target) < OUT_STALE
            if tgt is None:
                continue
            if not fresh:
                self._q = None  # resync to the next fresh target (tick re-anchors it to measured)
                continue
            q = self._step(tgt, dt)
            with self._lock:
                self._dts.append(dt * 1000.0)
            msg = Float64MultiArray()
            msg.data = [float(x) for x in q]
            self.pub.publish(msg)


try:
    from pink.barriers import SelfCollisionBarrier as _SCBarrier
except Exception:
    _SCBarrier = None

if _SCBarrier is not None:
    class MarginSelfCollisionBarrier(_SCBarrier):
        """SelfCollisionBarrier with a per-pair floor: h_i = d_i - d_min_i
        (self pairs get D_MIN_SELF, everything else D_MIN). Rows of h and J are
        both selected by MARGIN — the parent selects J rows by raw distance,
        which would mismatch h rows under per-pair floors."""
        def __init__(self, n, gain, safe_displacement_gain, d_min_vec):
            d_min_vec = np.asarray(d_min_vec, float)
            super().__init__(n, gain=gain, safe_displacement_gain=safe_displacement_gain, d_min=float(d_min_vec.min()))
            self.d_min_vec = d_min_vec

        def _margins(self, configuration):
            n_pairs = len(configuration.collision_model.collisionPairs)
            d = np.array([configuration.collision_data.distanceResults[k].min_distance for k in range(n_pairs)])
            return d - self.d_min_vec[:n_pairs]

        def _select(self, margins):
            if self.dim >= len(margins):
                return np.arange(len(margins))
            return np.argpartition(-margins, -self.dim)[-self.dim:]

        def compute_barrier(self, configuration):
            m = self._margins(configuration)
            return m[self._select(m)]

        def compute_jacobian(self, configuration):
            model, data = configuration.model, configuration.data
            cm, cd = configuration.collision_model, configuration.collision_data
            m = self._margins(configuration)
            idxs = self._select(m)
            J = np.zeros((self.dim, model.nv))
            for i, k in enumerate(idxs):
                k = int(k)
                cp = cm.collisionPairs[k]
                dr = cd.distanceResults[k]
                j1_id = cm.geometryObjects[cp.first].parentJoint
                j2_id = cm.geometryObjects[cp.second].parentJoint
                w1 = np.array(dr.getNearestPoint1())
                w2 = np.array(dr.getNearestPoint2())
                r1 = w1 - data.oMi[j1_id].translation
                r2 = w2 - data.oMi[j2_id].translation
                if np.allclose(w1, w2):
                    continue
                n = (w1 - w2) / np.linalg.norm(w1 - w2)
                J1 = pin.getJointJacobian(model, data, j1_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                row = n.T @ J1[:3, :] + (pin.skew(r1) @ n).T @ J1[3:, :]
                J2 = pin.getJointJacobian(model, data, j2_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                row -= n.T @ J2[:3, :] + (pin.skew(r2) @ n).T @ J2[3:, :]
                J[i] = row
            return np.nan_to_num(J)


class PinkIK:
    """Pinocchio model + Pink differential IK for the mantis left arm with
    self-collision / environment collision avoidance (dual-arm model, everything
    outside `arm_joints` locked at its parked pose)."""
    def __init__(self, urdf_path, ee_frame, arm_joints, position_cost=POSITION_COST, orientation_cost=ORIENTATION_COST, lm_damping=LM_DAMPING, gain=TASK_GAIN, posture_cost=POSTURE_COST, vel_scale=VEL_SCALE, solver=None, srdf_path=None, package_dirs=None, collision_margin=COLLISION_MARGIN, locked_q=None, logger=None):
        """Build the reduced model and collision geometry (convex hulls + table box),
        set up frame/posture tasks, joint limits, the collision barrier and the QP solver.
        locked_q: parked pose for the non-teleoperated joints (defaults to config LOCKED_Q)."""
        self.log = logger
        full = pin.buildModelFromUrdf(urdf_path)
        keep = set(arm_joints)
        locked = [full.getJointId(n) for n in full.names[1:] if n not in keep]
        q_ref = pin.neutral(full)
        if locked_q is None:
            locked_q = LOCKED_Q
        for jname, val in locked_q.items():
            if full.existJointName(jname):
                q_ref[full.joints[full.getJointId(jname)].idx_q] = float(val)
            else:
                self._warn(f"locked_q joint '{jname}' not in URDF, ignored")
        # remember the pose every locked 1-dof joint was frozen at, so the Bridge can
        # detect a mismatch vs the real parked robot and rebuild
        self.locked_ref = {}
        for jid in locked:
            j = full.joints[jid]
            if j.nq == 1:
                self.locked_ref[full.names[jid]] = float(q_ref[j.idx_q])
        self.geom = None
        self.blocked = False
        if srdf_path and package_dirs:
            geom_full = pin.buildGeomFromUrdf(full, urdf_path, pin.GeometryType.COLLISION, package_dirs=list(package_dirs))
            if locked:
                self.model, self.geom = pin.buildReducedModel(full, geom_full, locked, q_ref)
            else:
                self.model, self.geom = full, geom_full
            self._convexify()
            self._table_box_override()
            self.geom.addAllCollisionPairs()
            pin.removeCollisionPairs(self.model, self.geom, srdf_path, False)
            self._filter_pairs(arm_joints)
            self.geom_data = self.geom.createData()
            for req in self.geom_data.collisionRequests:
                req.security_margin = float(collision_margin)
            self.col_data = self.model.createData()
        else:
            self.model = pin.buildReducedModel(full, locked, q_ref) if locked else full
        self.data = self.model.createData()
        if not self.model.existFrame(ee_frame):
            raise ValueError(f"EE frame '{ee_frame}' not found in URDF")
        self.ee = ee_frame
        self.arm_joints = list(arm_joints)
        self._qidx = {j: self.model.joints[self.model.getJointId(j)].idx_q for j in self.arm_joints}
        self.fix_limits(vel_scale)
        self.solver = solver or ("daqp" if "daqp" in qpsolvers.available_solvers else qpsolvers.available_solvers[0])
        self.ee_task = FrameTask(ee_frame, position_cost=position_cost, orientation_cost=orientation_cost, lm_damping=lm_damping, gain=gain)
        self.posture = PostureTask(cost=posture_cost)
        self.barrier = None
        self.pair_dmin = None
        if COLLISION_BARRIER and self.geom is not None:
            n_avail = len(self.geom.collisionPairs)
            if n_avail > 0:
                try:
                    if _SCBarrier is None:
                        raise ImportError("pink.barriers not importable")
                    arm_jids = {self.model.getJointId(j) for j in self.arm_joints if self.model.existJointName(j)}
                    on_arm = lambda gi: self.geom.geometryObjects[gi].parentJoint in arm_jids
                    self.pair_dmin = np.array([D_MIN_SELF if (on_arm(cp.first) and on_arm(cp.second)) else D_MIN
                                               for cp in self.geom.collisionPairs])
                    n_bar = n_avail if N_COLLISION_PAIRS <= 0 else min(N_COLLISION_PAIRS, n_avail)
                    self.barrier = MarginSelfCollisionBarrier(n_bar, gain=BARRIER_GAIN, safe_displacement_gain=BARRIER_SAFE_GAIN, d_min_vec=self.pair_dmin)
                    n_self = int(np.sum(self.pair_dmin == D_MIN_SELF))
                    self._info(f"MarginSelfCollisionBarrier dim={n_bar} of {n_avail} pairs (d_min={D_MIN}, d_min_self={D_MIN_SELF} on {n_self} self pairs, gain={BARRIER_GAIN})")
                except Exception as exc:
                    self._warn(f"pink.barriers unavailable ({exc}) -> falling back to collision reject")
            else:
                self._warn("collision_barrier on, but 0 collision pairs after filtering")
        if self.barrier is not None:
            self.configuration = Configuration(self.model, self.data, pin.neutral(self.model), collision_model=self.geom, collision_data=self.geom_data)
        else:
            self.configuration = Configuration(self.model, self.data, pin.neutral(self.model))
        self.posture.set_target(self.configuration.q)
        self.limits = [ConfigurationLimit(self.model), VelocityLimit(self.model)]
        self.retreats = 0

    def _info(self, msg):
        if self.log is not None:
            self.log.info(msg)

    def _warn(self, msg, throttle=None):
        if self.log is not None:
            if throttle is None:
                self.log.warn(msg)
            else:
                self.log.warn(msg, throttle_duration_sec=throttle)

    def _convexify(self):
        """Replace BVH collision meshes with their convex hulls: fast and reliable
        distance queries, required by the barrier (undefined on raw non-convex BVH).
        The table is skipped here and replaced by a box in _table_box_override."""
        try:
            import coal as fcl
        except Exception:
            import hppfcl as fcl
        def _to_convex(V):
            hull = ConvexHull(V)
            used = np.unique(np.concatenate([hull.vertices, hull.simplices.ravel()]))
            remap = {int(old): new for new, old in enumerate(used)}
            pv = fcl.StdVec_Vec3s()
            for p in V[used]:
                pv.append(np.asarray(p, float))
            tris = fcl.StdVec_Triangle()
            for s in hull.simplices:
                tris.append(fcl.Triangle(remap[int(s[0])], remap[int(s[1])], remap[int(s[2])]))
            return fcl.Convex(pv, tris)
        n_hull, failed = 0, []
        for go in self.geom.geometryObjects:
            g = go.geometry
            if not isinstance(g, fcl.BVHModelBase):
                continue
            if go.name == "table_link_0":
                continue
            V = np.asarray(g.vertices())
            done = False
            # coal.Convex rejects hulls with high-degree vertices ("Too many neighbors");
            # decimating on a coarser grid smooths those apexes away
            for grid in (0.0, 0.002, 0.005, 0.01):
                try:
                    Vg = V if grid == 0.0 else np.unique(np.round(V / grid) * grid, axis=0)
                    go.geometry = _to_convex(Vg)
                    n_hull += 1
                    done = True
                    if grid > 0.0:
                        self._info(f"hull {go.name}: decimated at {grid*1000:.0f}mm grid")
                    break
                except Exception:
                    continue
            if not done:
                failed.append(go.name)
                self._warn(f"hull FAILED for {go.name} -> left as raw BVH, distances unreliable")
        self._info(f"built {n_hull} convex hulls" + (f", failed: {failed}" if failed else ""))

    def _table_box_override(self):
        """Replace the huge non-convex table mesh with a flat box snapped to its top
        surface (the only part the arm can reach), so distance queries are exact."""
        try:
            import coal as fcl
        except Exception:
            import hppfcl as fcl
        for gid, go in enumerate(self.geom.geometryObjects):
            if go.name != "table_link_0":
                continue
            try:
                go.geometry.computeLocalAABB()
                al = go.geometry.aabb_local
                mn = np.array(al.min_); mx = np.array(al.max_)
                d = self.model.createData(); gd = self.geom.createData()
                pin.updateGeometryPlacements(self.model, d, self.geom, gd, pin.neutral(self.model))
                oMg = gd.oMg[gid]
                corners = np.array([[x, y, z] for x in (mn[0], mx[0]) for y in (mn[1], mx[1]) for z in (mn[2], mx[2])])
                wc = (oMg.rotation @ corners.T).T + oMg.translation
                wmin = wc.min(0); wmax = wc.max(0)
                sx = float(wmax[0] - wmin[0]) + 0.02
                sy = float(wmax[1] - wmin[1]) + 0.02
                sz = 0.10
                top = float(wmax[2])
                center = np.array([0.5 * (wmin[0] + wmax[0]), 0.5 * (wmin[1] + wmax[1]), top - 0.5 * sz])
                oMj = d.oMi[go.parentJoint]
                go.geometry = fcl.Box(sx, sy, sz)
                go.placement = oMj.inverse() * pin.SE3(np.eye(3), center)
                self._info(f"table override: mesh -> Box size=[{sx:.3f},{sy:.3f},{sz:.3f}] top={top:.3f}")
            except Exception as exc:
                self._warn(f"table override skipped: {exc}")
            break

    def _filter_pairs(self, arm_joints):
        """Collision-pair funnel: keep only pairs that involve the teleoperated arm;
        drop structurally-adjacent arm pairs (< SELF_MIN_HOPS joints apart); drop
        arm-vs-static pairs that are already near contact at the reference pose
        (permanent fixtures), except environment objects (table/walls)."""
        arm_jids = {self.model.getJointId(j) for j in arm_joints if self.model.existJointName(j)}
        on_arm = lambda gi: self.geom.geometryObjects[gi].parentJoint in arm_jids
        def _hops(a, b):
            chain = []
            x = a
            while True:
                chain.append(x)
                if x == 0:
                    break
                x = self.model.parents[x]
            depth = {j: i for i, j in enumerate(chain)}
            db, x = 0, b
            while x not in depth:
                x = self.model.parents[x]
                db += 1
            return depth[x] + db
        n0 = len(self.geom.collisionPairs)
        pairs = [pin.CollisionPair(cp.first, cp.second) for cp in self.geom.collisionPairs if on_arm(cp.first) or on_arm(cp.second)]
        n_moving = len(pairs)
        def _struct_neighbor(cp):
            if not (on_arm(cp.first) and on_arm(cp.second)):
                return False
            ja = self.geom.geometryObjects[cp.first].parentJoint
            jb = self.geom.geometryObjects[cp.second].parentJoint
            return _hops(ja, jb) < SELF_MIN_HOPS
        pairs = [cp for cp in pairs if not _struct_neighbor(cp)]
        n_topo = len(pairs)
        self.geom.removeAllCollisionPairs()
        for cp in pairs:
            self.geom.addCollisionPair(cp)
        # measure reference-pose distances to weed out permanently-near fixtures
        gd = self.geom.createData()
        dcol = self.model.createData()
        pin.computeDistances(self.model, dcol, self.geom, gd, pin.neutral(self.model))
        dmins = [gd.distanceResults[k].min_distance for k in range(len(self.geom.collisionPairs))]
        thresh = max(2.0 * D_MIN, DROP_DIST_THRESH)
        def _is_env(cp):
            na = self.geom.geometryObjects[cp.first].name.lower()
            nb = self.geom.geometryObjects[cp.second].name.lower()
            return any(k in na or k in nb for k in ENV_KEYWORDS)
        both_arm = lambda cp: on_arm(cp.first) and on_arm(cp.second)
        final_pairs = []
        for k, cp in enumerate(self.geom.collisionPairs):
            if (not both_arm(cp)) and (not _is_env(cp)) and dmins[k] < thresh:
                a = self.geom.geometryObjects[cp.first].name
                b = self.geom.geometryObjects[cp.second].name
                self._info(f"drop permanently-near pair {a}<->{b} (d={dmins[k]*1000:.0f}mm at reference)")
                continue
            final_pairs.append(pin.CollisionPair(cp.first, cp.second))
        self.geom.removeAllCollisionPairs()
        for cp in final_pairs:
            self.geom.addCollisionPair(cp)
        n_dist = len(final_pairs)
        n_reach = self._reachability_prune(arm_joints, on_arm, both_arm)
        self._info(f"collision pairs funnel: srdf={n0} -> moving={n_moving} -> topo={n_topo} -> dist={n_dist} -> reach={n_reach}")

    def _reachability_prune(self, arm_joints, on_arm, both_arm):
        """Drop arm-vs-static pairs whose bounding spheres can never come within
        PRUNE_MARGIN of each other over PRUNE_SAMPLES random arm configurations.
        Sphere distance lower-bounds true distance, so the prune is conservative.
        The barrier jacobian cost is linear in pair count — this is the speed lever."""
        if PRUNE_MARGIN <= 0.0 or PRUNE_SAMPLES <= 0 or len(self.geom.collisionPairs) == 0:
            return len(self.geom.collisionPairs)
        # bounding sphere (local center + radius) per geometry
        centers, radii = [], []
        for go in self.geom.geometryObjects:
            try:
                go.geometry.computeLocalAABB()
                al = go.geometry.aabb_local
                mn = np.array(al.min_); mx = np.array(al.max_)
                centers.append(0.5 * (mn + mx))
                radii.append(0.5 * float(np.linalg.norm(mx - mn)))
            except Exception:
                centers.append(np.zeros(3))
                radii.append(1e3)  # unknown extent -> effectively never prune this geom
        rng = np.random.default_rng(0)
        lo, hi = self.model.lowerPositionLimit, self.model.upperPositionLimit
        qidx = [self.model.joints[self.model.getJointId(j)].idx_q for j in arm_joints]
        d = self.model.createData()
        gd = self.geom.createData()
        q = pin.neutral(self.model)
        moving = [gi for gi in range(len(self.geom.geometryObjects)) if on_arm(gi)]
        static = [gi for gi in range(len(self.geom.geometryObjects)) if not on_arm(gi)]
        # static sphere centers in world (independent of arm config)
        pin.updateGeometryPlacements(self.model, d, self.geom, gd, q)
        c_static = {gi: gd.oMg[gi].rotation @ centers[gi] + gd.oMg[gi].translation for gi in static}
        # phase 1: trace world sphere-centers of every arm geom over the samples
        samples = rng.uniform([lo[i] for i in qidx], [hi[i] for i in qidx], size=(PRUNE_SAMPLES, len(qidx)))
        traces = {gi: np.empty((PRUNE_SAMPLES, 3)) for gi in moving}
        for si, s in enumerate(samples):
            for kk, i in enumerate(qidx):
                q[i] = s[kk]
            pin.forwardKinematics(self.model, d, q)
            pin.updateGeometryPlacements(self.model, d, self.geom, gd)
            for gi in moving:
                traces[gi][si] = gd.oMg[gi].rotation @ centers[gi] + gd.oMg[gi].translation
        # phase 2: per pair, vectorized lower bound of the achievable distance
        keep = []
        for cp in self.geom.collisionPairs:
            a, b = cp.first, cp.second
            if both_arm(cp):
                keep.append(pin.CollisionPair(a, b))
                continue
            ga, gb = (a, b) if on_arm(a) else (b, a)
            cb = c_static.get(gb)
            if cb is None:
                keep.append(pin.CollisionPair(a, b))
                continue
            dmin = float(np.min(np.linalg.norm(traces[ga] - cb, axis=1))) - radii[ga] - radii[gb]
            if dmin > PRUNE_MARGIN:
                continue
            keep.append(pin.CollisionPair(a, b))
        self.geom.removeAllCollisionPairs()
        for cp in keep:
            self.geom.addCollisionPair(cp)
        return len(keep)

    def fix_limits(self, vel_scale):
        """Replace non-finite position/velocity limits, apply extra per-joint clamps from RLIMITS,
        and scale velocity limits."""
        lo = np.array(self.model.lowerPositionLimit, float)
        hi = np.array(self.model.upperPositionLimit, float)
        lo[~np.isfinite(lo)] = -np.pi
        hi[~np.isfinite(hi)] = np.pi
        for j, (jlo, jhi) in RLIMITS.items():
            if j in self._qidx:
                qi = self._qidx[j]
                lo[qi] = max(lo[qi], float(jlo))
                hi[qi] = min(hi[qi], float(jhi))
        self.model.lowerPositionLimit, self.model.upperPositionLimit = lo, hi
        vl = np.array(self.model.velocityLimit, float)
        vl[~np.isfinite(vl) | (vl <= 0)] = np.pi
        self.model.velocityLimit = vl * float(vel_scale)

    def in_collision(self, q):
        """Return True if configuration q is in collision (False when no geometry is loaded)."""
        if self.geom is None:
            return False
        return bool(pin.computeCollisions(self.model, self.col_data, self.geom, self.geom_data, np.asarray(q, float), True))

    def qindex(self, joint_name):
        """Index of a named joint inside the configuration vector q."""
        return self._qidx[joint_name]

    def neutral(self):
        """Model neutral configuration."""
        return pin.neutral(self.model)

    @property
    def q(self):
        """Current configuration q (copy)."""
        return self.configuration.q.copy()

    def arm_positions(self):
        """Current positions of the arm joints, in ARM order."""
        q = self.configuration.q
        return np.array([q[self._qidx[j]] for j in self.arm_joints], float)

    def fk_rotation(self):
        """EE frame rotation in world for the current configuration."""
        return self.configuration.get_transform_frame_to_world(self.ee).rotation.copy()

    def fk_translation(self):
        """EE frame position in world for the current configuration."""
        return self.configuration.get_transform_frame_to_world(self.ee).translation.copy()

    def reset_to(self, qf):
        """Reset the configuration to qf and re-anchor the posture target there."""
        qf = np.clip(np.asarray(qf, float), self.model.lowerPositionLimit, self.model.upperPositionLimit)
        if self.barrier is not None:
            self.configuration = Configuration(self.model, self.data, qf, collision_model=self.geom, collision_data=self.geom_data)
        else:
            self.configuration = Configuration(self.model, self.data, qf)
        self.posture.set_target(self.configuration.q)

    def set_arm(self, values):
        """Overwrite arm joint positions in the internal configuration (used to pull the
        IK state back toward the measured robot when the command runs too far ahead)."""
        q = self.configuration.q.copy()
        for j, v in zip(self.arm_joints, values):
            q[self._qidx[j]] = float(v)
        q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
        self.configuration.update(q)

    def _floors(self):
        """Per-pair floor vector (d_min_self for self pairs, d_min for the rest)."""
        n = len(self.geom.collisionPairs)
        if self.pair_dmin is not None and len(self.pair_dmin) >= n:
            return self.pair_dmin[:n]
        return np.full(n, D_MIN)

    def min_gap(self):
        """(distance, margin, pair-name) of the LOWEST-MARGIN collision pair
        (margin = distance - that pair's floor) for the current configuration."""
        if self.geom is None or len(self.geom.collisionPairs) == 0:
            return float("inf"), float("inf"), ""
        cd = getattr(self.configuration, "collision_data", None)
        if cd is None:
            pin.computeDistances(self.model, self.col_data, self.geom, self.geom_data, self.configuration.q)
            cd = self.geom_data
        dists = np.array([cd.distanceResults[k].min_distance for k in range(len(self.geom.collisionPairs))])
        margins = dists - self._floors()
        k = int(np.argmin(margins))
        cp = self.geom.collisionPairs[k]
        name = self.geom.geometryObjects[cp.first].name + "<->" + self.geom.geometryObjects[cp.second].name
        return float(dists[k]), float(margins[k]), name

    def _min_margin_at(self, q):
        pin.computeDistances(self.model, self.col_data, self.geom, self.geom_data, np.asarray(q, float))
        n = len(self.geom.collisionPairs)
        if n == 0:
            return 1e9
        dists = np.array([self.geom_data.distanceResults[k].min_distance for k in range(n)])
        return float(np.min(dists - self._floors()))

    def step(self, target_pos, target_R, dt=DT):
        """One diff-IK step toward (target_pos, target_R). With the barrier the collision
        gap is a hard QP inequality (the arm dodges); on an infeasible solve we retreat
        along the outward barrier gradient instead of freezing. Without the barrier,
        fall back to reject-on-collision."""
        T = pin.SE3(np.asarray(target_R, float), np.asarray(target_pos, float))
        self.ee_task.set_target(T)
        q_prec = self.configuration.q.copy()
        lo, hi = self.model.lowerPositionLimit, self.model.upperPositionLimit

        if self.barrier is None:
            q_new = q_prec
            try:
                v = solve_ik(self.configuration, [self.ee_task, self.posture], dt, solver=self.solver, limits=self.limits, safety_break=False)
                q_new = pin.integrate(self.model, q_prec, v * dt)
            except Exception as exc:
                self._warn(f"IK solve skipped: {exc}", throttle=2.0)
            q_new = np.clip(q_new, lo, hi)
            if not np.isfinite(q_new).all():
                q_new = q_prec
            self.blocked = self.in_collision(q_new)
            if self.blocked:
                q_new = q_prec
            if not np.array_equal(q_new, self.configuration.q):
                self.configuration.update(q_new)
            return self.arm_positions()

        v, reason = None, None
        try:
            v = solve_ik(self.configuration, [self.ee_task, self.posture], dt, solver=self.solver, limits=self.limits, barriers=[self.barrier], safety_break=False)
        except PinkError as exc:
            reason = f"PinkError: {exc}"
        except Exception as exc:
            reason = f"{type(exc).__name__}: {exc}"
        if v is None and reason is None:
            reason = "solve_ik returned None"
        q_new = None
        if v is not None:
            q_new = np.clip(pin.integrate(self.model, q_prec, v * dt), lo, hi)
            if not np.isfinite(q_new).all():
                reason = "non-finite q from IK"
                q_new = None
        if q_new is None:
            self.retreats += 1
            self._warn(f"IK(barrier) infeasible -> retreat ({reason})", throttle=1.0)
            q_new = self._retreat(q_prec, dt)
        if not np.array_equal(q_new, self.configuration.q):
            self.configuration.update(q_new)
        # "near collision" = within (d_influence - d_min) of the pair's own floor,
        # so self pairs with a small floor don't flag it permanently
        self.blocked = self.min_gap()[1] < (D_INFLUENCE - D_MIN)
        return self.arm_positions()

    def _retreat(self, q_prec, dt):
        """Move away from the closest collision pair along the outward barrier gradient.
        The gradient sign is chosen live with a probe step against the min gap. Holds at
        q_prec (restoring distanceResults to q_prec) if the step is non-finite or would
        worsen the gap."""
        lo, hi = self.model.lowerPositionLimit, self.model.upperPositionLimit
        try:
            h = self.barrier.compute_barrier(self.configuration)
            J = self.barrier.compute_jacobian(self.configuration)
            g = J[int(np.argmin(h))]
            gn = float(np.linalg.norm(g))
            if gn < 1e-9:
                self._warn("retreat gradient ~0 -> holding", throttle=1.0)
                return q_prec
            g = g / gn
            g0 = self._min_margin_at(q_prec)
            q_probe = np.clip(pin.integrate(self.model, q_prec, g * 1e-3), lo, hi)
            if self._min_margin_at(q_probe) < g0:
                g = -g
            k_ret = 0.1 * float(np.max(self.model.velocityLimit))
            v_ret = np.clip(k_ret * g, -self.model.velocityLimit, self.model.velocityLimit)
            q_ret = np.clip(pin.integrate(self.model, q_prec, v_ret * dt), lo, hi)
            if not np.isfinite(q_ret).all():
                self._min_margin_at(q_prec)
                self._warn("retreat produced non-finite q -> holding", throttle=1.0)
                return q_prec
            if self._min_margin_at(q_ret) < g0:
                self._min_margin_at(q_prec)
                self._warn("retreat worsened gap -> holding", throttle=1.0)
                return q_prec
            return q_ret
        except Exception as exc:
            try:
                self._min_margin_at(q_prec)
            except Exception:
                pass
            self._warn(f"retreat failed ({exc}) -> holding", throttle=1.0)
            return q_prec

class Bridge(Node):
    """ROS2 node: HTC Vive controller (via /vive topics) -> Pink diff-IK -> mantis left arm."""
    def __init__(self):
        """Build IK, compute shoulder origin and reach-shell radii,
        set up publishers/subscribers/timers and shared teleop state."""
        super().__init__("vive_mantis_pink_bridge")
        self.ik = PinkIK(URDF, EE_FRAME, ARM, srdf_path=srdf_path(), package_dirs=mesh_pkg_dirs(), logger=self.get_logger())
        self.model = self.ik.model
        self.shoulder = self.shoulder_origin()
        mn, mx = self.reach_shell()
        self.r_min = mn + REACH_LO_FRAC * (mx - mn)
        self.r_max = mn + REACH_HI_FRAC * (mx - mn)
        self.get_logger().info(f"reach shell: r_min={self.r_min:.3f} r_max={self.r_max:.3f} (raw {mn:.3f}..{mx:.3f})")
        self.phase = "wait"
        self.pub = self.create_publisher(Float64MultiArray, ARM_CMD_TOPIC, 10)
        self.shaper = None
        if OUT_RATE > 0:
            self.shaper = CommandShaper(self.pub, OUT_RATE, OUT_VEL, OUT_ACCEL, OUT_KP)
            self.shaper.start()
            self.get_logger().info(f"CommandShaper: {OUT_RATE:.0f} Hz, v_max={OUT_VEL}, a_max={OUT_ACCEL}, kp={OUT_KP}")
        # best-effort keep-last on streams: never build a stale backlog. vive topics have
        # a single publisher -> depth 1 (latest only). /joint_states is SHARED by several
        # broadcasters (arms + hand) -> depth 5, or a burst from one publisher displaces
        # the other's message in a depth-1 queue and the arm freshness watchdog flaps.
        qos_latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        qos_js = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self.on_joint_states, qos_js)
        self.create_subscription(Float64MultiArray, VIVE_POSE_TOPIC, self.on_vive_pose, qos_latest)
        self.create_subscription(Float64MultiArray, VIVE_BUTTONS_TOPIC, self.on_vive_buttons, qos_latest)
        self.create_timer(DT, self.tick)
        self.create_timer(VR_DT, self.vr_tick)
        self.grip = ActionClient(self, GripperCommand, GRIP_ACTION)
        self._grip_last = None
        self._held_since = None
        self.Rc_ref = None
        self.R_anchor = None
        self._blk = 0
        self.pos = 0
        self.eff = 0
        self.dbg = 0
        self.shared = {"target": None, "home": None, "anchor": None, "ref": None, "engaged": False, "ready": False, "Rc": np.eye(3)}
        self._cur = None
        self._pose_t = 0.0
        self._pose_lost = False
        self._js_t = 0.0
        self._btn_t = 0.0
        self._R_ref = np.eye(3)
        self._R_des_prev = None
        self._pad_was = False
        self._pad_t = 0.0
        self._pad_now = False
        self._menu_was = False
        self._menu_now = False
        self._trig_now = 0.0
        self.mark = False
        self._blend_from = None
        self._blend_n = 0
        self._joints_lost = False
        self._last_cmd = None
        self._pos_filter = OneEuroFilter(FILTER_MIN_CUTOFF, FILTER_BETA)
        self._steptimes = []
        self._steptime_last = time.monotonic()
        self._retreats_last = 0
        self.get_logger().info("Bridge up. Waiting for robot...")

    def shoulder_origin(self):
        """World position of the shoulder-lift joint at neutral, used as the teleop workspace center."""
        q0 = self.ik.neutral()
        fk_data = self.model.createData()
        pin.forwardKinematics(self.model, fk_data, q0)
        pin.updateFramePlacements(self.model, fk_data)
        jid = self.model.getJointId(SHOULDER_JOINT)
        return fk_data.oMi[jid].translation.copy()

    def reach_shell(self, n=60000, seed=0):
        """Monte-Carlo sample the first four arm joints to estimate min/max EE distance
        from the shoulder (reach envelope)."""
        rng = np.random.default_rng(seed)
        lo_lim = self.model.lowerPositionLimit
        hi_lim = self.model.upperPositionLimit
        qidx = [self.ik.qindex(j) for j in ARM[:4]]
        q = self.ik.neutral()
        lo_d, hi_d = 1e9, 0.0
        samples = rng.uniform([lo_lim[i] for i in qidx], [hi_lim[i] for i in qidx], size=(n, len(qidx)))
        fk_data = self.model.createData()
        fid = self.model.getFrameId(EE_FRAME)
        for s in samples:
            for k, i in enumerate(qidx):
                q[i] = s[k]
            pin.forwardKinematics(self.model, fk_data, q)
            pin.updateFramePlacements(self.model, fk_data)
            d = np.linalg.norm(fk_data.oMf[fid].translation - self.shoulder)
            lo_d = min(lo_d, d)
            hi_d = max(hi_d, d)
        return lo_d, hi_d

    def on_vive_pose(self, msg):
        """Cache the controller pose: one-euro-filtered position + orthonormalized rotation."""
        d = msg.data
        if len(d) < 12:
            return
        arr = np.asarray(d[:12], float)
        if not np.isfinite(arr).all():
            return
        Rc = arr[3:].reshape(3, 3)
        U, _, Vt = np.linalg.svd(Rc)
        Rc = U @ Vt
        if np.linalg.det(Rc) < 0.0:
            Rc[:, -1] *= -1.0
        now = time.monotonic()
        # first pose after a stale gap: reset the filter, otherwise it blends the
        # pre-dropout position in and the anchor lands centimeters off the hand
        if self._cur is not None and (now - self._pose_t) >= POSE_TIMEOUT:
            self._pos_filter.t_prev = None
        self._cur = self._pos_filter(arr[:3], now)
        self._pose_t = now
        self.shared["Rc"] = Rc

    def on_vive_buttons(self, msg):
        d = msg.data
        if len(d) < 3:
            return
        self._btn_t = time.monotonic()
        self._trig_now = float(d[0])
        self._pad_now = d[1] > 0.5
        self._menu_now = d[2] > 0.5

    def yaw_frame(self, Rc):
        """Heading-only frame from the controller orientation: horizontal forward/right + true up.
        Makes the position mapping robust to how the controller is pitched/rolled at engage.
        A near-vertical nose carries no usable heading, so within yaw_fallback_deg of
        vertical the heading comes from the controller TOP axis instead (near-horizontal
        exactly then); its sign follows the nose so that "away from the user" stays
        forward: nose down -> top points away, nose up -> top points back."""
        Rc = np.asarray(Rc)
        up = np.array([0.0, 1.0, 0.0])
        nose = -Rc[:, 2]
        fwd = nose
        if np.linalg.norm(fwd - (fwd @ up) * up) < YAW_FALLBACK_SIN:
            fwd = -np.sign(nose @ up) * Rc[:, 1]
            self.get_logger().warn("yaw_frame: nose near-vertical -> heading taken from controller top axis")
        back_h = (fwd @ up) * up - fwd
        n = np.linalg.norm(back_h)
        if n < 1e-6:
            return np.eye(3)
        back_h = back_h / n
        right = np.cross(up, back_h)
        right = right / np.linalg.norm(right)
        return np.column_stack([right, up, back_h])

    def _capture_refs(self):
        """Anchor ALL refs (position + orientation) to the current controller pose and
        current EE, atomically. Used at engage and on recovery after a pose dropout."""
        self.shared["ref"] = self._cur.copy()
        self._R_ref = self.yaw_frame(self.shared["Rc"])
        self.shared["anchor"] = self.shared["target"].copy()
        self.Rc_ref = self.shared["Rc"].copy()
        self.R_anchor = self.ik.fk_rotation()
        self._R_des_prev = self.R_anchor.copy()

    def _joints_fresh(self):
        return isinstance(self.pos, dict) and (time.monotonic() - self._js_t) < JOINT_TIMEOUT and all(j in self.pos for j in ARM_CMD_JOINTS)

    def _on_engage(self):
        self.shared["engaged"] = True
        self._pose_lost = False
        self._blend_n = 0
        self._capture_refs()
        self.get_logger().info("ENGAGED")

    def _on_disengage(self):
        """Freeze: re-sync the IK state to the measured robot (kills any accumulated
        cmd/meas divergence), park the target on the resulting EE pose, and blend the
        command over a few ticks so there is no step."""
        self.shared["engaged"] = False
        if self._joints_fresh():
            self._blend_from = self.ik.arm_positions().copy()
            self._blend_n = BLEND_TICKS
            q = self.ik.neutral()
            for j in ARM:
                q[self.ik.qindex(j)] = self.pos[j]
            self.ik.reset_to(q)
        self.shared["target"] = self.ik.fk_translation()
        self._R_des_prev = None
        self.get_logger().info("FROZEN")

    def _ori_step(self, R_target):
        """Exponential smoothing of the desired orientation toward R_target + per-tick angular-step cap."""
        R_target = np.asarray(R_target, float)
        if self._R_des_prev is None:
            self._R_des_prev = R_target.copy()
            return self._R_des_prev
        w = ORI_ALPHA * pin.log3(R_target @ self._R_des_prev.T)
        ang = float(np.linalg.norm(w))
        if ang > MAX_ANG_STEP and ang > 1e-9:
            w = w * (MAX_ANG_STEP / ang)
        R_new = pin.exp3(w) @ self._R_des_prev
        self._R_des_prev = R_new
        return R_new

    def vr_tick(self):
        """250 Hz: move the target inside the reach shell while engaged (with dropout
        freeze/re-anchor), toggle clutch on trackpad (debounced; freeze always allowed,
        engage only on a fresh pose), mark on menu."""
        now = time.monotonic()
        fresh = self._cur is not None and (now - self._pose_t) < POSE_TIMEOUT

        if self.shared["ready"] and self.shared["engaged"]:
            if not fresh:
                if not self._pose_lost:
                    self._pose_lost = True
                    self.get_logger().warn("pose stale -> target frozen")
            else:
                if self._pose_lost:
                    self._pose_lost = False
                    self._capture_refs()
                    self.get_logger().warn("pose recovered -> re-anchored")
                dl = self._R_ref.T @ (self._cur - self.shared["ref"])
                d = dl[AXIS_MAP]
                newp = self.shared["anchor"] + SCALE * AXIS_SIGN * d
                arel = self.shared["anchor"] - self.shoulder
                rel0 = newp - self.shoulder
                az0 = np.arctan2(arel[1], arel[0])
                az = np.arctan2(rel0[1], rel0[0])
                # saturate so the amplified azimuth can't cross the antipode: the raw
                # wrap at +-pi would flip the target to the mirrored side of the base
                daz_max = np.pi / max(AZ_GAIN, 1.0)
                daz = np.clip((az - az0 + np.pi) % (2 * np.pi) - np.pi, -daz_max, daz_max)
                naz = az0 + AZ_GAIN * daz
                rh = np.hypot(rel0[0], rel0[1])
                if rh > 1e-3:
                    newp = self.shoulder + np.array([rh * np.cos(naz), rh * np.sin(naz), rel0[2]])
                rel = newp - self.shoulder
                r = np.linalg.norm(rel)
                if r < 1e-6:
                    newp = self.shared["target"]
                elif r > self.r_max:
                    newp = self.shoulder + rel * (self.r_max / r)
                elif r < self.r_min:
                    newp = self.shoulder + rel * (self.r_min / r)
                prev = self.shared["target"]
                stepv = newp - prev
                sn = np.linalg.norm(stepv)
                if sn > MAX_TARGET_STEP:
                    newp = prev + stepv * (MAX_TARGET_STEP / sn)
                ee = self.ik.fk_translation()
                lead = newp - ee
                ln = np.linalg.norm(lead)
                if ln > MAX_LEAD:
                    newp = ee + lead * (MAX_LEAD / ln)
                self.shared["target"] = newp

        pad = self._pad_now
        menu = self._menu_now
        if pad and not self._pad_was and (now - self._pad_t) > PAD_DEBOUNCE:
            # freeze must ALWAYS work (it needs no pose); only engage requires a fresh pose
            if self.shared["engaged"]:
                self._pad_t = now
                self._on_disengage()
            elif self.shared["ready"] and fresh:
                self._pad_t = now
                self._on_engage()
            else:
                self.get_logger().warn(f"PAD ignored: ready={self.shared['ready']} fresh={fresh} pose_age={now - self._pose_t:.2f}s")
        self._pad_was = pad
        if menu and not self._menu_was:
            self.mark = not self.mark
            self.get_logger().info(f"RECORD {'ON' if self.mark else 'OFF'}")
        self._menu_was = menu

    def full_cmd(self, positions):
        """Full ARM_CMD_JOINTS vector: teleoperated joints from `positions`, the
        rest filled from their measured positions. None while any joint is unknown."""
        cmd = dict(zip(ARM, positions))
        data = []
        for j in ARM_CMD_JOINTS:
            if j in cmd:
                data.append(float(cmd[j]))
            elif isinstance(self.pos, dict) and j in self.pos:
                data.append(float(self.pos[j]))
            else:
                return None
        return data

    def send_arm(self, positions):
        """Hand the command to the output shaper (which publishes at OUT_RATE), or
        publish directly when the shaper is disabled (out_rate: 0)."""
        data = self.full_cmd(positions)
        if data is None:
            return
        if self.shaper is not None:
            self.shaper.set_target(data)
            return
        msg = Float64MultiArray()
        msg.data = data
        self.pub.publish(msg)

    def send_grip(self, pos):
        """Send a gripper position goal via the action client; no-op if the server is not ready."""
        if not self.grip.server_is_ready():
            self.get_logger().warn("gripper action server not ready", throttle_duration_sec=2.0)
            return False
        goal = GripperCommand.Goal()
        goal.command.position = float(pos)
        goal.command.max_effort = GRIP_EFFORT
        self.grip.send_goal_async(goal)
        return True

    def on_joint_states(self, msg):
        """Cache measured joint positions/efforts (MERGED — several broadcasters may share
        /joint_states, e.g. the hand publishes its own partial messages); freshness is
        stamped only by messages that carry the teleoperated arm. On the first complete
        state, re-lock the collision model to the REAL parked pose of the other joints
        if it differs from locked_q, then sync IK to measured and enter teleop."""
        nm = dict(zip(msg.name, msg.position))
        if isinstance(self.pos, dict):
            self.pos.update(nm)
        else:
            self.pos = dict(nm)
        if msg.effort:
            em = dict(zip(msg.name, msg.effort))
            if isinstance(self.eff, dict):
                self.eff.update(em)
            else:
                self.eff = em
        if all(j in nm for j in ARM):
            # freshness = ARRIVAL time of a message carrying the arm joints. (Do NOT
            # use msg.header.stamp: Gazebo stamps sim-time, which vs the node's wall
            # clock yields a ~1e9 s "age" and makes joint_states look permanently stale.
            # depth-5 KEEP_LAST QoS already bounds any post-dropout backlog.)
            self._js_t = time.monotonic()
        if not all(j in self.pos for j in ARM_CMD_JOINTS):
            return
        if self.phase == "wait":
            mism = {n: self.pos[n] for n, v in self.ik.locked_ref.items()
                    if n in self.pos and abs(self.pos[n] - v) > 0.1}
            if mism:
                self.get_logger().warn(f"parked joints differ from locked_q by >0.1 rad ({sorted(mism)}); rebuilding collision model at the measured parked pose...")
                locked_q = dict(self.ik.locked_ref)
                locked_q.update({n: self.pos[n] for n in self.ik.locked_ref if n in self.pos})
                self.ik = PinkIK(URDF, EE_FRAME, ARM, srdf_path=srdf_path(), package_dirs=mesh_pkg_dirs(), locked_q=locked_q, logger=self.get_logger())
                self.model = self.ik.model
            q0 = self.ik.neutral()
            for name in ARM:
                q0[self.ik.qindex(name)] = self.pos[name]
            self.ik.reset_to(q0)
            self.shared["home"] = self.ik.fk_translation()
            self.shared["target"] = self.shared["home"].copy()
            self.shared["anchor"] = self.shared["home"].copy()
            self.shared["engaged"] = False
            self.shared["ready"] = True
            self.phase = "teleop"
            self.get_logger().info("Teleop ready. Click trackpad to engage.")

    def tick(self):
        """Main 100 Hz loop: run diff-IK toward the VR target (barrier dodges
        collisions), clamp the command against the measured robot, blend after a
        freeze, watchdog joint_states, and drive arm and gripper."""
        if self.phase != "teleop":
            return

        # joint_states watchdog: while measurements are stale, HOLD the IK integrator
        # (otherwise it free-runs away from the physically held robot and the recovery
        # tick would publish a violent multi-joint step); on recovery, re-sync to the
        # measured state exactly like a freeze does and re-anchor the controller refs.
        if not self._joints_fresh():
            self._joints_lost = True
            self.get_logger().warn("joint_states stale -> IK held", throttle_duration_sec=1.0)
            return
        if self._joints_lost:
            self._joints_lost = False
            self._blend_from = self.ik.arm_positions().copy()
            self._blend_n = BLEND_TICKS
            q = self.ik.neutral()
            for j in ARM:
                q[self.ik.qindex(j)] = self.pos[j]
            self.ik.reset_to(q)
            self.shared["target"] = self.ik.fk_translation()
            self._R_des_prev = None
            if self.shared["engaged"]:
                self._pose_lost = True  # forces _capture_refs() on the next fresh pose
            self.get_logger().warn("joint_states recovered -> re-synced to measured")

        tgt = self.shared["target"].copy()
        Rc = self.shared["Rc"].copy()
        engaged = self.shared["engaged"]

        if engaged and self.Rc_ref is not None:
            # Heading-normalize the orientation delta by the ENGAGE yaw frame (the
            # same _R_ref the position path uses). The raw world-frame delta axis
            # rotates with the controller's absolute heading, so the same wrist
            # gesture pitched the gripper DOWN at one engage heading and UP at the
            # opposite one (approach flip). NOTE: goes together with ori_sign
            # [1,1,1] — the old [-1,-1,1] was the 180deg-about-vertical compensator
            # for the raw VR-universe heading, which _R_ref now removes per-engage.
            dR = self._R_ref.T @ (Rc @ self.Rc_ref.T) @ self._R_ref
            w = ORI_SIGN * pin.log3(M @ dR @ M.T)
            R_des = pin.exp3(w) @ self.R_anchor
        else:
            R_des = self.ik.fk_rotation()
        R_des = self._ori_step(R_des)
        R_ee = self.ik.fk_rotation()
        w_lead = pin.log3(R_des @ R_ee.T)
        a_lead = float(np.linalg.norm(w_lead))
        if a_lead > MAX_ANG_LEAD and a_lead > 1e-9:
            R_des = pin.exp3(w_lead * (MAX_ANG_LEAD / a_lead)) @ R_ee
            self._R_des_prev = R_des

        t0 = time.perf_counter()
        q_arm = self.ik.step(tgt, R_des, DT)
        self._steptimes.append((time.perf_counter() - t0) * 1000.0)

        if MAX_JOINT_LEAD > 0 and self._joints_fresh() and all(j in self.pos for j in ARM):
            meas = np.array([self.pos[j] for j in ARM], float)
            lead = q_arm - meas
            mlead = float(np.max(np.abs(lead)))
            if mlead > MAX_JOINT_LEAD:
                # uniform scale keeps the command on the meas->cmd line (a per-joint box
                # clamp mixes a corner configuration the barrier never certified)
                self.ik.set_arm(meas + lead * (MAX_JOINT_LEAD / mlead))
                if self.ik.geom is not None and self.ik.min_gap()[1] < 0 and self._last_cmd is not None:
                    self.ik.set_arm(self._last_cmd)  # scaled cmd violates the barrier floor -> hold last safe cmd
                q_arm = self.ik.arm_positions()
                self.get_logger().warn("cmd ran ahead of robot -> clamped to measured + max_joint_lead", throttle_duration_sec=1.0)

        if self._blend_n > 0 and self._blend_from is not None:
            a = 1.0 - (self._blend_n - 1) / float(BLEND_TICKS)
            q_arm = (1.0 - a) * self._blend_from + a * q_arm
            self._blend_n -= 1

        self.dbg += 1
        if self.dbg % 50 == 0:
            gap_d, gap_m, gap_pair = self.ik.min_gap()
            print("blocked=%s min-gap=%+.4f margin=%+.4f %s" % (self.ik.blocked, gap_d, gap_m, gap_pair))
            for i, j in enumerate(ARM):
                cmd = float(q_arm[i])
                meas = self.pos.get(j) if isinstance(self.pos, dict) else None
                eff = self.eff.get(j) if isinstance(self.eff, dict) else None
                ms = "n/a" if meas is None else "%+.4f" % meas
                gs = "n/a" if meas is None else "%+.4f" % (cmd - meas)
                es = "n/a" if eff is None else "%+.3f" % eff
                print("%-26s cmd=%+.4f meas=%s gap=%s eff=%s" % (j, cmd, ms, gs, es))
        now_m = time.monotonic()
        if self._steptimes and now_m - self._steptime_last >= 10.0:
            a = np.array(self._steptimes)
            d_ret = self.ik.retreats - self._retreats_last
            self._retreats_last = self.ik.retreats
            self.get_logger().info(f"STEPTIME n={len(a)} p50={np.percentile(a,50):.2f} p95={np.percentile(a,95):.2f} max={a.max():.2f} ms | retreats +{d_ret}/10s")
            if self.shaper is not None:
                st = self.shaper.stats()
                if st is not None:
                    self.get_logger().info(f"OUTSTREAM n={st[0]} p50={st[1]:.2f} p95={st[2]:.2f} max={st[3]:.2f} ms")
            self._steptimes = []
            self._steptime_last = now_m
        if self.ik.blocked:
            self._blk += 1
            if self._blk % 100 == 1:
                self.get_logger().info("near collision -> dodging", throttle_duration_sec=1.0)
        else:
            self._blk = 0
        self._last_cmd = q_arm.copy()
        self.send_arm(q_arm)

        # gripper: only act on fresh controller input, so a controller/publisher dropout
        # latches the last grip state instead of auto-opening and dropping the object
        now = time.monotonic()
        if (now - self._btn_t) < BUTTONS_TIMEOUT:
            down = self._trig_now > 0.5
            if not down:
                self._held_since = None
            elif self._held_since is None:
                self._held_since = now
            want = down and (now - self._held_since) >= TRIG_HOLD
            if want != self._grip_last and self.send_grip(GRIP_CLOSE if want else GRIP_OPEN):
                self._grip_last = want


def main():
    """Init rclpy, spin the Bridge node, shut down ROS cleanly."""
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if getattr(node, "shaper", None) is not None:
            node.shaper.stop()
        node.destroy_node()
        rclpy.ok() and rclpy.shutdown()


if __name__ == "__main__":
    main()
