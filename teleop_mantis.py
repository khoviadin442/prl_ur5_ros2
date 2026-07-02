import os
import yaml
import time
import numpy as np
import pinocchio as pin
import qpsolvers
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from pink import Configuration, solve_ik
from pink.exceptions import PinkError
from pink.tasks import FrameTask, PostureTask
from pink.limits import ConfigurationLimit, VelocityLimit
from ament_index_python.packages import get_package_share_directory
from scipy.spatial import ConvexHull

path = os.environ.get("teleop_config", os.path.join(os.path.dirname(os.path.abspath(__file__)), "config_teleop_mantis.yaml"))
with open(path) as f:
    CFG = yaml.safe_load(f)

URDF = CFG["urdf"]
EE_FRAME = CFG["ee_frame"]
ARM = list(CFG["arm"])
GRIPPER_JOINT = CFG["gripper_joint"]
ARM_CMD_JOINTS = list(CFG["arm_cmd_joints"])

RATE = float(CFG["rate"])
DT = 1.0 / RATE

POSITION_COST = float(CFG["ik"]["position_cost"])
ORIENTATION_COST = float(CFG["ik"]["orientation_cost"])
LM_DAMPING = float(CFG["ik"]["lm_damping"])
TASK_GAIN = float(CFG["ik"]["task_gain"])
POSTURE_COST = float(CFG["ik"]["posture_cost"])
VEL_SCALE = float(CFG["ik"]["vel_scale"])
COLLISION_MARGIN = float(CFG["ik"]["collision_margin"])

SCALE = float(CFG["teleop"]["scale"])
AZ_GAIN = float(CFG["teleop"]["az_gain"])
REACH_LO_FRAC = float(CFG["teleop"]["reach_lo_frac"])
REACH_HI_FRAC = float(CFG["teleop"]["reach_hi_frac"])

AXIS_SIGN = np.array(CFG["teleop"]["axis_sign"])
AXIS_MAP = list(CFG["teleop"]["axis_map"])
M = np.zeros((3, 3))
for i in range(3):
    M[i, AXIS_MAP[i]] = AXIS_SIGN[i]

COLLISION_MODE   = CFG["teleop"].get("collision_mode", "reanchor")
MAX_TARGET_SPEED = float(CFG["teleop"].get("max_target_speed", 0.5))
MAX_LEAD         = float(CFG["teleop"].get("max_lead", 0.05))
MAX_ANG_LEAD     = float(CFG["teleop"].get("max_ang_lead", 0.15))
MAX_ANG_SPEED    = float(CFG["teleop"].get("max_ang_speed", 2.0))
ORI_ALPHA        = float(CFG["teleop"].get("ori_alpha", 0.6))
FILTER_MIN_CUTOFF= float(CFG["teleop"].get("filter_min_cutoff", 1.0))
FILTER_BETA      = float(CFG["teleop"].get("filter_beta", 0.007))
POSE_TIMEOUT     = float(CFG["teleop"].get("pose_timeout", 0.2))
JOINT_TIMEOUT    = float(CFG["teleop"].get("joint_timeout", 0.3))
BLEND_TICKS      = int(CFG["teleop"].get("disengage_blend_ticks", 5))
VR_DT            = 1.0 / 250.0
MAX_TARGET_STEP  = MAX_TARGET_SPEED * VR_DT
MAX_ANG_STEP     = MAX_ANG_SPEED * DT
ORI_SIGN         = np.array(CFG["teleop"].get("ori_sign", [1.0, 1.0, 1.0]), float)
COLLISION_BARRIER = bool(CFG["teleop"].get("collision_barrier", True))
D_INFLUENCE       = float(CFG["teleop"].get("d_influence", 0.04))
D_MIN             = float(CFG["teleop"].get("d_min", 0.01))
BARRIER_GAIN      = float(CFG["teleop"].get("barrier_gain", 100.0))
BARRIER_SAFE_GAIN = float(CFG["teleop"].get("barrier_safe_gain", 1.0))
SELF_MIN_HOPS     = int(CFG["teleop"].get("self_collision_min_hops", 3))
DROP_DIST_THRESH  = float(CFG["teleop"].get("drop_dist_thresh", 0.03))
N_COLLISION_PAIRS = int(CFG["teleop"].get("n_collision_pairs", 40))
DIAG_NEAR = 0.025
DIAG_DUMP_PERIOD = 10.0 

# HOME_TIME = float(CFG["home"]["time"])
# HOME_Q = list(CFG["home"]["q"])

GRIP_OPEN = float(CFG["gripper"]["grip_open"])
GRIP_CLOSE = float(CFG["gripper"]["grip_close"])
TRIG_TIMEOUT = float(CFG["gripper"]["trig_timeout"])
TRIG_HOLD = float(CFG["gripper"]["trig_hold"])
GRIP_ACTION = CFG["gripper"]["action"]
GRIP_EFFORT = float(CFG["gripper"]["effort"])

ARM_CMD_TOPIC = CFG["topics"]["arm_cmd"]
JOINT_STATES_TOPIC = CFG["topics"]["joint_states"]

RLIMITS = {k: tuple(v) for k,v in CFG["rlimits"].items()}

def mesh_pkg_dirs():
    """Package dirs used to resolve mesh paths referenced by the URDF."""
    prefix = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
    return [os.path.join(p, "share") for p in prefix if p]

def srdf_path():
    """Path to the MoveIt SRDF, used to disable allowed collision pairs."""
    return CFG["srdf"]

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

class PinkIK:
    """Pinocchio model + Pink differential IK for the SO-100 arm with self-collision checking."""
    def __init__(self, urdf_path, ee_frame, arm_joints, gripper_joint=None, position_cost=POSITION_COST, orientation_cost=ORIENTATION_COST,lm_damping=LM_DAMPING, gain=TASK_GAIN, posture_cost=POSTURE_COST, vel_scale=VEL_SCALE, solver=None, srdf_path=None, package_dirs=None, collision_margin=COLLISION_MARGIN, collision_mode=COLLISION_MODE, logger=None):
        """Build the gripper-locked model and collision geometry, set up frame/posture tasks,
        joint limits and the QP solver."""
        self.log = logger
        self.collision_mode = collision_mode
        full = pin.buildModelFromUrdf(urdf_path)
        locked = []
        keep = set(arm_joints)
        locked = [full.getJointId(n) for n in full.names[1:] if n not in keep]
        self.geom = None
        self.blocked = False
        if srdf_path and package_dirs:
            try:
                geom_full = pin.buildGeomFromUrdf(full, urdf_path, pin.GeometryType.COLLISION, package_dirs=list(package_dirs))
            except Exception as e:
                if logger is not None:
                    logger.error(f"COLLISION GEOM FAILED({e}) -> collision DISABLED")
                raise RuntimeError("collision geometry failed to load - refusing to run without collision protection")
            self.geom = None
            if locked:
                self.model, self.geom = pin.buildReducedModel(full, geom_full, locked, pin.neutral(full))
            else:
                self.model, self.geom = full, geom_full
            try:
                import coal as fcl
            except Exception:
                import hppfcl as fcl
            from scipy.spatial import ConvexHull
            n_hull = 0
            failed_links = []
            for go in self.geom.geometryObjects:
                g = go.geometry
                if not isinstance(g, fcl.BVHModelBase):
                    continue
                if go.name == "table_link_0":
                    continue
                try:
                    V = np.asarray(g.vertices())
                    n_before = len(V)
                    hull = ConvexHull(V)
                    used = np.unique(np.concatenate([hull.vertices, hull.simplices.ravel()]))
                    remap = {int(old): new for new, old in enumerate(used)}
                    pv = fcl.StdVec_Vec3s()
                    for p in V[used]:
                        pv.append(np.asarray(p, float))
                    tris = fcl.StdVec_Triangle()
                    for s in hull.simplices:
                        tris.append(fcl.Triangle(remap[int(s[0])], remap[int(s[1])], remap[int(s[2])]))
                    go.geometry = fcl.Convex(pv, tris)
                    n_after = go.geometry.num_points
                    n_hull += 1
                    if logger is not None:
                        logger.info(f"hull {go.name}: {n_before} -> {n_after}")
                        if n_after >= n_before:
                            logger.warn(f"hull {go.name}: NOT reduced ({n_before} -> {n_after}) - possible reuse/degenerate")
                except Exception as e:
                    failed_links.append(go.name)
                    if logger is not None:
                        logger.error(f"hull FAILED for {go.name} ({e}) -> link distance-UNSAFE, left as raw BVH")
            if logger is not None:
                logger.info(f"built {n_hull} convex hulls")
                if failed_links:
                    logger.error(f"HULL FAILURES ({len(failed_links)}): {failed_links} -> these links left as raw BVH, self-collision distances unreliable")
            for _gid, _go in enumerate(self.geom.geometryObjects):
                if _go.name != "table_link_0":
                    continue
                try:
                    _go.geometry.computeLocalAABB()
                    _al = _go.geometry.aabb_local
                    _mn = np.array(_al.min_); _mx = np.array(_al.max_)
                    _d = self.model.createData(); _gd = self.geom.createData()
                    pin.updateGeometryPlacements(self.model, _d, self.geom, _gd, pin.neutral(self.model))
                    _oMg = _gd.oMg[_gid]
                    _corners = np.array([[x, y, z] for x in (_mn[0], _mx[0]) for y in (_mn[1], _mx[1]) for z in (_mn[2], _mx[2])])
                    _wc = (_oMg.rotation @ _corners.T).T + _oMg.translation
                    _wmin = _wc.min(0); _wmax = _wc.max(0)
                    _sx = float(_wmax[0] - _wmin[0]) + 0.02
                    _sy = float(_wmax[1] - _wmin[1]) + 0.02
                    _sz = 0.10
                    _top = float(_wmax[2])
                    _center = np.array([0.5 * (_wmin[0] + _wmax[0]), 0.5 * (_wmin[1] + _wmax[1]), _top - 0.5 * _sz])
                    _Rw = _oMg.rotation.copy() if not np.allclose(_oMg.rotation, np.eye(3)) else np.eye(3)
                    _oMj = _d.oMi[_go.parentJoint]
                    _go.geometry = fcl.Box(_sx, _sy, _sz)
                    _go.placement = _oMj.inverse() * pin.SE3(_Rw, _center)
                    if logger is not None:
                        logger.info(f"table override: Convex -> Box size={[round(_sx,3), round(_sy,3), round(_sz,3)]} center={[round(float(_center[0]),3), round(float(_center[1]),3), round(float(_center[2]),3)]}")
                except Exception as e:
                    if logger is not None:
                        logger.warn(f"table override skipped: {e}")
                break
            self.geom.addAllCollisionPairs()
            pin.removeCollisionPairs(self.model, self.geom, srdf_path, False)
            arm_jids = {self.model.getJointId(j) for j in arm_joints}
            on_arm = lambda gi: self.geom.geometryObjects[gi].parentJoint in arm_jids
            both_on_arm = lambda cp: on_arm(cp.first) and on_arm(cp.second)
            if logger is not None:
                for i, go in enumerate(self.geom.geometryObjects):
                    logger.info(f"geom[{i}] {go.name} type={type(go.geometry).__name__} joint={go.parentJoint}")
                for cp in self.geom.collisionPairs:
                    a = self.geom.geometryObjects[cp.first].name
                    b = self.geom.geometryObjects[cp.second].name
                    logger.info(f"PAIR {a} <-> {b}")
            def _hops(a, b):
                """Number of joints between joint ids a and b in the kinematic tree."""
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
                if not both_on_arm(cp):
                    return False
                ja = self.geom.geometryObjects[cp.first].parentJoint
                jb = self.geom.geometryObjects[cp.second].parentJoint
                return _hops(ja, jb) < SELF_MIN_HOPS
            pairs = [cp for cp in pairs if not _struct_neighbor(cp)]
            n_topo = len(pairs)

            self.geom.removeAllCollisionPairs()
            for cp in pairs:
                self.geom.addCollisionPair(cp)
            _gd = self.geom.createData()
            _tmp_cfg = Configuration(self.model, self.model.createData(), pin.neutral(self.model), collision_model=self.geom, collision_data=_gd)
            _tmp_cfg.update(pin.neutral(self.model))
            _dmins = [_gd.distanceResults[k].min_distance for k in range(len(self.geom.collisionPairs))]

            _thresh = max(2.0 * D_MIN, DROP_DIST_THRESH)
            _ENV_KW = ("table", "wall", "floor", "ground")
            def _is_env(cp):
                na = self.geom.geometryObjects[cp.first].name.lower()
                nb = self.geom.geometryObjects[cp.second].name.lower()
                return any(k in na or k in nb for k in _ENV_KW)
            final_pairs = []
            for k, cp in enumerate(self.geom.collisionPairs):
                if (not both_on_arm(cp)) and (not _is_env(cp)) and _dmins[k] < _thresh:
                    continue
                final_pairs.append(pin.CollisionPair(cp.first, cp.second))

            self.geom.removeAllCollisionPairs()
            for cp in final_pairs:
                self.geom.addCollisionPair(cp)
            n_final = len(self.geom.collisionPairs)
            if logger is not None:
                logger.info(f"collision pairs funnel: all={n0} -> moving={n_moving} -> topo={n_topo} -> dist={n_final}")
            
            self.geom_data = self.geom.createData()
            for k in range(len(self.geom_data.collisionRequests)):
                self.geom_data.collisionRequests[k].security_margin = float(collision_margin)
            self.col_data = self.model.createData()
        else:
            self.model = pin.buildReducedModel(full, locked, pin.neutral(full)) if locked else full
        self.data = self.model.createData()
        try:
            from pink.barriers import SelfCollisionBarrier
            self._has_barrier = True
            self._SCB = SelfCollisionBarrier
        except Exception as e:
            self._has_barrier = False
            self._SCB = None
            if logger is not None:
                logger.error(f"pink.barriers unavailable ({e}) — collision barrier disabled, falling back to reject")
        self.barrier = None
        if COLLISION_BARRIER and self._has_barrier and self.geom is not None:
            n_avail = len(self.geom.collisionPairs)
            if n_avail > 0:
                n_bar = n_avail if N_COLLISION_PAIRS <= 0 else min(N_COLLISION_PAIRS, n_avail)
                self.barrier = self._SCB(n_bar, gain=BARRIER_GAIN, safe_displacement_gain=BARRIER_SAFE_GAIN, d_min=D_MIN)
                if logger is not None:
                    logger.info(f"SelfCollisionBarrier dim={n_bar} of {n_avail} pairs "f"(d_min={D_MIN}, d_infl={D_INFLUENCE} indicator-only)")
            elif logger is not None:
                logger.warn("collision_barrier on, but 0 collision pairs after filtering — barrier inactive")
        if self.model.nq != self.model.nv and logger is not None:
            logger.warn(f"model.nq={self.model.nq} != nv={self.model.nv}: continuous joints present; ""scalar q-indexing and clipping may be wrong — review before hardware")
        if not self.model.existFrame(ee_frame):
            raise ValueError(f"EE frame '{ee_frame}' not found in URDF")
        self.ee = ee_frame
        self.arm_joints = list(arm_joints)
        self._qidx = {j: self.model.joints[self.model.getJointId(j)].idx_q for j in self.arm_joints}
        self.fix_limits(vel_scale)
        self.solver = solver or ("daqp" if "daqp" in qpsolvers.available_solvers else qpsolvers.available_solvers[0])
        self.ee_task = FrameTask(ee_frame, position_cost=position_cost, orientation_cost=orientation_cost,lm_damping=lm_damping, gain=gain)
        self.posture = PostureTask(cost=posture_cost)
        if self.barrier is not None:
            self.configuration = Configuration(self.model, self.data, pin.neutral(self.model), collision_model=self.geom, collision_data=self.geom_data)
        else:
            self.configuration = Configuration(self.model, self.data, pin.neutral(self.model))
        self.posture.set_target(self.configuration.q)
        self.limits = [ConfigurationLimit(self.model), VelocityLimit(self.model)]
        self._diag_t0 = time.monotonic()
        self._diag_prev_t = None
        self._diag_last_dump = self._diag_t0
        self._diag_near = {}
        self._diag_retreats = 0
        self._diag_retreats_last = 0
        if self.geom is not None and self.log is not None:
            self.log.info(f"DIAG collisionPairs={len(self.geom.collisionPairs)} (N_COLLISION_PAIRS={N_COLLISION_PAIRS}, d_min={D_MIN})")

    def fix_limits(self, vel_scale):
        """Replace non-finite position/velocity limits, apply extra per-joint clamps from RLIMITS,
        and scale velocity limits."""
        lo = np.array(self.model.lowerPositionLimit, float)
        hi = np.array(self.model.upperPositionLimit, float)
        lo[~np.isfinite(lo)] = -np.pi
        hi[~np.isfinite(hi)] = np.pi
        for j, (jlo,jhi) in RLIMITS.items():
            if j in self._qidx:
                qi = self._qidx[j]
                lo[qi] = max(lo[qi], float(jlo))
                hi[qi] = min(hi[qi], float(jhi))
        self.model.lowerPositionLimit, self.model.upperPositionLimit = lo, hi
        vl = np.array(self.model.velocityLimit, float)
        vl[~np.isfinite(vl) | (vl <= 0)] = np.pi
        self.model.velocityLimit = vl * float(vel_scale)

    def in_collision(self, q, report=False):
        """Return True if configuration q is in self-collision (False when no geometry is loaded)."""
        if self.geom is None:
            return False
        hit = bool(pin.computeCollisions(self.model, self.col_data, self.geom, self.geom_data, np.asarray(q, float), False))
        if hit and report and self.log is not None:
            pairs = set()
            for k in range(len(self.geom.collisionPairs)):
                if self.geom_data.collisionResults[k].isCollision():
                    cp = self.geom.collisionPairs[k]
                    pairs.add(self.geom.geometryObjects[cp.first].name + " <-> " + self.geom.geometryObjects[cp.second].name)
            key = tuple(sorted(pairs))
            if key != getattr(self, "_last_coll", None):
                self.log.info("collision: " + " ; ".join(key), throttle_duration_sec=1.0)
                self._last_coll = key
        return hit

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
        if self.barrier is not None:
            self.configuration = Configuration(self.model, self.data, np.asarray(qf, float), collision_model=self.geom, collision_data=self.geom_data)
        else:
            self.configuration = Configuration(self.model, self.data, np.asarray(qf, float))
        self.posture.set_target(self.configuration.q)

    def _min_gap(self,q):
        pin.computeDistances(self.model, self.col_data, self.geom, self.geom_data, np.asarray(q,float))
        return min((self.geom_data.distanceResults[k].min_distance for k in range(len(self.geom.collisionPairs))), default = 1e9)

    def step(self, target_pos, target_R, dt=DT):
        """One diff-IK step toward (target_pos, target_R). The CBF barrier enforces the
        collision gap as a hard QP inequality; on an infeasible/None/NaN solve we retreat
        along the outward gradient of the closest pair instead of freezing."""
        T = pin.SE3(np.asarray(target_R, float), np.asarray(target_pos, float))
        self.ee_task.set_target(T)
        q_prec = self.configuration.q.copy()
        lo, hi = self.model.lowerPositionLimit, self.model.upperPositionLimit
        if self.barrier is None:
            try:
                v = solve_ik(self.configuration, [self.ee_task, self.posture], dt, solver=self.solver, limits=self.limits, safety_break=False)
            except Exception as exc:
                v = np.zeros(self.model.nv)
                if self.log is not None:
                    self.log.warn(f"IK solve skipped: {exc}", throttle_duration_sec=2.0)
            q_new = np.clip(pin.integrate(self.model, q_prec, v * dt), lo, hi)
            if not np.isfinite(q_new).all():
                q_new = q_prec
            if not np.array_equal(q_new, self.configuration.q):
                self.configuration.update(q_new)
            self._log_min_gap()
            return self.arm_positions()

        v = None
        reason = None
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
            self._diag_retreats += 1
            if self.log is not None:
                self.log.warn(f"IK(barrier) infeasible -> retreat ({reason})", throttle_duration_sec=1.0)
            q_new = self._retreat(q_prec, dt)

        if not np.array_equal(q_new, self.configuration.q):
            self.configuration.update(q_new)
        self.blocked = self._near_collision()
        self._log_min_gap()
        return self.arm_positions()

    def _retreat(self, q_prec, dt):
        """Move away from the closest collision pair along the outward barrier gradient.
        The gradient direction is checked at runtime against self._min_gap (probe step),
        so the sign is chosen live rather than hard-coded. Logs the closest pair, the gap
        before/after and the chosen sign. Holds at q_prec (restoring distanceResults to
        q_prec, since step() skips configuration.update on a hold) if the step is non-finite
        or would WORSEN the gap."""
        lo, hi = self.model.lowerPositionLimit, self.model.upperPositionLimit
        try:
            h = self.barrier.compute_barrier(self.configuration)
            J = self.barrier.compute_jacobian(self.configuration)
            k = int(np.argmin(h))
            g = J[k]
            gn = float(np.linalg.norm(g))
            kg = min(range(len(self.geom.collisionPairs)), key=lambda i: self.geom_data.distanceResults[i].min_distance)
            _cp = self.geom.collisionPairs[kg]
            pair_name = (self.geom.geometryObjects[_cp.first].name + "<->" + self.geom.geometryObjects[_cp.second].name)
            if gn < 1e-9:
                if self.log is not None:
                    self.log.error(f"retreat gradient ~0 (pair {pair_name}) -> holding", throttle_duration_sec=1.0)
                return q_prec
            g = g / gn
            g0 = self._min_gap(q_prec)
            q_probe = np.clip(pin.integrate(self.model, q_prec, g * 1e-3), lo, hi)
            sign = 1.0
            if self._min_gap(q_probe) < g0:
                g = -g
                sign = -1.0
            k_ret = 0.1 * float(np.max(self.model.velocityLimit))
            v_ret = np.clip(k_ret * g, -self.model.velocityLimit, self.model.velocityLimit)
            q_ret = np.clip(pin.integrate(self.model, q_prec, v_ret * dt), lo, hi)
            if not np.isfinite(q_ret).all():
                self._min_gap(q_prec)
                if self.log is not None:
                    self.log.error("retreat produced non-finite q -> holding", throttle_duration_sec=1.0)
                return q_prec
            g1 = self._min_gap(q_ret)
            if self.log is not None:
                self.log.warn(f"retreat pair={pair_name} sign={sign:+.0f}" f"gap {g0*1000:+.1f}->{g1*1000:+.1f} mm", throttle_duration_sec=0.5)
            if g1 < g0:
                self._min_gap(q_prec)
                if self.log is not None:
                    self.log.error(f"retreat worsened gap ({g0*1000:+.1f}->{g1*1000:+.1f} mm) -> holding", throttle_duration_sec=0.5)
                return q_prec
            return q_ret
        except Exception as exc:
            try:
                self._min_gap(q_prec)
            except Exception:
                pass
            if self.log is not None:
                self.log.error(f"retreat failed ({exc}) -> holding", throttle_duration_sec=1.0)
            return q_prec

    def _log_min_gap(self):
        """TEMP telemetry: min gap + closest pair, per-pair dwell below DIAG_NEAR, retreat rate."""
        if self.geom is None or self.log is None:
            return
        try:
            n = len(self.geom.collisionPairs)
            if n == 0:
                return
            if getattr(self.configuration, "collision_data", None) is None:
                pin.computeDistances(self.model, self.col_data, self.geom, self.geom_data, self.configuration.q)
            now = time.monotonic()
            dt = 0.0 if self._diag_prev_t is None else now - self._diag_prev_t
            self._diag_prev_t = now
            dists = [self.geom_data.distanceResults[k].min_distance for k in range(n)]
            if 0.0 < dt < 0.2:
                for k in range(n):
                    if dists[k] < DIAG_NEAR:
                        cp = self.geom.collisionPairs[k]
                        nm = self.geom.geometryObjects[cp.first].name + "<->" + self.geom.geometryObjects[cp.second].name
                        self._diag_near[nm] = self._diag_near.get(nm, 0.0) + dt
            k = int(np.argmin(dists))
            cp = self.geom.collisionPairs[k]
            a = self.geom.geometryObjects[cp.first].name
            b = self.geom.geometryObjects[cp.second].name
            self.log.info(f"min-gap {dists[k]:+.4f} m  {a} <-> {b}", throttle_duration_sec=0.3)
            if now - self._diag_last_dump >= DIAG_DUMP_PERIOD:
                span_min = (now - self._diag_t0) / 60.0
                d_ret = self._diag_retreats - self._diag_retreats_last
                self._diag_retreats_last = self._diag_retreats
                top = sorted(self._diag_near.items(), key=lambda kv: -kv[1])[:5]
                tops = "; ".join(f"{nm} {t:.1f}s" for nm, t in top) or "none"
                self.log.info(f"DIAG near<{DIAG_NEAR}: {tops} | retreats +{d_ret}/{DIAG_DUMP_PERIOD:.0f}s (cum {self._diag_retreats}, {self._diag_retreats/max(span_min,1e-6):.1f}/min over {span_min:.1f}min)")
                self._diag_last_dump = now
        except Exception as e:
            if self.log is not None:
                self.log.warn(f"diag _log_min_gap failed: {e}", throttle_duration_sec=5.0)

    def _near_collision(self):
        """True if the smallest collision-pair gap is below D_INFLUENCE (indicator for logs/haptics)."""
        cd = getattr(self.configuration, "collision_data", None)
        if cd is None or self.geom is None:
            return False
        try:
            npairs = len(self.geom.collisionPairs)
            dmin = min((cd.distanceResults[k].min_distance for k in range(npairs)), default=1e9)
            return dmin < D_INFLUENCE
        except Exception:
            return False

class Bridge(Node):
    """ROS2 node: HTC Vive controller -> Pink diff-IK -> SO-100 arm and gripper."""
    def __init__(self):
        """Build IK, compute shoulder origin and reach-shell radii,
        set up publishers/subscribers/timers and shared teleop state."""
        super().__init__("vive_so100_pink_bridge")
        self.ik = PinkIK(URDF, EE_FRAME, ARM, srdf_path=srdf_path(), package_dirs=mesh_pkg_dirs(), collision_mode=COLLISION_MODE, logger=self.get_logger())
        self.model = self.ik.model
        self.shoulder = self.shoulder_origin()
        mn,mx = self.reach_shell()
        self.r_min = mn + REACH_LO_FRAC * (mx - mn)
        self.r_max = mn + REACH_HI_FRAC * (mx - mn)
        self.phase = "wait"
        self.t_home = None
        self.q_start = None
        self.pub = self.create_publisher(Float64MultiArray, ARM_CMD_TOPIC, 10)
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self.on_joint_states, 10)
        self.create_timer(DT, self.tick)
        self.create_timer(1.0/250.0, self.vr_tick)
        self.grip = ActionClient(self, GripperCommand, GRIP_ACTION)
        self._grip_last = None
        self._held_since = None
        self._trig_down_t = 0.0
        self._was_engaged = False
        self.Rc_ref = None
        self.R_anchor = None
        self._blk =  0
        self.pos = 0
        self.eff = 0
        self.dbg = 0
        self.shared = {"target": None, "home": None, "anchor": None, "ref": None, "engaged": False, "ready": False, "Rc": np.eye(3)}
        self._cur = None
        self._R_ref = np.eye(3)
        self._pad_was = False
        self._menu_was = False
        self._pad_now = False
        self._menu_now = False
        self._trig_now = 0.0
        self.mark = False
        self._pose_t = 0.0
        self._pose_lost = False
        self._js_t = 0.0
        self._R_des_prev = None
        self._blend_from = None
        self._blend_n = 0
        self._steptimes = []
        self._steptime_last = time.monotonic()
        self._lead_ratios = []
        self._ang_lead_ratios = []
        self._ee_speeds = []
        self._prev_ee = None
        self._pos_filter = OneEuroFilter(FILTER_MIN_CUTOFF, FILTER_BETA)
        self.create_subscription(Float64MultiArray, "/vive/pose", self.on_vive_pose, 10)
        self.create_subscription(Float64MultiArray, "/vive/buttons", self.on_vive_buttons, 10)
        self.get_logger().info("Bridge up. Waiting for robot...")

    def shoulder_origin(self):
        """World position of the Shoulder_Pitch joint at neutral, used as the teleop workspace center."""
        q0 = self.ik.neutral()
        fk_data = self.model.createData()
        pin.forwardKinematics(self.model, fk_data, q0)
        pin.updateFramePlacements(self.model, fk_data)
        jid = self.model.getJointId("left_shoulder_lift_joint")
        return fk_data.oMi[jid].translation.copy()

    def reach_shell(self, n=60000, seed=0):
        """Monte-Carlo sample the first three joints to estimate min/max EE distance
        from the shoulder (reach envelope)."""
        rng = np.random.default_rng(seed)
        lo_lim = self.model.lowerPositionLimit
        hi_lim = self.model.upperPositionLimit
        qidx = [self.ik.qindex(j) for j in ARM[:3]]
        q = self.ik.neutral()
        lo_d, hi_d = 1e9, 0.0
        samples = rng.uniform([lo_lim[i] for i in qidx], [hi_lim[i] for i in qidx], size=(n, 3))
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
        self._pose_t = time.monotonic()
        self._cur = self._pos_filter(arr[:3], self._pose_t)
        self.shared["Rc"] = Rc

    def on_vive_buttons(self, msg):
        d = msg.data
        if len(d) < 3:
            return
        self._trig_now = float(d[0])
        self._pad_now = d[1] > 0.5
        self._menu_now = d[2] > 0.5

    def _yaw_frame(self, Rc):
        """Heading-only frame from the controller orientation: horizontal forward/right + true up.
        Makes the position mapping robust to how the controller is pitched/rolled at engage."""
        up = np.array([0.0, 1.0, 0.0])
        nose = -np.asarray(Rc)[:, 2]
        back_h = (nose @ up) * up - nose
        n = np.linalg.norm(back_h)
        if n < 1e-6:
            return np.eye(3)
        back_h = back_h / n
        right = np.cross(up, back_h)
        right = right / np.linalg.norm(right)
        return np.column_stack([right, up, back_h])

    def _capture_refs(self):
        """Anchor ALL refs (position + orientation) to the current controller pose and current EE.
        Used at engage and on recovery after a pose dropout."""
        if self._cur is None:
            return
        self.shared["ref"] = self._cur.copy()
        self._R_ref = self._yaw_frame(self.shared["Rc"])
        self.shared["anchor"] = self.shared["target"].copy()
        self.Rc_ref = self.shared["Rc"].copy()
        self.R_anchor = self.ik.fk_rotation()
        self._R_des_prev = self.R_anchor.copy()

    def _ori_step(self, R_target):
        """Exponential smoothing of the desired orientation toward R_target + per-tick angular-step cap."""
        R_target = np.asarray(R_target, float)
        if self._R_des_prev is None:
            self._R_des_prev = R_target.copy()
            return self._R_des_prev
        R_prev = self._R_des_prev
        w = ORI_ALPHA * pin.log3(R_target @ R_prev.T)
        ang = float(np.linalg.norm(w))
        if ang > MAX_ANG_STEP and ang > 1e-9:
            w = w * (MAX_ANG_STEP / ang)
        R_new = pin.exp3(w) @ R_prev
        self._R_des_prev = R_new
        return R_new

    def vr_tick(self):
        cur = self._cur
        now = time.monotonic()
        fresh = cur is not None and (now - self._pose_t) < POSE_TIMEOUT

        if self.shared["ready"] and self.shared["engaged"]:
            if not fresh:
                if not self._pose_lost:
                    self._pose_lost = True
                    self.get_logger().warn("pose stale -> target frozen", throttle_duration_sec=1.0)
            else:
                if self._pose_lost:
                    self._pose_lost = False
                    self._capture_refs()
                    self.get_logger().warn("pose recovered -> re-anchored")
                dl = self._R_ref.T @ (cur - self.shared["ref"])
                d = dl[AXIS_MAP]
                off = SCALE * AXIS_SIGN * d
                newp = self.shared["anchor"] + off
                arel = self.shared["anchor"] - self.shoulder
                rel0 = newp - self.shoulder
                az0 = np.arctan2(arel[1], arel[0])
                az = np.arctan2(rel0[1], rel0[0])
                daz = (az - az0 + np.pi) % (2 * np.pi) - np.pi
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
        if pad and not self._pad_was:
            if self.shared["ready"] and fresh:
                if not self.shared["engaged"]:
                    self.shared["engaged"] = True
                    self._pose_lost = False
                    self._blend_n = 0
                    self._capture_refs()
                    self.get_logger().info("ENGAGED")
                else:
                    self.shared["engaged"] = False
                    self.get_logger().info("FROZEN")
            else:
                age = now - self._pose_t
                self.get_logger().warn(f"PAD ignored: ready={self.shared['ready']} fresh={fresh} pose_age={age:.2f}s")
        self._pad_was = pad

        if menu and not self._menu_was:
            self.mark = not self.mark
            self.get_logger().info(f"RECORD {'ON' if self.mark else 'OFF'}")
        self._menu_was = menu

    def send_arm(self, positions):
        """Publish arm joint positions to the arm controller."""
        cmd = dict(zip(ARM, positions))
        data = []
        for j in ARM_CMD_JOINTS:
            if j in cmd:
                data.append(cmd[j])
            elif isinstance(self.pos, dict) and j in self.pos:
                data.append(self.pos[j])
            else:
                return
        msg = Float64MultiArray()
        msg.data = [float(x) for x in data]
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
        """Cache measured joint positions/efforts and start homing once all arm joints are known."""
        self._js_t = time.monotonic()
        nm = dict(zip(msg.name, msg.position))
        self.pos = nm
        self.eff = dict(zip(msg.name, msg.effort)) if msg.effort else {}
        if not all(j in nm for j in ARM_CMD_JOINTS):
            return
        if self.phase == "wait":
            q0 = self.ik.neutral()
            for name in ARM:
                q0[self.ik.qindex(name)] = nm[name]
            self.ik.reset_to(q0)
            self.shared["home"] = self.ik.fk_translation()
            self.shared["target"] = self.shared["home"].copy()
            self.shared["anchor"] = self.shared["home"].copy()
            self.shared["engaged"] = False
            self.shared["ready"] = True
            self.phase = "teleop"
            self.get_logger().info("Teleop ready. Click trackpad to engage.")

    def tick(self):
        """Main 100 Hz loop: run diff-IK toward the VR target, handle collision (Variant A reanchor),
        blend on disengage, watchdog /joint_states, and drive arm and gripper."""
        if self.phase != "teleop":
            return
        tgt = self.shared["target"].copy()
        Rc = self.shared["Rc"].copy()
        engaged = self.shared["engaged"]

        if (not engaged) and self._was_engaged and all(j in self.pos for j in ARM):
            self._blend_from = self.ik.arm_positions().copy()
            self._blend_n = BLEND_TICKS
            q_hold = self.ik.neutral()
            for name in ARM:
                q_hold[self.ik.qindex(name)] = self.pos[name]
            self.ik.reset_to(q_hold)
            self.shared["target"] = self.ik.fk_translation()
            tgt = self.shared["target"].copy()
        self._was_engaged = engaged

        if engaged and self.Rc_ref is not None:
            w = ORI_SIGN * pin.log3(M @ (Rc @ self.Rc_ref.T) @ M.T)
            R_des = pin.exp3(w) @ self.R_anchor
        else:
            R_des = self.ik.fk_rotation()
        R_des = self._ori_step(R_des)
        R_ee = self.ik.fk_rotation()
        w_lead = pin.log3(R_des @ R_ee.T)
        a_lead = float(np.linalg.norm(w_lead))
        self._ang_lead_ratios.append(a_lead / MAX_ANG_LEAD)
        if a_lead > MAX_ANG_LEAD and a_lead > 1e-9:
            R_des = pin.exp3(w_lead * (MAX_ANG_LEAD / a_lead)) @ R_ee
            self._R_des_prev = R_des

        _ee = self.ik.fk_translation()
        self._lead_ratios.append(float(np.linalg.norm(tgt - _ee)) / MAX_LEAD)
        if self._prev_ee is not None:
            self._ee_speeds.append(float(np.linalg.norm(_ee - self._prev_ee)) / DT)
        self._prev_ee = _ee

        _t_step0 = time.perf_counter()
        q_arm = self.ik.step(tgt, R_des, DT)
        self._steptimes.append((time.perf_counter() - _t_step0) * 1000.0)
        _t_now = time.monotonic()
        if self._steptimes and _t_now - self._steptime_last >= 10.0:
            _a = np.array(self._steptimes)
            self.get_logger().info(f"STEPTIME n={len(_a)} p50={np.percentile(_a,50):.2f} " f"p95={np.percentile(_a,95):.2f} max={_a.max():.2f} ms")
            if self._lead_ratios:
                _lr = np.array(self._lead_ratios)
                self.get_logger().info(f"LEAD n={len(_lr)} p50={np.percentile(_lr,50):.2f} " f"p95={np.percentile(_lr,95):.2f} sat>0.95={float(np.mean(_lr > 0.95)):.2f}")
            if self._ang_lead_ratios:
                _ar = np.array(self._ang_lead_ratios)
                self.get_logger().info(f"ANGLEAD n={len(_ar)} p50={np.percentile(_ar,50):.2f} " f"p95={np.percentile(_ar,95):.2f} sat>0.95={float(np.mean(_ar > 0.95)):.2f}")
            if self._ee_speeds:
                _es = np.array(self._ee_speeds) * 100.0
                self.get_logger().info(f"EESPEED n={len(_es)} p50={np.percentile(_es,50):.2f} " f"p95={np.percentile(_es,95):.2f} cm/s")
            self._steptimes = []
            self._lead_ratios = []
            self._ang_lead_ratios = []
            self._ee_speeds = []
            self._steptime_last = _t_now

        if self._blend_n > 0 and self._blend_from is not None:
            a = 1.0 - (self._blend_n - 1)/ float(BLEND_TICKS)
            q_arm = (1.0 - a) * self._blend_from + a * q_arm
            self._blend_n -= 1
        if (time.monotonic() - self._js_t) < JOINT_TIMEOUT:
            self.send_arm(q_arm)
        else:
            self.get_logger().warn("joint_states stale -> arm command held", throttle_duration_sec=1.0)

        now = time.monotonic()
        down = self._trig_now > 0.5
        if not down:
            self._held_since = None
        elif self._held_since is None:
            self._held_since = now
        want = down and (now - self._held_since) >= TRIG_HOLD
        if want != self._grip_last and self.send_grip(GRIP_CLOSE if want else GRIP_OPEN):
            self._grip_last = want

def main():
    """Init rclpy, spin the Bridge node, shut down VR and ROS cleanly."""
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.ok() and rclpy.shutdown()

if __name__ == "__main__":
    main()
