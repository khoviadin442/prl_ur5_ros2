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
from pink.tasks import FrameTask, PostureTask
from pink.limits import ConfigurationLimit, VelocityLimit
from ament_index_python.packages import get_package_share_directory

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
            geom_full = pin.buildGeomFromUrdf(full, urdf_path, pin.GeometryType.COLLISION, package_dirs=list(package_dirs))
            if locked:
                self.model, self.geom = pin.buildReducedModel(full, geom_full, locked, pin.neutral(full))
            else:
                self.model, self.geom = full, geom_full
            self.geom.addAllCollisionPairs()
            pin.removeCollisionPairs(self.model, self.geom, srdf_path, False)
            arm_jids = {self.model.getJointId(j) for j in arm_joints}
            mv = lambda gi: self.geom.geometryObjects[gi].parentJoint in arm_jids
            keep_pairs = [pin.CollisionPair(cp.first, cp.second) for cp in self.geom.collisionPairs if mv(cp.first) or mv(cp.second)]
            n_before = len(self.geom.collisionPairs)
            self.geom.removeAllCollisionPairs()
            for cp in keep_pairs:
                self.geom.addCollisionPair(cp)
            if logger is not None:
                logger.info(f"collision pairs: {n_before} -> {len(self.geom.collisionPairs)}")
            self.geom_data = self.geom.createData()
            for k in range(len(self.geom_data.collisionRequests)):
                self.geom_data.collisionRequests[k].security_margin = float(collision_margin)
            self.col_data = self.model.createData()
        else:
            self.model = pin.buildReducedModel(full, locked, pin.neutral(full)) if locked else full
        self.data = self.model.createData()
        if self.model.nq != self.model.nv and logger is not None:
            logger.warn(f"model.nq={self.model.nq} != nv={self.model.nv}: continuous joints present; "
                        "scalar q-indexing and clipping may be wrong — review before hardware")
        if not self.model.existFrame(ee_frame):
            raise ValueError(f"EE frame '{ee_frame}' not found in URDF")
        self.ee = ee_frame
        self.arm_joints = list(arm_joints)
        self._qidx = {j: self.model.joints[self.model.getJointId(j)].idx_q for j in self.arm_joints}
        self.fix_limits(vel_scale)
        self.solver = solver or ("daqp" if "daqp" in qpsolvers.available_solvers else qpsolvers.available_solvers[0])
        self.ee_task = FrameTask(ee_frame, position_cost=position_cost, orientation_cost=orientation_cost,lm_damping=lm_damping, gain=gain)
        self.posture = PostureTask(cost=posture_cost)
        self.configuration = Configuration(self.model, self.data, pin.neutral(self.model))
        self.posture.set_target(self.configuration.q)
        self.limits = [ConfigurationLimit(self.model), VelocityLimit(self.model)]

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
        self.configuration = Configuration(self.model, self.data, np.asarray(qf, float))
        self.posture.set_target(self.configuration.q)

    def step(self, target_pos, target_R, dt=DT):
        """One diff-IK step toward (target_pos, target_R): solve, clamp to limits,
        handle collision per mode (reanchor rejects, slide line-searches), return new arm positions."""
        T = pin.SE3(np.asarray(target_R, float), np.asarray(target_pos, float))
        self.ee_task.set_target(T)
        q_prec = self.configuration.q.copy()
        lo, hi = self.model.lowerPositionLimit, self.model.upperPositionLimit
        v = np.zeros(self.model.nv)
        try:
            v = solve_ik(self.configuration, [self.ee_task, self.posture], dt, solver=self.solver, limits=self.limits, safety_break=False)
        except Exception as exc:
            if self.log is not None:
                self.log.warn(f"IK solve skipped: {exc}", throttle_duration_sec=2.0)
        q_full = np.clip(pin.integrate(self.model, q_prec, v * dt), lo, hi)

        if not np.isfinite(q_full).all():
            q_new, self.blocked = q_prec, False
        elif self.in_collision(q_full, report=True):
            if self.collision_mode == "slide":
                a_lo, a_hi = 0.0, 1.0
                for _ in range(8):
                    mid = 0.5 * (a_lo + a_hi)
                    if self.in_collision(np.clip(pin.integrate(self.model, q_prec, v * dt * mid), lo, hi)):
                        a_hi = mid
                    else:
                        a_lo = mid
                q_new = np.clip(pin.integrate(self.model, q_prec, v * dt * a_lo), lo, hi)
            else:
                q_new = q_prec
            self.blocked = True
        else:
            q_new, self.blocked = q_full, False

        if not np.array_equal(q_new, self.configuration.q):
            self.configuration.update(q_new)
        return self.arm_positions()

class Bridge(Node):
    """ROS2 node: HTC Vive controller -> Pink diff-IK -> SO-100 arm and gripper."""
    def __init__(self):
        """Build IK, compute shoulder origin and reach-shell radii,
        set up publishers/subscribers/timers and shared teleop state."""
        super().__init__("vive_so100_pink_bridge")
        self.ik = PinkIK(URDF, EE_FRAME, ARM, srdf_path=srdf_path(), package_dirs=mesh_pkg_dirs(),
                         collision_mode=COLLISION_MODE, logger=self.get_logger())
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

        q_arm = self.ik.step(tgt, R_des, DT)

        if self.ik.blocked and engaged and COLLISION_MODE == "reanchor":
            self.shared["target"] = self.ik.fk_translation()
            self._capture_refs()

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
