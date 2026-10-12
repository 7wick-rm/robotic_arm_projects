"""
SimpleEnvTools: y_env.py rewritten for
  * 4 tools (wrench / screwdriver / hammer / pliers) + 2 trays instead of mug + plate
  * OMY arm on a linear actuator (prismatic 'slide_joint', see asset/robotis_omy/omy_slider.xml)
  * language instruction per episode

Vector layouts (IMPORTANT, these go into the LeRobot dataset):
  ctrl   (MuJoCo, 11) = [j1..j6, rh_r1, rh_r2, rh_l1, rh_l2, slide]     <- order of <actuator> in the XML
  state  (8)          = [j1..j6, slide_pos, gripper(0/1)]
  action (8)          = [j1..j6, slide_cmd, gripper_cmd]                 <- 'joint_angle' action space
  eef_pose action (7) = [dx,dy,dz, droll,dpitch,dyaw, gripper]           <- only used by the scripted expert
  obj_init (16)       = 4 tools x [x, y, z, yaw]   (not used for training, only for replay)
"""
import copy
import glfw
import numpy as np
from mujoco_env.mujoco_parser import MuJoCoParserClass
from mujoco_env.utils import add_title_to_img, rotation_matrix
from mujoco_env.ik import solve_ik
from mujoco_env.transforms import rpy2r, r2rpy

TOOLS = ['wrench', 'screwdriver', 'hammer', 'pliers']
PROMPT = "pick up the {} and place it in the tray"      # use EXACTLY this at train and test time

SLIDE_LIM = 0.35                                         # must match range in omy_slider.xml
SLOT_XY = np.array([[-0.09, -0.07], [-0.09, 0.07], [0.09, -0.07], [0.09, 0.07]])  # 2x2 slots, tray frame
TRAY_FLOOR_DZ = 0.02                                     # tray floor half height in tools_scene.xml


class SimpleEnvTools:
    def __init__(self, xml_path, action_type='eef_pose', state_type='joint_angle', seed=None):
        """action_type: 'eef_pose' (scripted expert) or 'joint_angle' (replay / policy deployment)."""
        assert action_type in ('eef_pose', 'joint_angle')
        self.env = MuJoCoParserClass(name='Tabletop', rel_xml_path=xml_path)
        self.action_type = action_type
        self.state_type = state_type
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.instruction = ''
        self.slide_mode = 'follow'   # 'follow': carriage tracks target y (scripted expert); 'manual': keys J/L (teleop)
        self.slide_delta = 0.0
        self.init_viewer()
        self.reset(seed)

    # ------------------------------------------------------------------ setup
    def init_viewer(self):
        self.env.reset()
        self.env.init_viewer(distance=2.0, elevation=-30, transparent=False, black_sky=True,
                             use_rgb_overlay=False, loc_rgb_overlay='top right')

    def reset(self, seed=None, n_pre_in_dst=0):
        """Home the robot, put the 4 tools into random slots of the SOURCE tray.
        n_pre_in_dst: this many random tools start already inside the destination tray (distractors)."""
        if seed is not None:
            np.random.seed(seed)                       # (y_env.py had seed=0 here by mistake)
        q_init = np.deg2rad([0, 0, 0, 0, 0, 0])
        q_zero, _, _ = solve_ik(env=self.env, joint_names_for_ik=self.joint_names, body_name_trgt='tcp_link',
                                q_init=q_init, p_trgt=np.array([0.3, 0.0, 1.0]),
                                R_trgt=rpy2r(np.deg2rad([90, -0., 90])))
        # also put the slider back to 0
        self.env.forward(q=np.concatenate([q_zero, [0.0]]), joint_names=self.joint_names + ['slide_joint'],
                         increase_tick=False)
        try:                                           # kill leftover velocities from the previous episode
            self.env.data.qvel[:] = 0
        except AttributeError:
            pass

        self._place_tools(n_pre_in_dst)
        self.env.forward(increase_tick=False)

        self.q_home6 = copy.deepcopy(q_zero)
        self.slide_cmd = 0.0
        self.slide_delta = 0.0
        self.q = np.concatenate([q_zero, np.zeros(4), [0.0]])      # full ctrl vector (11)
        self.compute_q = q_zero
        self.p0, self.R0 = self.env.get_pR_body(body_name='tcp_link')
        self.obj_init_pose = self.get_obj_pose()
        for _ in range(100):
            self.step_env()
        self.gripper_state = False
        print("DONE INITIALIZATION")

    def _tray_xyz(self, which):
        return self.env.get_p_body('body_tray_' + which)

    def _slot_pose(self, which, slot, jitter=True):
        t = self._tray_xyz(which)
        j = np.random.uniform([-0.004, -0.01], [0.004, 0.01]) if jitter else np.zeros(2)
        xy = t[:2] + SLOT_XY[slot] + j
        return np.array([xy[0], xy[1], t[2] + TRAY_FLOOR_DZ + 0.02])   # drop slightly above the floor

    def _place_tools(self, n_pre_in_dst):
        order = np.random.permutation(len(TOOLS))
        src_slots = np.random.permutation(4)
        dst_slots = np.random.permutation(4)
        for k, ti in enumerate(order):
            if k < n_pre_in_dst:
                p = self._slot_pose('dst', dst_slots[k])
            else:
                p = self._slot_pose('src', src_slots[k - n_pre_in_dst])
            yaw = np.deg2rad(np.random.uniform(-12, 12))
            self.set_tool_pose(TOOLS[ti], p, yaw)

    def set_tool_pose(self, tool, p, yaw=0.0):
        body = 'body_obj_' + tool
        self.env.set_p_base_body(body_name=body, p=p)
        self.env.set_R_base_body(body_name=body, R=rpy2r(np.array([0, 0, yaw])))

    # ------------------------------------------------------------------ stepping
    def step(self, action):
        """eef_pose action (7,)  or joint_angle action (8,). Returns the state BEFORE physics advances."""
        if self.action_type == 'eef_pose':
            q = self.env.get_qpos_joints(joint_names=self.joint_names)
            self.p0 += action[:3]
            self.R0 = self.R0.dot(rpy2r(action[3:6]))
            if self.slide_mode == 'manual':
                # teleop: J/L move the whole arm sideways; the tcp target travels with the carriage
                new = float(np.clip(self.slide_cmd + self.slide_delta, -SLIDE_LIM, SLIDE_LIM))
                self.p0[1] += new - self.slide_cmd
                self.slide_cmd = new
            else:
                self.slide_cmd = float(np.clip(self.p0[1], -SLIDE_LIM, SLIDE_LIM))   # carriage follows target y
            q, _, _ = solve_ik(env=self.env, joint_names_for_ik=self.joint_names, body_name_trgt='tcp_link',
                               q_init=q, p_trgt=self.p0, R_trgt=self.R0, max_ik_tick=50, ik_stepsize=1.0,
                               ik_eps=1e-2, ik_th=np.radians(5.0), render=False, verbose_warning=False)
            g = float(action[-1])
        else:  # joint_angle
            q = np.asarray(action[:6], dtype=np.float64)
            self.slide_cmd = float(np.clip(action[6], -SLIDE_LIM, SLIDE_LIM))
            g = float(np.clip(action[7], 0.0, 1.0))
        gripper_cmd = np.array([g] * 4)
        gripper_cmd[[1, 3]] *= 0.8
        self.compute_q = q
        self.q = np.concatenate([q, gripper_cmd, [self.slide_cmd]])
        return self.get_joint_state()

    def step_env(self):
        self.env.step(self.q)

    # ------------------------------------------------------------------ observations
    def get_joint_state(self):
        qpos = self.env.get_qpos_joints(joint_names=self.joint_names)
        slide = self.env.get_qpos_joint('slide_joint')
        grip = self.env.get_qpos_joint('rh_r1')
        return np.concatenate([qpos, slide, [1.0 if grip[0] > 0.5 else 0.0]]).astype(np.float32)

    def get_action_vec(self):
        """8-D joint-space action that reproduces the last commanded ctrl (what we record)."""
        return np.concatenate([self.q[:6], [self.q[10]], [self.q[6]]]).astype(np.float32)

    def home_action_vec(self):
        return np.concatenate([self.q_home6, [0.0], [0.0]]).astype(np.float32)

    def grab_image(self):
        self.rgb_agent = self.env.get_fixed_cam_rgb(cam_name='agentview')
        self.rgb_ego = self.env.get_fixed_cam_rgb(cam_name='egocentric')
        self.rgb_side = self.env.get_fixed_cam_rgb(cam_name='sideview')
        return self.rgb_agent, self.rgb_ego

    def render(self, teleop=False, idx=None):
        self.env.plot_time()
        p, R = self.env.get_pR_body(body_name='tcp_link')
        R = R @ np.array([[1, 0, 0], [0, 0, 1], [0, 1, 0]])
        self.env.plot_sphere(p=p, r=0.02, rgba=[0.95, 0.05, 0.05, 0.5])
        self.env.plot_capsule(p=p, R=R, r=0.01, h=0.2, rgba=[0.05, 0.95, 0.05, 0.5])
        self.env.viewer_rgb_overlay(add_title_to_img(self.rgb_agent, text='Agent View', shape=(640, 480)), loc='top right')
        self.env.viewer_rgb_overlay(add_title_to_img(self.rgb_ego, text='Egocentric View', shape=(640, 480)), loc='bottom right')
        if teleop:
            self.env.viewer_rgb_overlay(add_title_to_img(self.rgb_side, text='Side View', shape=(640, 480)), loc='top left')
            self.env.viewer_text_overlay(text1='Episode', text2=str(idx))
            self.env.viewer_text_overlay(text1='Key Pressed', text2='%s' % (self.env.get_key_pressed_list()))
        self.env.viewer_text_overlay(text1='Task', text2=self.instruction)
        self.env.render()

    # ------------------------------------------------------------------ keyboard teleop
    def teleop_robot(self):
        """
        Keyboard teleop (use with action_type='eef_pose' and env.slide_mode='manual').
          W/S : x  (W = towards robot, S = away)     A/D : y  (arm only, base stays)
          R/F : z up / down
          J/L : LINEAR ACTUATOR  left (-y) / right (+y)  (moves the whole arm)
          Q/E : yaw      arrows : pitch / roll
          SPACE : toggle gripper        Z : discard episode and reset
        returns: action (7,) [dpos, drot, gripper], reset(bool). Slider motion is passed via self.slide_delta.
        """
        dpos = np.zeros(3)
        drot = np.eye(3)
        self.slide_delta = 0.0
        k = self.env
        if k.is_key_pressed_repeat(key=glfw.KEY_S): dpos += np.array([0.007, 0.0, 0.0])
        if k.is_key_pressed_repeat(key=glfw.KEY_W): dpos += np.array([-0.007, 0.0, 0.0])
        if k.is_key_pressed_repeat(key=glfw.KEY_A): dpos += np.array([0.0, -0.007, 0.0])
        if k.is_key_pressed_repeat(key=glfw.KEY_D): dpos += np.array([0.0, 0.007, 0.0])
        if k.is_key_pressed_repeat(key=glfw.KEY_R): dpos += np.array([0.0, 0.0, 0.007])
        if k.is_key_pressed_repeat(key=glfw.KEY_F): dpos += np.array([0.0, 0.0, -0.007])
        if k.is_key_pressed_repeat(key=glfw.KEY_J): self.slide_delta -= 0.01
        if k.is_key_pressed_repeat(key=glfw.KEY_L): self.slide_delta += 0.01
        if k.is_key_pressed_repeat(key=glfw.KEY_LEFT):  drot = rotation_matrix(angle=0.1 * 0.3, direction=[0.0, 1.0, 0.0])[:3, :3]
        if k.is_key_pressed_repeat(key=glfw.KEY_RIGHT): drot = rotation_matrix(angle=-0.1 * 0.3, direction=[0.0, 1.0, 0.0])[:3, :3]
        if k.is_key_pressed_repeat(key=glfw.KEY_DOWN):  drot = rotation_matrix(angle=0.1 * 0.3, direction=[1.0, 0.0, 0.0])[:3, :3]
        if k.is_key_pressed_repeat(key=glfw.KEY_UP):    drot = rotation_matrix(angle=-0.1 * 0.3, direction=[1.0, 0.0, 0.0])[:3, :3]
        if k.is_key_pressed_repeat(key=glfw.KEY_Q): drot = rotation_matrix(angle=0.1 * 0.3, direction=[0.0, 0.0, 1.0])[:3, :3]
        if k.is_key_pressed_repeat(key=glfw.KEY_E): drot = rotation_matrix(angle=-0.1 * 0.3, direction=[0.0, 0.0, 1.0])[:3, :3]
        if k.is_key_pressed_once(key=glfw.KEY_Z):
            self.slide_delta = 0.0
            return np.zeros(7, dtype=np.float32), True
        if k.is_key_pressed_once(key=glfw.KEY_SPACE):
            self.gripper_state = not self.gripper_state
        drot = r2rpy(drot)
        action = np.concatenate([dpos, drot, np.array([self.gripper_state], dtype=np.float32)], dtype=np.float32)
        return action, False

    # ------------------------------------------------------------------ task logic
    def set_instruction(self, text):
        self.instruction = text

    def get_obj_pose(self):
        """16-D: 4 x [x,y,z,yaw]"""
        out = []
        for t in TOOLS:
            p, R = self.env.get_pR_body('body_obj_' + t)
            out += [*p, r2rpy(R)[2]]
        return np.array(out, dtype=np.float32)

    def set_obj_pose(self, vec16):
        v = np.asarray(vec16, dtype=np.float64).reshape(4, 4)
        for t, row in zip(TOOLS, v):
            self.set_tool_pose(t, row[:3], row[3])
        self.step_env()

    def tool_in_tray(self, tool, which='dst'):
        p = self.env.get_p_body('body_obj_' + tool)
        t = self._tray_xyz(which)
        return bool(abs(p[0] - t[0]) < 0.17 and abs(p[1] - t[1]) < 0.13 and -0.02 < p[2] - t[2] < 0.10)

    def tools_in_dst(self):
        return [t for t in TOOLS if self.tool_in_tray(t, 'dst')]

    def free_dst_slots(self):
        d = self._tray_xyz('dst')
        used = set()
        for t in self.tools_in_dst():
            p = self.env.get_p_body('body_obj_' + t)
            used.add(int(np.argmin(np.linalg.norm(d[:2] + SLOT_XY - p[:2], axis=1))))
        return [s for s in range(4) if s not in used]

    def check_success(self, tool):
        """tool is inside the destination tray, gripper released, tcp lifted above 0.9 m."""
        if not self.tool_in_tray(tool, 'dst'):
            return False
        if self.env.get_qpos_joint('rh_r1')[0] > 0.1:
            return False
        return bool(self.env.get_p_body('tcp_link')[2] > 0.9)
