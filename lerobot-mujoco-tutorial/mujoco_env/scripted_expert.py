"""Scripted pick-and-place expert for SimpleEnvTools (action_type='eef_pose', slide_mode='follow').
Each call to act() returns one 20 Hz eef_pose action. Randomised on purpose (see RANDOMISE) so the
recorded demos are not all the exact same trajectory."""
import numpy as np

# ---- calibration (tune once with a visual run, see README) ----
HOVER_Z = 0.93        # tcp height while travelling (>0.9 needed for 'success', <=0.94 reachable at every slot)
GRASP_DZ = 0.005      # tcp z minus tool-origin z when closing the gripper
RELEASE_DZ = 0.03     # extra clearance above the tray floor when releasing

RANDOMISE = True      # speed / hover height / grasp offset / hold time jitter


class ScriptedPnP:
    def __init__(self, env, tool, rng=np.random):
        self.env, self.tool = env, tool
        p = env.env.get_p_body('body_obj_' + tool)
        slot = int(rng.choice(env.free_dst_slots()))
        d = env._slot_pose('dst', slot)
        if RANDOMISE:
            spd = rng.uniform(0.7, 1.3)                                  # overall speed factor
            hover = HOVER_Z - rng.uniform(0.0, 0.015)
            gx, gy = np.clip(rng.normal(0, 0.003, 2), -0.006, 0.006)     # small grasp offset along/across the handle
            px, py = np.clip(rng.normal(0, 0.006, 2), -0.012, 0.012)     # small place offset
            h = lambda: int(rng.randint(10, 20))
        else:
            spd, hover, gx, gy, px, py, h = 1.0, HOVER_Z, 0, 0, 0, 0, (lambda: 14)
        tx, ty, tz = p[0] + gx, p[1] + gy, p[2]
        grasp_z = tz + GRASP_DZ
        place_z = d[2] + 0.015 + GRASP_DZ + RELEASE_DZ
        dx, dy = d[0] + px, d[1] + py
        W = lambda xyz, g, speed, hold=0: dict(xyz=np.array(xyz, float), g=g, speed=speed * spd, hold=hold)
        self.wps = [
            W((tx, ty, hover), 0, 0.012),               # above tool (slider lines up with the tool)
            W((tx, ty, grasp_z), 0, 0.006),             # descend
            W((tx, ty, grasp_z), 1, 0.006, hold=h()),   # close
            W((tx, ty, hover), 1, 0.007),               # lift
            W((dx, dy, hover), 1, 0.012),               # carry across (slider follows y)
            W((dx, dy, place_z), 1, 0.006),             # lower
            W((dx, dy, place_z), 0, 0.006, hold=h()),   # release
            W((dx, dy, max(hover, 0.925)), 0, 0.008),   # rise (success needs tcp z > 0.9, keep a margin)
        ]
        self.i, self.t_hold = 0, 0

    def act(self):
        """returns eef_pose action (7,), finished"""
        if self.i >= len(self.wps):
            return np.zeros(7, np.float32), True
        w = self.wps[self.i]
        err = w['xyz'] - self.env.p0
        n = np.linalg.norm(err)
        if n > 0.004:
            dpos = err * min(1.0, w['speed'] / n)
        else:
            dpos = np.zeros(3)
            self.t_hold += 1
            if self.t_hold >= max(1, w['hold']):
                self.i += 1
                self.t_hold = 0
        return np.concatenate([dpos, np.zeros(3), [w['g']]]).astype(np.float32), False
