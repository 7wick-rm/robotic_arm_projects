import sys, time, threading, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from panel_core import RailError, SerialRail, Controller, MockRail, MockRunner, RolloutRunner, load_config, parse_keywords, GestureInput

CFG = load_config(Path(__file__).resolve().parents[1] / "workspaces.yaml")
CFG["mock"] = True
for m in CFG["models"].values():
    m["duration_s"] = 1.0


def wait(c, state, t=10):
    t0 = time.time()
    while c.state != state and time.time() - t0 < t:
        time.sleep(0.05)
    return c.state == state


class T(unittest.TestCase):
    def mk(self):
        logs = []
        return Controller(CFG, logf=logs.append, rail=MockRail(CFG["rail"]["positions"], 0.1), runner=MockRunner(logs.append)), logs

    def test_keywords_scoped(self):
        ws2, ws3 = CFG["workspaces"][1]["options"], CFG["workspaces"][2]["options"]
        self.assertEqual(parse_keywords("please pick up the hammer", ws2), "hammer")
        self.assertEqual(parse_keywords("hammer or wrench", ws2), None)          # two options -> reject
        self.assertEqual(parse_keywords("Thor's hammer", ws3), "thor_hammer")
        self.assertEqual(parse_keywords("make a burger", ws3), "burger")
        self.assertEqual(parse_keywords("hammer", ws3), None)                    # WS3 does not listen for plain hammer
        self.assertEqual(parse_keywords("hammered", ws2), None)                  # whole words only

    def test_full_flow_and_busy_ignore(self):
        c, logs = self.mk()
        c.choose("hammer")                       # before a workspace is selected -> ignored
        self.assertEqual(c.state, "IDLE")
        c.select_workspace(2)
        c.select_workspace(3)                    # busy -> ignored
        self.assertTrue(wait(c, "READY")); self.assertEqual(c.ws, 2)
        c.choose("burger")                       # not an option of WS2 -> ignored
        self.assertEqual(c.state, "READY")
        c.choose("wrench"); c.choose("hammer")   # second while busy -> ignored
        self.assertTrue(wait(c, "RUNNING"))
        self.assertTrue(wait(c, "READY", 15))
        self.assertEqual(sum("command via" in l for l in logs), 1)
        c.back(); self.assertEqual(c.state, "IDLE")

    def test_stop_ends_run(self):
        CFG["models"]["screw"]["duration_s"] = 30
        c, _ = self.mk()
        c.select_workspace(1); self.assertTrue(wait(c, "READY"))
        c.choose("screw"); self.assertTrue(wait(c, "RUNNING"))
        t0 = time.time(); c.stop()
        self.assertTrue(wait(c, "READY", 10)); self.assertLess(time.time() - t0, 5)
        CFG["models"]["screw"]["duration_s"] = 1.0

    def test_rail_fault_and_interlock(self):
        class Bad(MockRail):
            def move_to(self, n): raise RuntimeError("stall")
        c = Controller(CFG, logf=lambda m: None, rail=Bad(CFG["rail"]["positions"]), runner=MockRunner(lambda m: None))
        c.select_workspace(1); self.assertTrue(wait(c, "FAULT"))
        c.select_workspace(2); self.assertEqual(c.state, "FAULT")                # ignored until acknowledged
        c.acknowledge_fault(); self.assertEqual(c.state, "IDLE")
        cfg2 = dict(CFG, mock=False, arm_starts_at_rest=False)
        cfg2["robot"] = dict(CFG["robot"], rest_pose={})                        # no stow possible, arm not at rest
        c2 = Controller(cfg2, logf=lambda m: None, rail=MockRail(CFG["rail"]["positions"]), runner=MockRunner(lambda m: None))
        c2.arm_at_rest = False
        c2.select_workspace(1); self.assertTrue(wait(c2, "FAULT"))              # interlock refuses the rail move

    def test_rollout_command(self):
        cfg = dict(CFG); cfg["robot"] = dict(CFG["robot"], id="my_arm", port="/dev/ttyACM0")
        for backend in ("sync", "remote"):
            cfg["inference"] = dict(CFG["inference"], backend=backend, ssh_host=None)
            cmd = RolloutRunner(cfg, print).build_cmd("hammer", dict(repo="u/hammer", task="pick the hammer", fps=30), 60)
            s = " ".join(cmd)
            self.assertIn("--interactive=true", s); self.assertIn("--robot.id=my_arm", s)
            self.assertIn("--robot.cameras={ front: {type: opencv", s)
            self.assertIn("--policy.path=u/hammer" if backend == "sync" else "--inference.type=remote", s)
            if backend == "remote":
                self.assertNotIn("--policy.path", s); self.assertIn("--inference.deployment=hammer", s)

    def test_finger_count(self):
        class P:  # fake landmark
            def __init__(s, x, y): s.x, s.y = x, y
        def hand(extended):
            lm = [P(0.5, 0.9)] * 21
            lm = list(lm)
            for i, (tip, pip) in enumerate(((8, 6), (12, 10), (16, 14), (20, 18))):
                lm[pip] = P(0.4 + 0.05 * i, 0.6)
                lm[tip] = P(0.4 + 0.05 * i, 0.3 if i < extended else 0.65)
            return lm
        for n in range(5):
            self.assertEqual(GestureInput.count_fingers(hand(n)), n)


class FakeEsp32:
    """Emulates the firmware protocol: boots UNHOMED (does not move), HOME -> HOMING -> READY, MOVE -> ACK -> DONE."""
    def __init__(self, fault_on_move=False, never_ready=False):
        import collections
        self.out, self.t_home, self.pos = collections.deque(), None, 0.0
        self.fault_on_move, self.never_ready, self.sent = fault_on_move, never_ready, []
        self.out.extend([b"ESP32 STARTED\n", b"UNHOMED: waiting for HOME command (rail will not move)\n"])

    def _state(self):
        if self.t_home is None: return "UNHOMED"
        if self.never_ready or time.time() - self.t_home < 0.5: return "HOMING"
        return "READY"

    def write(self, data):
        for line in data.decode().strip().split("\n"):
            c = line.split()
            if not c: continue
            self.sent.append(c[0])
            if c[0] == "STATUS?":
                self.out.append(f"STATE {self._state()} POS {self.pos:.2f} TARGET 0 TRAVEL 250 MODE SERIAL\n".encode())
            elif c[0] == "HOME":
                self.t_home = time.time()
            elif c[0] in ("MOVE", "MOVEPCT"):
                if self._state() != "READY":
                    self.out.append(b"ERR NOT_HOMED\n"); continue
                tgt = float(c[1]) / 100 * 250 if c[0] == "MOVEPCT" else float(c[1])
                self.out.append(f"ACK {tgt:.2f}\n".encode())
                self.out.append(b"FAULT: Stall\n" if self.fault_on_move else f"DONE {tgt:.2f}\n".encode())
                self.pos = tgt
            elif c[0] == "STOP":
                if self._state() == "HOMING": self.t_home = None
                self.out.append(b"STOPPED 0\n")

    def readline(self):
        if self.out: return self.out.popleft()
        time.sleep(0.02); return b""
    def reset_input_buffer(self): self.out.clear()
    def close(self): pass


class RailTests(unittest.TestCase):
    cfg = {"driver": "esp32", "unit": "pct", "positions": {"a": 100, "b": 50}, "move_timeout_s": 3, "boot_timeout_s": 5}

    def test_connect_does_not_home_and_first_move_homes(self):
        f = FakeEsp32(); r = SerialRail(self.cfg, lambda m: None, serial_factory=lambda: f)
        r.connect()
        self.assertNotIn("HOME", f.sent)                                    # connecting must never move the rail
        t0 = time.time(); r.move_to("b")
        self.assertIn("HOME", f.sent); self.assertGreater(time.time() - t0, 0.4)   # homed first, then moved
        self.assertAlmostEqual(r.position(), 125.0)
        n = f.sent.count("HOME"); r.move_to("a"); self.assertEqual(f.sent.count("HOME"), n)   # already homed: no re-home

    def test_fault_and_timeout(self):
        r = SerialRail(self.cfg, lambda m: None, serial_factory=lambda: FakeEsp32(fault_on_move=True)); r.connect()
        with self.assertRaises(RailError): r.move_to("a")
        cfg = dict(self.cfg, home_timeout_s=1)
        r2 = SerialRail(cfg, lambda m: None, serial_factory=lambda: FakeEsp32(never_ready=True)); r2.connect()
        with self.assertRaises(RailError): r2.move_to("a")                  # homing never finishes -> error + STOP sent
        self.assertIn("STOP", r2.ser.sent)

    def test_stop_during_homing_raises(self):
        f = FakeEsp32(never_ready=True); r = SerialRail(dict(self.cfg, home_timeout_s=5), lambda m: None, serial_factory=lambda: f)
        r.connect()
        import threading
        threading.Timer(0.6, r.stop).start()
        with self.assertRaises(RailError): r.move_to("a")


if __name__ == "__main__":
    unittest.main(verbosity=2)
