"""Link with the ESP32 (firmware 06_ESP32/arm_bridge): real serial port (pyserial) or simulated robot.

Both classes have the same interface: send(line), close(), and a 'received' queue where the lines sent by the ESP32
arrive ("BRIDGE ready", "pos ...", "done", "ok", "err ..."). The application empties this queue from the Tk loop:
no Tk call is ever made from the reader threads.
"""
import math
import queue
import threading
import time

try:
    import serial
    import serial.tools.list_ports
    PYSERIAL = True
except ImportError:          # pip install pyserial
    PYSERIAL = False

SIMULATION = "Simulation (no robot)"


def list_ports():
    """Available ports: the simulation, then the serial ports (e.g. 'COM5 - Silicon Labs CP210x ...')."""
    ports = [SIMULATION]
    if PYSERIAL:
        for p in sorted(serial.tools.list_ports.comports(), key=lambda p: p.device):
            ports.append("%s - %s" % (p.device, p.description))
    return ports


class SerialLink:
    def __init__(self, port_name):
        if not PYSERIAL:
            raise RuntimeError("the pyserial module is not installed: type  pip install pyserial  in a terminal, "
                               "then restart the application")
        self.received = queue.Queue()
        self.name = port_name.split(" - ")[0].strip()
        self.ser = serial.Serial()
        self.ser.port = self.name
        self.ser.baudrate = 115200
        self.ser.timeout = 0.05
        self.ser.write_timeout = 0.5
        self.ser.dtr = False       # DTR and RTS idle: the ESP32 does not reboot when the port is opened
        self.ser.rts = False
        self.ser.open()
        self.is_open = True
        self._lock = threading.Lock()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        buffer = b""
        while self.is_open:
            try:
                d = self.ser.read(256)
            except Exception as e:                       # cable unplugged...
                if self.is_open:
                    self.received.put("!disconnected " + str(e))
                self.is_open = False
                break
            if d:
                buffer += d
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    line = line.decode("ascii", "replace").strip()
                    if line:
                        self.received.put(line)

    def send(self, line):
        if not self.is_open:
            return False
        try:
            with self._lock:
                self.ser.write((line + "\n").encode("ascii"))
            return True
        except Exception as e:
            self.received.put("!disconnected " + str(e))
            self.is_open = False
            return False

    def close(self):
        self.is_open = False
        try:
            self.ser.close()
        except Exception:
            pass


class SimulatedLink:
    """Simulated robot: reproduces the arm_bridge firmware (same commands, same motions, same answers)."""
    N = 5
    US_MIN, US_MAX = 400.0, 2600.0

    def __init__(self):
        self.name = "simulation"
        self.received = queue.Queue()
        self._input = queue.Queue()
        self.current = [1500.0] * self.N
        self.target = [1500.0] * self.N
        self.start = [1500.0] * self.N
        self.active = [False] * self.N
        self.sync = False
        self.t_sync = 0.0
        self.sync_duration = 0.001
        self.v_max, self.a_max, self.v_follow = 900.0, 4000.0, 0.0
        self.moving = False
        self.changed = False
        self.is_open = True
        self.received.put("BRIDGE ready")
        self._pos()
        threading.Thread(target=self._loop, daemon=True).start()

    def send(self, line):
        if self.is_open:
            self._input.put(line)
        return self.is_open

    def close(self):
        self.is_open = False

    # ------------------------------------------------------------ same behaviour as the firmware
    def _pos(self, prefix="pos"):
        self.received.put(prefix + " " + " ".join(str(int(round(self.current[c]))) if self.active[c] else "0"
                                                  for c in range(self.N)))

    def _clamp(self, u):
        return min(self.US_MAX, max(self.US_MIN, u))

    def _activate(self, c, u):
        if not self.active[c]:
            self.active[c] = True
            self.current[c] = u

    def _handle(self, line):
        line = line.strip()
        if not line:
            return
        cmd = line[0].upper()
        try:
            v = [float(x) for x in line[1:].split()]
        except ValueError:
            v = []
        self.changed |= cmd in "TMPO"
        if cmd == "?":
            self._pos("BRIDGE 1 pca=ok err=0 v=%.0f a=%.0f pos" % (self.v_max, self.a_max))
        elif cmd == "T":
            if len(v) != self.N:
                self.received.put("err T expects 5 values")
                return
            self.sync = False
            for c in range(self.N):
                if v[c] > 0:
                    self.target[c] = self._clamp(v[c])
                    self._activate(c, self.target[c])
        elif cmd == "M":
            if len(v) != self.N + 1 or v[0] < 0:
                self.received.put("err M expects a duration and 5 values")
                return
            for c in range(self.N):
                if v[c + 1] > 0:
                    self.target[c] = self._clamp(v[c + 1])
                    self._activate(c, self.target[c])
                self.start[c] = self.current[c]
            self.v_follow = 0.0
            self.sync_duration = max(1.0, v[0]) / 1000.0
            self.t_sync = time.monotonic()
            self.sync = True
            self.received.put("ok")
        elif cmd == "P":
            if len(v) != 2 or not 0 <= int(v[0]) < self.N:
                self.received.put("err P expects a channel (0-4) and a value")
                return
            c = int(v[0])
            self.sync = False
            self.active[c] = True
            self.current[c] = self.target[c] = self._clamp(v[1])
            self.received.put("ok")
        elif cmd == "V":
            if len(v) >= 1 and v[0] > 0:
                self.v_max = v[0]
            if len(v) >= 2 and v[1] > 0:
                self.a_max = v[1]
            self.received.put("ok v=%.0f a=%.0f" % (self.v_max, self.a_max))
        elif cmd == "O":
            self.sync = False
            for c in range(self.N):
                if not v or int(v[0]) == c:
                    self.active[c] = False
            self.v_follow = 0.0
            self.received.put("ok")
        else:
            self.received.put("err unknown command")

    def _update(self, dt):
        self.moving = False
        if self.sync:
            t = min(1.0, (time.monotonic() - self.t_sync) / self.sync_duration)
            s = t * t * (3 - 2 * t)
            for c in range(self.N):
                if self.active[c]:
                    u = self.start[c] + s * (self.target[c] - self.start[c])
                    self.moving |= u != self.current[c]
                    self.current[c] = u
            if t >= 1:
                self.sync = False
                self.received.put("done")
        else:
            E = max([abs(self.target[c] - self.current[c]) for c in range(self.N) if self.active[c]] or [0.0])
            if E < 0.5:
                for c in range(self.N):
                    if self.active[c]:
                        self.current[c] = self.target[c]
                self.v_follow = 0.0
            else:
                v_wanted = min(self.v_max, math.sqrt(2 * self.a_max * E))
                dv = max(-self.a_max * dt, min(self.a_max * dt, v_wanted - self.v_follow))
                self.v_follow += dv
                f = min(self.v_follow * dt, E) / E
                for c in range(self.N):
                    if self.active[c]:
                        self.current[c] += f * (self.target[c] - self.current[c])
                self.moving = True

    def _loop(self):
        t_update = t_pos = time.monotonic()
        has_moved = False
        while self.is_open:
            while True:
                try:
                    self._handle(self._input.get_nowait())
                except queue.Empty:
                    break
            t = time.monotonic()
            if t - t_update >= 0.02:
                t_update = t
                self._update(0.02)
                has_moved |= self.moving
            if t - t_pos >= 0.1:
                t_pos = t
                if has_moved or self.changed:
                    self._pos()
                has_moved = self.moving
                self.changed = False
            time.sleep(0.004)
