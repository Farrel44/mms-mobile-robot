#!/usr/bin/env python3
"""
Indexed Runtime Command Sequencer — Table-based Motion Sequencer.

Menerima tabel step dari Foxglove via /mission/control (JSON),
mengeksekusi step berdasarkan index (0 → 1 → 2 → …),
dan memonitor exit condition per step.

Foxglove = editor tabel (programming panel), BUKAN teleop.
Sequencer = interpreter + executor.

Data flow:
  Foxglove → /mission/control (JSON tabel + perintah kontrol)
  Sequencer → /cmd_vel (streaming 20-50 Hz saat RUNNING)
  Sequencer ← /odom (feedback untuk exit ANGLE/DIST)
  Sequencer → /mission/status (JSON status untuk Foxglove)
"""

import json
import math
import operator
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# =====================================================================
# Konstanta state sequencer
# =====================================================================
STATE_IDLE = 'IDLE'
STATE_RUNNING = 'RUNNING'
STATE_PAUSED = 'PAUSED'
STATE_DONE = 'DONE'
STATE_ABORTED = 'ABORTED'
STATE_ERROR = 'ERROR'

# Map operator string → fungsi pembanding
COMPARATOR_MAP = {
    '>=': operator.ge,
    '<=': operator.le,
    '>':  operator.gt,
    '<':  operator.lt,
    '==': operator.eq,
}


# =====================================================================
# Command Dictionary
# =====================================================================
# Setiap CMD mendefinisikan:
#   - generate(x, y, w) → (vx, vy, wz) untuk /cmd_vel
#   - deskripsi parameter X/Y/W
#
# CMD = intent, BUKAN velocity mentah.
# X/Y/W = parameter per-CMD (beda arti tiap CMD).

def _gen_maju(x, y, w):
    """MAJU: X = kecepatan linear maju (m/s). Y,W tidak dipakai."""
    return (abs(x), 0.0, 0.0)

def _gen_mundur(x, y, w):
    """MUNDUR: X = kecepatan linear mundur (m/s). Y,W tidak dipakai."""
    return (-abs(x), 0.0, 0.0)

def _gen_kiri(x, y, w):
    """KIRI (strafe): Y = kecepatan lateral kiri (m/s). X,W tidak dipakai."""
    return (0.0, abs(y), 0.0)

def _gen_kanan(x, y, w):
    """KANAN (strafe): Y = kecepatan lateral kanan (m/s). X,W tidak dipakai."""
    return (0.0, -abs(y), 0.0)

def _gen_putar_kiri(x, y, w):
    """PUTAR_KIRI: W = kecepatan angular CCW (rad/s). X,Y tidak dipakai."""
    return (0.0, 0.0, abs(w))

def _gen_putar_kanan(x, y, w):
    """PUTAR_KANAN: W = kecepatan angular CW (rad/s). X,Y tidak dipakai."""
    return (0.0, 0.0, -abs(w))

def _gen_balance(x, y, w):
    """BALANCE: X/Y/W = parameter langsung (pass-through)."""
    return (x, y, w)

def _gen_stop(x, y, w):
    """STOP: diam total, semua parameter diabaikan."""
    return (0.0, 0.0, 0.0)

# Registry: nama CMD → fungsi generator
COMMAND_REGISTRY = {
    'MAJU':         _gen_maju,
    'MUNDUR':       _gen_mundur,
    'KIRI':         _gen_kiri,
    'KANAN':        _gen_kanan,
    'PUTAR_KIRI':   _gen_putar_kiri,
    'PUTAR_KANAN':  _gen_putar_kanan,
    'BALANCE':      _gen_balance,
    'STOP':         _gen_stop,
}


# =====================================================================
# Utilitas yaw (derajat, normalize)
# =====================================================================
def _yaw_from_quaternion(qz, qw):
    """Ekstrak yaw (radian) dari quaternion 2D (hanya z,w)."""
    return 2.0 * math.atan2(qz, qw)

def _rad_to_deg(rad):
    return rad * (180.0 / math.pi)

def _normalize_angle_deg(deg):
    """Wrap sudut ke rentang (-360, 360) — TIDAK di-wrap ke ±180.
    Agar akumulasi delta yaw bisa melewati ±180 untuk exit condition.
    """
    return deg


# =====================================================================
# Node utama: MissionSequencer
# =====================================================================
class MissionSequencer(Node):
    """Indexed Runtime Command Sequencer.

    Menerima tabel step dari Foxglove, mengeksekusi berdasarkan index,
    dan memantau exit condition per step.
    """

    def __init__(self):
        super().__init__('mission_sequencer')

        # -- Parameter ROS --
        self._declare_params()
        self._load_params()

        # -- State sequencer --
        self.state = STATE_IDLE
        self.steps = []          # tabel step dari Foxglove
        self.active_index = 0
        self.warning = ''
        self.error_reason = ''

        # -- Step context (dibuat saat step dimulai) --
        self.step_snapshot = None       # snapshot step aktif (lock)
        self.step_start_time_ns = 0     # timestamp mulai step (ns)
        self.step_start_yaw_deg = 0.0   # yaw saat step mulai (derajat)
        self.step_start_x = 0.0         # posisi x saat step mulai
        self.step_start_y = 0.0         # posisi y saat step mulai
        self.accumulated_yaw_deg = 0.0  # akumulasi delta yaw dari odom

        # -- Output terakhir (untuk status) --
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_wz = 0.0

        # -- Odom state --
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw_deg = 0.0
        self.odom_last_stamp_ns = 0  # timestamp odom terakhir
        self.odom_received = False
        self.prev_odom_yaw_deg = None  # untuk hitung delta yaw

        # -- Stop hold: publish /cmd_vel=0 beberapa tick saat berhenti --
        self.stop_hold_remaining = 0
        self._stop_hold_total = 0
        self._stop_hold_start_vx = 0.0
        self._stop_hold_start_vy = 0.0
        self._stop_hold_start_wz = 0.0

        # -- STOP step ramp-down (halus) --
        self._stop_step_ramp_remaining = 0
        self._stop_step_ramp_total = 0
        self._stop_step_start_vx = 0.0
        self._stop_step_start_vy = 0.0
        self._stop_step_start_wz = 0.0
        # Idle keepalive state (publish /cmd_vel=0 periodically when not RUNNING)
        self._last_idle_keepalive_ns = 0

        # -- ROS interfaces --
        self.control_sub = self.create_subscription(
            String, '/mission/control', self._on_control, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._on_odom, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        # Timer eksekusi (publish /cmd_vel + evaluasi exit)
        timer_period = 1.0 / self.publish_rate_hz
        self.exec_timer = self.create_timer(timer_period, self._tick)

        # Timer status (lebih lambat, cukup 5 Hz)
        self.status_timer = self.create_timer(0.2, self._publish_status)

        self.get_logger().info(
            f'Mission Sequencer siap (rate={self.publish_rate_hz} Hz)')

    # =================================================================
    # Parameter
    # =================================================================
    def _declare_params(self):
        self.declare_parameter('publish_rate_hz', 25.0)
        self.declare_parameter('odom_max_age_ms', 500.0)
        self.declare_parameter('stop_hold_ticks', 10)
        # Publish /cmd_vel=0 periodically when not RUNNING.
        # This avoids the motor bridge watchdog doing a hard-stop after sequencing ends.
        # Default 5 Hz (200 ms) is comfortably below the default watchdog (0.5 s).
        self.declare_parameter('idle_keepalive_hz', 5.0)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 3.14)

    def _load_params(self):
        self.publish_rate_hz = self.get_parameter(
            'publish_rate_hz').value or 25.0
        self.odom_max_age_ms = self.get_parameter(
            'odom_max_age_ms').value or 500.0
        self.stop_hold_ticks = self.get_parameter(
            'stop_hold_ticks').value or 10
        self.max_linear_speed = self.get_parameter(
            'max_linear_speed').value or 1.0
        self.max_angular_speed = self.get_parameter(
            'max_angular_speed').value or 3.14
        self.idle_keepalive_hz = self.get_parameter(
            'idle_keepalive_hz').value or 5.0

    # =================================================================
    # Callback /odom — update state pose dari driver
    # =================================================================
    def _on_odom(self, msg):
        """Simpan posisi & yaw terbaru dari /odom."""
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw_rad = _yaw_from_quaternion(qz, qw)
        new_yaw_deg = _rad_to_deg(yaw_rad)

        # Hitung delta yaw (akumulasi, untuk exit ANGLE)
        if self.prev_odom_yaw_deg is not None:
            delta = new_yaw_deg - self.prev_odom_yaw_deg
            # Wrap delta ke -180..+180 agar tidak loncat saat cross ±180
            if delta > 180.0:
                delta -= 360.0
            elif delta < -180.0:
                delta += 360.0
            self.accumulated_yaw_deg += delta

        self.prev_odom_yaw_deg = new_yaw_deg
        self.odom_yaw_deg = new_yaw_deg
        self.odom_last_stamp_ns = self.get_clock().now().nanoseconds
        self.odom_received = True

    # =================================================================
    # Callback /mission/control — parse JSON command dari Foxglove
    # =================================================================
    def _on_control(self, msg):
        """Parse dan dispatch command JSON dari Foxglove."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.warning = f'JSON tidak valid: {e}'
            self.get_logger().warn(self.warning)
            return

        cmd = data.get('cmd', '').upper()

        if cmd == 'EXECUTE_RAW':
            self._handle_execute_raw(data)
        elif cmd == 'UPDATE_STEPS':
            self._handle_update_steps(data)
        elif cmd == 'RUN':
            self._handle_run()
        elif cmd == 'PAUSE':
            self._handle_pause()
        elif cmd == 'RESUME':
            self._handle_resume()
        elif cmd == 'ABORT':
            self._handle_abort()
        elif cmd == 'SET_INDEX':
            self._handle_set_index(data)
        else:
            self.warning = f'Command tidak dikenal: {cmd}'
            self.get_logger().warn(self.warning)

    # =================================================================
    # Command handlers
    # =================================================================
    def _handle_execute_raw(self, data):
        """EXECUTE_RAW: terima tabel step, simpan di memori, siap RUN."""
        steps = data.get('steps', [])
        if not isinstance(steps, list) or len(steps) == 0:
            self.warning = 'EXECUTE_RAW: steps kosong atau bukan list'
            self.get_logger().warn(self.warning)
            return

        # Validasi setiap step
        for i, step in enumerate(steps):
            if not self._validate_step(step, i):
                return

        # Stop dulu kalau sedang RUNNING
        if self.state == STATE_RUNNING:
            self._force_stop()

        self.steps = steps
        self.active_index = 0
        self.state = STATE_IDLE
        self.warning = 'Menunggu RUN'
        self.error_reason = ''
        self.get_logger().info(
            f'EXECUTE_RAW: {len(steps)} step diterima, siap RUN')

    def _handle_update_steps(self, data):
        """UPDATE_STEPS: update tabel di memori.
        Saat RUNNING: step aktif tetap pakai snapshot (lock).
        """
        steps = data.get('steps', [])
        if not isinstance(steps, list) or len(steps) == 0:
            self.warning = 'UPDATE_STEPS: steps kosong atau bukan list'
            self.get_logger().warn(self.warning)
            return

        for i, step in enumerate(steps):
            if not self._validate_step(step, i):
                return

        self.steps = steps
        self.warning = ''
        self.get_logger().info(
            f'UPDATE_STEPS: tabel diupdate ({len(steps)} step)')

        # Jika index di luar range setelah update, clamp
        if self.active_index >= len(self.steps):
            self.active_index = max(0, len(self.steps) - 1)

    def _handle_run(self):
        """RUN: mulai eksekusi dari active_index."""
        if len(self.steps) == 0:
            self.warning = 'RUN: tidak ada step di memori'
            self.get_logger().warn(self.warning)
            return

        if self.state == STATE_RUNNING:
            # RUN bersifat idempotent. Jangan bikin warning permanen di status.
            return

        if self.active_index >= len(self.steps):
            self.warning = 'RUN: index sudah melewati akhir tabel'
            self.get_logger().warn(self.warning)
            return

        self.state = STATE_RUNNING
        self.error_reason = ''
        self.warning = ''
        self._init_step_context(self.active_index)
        self.get_logger().info(
            f'RUN: mulai dari index {self.active_index}')

    def _handle_pause(self):
        """PAUSE: stop output, tahan index + konteks step."""
        if self.state != STATE_RUNNING:
            self.warning = 'PAUSE: tidak sedang RUNNING'
            return

        self.state = STATE_PAUSED
        self._begin_stop_hold()
        self.get_logger().info(
            f'PAUSED di index {self.active_index}')

    def _handle_resume(self):
        """RESUME: lanjut eksekusi dari step yang sama."""
        if self.state != STATE_PAUSED:
            self.warning = 'RESUME: tidak sedang PAUSED'
            return

        self.state = STATE_RUNNING
        self.warning = ''
        self.get_logger().info(
            f'RESUME dari index {self.active_index}')

    def _handle_abort(self):
        """ABORT: stop paksa dari state apa pun."""
        prev_state = self.state
        self.state = STATE_ABORTED
        self._begin_stop_hold()
        self.step_snapshot = None
        self.get_logger().info(
            f'ABORT dari state {prev_state}')

    def _handle_set_index(self, data):
        """SET_INDEX: set pointer ke index tertentu. TIDAK auto-RUN."""
        index = data.get('index', 0)
        if not isinstance(index, int) or index < 0:
            self.warning = f'SET_INDEX: index tidak valid ({index})'
            self.get_logger().warn(self.warning)
            return

        if index >= len(self.steps):
            self.warning = f'SET_INDEX: index {index} >= total {len(self.steps)}'
            self.get_logger().warn(self.warning)
            return

        # Stop dulu kalau RUNNING
        if self.state == STATE_RUNNING:
            self._force_stop()

        self.active_index = index
        self.step_snapshot = None  # reset context
        # State: kembali ke IDLE, menunggu RUN
        if self.state not in (STATE_ERROR, STATE_ABORTED):
            self.state = STATE_IDLE
        self.warning = 'Menunggu RUN'
        self.get_logger().info(
            f'SET_INDEX: pointer ke {index}, menunggu RUN')

    # =================================================================
    # Validasi step
    # =================================================================
    def _validate_step(self, step, index):
        """Validasi format step. Return True jika valid."""
        if not isinstance(step, dict):
            self.warning = f'Step [{index}]: bukan dict'
            self.state = STATE_ERROR
            self.error_reason = self.warning
            self.get_logger().error(self.warning)
            return False

        cmd = step.get('cmd', '')
        if cmd not in COMMAND_REGISTRY:
            self.warning = f'Step [{index}]: CMD "{cmd}" tidak dikenal'
            self.state = STATE_ERROR
            self.error_reason = self.warning
            self.get_logger().error(self.warning)
            return False

        exit_cond = step.get('exit')
        if not isinstance(exit_cond, dict):
            self.warning = f'Step [{index}]: exit condition tidak valid'
            self.state = STATE_ERROR
            self.error_reason = self.warning
            self.get_logger().error(self.warning)
            return False

        mode = exit_cond.get('mode', '').upper()
        if mode not in ('TIME', 'ANGLE', 'DIST'):
            self.warning = f'Step [{index}]: exit mode "{mode}" tidak dikenal'
            self.state = STATE_ERROR
            self.error_reason = self.warning
            self.get_logger().error(self.warning)
            return False

        op = exit_cond.get('op', '')
        if op not in COMPARATOR_MAP:
            self.warning = f'Step [{index}]: comparator "{op}" tidak valid'
            self.state = STATE_ERROR
            self.error_reason = self.warning
            self.get_logger().error(self.warning)
            return False

        return True

    # =================================================================
    # Step context management
    # =================================================================
    def _init_step_context(self, index):
        """Inisialisasi context untuk step baru (anchor waktu/pose)."""
        step = self.steps[index]
        self.step_snapshot = dict(step)  # snapshot (lock)
        self.step_start_time_ns = self.get_clock().now().nanoseconds

        # Anchor odom untuk exit ANGLE/DIST
        self.step_start_yaw_deg = self.odom_yaw_deg
        self.step_start_x = self.odom_x
        self.step_start_y = self.odom_y

        # Reset akumulasi yaw untuk step baru
        self.accumulated_yaw_deg = 0.0

        # Jika step adalah STOP, siapkan ramp-down halus dari output terakhir.
        cmd = step.get('cmd', '?')
        if cmd == 'STOP':
            self._arm_stop_step_ramp(step)

        exit_info = step.get('exit', {})
        self.get_logger().info(
            f'Step [{index}] mulai: CMD={cmd} '
            f'EXIT={exit_info.get("mode")} {exit_info.get("op")} '
            f'{exit_info.get("val")}')

    def _arm_stop_step_ramp(self, step):
        """Siapkan ramp-down untuk step STOP agar deselerasi halus.

        Ramp menggunakan stop_hold_ticks sebagai jumlah tick ramp.
        Jika STOP step TIME terlalu pendek, ramp akan dipotong agar muat.
        """
        start_vx, start_vy, start_wz = self.last_vx, self.last_vy, self.last_wz
        if abs(start_vx) < 1e-6 and abs(start_vy) < 1e-6 and abs(start_wz) < 1e-6:
            self._stop_step_ramp_remaining = 0
            self._stop_step_ramp_total = 0
            return

        total = int(self.stop_hold_ticks or 0)
        if total <= 0:
            self._stop_step_ramp_remaining = 0
            self._stop_step_ramp_total = 0
            return

        # Jika exit TIME, batasi ramp agar muat di durasi STOP step.
        exit_cond = step.get('exit', {})
        mode = str(exit_cond.get('mode', '')).upper()
        if mode == 'TIME':
            try:
                val_ms = float(exit_cond.get('val', 0.0))
            except (TypeError, ValueError):
                val_ms = 0.0
            if val_ms > 0.0:
                period_ms = 1000.0 / float(self.publish_rate_hz or 25.0)
                available_ticks = max(1, int(math.ceil(val_ms / max(1e-6, period_ms))))
                total = min(total, available_ticks)

        self._stop_step_ramp_total = total
        self._stop_step_ramp_remaining = total
        self._stop_step_start_vx = start_vx
        self._stop_step_start_vy = start_vy
        self._stop_step_start_wz = start_wz

    # =================================================================
    # Tick utama — eksekusi step + evaluasi exit (dipanggil setiap period)
    # =================================================================
    def _tick(self):
        """Loop utama sequencer, dipanggil oleh timer."""
        # Stop hold: ramp-down halus ke /cmd_vel=0 beberapa tick setelah berhenti
        if self.stop_hold_remaining > 0:
            vx, vy, wz = self._compute_ramp_down_cmd(
                self.stop_hold_remaining,
                self._stop_hold_total,
                self._stop_hold_start_vx,
                self._stop_hold_start_vy,
                self._stop_hold_start_wz,
            )
            self._publish_cmd_vel(vx, vy, wz)
            self.stop_hold_remaining -= 1
            return

        # Keepalive: saat tidak RUNNING, tetap publish 0 secara periodik
        # supaya watchdog di motor bridge tidak melakukan hard-stop.
        if self.state != STATE_RUNNING:
            hz = float(self.idle_keepalive_hz or 0.0)
            if hz > 0.0:
                now_ns = self.get_clock().now().nanoseconds
                period_ns = int(1e9 / hz)
                if self._last_idle_keepalive_ns == 0 or (now_ns - self._last_idle_keepalive_ns) >= period_ns:
                    self._publish_cmd_vel(0.0, 0.0, 0.0)
                    self._last_idle_keepalive_ns = now_ns
            return

        if self.step_snapshot is None:
            self._enter_error('Step snapshot kosong saat RUNNING')
            return

        # -- Guard: cek odom stale untuk exit yang butuh odom --
        exit_mode = self.step_snapshot.get('exit', {}).get('mode', '').upper()
        if exit_mode in ('ANGLE', 'DIST'):
            if not self.odom_received:
                self._enter_error('Odom belum pernah diterima (butuh untuk exit)')
                return
            odom_age_ms = self._get_odom_age_ms()
            if odom_age_ms > self.odom_max_age_ms:
                self._enter_error(
                    f'Odom stale ({odom_age_ms:.0f}ms > {self.odom_max_age_ms}ms)')
                return

        # -- Interpret CMD → (vx, vy, wz) --
        vx, vy, wz = self._compute_step_cmd_vel(self.step_snapshot)

        # Clamp safety
        vx = max(-self.max_linear_speed, min(self.max_linear_speed, vx))
        vy = max(-self.max_linear_speed, min(self.max_linear_speed, vy))
        wz = max(-self.max_angular_speed, min(self.max_angular_speed, wz))

        # Publish /cmd_vel
        self._publish_cmd_vel(vx, vy, wz)

        # -- Evaluasi exit condition --
        if self._evaluate_exit(self.step_snapshot):
            self._advance_step()

    # =================================================================
    # Interpreter CMD → (vx, vy, wz)
    # =================================================================
    def _interpret_step(self, step):
        """Translate CMD + X/Y/W → (vx, vy, wz) untuk /cmd_vel."""
        cmd = step.get('cmd', 'STOP')
        x = float(step.get('x', 0.0))
        y = float(step.get('y', 0.0))
        w = float(step.get('w', 0.0))

        gen_fn = COMMAND_REGISTRY.get(cmd, _gen_stop)
        return gen_fn(x, y, w)

    def _compute_step_cmd_vel(self, step):
        """Hitung output cmd_vel untuk step aktif.

        Khusus STOP: lakukan ramp-down halus dari output sebelumnya.
        """
        cmd = step.get('cmd', 'STOP')
        if cmd == 'STOP' and self._stop_step_ramp_remaining > 0 and self._stop_step_ramp_total > 0:
            vx, vy, wz = self._compute_ramp_down_cmd(
                self._stop_step_ramp_remaining,
                self._stop_step_ramp_total,
                self._stop_step_start_vx,
                self._stop_step_start_vy,
                self._stop_step_start_wz,
            )
            self._stop_step_ramp_remaining -= 1
            return vx, vy, wz

        return self._interpret_step(step)

    def _compute_ramp_down_cmd(self, remaining, total, start_vx, start_vy, start_wz):
        """Linear ramp dari (start_v*) menuju 0 selama N tick.

        Menghasilkan alpha 1.0 → 0.0, inclusive, agar tick terakhir tepat 0.
        """
        remaining_i = int(remaining or 0)
        total_i = int(total or 0)
        if total_i <= 1 or remaining_i <= 1:
            return 0.0, 0.0, 0.0

        alpha = float(remaining_i - 1) / float(total_i - 1)
        return start_vx * alpha, start_vy * alpha, start_wz * alpha

    # =================================================================
    # Exit engine — evaluasi condition per step
    # =================================================================
    def _evaluate_exit(self, step):
        """Hitung runtime value & evaluasi comparator. Return True jika exit."""
        exit_cond = step.get('exit', {})
        mode = exit_cond.get('mode', '').upper()
        op_str = exit_cond.get('op', '>=')
        val = float(exit_cond.get('val', 0))

        runtime_value = self._compute_runtime_value(mode)

        comp_fn = COMPARATOR_MAP.get(op_str)
        if comp_fn is None:
            return False

        return comp_fn(runtime_value, val)

    def _compute_runtime_value(self, mode):
        """Hitung nilai runtime sesuai exit mode."""
        if mode == 'TIME':
            # Elapsed sejak step dimulai (ms)
            elapsed_ns = self.get_clock().now().nanoseconds - self.step_start_time_ns
            return elapsed_ns / 1e6  # ns → ms

        elif mode == 'ANGLE':
            # Delta yaw akumulasi sejak step dimulai (derajat)
            return self.accumulated_yaw_deg

        elif mode == 'DIST':
            # Jarak euclidean dari posisi start (meter)
            dx = self.odom_x - self.step_start_x
            dy = self.odom_y - self.step_start_y
            return math.hypot(dx, dy)

        return 0.0

    # =================================================================
    # Advance step → index++
    # =================================================================
    def _advance_step(self):
        """Step selesai, naik ke index berikutnya."""
        old_index = self.active_index
        self.active_index += 1

        self.get_logger().info(
            f'Step [{old_index}] selesai → index {self.active_index}')

        # Semua step sudah dieksekusi?
        if self.active_index >= len(self.steps):
            self.state = STATE_DONE
            self.step_snapshot = None
            self._begin_stop_hold()
            self.get_logger().info('Semua step selesai → DONE')
            return

        # Mulai step berikutnya
        self._init_step_context(self.active_index)

    # =================================================================
    # Helper: publish /cmd_vel
    # =================================================================
    def _publish_cmd_vel(self, vx, vy, wz):
        """Publish Twist ke /cmd_vel dan simpan untuk status."""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.cmd_vel_pub.publish(msg)

        self.last_vx = vx
        self.last_vy = vy
        self.last_wz = wz

    # =================================================================
    # Helper: stop & error
    # =================================================================
    def _force_stop(self):
        """Publish /cmd_vel=0 beberapa tick (untuk transisi state)."""
        self._begin_stop_hold()

    def _begin_stop_hold(self):
        """Mulai fase stop-hold (ramp-down halus lalu 0).

        Menggunakan stop_hold_ticks sebagai jumlah tick.
        """
        total = int(self.stop_hold_ticks or 0)
        if total <= 0:
            self.stop_hold_remaining = 0
            self._stop_hold_total = 0
            return

        self._stop_hold_total = total
        self.stop_hold_remaining = total
        self._stop_hold_start_vx = self.last_vx
        self._stop_hold_start_vy = self.last_vy
        self._stop_hold_start_wz = self.last_wz

    def _enter_error(self, reason):
        """Masuk state ERROR + stop."""
        self.state = STATE_ERROR
        self.error_reason = reason
        self.step_snapshot = None
        self._begin_stop_hold()
        self.get_logger().error(f'ERROR: {reason}')

    def _get_odom_age_ms(self):
        """Hitung umur odom terakhir dalam ms."""
        if self.odom_last_stamp_ns == 0:
            return float('inf')
        now_ns = self.get_clock().now().nanoseconds
        return (now_ns - self.odom_last_stamp_ns) / 1e6

    # =================================================================
    # Publish /mission/status (JSON untuk Foxglove)
    # =================================================================
    def _publish_status(self):
        """Kirim JSON status ke /mission/status untuk monitoring."""
        # Exit runtime value (hanya dihitung saat ada snapshot)
        exit_runtime = 0.0
        exit_mode = ''
        exit_op = ''
        exit_val = 0.0
        if self.step_snapshot is not None:
            exit_cond = self.step_snapshot.get('exit', {})
            exit_mode = exit_cond.get('mode', '')
            exit_op = exit_cond.get('op', '')
            exit_val = exit_cond.get('val', 0)
            exit_runtime = self._compute_runtime_value(exit_mode.upper())

        status = {
            'state': self.state,
            'active_index': self.active_index,
            'total_steps': len(self.steps),
            'active_step': self.step_snapshot,
            'exit_runtime_value': round(exit_runtime, 3),
            'exit_mode': exit_mode,
            'exit_op': exit_op,
            'exit_val': exit_val,
            'cmd_vel_out': {
                'vx': round(self.last_vx, 4),
                'vy': round(self.last_vy, 4),
                'wz': round(self.last_wz, 4),
            },
            'odom_age_ms': round(self._get_odom_age_ms(), 1),
            'warning': self.warning,
            'error_reason': self.error_reason,
        }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


# =====================================================================
# Entry point
# =====================================================================
def main(args=None):
    rclpy.init(args=args)
    node = MissionSequencer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
        # Kirim stop sebelum keluar
        node._publish_cmd_vel(0.0, 0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
