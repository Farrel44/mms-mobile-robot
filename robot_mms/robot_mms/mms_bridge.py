#!/usr/bin/env python3
"""
Node ROS2 untuk bridge /cmd_vel <-> STM32 via protokol serial biner (UART).

Alur utama:
  Command : /cmd_vel -> inverse kinematics -> RPM -> serial TX
  Feedback: serial RX -> forward kinematics -> odometry -> /odom + TF + /imu

Koneksi:
  Raspberry Pi UART (TX/RX) <-> STM32 USART2 (115200 baud, 8N1)
  Port default: /dev/serial0 (pastikan Bluetooth sudah dipindah atau
  dinonaktifkan agar UART0 bebas — lihat /boot/config.txt).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster
import serial
import struct
import time
import math
import threading

from . import kinematics

# Protokol serial (sesuai firmware STM32 — identik dengan ESP32)
PACKET_HEADER = 0xA5
CMD_PACKET_SIZE = 8       # header(1) + 3×int16(6) + xor(1)
FEEDBACK_PACKET_SIZE = 18  # header(1) + 3×int32(12) + 2×int16(4) + xor(1)


class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge_node')

        # =============================================================
        # Deklarasi & baca semua parameter ROS
        # =============================================================
        self._declare_all_parameters()
        self._load_all_parameters()

        # =============================================================
        # State watchdog (stop motor kalau cmd_vel timeout)
        # =============================================================
        self.last_cmd_stamp = self.get_clock().now()
        self.watchdog_triggered = False

        # =============================================================
        # State feedback (diakses dari callback serial & publisher)
        # =============================================================
        self.feedback_lock = threading.Lock()
        self.encoder_ticks = [0, 0, 0]
        self.gyro_z = 0.0
        self.angle_x_rad = 0.0

        # =============================================================
        # State odometry: pose (x, y, yaw) di frame odom
        # =============================================================
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_feedback_stamp = None

        # Twist terakhir (untuk di-publish di /odom)
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_wz = 0.0
        self.last_odom_stamp = None

        # =============================================================
        # Buffer RX serial (stream-safe, partial-packet handling)
        # =============================================================
        self.rx_buf = bytearray()

        # =============================================================
        # Koneksi serial ke STM32 via UART
        # =============================================================
        try:
            self.ser = serial.Serial(
                self.serial_port,
                self.serial_baudrate,
                timeout=self.serial_timeout_s,
                # UART-safe: nonaktifkan flow control & sinyal modem
                # agar tidak mengganggu jalur TX/RX murni ke STM32.
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            # Pastikan DTR/RTS tidak aktif (beberapa driver UART
            # mengubah level pin ini saat port dibuka).
            self.ser.dtr = False
            self.ser.rts = False
            self._init_serial_connection()
            self.get_logger().info(
                f'Serial terhubung: {self.serial_port} @ {self.serial_baudrate}'
            )
        except Exception as e:
            self.get_logger().error(f'Gagal koneksi serial: {e}')
            raise SystemExit(1)

        # =============================================================
        # ROS subscribers, publishers, services, timers
        # =============================================================
        self._setup_ros_interfaces()

        self.get_logger().info('Motor bridge siap')

    # =================================================================
    # Parameter
    # =================================================================

    def _declare_all_parameters(self):
        """Deklarasi parameter dengan default value."""
        # Serial (STM32 via UART — default /dev/serial0)
        self.declare_parameter('serial_port', '/dev/serial0')
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('serial_timeout_s', 0.01)
        self.declare_parameter('serial_boot_wait_s', 0.5)
        self.declare_parameter('serial_sync_max_attempts', 2000)
        self.declare_parameter('serial_reset_buffers_on_start', False)

        # Geometri robot
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('robot_radius', 0.10)
        self.declare_parameter('ticks_per_rev', 380)
        self.declare_parameter('max_rpm', 600)

        # Safety
        self.declare_parameter('watchdog_timeout_s', 0.5)

        # Frame & topic names
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('encoder_ticks_topic', '/encoder_ticks')
        self.declare_parameter('imu_raw_topic', '/imu_raw')
        self.declare_parameter('imu_data_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('reset_odom_service', '/reset_odom')

    def _load_all_parameters(self):
        """Baca nilai parameter ke atribut instance."""
        def _get(name, default, conv):
            val = self.get_parameter(name).value
            return conv(val) if val is not None else default

        # Serial (STM32 via UART)
        self.serial_port = _get('serial_port', '/dev/serial0', str)
        self.serial_baudrate = _get('serial_baudrate', 115200, int)
        self.serial_timeout_s = _get('serial_timeout_s', 0.01, float)
        self.serial_boot_wait_s = _get('serial_boot_wait_s', 0.5, float)
        self.serial_sync_max_attempts = _get('serial_sync_max_attempts', 2000, int)
        self.serial_reset_buffers_on_start = _get('serial_reset_buffers_on_start', False, bool)

        # Geometri
        self.wheel_radius = _get('wheel_radius', 0.05, float)
        self.robot_radius = _get('robot_radius', 0.10, float)
        self.ticks_per_rev = _get('ticks_per_rev', 380, int)
        self.max_rpm = _get('max_rpm', 600, int)

        # Safety
        self.watchdog_timeout = _get('watchdog_timeout_s', 0.5, float)

        # Frame
        self.odom_frame_id = _get('odom_frame_id', 'odom', str)
        self.base_frame_id = _get('base_frame_id', 'base_link', str)

    def _setup_ros_interfaces(self):
        """Buat subscriber, publisher, service, dan timer."""
        def _get_str(name):
            val = self.get_parameter(name).value
            return str(val) if val is not None else ''

        cmd_vel_topic = _get_str('cmd_vel_topic')
        encoder_topic = _get_str('encoder_ticks_topic')
        imu_raw_topic = _get_str('imu_raw_topic')
        imu_data_topic = _get_str('imu_data_topic')
        odom_topic = _get_str('odom_topic')
        reset_srv_name = _get_str('reset_odom_service')

        # Subscriber
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)

        # Publisher debug (sensor QoS: data terbaru, tidak numpuk)
        self.encoder_pub = self.create_publisher(
            Float32MultiArray, encoder_topic, qos_profile_sensor_data)
        self.imu_raw_pub = self.create_publisher(
            Float32MultiArray, imu_raw_topic, qos_profile_sensor_data)

        # Publisher standar
        self.imu_data_pub = self.create_publisher(
            Imu, imu_data_topic, qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Service reset odom
        self.create_service(Empty, reset_srv_name, self.reset_odom_callback)

        # Timer: baca serial (~50 Hz) dan watchdog (~10 Hz)
        self.create_timer(0.02, self.read_feedback_callback)
        self.create_timer(0.1, self.watchdog_callback)

        # Timer: publish pose/TF periodik agar frame odom selalu ada
        # (Foxglove/RViz butuh /tf untuk menampilkan daftar fixed frame)
        self.create_timer(0.1, self.publish_pose_callback)
        
    
    def publish_pose_callback(self):
        """Publish /odom dan TF walau tidak ada feedback baru."""
        stamp_now = self.get_clock().now()
        self._publish_pose_only(stamp_now)

    def _publish_pose_only(self, stamp_now):
        """Publish odom + TF saja (tanpa debug)."""
        with self.feedback_lock:
            x = self.x
            y = self.y
            yaw = self.yaw
            vx = self.last_vx
            vy = self.last_vy
            wz = self.last_wz

        stamp_msg = stamp_now.to_msg()

        # Quaternion yaw (2D)
        half_yaw = yaw * 0.5
        sin_half = math.sin(half_yaw)
        cos_half = math.cos(half_yaw)

        odom = Odometry()
        odom.header.stamp = stamp_msg
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = sin_half
        odom.pose.pose.orientation.w = cos_half
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp_msg
        tf.header.frame_id = self.odom_frame_id
        tf.child_frame_id = self.base_frame_id
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = sin_half
        tf.transform.rotation.w = cos_half
        self.tf_broadcaster.sendTransform(tf)

    # =================================================================
    # Serial I/O (Transport Layer)
    # =================================================================

    def _init_serial_connection(self):
        """Startup serial: tunggu boot STM32 -> sync paket feedback.

        Berbeda dengan ESP32 (perlu kirim 'b' untuk masuk binary mode),
        STM32 langsung beroperasi dalam mode biner sejak FreeRTOS boot.
        Cukup tunggu sebentar lalu sinkronisasi ke stream feedback.
        """
        self.get_logger().info('Inisialisasi serial (STM32 UART)...')

        if self.serial_reset_buffers_on_start:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

        if self.serial_boot_wait_s > 0.0:
            self.get_logger().info(
                f'Tunggu boot STM32 ({self.serial_boot_wait_s:.1f}s)...')
            time.sleep(self.serial_boot_wait_s)

        # STM32 sudah dalam binary mode — langsung flush buffer
        # untuk membuang data sisa saat boot.
        self.ser.reset_input_buffer()

        # Sync: cari paket feedback valid pertama
        self.get_logger().info('Sync ke stream feedback STM32...')
        if self._sync_to_packet(self.serial_sync_max_attempts):
            self.get_logger().info('Sync OK (paket valid ditemukan)')
        else:
            self.get_logger().warn('Sync gagal (belum ada paket valid dari STM32)')

    def _sync_to_packet(self, max_attempts=1000):
        """Baca bytes sampai ketemu 1 feedback packet valid."""
        for _ in range(max_attempts):
            waiting = self.ser.in_waiting or 0
            if waiting > 0:
                chunk = self.ser.read(min(int(waiting), 256))
            else:
                chunk = self.ser.read(1)

            if chunk:
                self._feed_rx(chunk)
                packets = self._extract_feedback_packets(max_packets=1)
                if packets:
                    return True
            else:
                time.sleep(0.001)
        return False

    def _feed_rx(self, data):
        """Tambah data ke buffer RX, batasi ukuran agar tidak membengkak."""
        if not data:
            return
        self.rx_buf.extend(data)
        # Buang data lama kalau buffer terlalu besar (noise tanpa header)
        if len(self.rx_buf) > 4096:
            self.rx_buf = self.rx_buf[-1024:]

    def _extract_feedback_packets(self, max_packets=None):
        """Ambil semua feedback packet valid dari buffer RX.

        - Paket belum lengkap disimpan (tidak dibuang).
        - Checksum gagal: buang 1 byte, lanjut cari header (resync).
        """
        packets = []
        header_byte = bytes([PACKET_HEADER])

        while True:
            if max_packets is not None and len(packets) >= max_packets:
                break

            # Cari posisi header 0xA5
            idx = self.rx_buf.find(header_byte)
            if idx < 0:
                self.rx_buf.clear()
                break

            # Buang noise sebelum header
            if idx > 0:
                del self.rx_buf[:idx]

            # Tunggu sampai data cukup 18 byte
            if len(self.rx_buf) < FEEDBACK_PACKET_SIZE:
                break

            candidate = bytes(self.rx_buf[:FEEDBACK_PACKET_SIZE])
            if self.verify_checksum(candidate):
                packets.append(candidate)
                del self.rx_buf[:FEEDBACK_PACKET_SIZE]
            else:
                # Checksum gagal: buang 1 byte dan resync
                del self.rx_buf[:1]

        return packets

    # =================================================================
    # Checksum & Packet Building (Protokol)
    # =================================================================

    def calculate_checksum(self, data):
        """XOR checksum semua byte."""
        checksum = 0
        for b in data:
            checksum ^= b
        return checksum & 0xFF

    def verify_checksum(self, packet):
        """Cek XOR checksum byte terakhir."""
        if len(packet) < 2:
            return False
        expected = self.calculate_checksum(packet[:-1])
        return expected == packet[-1]

    def build_command_packet(self, rpm1, rpm2, rpm3):
        """Bangun paket command 8 byte: [0xA5, RPM1(2), RPM2(2), RPM3(2), XOR].

        RPM di-clamp ke ±max_rpm, format big-endian int16.
        """
        rpm1 = int(max(min(rpm1, self.max_rpm), -self.max_rpm))
        rpm2 = int(max(min(rpm2, self.max_rpm), -self.max_rpm))
        rpm3 = int(max(min(rpm3, self.max_rpm), -self.max_rpm))

        rpm_bytes = struct.pack('>hhh', rpm1, rpm2, rpm3)
        packet = bytes([PACKET_HEADER]) + rpm_bytes
        checksum = self.calculate_checksum(packet)
        return packet + bytes([checksum])

    def parse_feedback_packet(self, packet):
        """Parse feedback 18 byte dari STM32.

        Format: [0xA5, Tick1(4), Tick2(4), Tick3(4), GyroZ(2), AngleX(2), XOR]
        STM32 mengirim gyro_z & angle_x dikali 1000 (int16, milli-rad).
        Protokol identik dengan versi ESP32 sebelumnya.

        Returns:
            (delta_ticks, gyro_z_rad_s, angle_x_rad) atau None jika gagal
        """
        if len(packet) != FEEDBACK_PACKET_SIZE:
            return None
        if packet[0] != PACKET_HEADER:
            return None
        if not self.verify_checksum(packet):
            self.get_logger().warn('Checksum feedback tidak cocok!')
            return None

        try:
            ticks = struct.unpack('>iii', packet[1:13])
            imu_raw = struct.unpack('>hh', packet[13:17])

            gyro_z = imu_raw[0] / 1000.0      # rad/s
            angle_x_rad = imu_raw[1] / 1000.0  # rad

            return list(ticks), gyro_z, angle_x_rad
        except struct.error as e:
            self.get_logger().error(f'Error unpack feedback: {e}')
            return None

    # =================================================================
    # Command Path: /cmd_vel -> inverse kinematics -> serial TX
    # =================================================================

    def cmd_vel_callback(self, msg):
        """Terima cmd_vel, hitung RPM, kirim ke STM32."""
        self.last_cmd_stamp = self.get_clock().now()
        self.watchdog_triggered = False

        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        self.get_logger().info(
            f'cmd_vel: vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}')

        # Inverse kinematics: cmd_vel -> kecepatan roda -> RPM
        v1, v2, v3 = kinematics.cmd_vel_to_wheel_velocities(
            vx, vy, wz, self.robot_radius)
        rpm1 = kinematics.velocity_to_rpm(v1, self.wheel_radius)
        rpm2 = kinematics.velocity_to_rpm(v2, self.wheel_radius)
        rpm3 = kinematics.velocity_to_rpm(v3, self.wheel_radius)

        self.get_logger().info(
            f'Wheel vel: v1={v1:.3f} v2={v2:.3f} v3={v3:.3f} m/s')

        self.send_rpm_command(rpm1, rpm2, rpm3)

    def send_rpm_command(self, rpm1, rpm2, rpm3):
        """Kirim paket RPM ke STM32."""
        packet = self.build_command_packet(rpm1, rpm2, rpm3)
        try:
            self.ser.write(packet)
            self.get_logger().info(
                f'TX RPM: M1={int(rpm1)} M2={int(rpm2)} M3={int(rpm3)}')
        except Exception as e:
            self.get_logger().error(f'Error tulis serial: {e}')

    def send_stop_command(self):
        """Kirim RPM=0 ke semua motor."""
        self.send_rpm_command(0, 0, 0)
        self.get_logger().warn('STOP dikirim!')

    # =================================================================
    # Feedback Path: serial RX -> forward kinematics -> odometry
    # =================================================================

    def read_feedback_callback(self):
        """Baca feedback dari STM32, proses batch, publish."""
        waiting = self.ser.in_waiting or 0
        if waiting <= 0:
            return

        chunk = self.ser.read(int(waiting))
        self._feed_rx(chunk)

        packets = self._extract_feedback_packets()
        if not packets:
            return

        # Parse semua paket valid
        samples = []
        for pkt in packets:
            result = self.parse_feedback_packet(pkt)
            if result is not None:
                samples.append(result)

        if not samples:
            return

        stamp_now = self.get_clock().now()
        self._integrate_odometry(samples, stamp_now)
        self._publish_all(stamp_now)

    def _integrate_odometry(self, samples, stamp_now):
        """Integrasi odometry dari batch feedback samples.

        Kalau ada N paket sekaligus, dt_total dibagi rata ke N sample
        supaya tidak ada sample dengan dt=0.

        Integrasi pose:
          1. Forward kinematics: delta ticks -> vx, vy (body frame)
          2. Heading: yaw += gyro_z * dt
          3. Rotasi body -> odom pakai yaw_mid (midpoint, lebih stabil)
          4. Update posisi x, y
        """
        with self.feedback_lock:
            # Simpan data sample terakhir untuk debug publisher
            ticks_last, gyro_z_last, angle_x_last = samples[-1]
            self.encoder_ticks = ticks_last
            self.gyro_z = gyro_z_last
            self.angle_x_rad = angle_x_last

            # Belum ada stamp sebelumnya -> simpan dan skip integrasi
            if self.last_feedback_stamp is None:
                self.last_feedback_stamp = stamp_now
                return

            dt_total = (stamp_now - self.last_feedback_stamp).nanoseconds / 1e9
            self.last_feedback_stamp = stamp_now

            # Guard: skip kalau dt aneh (startup / backlog)
            if dt_total <= 0.0 or dt_total > 0.2:
                return

            dt_per_sample = dt_total / float(len(samples))
            if dt_per_sample <= 0.0:
                return

            # Integrasi tiap sample
            vx_body = 0.0
            vy_body = 0.0
            wz_gyro = 0.0

            for ticks, gyro_z, _angle_x in samples:
                # Forward kinematics: delta ticks -> twist body
                vx, vy, _wz_enc = kinematics.delta_ticks_to_body_twist(
                    ticks,
                    self.ticks_per_rev,
                    self.wheel_radius,
                    self.robot_radius,
                    dt_per_sample,
                )

                # Heading dari gyro (lebih akurat dari encoder untuk omni)
                yaw_sebelum = self.yaw
                delta_yaw = gyro_z * dt_per_sample
                yaw_mid = yaw_sebelum + 0.5 * delta_yaw  # midpoint
                self.yaw = yaw_sebelum + delta_yaw

                # Displacement body frame
                dx_body = vx * dt_per_sample
                dy_body = vy * dt_per_sample

                # Rotasi body -> odom (pakai yaw midpoint)
                cos_mid = math.cos(yaw_mid)
                sin_mid = math.sin(yaw_mid)
                self.x += dx_body * cos_mid - dy_body * sin_mid
                self.y += dx_body * sin_mid + dy_body * cos_mid

                # Simpan twist terakhir untuk publish
                vx_body = vx
                vy_body = vy
                wz_gyro = gyro_z

            self.last_vx = vx_body
            self.last_vy = vy_body
            self.last_wz = wz_gyro
            self.last_odom_stamp = stamp_now

    # =================================================================
    # ROS Interface: Publish odom, TF, IMU, debug topics
    # =================================================================

    def _publish_all(self, stamp_now):
        """Publish semua topic: debug, IMU standar, odom, dan TF."""
        # Ambil snapshot state (thread-safe)
        with self.feedback_lock:
            ticks = list(self.encoder_ticks)
            gyro_z = self.gyro_z
            angle_x_rad = self.angle_x_rad
            x = self.x
            y = self.y
            yaw = self.yaw
            vx = self.last_vx
            vy = self.last_vy
            wz = self.last_wz

        stamp_msg = stamp_now.to_msg()

        # --- Debug: delta ticks ---
        enc_msg = Float32MultiArray()
        enc_msg.data = [float(t) for t in ticks]
        self.encoder_pub.publish(enc_msg)

        # --- Debug: IMU raw ---
        imu_raw_msg = Float32MultiArray()
        imu_raw_msg.data = [gyro_z, angle_x_rad]
        self.imu_raw_pub.publish(imu_raw_msg)

        # --- IMU standar ---
        imu_msg = Imu()
        imu_msg.header.stamp = stamp_msg
        imu_msg.header.frame_id = self.base_frame_id
        imu_msg.angular_velocity.z = gyro_z
        # Orientation & linear accel belum tersedia
        imu_msg.orientation_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0
        self.imu_data_pub.publish(imu_msg)

        # --- Odometry ---
        half_yaw = yaw * 0.5
        sin_half = math.sin(half_yaw)
        cos_half = math.cos(half_yaw)

        odom = Odometry()
        odom.header.stamp = stamp_msg
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = sin_half
        odom.pose.pose.orientation.w = cos_half

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        # --- TF: odom -> base_link ---
        tf = TransformStamped()
        tf.header.stamp = stamp_msg
        tf.header.frame_id = self.odom_frame_id
        tf.child_frame_id = self.base_frame_id
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = sin_half
        tf.transform.rotation.w = cos_half
        self.tf_broadcaster.sendTransform(tf)

    # =================================================================
    # Service: Reset Odometry
    # =================================================================

    def reset_odom_callback(self, request, response):
        """Reset posisi odom ke (0, 0, 0) tanpa ganggu serial."""
        with self.feedback_lock:
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self.last_feedback_stamp = None
            self.last_vx = 0.0
            self.last_vy = 0.0
            self.last_wz = 0.0
        self.get_logger().info('Odom di-reset: x=0 y=0 yaw=0')
        return response

    # =================================================================
    # Watchdog: stop motor kalau cmd_vel timeout
    # =================================================================

    def watchdog_callback(self):
        """Hentikan motor kalau tidak ada cmd_vel dalam batas waktu."""
        elapsed = (self.get_clock().now() - self.last_cmd_stamp).nanoseconds / 1e9
        if elapsed > self.watchdog_timeout and not self.watchdog_triggered:
            self.get_logger().warn(
                f'WATCHDOG: cmd_vel timeout ({elapsed:.2f}s) -> motor stop')
            self.send_stop_command()
            self.watchdog_triggered = True


# =================================================================
# Entry point
# =================================================================

def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
        node.send_stop_command()
    finally:
        # Best-effort cleanup (Ctrl-C handling differs across ROS 2 distros)
        try:
            if getattr(node, 'ser', None) is not None:
                node.ser.close()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
