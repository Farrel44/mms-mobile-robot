#!/usr/bin/env python3
"""
ROS2 node bridging /cmd_vel to ESP32 motor controller via binary serial protocol.
Implements 500ms watchdog for safety and publishes encoder + IMU feedback.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
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

PACKET_HEADER = 0xA5
CMD_PACKET_SIZE = 8
FEEDBACK_PACKET_SIZE = 18

class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge_node')

        # Parameters (defaults match previous hardcoded values)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('serial_timeout_s', 0.01)
        self.declare_parameter('serial_boot_wait_s', 2.5)
        self.declare_parameter('serial_mode_switch_wait_s', 0.5)
        self.declare_parameter('serial_sync_max_attempts', 2000)
        self.declare_parameter('serial_reset_buffers_on_start', False)

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('robot_radius', 0.10)
        self.declare_parameter('ticks_per_rev', 380)
        self.declare_parameter('max_rpm', 600)

        self.declare_parameter('watchdog_timeout_s', 0.5)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('encoder_ticks_topic', '/encoder_ticks')
        self.declare_parameter('imu_raw_topic', '/imu_raw')
        self.declare_parameter('imu_data_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('reset_odom_service', '/reset_odom')

        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baudrate = int(self.get_parameter('serial_baudrate').value)
        self.serial_timeout_s = float(self.get_parameter('serial_timeout_s').value)
        self.serial_boot_wait_s = float(self.get_parameter('serial_boot_wait_s').value)
        self.serial_mode_switch_wait_s = float(self.get_parameter('serial_mode_switch_wait_s').value)
        self.serial_sync_max_attempts = int(self.get_parameter('serial_sync_max_attempts').value)
        self.serial_reset_buffers_on_start = bool(self.get_parameter('serial_reset_buffers_on_start').value)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.ticks_per_rev = int(self.get_parameter('ticks_per_rev').value)
        self.max_rpm = int(self.get_parameter('max_rpm').value)

        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout_s').value)
        self.last_cmd_stamp = self.get_clock().now()
        self.watchdog_triggered = False
            
        # Thread-safe feedback state shared between serial reader and publishers
        self.encoder_ticks = [0, 0, 0]
        self.gyro_z = 0.0
        self.angle_x_rad = 0.0

        # Odometry state (odom -> base_link)
        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._last_feedback_stamp = None

        self.feedback_lock = threading.Lock()

        # Serial RX buffer for stream-safe parsing (handles partial reads/resync)
        self._rx_buf = bytearray()
        
        try:
            self.ser = serial.Serial(
                self.serial_port,
                self.serial_baudrate,
                timeout=self.serial_timeout_s,
            )
            self._init_serial_connection()
            self.get_logger().info(
                f"Serial connected to ESP32 on {self.serial_port} @ {self.serial_baudrate}"
            )
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            exit()

        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.subscription = self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        
        encoder_ticks_topic = str(self.get_parameter('encoder_ticks_topic').value)
        imu_raw_topic = str(self.get_parameter('imu_raw_topic').value)
        imu_data_topic = str(self.get_parameter('imu_data_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)

        # QoS sensor_data: utamakan data terbaru, tidak menumpuk queue.
        self.encoder_pub = self.create_publisher(Float32MultiArray, encoder_ticks_topic, qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Float32MultiArray, imu_raw_topic, qos_profile_sensor_data)
        self.imu_data_pub = self.create_publisher(Imu, imu_data_topic, qos_profile_sensor_data)

        # Odometry biasanya dipakai node lain untuk integrasi/TF, tetap reliable (default).
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Service sederhana untuk reset odom saat testing/kalibrasi.
        reset_srv_name = str(self.get_parameter('reset_odom_service').value)
        self.reset_odom_srv = self.create_service(Empty, reset_srv_name, self.reset_odom_callback)
        
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)
        self.serial_read_timer = self.create_timer(0.02, self.read_feedback_callback)
        
        self.get_logger().info("Motor bridge ready")

    def _init_serial_connection(self):
        """Sync to ESP32 packet stream, preventing boot message corruption in buffer"""
        self.get_logger().info("Initializing serial connection...")

        if self.serial_reset_buffers_on_start:
            # Optional: can be helpful if the port is noisy, but may drop valid bytes.
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

        if self.serial_boot_wait_s > 0.0:
            time.sleep(self.serial_boot_wait_s)
        
        # Send binary mode command
        self.get_logger().info("Switching ESP32 to binary mode...")
        self.ser.write(b'b')

        if self.serial_mode_switch_wait_s > 0.0:
            time.sleep(self.serial_mode_switch_wait_s)
        
        self.get_logger().info("Binary mode active, syncing to feedback stream...")
        if self._sync_to_packet(max_attempts=self.serial_sync_max_attempts):
            self.get_logger().info("Serial sync complete (valid packet found)")
        else:
            self.get_logger().warn("Serial sync failed (no valid packet found yet)")
    
    def _sync_to_packet(self, max_attempts=1000):
        """Find a valid feedback packet by streaming bytes into the RX buffer."""
        for _ in range(max_attempts):
            n = int(getattr(self.ser, 'in_waiting', 0) or 0)
            if n <= 0:
                chunk = self.ser.read(1)
            else:
                chunk = self.ser.read(min(n, 256))

            if chunk:
                self._feed_rx(chunk)
                packets = self._extract_feedback_packets(max_packets=1)
                if packets:
                    return True
            else:
                time.sleep(0.001)
        return False

    def _feed_rx(self, data):
        if not data:
            return
        self._rx_buf.extend(data)

        # Prevent unbounded growth if noise/no headers.
        if len(self._rx_buf) > 4096:
            self._rx_buf = self._rx_buf[-1024:]

    def _extract_feedback_packets(self, max_packets=None):
        """Extract valid 18-byte feedback packets from the RX buffer.

        - Keeps partial packets in the buffer.
        - On checksum failure, drops one byte and continues (resync).
        """
        packets = []
        header_byte = bytes([PACKET_HEADER])

        while True:
            if max_packets is not None and len(packets) >= max_packets:
                break

            idx = self._rx_buf.find(header_byte)
            if idx < 0:
                # No header present: drop all buffered noise
                self._rx_buf.clear()
                break

            if idx > 0:
                # Drop noise before header
                del self._rx_buf[:idx]

            if len(self._rx_buf) < FEEDBACK_PACKET_SIZE:
                break

            candidate = bytes(self._rx_buf[:FEEDBACK_PACKET_SIZE])
            if self.verify_checksum(candidate):
                packets.append(candidate)
                del self._rx_buf[:FEEDBACK_PACKET_SIZE]
                continue

            # Bad checksum: drop one byte (the header) and resync
            del self._rx_buf[:1]

        return packets

    def reset_odom_callback(self, request, response):
        # Reset posisi odom tanpa mengubah koneksi serial.
        with self.feedback_lock:
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self._last_feedback_stamp = None
            self._last_vx = 0.0
            self._last_vy = 0.0
            self._last_wz = 0.0

        self.get_logger().info("Odom reset: x=0 y=0 yaw=0")
        return response

    def calculate_checksum(self, data):
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum & 0xFF
    
    def verify_checksum(self, packet):
        if len(packet) < 2:
            return False
        data = packet[:-1]
        received_checksum = packet[-1]
        calculated = self.calculate_checksum(data)
        return calculated == received_checksum
    
    def build_command_packet(self, rpm1, rpm2, rpm3):
        """Build 8-byte command: [0xA5, RPM1(2), RPM2(2), RPM3(2), XOR]"""
        rpm1 = int(max(min(rpm1, self.max_rpm), -self.max_rpm))
        rpm2 = int(max(min(rpm2, self.max_rpm), -self.max_rpm))
        rpm3 = int(max(min(rpm3, self.max_rpm), -self.max_rpm))
        
        rpm_bytes = struct.pack('>hhh', rpm1, rpm2, rpm3)
        packet = bytes([PACKET_HEADER]) + rpm_bytes
        checksum = self.calculate_checksum(packet)
        packet += bytes([checksum])
        
        return packet
    
    def parse_feedback_packet(self, packet):
        """
        Parse 18-byte feedback: [0xA5, Tick1(4), Tick2(4), Tick3(4), GyroZ(2), AngleX(2), XOR]
        
        ESP32 sends:
        - gyro_z_scaled = gyro_z_rad_s * 1000
        - angle_x_scaled = angle_x_rad * 1000
        """
        if len(packet) != FEEDBACK_PACKET_SIZE:
            return None
        
        if packet[0] != PACKET_HEADER:
            return None
        
        if not self.verify_checksum(packet):
            self.get_logger().warn("Feedback checksum mismatch!")
            return None
        
        try:
            ticks = struct.unpack('>iii', packet[1:13])
            imu_raw = struct.unpack('>hh', packet[13:17])
            
            gyro_z = imu_raw[0] / 1000.0
            angle_x_rad = imu_raw[1] / 1000.0
            
            return (list(ticks), gyro_z, angle_x_rad)
        except struct.error as e:
            self.get_logger().error(f"Struct unpack error: {e}")
            return None

    def cmd_vel_callback(self, msg):
        """Convert /cmd_vel to wheel RPMs using kiwi drive kinematics"""
        self.last_cmd_stamp = self.get_clock().now()
        self.watchdog_triggered = False
        
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        self.get_logger().info(f"Received cmd_vel: vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}")

        v1, v2, v3 = kinematics.cmd_vel_to_wheel_velocities(vx, vy, wz, self.robot_radius)
        rpm1 = kinematics.velocity_to_rpm(v1, self.wheel_radius)
        rpm2 = kinematics.velocity_to_rpm(v2, self.wheel_radius)
        rpm3 = kinematics.velocity_to_rpm(v3, self.wheel_radius)
        
        self.get_logger().info(f"Wheel velocities: v1={v1:.3f} v2={v2:.3f} v3={v3:.3f} m/s")

        self.send_rpm_command(rpm1, rpm2, rpm3)
    
    def send_rpm_command(self, rpm1, rpm2, rpm3):
        packet = self.build_command_packet(rpm1, rpm2, rpm3)
        
        try:
            bytes_written = self.ser.write(packet)
            hex_str = ' '.join(f'{b:02X}' for b in packet)
            self.get_logger().info(f"TX [{bytes_written}/{len(packet)} bytes]: {hex_str}")
            self.get_logger().info(f"RPM Command -> M1:{int(rpm1)} M2:{int(rpm2)} M3:{int(rpm3)}")
        except Exception as e:
            self.get_logger().error(f"Serial Write Error: {e}")
    
    def send_stop_command(self):
        self.send_rpm_command(0, 0, 0)
        self.get_logger().warn("STOP command sent!")
    
    def read_feedback_callback(self):
        """Read and parse incoming feedback packets from ESP32"""
        n = int(getattr(self.ser, 'in_waiting', 0) or 0)
        if n > 0:
            chunk = self.ser.read(n)
            self._feed_rx(chunk)
        else:
            return

        packets = self._extract_feedback_packets()
        if not packets:
            return

        samples = []
        for packet in packets:
            result = self.parse_feedback_packet(packet)
            if not result:
                continue
            samples.append(result)

        if not samples:
            return

        stamp_now = self.get_clock().now()
        self._handle_feedback_batch(samples, stamp_now)
        self.publish_feedback()

    def _handle_feedback_batch(self, samples, stamp_now):
        """Update shared state and integrate odometry for 1..N samples.

        If multiple packets are received in one callback, dt is distributed evenly
        over the elapsed time since the last callback to avoid dt≈0 integration.
        """
        with self.feedback_lock:
            # Always publish the latest sample values
            ticks_last, gyro_z_last, angle_x_last = samples[-1]
            self.encoder_ticks = ticks_last
            self.gyro_z = gyro_z_last
            self.angle_x_rad = angle_x_last

            if self._last_feedback_stamp is None:
                self._last_feedback_stamp = stamp_now
                return

            dt_total = (stamp_now - self._last_feedback_stamp).nanoseconds / 1e9
            self._last_feedback_stamp = stamp_now

            # Guard against bad dt (startup jitter / backlog / clock issues)
            if dt_total <= 0.0 or dt_total > 0.2:
                return

            dt_each = dt_total / float(len(samples))
            if dt_each <= 0.0:
                return

            vx_last = 0.0
            vy_last = 0.0
            wz_last = 0.0

            for ticks, gyro_z, _angle_x_rad in samples:
                vx, vy, _wz_enc = kinematics.delta_ticks_to_body_twist(
                    ticks,
                    self.ticks_per_rev,
                    self.wheel_radius,
                    self.robot_radius,
                    dt_each,
                )

                # Integrate yaw from gyro (preferred over encoder yaw on omni)
                yaw_prev = self.yaw
                delta_yaw = gyro_z * dt_each
                yaw_mid = yaw_prev + 0.5 * delta_yaw
                self.yaw = yaw_prev + delta_yaw

                # Integrate translation in odom frame
                dx_body = vx * dt_each
                dy_body = vy * dt_each

                cos_y = math.cos(yaw_mid)
                sin_y = math.sin(yaw_mid)
                dx = dx_body * cos_y - dy_body * sin_y
                dy = dx_body * sin_y + dy_body * cos_y

                self.x += dx
                self.y += dy

                vx_last = vx
                vy_last = vy
                wz_last = gyro_z

            # Cache twist for publishing (last sample)
            self._last_vx = vx_last
            self._last_vy = vy_last
            self._last_wz = wz_last
            self._last_odom_stamp = stamp_now
    
    def publish_feedback(self):
        stamp = getattr(self, '_last_odom_stamp', None)
        if stamp is None:
            stamp = self.get_clock().now()

        with self.feedback_lock:
            ticks = list(self.encoder_ticks)
            gyro_z = float(self.gyro_z)
            angle_x_rad = float(self.angle_x_rad)
            x = float(self.x)
            y = float(self.y)
            yaw = float(self.yaw)
            vx = float(getattr(self, '_last_vx', 0.0))
            vy = float(getattr(self, '_last_vy', 0.0))
            wz = float(getattr(self, '_last_wz', 0.0))

        # Debug topics
        enc_msg = Float32MultiArray()
        enc_msg.data = [float(t) for t in ticks]
        self.encoder_pub.publish(enc_msg)

        imu_raw_msg = Float32MultiArray()
        imu_raw_msg.data = [gyro_z, angle_x_rad]
        self.imu_pub.publish(imu_raw_msg)

        # Standard IMU
        imu_msg = Imu()
        imu_msg.header.stamp = stamp.to_msg()
        imu_msg.header.frame_id = self.base_frame_id
        imu_msg.angular_velocity.z = gyro_z
        # Unknown orientation/accel (not provided)
        imu_msg.orientation_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0
        self.imu_data_pub.publish(imu_msg)

        # Odometry
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y

        half = 0.5 * yaw
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        # TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(half)
        t.transform.rotation.w = math.cos(half)
        self.tf_broadcaster.sendTransform(t)

        # Throttled debug logs (ROS time)
        if not hasattr(self, '_last_imu_log_stamp'):
            self._last_imu_log_stamp = stamp
        if (stamp - self._last_imu_log_stamp).nanoseconds / 1e9 > 1.0:
            self.get_logger().debug(
                f"IMU: gyro_z={gyro_z:.4f} rad/s, angle_x={angle_x_rad:.4f} rad | Odom: x={x:.3f} y={y:.3f} yaw={yaw:.3f}"
            )
            self._last_imu_log_stamp = stamp
    
    def watchdog_callback(self):
        """Safety: stop motors if no cmd_vel received within timeout"""
        elapsed = (self.get_clock().now() - self.last_cmd_stamp).nanoseconds / 1e9
        
        if elapsed > self.watchdog_timeout and not self.watchdog_triggered:
            self.get_logger().warn(f"WATCHDOG: No cmd_vel for {elapsed:.2f}s - Stopping motors!")
            self.send_stop_command()
            self.watchdog_triggered = True

def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
        node.send_stop_command()
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
