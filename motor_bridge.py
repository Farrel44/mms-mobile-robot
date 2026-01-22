#!/usr/bin/env python3
"""
ROS2 node bridging /cmd_vel to ESP32 motor controller via binary serial protocol.
Implements 500ms watchdog for safety and publishes encoder + IMU feedback.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import serial
import struct
import time
import math
import threading

PACKET_HEADER = 0xA5
CMD_PACKET_SIZE = 8
FEEDBACK_PACKET_SIZE = 18

class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge_node')
        
        self.wheel_radius = 0.05
        self.robot_radius = 0.10
        self.max_rpm = 600
        
        self.watchdog_timeout = 0.5
        self.last_cmd_time = time.time()
        self.watchdog_triggered = False
            
        # Thread-safe feedback state shared between serial reader and publishers
        self.encoder_ticks = [0, 0, 0]
        self.gyro_z = 0.0
        self.accel_z = 0.0
        self.feedback_lock = threading.Lock()
        
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.01)
            self._init_serial_connection()
            self.get_logger().info("Serial connected to ESP32")
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            exit()

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.encoder_pub = self.create_publisher(Float32MultiArray, '/encoder_ticks', 10)
        self.imu_pub = self.create_publisher(Float32MultiArray, '/imu_raw', 10)
        
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)
        self.serial_read_timer = self.create_timer(0.02, self.read_feedback_callback)
        
        self.get_logger().info("🚀 Motor bridge ready")

    def _init_serial_connection(self):
        """Sync to ESP32 packet stream, preventing boot message corruption in buffer"""
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        time.sleep(2.5)
        
        self.ser.write(b'b')
        time.sleep(0.3)
        
        if self.ser.in_waiting > 0:
            garbage = self.ser.read(self.ser.in_waiting)
            self.get_logger().info(f"Cleared {len(garbage)} bytes boot data")
        
        if self._sync_to_header(max_attempts=500):
            self.ser.read(17)
            self.ser.reset_input_buffer()
            self.get_logger().info("Synced to packet stream")
    
    def _sync_to_header(self, max_attempts=100):
        """Find 0xA5 header without CPU spinning on empty buffer"""
        for _ in range(max_attempts):
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                if len(byte) > 0 and byte[0] == PACKET_HEADER:
                    return True
            else:
                time.sleep(0.001)
        return False

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

    def velocity_to_rpm(self, wheel_velocity):
        rpm = (wheel_velocity * 60.0) / (2.0 * math.pi * self.wheel_radius)
        return rpm
    
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
        Parse 18-byte feedback: [0xA5, Tick1(4), Tick2(4), Tick3(4), GyroZ(2), Pitch(2), XOR]
        ESP32 scales IMU by 1000, we divide back to rad/s and rad.
        """
        if len(packet) != FEEDBACK_PACKET_SIZE:
            return None
        
        if packet[0] != PACKET_HEADER:
            return None
        
        if not self.verify_checksum(packet):
            self.get_logger().warn("⚠️ Feedback checksum mismatch!")
            return None
        
        try:
            ticks = struct.unpack('>iii', packet[1:13])
            imu_raw = struct.unpack('>hh', packet[13:17])
            
            gyro_z = imu_raw[0] / 1000.0
            accel_z = imu_raw[1] / 1000.0
            
            return (list(ticks), gyro_z, accel_z)
        except struct.error as e:
            self.get_logger().error(f"Struct unpack error: {e}")
            return None

    def cmd_vel_callback(self, msg):
        """Convert /cmd_vel to wheel RPMs using kiwi drive kinematics"""
        self.last_cmd_time = time.time()
        self.watchdog_triggered = False
        
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        rotation_vel = self.robot_radius * wz
        
        v1 = -0.5 * vx - 0.866 * vy + rotation_vel
        v2 = -0.5 * vx + 0.866 * vy + rotation_vel
        v3 = vx + rotation_vel
    
        rpm1 = self.velocity_to_rpm(v1)
        rpm2 = self.velocity_to_rpm(v2)
        rpm3 = self.velocity_to_rpm(v3)

        self.send_rpm_command(rpm1, rpm2, rpm3)
    
    def send_rpm_command(self, rpm1, rpm2, rpm3):
        packet = self.build_command_packet(rpm1, rpm2, rpm3)
        
        try:
            self.ser.write(packet)
            hex_str = ' '.join(f'{b:02X}' for b in packet)
            self.get_logger().debug(f"TX [{len(packet)}]: {hex_str}")
            self.get_logger().info(f"Sent RPM: {int(rpm1)}, {int(rpm2)}, {int(rpm3)}")
        except Exception as e:
            self.get_logger().error(f"Serial Write Error: {e}")
    
    def send_stop_command(self):
        self.send_rpm_command(0, 0, 0)
        self.get_logger().warn("🛑 STOP command sent!")
    
    def read_feedback_callback(self):
        """Read and parse incoming feedback packets from ESP32"""
        if self.ser.in_waiting >= FEEDBACK_PACKET_SIZE:
            while self.ser.in_waiting > 0:
                first_byte = self.ser.read(1)
                if len(first_byte) == 0:
                    break
                    
                if first_byte[0] == PACKET_HEADER:
                    remaining = self.ser.read(FEEDBACK_PACKET_SIZE - 1)
                    if len(remaining) == FEEDBACK_PACKET_SIZE - 1:
                        packet = first_byte + remaining
                        result = self.parse_feedback_packet(packet)
                        
                        if result:
                            ticks, gyro_z, accel_z = result
                            
                            with self.feedback_lock:
                                self.encoder_ticks = ticks
                                self.gyro_z = gyro_z
                                self.accel_z = accel_z
                            
                            self.publish_feedback()
                            
                            self.get_logger().debug(
                                f"RX Ticks: {ticks}, Gyro: {gyro_z:.3f}, Accel: {accel_z:.3f}")
                    break
    
    def publish_feedback(self):
        enc_msg = Float32MultiArray()
        enc_msg.data = [float(t) for t in self.encoder_ticks]
        self.encoder_pub.publish(enc_msg)
        
        imu_msg = Float32MultiArray()
        imu_msg.data = [self.gyro_z, self.accel_z]
        self.imu_pub.publish(imu_msg)
        
        if not hasattr(self, '_last_imu_log_time'):
            self._last_imu_log_time = 0
        
        import time as t
        current_time = t.time()
        if current_time - self._last_imu_log_time > 1.0:
            self.get_logger().debug(
                f"IMU: gyro_z={self.gyro_z:.4f} rad/s, accel_z={self.accel_z:.2f} m/s²"
            )
            self._last_imu_log_time = current_time
    
    def watchdog_callback(self):
        """Safety: stop motors if no cmd_vel received within timeout"""
        elapsed = time.time() - self.last_cmd_time
        
        if elapsed > self.watchdog_timeout and not self.watchdog_triggered:
            self.get_logger().warn(f"⚠️ WATCHDOG: No cmd_vel for {elapsed:.2f}s - Stopping motors!")
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
