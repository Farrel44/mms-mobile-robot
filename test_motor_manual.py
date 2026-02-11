#!/usr/bin/env python3
"""
Test manual motor: Test satu-satu motor untuk verify mapping dan direction.

Expected behavior untuk kiwi drive (120° layout):
  - Motor 1 (Roda di 120°): di belakang kiri
  - Motor 2 (Roda di 240°): di belakang kanan  
  - Motor 3 (Roda di 0°):   di depan tengah

Untuk MAJU (vx=0.2):
  - M1 harus putar MUNDUR (RPM negatif)
  - M2 harus putar MUNDUR (RPM negatif)
  - M3 harus putar MAJU (RPM positif)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import time
import sys

class MotorTest(Node):
    def __init__(self):
        super().__init__('motor_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Command yang akan di-hold (dipublish berkala) selama window test.
        self.active_cmd = Twist()
        
        # Subscribe to encoder feedback with sensor_data QoS (BEST_EFFORT)
        self.encoder_sub = self.create_subscription(
            Float32MultiArray, '/encoder_ticks', self.encoder_callback, 
            qos_profile_sensor_data)
        
        # Encoder data
        # NOTE: /encoder_ticks dari mms_bridge adalah *delta ticks* per feedback packet.
        # Jadi untuk "berapa banyak roda bergerak" selama window test,
        # kita perlu *integrate/sum* delta ticks, bukan ambil selisih baseline.
        self.encoder_delta = [0.0, 0.0, 0.0]
        self.encoder_received = False
        self.encoder_integrated = [0.0, 0.0, 0.0]
        self.integrate_enabled = False
        
        time.sleep(0.5)  # Wait for connections
        
    def encoder_callback(self, msg):
        """Save latest encoder delta ticks and optionally integrate."""
        if len(msg.data) >= 3:
            self.encoder_delta = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2])]
            self.encoder_received = True

            if self.integrate_enabled:
                self.encoder_integrated[0] += self.encoder_delta[0]
                self.encoder_integrated[1] += self.encoder_delta[1]
                self.encoder_integrated[2] += self.encoder_delta[2]
    
    def reset_integrator(self):
        """Reset integrated ticks accumulator (for a new measurement window)."""
        self.encoder_integrated = [0.0, 0.0, 0.0]

    def start_integrating(self):
        self.reset_integrator()
        self.integrate_enabled = True

    def stop_integrating(self):
        self.integrate_enabled = False
        
    def get_delta(self):
        """Return integrated delta ticks for the active window."""
        return self.encoder_integrated.copy()
    
    def print_encoder_status(self):
        """Print current encoder readings."""
        delta = self.get_delta()
        print(f'\n  Encoder ticks (integrated delta over test window):')
        print(f'    M1: {delta[0]:+8.0f}  {"✓ BERGERAK" if abs(delta[0]) > 10 else ""}')
        print(f'    M2: {delta[1]:+8.0f}  {"✓ BERGERAK" if abs(delta[1]) > 10 else ""}')
        print(f'    M3: {delta[2]:+8.0f}  {"✓ BERGERAK" if abs(delta[2]) > 10 else ""}')
        print(f'  Arah: M1={self._get_direction(delta[0])}, M2={self._get_direction(delta[1])}, M3={self._get_direction(delta[2])}')
        
    def _get_direction(self, delta):
        """Get rotation direction from delta."""
        if abs(delta) < 10:
            return "DIAM"
        elif delta > 0:
            return "MAJU(+)"
        else:
            return "MUNDUR(-)"
        
    def stop(self):
        """Stop all motors."""
        msg = Twist()
        self.active_cmd = msg
        self.pub.publish(msg)
        print('  → STOP: All motors off')
        
    def test_forward(self, speed=0.1):
        """Test forward motion (MAJU)."""
        msg = Twist()
        msg.linear.x = speed
        self.active_cmd = msg
        self.pub.publish(msg)
        print(f'  → MAJU: vx={speed} m/s')
        print(f'  → Expected: M1 mundur(-), M2 mundur(-), M3 maju(+)')
        
    def test_backward(self, speed=0.1):
        """Test backward motion (MUNDUR)."""
        msg = Twist()
        msg.linear.x = -speed
        self.active_cmd = msg
        self.pub.publish(msg)
        print(f'  → MUNDUR: vx={-speed} m/s')
        print(f'  → Expected: M1 maju(+), M2 maju(+), M3 mundur(-)')
        
    def test_rotate_ccw(self, speed=0.5):
        """Test counter-clockwise rotation (PUTAR KIRI)."""
        msg = Twist()
        msg.angular.z = speed
        self.active_cmd = msg
        self.pub.publish(msg)
        print(f'  → PUTAR KIRI (CCW): wz={speed} rad/s')
        print(f'  → Expected: M1 maju(+), M2 maju(+), M3 maju(+)')
        
    def test_rotate_cw(self, speed=0.5):
        """Test clockwise rotation (PUTAR KANAN)."""
        msg = Twist()
        msg.angular.z = -speed
        self.active_cmd = msg
        self.pub.publish(msg)
        print(f'  → PUTAR KANAN (CW): wz={-speed} rad/s')
        print(f'  → Expected: M1 mundur(-), M2 mundur(-), M3 mundur(-)')
        
    def test_strafe_left(self, speed=0.1):
        """Test strafe left (KIRI)."""
        msg = Twist()
        msg.linear.y = speed
        self.active_cmd = msg
        self.pub.publish(msg)
        print(f'  → STRAFE KIRI: vy={speed} m/s')
        print(f'  → Expected: M1 mundur(-), M2 maju(+), M3 diam')
    
    def test_strafe_right(self, speed=0.1):
        """Test strafe right (KANAN)."""
        msg = Twist()
        msg.linear.y = -speed
        self.active_cmd = msg
        self.pub.publish(msg)
        print(f'  → STRAFE KANAN: vy={-speed} m/s')
        print(f'  → Expected: M1 maju(+), M2 mundur(-), M3 diam')

def run_test(node, test_name, test_func):
    """Run a single test with encoder monitoring."""
    print(f'\n{"="*70}')
    print(f'TEST: {test_name}')
    print(f'{"="*70}')
    
    # Wait for encoder data
    if not node.encoder_received:
        print('⏳ Menunggu data encoder...')
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.encoder_received:
                break
        if not node.encoder_received:
            print('❌ WARNING: Encoder tidak terdeteksi! Data mungkin tidak akurat.')
    
    # Reset integrated delta accumulator for this test window
    node.start_integrating()

    # Warn kalau ada publisher lain yang ikut publish /cmd_vel
    try:
        pub_count = node.count_publishers('/cmd_vel')
        if pub_count > 1:
            print(f'⚠️  WARNING: Ada {pub_count} publisher di /cmd_vel. Hasil test bisa tercampur!')
    except Exception:
        pass
    
    # Run test
    test_func()
    
    # Monitor for 2 seconds
    print('\n⏱️  Monitoring encoder selama 2 detik...')
    start_time = time.time()
    last_status_print = 0.0
    while time.time() - start_time < 2.0:
        # Hold command (publish berkala) supaya tidak kena watchdog bridge/ESP
        node.pub.publish(node.active_cmd)
        rclpy.spin_once(node, timeout_sec=0.1)
        # Update display every 0.5 seconds
        elapsed = time.time() - start_time
        if elapsed - last_status_print >= 0.5:
            last_status_print = elapsed
            sys.stdout.write('\r  Elapsed: {:.1f}s'.format(elapsed))
            sys.stdout.flush()
    
    print('\n')
    
    # Stop integrating, then stop motors and show results
    node.stop_integrating()
    node.stop()
    time.sleep(0.3)
    rclpy.spin_once(node, timeout_sec=0.1)
    
    # Display encoder delta
    node.print_encoder_status()
    
    return True

def main():
    rclpy.init()
    node = MotorTest()
    
    print('\n' + '='*70)
    print(' '*20 + 'TEST MOTOR & ENCODER')
    print('='*70)
    print('Tujuan:')
    print('  1. Verify mapping motor M1/M2/M3 ke roda fisik')
    print('  2. Verify direction (maju/mundur) tiap motor')
    print('  3. Check encoder feedback dari tiap motor')
    print()
    print('Instruksi:')
    print('  - Robot HARUS angkat (roda tidak menyentuh lantai)')
    print('  - Amati arah putaran tiap roda')
    print('  - Bandingkan dengan expected behavior dan encoder delta')
    print('='*70)
    
    try:
        input('\nPress ENTER untuk mulai test...')
        
        # Wait for initial encoder data
        print('\n📡 Waiting for encoder data...')
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.encoder_received:
                print('✅ Encoder connected!')
                break
        else:
            print('⚠️  WARNING: No encoder data received. Continuing anyway...')
        
        # Test sequence
        tests = [
            ('1. MAJU (Forward)', lambda: node.test_forward(0.15)),
            ('2. MUNDUR (Backward)', lambda: node.test_backward(0.15)),
            ('3. PUTAR KIRI (CCW)', lambda: node.test_rotate_ccw(0.6)),
            ('4. PUTAR KANAN (CW)', lambda: node.test_rotate_cw(0.6)),
            ('5. STRAFE KIRI (Left)', lambda: node.test_strafe_left(0.15)),
            ('6. STRAFE KANAN (Right)', lambda: node.test_strafe_right(0.15)),
        ]
        
        for test_name, test_func in tests:
            if not run_test(node, test_name, test_func):
                break
            input('\nPress ENTER untuk test berikutnya (atau Ctrl+C untuk stop)...')
        
        # Summary
        print('\n' + '='*70)
        print(' '*25 + 'ANALISA HASIL')
        print('='*70)
        print('\n✅ HASIL BENAR jika:')
        print('  • Arah putaran motor sesuai expected behavior')
        print('  • Encoder delta sesuai arah (positif=maju, negatif=mundur)')
        print('  • Magnitude encoder delta proporsional dengan kecepatan')
        print()
        print('❌ MASALAH jika:')
        print('  • Arah putaran terbalik → Fix di firmware (negate RPM)')
        print('  • Encoder tidak berubah → Check wiring encoder')
        print('  • Motor salah yang jalan → Check mapping M1/M2/M3 di wiring')
        print()
        print('📋 EXPECTED BEHAVIOR (kiwi 120° layout):')
        print('┌──────────────┬────────┬────────┬────────┐')
        print('│ Command      │   M1   │   M2   │   M3   │')
        print('├──────────────┼────────┼────────┼────────┤')
        print('│ MAJU         │   (-)  │   (-)  │   (+)  │')
        print('│ MUNDUR       │   (+)  │   (+)  │   (-)  │')
        print('│ PUTAR KIRI   │   (+)  │   (+)  │   (+)  │')
        print('│ PUTAR KANAN  │   (-)  │   (-)  │   (-)  │')
        print('│ STRAFE KIRI  │   (-)  │   (+)  │    0   │')
        print('│ STRAFE KANAN │   (+)  │   (-)  │    0   │')
        print('└──────────────┴────────┴────────┴────────┘')
        print()
        print('Keterangan:')
        print('  M1 = Motor 1 (Roda di 120° = belakang kiri)')
        print('  M2 = Motor 2 (Roda di 240° = belakang kanan)')
        print('  M3 = Motor 3 (Roda di 0° = depan tengah)')
        print('  (+) = Encoder increase (roda maju)')
        print('  (-) = Encoder decrease (roda mundur)')
        print('='*70)
        
    except KeyboardInterrupt:
        print('\n\n⚠️  Test dibatalkan oleh user.')
    finally:
        node.stop()
        time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
