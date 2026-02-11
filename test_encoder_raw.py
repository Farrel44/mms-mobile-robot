#!/usr/bin/env python3
"""
Direct encoder feedback test: Run motor at high speed and watch encoder values.
If encoder stays 0, problem is ESP32 encoder reading (wiring or firmware).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import time

class EncoderRawTest(Node):
    def __init__(self):
        super().__init__('encoder_raw_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            Float32MultiArray, '/encoder_ticks', self.encoder_cb, qos_profile_sensor_data)
        
        # /encoder_ticks dari mms_bridge adalah delta ticks per feedback packet.
        # Untuk memastikan encoder "hidup", kita akumulasi delta selama window test.
        self.latest_delta = [0.0, 0.0, 0.0]
        self.integrated_ticks = [0.0, 0.0, 0.0]
        self.encoder_count = 0
        self.encoder_changed = False
        
    def encoder_cb(self, msg):
        self.encoder_count += 1
        if len(msg.data) >= 3:
            self.latest_delta = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2])]
            self.integrated_ticks[0] += self.latest_delta[0]
            self.integrated_ticks[1] += self.latest_delta[1]
            self.integrated_ticks[2] += self.latest_delta[2]

        if any(abs(x) > 0.1 for x in self.latest_delta):
            self.encoder_changed = True
            
    def run_high_speed_test(self):
        print('\n' + '='*70)
        print('TEST ENCODER FEEDBACK - HIGH SPEED')
        print('='*70)
        print('Mengirim cmd_vel dengan kecepatan TINGGI selama 5 detik')
        print('Jika encoder tetap 0 → Problem di ESP32 (wiring/firmware)\n')
        
        # High speed forward
        twist = Twist()
        twist.linear.x = 0.3  # 30 cm/s

        # Reset integrated measurement window
        self.integrated_ticks = [0.0, 0.0, 0.0]
        self.encoder_changed = False
        self.encoder_count = 0
        
        print('⏱️  Running motor at 0.3 m/s for 5 seconds...')
        start_time = time.time()
        
        while time.time() - start_time < 5.0:
            self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            
            # Print live updates
            if self.encoder_count % 50 == 0:
                print(
                    f'  [{time.time()-start_time:.1f}s] Delta: '
                    f'M1={self.latest_delta[0]:+.0f}, M2={self.latest_delta[1]:+.0f}, M3={self.latest_delta[2]:+.0f} | '
                    f'Integrated: M1={self.integrated_ticks[0]:+.0f}, M2={self.integrated_ticks[1]:+.0f}, M3={self.integrated_ticks[2]:+.0f}'
                )
        
        # Stop
        twist.linear.x = 0.0
        self.pub.publish(twist)
        time.sleep(0.2)
        
        print('\n' + '='*70)
        print('HASIL:')
        print('='*70)
        print(f'Total encoder callbacks: {self.encoder_count}')
        print(
            f'Last delta ticks: M1={self.latest_delta[0]:+.0f}, '
            f'M2={self.latest_delta[1]:+.0f}, M3={self.latest_delta[2]:+.0f}'
        )
        print(
            f'Integrated ticks (5s): M1={self.integrated_ticks[0]:+.0f}, '
            f'M2={self.integrated_ticks[1]:+.0f}, M3={self.integrated_ticks[2]:+.0f}'
        )
        
        if any(abs(x) > 5.0 for x in self.integrated_ticks):
            print('✅ Encoder BERUBAH - hardware OK!')
        else:
            print('❌ Encoder TETAP 0 - Problem di ESP32!')
            print('\nKemungkinan penyebab:')
            print('  1. Encoder cable tidak terhubung ke ESP32')
            print('  2. ESP32 pin encoder salah (A/B channels)')
            print('  3. Firmware ESP32 tidak setup encoder interrupt')
            print('  4. Encoder rusak (tapi motor jalan berarti driver OK)')
            print('\n📋 Action required:')
            print('  • Check ESP32 wiring: Encoder A/B pin connections')
            print('  • Verify firmware: encoder interrupt setup')
            print('  • Test encoder with multimeter (should pulse when motor runs)')


def main():
    rclpy.init()
    node = EncoderRawTest()
    
    try:
        node.run_high_speed_test()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
