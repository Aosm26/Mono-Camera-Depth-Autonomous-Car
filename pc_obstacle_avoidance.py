#!/usr/bin/env python3
"""
PC Obstacle Avoidance Controller Node — Hysteresis + Potansiyel Alan
Salınımı (oscillation) önlemek için yön taahhüdü (direction commitment) ve
U-dönüşü engellemek için her zaman ileri hareket tercihi kullanır.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import time

# ANSI Colors
class Colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class PcObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('pc_obstacle_avoidance')
        
        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )
        
        # ── Hız Parametreleri ──
        self.max_linear_speed = 0.7
        self.max_angular_speed = 0.9    # Azaltıldı - daha yumuşak dönüşler

        # ── Mesafe Eşikleri ──
        self.critical_distance = 0.4    # DUR + GERİ
        self.brake_distance = 2.5       # Yavaşlamaya başla (düşürüldü - daha az konservatif)
        self.side_clearance = 1.5       # Yan güvenlik (düşürüldü - kenar sürtme dengeli)
        
        # ── Şerit Ayarı ──
        self.strip_ratio = 0.30
        self.max_depth = 8.0

        # ── Anti-Salınım: Yön Taahhüdü (Hysteresis) ──
        self.committed_direction = 0     # -1: sağa, +1: sola, 0: taahhüt yok
        self.commit_start_time = 0.0
        self.min_commit_duration = 1.0   # Bir yöne karar verdikten sonra en az 1 saniye devam et
        
        # ── Yumuşatma (Smoothing) ──
        self.prev_angular = 0.0
        self.smooth_factor = 0.3         # 0=tamamen eski, 1=tamamen yeni (düşük=daha yumuşak)

        self.get_logger().info(f"{Colors.OKGREEN}{Colors.BOLD}Obstacle Avoidance (Anti-Oscillation) started{Colors.ENDC}")
        print("="*65)
        print(f"{Colors.BOLD}   Otonom Araç — Hysteresis + Potansiyel Alan{Colors.ENDC}")
        print(f"{Colors.OKCYAN}   Anti-Salınım | İleri Tercih | Yumuşak Direksiyon{Colors.ENDC}")
        print("="*65)

    def depth_callback(self, msg):
        # ── Depth image → numpy ──
        if msg.encoding == '32FC1':
            depth_array = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        elif msg.encoding == '16UC1':
            depth_array = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            depth_array = depth_array.astype(np.float32) / 1000.0
        else:
            return

        h, w = depth_array.shape
        depth_array = np.where(np.isnan(depth_array) | np.isinf(depth_array), self.max_depth, depth_array)
        depth_array = np.clip(depth_array, 0.0, self.max_depth)

        # ── YATAY ORTA ŞERİTİ AL ──
        band_h = max(20, int(h * self.strip_ratio))
        mid_start = h // 2 - band_h // 2
        mid_end = h // 2 + band_h // 2
        strip = depth_array[mid_start:mid_end, :]

        # Her sütun için MİNİMUM derinlik (en yakın engeli yakala)
        horizontal_profile = np.min(strip, axis=0)

        # ── 5 SEKTÖR ──
        fifth = w // 5
        min_far_left  = float(np.min(horizontal_profile[0:fifth]))
        min_left      = float(np.min(horizontal_profile[fifth:2*fifth]))
        min_center    = float(np.min(horizontal_profile[2*fifth:3*fifth]))
        min_right     = float(np.min(horizontal_profile[3*fifth:4*fifth]))
        min_far_right = float(np.min(horizontal_profile[4*fifth:]))
        
        avg_left_zone  = float(np.mean(horizontal_profile[0:2*fifth]))       # Sol yarı ortalama
        avg_right_zone = float(np.mean(horizontal_profile[3*fifth:]))        # Sağ yarı ortalama
        
        global_min = float(np.min(horizontal_profile))
        now = time.time()

        # ─────────────────────────────────────
        #  KARAR MANTIĞI
        # ─────────────────────────────────────

        twist = Twist()
        raw_angular = 0.0
        
        # Taahhüt süresi doldu mu kontrol et
        commit_active = (self.committed_direction != 0 and 
                        (now - self.commit_start_time) < self.min_commit_duration)

        # ── 1. KRİTİK: Çok yakın → GERİ ──
        if global_min < self.critical_distance:
            twist.linear.x = -0.15
            if commit_active:
                raw_angular = self.committed_direction * self.max_angular_speed
            elif avg_left_zone > avg_right_zone:
                raw_angular = self.max_angular_speed
                self._commit(1, now)
            else:
                raw_angular = -self.max_angular_speed
                self._commit(-1, now)
            label = f"{Colors.FAIL}🚨 KRİTİK → [GERİ+DÖN]{Colors.ENDC}"

        # ── 2. ENGEL ÖNDe: Yavaşla + Kaçın ──
        elif min_center < self.brake_distance:
            # Orantısal hız
            ratio = (min_center - self.critical_distance) / (self.brake_distance - self.critical_distance)
            ratio = np.clip(ratio, 0.0, 1.0)
            twist.linear.x = 0.1 + ratio * (self.max_linear_speed - 0.1)
            
            # Dönüş yönü: Taahhüt varsa onu kullan, yoksa yeni taahhüt oluştur
            steer_intensity = (1.0 - ratio) * self.max_angular_speed
            
            if commit_active:
                # Taahhüt devam ediyor — aynı yöne dönmeye devam et
                raw_angular = self.committed_direction * steer_intensity
            else:
                # Yeni taahhüt oluştur — en açık yöne karar ver ve kilitle
                if avg_left_zone > avg_right_zone:
                    raw_angular = steer_intensity
                    self._commit(1, now)
                else:
                    raw_angular = -steer_intensity
                    self._commit(-1, now)
            
            label = f"{Colors.WARNING}⚠️  ENGEL → [KAÇIN] v:{twist.linear.x:.2f}{Colors.ENDC}"

        # ── 3. YAN DÜZELTME: Çok hafif, U-dönüşü yapamayacak kadar ──
        elif min_left < self.side_clearance or min_right < self.side_clearance:
            twist.linear.x = self.max_linear_speed * 0.85  # Çok az yavaşla
            
            # Çok hafif düzeltme — agresif değil, sadece kaydırma
            left_push = max(0, self.side_clearance - min_left) / self.side_clearance
            right_push = max(0, self.side_clearance - min_right) / self.side_clearance
            net_push = left_push - right_push
            
            # Maksimum yan düzeltme sınırı: max_angular'ın %25'i (U-dönüşü imkansız)
            raw_angular = -net_push * self.max_angular_speed * 0.25
            
            # Yan düzeltme sırasında taahhüdü temizle
            self.committed_direction = 0
            
            side = "SOL" if left_push > right_push else "SAĞ"
            label = f"{Colors.OKCYAN}🔧 YAN ({side}) dönüş:{raw_angular:.2f}{Colors.ENDC}"

        # ── 4. YOL AÇIK ──
        else:
            twist.linear.x = self.max_linear_speed
            raw_angular = 0.0
            # Taahhüdü temizle — yol açıldığında serbest bırak
            self.committed_direction = 0
            label = f"{Colors.OKGREEN}✅ YOL AÇIK → [TAM HIZ]{Colors.ENDC}"
        
        # ── YUMUŞATMA: Ani dönüş değişikliklerini filtrele ──
        smoothed_angular = self.prev_angular * (1 - self.smooth_factor) + raw_angular * self.smooth_factor
        self.prev_angular = smoothed_angular
        twist.angular.z = smoothed_angular
        
        # Publish
        self.cmd_vel_pub.publish(twist)
        
        info = f" ({min_far_left:.1f}|{min_left:.1f}|{min_center:.1f}|{min_right:.1f}|{min_far_right:.1f} min:{global_min:.2f}m)"
        commit_info = f" [{'←' if self.committed_direction > 0 else '→' if self.committed_direction < 0 else '·'}]" if commit_active else ""
        print(label + info + commit_info)

    def _commit(self, direction, now):
        """Bir dönüş yönüne taahhüt et — salınımı önler"""
        if self.committed_direction != direction:
            self.committed_direction = direction
            self.commit_start_time = now

def main(args=None):
    rclpy.init(args=args)
    node = PcObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n{Colors.OKBLUE}Durduruluyor...{Colors.ENDC}")
    finally:
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
