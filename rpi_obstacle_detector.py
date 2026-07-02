#!/usr/bin/env python3
"""
Raspberry Pi 5 - Hailo-8L NPU Otonom Engel Algılama ve ROS2 Köprüsü
Bu script:
1. PC'deki ROS2'den (rosbridge) RGB kamera görüntüsünü alır.
2. Hailo-8L NPU üzerinde SCDepthV3 modeli ile derinlik tahmin eder.
3. Derinlik haritasını 2D LaserScan formatına dönüştürür.
4. LaserScan verisini PC'deki ROS2'ye geri gönderir.
"""

import cv2
import numpy as np
import time
import base64
import sys
import os
from hailo_platform import (
    HEF, VDevice, HailoStreamInterface,
    InferVStreams, ConfigureParams,
    InputVStreamParams, OutputVStreamParams,
    FormatType
)
import roslibpy

# ============ KONFİGÜRASYON ============
PC_IP = "127.0.0.1"  # PC'nizin IP adresini buraya yazın (örn: "192.168.1.50")
PORT = 9090          # Rosbridge websocket portu

HEF_PATH = "/usr/local/hailo/resources/models/hailo8l/scdepthv3.hef"

# Model giriş/çıkış çözünürlükleri (SCDepthV3 için)
MODEL_W = 320
MODEL_H = 256

# Sanal LaserScan Ayarları
FOV = 1.089          # Kameranın Yatay Görüş Açısı (Radyan cinsinden, ~62.4 derece)
MIN_RANGE = 0.2      # Minimum algılama mesafesi (metre)
MAX_RANGE = 6.0      # Maksimum algılama mesafesi (metre)
# =======================================

class RpiObstacleDetector:
    def __init__(self):
        print(f"SCDepthV3 HEF yükleniyor: {HEF_PATH}")
        if not os.path.exists(HEF_PATH):
            print(f"HATA: HEF dosyası bulunamadı! Lütfen yolu kontrol edin: {HEF_PATH}")
            sys.exit(1)

        self.hef = HEF(HEF_PATH)
        self.vdevice = VDevice()
        
        # Model Yapılandırması
        self.configure_params = ConfigureParams.create_from_hef(
            hef=self.hef, interface=HailoStreamInterface.PCIe
        )
        self.network_groups = self.vdevice.configure(self.hef, self.configure_params)
        self.network_group = self.network_groups[0]
        
        # Input/Output Stream Parametreleri
        self.input_vstream_params = InputVStreamParams.make(
            self.network_group, format_type=FormatType.UINT8
        )
        self.output_vstream_params = OutputVStreamParams.make(
            self.network_group, format_type=FormatType.FLOAT32
        )
        
        # Aktifleştirme
        self.activation = self.network_group.activate()
        self.activation.__enter__()
        self.infer_pipeline = InferVStreams(
            self.network_group, self.input_vstream_params, self.output_vstream_params
        )
        self.infer_pipeline.__enter__()

        # ROS Bridge Bağlantısı
        print(f"PC'ye bağlanılıyor... ({PC_IP}:{PORT})")
        self.client = roslibpy.Ros(host=PC_IP, port=PORT)
        
        # Publisher & Subscriber Tanımları
        self.scan_pub = roslibpy.Topic(self.client, '/scan', 'sensor_msgs/LaserScan')
        self.depth_img_pub = roslibpy.Topic(self.client, '/camera/depth/image_raw', 'sensor_msgs/Image')
        self.image_sub = roslibpy.Topic(self.client, '/camera/image_raw', 'sensor_msgs/Image')
        
        self.client.on_ready(self.on_ros_ready)
        self.is_connected = False
        
        self.last_time = time.time()
        self.frame_count = 0

    def on_ros_ready(self):
        print("ROS Bridge bağlantısı kuruldu! Dinleme başlatılıyor...")
        self.is_connected = True
        self.image_sub.subscribe(self.image_callback)

    def image_callback(self, msg):
        try:
            # 1. Görüntüyü ROS formatından OpenCV formatına dönüştür
            height = msg['height']
            width = msg['width']
            encoding = msg['encoding']
            
            # Base64 verisini decode et
            img_data = base64.b64decode(msg['data'])
            np_arr = np.frombuffer(img_data, dtype=np.uint8)
            
            if encoding == "rgb8":
                img = np_arr.reshape((height, width, 3))
            elif encoding == "bgr8":
                img = cv2.cvtColor(np_arr.reshape((height, width, 3)), cv2.COLOR_BGR2RGB)
            else:
                # Varsayılan deneme
                img = np_arr.reshape((height, width, 3))

            # 2. NPU Girişine Uygun Şekillendir (256x320)
            resized = cv2.resize(img, (MODEL_W, MODEL_H))
            input_data = np.expand_dims(resized, axis=0).astype(np.uint8)

            # 3. NPU Inference
            input_name = self.hef.get_input_vstream_infos()[0].name
            output_name = self.hef.get_output_vstream_infos()[0].name
            
            results = self.infer_pipeline.infer({input_name: input_data})
            depth_map = results[output_name][0]
            
            if len(depth_map.shape) == 3:
                depth_map = depth_map[:, :, 0]

            # 4. Derinliği LaserScan ve Derinlik Görüntüsüne Dönüştür
            self.publish_scan(depth_map, msg['header'])
            self.publish_depth_image(depth_map, msg['header'])

            # 5. FPS Göster
            self.frame_count += 1
            now = time.time()
            if now - self.last_time >= 1.0:
                fps = self.frame_count / (now - self.last_time)
                print(f"NPU Derinlik & ROS2 Aktarımı: {fps:.1f} FPS")
                self.frame_count = 0
                self.last_time = now

        except Exception as e:
            print(f"Görüntü işleme hatası: {e}")

    def publish_scan(self, depth_map, original_header):
        # SCDepthV3 log-depth veya normalize edilmemiş ters derinlik çıktısı üretir.
        # Bu değerleri makul metre aralıklarına doğrusal eşleriz.
        d_min = depth_map.min()
        d_max = depth_map.max()
        
        # Sıfıra bölme hatasını engelle
        if d_max == d_min:
            d_max += 0.001

        # Görüntünün ortasındaki birkaç satırı alıp dikey ortalamasını kullanalım
        mid_row_start = MODEL_H // 2 - 5
        mid_row_end = MODEL_H // 2 + 5
        horizontal_profile = np.mean(depth_map[mid_row_start:mid_row_end, :], axis=0)

        # Eşleme işlemi: Büyük değerler yakını, küçük değerler uzağı temsil eder
        norm_profile = (horizontal_profile - d_min) / (d_max - d_min)
        ranges = MIN_RANGE + (1.0 - norm_profile) * (MAX_RANGE - MIN_RANGE)
        
        # ROS LaserScan kuralına göre: Açı dizisi soldan sağa (artıdan eksiye) olmalı.
        # depth_map'in sol sütunu robotun solunu temsil eder.
        # ranges listesini LaserScan formatına hazırlıyoruz.
        ranges_list = ranges.tolist()

        # LaserScan Mesajı Oluştur
        scan_msg = {
            'header': {
                'stamp': original_header['stamp'], # Görüntünün çekildiği zaman damgasını koru
                'frame_id': 'camera_link'          # Robot modelindeki kamera linki
            },
            'angle_min': -FOV / 2.0,
            'angle_max': FOV / 2.0,
            'angle_increment': FOV / float(MODEL_W),
            'time_increment': 0.0,
            'scan_time': 0.06,  # ~15 FPS
            'range_min': MIN_RANGE,
            'range_max': MAX_RANGE,
            'ranges': ranges_list,
            'intensities': []
        }

        # PC'ye gönder
        if self.client.is_connected:
            self.scan_pub.publish(roslibpy.Message(scan_msg))

    def publish_depth_image(self, depth_map, original_header):
        try:
            d_min = depth_map.min()
            d_max = depth_map.max()
            if d_max == d_min:
                d_max += 0.001
            
            # Derinliği 0-255 arasına normalleştir
            norm_depth = ((depth_map - d_min) / (d_max - d_min) * 255).astype(np.uint8)
            
            # Renklendir (JET haritası: Yakın yerler kırmızı/turuncu, uzak yerler mavi/lacivert)
            color_depth = cv2.applyColorMap(norm_depth, cv2.COLORMAP_JET)
            
            # ROS'un standart rgb8 formatı için BGR'dan RGB'ye çevir
            color_depth_rgb = cv2.cvtColor(color_depth, cv2.COLOR_BGR2RGB)
            
            h, w, c = color_depth_rgb.shape
            raw_bytes = color_depth_rgb.tobytes()
            encoded_data = base64.b64encode(raw_bytes).decode('utf-8')
            
            depth_img_msg = {
                'header': {
                    'stamp': original_header['stamp'],
                    'frame_id': 'camera_depth_optical_frame'
                },
                'height': h,
                'width': w,
                'encoding': 'rgb8',
                'is_bigendian': 0,
                'step': w * c,
                'data': encoded_data
            }
            
            if self.client.is_connected:
                self.depth_img_pub.publish(roslibpy.Message(depth_img_msg))
        except Exception as e:
            print(f"Derinlik görüntüsü yayınlama hatası: {e}")

    def run(self):
        try:
            self.client.run_forever()
        except KeyboardInterrupt:
            print("Kapatılıyor...")
        finally:
            # Temizlik
            self.infer_pipeline.__exit__(None, None, None)
            self.activation.__exit__(None, None, None)
            self.vdevice.close()
            self.client.terminate()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        PC_IP = sys.argv[1]
    
    print(f"ROS Bridge sunucusuna bağlanılıyor: ws://{PC_IP}:{PORT}")
    detector = RpiObstacleDetector()
    detector.run()
