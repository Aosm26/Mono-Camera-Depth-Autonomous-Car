#!/usr/bin/env python3
"""
Raspberry Pi 5 - Hailo-8L NPU Otonom Engel Algılama ve ROS2 Köprüsü
Bu script:
1. PC'deki ROS2'den (rosbridge) RGB kamera görüntüsünü alır.
2. Hailo-8L NPU üzerinde SCDepthV3 modeli ile derinlik tahmin eder.
3. Derinlik haritasını 2D LaserScan formatına dönüştürür.
4. LaserScan verisini PC'deki ROS2'ye geri gönderir.
5. Canlı web yayını (MJPEG ve Telemetri) sunar.
"""

import cv2
import numpy as np
import time
import base64
import sys
import os
import threading
from flask import Flask, Response, render_template, jsonify
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

# Model Seçim Haritası
MODELS_MAP = {
    "scdepth": "/home/rpi5/autonomous_car_rpi/scdepthv3.hef",
    "fastdepth": "/home/rpi5/autonomous_car_rpi/fast_depth.hef",
}
DEFAULT_HEF_PATH = MODELS_MAP["scdepth"]

# Sanal LaserScan Ayarları
FOV = 1.089          # Kameranın Yatay Görüş Açısı (Radyan cinsinden, ~62.4 derece)
MIN_RANGE = 0.2      # Minimum algılama mesafesi (metre)
MAX_RANGE = 6.0      # Maksimum algılama mesafesi (metre)
# =======================================

class RpiObstacleDetector:
    def __init__(self, hef_path=DEFAULT_HEF_PATH):
        print(f"HEF yükleniyor: {hef_path}")
        if not os.path.exists(hef_path):
            print(f"HATA: HEF dosyası bulunamadı! Lütfen yolu kontrol edin: {hef_path}")
            sys.exit(1)

        self.hef = HEF(hef_path)
        
        # HEF dosyasından giriş çözünürlüklerini dinamik olarak al
        input_infos = self.hef.get_input_vstream_infos()
        self.model_h, self.model_w, self.model_c = input_infos[0].shape
        print(f"Model çözünürlüğü dinamik tespit edildi: {self.model_w}x{self.model_h}x{self.model_c}")

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
        self.image_sub = roslibpy.Topic(
            self.client, '/camera/image_raw/compressed', 'sensor_msgs/CompressedImage',
            queue_length=1      # ROS Bridge tarafında eski kareleri biriktirmeden sil
        )
        
        self.client.on_ready(self.on_ros_ready)
        self.is_connected = False
        
        self.last_time = time.time()
        self.frame_count = 0

        # Canlı Web Yayını & Paylaşılan Veri Değişkenleri
        self.data_lock = threading.Lock()
        self.latest_rgb_jpeg = None
        self.latest_depth_jpeg = None
        self.latest_ranges = []
        self.npu_fps = 0.0

        # Görüntü işleme kuyruğu ve senkronizasyon (Frame Dropping için)
        self.latest_msg = None
        self.msg_event = threading.Event()
        self.processing_running = True
        self.process_thread = threading.Thread(target=self.process_loop, daemon=True)
        self.process_thread.start()

        # Flask Web Sunucusu Kurulumu
        self.app = Flask(__name__)
        self.flask_thread = threading.Thread(target=self.run_flask, daemon=True)
        self.flask_thread.start()

    def run_flask(self):
        # Werkzeug loglarını sadece hata seviyesinde göstererek terminal kirliliğini önle
        import logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        
        @self.app.route('/')
        def index():
            return render_template('index.html')
            
        @self.app.route('/video_feed/rgb')
        def video_feed_rgb():
            return Response(self.gen_rgb(), mimetype='multipart/x-mixed-replace; boundary=frame')
            
        @self.app.route('/video_feed/depth')
        def video_feed_depth():
            return Response(self.gen_depth(), mimetype='multipart/x-mixed-replace; boundary=frame')
            
        @self.app.route('/telemetry')
        def telemetry():
            with self.data_lock:
                return jsonify({
                    'connected': self.is_connected,
                    'fps': self.npu_fps,
                    'pc_ip': PC_IP,
                    'pc_port': PORT,
                    'ranges': self.latest_ranges
                })
                
        # Flask sunucusunu tüm arayüzlerde çalıştır
        self.app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

    def gen_rgb(self):
        while True:
            with self.data_lock:
                jpeg_bytes = self.latest_rgb_jpeg
            if jpeg_bytes is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg_bytes + b'\r\n')
            time.sleep(0.05) # ~20 FPS limit
            
    def gen_depth(self):
        while True:
            with self.data_lock:
                jpeg_bytes = self.latest_depth_jpeg
            if jpeg_bytes is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg_bytes + b'\r\n')
            time.sleep(0.05)

    def on_ros_ready(self):
        print("ROS Bridge bağlantısı kuruldu! Dinleme başlatılıyor...")
        self.is_connected = True
        self.image_sub.subscribe(self.image_callback)

    def image_callback(self, msg):
        # Sadece en son gelen görüntüyü kaydet ve işleme thread'ini uyandır
        with self.data_lock:
            self.latest_msg = msg
        self.msg_event.set()

    def process_loop(self):
        while self.processing_running:
            # Yeni bir görüntü gelene kadar bekle
            self.msg_event.wait(timeout=0.1)
            if not self.processing_running:
                break
            if not self.msg_event.is_set():
                continue
            
            # Son gelen mesajı al ve event'i temizle
            with self.data_lock:
                msg = self.latest_msg
                self.latest_msg = None
                self.msg_event.clear()

            if msg is None:
                continue

            try:
                # 1. Görüntüyü ROS formatından (CompressedImage) OpenCV formatına dönüştür
                # Base64 verisini decode et
                img_data = base64.b64decode(msg['data'])
                np_arr = np.frombuffer(img_data, dtype=np.uint8)
                
                # cv2.imdecode returns BGR image
                img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if img_bgr is None:
                    continue
                
                # Convert to RGB for NPU processing
                img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

                # Web yayını için gelen RGB görüntüyü JPEG kodla (Zaten elimizde JPEG var)
                rgb_bytes = img_data

                # 2. NPU Girişine Uygun Şekillendir
                resized = cv2.resize(img, (self.model_w, self.model_h))
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
                color_depth_bgr = self.publish_depth_image(depth_map, msg['header'])

                # Web yayını için Jet renkli derinlik haritasını JPEG kodla
                if color_depth_bgr is not None:
                    _, jpeg_depth = cv2.imencode('.jpg', color_depth_bgr)
                    depth_bytes = jpeg_depth.tobytes()
                else:
                    depth_bytes = None

                # 5. FPS Göster ve Web Sunucusu İçin Verileri Güncelle
                self.frame_count += 1
                now = time.time()
                fps = self.npu_fps
                if now - self.last_time >= 1.0:
                    fps = self.frame_count / (now - self.last_time)
                    print(f"NPU Derinlik & ROS2 Aktarımı: {fps:.1f} FPS")
                    self.frame_count = 0
                    self.last_time = now

                with self.data_lock:
                    self.latest_rgb_jpeg = rgb_bytes
                    if depth_bytes is not None:
                        self.latest_depth_jpeg = depth_bytes
                    self.npu_fps = fps

            except Exception as e:
                print(f"Görüntü işleme hatası: {e}")

    def publish_scan(self, depth_map, original_header):
        # SCDepthV3 log-depth veya normalize edilmemiş ters derinlik çıktısı üretir.
        d_min = depth_map.min()
        d_max = depth_map.max()
        
        # Sıfıra bölme hatasını engelle
        if d_max == d_min:
            d_max += 0.001

        # Görüntünün ortasındaki birkaç satırı alıp dikey ortalamasını kullanalım
        mid_row_start = self.model_h // 2 - 5
        mid_row_end = self.model_h // 2 + 5
        horizontal_profile = np.mean(depth_map[mid_row_start:mid_row_end, :], axis=0)

        # Eşleme işlemi: Büyük değerler yakını, küçük değerler uzağı temsil eder
        norm_profile = (horizontal_profile - d_min) / (d_max - d_min)
        ranges = MIN_RANGE + (1.0 - norm_profile) * (MAX_RANGE - MIN_RANGE)
        
        # ROS LaserScan kuralına göre: Açı dizisi soldan sağa (artıdan eksiye) olmalı.
        ranges_list = ranges.tolist()

        # Web sunucusu için son tarama verisini güncelle
        with self.data_lock:
            self.latest_ranges = ranges_list

        # LaserScan Mesajı Oluştur
        scan_msg = {
            'header': {
                'stamp': original_header['stamp'], # Görüntünün çekildiği zaman damgasını koru
                'frame_id': 'camera_link'          # Robot modelindeki kamera linki
            },
            'angle_min': -FOV / 2.0,
            'angle_max': FOV / 2.0,
            'angle_increment': FOV / float(self.model_w),
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

            # Web yayını için BGR formatındaki renkli derinlik görselini döndür
            return color_depth
        except Exception as e:
            print(f"Derinlik görüntüsü yayınlama hatası: {e}")
            return None

    def run(self):
        try:
            self.client.run_forever()
        except KeyboardInterrupt:
            print("Kapatılıyor...")
        finally:
            # İşleyici thread'i durdur
            self.processing_running = False
            self.msg_event.set()
            try:
                self.process_thread.join(timeout=1.0)
            except:
                pass
            
            # Temizlik (Hata fırlatmadan temiz kapanış yapabilmesi için try-except blokları eklendi)
            try:
                self.infer_pipeline.__exit__(None, None, None)
            except:
                pass
            try:
                self.activation.__exit__(None, None, None)
            except:
                pass
            try:
                self.vdevice.release()
            except:
                pass
            try:
                self.client.terminate()
            except Exception as e:
                pass

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Raspberry Pi 5 Hailo-8L Obstacle Detector")
    parser.add_argument("ip", nargs="?", default="127.0.0.1", help="PC ROS Bridge IP address")
    parser.add_argument("--hef", "-m", "--model", default="scdepth",
                        help="Model key ('scdepth' or 'fastdepth') or direct HEF file path")
    args = parser.parse_args()
    
    PC_IP = args.ip
    model_choice = args.hef.lower()
    
    # Eşleme haritasından HEF yolunu al veya doğrudan girilen yolu kullan
    if model_choice in MODELS_MAP:
        hef_path = MODELS_MAP[model_choice]
    else:
        hef_path = args.hef
        # Eğer girilen dosya mevcut değilse ve uzantısız girildiyse varsayılan dizinde ara
        if not os.path.exists(hef_path):
            default_dir = "/usr/local/hailo/resources/models/hailo8l"
            potential_path = os.path.join(default_dir, hef_path)
            if not potential_path.endswith(".hef"):
                potential_path += ".hef"
            if os.path.exists(potential_path):
                hef_path = potential_path
            
    print(f"ROS Bridge sunucusuna bağlanılıyor: ws://{PC_IP}:{PORT}")
    print(f"Seçilen Model: {model_choice} -> {hef_path}")
    detector = RpiObstacleDetector(hef_path=hef_path)
    detector.run()

