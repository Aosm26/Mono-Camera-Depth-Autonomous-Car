# SC-Depth Modeli İçin Veriseti Toplama ve Hazırlama Rehberi

Bu klasör, mevcut simülasyon ve ROS 2 dosyalarınızı bozmadan, SC-Depth (özellikle SC-Depth V3) modelinin eğitimi için gereken verisetini toplamak, organize etmek ve filtrelemek amacıyla kurulmuştur.

---

## 1. Kritik Sürüş ve Veri Toplama Kuralları

Self-supervised (öz-denetimli) monoküler derinlik modellerinin öğrenme mekanizması tamamen **kamera hareketiyle oluşan piksel yer değiştirmelerine (triangulation/parallaks)** dayanır. Bu yüzden veri toplarken aşağıdaki kurallara mutlaka uymalısınız:

1. **Öteleme (Translation) Hareketi Yapın:** Aracınızı çoğunlukla düz hatlarda ileriye doğru sürün. Model, nesnelere yaklaştıkça piksellerin nasıl büyüdüğünü görerek derinliği kavrar.
2. **Kendi Ekseni Etrafında Dönmekten (Pure Rotation) Kaçının:** Araç durduğu yerde hızla dönerse (tank dönüşü gibi), kamera görüntüsünde derinlik ipucu (parallax) oluşmaz. Monoküler derinlik tahmininin önündeki en büyük engel kamera rotasyonlarıdır. Dönüşleri olabildiğince **geniş kavislerle ve hareket halindeyken** (öteleme ile rotasyon bir aradayken) yapın.
3. **Hızı Dengeli Tutun:** Çok yavaş giderseniz ardışık kareler arasındaki hareket (optical flow) çok az olacağından model derinliği öğrenemez. Çok hızlı giderseniz de simülasyonda görüntüde kırılmalar/donmalar yaşanabilir. **Orta ve sabit bir hız (örn: 0.5 - 1.0 m/s)** idealdir.
4. **Farklı Açılardan Yaklaşın:** Parkurdaki engellere tek bir açıdan değil; sağından, solundan ve farklı mesafelerden yaklaşarak sürüş yapın. Böylece model engellerin geometrisini çok yönlü öğrenir.

---

## 2. Kurulum ve Çalıştırma Adımları

Veri toplama aracını çalıştırmadan önce terminalinizde ROS 2 ortamını kaynaklandırmanız (source etmeniz) gerekir.

### Adım A: Simülasyonu Başlatın
Mevcut simülasyonu başlatmak için ana çalışma alanınızda (workspace) şu komutu kullanın:
```bash
cd "/home/aosm/Mono Camera Depth Autonomuos Car"
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch autonomous_car sim_launch.py
```

### Adım B: Veri Toplayıcıyı (Dataset Collector) Çalıştırın
Simülasyon arka planda açıkken, ayrı bir terminalde veri toplayıcı düğümü başlatın. Bu düğüm `/camera/image_raw` ve `/camera/camera_info` topiclerini dinleyerek verileri doğrudan SC-Depth formatında kaydeder.

```bash
cd "/home/aosm/Mono Camera Depth Autonomuos Car/scdepth_dataset_prep"
source /opt/ros/foxy/setup.bash

# Varsayılan ayarlarla çalıştırma (Otomatik scene_000, scene_001 adlarıyla kaydeder)
/usr/bin/python3 dataset_collector.py

# Alternatif: Özel bir sahne adı belirterek çalıştırma
/usr/bin/python3 dataset_collector.py --scene_name scene_kullanici_tanimli

# Alternatif: Kare atlama (örn: her 2 karede 1 resim kaydetmek için)
/usr/bin/python3 dataset_collector.py --skip_frames 1
```

### Adım C: Robotu Kontrol Edin
Veri toplayıcı çalışırken, robotu yönlendirmek için teleop düğümünü çalıştırabilir ve yukarıdaki sürüş kurallarına uygun şekilde parkurda sürüş yapabilirsiniz:
```bash
source /opt/ros/foxy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
*Sürüş bittiğinde `dataset_collector.py` çalışan terminalde **Ctrl+C** tuşlarına basarak kaydı güvenle durdurabilirsiniz.*

---

## 3. Alternatif Yöntem: Rosbag ile Veri Toplama

Eğer verileri doğrudan kaydetmek yerine önce ROS bag kaydı alıp daha sonra dışa aktarmak isterseniz şu adımları izleyebilirsiniz:

1. **Bag Kaydı Başlatın:**
   ```bash
   ros2 bag record /camera/image_raw /camera/camera_info -o "/home/aosm/Mono Camera Depth Autonomuos Car/scdepth_dataset_prep/sim_kaydi_1"
   ```
2. **Kareleri Dışarı Aktarın (Replay ile Kolay Export):**
   Kaydettiğiniz bag dosyasını oynatıp, aynı anda `dataset_collector.py` düğümünü çalıştırarak verileri otomatik olarak dışa aktarabilirsiniz. Düğüm, oynatılan konulardan veriyi canlıymış gibi yakalayıp kaydeder:
   
   *Terminal 1 (Collector):*
   ```bash
   /usr/bin/python3 dataset_collector.py --scene_name scene_bag_export
   ```
   *Terminal 2 (Bag Playback):*
   ```bash
   ros2 bag play "/home/aosm/Mono Camera Depth Autonomuos Car/scdepth_dataset_prep/sim_kaydi_1"
   ```

---

## 4. Veri Setini Filtreleme (Optik Akış Filtresi)

Araç durduğunda veya çok keskin döndüğünde oluşan verimsiz kareleri ayıklamak ve SC-Depth eğitiminde kullanılabilecek kaliteli kare indekslerini oluşturmak için:

```bash
cd "/home/aosm/Mono Camera Depth Autonomuos Car/scdepth_dataset_prep"
/usr/bin/python3 generate_valid_frame_index.py --dataset_dir ./dataset --threshold 0.5
```

Bu komut:
1. `dataset/training/` altındaki her bir sahneyi tarar.
2. Ardışık kareler arasındaki hareket miktarını hesaplar.
3. Koşulu sağlayan karelerin indekslerini içeren bir `frame_index.txt` dosyası oluşturur.

### SC-Depth Eğitimi Sırasında Kullanımı
Eğitimi başlatırken ilgili eğitim betiğine şu parametreyi eklemeniz yeterlidir:
```bash
--use_frame_index
```
Model, bu parametre sayesinde statik ve bozuk kareleri atlayarak yalnızca `frame_index.txt` dosyasındaki kaliteli veriler üzerinden öğrenim gerçekleştirecektir.

---

## 5. Klasör Yapısı

İşlemler tamamlandığında oluşan klasör yapısı şu şekilde olacaktır:
```text
/home/aosm/Mono Camera Depth Autonomuos Car/scdepth_dataset_prep/
├── dataset_collector.py
├── generate_valid_frame_index.py
├── README.md
└── dataset/
    └── training/
        ├── scene_000/
        │   ├── 000000.jpg
        │   ├── 000001.jpg
        │   ├── ...
        │   ├── cam.txt
        │   └── frame_index.txt
        └── scene_001/
            ├── ...
```
