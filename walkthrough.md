# Adımsal Çalıştırma Rehberi (Walkthrough)

Bu kılavuz, **Mono Camera Depth Autonomous Car** projesini bilgisayarınızda nasıl derleyeceğinizi, çalıştıracağınızı ve test edeceğinizi açıklar.

---

## 1. Hazırlık ve Derleme (Colcon Build)

Sisteminizde yüklü olan Anaconda/Miniconda ortamı, ROS2'nin ihtiyaç duyduğu Python kütüphanelerini (`catkin_pkg` vb.) ezebilir. Bu nedenle derleme işlemini temiz bir PATH çevre değişkeni ile başlatmanız gerekmektedir.

Aşağıdaki komutla Anaconda yollarını geçici olarak devre dışı bırakıp projenizi derleyin:

```bash
# Anaconda yollarını devre dışı bırakın, ROS2'yi kaynak olarak ekleyin ve derleyin
export PATH=$(echo $PATH | tr ':' '\n' | grep -v 'anaconda' | paste -sd:)
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

Derleme işlemi bittiğinde projenin kurulu olduğu dizinde `build`, `install` ve `log` klasörleri oluşacaktır.

---

## 2. Gazebo Simülasyonunu Başlatma (Terminal 1)

İlk terminalinizde simülasyon ortamını (dünya ve robot modelini) başlatın.

1. Yeni bir terminal açın.
2. Anaconda çakışmasını önlemek ve ROS2 kaynaklarını yüklemek için şu komutları sırasıyla girin:

```bash
cd ~/Mono\ Camera\ Depth\ Autonomuos\ Car # Veya projenin yüklü olduğu konum
export PATH=$(echo $PATH | tr ':' '\n' | grep -v 'anaconda' | paste -sd:)
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch autonomous_car sim_launch.py
```

*Bu komut Gazebo simülasyonunu başlatacak, robotu spawn edecek ve derinlik kamerası verisini `/scan` (Lazer taraması) formatına dönüştürecektir.*

---

## 3. Navigasyon ve Kontrol Sistemini Başlatma (Terminal 2)

Robotun otonom hareket edebilmesi için Nav2 navigasyon yığınını başlatın.

1. İkinci bir terminal açın.
2. Şu komutları girin:

```bash
cd ~/Mono\ Camera\ Depth\ Autonomuos\ Car
export PATH=$(echo $PATH | tr ':' '\n' | grep -v 'anaconda' | paste -sd:)
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch autonomous_car nav_launch.py
```

*Bu komut Nav2 planlayıcılarını, haritasız (mapless) odometri navigasyonunu ve görselleştirme için RViz2 ekranını başlatacaktır.*

---

## 4. Otonom Sürüş ve Hedef Gönderme (RViz2)

1. Navigasyon başlatıldığında açılan **RViz2** penceresini bulun.
2. Üst menü barında bulunan **"2D Goal Pose"** butonuna tıklayın.
3. Harita üzerinde robotun gitmesini istediğiniz herhangi bir noktaya tıklayıp basılı tutarak yön belirtin.
4. Robot, derinlik kamerasından gelen verileri kullanarak engellerden kaçarak belirlediğiniz hedefe otonom şekilde hareket edecektir.

---

## 5. Tüm Süreçleri Sonlandırma

Çalışmanız bittiğinde veya sistem kaynaklarını serbest bırakmak istediğinizde, arka planda çalışan tüm ROS2, Gazebo ve Nav2 süreçlerini tek seferde kapatmak için şu komutu çalıştırabilirsiniz:

```bash
./stop_all.sh
```
