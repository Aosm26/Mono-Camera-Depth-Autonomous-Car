#!/bin/bash
set -e

echo "============================================="
echo "  Ubuntu 20.04 PC - ROS2 Foxy Kurulum Scripti"
echo "============================================="

# 1. UTF-8 Locale Ayarları
echo "[1/5] Locale ayarlanıyor..."
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. ROS2 Foxy Depoları Ekleme
echo "[2/5] ROS2 Foxy depoları ekleniyor..."
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Paket Yüklemeleri
echo "[3/5] ROS2 Desktop, Gazebo ve Nav2 yükleniyor..."
sudo apt update
sudo apt install -y \
  ros-foxy-desktop \
  ros-foxy-navigation2 \
  ros-foxy-nav2-bringup \
  ros-foxy-gazebo-ros-pkgs \
  ros-foxy-depthimage-to-laserscan \
  ros-foxy-xacro \
  python3-colcon-common-extensions

# 4. ROS2 Çevre Değişkenleri
echo "[4/5] ~/.bashrc dosyasına ROS2 çevre değişkenleri ekleniyor..."
if ! grep -q "source /opt/ros/foxy/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
fi

# 5. Workspace Derleme Talimatı
echo "[5/5] Tamamlandı!"
echo ""
echo "Şimdi yapmanız gerekenler:"
echo "1. RPi5'teki 'autonomous_car_ws' klasörünü bilgisayarınıza kopyalayın (örn: ~/autonomous_car_ws)."
echo "2. Bilgisayarınızda terminal açıp şu komutları sırasıyla çalıştırın:"
echo "   cd ~/autonomous_car_ws"
echo "   source /opt/ros/foxy/setup.bash"
echo "   colcon build --symlink-install"
echo "   source install/setup.bash"
echo ""
echo "3. Simülasyonu başlatmak için (1. Terminal):"
echo "   ros2 launch autonomous_car sim_launch.py"
echo ""
echo "4. Otonom Navigasyonu başlatmak için (2. Terminal):"
echo "   ros2 launch autonomous_car nav_launch.py"
echo ""
echo "5. Hedef göndermek için RViz2'yi başlatıp '2D Goal Pose' butonunu kullanabilirsiniz."
