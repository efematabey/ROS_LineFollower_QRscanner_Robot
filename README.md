# QR Kod Kontrollü Çizgi Takip Eden ve Harita Çıkaran Otonom Robot

Bu proje, [Ubuntu 20.04](https://releases.ubuntu.com/focal/) işletim sistemi ve [ROS Noetic](http://wiki.ros.org/noetic) üzerinde çalışan otonom bir robot simülasyonudur. Simülasyon ortamı olarak [Gazebo](https://gazebosim.org/home) kullanılır.

Otonom robot, önceden tanımlanmış bir QR kod algıladığında çizgi takibine başlar, bu sırada ortamın haritasını LIDAR sensörü kullanarak çıkarır. Robot çizgi üzerindeki engellerden kaçarak hattına geri döner. Aynı QR kodu tekrar algıladığında ise çizgi takibini durdurur ve oluşturduğu haritayı kayıt eder.

QR kod algılama ve çizgi takibi işlemleri kamera ile [OpenCV](https://opencv.org/) kütüphanesi kullanılarak yapılır. Aynı zamanda ortamın haritası çıkarılırken [RViz](http://wiki.ros.org/rviz) ortamında görselleştirme imkanı sunulur, ardından kayıt edilir.

Robot modeli olarak, kamera ve LIDAR sensörü bulundurduğundan [TurtleBot3 Waffle](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview) tercih edilmiştir.

### Gerekli adımlar

1. Workspace oluşturulur, derlenir ve ayarlanır:

    ```bash
    mkdir -p metinol_ws/src
    cd metinol_ws
    catkin_make
    echo "source metinol_ws/devel/setup.bash" >> .bashrc
    source .bashrc
    ```
2. Paket oluşturulur, gerekli ROS bağımlılıkları belirtilir:
    ```bash
    cd metinol_ws/src
    catkin_create_pkg raclab_follow roscpp std_msgs sensor_msgs geometry_msgs cv_bridge image_transport
    ```
3. Script oluşturulur ve çalıştırılma izni verilir:
    ```bash
    touch metinol_ws/src/raclab_follow/src/line.py
    chmod +x metinol_ws/src/raclab_follow/src/line.py
    ```
4. Çalışma alanına (ROS Workspace) proje klonlanır:
    ```bash
    cd metinol_ws/src
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    ```
5. Gazebo'da robot içeren dünya açılır:
    ```bash
    roslaunch turtlebot3_gazebo raclab_dunya.launch 
    ```
6. Launch dosyası ile bütün işlemler başlatılır:
    ```bash
    roslaunch raclab_follow line.launch 
    ```
    Yukarıdaki komut ile "raclab_follow" paketi altındaki Script'ler (python, cpp) ve RViz çalıştırılır.
7. Sadece Script çalıştırılır:
    ```bash
    rosrun raclab_follow line.py 
    ```
### Gazebo'ya model eklemek için ekstra kaynak

Otonom robot simülasyonu için Gazebo ortamında bir dünya oluşturulmuştur. Oluşturulan dünyaya QR kod modeli eklemek için aşağıdaki kaynak yararlıdır.

1. GitHub Repository:

    ```bash
    https://github.com/mikaelarguedas/gazebo_models
    ```
2. YouTube:
    ```bash
    https://youtu.be/A3fJwTL2O4g?si=xMR9dMEJOQn9cwpc
    ```
