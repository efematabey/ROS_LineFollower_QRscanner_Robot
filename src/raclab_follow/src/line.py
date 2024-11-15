#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np
from pyzbar import pyzbar
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class FollowTrack:
    
    def __init__(self):
        # CvBridge sınıfı ile ROS görüntülerini OpenCV görüntülerine dönüştür
        self.bridge = cv_bridge.CvBridge()
        
        # Publisher ve Subscriber
        self.sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Twist mesajı nesnesi
        self.twist_obj = Twist()
        self.following_enabled = False  # Çizgi takip özelliği başlangıçta kapalı

    def image_callback(self, msg):
        # ROS mesajını OpenCV görüntüsüne çevirme
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # QR kodları kontrol et
        self.check_for_qr_code(image)
        
        # Çizgi takibi etkinse çizgi takip işlemini yap
        if self.following_enabled:
            self.follow_line(image)
        
        # Görüntüyü küçük bir boyutta göstermek için yeniden boyutlandır
        small_image = cv2.resize(image, (400, 400))  # 300x300 piksel boyutunda göster
        cv2.imshow("raclab_bot_camera", small_image)
        cv2.waitKey(4)

    def check_for_qr_code(self, image):
        # QR kodları tespit et
        decoded_objects = pyzbar.decode(image)
        for obj in decoded_objects:
            qr_data = obj.data.decode("utf-8")
            if qr_data == "efemQR":
                # QR kod tespit edildiğinde çizgi takip durumunu değiştir
                self.following_enabled = not self.following_enabled
                if self.following_enabled:
                    print("QR kod okundu, çizgi takip ve haritalandırma başladı.")
                else:
                    print("QR kod okundu, çizgi takip ve haritalandırma bitti.")
                    self.stop_robot()  # Robotu durdur
                break

    def follow_line(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Siyah rengin HSV aralığı
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])

        # Siyah çizgiyi tespit etmek için maskeyi oluştur
        mask = cv2.inRange(hsv, lower_black, upper_black)

        # Görüntü boyutlarını al
        height, w, _ = image.shape
        top = int(3 * height / 4)
        mask[0:top, 0:w] = 0  # Üst kısmı maskeleyerek sadece alt kısmı kullan

        # Maskedeki merkez hesaplaması
        M = cv2.moments(mask)
        
        if M['m00'] > 0:
            # Maskedeki blob'un merkezini bul
            cx = int(M['m10'] / M['m00'])
            error = cx - w / 2  # Hedefin ortasına olan hata değeri

            # Hata değeri belirli bir aralıkta ise ileri git
            if -500 <= error <= 500:
                self.twist_obj.linear.x = 0.26
                self.twist_obj.angular.z = 0
                print("İleri git!")
            else:
                # Hata sağa veya sola kayarsa döndür
                self.twist_obj.linear.x = 0.05
                self.twist_obj.angular.z = -float(error) / 5000
                if error < 0:
                    print("Sola dön!")
                else:
                    print("Sağa dön!")
                
            self.pub.publish(self.twist_obj)
        else:
            self.stop_robot()  # Çizgi yoksa robotu durdur

    def stop_robot(self):
        # Robotu durdurmak için linear ve angular hızları sıfırla
        self.twist_obj.linear.x = 0
        self.twist_obj.angular.z = 0
        self.pub.publish(self.twist_obj)

if __name__ == '__main__':
    try:
        rospy.init_node('line')
        follower_obj = FollowTrack()
        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass

