#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from pyzbar import pyzbar
import cv2

def qr_code_scanner():
    cap = cv2.VideoCapture(0)
    rospy.init_node('qr_code_scanner', anonymous=True)
    pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Robotun ileri doğru hareket etmesi için Twist mesajı
    move_cmd = Twist()
    move_cmd.linear.x = 1.0  # İleri hız (m/s)
    move_cmd.angular.z = 0.0  # Dönüş hızı (rad/s)

    # Robotu durdurmak için Twist mesajı
    stop_cmd = Twist()
    stop_cmd.linear.x = 0.0
    stop_cmd.angular.z = 0.0

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        barcodes = pyzbar.decode(gray)

        for barcode in barcodes:
            barcode_data = barcode.data.decode("utf-8")

            # QR kodunu kontrol et
            if barcode_data == "efemQR":
                rospy.loginfo("Olumlu: QR kodu 'efemQR' ile eşleşti.")
            else:
                rospy.loginfo("Olumsuz: QR kodu 'efemQR' ile eşleşmedi.")

            # QR kodu okunduğunda robotu durdur
            pub_cmd_vel.publish(stop_cmd)
            rospy.sleep(1)  # 1 saniye duraklatma

            # Robotu tekrar hareket ettir
            pub_cmd_vel.publish(move_cmd)

        # Robotu ileri hareket ettir
        pub_cmd_vel.publish(move_cmd)

        cv2.imshow('QR Kod Tarayıcı', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        qr_code_scanner()
    except rospy.ROSInterruptException:
        pass

