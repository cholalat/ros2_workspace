from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy



class leer_mapa(Node):
    def __init__(self):
        super().__init__('El_mapa_de_dora')
        print("Cosa creada")


        # self.publisher = self.create_publisher(Vector3, "/distancia_paredes", 10)

        self.subscription = self.create_subscription(
            Image, '/map', self.procesar_imagen, 1)
        

    def procesar_imagen(self, msg):
        # Convertir el mensaje de imagen a un formato OpenCV
        map_img = cv2.imread('/home/paulo/zuniversidad/5to_semestre/robotica_movil/ros2_ws/src/lab_3_rm/imagenes/mapa.pgm', cv2.IMREAD_GRAYSCALE)
        print(map_img)

        #Quiero plotear el mapa
        cv2.imshow("Mapa", map_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


        self.bridge = CvBridge()








def main(args=None):
    print("corriendo una maraton")
    rclpy.init(args=args)

    node = leer_mapa()
    node.procesar_imagen(None)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()