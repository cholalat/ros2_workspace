import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
import cv2
import numpy as np
import time





class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('seguir_pasillo')


        self.vector_de_obstaculos = [0,0,0]

        self.publisher = self.create_publisher(Vector3, "/distancia_paredes", 10)

        self.subscription = self.create_subscription(
            Image, 'camera/depth/image_raw', self.procesar_imagen, 1)
        

        self.bridge = CvBridge()


    def procesar_imagen(self, msg):

        imagen = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        height, width = imagen.shape

        tercio_inicio = height // 3
        tercio_fin = 2 * height // 3

        #Dividomos la pantalla en 3 zonas
        area_izquierda = imagen[tercio_inicio:tercio_fin, :width//5]
        area_centro = imagen[tercio_inicio:tercio_fin, width//5:4*width//5]
        area_derecha = imagen[tercio_inicio:tercio_fin, 4*width//5:]

        #Quitamos los Nan
        area_izquierda = np.nan_to_num(area_izquierda, nan=0.0)
        area_centro = np.nan_to_num(area_centro, nan=0.0)
        area_derecha = np.nan_to_num(area_derecha, nan=0.0)


        # Selecciona solo el tercio central vertical de cada extremo


        dis_izquierda = np.average(area_izquierda)
        dis_centro = np.average(area_centro)
        dis_derecha = np.average(area_derecha)



        print("promedio:", dis_izquierda,dis_centro, dis_derecha)


        #Vemos la distancia promedio de cada zona y la enviamos al controlador
        self.vector_de_obstaculos = [
            dis_izquierda,
            dis_centro,
            dis_derecha
        ]


        #Enviamos la informacion por el la cosa por donde se envian las cosas (topico)
        mi_vector = Vector3()

        mi_vector.x = float(self.vector_de_obstaculos[0])
        mi_vector.y = float(self.vector_de_obstaculos[1])
        mi_vector.z = float(self.vector_de_obstaculos[2])

        self.publisher.publish(mi_vector)





        """El script de aqui abajo es solamente con fines de visualizacion, para poder verificar que los resultados 
        obtenidos en el vector son razonables, no es parte de lo pedido en la tarea.
        Este ha sido obtenido al preguntarle a la IA deepseek como interpretar las imagenes pedidas para poder visualizarlas."""
        try:
            # 1. Convertir ROS Image a array NumPy (conservando el formato 32FC1)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 2. Normalizar los valores para visualización
            depth_image = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)
            
            # 3. Escalar a 8 bits (0-255)
            depth_normalized = cv2.normalize(
                depth_image,
                None,
                alpha=0,
                beta=255,
                norm_type=cv2.NORM_MINMAX,
                dtype=cv2.CV_8UC1
            )

            # 4. Ecualizar histograma para mejorar contraste
            depth_eq = cv2.equalizeHist(depth_normalized)

            # 5. Aplicar colormap perceptual (Turbo si está disponible, si no JET)
            if hasattr(cv2, 'COLORMAP_TURBO'):
                depth_colormap = cv2.applyColorMap(depth_eq, cv2.COLORMAP_TURBO)
            else:
                depth_colormap = cv2.applyColorMap(depth_eq, cv2.COLORMAP_JET)

            # 6. Mostrar la imagen
            cv2.imshow("Profundidad (Contraste Mejorado)", depth_colormap)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error al procesar profundidad: {str(e)}")








def main(args=None):
    print("Corriendo una media maraton")
    rclpy.init(args=args)
    node = ObstacleDetector()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
