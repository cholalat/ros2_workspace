from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
import numpy as np
import sensor_msgs.msg as sensor_msgs
from scipy import spatial
import cv2
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter

def procesar_imagen():
    # Cargar el mapa como imagen en escala de grises
    map_img = cv2.imread('/home/paulo/zuniversidad/5to_semestre/robotica_movil/ros2_ws/src/lab_3_rm/imagenes/mapa.pgm', cv2.IMREAD_GRAYSCALE)

    ocupados = np.where(map_img == 0)
    lista_puntos_ocupados = list(zip(ocupados[0], ocupados[1]))

    resolution = 0.01
    origin = [0.0, 0.0]
    height = map_img.shape[0]

    # Convertir la lista de puntos ocupados a coordenadas en el mapa (origen abajo-izquierda)
    coordenadas_ocupadas = [
        (
            origin[0] + col * resolution,
            origin[1] + (height - 1 - row) * resolution
        )
        for row, col in lista_puntos_ocupados
    ]



    # # Graficar las coordenadas sobre la imagen
    # plt.figure(figsize=(8, 8))
    # xs = [x for x, y in coordenadas_ocupadas]
    # ys = [y for x, y in coordenadas_ocupadas]
    # px = [int((x - origin[0]) / resolution) for x in xs]
    # py = [int((y - origin[1]) / resolution) for y in ys]
    # plt.scatter(px, py, s=1, c='red')
    # plt.title("Puntos ocupados sobre el mapa")
    # plt.show()

    return coordenadas_ocupadas









def likelihood_field_range_finder_model(z_t, theta_z_t, x_t, theta_robot, tree, step=2):
    # step: salta rayos para acelerar (por ejemplo, step=5 toma 1 de cada 5 rayos)
    z_max = 4
    sigma = 0.05
    q = 1.0

    for k in range(0, len(z_t), step):
        if z_t[k] < z_max:
            x_ztk = x_t[0] + z_t[k] * np.cos(theta_robot + theta_z_t[k])
            y_ztk = x_t[1] + z_t[k] * np.sin(theta_robot + theta_z_t[k])
            dist, _ = tree.query((x_ztk, y_ztk))
            p_hit = np.exp(-0.5 * (dist**2) / (sigma**2)) / np.sqrt(2 * np.pi * sigma**2)
            
            
            
            q *= p_hit


    return q




class NodoIntento(Node):
    def __init__(self):
        super().__init__('nodo_intento')
        self.subscription = self.create_subscription(
            sensor_msgs.LaserScan,
            '/lidar_filtrado',
            self.listener_callback,
            1
        )
        self.i = 0
        self.bridge = CvBridge()
        self.get_logger().info("NodoIntento iniciado")

        coordenadas_ocupadas = procesar_imagen()
        self.tree = spatial.KDTree(coordenadas_ocupadas)

        # --- Cargar imagen de fondo y preparar figura ---
        self.mapa_img = cv2.imread(
            '/home/paulo/zuniversidad/5to_semestre/robotica_movil/ros2_ws/src/lab_3_rm/imagenes/mapa.pgm',
            cv2.IMREAD_GRAYSCALE
        )
        self.resolution = 0.01  # Debe coincidir con procesar_imagen
        self.height, self.width = self.mapa_img.shape

        # Invertir eje Y para que el origen sea abajo-izquierda
        self.mapa_img = np.flipud(self.mapa_img)

        # --- OPTIMIZACIÓN: Usa un grid más pequeño si puedes ---
        self.grid_size = 30  # Antes 50, ahora 30 para acelerar
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.imshow(
            self.mapa_img,
            cmap='gray',
            origin='lower',
            extent=[0, self.width * self.resolution, 0, self.height * self.resolution],
            alpha=0.7
        )
        self.im = self.ax.imshow(
            np.zeros((self.grid_size, self.grid_size)),
            origin='lower',
            cmap='hot',
            extent=[0, self.width * self.resolution, 0, self.height * self.resolution],
            alpha=0.6
        )
        self.fig.colorbar(self.im, ax=self.ax, label='Verosimilitud')
        self.ax.set_title('Mapa de calor de verosimilitud')
        self.ax.set_xlabel('x (m)')
        self.ax.set_ylabel('y (m)')
        plt.ion()
        plt.show()

    def listener_callback(self, msg):
        print("Recibiendo mensaje en NodoIntento")
        # OPTIMIZACIÓN: Precalcula los puntos del grid solo una vez por callback
        xs = np.linspace(0, self.width * self.resolution, self.grid_size)
        ys = np.linspace(0, self.height * self.resolution, self.grid_size)
        puntos = [(x, y) for x in xs for y in ys]

        theta_z_t = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment

        # OPTIMIZACIÓN: Inicializa el array de verosimilitud
        verosimilitudes = np.zeros((self.grid_size, self.grid_size))

        # OPTIMIZACIÓN: Usa menos rayos (step=5)
        for idx, punto in enumerate(puntos):
            i = idx // self.grid_size
            k = idx % self.grid_size
            verosimilitud = likelihood_field_range_finder_model(
                msg.ranges, theta_z_t, punto, 0, self.tree, step=5
            )
            verosimilitudes[k, i] = verosimilitud  # Reflexión respecto a x=y
        # Actualizar el mapa de calor
        self.im.set_data(verosimilitudes)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        print("Mensaje recibido en NodoIntento ", self.i)
        self.i += 1



















def main(args=None):
    rclpy.init(args=args)
    nodo_intento = NodoIntento()
    rclpy.spin(nodo_intento)
    nodo_intento.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()