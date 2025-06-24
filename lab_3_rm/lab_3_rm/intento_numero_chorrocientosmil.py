from scipy import spatial
import numpy as np
import matplotlib.pyplot as plt
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy


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

    print("Cantidad de puntos ocupados:", len(lista_puntos_ocupados))
    print("Ejemplo de puntos ocupados:", lista_puntos_ocupados[3000], lista_puntos_ocupados[9000])


    # Graficar las coordenadas sobre la imagen
    plt.figure(figsize=(8, 8))
    xs = [x for x, y in coordenadas_ocupadas]
    ys = [y for x, y in coordenadas_ocupadas]
    px = [int((x - origin[0]) / resolution) for x in xs]
    py = [int((y - origin[1]) / resolution) for y in ys]
    plt.scatter(px, py, s=1, c='red')
    plt.title("Puntos ocupados sobre el mapa")
    plt.show()





def likelihood_field_range_finder_model(z_t, theta_z_t, x_t, theta_robot, obstaculos_mapa):

    z_max = 4
    sigma = 0.1

    q = 1
    tree = spatial.KDTree(obstaculos_mapa)
    
    for k in range (len(z_t)):
        if z_t[k] < z_max:
            x_ztk = x_t[0] + z_t * np.cos(theta_robot + theta_z_t)
            y_ztk = x_t[1] + z_t * np.sin(theta_robot + theta_z_t)

            dist, point_id = tree.query((x_ztk, y_ztk))

            p_hit = np.exp(-0.5 * (dist**2) / (sigma**2)) / np.sqrt(2 * np.pi * sigma**2)
            q = q * p_hit

    return q






procesar_imagen()