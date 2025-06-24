import numpy as np
import cv2

map_img = cv2.imread('/home/paulo/zuniversidad/5to_semestre/robotica_movil/ros2_ws/src/lab_3_rm/imagenes/mapa.pgm', cv2.IMREAD_GRAYSCALE)
obstacle_coords = np.argwhere(map_img == 0)  # Filas y columnas de obstáculos
obstacle_coords = obstacle_coords * 0.01     # Convertir a metros (resolución: 0.01 m/píxel)



from scipy.spatial import KDTree
from scipy.stats import norm

sigma_hit = 0.1  # Parámetro ajustable
likelihood_field = np.zeros_like(map_img, dtype=np.float32)

# KDTree para búsqueda eficiente del obstáculo más cercano
tree = KDTree(obstacle_coords)

# Calcular verosimilitud para cada celda del mapa
for y in range(map_img.shape[0]):
    for x in range(map_img.shape[1]):
        coord = np.array([x * 0.01, y * 0.01])  # Convertir a metros
        dist, _ = tree.query(coord)             # Distancia al obstáculo más cercano
        likelihood_field[y, x] = norm.pdf(dist, loc=0, scale=sigma_hit)



import matplotlib.pyplot as plt

plt.imshow(likelihood_field, cmap='hot', interpolation='nearest')
plt.colorbar(label='Verosimilitud')
plt.scatter(obstacle_coords[:, 1]/0.01, obstacle_coords[:, 0]/0.01, c='blue', s=1, label='Obstáculos')
plt.title('Campo de Verosimilitud Precalculado')
plt.show()



def p_hit(x_robot, y_robot, theta_robot, z_tk, sensor_angle):
    x_end = x_robot + z_tk * np.cos(theta_robot + sensor_angle)
    y_end = y_robot + z_tk * np.sin(theta_robot + sensor_angle)
    x_px = int(x_end / 0.01)  # Convertir a píxeles
    y_px = int(y_end / 0.01)
    return likelihood_field[y_px, x_px] if (0 <= x_px < map_img.shape[1] and 0 <= y_px < map_img.shape[0]) else 0.0