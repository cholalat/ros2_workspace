import numpy as np
from scipy.ndimage import distance_transform_edt
import cv2
import matplotlib.pyplot as plt

# Cargar el mapa PGM
map_img = cv2.imread('/home/paulo/zuniversidad/5to_semestre/robotica_movil/ros2_ws/src/lab_3_rm/imagenes/mapa.pgm', cv2.IMREAD_GRAYSCALE)


# Parámetros del mapa (del archivo YAML)
resolution = 0.01  # metros por píxel
origin = [0.0, 0.0]  # coordenada inferior izquierda en metros

# Identificar obstáculos (0=obstáculo, 255=libre)
obstacles = (map_img == 0)
free_space = (map_img == 255)


# Calcular la distancia euclidiana al obstáculo más cercano para cada celda
# (distance_transform_edt calcula la distancia en píxeles, luego convertimos a metros)
distance_map = distance_transform_edt(~obstacles) * resolution

# Visualización opcional
plt.imshow(distance_map, cmap='viridis')
plt.colorbar(label='Distancia al obstáculo más cercano (m)')
plt.title('Mapa de Distancias a Obstáculos')
plt.show() 


