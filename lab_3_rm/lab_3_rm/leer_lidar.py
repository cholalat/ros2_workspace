from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
import sensor_msgs.msg as sensor_msgs
import rclpy
import matplotlib.pyplot as plt
import numpy as np




class leer_lidar(Node):
    def __init__(self):
        super().__init__('Lector_promedio')

        self.publisher = self.create_publisher(sensor_msgs.LaserScan, "/lidar_filtrado", 10)
        self.filtered_scan_pub = self.create_publisher(sensor_msgs.LaserScan, "/scan_filtrado", 10)
        self.subscription = self.create_subscription(
            sensor_msgs.LaserScan, '/scan', self.procesar_imagen, 1)

        # Inicializa la figura de matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-')
        self.ax.set_ylim(0, 5)  # Ajusta según el rango de tu LIDAR
        self.ax.set_xlim(-90, 90)  # Ahora el eje x es el ángulo en grados
        self.ax.set_xlabel('Ángulo (grados)')
        self.ax.set_ylabel('Distancia (m)')

    def procesar_imagen(self, msg):
        # Calcula los ángulos de cada medición
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        angles_deg = np.degrees(angles)
        ranges = np.array(msg.ranges)

        # Aplica las condiciones pedidas
        mask_outside = (angles_deg < -57) | (angles_deg > 57)
        ranges[mask_outside] = 4  # Si el ángulo está fuera de [-57, 57], distancia = 4
        ranges[ranges > 4] = 4    # Si la distancia es mayor a 4, distancia = 4

        # Crear un nuevo mensaje LaserScan con los datos filtrados
        filtered_msg = sensor_msgs.LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.min
        filtered_msg.range_max = msg.max
        filtered_msg.ranges = ranges.tolist()
        filtered_msg.intensities = msg.intensities

        # Publicar el mensaje filtrado
        self.publisher.publish(filtered_msg)

        # Plotea
        self.line.set_xdata(angles_deg)
        self.line.set_ydata(ranges)
        self.ax.set_xlim(-90, 90)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()





def main(args=None):
    rclpy.init(args=args)
    node = leer_lidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()