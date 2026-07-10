#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import time
import math
from collections import deque
from rclpy.qos import qos_profile_sensor_data


class FollowTheGapNode(Node):
    def __init__(self):
        super().__init__('follow_the_gap_node')

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # 1. PARÁMETROS COMUNES (Compartidos por todos los autos)

        # Filtros base
        self.lidar_filter_window = 3
        self.raw_angle_ema       = 0.0


        self.car_width           = 0.33  # Ancho real del auto (0.27) + tolerancia

        # Control PD
        self.Kp                  = 0.42
        self.Kd                  = 0.69
        self.prev_error          = 0.0

        # Escala de velocidad desde el launch file (Identificador de rol)
        self.declare_parameter('speed_scale', 1.0)
        self.speed_scale = self.get_parameter('speed_scale').value

        # Limitador de latigazos mecánicos (Slew-Rate)
        self.rate_open           = 0.22
        self.rate_tight          = 0.67
        self.angle_trigger_min   = 0.07
        self.angle_trigger_max   = 0.20
        self.steering_deadband   = 0.010
        self.prev_steering_angle = 0.0
        self.max_steer           = 0.4189
        self.instability_ema     = 0.0

        # Parámetros de Control de Velocidad (Acelerador)
        self.speed_base          = 5.2   # Velocidad constante base 
        self.speed_dist_gain     = 0.10  # Aceleración extra por distancia libre al frente  
        self.speed_steer_penalty = 22.0  # Freno aplicado (resta) al girar el volante 
        self.speed_min           = 4.5   # Tope mínimo de velocidad (curvas cerradas) 
        self.speed_max           = 9.5   # Tope máximo de velocidad (rectas largas) 
        self.current_speed       = 0.0   # Velocidad actual (para ventana de visión dinámica), viene de odom 

        # Ventana de visión dinámica (Ceguera periférica en curvas)
        self.fov_narrow = (0.30, 0.70)  
        self.fov_wide   = (0.22, 0.78)  
        self.speed_low  = 4.5
        self.speed_high = 9.0

        # Variables de memoria e histeresis (inicialización nula)
        self.prev_gap_start      = None
        self.prev_gap_end        = None
        self.angle_history       = deque(maxlen=8)
        self._last_valid_ranges  = None
        self.bubble_min_dist     = 0.37  

        # 2. PARÁMETROS ESPECÍFICOS SEGÚN EL ROL (Multi-Agente)
        
        if self.speed_scale < 1.0:
            # OPONENTES
            self.hard_min_clearance = 0.50  # Margen contra muros internos
            self.max_weight_depth   = 0.05  
            self.gap_switch_margin  = 1.50
            self.ema_alpha          = 0.45  
        else:
            # AUTO PRINCIPAL
            self.hard_min_clearance = 0.22   # Reducido para que no entre en pánico al pasar rozando al oponente 
            self.max_weight_depth   = 0.45   # Busca la salida (el ápice)
            self.gap_switch_margin  = 1.15   # Cambia de idea rápido para rebasar
            self.ema_alpha          = 0.95  


        # 3. TELEMETRÍA
        self.start_x           = None
        self.start_y           = None
        self.lap_start_time    = None
        self.lap_count         = 0
        self.distance_traveled = 0.0
        self.last_x            = None
        self.last_y            = None
        self.left_start_zone   = False

        self._odom_check_timer = self.create_timer(3.0, self._check_odom_connection)
        self.get_logger().info("Piloto FTG Multi-Agente iniciado.")


    # FUNCIONES DE UTILIDAD Y CALLBACKS

    def _check_odom_connection(self):
        """Verifica el estado de conexión del tópico de odometría inicial y detiene el temporizador."""
        if self.get_name() in ['ftg_ego', 'follow_the_gap_node']:
            if self.start_x is None:
                self.get_logger().error("Sin odometría en '/ego_racecar/odom' tras 3s.")
            else:
                self.get_logger().info("Odometría OK en '/ego_racecar/odom'.")
        self._odom_check_timer.cancel()

    def odom_callback(self, msg):
        """Procesa la velocidad actual del vehículo y gestiona la telemetría de vueltas y tiempos."""
        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y
        self.current_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

        if self.start_x is None:
            self.start_x = cx;  self.start_y = cy
            self.last_x  = cx;  self.last_y  = cy
            self.lap_start_time = time.time()
            return
            
        self.distance_traveled += math.hypot(cx - self.last_x, cy - self.last_y)
        self.last_x = cx;  self.last_y = cy
        
        if self.distance_traveled > 15.0: 
            self.left_start_zone = True
        
        if self.left_start_zone and math.hypot(cx - self.start_x, cy - self.start_y) < 2.0:
            lap_time = time.time() - self.lap_start_time
            self.lap_count += 1
            if self.get_name() in ['ftg_ego', 'follow_the_gap_node']:
                self.get_logger().info(f"🏁 [VUELTA {self.lap_count}] Tiempo: {lap_time:.2f} s")
            self.lap_start_time = time.time(); self.distance_traveled = 0.0; self.left_start_zone = False

    def preprocess_lidar(self, ranges):
        """Sanea los datos brutos del LiDAR, rellenando valores nulos o infinitos con la última lectura válida."""
        r = np.array(ranges, dtype=np.float32)
        invalid = np.isinf(r) | np.isnan(r)
        if self._last_valid_ranges is None:
            r[invalid] = 0.0
        else:
            r[invalid] = self._last_valid_ranges[invalid]
        self._last_valid_ranges = r.copy()
        return r

    def smooth_ranges(self, ranges):
        """Aplica un filtro de convolución (media móvil) para reducir el ruido en las lecturas láser."""
        k = np.ones(self.lidar_filter_window) / self.lidar_filter_window
        return np.convolve(ranges, k, mode='same')

    def get_fov_indices(self, msg, n):
        """Calcula la ventana de vision: la mas estrecha entre el limite duro de 180 grados
        frontales y la ventana dinamica segun velocidad."""
        max_half_angle = math.radians(90)
        hard_lo = max(0, int((-max_half_angle - msg.angle_min) / msg.angle_increment))
        hard_hi = min(n, int((max_half_angle - msg.angle_min) / msg.angle_increment))

        t = float(np.clip((self.current_speed - self.speed_low) / (self.speed_high - self.speed_low), 0.0, 1.0))
        lo_frac = self.fov_wide[0] + t * (self.fov_narrow[0] - self.fov_wide[0])
        hi_frac = self.fov_wide[1] + t * (self.fov_narrow[1] - self.fov_wide[1])
        dyn_lo = int(n * lo_frac)
        dyn_hi = int(n * hi_frac)

        start_idx = max(hard_lo, dyn_lo)
        end_idx   = min(hard_hi, dyn_hi)
        return start_idx, end_idx

    def select_gap(self, front_ranges, msg, straight_idx):
        """Localiza gaps con ancho fisico suficiente para pasar, y entre esos, prefiere
        el que requiera MENOR cambio de rumbo respecto al frente del auto."""
        nz = np.where(front_ranges > 0.0)[0]
        if len(nz) == 0:
            return None

        gaps = np.split(nz, np.where(np.diff(nz) != 1)[0] + 1)

        min_required_width = self.car_width + 0.08
        viable = []
        for g in gaps:
            avg_dist = float(np.mean(front_ranges[g]))
            width_m = avg_dist * msg.angle_increment * len(g)
            if width_m >= min_required_width:
                viable.append(g)
        if not viable:
            viable = gaps

        def offset(g):
            return abs((g[0] + g[-1]) / 2.0 - straight_idx)

        best_gap = min(viable, key=offset)

        if self.prev_gap_start is not None:
            prev_candidates = [g for g in viable if g[0] <= self.prev_gap_end and g[-1] >= self.prev_gap_start]
            if prev_candidates:
                prev_match = max(prev_candidates, key=len)
                if offset(best_gap) * self.gap_switch_margin >= offset(prev_match):
                    best_gap = prev_match

        self.prev_gap_start = int(best_gap[0])
        self.prev_gap_end   = int(best_gap[-1])
        return best_gap


    # CICLO PRINCIPAL DE CONTROL

    def scan_callback(self, msg):
        """Ejecuta secuencialmente las fases del algoritmo Follow The Gap con los datos LiDAR recientes."""
        ranges = self.preprocess_lidar(msg.ranges)
        ranges = self.smooth_ranges(ranges)
        n = len(ranges)

        # PASO 1: Burbuja Global con ancho adaptativo segun proximidad
        closest_idx_global  = int(np.argmin(ranges))
        closest_dist_global = float(ranges[closest_idx_global])

        # Si el objeto mas cercano esta a una distancia tipica de encuentro con otro auto,
        # se asume margen combinado (dos cuerpos). Lejos, se mantiene el margen ajustado
        # que permite buenas trazadas de apex en curva.
        proximity_threshold = 3.5  # metros
        if closest_dist_global < proximity_threshold:
            effective_width = self.car_width + 0.12  # margen extra estimado para un segundo cuerpo
        else:
            effective_width = self.car_width

        bubble_angle      = math.atan2(effective_width / 2.0, max(closest_dist_global, self.bubble_min_dist))
        rays_to_eliminate = int(bubble_angle / msg.angle_increment)
        b_start = max(0, closest_idx_global - rays_to_eliminate)
        b_end   = min(n, closest_idx_global + rays_to_eliminate)
        ranges[b_start:b_end] = 0.0

        ranges[ranges < self.hard_min_clearance] = 0.0

        # PASO 2: Visión dinámica y búsqueda de gaps con histeresis
        start_idx, end_idx = self.get_fov_indices(msg, n)
        front_ranges = ranges[start_idx:end_idx].copy()

        # Indice que representa "recto al frente" (angulo 0) dentro de front_ranges
        straight_idx = int(np.clip((n // 2) - start_idx, 0, len(front_ranges) - 1))

        largest_gap = self.select_gap(front_ranges, msg, straight_idx)
        if largest_gap is None:
            out = AckermannDriveStamped()
            out.drive.speed = 3.0; out.drive.steering_angle = 0.0
            self.drive_pub.publish(out)
            return

        # PASO 3: Trazada, usando estabilidad de la escena en vez del angulo filtrado propio
        gap_ranges = np.clip(front_ranges[largest_gap], 0, 6.0)
        max_val = float(np.max(gap_ranges))
        deep_indices = np.where(gap_ranges >= (max_val * 0.85))[0]
        stable_max_depth_idx = int(np.mean(deep_indices))
        center_of_gap = len(largest_gap) // 2

        if len(self.angle_history) >= 2:
            instability = float(np.std(self.angle_history))
        else:
            instability = 0.0
            
        self.instability_ema = 0.85 * self.instability_ema + 0.15 * instability
        t_traj = float(np.clip((self.instability_ema - self.angle_trigger_min) /
                               (self.angle_trigger_max - self.angle_trigger_min), 0.0, 1.0))

        weight_depth = float(np.clip(self.max_weight_depth - (t_traj * self.max_weight_depth), 0.0, 1.0))
        weight_center = 1.0 - weight_depth

        raw_idx = int(np.clip(weight_depth * stable_max_depth_idx + weight_center * center_of_gap, 0, len(largest_gap) - 1))
        best_idx = largest_gap[raw_idx]

        target_angle = float(msg.angle_min + (start_idx + best_idx) * msg.angle_increment)
        target_distance = float(front_ranges[best_idx])
        self.angle_history.append(target_angle)

        # PASO 4: Filtro EMA sobre el angulo objetivo
        self.raw_angle_ema = (self.ema_alpha * target_angle + (1.0 - self.ema_alpha) * self.raw_angle_ema)
        target_filtered = float(np.clip(self.raw_angle_ema, -self.max_steer, self.max_steer))

        # PASO 5: PD sobre el error respecto al angulo ya aplicado, con slew-rate real
        error = target_filtered - self.prev_steering_angle
        derivative = error - self.prev_error
        pd_command = self.prev_steering_angle + self.Kp * error + self.Kd * derivative
        pd_command = float(np.clip(pd_command, -self.max_steer, self.max_steer))
        self.prev_error = error

        if abs(error) >= self.angle_trigger_max:
            max_delta = self.rate_tight
        elif abs(error) <= self.angle_trigger_min:
            max_delta = self.rate_open
        else:
            span = self.angle_trigger_max - self.angle_trigger_min
            frac = (abs(error) - self.angle_trigger_min) / span
            max_delta = self.rate_open + frac * (self.rate_tight - self.rate_open)

        delta = float(np.clip(pd_command - self.prev_steering_angle, -max_delta, max_delta))
        if abs(delta) < self.steering_deadband:
            delta = 0.0
        smoothed = self.prev_steering_angle + delta
        self.prev_steering_angle = smoothed

        # PASO 6: Acelerador dinámico paramétrico
        abs_steer = abs(smoothed)
        
        # Fórmula extraída utilizando las nuevas variables del __init__
        raw_speed = self.speed_base + (target_distance * self.speed_dist_gain) - (self.speed_steer_penalty * abs_steer)
        speed = float(np.clip(raw_speed, self.speed_min, self.speed_max))

        out = AckermannDriveStamped()
        
        # Aplicamos el multiplicador a la velocidad final:
        out.drive.speed = speed * self.speed_scale
        out.drive.steering_angle = smoothed
        self.drive_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()