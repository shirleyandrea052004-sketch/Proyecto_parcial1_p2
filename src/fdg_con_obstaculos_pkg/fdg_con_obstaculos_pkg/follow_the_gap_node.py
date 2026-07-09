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

        self.car_width = 1.17  # Protege la cola sin cegar al auto

        # Filtros de visión y dirección
        self.lidar_filter_window = 5
        self.ema_alpha           = 0.35
        self.raw_angle_ema       = 0.0

        # Control PD, conectado a la salida final
        self.Kp                  = 0.28
        self.Kd                  = 0.62
        self.prev_error          = 0.0

        # NUEVO: Escala de velocidad desde el launch file
        self.declare_parameter('speed_scale', 1.0)
        self.speed_scale = self.get_parameter('speed_scale').value

        # Limitador de latigazos mecánicos (Slew-Rate), aplicado por ciclo
        self.rate_open           = 0.03
        self.rate_tight          = 0.09
        self.angle_trigger_min   = 0.07
        self.angle_trigger_max   = 0.27
        self.steering_deadband   = 0.015
        self.prev_steering_angle = 0.0
        self.max_steer           = 0.4189
        self.instability_ema     = 0.0

        # Velocidad actual (para ventana de visión dinámica), viene de odom
        self.current_speed = 0.0

        # Ventana de visión: límites entre los que se interpola según velocidad
        self.fov_narrow = (0.25, 0.75)  # a alta velocidad, en recta
        self.fov_wide   = (0.10, 0.90)  # a baja velocidad, en chicanas/curvas cerradas
        self.speed_low  = 4.5
        self.speed_high = 9.0

        # Histeresis de selección de gap (evita que el "gap ganador" salte entre frames)
        self.prev_gap_start    = None
        self.prev_gap_end      = None
        self.gap_switch_margin = 1.20  # el nuevo gap debe ser 20% mas grande para reemplazar al anterior

        # Métrica de estabilidad de la escena (reemplaza a raw_angle_ema como criterio de t_traj)
        self.angle_history = deque(maxlen=8)

        # Sostiene la ultima lectura valida del LiDAR, para rellenar dropouts (inf/nan)
        self._last_valid_ranges = None

        # Margenes de seguridad respecto al borde de la pista
        self.hard_min_clearance = 0.55  # metros, descarta como "camino" cualquier lectura mas cercana
        self.bubble_min_dist    = 0.28  # piso para el calculo de la burbuja de seguridad
        self.max_weight_depth   = 0.60  # tope de cuanto se favorece el punto mas profundo vs el centro

        # --- TELEMETRÍA ---
        self.start_x           = None
        self.start_y           = None
        self.lap_start_time    = None
        self.lap_count         = 0
        self.distance_traveled = 0.0
        self.last_x            = None
        self.last_y            = None
        self.left_start_zone   = False

        self._odom_check_timer = self.create_timer(3.0, self._check_odom_connection)
        self.get_logger().info("Piloto FTG v4 iniciado.")

    def _check_odom_connection(self):
        # Acepta el nombre del launch (ftg_ego) o el nombre por defecto (follow_the_gap_node)
        if self.get_name() in ['ftg_ego', 'follow_the_gap_node']:
            if self.start_x is None:
                self.get_logger().error("Sin odometría en '/ego_racecar/odom' tras 3s.")
            else:
                self.get_logger().info("Odometría OK en '/ego_racecar/odom'.")
                
        # Pero TODOS los autos cancelan su temporizador para no consumir recursos
        self._odom_check_timer.cancel()

    def odom_callback(self, msg):
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
        if self.distance_traveled > 15.0: self.left_start_zone = True
        
        if self.left_start_zone and math.hypot(cx - self.start_x, cy - self.start_y) < 2.0:
            lap_time = time.time() - self.lap_start_time
            self.lap_count += 1
            
            # NUEVO: Verificamos que sea el auto principal, ya sea por launch o directo
            if self.get_name() in ['ftg_ego', 'follow_the_gap_node']:
                self.get_logger().info(f"🏁 [VUELTA {self.lap_count}] Tiempo: {lap_time:.2f} s")
                
            self.lap_start_time = time.time(); self.distance_traveled = 0.0; self.left_start_zone = False

    def preprocess_lidar(self, ranges):
        # Rellena dropouts (inf/nan) con la ultima lectura valida en vez de asumir via libre
        r = np.array(ranges, dtype=np.float32)
        invalid = np.isinf(r) | np.isnan(r)

        if self._last_valid_ranges is None:
            r[invalid] = 0.0
        else:
            r[invalid] = self._last_valid_ranges[invalid]

        self._last_valid_ranges = r.copy()
        return r

    def smooth_ranges(self, ranges):
        k = np.ones(self.lidar_filter_window) / self.lidar_filter_window
        return np.convolve(ranges, k, mode='same')

    def get_dynamic_fov(self):
        # Interpola el FOV util entre estrecho (recta rapida) y amplio (chicana/curva lenta)
        t = float(np.clip((self.current_speed - self.speed_low) / (self.speed_high - self.speed_low), 0.0, 1.0))
        lo = self.fov_wide[0] + t * (self.fov_narrow[0] - self.fov_wide[0])
        hi = self.fov_wide[1] + t * (self.fov_narrow[1] - self.fov_wide[1])
        return lo, hi

    def select_gap(self, front_ranges):
        # Division simple por continuidad de indice (base estable, sin split por distancia)
        nz = np.where(front_ranges > 0.0)[0]
        if len(nz) == 0:
            return None

        gaps = np.split(nz, np.where(np.diff(nz) != 1)[0] + 1)
        best_gap = max(gaps, key=len)

        if self.prev_gap_start is not None:
            prev_candidates = [g for g in gaps if g[0] <= self.prev_gap_end and g[-1] >= self.prev_gap_start]
            if prev_candidates:
                prev_match = max(prev_candidates, key=len)
                if len(best_gap) < len(prev_match) * self.gap_switch_margin:
                    best_gap = prev_match

        self.prev_gap_start = int(best_gap[0])
        self.prev_gap_end   = int(best_gap[-1])
        return best_gap

    def scan_callback(self, msg):
        ranges = self.preprocess_lidar(msg.ranges)
        ranges = self.smooth_ranges(ranges)
        n = len(ranges)

        # PASO 1: Burbuja Global
        closest_idx_global  = int(np.argmin(ranges))
        closest_dist_global = float(ranges[closest_idx_global])
        bubble_angle      = math.atan2(self.car_width / 2.0, max(closest_dist_global, self.bubble_min_dist))
        rays_to_eliminate = int(bubble_angle / msg.angle_increment)
        b_start = max(0, closest_idx_global - rays_to_eliminate)
        b_end   = min(n, closest_idx_global + rays_to_eliminate)
        ranges[b_start:b_end] = 0.0

        ranges[ranges < self.hard_min_clearance] = 0.0

        # PASO 2: Visión dinámica y búsqueda de gaps con histeresis
        fov_lo, fov_hi = self.get_dynamic_fov()
        start_idx = int(n * fov_lo)
        end_idx   = int(n * fov_hi)
        front_ranges = ranges[start_idx:end_idx].copy()

        largest_gap = self.select_gap(front_ranges)
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

        # PASO 6: Acelerador
        abs_steer = abs(smoothed)
        speed = float(np.clip(5.0+ (target_distance * 0.20) - (10.0 * abs_steer), 4.4, 13.0))

        out = AckermannDriveStamped()
        # NUEVO: Escala de velocidad desde el launch file
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