# Proyecto Parcial 1: F1TENTH Multivehículo (Follow The Gap)

## 1. Descripción General

Este repositorio contiene un espacio de trabajo de **ROS 2** (`f1tenth_multi_ws`) que implementa una versión modificada del algoritmo de evasión de obstáculos **Follow The Gap** para vehículos autónomos.

El sistema está diseñado para operar en el simulador **F1TENTH**, procesando datos de escaneo **LiDAR 2D** (`/scan`) y de odometría (`/odom`) para determinar el espacio libre de colisión y calcular comandos dinámicos de dirección y velocidad (`AckermannDriveStamped`).

La arquitectura del paquete principal (`fdg_con_obstaculos_pkg`) soporta escenarios multivehículo mediante el remapeo de tópicos en el archivo de lanzamiento. Esto permite la ejecución paralela de múltiples instancias del mismo nodo para simular carreras con agentes estáticos y dinámicos.

## 1.2 Videos demostrativos
Video con obstàculos fijos: https://youtu.be/HVPSQLNa-fM
Video con obstàculos fijos y mòviles: https://youtu.be/AIHKWGRq9hU

Nota: Con la configuración actual de los parámetros de calibracián el vehìculo tiene problemas para rebasar los obtáculos mòviles. Esto será resuelto en la siguiente actualización. 

# 2. Estructura del Repositorio

Este repositorio contiene únicamente el código fuente (`src`). Las carpetas generadas automáticamente por ROS 2 (`build`, `install` y `log`) se omiten siguiendo las buenas prácticas, ya que estas se generarán al compilar.

```text
f1tenth_multi_ws/
├── src/
│   ├── f1tenth_gym_ros/
│   │   ├── config/
│   │   │   └── sim.yaml
│   │   ├── launch/
│   │   │   └── gym_bridge_launch.py
│   │   └── ...
│   │
│   └── fdg_con_obstaculos_pkg/
│       ├── fdg_con_obstaculos_pkg/
│       │   └── follow_the_gap_node.py
│       ├── launch/
│       │   └── carrera_launch.py
│       ├── package.xml
│       └── setup.py
```

---

# 3. Requisitos Previos

- **Sistema Operativo:** Ubuntu 22.04 LTS
- **ROS:** ROS 2 Humble Hawksbill

## Dependencias de Python

- `rclpy`
- `sensor_msgs`
- `nav_msgs`
- `ackermann_msgs`
- `numpy`

---

# 4. Instalación y Compilación

## 4.1 Clonar el repositorio

```bash
git clone https://github.com/shirleyandrea052004-sketch/Proyecto_parcial1_p2.git f1tenth_multi_ws
```

## 4.2 Ingresar al workspace

```bash
cd f1tenth_multi_ws
```

## 4.3 Compilar

```bash
colcon build --symlink-install
```

## 4.4 Cargar el entorno

```bash
source install/setup.bash
```

---

# 5. Configuración del Simulador

Antes de ejecutar el proyecto, configure el archivo:

```text
src/f1tenth_gym_ros/config/sim.yaml
```

## Seleccionar el mapa

Modificar:

```yaml
map_path: Oschersleben_obs
```

(o cualquier otro mapa disponible).

## Número de agentes

### Pruebas individuales

```yaml
num_agent: 1
```

### Carrera multivehículo

```yaml
num_agent: 3
```

---

# 6. Ejecución

## 6.1 Modo Individual

Utilizado para validar la resistencia del algoritmo (por ejemplo, completar 10 vueltas consecutivas sin colisiones).

### Terminal 1

```bash
cd ~/f1tenth_multi_ws
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Terminal 2

```bash
cd ~/f1tenth_multi_ws
source install/setup.bash
ros2 run fdg_con_obstaculos_pkg follow_the_gap
```

---

## 6.2 Modo Carrera

Ejecuta tres instancias del algoritmo simultáneamente para generar escenarios de adelantamiento y evasión.

Configure previamente:

```yaml
num_agent: 3
```

### Terminal 1

```bash
cd ~/f1tenth_multi_ws
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Terminal 2

```bash
cd ~/f1tenth_multi_ws
source install/setup.bash
ros2 launch fdg_con_obstaculos_pkg carrera_launch.py
```

---

# 7. Calibración del Algoritmo (Tuning)

Todos los parámetros principales se encuentran definidos en:

```text
follow_the_gap_node.py
```

dentro de la clase:

```python
FollowTheGapNode
```

## Parámetros

| Parámetro | Función | Aumentar | Disminuir |
|-----------|----------|----------|-----------|
| **car_width** | Radio base de la burbuja de seguridad | Mayor separación respecto a obstáculos | Permite rebases más estrechos, aumenta riesgo de colisión |
| **ema_alpha** | Filtro EMA del ángulo objetivo | Respuesta más rápida | Mayor suavizado, introduce retardo |
| **Kp** | Ganancia proporcional del controlador PD | Correcciones agresivas, posible serpenteo | Dirección lenta en curvas |
| **Kd** | Ganancia derivativa | Reduce oscilaciones | Mayor vibración direccional |
| **rate_tight** | Límite de velocidad del servo | Maniobras evasivas más rápidas | Menor agilidad |
| **gap_switch_margin** | Histéresis para cambiar de trayectoria | Mantiene la ruta actual | Cambios frecuentes de trayectoria |
| **max_weight_depth** | Peso del punto más lejano respecto al centro del gap | Prioriza distancia máxima, puede generar inestabilidad | Prioriza el centro del espacio libre, conducción más estable |

---

