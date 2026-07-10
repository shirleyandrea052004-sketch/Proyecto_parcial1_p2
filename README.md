# Proyecto Parcial 1: F1TENTH Multivehículo (Follow The Gap)

## 1. Descripción General

Este repositorio contiene un espacio de trabajo de **ROS 2** (`f1tenth_multi_ws`) que implementa una versión modificada del algoritmo de evasión de obstáculos **Follow The Gap** para vehículos autónomos.

El sistema está diseñado para operar en el simulador **F1TENTH**, procesando datos de escaneo **LiDAR 2D** (`/scan`) y de odometría (`/odom`) para determinar el espacio libre de colisión y calcular comandos dinámicos de dirección y velocidad (`AckermannDriveStamped`).

La lógica de automatización se mantiene contenida en su totalidad dentro de un único programa principal. Esta arquitectura centralizada optimiza el procesamiento del simulador y evita la segmentación en múltiples bloques funcionales. Es importante destacar que todos los vehículos (tanto el principal como los oponentes) ejecutan exactamente el mismo nodo. La diferenciación de comportamientos se realiza a través del archivo de lanzamiento (`launch`), el cual configura la relación de velocidades (`speed_scale`). A partir de este valor, el algoritmo asigna internamente los parámetros geométricos y de control correspondientes para cada rol.

### 1.1 Mapas y Entornos de Simulación

Este espacio de trabajo integra un paquete dedicado exclusivamente a las pistas de carreras (f1tenth_racetracks). Además de los entornos base incluidos en la instalación estándar del simulador, se ha incorporado esta colección de mapas suplementarios extraídos de repositorios externos especializados en trazados de competición (como es el caso del circuito Oschersleben_obs). Todos los archivos de imagen (.png) y configuración geométrica (.yaml) se encuentran centralizados en este directorio paralelo dentro de la arquitectura del proyecto, garantizando la ejecución inmediata de cualquier circuito sin requerir dependencias o descargas adicionales.

### 1.2 Modificaciones respecto a la primera versión

- **Implementación de Roles Dinámicos:** Se reestructuró la asignación de parámetros para soportar agentes con distintos perfiles de conducción (conservador vs. agresivo) operando sobre el mismo script, utilizando la variable `speed_scale` como identificador.
- **Evasión Dinámica:** Se integró un control de histéresis modificado y limitadores de tasa de giro (slew-rate) para permitir rebases a obstáculos móviles.
- **Resolución de Colisiones Internas:** Se ajustó la ponderación de profundidad y la tolerancia de proximidad para evitar que los oponentes de baja velocidad colisionen contra los muros internos de las curvas.


### 1.3 Videos demostrativos

- **Video con obstáculos fijos:** <https://youtu.be/HVPSQLNa-fM>
- **Video de carrera multivehículo (2 oponentes, 10 vueltas):** <https://youtu.be/k7g-zZgQDug>

> **Nota:** El sistema actual es capaz de completar 10 vueltas autónomas consecutivas en escenarios con múltiples agentes. Sin embargo, el algoritmo con la configuración presente aún tiene detalles por pulir frente a encierros generados por oponentes. La forma en la que está estructurado lo hace adecuado tanto para realizar giros a alta velocidad como para adoptar posturas más conservadoras para evasión de obstáculos. 
Una conducción óptima y libre de colisiones residuales requiere de un ajuste más fino de los parámetros presentados.

---

## 2. Estructura del Repositorio

Este repositorio contiene únicamente el código fuente (src). Las carpetas generadas automáticamente por ROS 2 (build, install y log) se omiten siguiendo las buenas prácticas de control de versiones. El directorio independiente f1tenth_racetracks contiene la totalidad de los circuitos base y suplementarios requeridos para la simulación.

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
│   ├── f1tenth_racetracks/
│   │   ├── maps/
│   │   │   ├── Oschersleben_obs.png
│   │   │   ├── Oschersleben_obs.yaml
│   │   │   └── ... (mapas consolidados)
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

Modificar la ruta correspondiente al entorno de prueba:

```yaml
map_path: Oschersleben_obs
```

## Número de agentes

- **Pruebas individuales:**

```yaml
num_agent: 1
```

- **Carrera multivehículo:**

```yaml
num_agent: 3
```

---

# 6. Ejecución

## 6.1 Modo Individual

Utilizado para validar la resistencia algorítmica y métricas base.

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

## 6.2 Modo Carrera

Ejecuta tres instancias del algoritmo simultáneamente con parámetros cinemáticos diferenciados. Requiere `num_agent: 3` en `sim.yaml`.

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

# 7. Arquitectura del Algoritmo

El nodo principal (`follow_the_gap_node.py`) procesa la información en un ciclo de seis etapas por cada mensaje recibido desde el tópico LiDAR:

1. **Saneamiento y Filtrado:** Los datos crudos (incluyendo lecturas infinitas o nulas) son interpolados y pasados a través de un filtro de media móvil para reducir ruido de alta frecuencia.

2. **Burbuja de Seguridad:** Se identifica el obstáculo más cercano y se anulan los rayos láser en un radio proyectado según el ancho físico del vehículo, previniendo colisiones de proximidad.

3. **Restricción Visual y Selección de Huecos:** El campo de visión (FOV) se recorta dinámicamente según la velocidad del vehículo. Posteriormente, el algoritmo segmenta los vectores no nulos para encontrar ventanas continuas que superen un ancho mínimo viable.

4. **Cálculo de Trazada:** Se evalúa el arreglo de distancias en el hueco seleccionado. La dirección final se pondera matemáticamente utilizando una relación entre el punto geométrico central del espacio libre y la lectura de mayor profundidad.

5. **Control Direccional PD:** La diferencia angular resultante es introducida a un controlador Proporcional-Derivativo, el cual incluye límites mecánicos de actuación (*slew-rate*) para emular restricciones físicas del servomotor y amortiguar sobreimpulsos.

6. **Acelerador Dinámico:** La velocidad lineal final se calcula modularmente. Aumenta con la distancia libre frontal y se somete a una penalización sustractiva proporcional a la magnitud del giro del volante para garantizar tracción.

---

# 8. Calibración del Algoritmo (Parámetros)

Las variables operativas del nodo determinan el comportamiento dinámico del vehículo. La modificación de las siguientes variables permite adaptar la conducción según el escenario.

## Parámetros de Percepción

### `lidar_filter_window`

Define la cantidad de rayos consecutivos promediados. Valores menores agudizan la detección de bordes, mientras que valores mayores suavizan la señal a costa de introducir retardo.

### `car_width`

Ancho base utilizado para calcular la viabilidad de un espacio. Aumentar este valor fuerza al vehículo a descartar trayectorias estrechas.

### `hard_min_clearance`

Margen absoluto de seguridad. Cualquier lectura LiDAR inferior a este umbral se considera bloqueada, operando como un perímetro repulsivo contra los muros.

### `bubble_min_dist`

Radio mínimo para la proyección de la burbuja de seguridad alrededor del objeto más cercano detectado.

---

## Parámetros de Control Direccional

### `Kp` (Proporcional)

Determina la agresividad de corrección hacia el ángulo objetivo.

### `Kd` (Derivativo)

Amortigua la inercia angular para evitar oscilaciones o sobreimpulsos cruzando la línea central.

### `rate_open` / `rate_tight`

Límites mecánicos simulados para el servomotor. Dictan la velocidad máxima permitida para cambios en el ángulo de dirección en condiciones normales y de emergencia, respectivamente.

### `ema_alpha`

Factor de ponderación del Filtro de Media Móvil Exponencial (EMA). Valores cercanos a `1.0` proporcionan reflejos instantáneos; valores menores introducen retención de trayectoria.

---

## Parámetros de Estrategia

### `max_weight_depth`

Ponderación geométrica de la trayectoria. Un valor alto hace que el vehículo apunte hacia la línea de visión más larga (el ápice de la curva), mientras que un valor bajo lo obliga a mantenerse en el centro geométrico del pasillo.

### `gap_switch_margin`

Margen de histéresis. Determina la ventaja proporcional que debe tener una ruta alternativa para que el vehículo decida abandonar su carril actual. Controla la resistencia a oscilar entre huecos (Asno de Buridán).

---

## Parámetros de Propulsión

### `speed_dist_gain`

Coeficiente multiplicador aplicado a la distancia libre frontal detectada.

### `speed_steer_penalty`

Factor de freno. Resta velocidad de forma directamente proporcional al ángulo aplicado en el volante, simulando transferencia de peso direccional.
