# `roboclaw_ros2`

Paquete ROS 2 para controlar una base movil diferencial con controladores Basicmicro RoboClaw.

Este paquete resuelve tres tareas principales:

1. Recibir `cmd_vel` y convertirlo a velocidades de rueda en ticks.
2. Leer y publicar la telemetria del RoboClaw.
3. Estimar la odometria de la base a partir de los encoders.

## Dependencia externa

La comunicacion serial con el RoboClaw se hace con la libreria `basicmicro`.

```bash
pip3 install basicmicro
```

Repositorio:

```text
https://github.com/basicmicro/basicmicro_python
```

## Que hace cada archivo

### Nodos ROS 2

`roboclaw_ros2/nodes/roboclaw_node.py`

- Es el nodo principal de control del RoboClaw.
- Se suscribe a `cmd_vel`.
- Convierte `linear.x` y `angular.z` a velocidad izquierda y derecha de las ruedas usando cinematica diferencial.
- Convierte esa velocidad a ticks usando:
  `ticks_per_meter = ticks_per_revolution / (2 * pi * wheel_radius)`.
  Cuentas pro revolución es un dato que te entrega el proveedor del motor. 
- Envia los setpoints al RoboClaw con `SpeedM1M2`.
- Publica telemetria:
  - `roboclaw/encoder/m1`
  - `roboclaw/encoder/m2`
  - `roboclaw/speed/m1`
  - `roboclaw/speed/m2`
  - `roboclaw/current/m1`
  - `roboclaw/current/m2`
  - `roboclaw/voltage/main_battery`
  - `roboclaw/voltage/logic_battery`
  - `roboclaw/temperature/sensor1`
  - `roboclaw/temperature/sensor2`
  - `roboclaw/error`
  - `roboclaw/cmd_ticks/left`
  - `roboclaw/cmd_ticks/right`
- Tiene timeout de seguridad: si deja de llegar `cmd_vel`, manda velocidad cero.

`roboclaw_ros2/nodes/roboclaw_odometry.py`

- Calcula la odometria de la base movil.
- Se suscribe a topicos de encoder izquierdo y derecho.
- Puede usar un encoder por lado o varios encoders por lado.
- Si hay varios encoders por lado, promedia sus deltas.
- Publica:
  - `odom` como `nav_msgs/msg/Odometry`
  - transformada TF `odom -> base_link`
- Ignora saltos muy grandes de encoder para no corromper la pose cuando un controlador se reconecta o resetea encoders.

### Scripts de escritorio

`roboclaw_ros2/scripts/Roboclaw_PWM_dashboard.py`

- Es una interfaz Tkinter para pruebas manuales.
- No usa ROS 2.
- Sirve para:
  - conectarse por serial al RoboClaw
  - mandar PWM o duty cycle manual a M1 y M2
  - detener motores
  - resetear encoders
  - ver telemetria en vivo
- Es util para diagnostico rapido cuando quieres probar hardware sin lanzar todo el stack ROS 2.

`roboclaw_ros2/scripts/roboclaw_data_acquisition.py`

- Es un script standalone para adquisicion de datos de identificacion de sistema.
- No usa ROS 2.
- Envia 4 niveles de voltaje al motor como comando duty/PWM sobre M1 o M2.
- Lee encoder y calcula velocidad en ticks/s con:
  `y = (ticks_actual - ticks_anterior) / Ts`
- Calcula la entrada equivalente en voltaje con:
  `u = V_bateria * duty_cycle`
- Guarda `u.mat` y `y.mat` con `scipy.io.savemat`.

### Configuracion

`config/params.yaml`

- Contiene los parametros por defecto del nodo de control y del nodo de odometria.
- Tiene una seccion para `roboclaw_node`.
- Tiene otra seccion para `roboclaw_odometry`.

### Launch files

`launch/roboclaw_node.launch.py`

- Lanza un solo nodo `roboclaw_node`.
- Usa por defecto el archivo `config/params.yaml`.

`launch/roboclaw_dual.launch.py`

- Lanza dos RoboClaw al mismo tiempo.
- Uno en `/dev/ttyACM0`.
- Otro en `/dev/ttyACM1`.
- Los separa por namespace:
  - `roboclawfront`
  - `roboclawrear`
- Ambos reciben el mismo `cmd_vel`.
- Sirve para una base 4WD o una base con dos controladores, uno delantero y uno trasero.

`launch/roboclaw_odometry.launch.py`

- Lanza el nodo `roboclaw_odometry`.
- Usa por defecto `config/params.yaml`.

### Otros archivos

`setup.py`

- Registra los ejecutables ROS 2:
  - `roboclaw_node`
  - `roboclaw_odometry`
- Instala tambien los archivos de configuracion y launch.

`package.xml`

- Declara las dependencias ROS 2 del paquete.

`test/test_flake8.py`
`test/test_pep257.py`
`test/test_copyright.py`

- Son pruebas de estilo y licencia.
- No controlan el hardware.

## Parametros principales

### `roboclaw_node`

- `port`: puerto serial del RoboClaw, por ejemplo `/dev/ttyACM0`.
- `baud`: baudrate serial.
- `address`: direccion del controlador, normalmente `128` (`0x80`).
- `poll_rate_hz`: frecuencia de lectura de telemetria.
- `control_rate_hz`: frecuencia del loop de control.
- `max_speed`: velocidad lineal maxima aceptada desde `cmd_vel`, en m/s.
- `ticks_per_revolution`: ticks de encoder por una vuelta completa de la rueda.
- `wheel_radius`: radio de la rueda en metros.
- `base_width`: distancia entre rueda izquierda y derecha.
- `cmd_vel_timeout`: tiempo maximo sin recibir `cmd_vel` antes de mandar cero.
- `reset_encoders_on_connect`: si es `true`, resetea encoders al conectarse.

### `roboclaw_odometry`

- `odom_topic`: topico de salida de odometria.
- `odom_frame_id`: frame padre de odometria.
- `base_frame_id`: frame hijo del robot.
- `publish_tf`: publica o no la transformada TF.
- `publish_rate_hz`: frecuencia de publicacion de odometria.
- `ticks_per_revolution`: mismo valor fisico usado en el nodo de control.
- `wheel_radius`: radio de la rueda en metros.
- `base_width`: distancia entre ruedas izquierda y derecha.
- `max_encoder_jump_counts`: filtro para descartar saltos grandes de encoder.
- `left_encoder_topics`: lista de topicos de encoder izquierdo.
- `right_encoder_topics`: lista de topicos de encoder derecho.

## Como usar el paquete

### 1. Compilar

```bash
cd ~/tesis_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select roboclaw_ros2
source install/setup.bash
```

### 2. Ejecutar un solo RoboClaw

```bash
ros2 run roboclaw_ros2 roboclaw_node --ros-args --params-file ~/tesis_ws/src/roboclaw_ros2/config/params.yaml
```

o con launch:

```bash
ros2 launch roboclaw_ros2 roboclaw_node.launch.py
```

### 3. Ejecutar dos RoboClaw

```bash
ros2 launch roboclaw_ros2 roboclaw_dual.launch.py
```

Esto crea dos nodos:

- `/roboclawfront/roboclaw_node`
- `/roboclawrear/roboclaw_node`

y separa su telemetria por namespace.

### 4. Ejecutar la odometria

```bash
ros2 run roboclaw_ros2 roboclaw_odometry --ros-args --params-file ~/tesis_ws/src/roboclaw_ros2/config/params.yaml
```

o con launch:

```bash
ros2 launch roboclaw_ros2 roboclaw_odometry.launch.py
```

## Configuracion de odometria para un solo RoboClaw

Los valores por defecto ya asumen:

```yaml
left_encoder_topics:
  - "roboclaw/encoder/m1"
right_encoder_topics:
  - "roboclaw/encoder/m2"
```

Esto significa:

- `M1` se interpreta como rueda izquierda
- `M2` se interpreta como rueda derecha

## Configuracion de odometria para dos RoboClaw

Si usas `roboclaw_dual.launch.py`, la telemetria sale con namespace. En ese caso, para la odometria debes cambiar los topicos a algo como:

```yaml
roboclaw_odometry:
  ros__parameters:
    left_encoder_topics:
      - "/roboclawfront/roboclaw/encoder/m1"
      - "/roboclawrear/roboclaw/encoder/m1"
    right_encoder_topics:
      - "/roboclawfront/roboclaw/encoder/m2"
      - "/roboclawrear/roboclaw/encoder/m2"
```

Con eso, el nodo de odometria promedia la rueda izquierda del RoboClaw delantero y trasero, y lo mismo para la rueda derecha.

## Suposiciones importantes

Este paquete asume:

- `M1` corresponde al lado izquierdo.
- `M2` corresponde al lado derecho.
- `ticks_per_revolution` corresponde a una vuelta completa de la rueda.

Si tu encoder esta medido en el eje del motor y no en la rueda, debes incluir la relacion de engranajes en ese valor.

## Recomendaciones practicas

- Primero prueba conectividad y telemetria con `roboclaw_node`.
- Luego verifica que al publicar `cmd_vel` las ruedas giren en el sentido esperado.
- Despues activa `roboclaw_odometry`.
- Si la odometria avanza al reves o gira al lado contrario, revisa:
  - el cableado fisico de izquierda/derecha
  - el signo real de los encoders
  - que `M1` y `M2` coincidan con la suposicion del codigo

## Resumen rapido

- `roboclaw_node.py`: control del RoboClaw + telemetria.
- `roboclaw_odometry.py`: odometria de la base movil.
- `Roboclaw_PWM_dashboard.py`: interfaz manual de diagnostico sin ROS.
- `roboclaw_node.launch.py`: un RoboClaw.
- `roboclaw_dual.launch.py`: dos RoboClaw.
- `roboclaw_odometry.launch.py`: nodo de odometria.
