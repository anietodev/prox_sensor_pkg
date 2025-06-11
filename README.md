# Turtlebot3 con Sensores de Proximidad en Entorno Simulado

## 🧠 Descripción

Este es un proyecto desarrollado para la asignatura de **Robótica Móvil**, cuyo objetivo es mejorar el comportamiento del robot **Turtlebot3** integrando sensores de proximidad. Gracias a estos sensores, el robot es capaz de:

- Reducir progresivamente su velocidad al acercarse a obstáculos.
- Detenerse completamente si el obstáculo está muy cerca.
- Bloquear los comandos de avance o retroceso según la posición del obstáculo.

El sistema se ha probado en un entorno simulado de almacén en **CoppeliaSim**.

---

## 🎯 Objetivos

- Simular un entorno seguro y realista para el Turtlebot3.
- Diseñar un sistema de control de velocidad basado en sensores de proximidad.
- Integrar y coordinar nodos ROS 2 para una navegación más segura.

---

## 🛠️ Desarrollo Técnico

### 🔧 Tareas realizadas

- **Creación del paquete** propio a partir de la práctica 2 de la asignatura, basado en `navigation2`.
- **Diseño del nodo `prox_sensor_node`**, que filtra los comandos de velocidad recibidos según la proximidad de obstáculos.
- **Integración de 6 sensores infrarrojos** (IR) dispuestos alrededor del robot.
- **Diseño de escena personalizada en CoppeliaSim** con estanterías para simular un almacén.
- **Publicación de velocidades seguras** a través del topic `/cmd_vel`.

---

## ⚙️ Lógica del Nodo

El nodo principal (`prox_sensor_node`) se suscribe a:

- `/cmd_vel_teleop`: comandos de velocidad desde teclado.
- `/ir1` a `/ir6`: sensores de proximidad colocados en distintas direcciones.
- `/odom`: odometría para visualización.

Publica:

- `/cmd_vel`: velocidad segura calculada tras analizar la proximidad de obstáculos.

### 🧩 Lógica de control

- Si hay un obstáculo a menos de **0.5 m**, se reduce la velocidad.
- Si está a menos de **0.15 m**, se detiene el robot.
- Se bloquean comandos de avance o retroceso si hay un obstáculo frontal o trasero, respectivamente.

---

## 🧾 Fragmento del Nodo

```cpp
geometry_msgs::msg::Twist scale_velocity_by_distance(
    const geometry_msgs::msg::Twist &input_cmd,
    float distance_ir1, float distance_ir2, float distance_ir3,
    float distance_ir4, float distance_ir5, float distance_ir6)
{
    auto scaled_cmd = input_cmd;
    const float min_distance = 0.15;
    const float max_distance = 0.5;
    const float vel_obst = -0.1;

    if (scaled_cmd.linear.x < 0.0) {
        if ((distance_ir1 <= min_distance) || (distance_ir2 <= min_distance) || (distance_ir3 <= min_distance)) {
            scaled_cmd.linear.x = 0.0;
        } else if ((distance_ir1 < max_distance) || (distance_ir2 < max_distance) || (distance_ir3 < max_distance)) {
            if (scaled_cmd.linear.x < vel_obst)
                scaled_cmd.linear.x = vel_obst;
        }
    } else if (scaled_cmd.linear.x > 0.0) {
        if ((distance_ir4 <= min_distance) || (distance_ir5 <= min_distance) || (distance_ir6 <= min_distance)) {
            scaled_cmd.linear.x = 0.0;
        } else if ((distance_ir4 < max_distance) || (distance_ir5 < max_distance) || (distance_ir6 < max_distance)) {
            if (scaled_cmd.linear.x > -vel_obst)
                scaled_cmd.linear.x = -vel_obst;
        }
    }

    return scaled_cmd;
}
```
---
## 📦 Instalación

### Requisitos previos

- ROS 2 (recomendado: Humble Hawksbill)
- CoppeliaSim instalado
- turtlebot3_simulations
- Paquete `teleop_twist_keyboard` para enviar comandos manuales

### Instrucciones

1. Clona el repositorio dentro de tu espacio de trabajo ROS 2:

```bash
cd ~/ros2_ws/src
git clone https://github.com/anietodev/prox_sensor_pkg.git
```
2. Compila el paquete:
- Abre CoppeliaSim y carga la escena del almacén incluida en el proyecto.

3. Ejecuta la simulación:
```bash
ros2 run prox_sensor_pkg prox_sensor_node
```
4. Para controlar manualmente el robot:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop
```
---
## 📝 Licencia
Este proyecto está bajo la licencia MIT.
Puedes utilizar, modificar y distribuir el código libremente siempre que incluyas el aviso de copyright.

---
## 🙌 Agradecimientos
A nuestro profesor de Robótica Móvil, Leopoldo Armesto, por su orientación durante el desarrollo del proyecto.
