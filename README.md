# UR5e Pick & Place Simulation (ROS 2 Humble + Gazebo + MoveIt)

Este repositorio contiene una guía y ejemplos básicos para simular el **UR5e** con **Gazebo (Ignition Gazebo Garden)** y planificar movimientos con **MoveIt 2**, como primer paso hacia una demo de **pick and place**.

---

## 🛠️ Instalación

Primero asegúrate de tener [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) instalado.

### 1. Instalar dependencias de Universal Robots
```bash
sudo apt update
sudo apt install ros-humble-ur
```

### 2. Instalar MoveIt 2
```bash
sudo apt install ros-humble-moveit
```

### 3. Instalar el simulador UR + Gazebo
```bash
sudo apt install ros-humble-ur-simulation-gz
```

---

## ▶️ Lanzar la simulación

Con todo instalado, puedes lanzar el UR5e en Gazebo con MoveIt 2 ya configurado:

```bash
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
```

Esto arranca:
- **Gazebo** con el UR5e
- **ros2_control** con los controladores necesarios
- **MoveIt 2** con RViz2 para planificar trayectorias

---

## 📡 Verificación

### 1. Revisa que los controladores estén cargados:
```bash
ros2 control list_controllers
```

Deberías ver algo como:
```
joint_state_broadcaster [active]
scaled_joint_trajectory_controller [active]
```

### 2. Comprueba que se publiquen los `joint_states`:
```bash
ros2 topic echo /joint_states --once
```

---

## 🤖 Primeros pasos en MoveIt

1. En **RViz2**, selecciona la pestaña **Motion Planning**.  
2. Define una **pose objetivo** para el efector final del UR5e.  
3. Pulsa **Plan** para generar la trayectoria.  
4. Pulsa **Execute** para que el robot simulado en Gazebo ejecute el movimiento.  

---

## 🧩 Próximos pasos: Pick & Place

Este repositorio se ampliará con:

- Un **plugin de Gazebo** para objetos a manipular.  
- **Scene Objects** en MoveIt para que el UR5e planifique evitando colisiones.  
- Un ejemplo de **pick & place** simple (mover un cubo de A → B).  

---

## 📚 Recursos útiles

- [Universal Robots ROS 2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)  
- [MoveIt 2 Tutorials](https://moveit.picknik.ai/humble/index.html)  
- [Ignition Gazebo](https://gazebosim.org/home)  
