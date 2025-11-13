# Parte Teórica: TFs y REP en ROS 2

Esta sección presenta el marco teórico que fundamenta el uso de transformaciones de coordenadas (**TFs**) en ROS 2, relacionando la práctica con los **ROS Enhancement Proposals (REP)** oficiales que definen las convenciones espaciales y los nombres de los *frames* de referencia.

---

## 1. Introducción a los REP

Los **ROS Enhancement Proposals (REPs)** son documentos normativos que definen estándares, convenciones y buenas prácticas dentro del ecosistema ROS.  
Son equivalentes a los *RFCs* en otros sistemas y garantizan la interoperabilidad entre distintos nodos, librerías y versiones.

Los REPs más relevantes para el trabajo con TFs son:

- **REP 103** → *Standard Units of Measure and Coordinate Conventions*  
- **REP 105** → *Coordinate Frames for Mobile Platforms*  
- **REP 120** → *TF2 and Time Representation*  
- **REP 2011** → *ROS 2 Coordinate Frames Convention (TF2 in ROS 2)*  

Cada uno define un aspecto de las transformaciones espaciales y su significado dentro de ROS 2.

### Aplicación en la práctica

El proyecto se compone de tres nodos principales en formato Python, cada uno implementado en un archivo independiente dentro del paquete `human_tf_camera_example`:

- **`static_broadcaster.py`**: publica una transformación estática entre el portátil (`base_link`) y la cámara (`camera_frame`), indicando la posición fija del sensor respecto al centro del equipo.  
- **`human_position_estimator.py`**: captura imágenes en tiempo real desde la cámara, detecta el rostro del humano mediante OpenCV y publica un frame dinámico (`human_in_camera`) que representa su posición respecto a la cámara.  
- **`human_position_to_baselink.py`**: escucha las transformaciones publicadas y calcula la posición del humano respecto al portátil (`base_link`), combinando los TFs anteriores para estimar la distancia total.

Estos tres componentes permiten construir y visualizar un árbol de transformaciones coherente en ROS 2, siguiendo las convenciones establecidas en los REPs 103, 105, 120 y 2011.


---

##  2. REP 103 – Standard Units of Measure and Coordinate Conventions

 **Enlace:** [https://www.ros.org/reps/rep-0103.html](https://www.ros.org/reps/rep-0103.html)

El **REP 103** establece las unidades y las convenciones geométricas que se deben usar en ROS:

- Se utiliza el **Sistema Internacional de Unidades (SI)**.  
- Las posiciones y distancias se expresan en **metros (m)**.  
- Las orientaciones se expresan en **radianes (rad)**.  
- El sistema de coordenadas es **right-handed** (regla de la mano derecha):  
  - **X → adelante**  
  - **Y → izquierda**  
  - **Z → arriba**

### Aplicación en la práctica

En el ejemplo:
```
base_link → camera_frame
```

<!-- La transformación `[x=0.1, y=0.0, z=0.2]` sigue la convención definida en REP 103, indicando que la cámara está situada **10 cm delante y 20 cm por encima** del origen del portátil. -->

En el archivo `static_broadcaster.py` se define la **transformación estática** entre el frame del portátil (`base_link`) y el frame de la cámara (`camera_frame`):

```python
t.header.frame_id = 'base_link'
t.child_frame_id = 'camera_frame'
t.transform.translation.x = 0.1
t.transform.translation.y = 0.0
t.transform.translation.z = 0.2
```

Esta configuración establece la posición de la cámara **10 cm por delante** y **20 cm por encima** del centro geométrico del portátil, considerado como el origen del frame `base_link`.  

Según el **REP 103 (Standard Units of Measure and Coordinate Conventions)**, ROS emplea un sistema de coordenadas *right-handed* (regla de la mano derecha), donde:
- El eje **X** apunta hacia adelante.  
- El eje **Y** apunta hacia la izquierda.  
- El eje **Z** apunta hacia arriba.  

De este modo, el vector de transformación:
```
(x=0.1, y=0.0, z=0.2)
```
describe una cámara situada en la parte superior frontal del portátil, simulando la ubicación real de una webcam.

Esta relación se mantiene fija durante toda la ejecución y se publica mediante un **TransformBroadcaster estático**, cumpliendo con las recomendaciones del **REP 105 (Coordinate Frames for Mobile Platforms)**, que define `base_link` como el origen del sistema de coordenadas del robot o plataforma móvil, y `camera_frame` como un sensor montado rigidamente sobre él.

En conjunto, la transformación:
```
base_link → camera_frame
```
proporciona la referencia espacial necesaria para expresar, en pasos posteriores, la posición del humano (`human_in_camera`) tanto en el sistema de la cámara como en el sistema global del portátil.


### Transformaciones estáticas y dinámicas en TFs

En ROS 2, las transformaciones publicadas mediante **TF2** se dividen en dos categorías principales: **estáticas** y **dinámicas**, según su relación temporal con el sistema de coordenadas.

- **Transformaciones estáticas**: describen una relación **fija en el tiempo** entre dos *frames*.  
  No cambian durante la ejecución y se publican normalmente una sola vez al inicio del sistema o a una frecuencia muy baja.  
  Se utilizan para representar elementos que no se mueven entre sí, como un sensor montado rígidamente sobre una base.  
  <br> *Ejemplo:* en `static_broadcaster.py`, la transformación `base_link → camera_frame` define la posición constante de la cámara respecto al portátil.

- **Transformaciones dinámicas**: representan relaciones **que varían con el tiempo**, como el movimiento de un objeto, un robot o una persona detectada.  
  Se actualizan periódicamente mediante un `TransformBroadcaster` que publica nuevas transformaciones con diferentes marcas temporales (`stamp`).  
  <br> *Ejemplo:* en `human_position_estimator.py`, la transformación `camera_frame → human_in_camera` cambia continuamente conforme el humano se mueve frente a la cámara.

En la práctica, un sistema TF combina ambos tipos:  
las transformaciones estáticas definen la geometría fija del robot, y las dinámicas expresan su interacción con el entorno en tiempo real.



---

## 3. REP 105 – Coordinate Frames for Mobile Platforms

 **Enlace:** [https://www.ros.org/reps/rep-0105.html](https://www.ros.org/reps/rep-0105.html)

El **REP 105** define los nombres estándar de los *frames* de referencia utilizados en robótica móvil, así como su jerarquía y relaciones espaciales.

### Frames típicos:
- `map` → sistema global de referencia.
- `odom` → sistema local sin corrección de deriva.
- `base_link` → origen del robot o plataforma móvil.
- `camera_frame`, `lidar_frame`, `imu_link` → sensores y actuadores.

### Jerarquía estándar:

```
map → odom → base_link → sensor_frame
```


### Aplicación en la práctica

En este proyecto se utiliza una jerarquía simplificada:

```
base_link → camera_frame → human_in_camera
```

- `base_link`: representa el portátil (plataforma base).  
- `camera_frame`: representa la cámara integrada.  
- `human_in_camera`: representa el frame dinámico del humano detectado.  

Esta estructura cumple las recomendaciones del REP 105 sobre nombres y relaciones espaciales coherentes.

![Árbol de transformaciones en RViz2](ros2_ws/src/human_tf_camera_example/resources/captura_rviz2.png)
*Figura 1. Representación del árbol de TFs en RViz2, mostrando la relación entre los frames del portátil (base_link), la cámara (camera_frame) y el humano (human_in_camera).*

---

## 4. REP 120 – TF2 and Time Representation

 **Enlace:** [https://www.ros.org/reps/rep-0120.html](https://www.ros.org/reps/rep-0120.html)

El **REP 120** describe cómo funciona el sistema **TF2**, que permite manejar transformaciones temporales entre distintos *frames*.

### Conceptos principales:
- Las transformaciones se publican como mensajes `TransformStamped` con una marca temporal (`stamp`).  
- Las consultas de transformación se realizan sobre un **Buffer** administrado por un **TransformListener**.  
- TF2 permite **interpolar transformaciones** en el tiempo para sincronizar sensores asíncronos.

### Aplicación en la práctica

El nodo `human_position_to_baselink.py` utiliza un **TransformListener** y un **Buffer** de TF2 para calcular la posición del humano en el frame `base_link` combinando las transformaciones:

```
base_link → camera_frame → human_in_camera
```


Este diseño cumple con la estructura definida en REP 120 y garantiza que las transformaciones se calculen de forma coherente en el tiempo.

---

## 5. REP 2011 – ROS 2 Coordinate Frames Convention (TF2 in ROS 2)

 **Enlace:** [https://ros.org/reps/rep-2011.html](https://ros.org/reps/rep-2011.html)

El **REP 2011** actualiza las convenciones de TF2 para ROS 2, manteniendo compatibilidad con los REPs anteriores (103 y 105), y estandariza los nombres de *frames* y su interpretación física.

### Puntos clave:
- Los *frames* deben tener nombres **significativos y consistentes**.  
- Los sistemas de coordenadas deben respetar el eje **right-handed**.  
- Todas las transformaciones deben tener **significado físico** dentro del robot.

### Aplicación en la práctica

Este proyecto sigue las directrices del REP 2011 al utilizar los *frames*:

```
base_link, camera_frame, human_in_camera
```


Estos nombres son semánticamente significativos, compatibles con las herramientas de ROS 2 (`rviz2`, `tf2_tools`) y mantienen coherencia con la jerarquía oficial de *frames*.

---

## 6. Conclusión

La práctica implementa un ejemplo completo de transformaciones de coordenadas en ROS 2 conforme a las convenciones establecidas por los REPs:

| REP | Contenido | Aplicación en el ejemplo |
|-----|------------|---------------------------|
| **103** | Unidades y ejes (X, Y, Z) | Ejes orientados según convención ROS |
| **105** | Frames estándar | `base_link`, `camera_frame`, `human_in_camera` |
| **120** | TF2 y tiempo | Uso de `TransformBroadcaster` y `TransformListener` |
| **2011** | Convenciones en ROS 2 | Coherencia semántica de los nombres de frames |

> En conjunto, estos REPs garantizan que las transformaciones definidas en este sistema (`base_link → camera_frame → human_in_camera`) sean interoperables, físicamente coherentes y compatibles con el ecosistema de ROS 2.

---

## Referencias

- ROS Enhancement Proposal REP 103: [https://www.ros.org/reps/rep-0103.html](https://www.ros.org/reps/rep-0103.html)  
- ROS Enhancement Proposal REP 105: [https://www.ros.org/reps/rep-0105.html](https://www.ros.org/reps/rep-0105.html)  
- ROS Enhancement Proposal REP 120: [https://www.ros.org/reps/rep-0120.html](https://www.ros.org/reps/rep-0120.html)  
- ROS Enhancement Proposal REP 2011: [https://ros.org/reps/rep-2011.html](https://ros.org/reps/rep-2011.html)

