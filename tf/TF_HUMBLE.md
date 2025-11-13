# Ejemplos de TFs y ROS 2 usando la cámara 2D del portátil

Este documento contiene tres ejemplos prácticos y combinables en ROS 2:

1. **TF estático:** publicar la transformación entre `base_link` (portátil) y `camera_frame`.  
2. **Detección:** estimar la posición del humano con respecto a la cámara mediante OpenCV.  
3. **TF encadenado:** calcular y publicar la posición del humano respecto al portátil (`base_link`).

Probado en **ROS 2 Humble**, compatible con **Iron**, **Jazzy** y versiones posteriores.

---

## Ejemplo – Publicar un TF entre `base_link` y `camera_frame`

### Objetivo
Simular que el **portátil** es el robot (`base_link`) y que la **cámara integrada** es su sensor frontal (`camera_frame`).

### Dependencias

```bash
sudo apt install ros-humble-v4l2-camera ros-humble-tf2-ros ros-humble-rviz2
```

### Estructura del paquete

.
└── src
    └── human_tf_camera_example
        ├── human_tf_camera_example
        │   ├── human_position_estimator.py
        │   ├── human_position_to_baselink.py
        │   ├── __init__.py
        │   └── static_broadcaster.py
        ├── launch
        │   └── human_tf.launch.py
        ├── package.xml
        ├── resource
        │   └── human_tf_camera_example
        ├── resources
        │   └── captura_rviz2.png
        ├── setup.cfg
        ├── setup.py
        └── test
            ├── test_copyright.py
            ├── test_flake8.py
            └── test_pep257.py


---

## Ejemplo – Coordenadas del humano respecto al portátil (`base_link`)

### Objetivo
Combinar los dos ejemplos anteriores para calcular la posición del humano tanto:
- En el **frame de la cámara (`camera_frame`)**  
- Como en el **frame del portátil (`base_link`)**, usando TFs encadenados.


###  Ejecución combinada

1. Compilar y lanzar el launch
   ```bash
   cd ros2_ws
   colcon build
   source install/setup.bash
   ros2 launch human_tf_camera_example human_tf.launch.py
   ```


En el terminal verás las coordenadas estimadas de la persona respecto al portátil y respecto a la cámara.

---

## Conclusiones

- El **portátil** actúa como `base_link`.  
- La **cámara** tiene su propio frame `camera_frame` fijo a él.  
- El humano se detecta en el frame de cámara y se transforma al frame del portátil mediante TFs.  
- Puedes visualizar toda la jerarquía (`base_link → camera_frame → human_in_camera`) en RViz.

---

