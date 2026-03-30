import mujoco
import mujoco.viewer
import numpy as np
import time

# 1. Cargar el modelo físico y sus datos
# Usamos scene.xml porque incluye el robot, iluminación y un suelo.
modelo = mujoco.MjModel.from_xml_path('universal_robots_ur5e/scene.xml')
datos = mujoco.MjData(modelo)

# Variables de control de estado
stage = 0
def key_callback(keycode):
    """Función que se ejecuta cuando se presiona una tecla en la ventana"""
    global stage, ejecutar_paso
    # 32 es el código ASCII para la barra espaciadora
    if keycode == 32:
        stage += 1 
        ejecutar_paso = True
        print(f"estado actual {stage}")

with mujoco.viewer.launch_passive(modelo, datos, key_callback=key_callback) as viewer:
    while viewer.is_running():
        
        if stage == 0:
            # NO escribimos en datos.ctrl aquí.
            # Al dejarlo vacío, los sliders de la interfaz toman el control.
            pass
        elif stage == 1:
            # Escribimos en datos.ctrl aquí.
            # Ahora el modelo se movera a esa posicion y no podremos mover el robot por interfaz
            datos.ctrl[:6] = np.array([0, -1.5, 1.5, -1.5, -1.5, 0])
        elif stage == 2:
            # Podemos acceder al estado de los joints con datos.qpos
            current_q = datos.qpos[:6]
            print("Joint positions:", current_q)
        elif stage == 3:
            # Podemos acceder al id de un joint, por ejemplo el writ_3_link, nuestro end_efector
            # Podemos acceder a la pose de los joints con datos.xpos
            ee_id = modelo.body('wrist_3_link').id
            pos = datos.xpos[ee_id]
            print("EE position:", pos)
            
        mujoco.mj_step(modelo, datos)
        viewer.sync()
        time.sleep(modelo.opt.timestep)