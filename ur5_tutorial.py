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

vel_x = 0.0
vel_y = 0.0
vel_z = 0.0

def key_callback(keycode):
    global stage, ejecutar_paso, vel_x, vel_y, vel_z
    #print("tecla:", keycode)
    if keycode == 32:  # espacio
        stage += 1 
        ejecutar_paso = True
        print(f"estado actual {stage}")

    elif keycode == 87:  # W
        vel_z += 1
        print("W presionado")

    elif keycode == 97:  # A
        print("A presionado")

    elif keycode == 83:  # S
        vel_z -= 1
        print("S presionado")

    elif keycode == 100:  # D
        print("D presionado")

with mujoco.viewer.launch_passive(modelo, datos, key_callback=key_callback) as viewer:

    dt = modelo.opt.timestep

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

        elif stage == 4:
            dx = np.zeros(6)
            dx[0] = vel_x
            dx[1] = vel_y
            dx[2] = vel_z

            # Obtener el Jacobiano
            jacp = np.zeros((3, modelo.nv))
            jacr = np.zeros((3, modelo.nv))
            mujoco.mj_jacSite(modelo, datos, jacp, jacr, 6)
            J = np.vstack((jacp, jacr))[:, :6]
            
            # dq = J_pseudo_inversa * Velocidad_Cartesiana
            dq = np.linalg.pinv(J) @ dx
            
            # Integrar la velocidad articular para obtener la posición objetivo
            # q_nueva = q_actual + dq * dt
            q_target =  datos.qpos[:6] + dq * dt
            print(dq * dt)
            datos.ctrl[:6] = q_target
        mujoco.mj_step(modelo, datos)
        viewer.sync()
        time.sleep(modelo.opt.timestep)