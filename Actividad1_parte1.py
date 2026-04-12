import mujoco
import mujoco.viewer
import numpy as np
import time

# 1. Cargar el modelo físico y sus datos
# Usamos scene.xml porque incluye el robot, iluminación y un suelo.
modelo = mujoco.MjModel.from_xml_path('mujoco_menagerie\universal_robots_ur5e\scene.xml')
datos = mujoco.MjData(modelo)

# Variables de control de estado
stage = 0


def key_callback(keycode):
    global stage
    #print("tecla:", keycode)
    if keycode == 32:  # espacio
        stage += 1 
        print(f"estado actual {stage}")


with mujoco.viewer.launch_passive(modelo, datos, key_callback=key_callback) as viewer:

    dt = modelo.opt.timestep

    while viewer.is_running():
        ####### --------- Ocupando IK lleve al robot a una posicion objetivo y corrobore el error de posicion con los datos del simulador --------------------------- ###############################
        if stage == 0:
            ee_id = modelo.body('wrist_3_link').id
            pos = datos.xpos[ee_id]
            print("EE position:", pos)

        ####### --------- Realice la trayectoria pedida en el enunciado --------------------------- ###############################   
        elif stage == 1:
            pass

        mujoco.mj_step(modelo, datos)
        viewer.sync()
        time.sleep(modelo.opt.timestep)