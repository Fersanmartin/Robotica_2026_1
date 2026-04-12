import mujoco
import mujoco.viewer
import numpy as np
import time
import utils as utils

# 1. Cargar el modelo físico y sus datos
# Usamos scene.xml porque incluye el robot, iluminación y un suelo.
modelo = mujoco.MjModel.from_xml_path('scene.xml')
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
        ####### --------- Compruebe su FK obtiene el valor del efctor final de forma correcta y calcule su error  --------------------------- ###############################
        if stage == 0:
            pass
        ####### --------- Compruebe su IK obtiene el valor correcto de los joints y calcule su error --------------------------- ###############################   
        elif stage == 1:
            pass
        ####### --------- Realice actividad 1 del enunciado --------------------------- ###############################   
        elif stage == 2:
            pass
            
        mujoco.mj_step(modelo, datos)
        viewer.sync()
        time.sleep(modelo.opt.timestep)