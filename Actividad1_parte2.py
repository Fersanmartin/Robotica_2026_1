import mujoco
import mujoco.viewer
import numpy as np
import time
import utils as utils


####### ----------------- Agregue Modelo de cubo y gripper a la escena del robot ur5e ###############################
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
        
        ####### --------- Encuentre la posicion del cubo a agarrar --------------------------- ###############################
        if stage == 0:
            cubo =  modelo.body('box').id
            pos = datos.xpos[cubo]
            print("pos cubo:", pos)

        ####### --------- Ocupando la IK del ur5 realizada en utils encuentre la posicion a la cual agarre el cubo --------------------------- ###############################
        elif stage == 1:
            pass

        ####### --------- LLevar el ur5 a esa posicion y cerrar el gripper --------------------------- ###############################
        elif stage == 2:
            pass
    
        ####### --------- llevar el cubo a la poscion objetivo y liberar agarre --------------------------- ###############################
        elif stage == 3:
            pass


        mujoco.mj_step(modelo, datos)
        viewer.sync()
        time.sleep(modelo.opt.timestep)