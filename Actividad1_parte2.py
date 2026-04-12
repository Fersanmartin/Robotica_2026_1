import mujoco
import mujoco.viewer
import numpy as np
import time
import utils as utils
from pynput import keyboard

modelo = mujoco.MjModel.from_xml_path('scene.xml')
datos = mujoco.MjData(modelo)
site_id = mujoco.mj_name2id(modelo, mujoco.mjtObj.mjOBJ_SITE, 'robot0:eef_site')
# Variables de control de estado
stage = 0



#Agregue todas las teclas necesarias
estado_teclas = {
    'w': False
}

def on_press(key):
    try:
        k = key.char.lower()
        print(k)
        if k in estado_teclas: estado_teclas[k] = True
    except AttributeError:
        pass

def on_release(key):
    try:
        k = key.char.lower()
        if k in estado_teclas: estado_teclas[k] = False
    except AttributeError:
        pass

# Iniciar el detector de teclado en un hilo separado
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

with mujoco.viewer.launch_passive(modelo, datos) as viewer:

    dt = modelo.opt.timestep

    while viewer.is_running():

        #Ocupano las teclas de su computador realice una teleoperacion basada en Jacobiano
        jacp = np.zeros((3, modelo.nv))
        jacr = np.zeros((3, modelo.nv))

        mujoco.mj_jacSite(modelo, datos, jacp, jacr, site_id)

        J = np.vstack((jacp, jacr))
            
       
        mujoco.mj_step(modelo, datos)
        viewer.sync()
        time.sleep(modelo.opt.timestep)