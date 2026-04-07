import mujoco
import mujoco.viewer
import numpy as np
import time
from pynput import keyboard

# 1. Cargar el modelo
model = mujoco.MjModel.from_xml_path('universal_robots_ur5e/scene.xml')
data = mujoco.MjData(model)

site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'attachment_site')

# 2. Configurar el estado del teclado
# True significa que la tecla está presionada en este instante
estado_teclas = {
    'w': False, 's': False, 'a': False, 'd': False, 'q': False, 'e': False,
    'up': False, 'down': False, 'left': False, 'right': False,
    ',': False, '.': False
}

def on_press(key):
    try:
        k = key.char.lower()
        if k in estado_teclas: estado_teclas[k] = True
    except AttributeError:
        if key == keyboard.Key.up: estado_teclas['up'] = True
        elif key == keyboard.Key.down: estado_teclas['down'] = True
        elif key == keyboard.Key.left: estado_teclas['left'] = True
        elif key == keyboard.Key.right: estado_teclas['right'] = True

def on_release(key):
    try:
        k = key.char.lower()
        if k in estado_teclas: estado_teclas[k] = False
    except AttributeError:
        if key == keyboard.Key.up: estado_teclas['up'] = False
        elif key == keyboard.Key.down: estado_teclas['down'] = False
        elif key == keyboard.Key.left: estado_teclas['left'] = False
        elif key == keyboard.Key.right: estado_teclas['right'] = False

# Iniciar el detector de teclado en un hilo separado
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# 3. Parámetros de Control Cinemático
q_target = np.array([0, -1.57, 1.57, -1.57, -1.57, 0.0]) 

# Velocidades (Metros por segundo y Radianes por segundo)
vel_lineal = 0.1   # 10 cm/s
vel_angular = 0.5  # ~28 grados/s

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Teleoperación Fluida Iniciada.")
    print("Mantén presionadas las teclas para mover el robot continuamente.")
    
    data.qpos[:6] = q_target
    mujoco.mj_forward(model, data)
    
    dt = model.opt.timestep # Paso de tiempo de la simulación (usualmente 0.002s)

    while viewer.is_running():
        dx = np.zeros(6) # Vector de velocidad cartesiana deseada [vx, vy, vz, wx, wy, wz]
        
        # Asignar velocidades cartesianas según teclas mantenidas
        if estado_teclas['w']: dx[0] = vel_lineal
        if estado_teclas['s']: dx[0] = -vel_lineal
        if estado_teclas['a']: dx[1] = vel_lineal
        if estado_teclas['d']: dx[1] = -vel_lineal
        if estado_teclas['q']: dx[2] = vel_lineal
        if estado_teclas['e']: dx[2] = -vel_lineal
        
        if estado_teclas['up']: dx[4] = vel_angular
        if estado_teclas['down']: dx[4] = -vel_angular
        if estado_teclas['left']: dx[5] = vel_angular
        if estado_teclas['right']: dx[5] = -vel_angular
        if estado_teclas[',']: dx[3] = vel_angular
        if estado_teclas['.']: dx[3] = -vel_angular

        # Si hay alguna velocidad comandada, calculamos la cinemática inversa
        if np.any(dx):
            # Obtener el Jacobiano
            jacp = np.zeros((3, model.nv))
            jacr = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
            J = np.vstack((jacp, jacr))[:, :6]
            
            # dq = J_pseudo_inversa * Velocidad_Cartesiana
            dq = np.linalg.pinv(J) @ dx
            
            # Integrar la velocidad articular para obtener la posición objetivo
            # q_nueva = q_actual + dq * dt
            q_target += dq * dt

        # Enviar comandos y avanzar simulación
        data.ctrl[:6] = q_target
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(dt)