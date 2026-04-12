import numpy as np

def traslation(x=0, y=0, z=0):
    return np.array([[1, 0, 0, x],
                     [0, 1, 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])

# Rotación en X:
def rot_x(qx):
    return np.array([[1, 0, 0, 0],
                     [0, np.cos(qx), -np.sin(qx), 0],
                     [0, np.sin(qx), np.cos(qx), 0],
                     [0, 0, 0, 1]])
# Rotación en Y:
def rot_y(qy):
    return np.array([[np.cos(qy), 0, np.sin(qy), 0],
                     [0, 1, 0, 0],
                     [-np.sin(qy), 0, np.cos(qy), 0],
                     [0, 0, 0, 1]])
# Rotación en Z:
def rot_z(qz):
    return np.array([[np.cos(qz), -np.sin(qz), 0, 0],
                     [np.sin(qz), np.cos(qz), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


def F_matrices(q):
    #Reescriba los largos de los links para que calcen con el robot pedido
    d1 = 0.1625
    d2 = 0.425
    d3 = 0.3922
    d4 = 0.1333
    d5 = 0.0997
    d6 = 0.0996
    tcp = 0.1

    theta1, theta2, theta3,  theta4,  theta5, theta6 = q

    #Escriba la funcion de cinematica directa del robot pedido
    T0 =  traslation(0, 0, 0) 
    T1 = T0 @ traslation(0, 0, 0) 

    # posición final
    pos = T1[0:3, 3]

    return pos


# ------------- JACOBIANO NUMÉRICO -------------
def jacobian(f, x, epsilon=1e-5):
    n = len(x)
    J = np.zeros((len(f(x)), n))
    f_x = f(x)
    for i in range(n):
        dx = np.zeros(n)
        dx[i] = epsilon
        J[:, i] = (f(x + dx) - f_x) / epsilon
    return J


# ------------- MÉTODO DE NEWTON-RAPHSON -------------
#Diseñe la funcion de N-R para el metodo numerico
def NewtonRaphson(f, q0, target_pos, tol=1e-4, max_iter=100):
    q = q0
    for _ in range(max_iter):
        J = None
        try:
            J_inv = np.linalg.inv(J)
        except np.linalg.LinAlgError:
            J_inv = np.linalg.pinv(J)

        q_new = None

    raise ValueError("No se encontró solución después de %d iteraciones" % max_iter)
