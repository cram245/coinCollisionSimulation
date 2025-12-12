import numpy as np

g = 9.8
dt = 0.05
T_max = 6
max_bounces = 50

v_0 = 6.3
x_0 = 0
z_0 = 1.7
theta_0 = np.pi

r = 1.1
h = 0.2
m = 2
w = 15
gamma = 0.7


Z_STAR = np.sqrt(r**2 + (h/2)**2)
X_CM = 0
THETA_C = np.arctan(h / (2 * r))
I_MOMENT = (m / 4) * r**2 + (m / 12) * h**2
E_C = m * g * Z_STAR


cos = np.cos
sin = np.sin
pi = np.pi

NOMBRES_VERTICES = ["( 1, -1)", "( 1,  1)", "( 1, -1)", "(-1, -1)"]