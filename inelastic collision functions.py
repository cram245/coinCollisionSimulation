import numpy as np

def vAfter(v, w, R, H, M , theta, GAMMA, Z_STAR, THETA_C):
    
    if (theta >= 0 and theta < np.pi/2) or (theta > -np.pi and theta < -np.pi/2):
        y = Z_STAR * np.sin(abs(theta) - THETA_C)
        
    else:
        y = -Z_STAR * np.sin(abs(theta) - THETA_C)
    
    I = (1/12) * M * (3*R**2 + H**2)
    
    v_change = -(1 + GAMMA) * (I / (I + M*y**2)) * (v + y*w)
    
    v += v_change
    return v

def wAfter(v, w, r, h, m , theta, gamma, z_star, theta_c):

    if (theta >= 0 and theta < np.pi/2) or (theta > -np.pi and theta < -np.pi/2):
        y = z_star * np.sin(abs(theta) - theta_c)

    else:
        y = -z_star * np.sin(abs(theta) - theta_c)

    I = (1/12) * m * (3*r**2 + h**2)
    
    w_change = -(1 + gamma) * (m*y / (I + m*y**2)) * (v + y*w)
    
    w += w_change
    return w

# sample:

vv = vAfter(0.5, 0.5, 12e-3, 2e-3, 7e-3, np.pi/3, 0.9, np.sqrt((12e-3)**2 + (1e-3)**2), np.arctan(2e-3 / (2*12e-3)))







