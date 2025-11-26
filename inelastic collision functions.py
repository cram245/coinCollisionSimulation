import numpy as np

def vAfter(v, w, r, h, m , gamma, z_star):
    y = z_star # because unbiased, any colliding corner is equidistant from the cm
    
    I = np.zeros(3)
    I[0] = (1/12) * m * (3*r**2 + h**2)
    I[1] = I[0]
    I[2] = (1/2) * m * r**2
    
    v_change = -(1 + gamma) * (I / (I + m*y**2)) * (v + y*w)
    
    v += v_change
    return v

def wAfter(v, w, r, h, m , gamma, z_star):
    y = z_star # because unbiased, any colliding corner is equidistant from the cm
    
    I = np.zeros(3)
    I[0] = (1/12) * m * (3*r**2 + h**2)
    I[1] = I[0]
    I[2] = (1/2) * m * r**2
    
    w_change = -(1 + gamma) * (m*y / (I + m*y**2)) * (v + y*w)
    
    w += w_change
    return w

# sample:
vv = vAfter([0, 0, 0.5], 0.5, 12e-3, 2e-3, 7e-3 , 0.3, np.sqrt((12e-3)**2 + (1e-3)**2))