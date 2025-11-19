import numpy as np
import scipy

z0 = 10
v0 = 0
theta0 = 0
z_star = 10
crit_angl = 0

dt = 1e-3


def v(t):
    return v0 - scipy.constants.g*t

def v_afterCol(t):
    return 0



def z(t):
    return z0 + v0*t - scipy.constants.g/2*t**2

def z_corner(j1, j2, t, alpha, w):
    return z(t) + j2 * z_star * np.cos(alpha(j1, j2, t, w))


def alpha(j1, j2, t, w):
    return crit_angl + j1*j2*theta(t, w)

def theta(t, w):
    return theta0 + w*t

# we know that if z > z* we have no collision, run the simulation until this is less

# trick to know the collision do the taylor expansion for the cos in the z_corner 
# formula, since it acts as a lower bound for the time (only 2 terms for the 
# expansion).


# returns the time in which the first possible collision can happen
def first_possibleCollision():
    return np.sqrt((z0 - z_star)/scipy.constants.g)

