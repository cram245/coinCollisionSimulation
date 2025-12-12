from simulationInitialConditions import *


def z(t, z_0, v_0):
    
    return z_0 + v_0*t - 0.5*g*t**2


def v_vel(t, v_0):
    
    return v_0 - g*t


def theta_ang(t, theta_0, w):
    
    return theta_0 + w*t


def alpha(t, j1, j2, theta_0, w):
    
    return THETA_C + j1*j2*theta_ang(t, theta_0, w)


def Z_j1j2(t, j1, j2, z_0, v_0, theta_0, w):
    
    return z(t, z_0, v_0) + j2 * Z_STAR * cos(alpha(t, j1, j2, theta_0, w))


def norm_angle(angle):
    # 1. Angle to [0, 2pi)
    angle_norm = angle % (2 * pi)

    # 2. Angle to (-pi, pi)
    if angle_norm > pi:
        angle_norm -= 2 * pi

    return angle_norm

def energy(z, v, w):
  return m*g*z + 0.5*m*v**2 + 0.5*I_MOMENT*w**2


def slide_until_cusp(z_curr, v_curr, theta_curr, w_curr, j1, j2):

    # --- 1. IDENTIFICAR L'OBJECTIU (CUSP) ---
    # "When the coin reaches a cusp... which can occur at theta = n(pi/2)"
    pi_2 = np.pi / 2

    # Determinem la direcció i el següent múltiple de 90 graus
    if w_curr > 0:
        target_k = np.floor(theta_curr / pi_2) + 1
    else:
        target_k = np.ceil(theta_curr / pi_2) - 1

    theta_new = target_k * pi_2

    # Correcció: Si ja som a la cúspide (per error numèric), forcem avançar a la següent
    if abs(theta_new - theta_curr) < 1e-5:
        theta_new += (pi_2 if w_curr > 0 else -pi_2)

    # --- 2. NOVA POSICIÓ (GEOMETRIA) ---
    # "The coin can move continuously along this curve"
    # Imosem la restricció Z_corner = 0
    alpha_new = alpha(0, j1, j2, theta_new, 0)
    z_new = -1 * j2 * Z_STAR * np.cos(alpha_new)

    if z_new < 0: z_new = 0.0 # Clamp de seguretat

    # --- 3. CONSERVACIÓ D'ENERGIA ---
    # "No loss of energy... depends on kinetic energy available"

    # Energia Total Inicial (Conservada)
    E_total = m * g * z_curr + 0.5 * m * v_curr**2 + 0.5 * I_MOMENT * w_curr**2

    # Factor geomètric derivat de la restricció v = f(w)
    # v = dz/dt = d/dt (-j2*z*cos(alpha)) = j2*z*sin(alpha) * (j1*j2*w) = j1*z*sin(alpha)*w
    geo_factor = j1 * Z_STAR * np.sin(alpha_new)

    # Nova Energia Potencial
    U_new = m * g * z_new

    # Energia Cinètica disponible
    K_available = E_total - U_new

    if K_available < 0:
        # "The coin cannot reverse direction... along the collision line"
        # Si no té energia per arribar-hi, es queda on és (o cau enrere, però aquí parem).
        return z_curr, 0.0, theta_curr, 0.0

    # Resoldre w_new
    # K = 0.5 * m * v^2 + 0.5 * I * w^2
    # K = 0.5 * w^2 * (m * geo_factor^2 + I)
    inertia_effective = (m * (geo_factor**2) + I_MOMENT)
    w_magnitude = np.sqrt(2 * K_available / inertia_effective)

    w_new = w_magnitude if w_curr > 0 else -w_magnitude
    v_new = geo_factor * w_new

    # --- 4. TRANSICIÓ (IMPULS) ---
    # "Continue the motion as if the coin is colliding with the floor with contact
    # only at the corner that is coming into new contact".

    # IMPORTANT: A la cúspide (pi/2), la moneda està PLANA.
    # Tècnicament, entra en contacte la 'nova' cantonada.
    # Però com que Z=0 per a totes dues, podem usar la funció de xoc estàndard
    # sobre l'estat actual i la física resoldrà la discontinuïtat de velocitat.

    v_final, w_final = resolve_bounce(v_new, w_new, theta_new, j1, j2)

    return z_new, v_final, theta_new, w_final



def update_state(z_curr, v_curr, THETA_Curr, w, dt):
    z_new = z(dt, z_curr, v_curr)
    v_new = v_vel(dt, v_curr)
    theta_new = theta_ang(dt, THETA_Curr, w)
    return z_new, v_new, theta_new


def falling_impact_study_time(z_0, v_0, direction):
    
    # Quadratic coefficients for: -0.5*g*t^2 + v_0*t + (z_0 - Z_STAR) = 0
    a = -0.5 * g
    b = v_0
    c = z_0 - Z_STAR

    delta = b**2 - 4*a*c

    if delta < 0:
        return None # Coin apex is lower than Z_STAR

    sqrt_delta = np.sqrt(delta)
    t1 = (-b - sqrt_delta) / (2*a)
    t2 = (-b + sqrt_delta) / (2*a)

    candidates = [t1, t2]

    # Find the first valid time matching the direction criteria
    for t in candidates:
        if t > 0: # Only future events
            current_vel = v_vel(t, v_0)
            if direction == 1 and current_vel > 0:
                return t
            elif direction == 0 and current_vel < 0:
                return t

    return None


def solve_collision_time(j1, j2, z_0, v_0, theta_0, w, min_t=-1e-9):
    
    # 1. Evaluate state at t_study
    z_com = z(0, z_0, v_0)
    v_com = v_vel(0, v_0)
    alpha_val = alpha(0, j1, j2, theta_0, w)

    cos_a = cos(alpha_val)
    sin_a = sin(alpha_val)

    # 2. Taylor Coefficients (Z(tau) ≈ A*tau^2 + B*tau + C)
    # C = Current Position
    C_coeff = z_com + j2 * Z_STAR * cos_a

    # B = Current Velocity (Linear + Rotational contribution)
    B_coeff = v_com - (j1 * Z_STAR * w * sin_a)

    # A = Current Acceleration (Gravity + Centripetal) / 2
    accel_total = -g - (j2 * Z_STAR * (w**2) * cos_a)
    A_coeff = 0.5 * accel_total

    # 3. Solve Quadratic
    delta = B_coeff**2 - 4*A_coeff*C_coeff
    if delta < 0: return None

    sqrt_delta = np.sqrt(delta)
    t1 = (-B_coeff - sqrt_delta) / (2*A_coeff)
    t2 = (-B_coeff + sqrt_delta) / (2*A_coeff)

    # 4. Filter Results (absolute time must be > t_study)
    candidates = []
    for t in [t1, t2]:
        if t > min_t:
            candidates.append(t)

    if not candidates: return None
    return min(candidates)


def find_next_impact(z_0, v_0, theta_0, w, min_t=-1e-9):
    
    corners = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
    valid_impacts = []

    for (j1, j2) in corners:
        dt = solve_collision_time(j1, j2, z_0, v_0, theta_0, w, min_t)
        if dt is not None:
            valid_impacts.append([j1, j2, dt])

    if not valid_impacts: return None

    # Sort by time (smallest time is the first impact)
    valid_impacts.sort(key=lambda x: x[2])
    return np.array(valid_impacts[0])


def simulate_until_impact(z_ini, v_ini, theta_ini, w_ini):
  
    curr_z = z_ini
    curr_v = v_ini
    curr_theta = theta_ini
    total_time = 0.0

    precision_limit = 1e-12
    max_steps = 50

    for i in range(max_steps):
        # Search from t=0 (current state)
        impact = find_next_impact(curr_z, curr_v, curr_theta, w_ini)

        if impact is None: return None # No impact found

        j1, j2, dt = impact

        curr_z, curr_v, curr_theta = update_state(curr_z, curr_v, curr_theta, w_ini, dt)
        total_time += dt

        # Check convergence
        if dt < precision_limit:
            return [int(j1), int(j2), total_time, curr_z, curr_v, curr_theta]

    return None


def resolve_bounce(v_old, w_old, theta_impact, j1, j2):

    # 1. Local Variables
    v = v_old
    w = w_old

    # IMPORTANT: Normalize theta to range [-PI, PI]
    # We use the existing helper function for consistency
    theta = norm_angle(theta_impact)

    # 2. CALCULATION OF 'y' (Lever Arm)
    # Determines the horizontal distance to contact point based on angle quadrants
    if (theta >= 0 and theta < np.pi/2) or (theta > -np.pi and theta < -np.pi/2):
        y = Z_STAR * np.sin(abs(theta) - THETA_C)
    else:
        y = -Z_STAR * np.sin(abs(theta) - THETA_C)

    # 3. MOMENT OF INERTIA (Class Formula)
    # Formula for a solid cylinder rotating around central diameter
    I = I_MOMENT

    # 4. CALCULATION OF VELOCITY CHANGES (Class Formulas)
    # Calculating the impulse effects on linear (v) and angular (w) velocity
    v_change = -(1 + gamma) * (I / (I + m*y**2)) * (v + y*w)
    w_change = -(1 + gamma) * (m*y / (I + m*y**2)) * (v + y*w)

    # 5. UPDATE AND RETURN
    v += v_change
    w += w_change

    return v, w


def check_next_event(z_curr, v_curr, THETA_Curr, w_curr):

    # A. Check time to next Impact
    impact_data = find_next_impact(z_curr, v_curr, THETA_Curr, w_curr, min_t=1e-5)

    if impact_data is not None:
        dt_impact = impact_data[2]
        impact_details = (int(impact_data[0]), int(impact_data[1]))
    else:
        dt_impact = float('inf')
        impact_details = None

    # B. Check time to Escape (reach Z_STAR)
    # We look for direction=1 (Rising)
    if z_curr > Z_STAR:
        dt_escape = 0.0 # Already escaped
    else:
        dt_escape = falling_impact_study_time(z_curr, v_curr, direction=1)
        if dt_escape is None: dt_escape = float('inf')

    # C. Compare
    if dt_impact == float('inf') and dt_escape == float('inf'):
        return {"type": "IMPACT", "dt": 0.001, "details": None}

    if dt_impact < dt_escape:
        return {"type": "IMPACT", "dt": dt_impact, "details": impact_details}
    else:
        return {"type": "ESCAPE", "dt": dt_escape, "details": None}


        
def determine_outcome_by_angle(theta_final, THETA_C):

    # 1. Normalize angle to [-pi, pi]
    # We use the helper function defined previously
    angle = norm_angle(theta_final)

    # Now 'angle' is always between -3.14 and 3.14

    # 2. REGION DEFINITION

    # --- EDGE ZONES (Verticals) ---
    # The coin is on its edge when vertical (angle 0 or angle pi/-pi)

    # Case 1: Around 0 radians
    if -THETA_C <= angle <= THETA_C:
        return "CANTÓ (Edge)"

    # Case 2: Around +/- pi radians (the ends of the interval)
    # Checks if we are close to pi or close to -pi
    elif (angle >= np.pi - THETA_C) or (angle <= -np.pi + THETA_C):
        return "CANTÓ (Edge)"

    # --- FACE ZONES (Flats) ---
    # If it is not Edge, it must be Heads or Tails.
    # These correspond to the horizontal positions around +/- pi/2.

    # Case Heads/Tails 1 (Positive half, around pi/2)
    elif 0 < angle < np.pi:
        return "CARA (Heads)"

    # Case Heads/Tails 2 (Negative half, around -pi/2)
    else:
        return "CREU (Tails)"

# Wrapper for compatibility with the main simulation loop
def determine_final_outcome(z_f, v_f, theta_f, w_f):
    return determine_outcome_by_angle(theta_f, THETA_C)


    
