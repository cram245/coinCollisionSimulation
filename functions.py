from simulationInitialConditions import g, Z_STAR, cos, sin, THETA_C, np


def z(t, z_0, v_0):
    """
    Calculates the vertical position of the Center of Mass (COM).
    Equation: z(t) = z0 + v0*t - 0.5*g*t^2
    """
    return z_0 + v_0*t - 0.5*g*t**2

def v_vel(t, v_0):
    """
    Calculates the vertical velocity of the COM.
    Equation: v(t) = v0 - g*t
    NOTE: Negative sign because 'g' is defined as positive 9.8.
    """
    return v_0 - g*t

def theta_ang(t, theta_0, w):
    """
    Calculates the angular orientation of the coin.
    Equation: theta(t) = theta0 + w*t
    """
    return theta_0 + w*t


def alpha(t, j1, j2, theta_0, w):
    """
    Calculates the instantaneous phase angle for a specific corner.

    Parameters:
    -----------
    j1 : int (+1 or -1) -> Horizontal selector (Right/Left)
    j2 : int (+1 or -1) -> Vertical selector (Top/Bottom)

    Returns:
    --------
    float : The angle used inside the cosine/sine functions for position.
    """
    return THETA_C + j1*j2*theta_ang(t, theta_0, w)

def Z_j1j2(t, j1, j2, z_0, v_0, theta_0, w):
    """
    Calculates the exact vertical height (Z) of a specific corner.
    Logic: Height of COM + Vertical projection of the corner radius.
    """
    return z(t, z_0, v_0) + j2 * Z_STAR * cos(alpha(t, j1, j2, theta_0, w))

def update_state(z_curr, v_curr, theta_curr, w, dt):
    """ Helper to advance physics state by dt """
    z_new = z(dt, z_curr, v_curr)
    v_new = v_vel(dt, v_curr)
    theta_new = theta_ang(dt, theta_curr, w)
    return z_new, v_new, theta_new



def solve_collision_time(j1, j2, z_0, v_0, theta_0, w, min_t=-1e-9):
    """
    NUMERICAL SOLVER (Taylor Expansion) for Corner Impact.
    Approximates the time 'dt' until a specific corner hits Z=0.

    Objective:
    ----------
    Because Z_j1j2(t) involves cos(t) and t^2, it cannot be solved analytically.
    We use a 2nd-order Taylor expansion around t=0 (current state) to approximate
    the trajectory as a parabola locally and find the root.

    Returns:
    --------
    float or None : The time 'dt' relative to t_study, or None.
    """
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
    """
    MANAGER for Impact Detection.
    Checks all 4 corners to see which one hits the floor first.

    Returns:
    --------
    np.array : [j1, j2, absolute_time]
    """
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



def falling_impact_study_time(z_0, v_0, direction):
    """
    ANALYTICAL SOLVER for Center of Mass height.
    Finds the time 't' when the COM reaches the critical height 'z_star'.

    Objective:
    ----------
    To determine if the coin has enough energy to reach the height where it is
    safe from hitting the ground (z_star), or if it is trapped below.

    Parameters:
    -----------
    direction : int
        1 -> Search for time when passing z_star while going UP (Escape).
        0 -> Search for time when passing z_star while going DOWN (Falling).

    Returns:
    --------
    float or None : The smallest positive time 't', or None if unreachable.
    """
    # Quadratic coefficients for: -0.5*g*t^2 + v_0*t + (z_0 - z_star) = 0
    a = -0.5 * g
    b = v_0
    c = z_0 - Z_STAR

    delta = b**2 - 4*a*c

    if delta < 0:
        return None # Coin apex is lower than z_star

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


def simulate_until_impact(z_ini, v_ini, theta_ini, w_ini):
    """
    ITERATIVE LOOP to find exact impact time.

    Objective:
    ----------
    Taylor expansion is only accurate locally. This function advances the simulation
    step-by-step, recalculating the Taylor approximation as it gets closer to the floor,
    until the error is negligible (< 0.1 microseconds).
    """
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