import numpy as np


from simulationInitialConditions import *
from ImplementationNB import *
from figureCreation import *




# obtener la posicion de los vertices a partir de la posicion del centro de massas y de el angulo con la vertical
def get_coin_vertices(POS_CM, theta):
    
    theta = theta + (np.pi/2) # los calculos se hacen con el angulo con la horizontal

    local_verts = np.array([
        [r, h/2],    # -> ( 1, -1)
        [-r, h/2],   # -> ( 1,  1)
        [-r, -h/2],  # -> ( 1, -1)
        [r, -h/2]    # -> (-1, -1)
    ])
    
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    rotated_verts = local_verts @ R

    world_verts = rotated_verts + POS_CM
    
    return world_verts


def simulateFallUntil(z_0, v_0, theta_0, w, t_start, t_end, frames_data, currSimInter, hasCollisionAtEnd):
    time_points = np.arange(0, t_end - dt, dt) # l'ultim temps te major precisio que 1e-3 aixi que el tractem per separat

    if t_end < 1e-5:
        return
    
    print()
    print(f"Abans de començar a simular el interval {currSimInter}, z_cm = {z_0}, theta = {theta_0}")
    
    #if hasCollisionAtEnd:
        #if time_points[-1] != t_end:
            #time_points = np.append(time_points, t_end) # asi tendremos el intervalo de tiempo en que choca entre nuestros time_points
    
     
    for t in time_points:
        z_cm = z(t, z_0, v_0)
        theta = theta_ang(t, theta_0, w)
    
        pos_vertices = get_coin_vertices(np.array([X_CM, z_cm]), theta)

        frame = {
            't': t + t_start,
            'pos_vertices': pos_vertices,
            'z_cm': z_cm,
            'theta': theta,
            'X_CM': x_0,
            'simInter': currSimInter
        }

        frames_data.append(frame)

    z_cm = z(t_end, z_0, v_0)
    theta = theta_ang(t_end, theta_0, w)
    pos_vertices = get_coin_vertices(np.array([X_CM, z_cm]), theta)

    frame = {
        't': t_end + t_start,
        'pos_vertices': pos_vertices,
        'z_cm': z_cm,
        'theta': theta,
        'X_CM': x_0,
        'simInter': currSimInter
    }

    frames_data.append(frame)


    # ara en fem una extra pero nomes per l'ultim temps
    
    print(f"Al acabar el interval de simulacio {currSimInter}, z_cm = {z_cm}, theta = {theta}, t_final = {t_start + t_end}")
    
    


currSimInter = 0
z_curr, v_curr, theta_curr, w_curr = z_0, v_0, theta_0, w
t_total = 0
bounce_count = 0

frames_data = []
simulating = True
bounces = 0

while simulating and bounce_count < max_bounces and t_total < T_max:

    E_total = energy(z_curr, v_curr, w_curr)
    if E_total < E_C:
        print("ja estem")
        simulating = False



    if z_curr > Z_STAR:
        direction = 0
        t_fall = falling_impact_study_time(z_curr, v_curr, direction)
    
        if t_fall:
            currSimInter += 1
            simulateFallUntil(z_curr, v_curr, theta_curr, w_curr, t_total, t_fall, frames_data, currSimInter, False)
            
            
            z_curr, v_curr, theta_curr = update_state(z_curr, v_curr, theta_curr, w_curr, t_fall)
            print(f"Segons el update state despres de el interval {currSimInter}, z_cm = {z_curr}, theta = {theta_curr}")

            t_total += t_fall
        else:
            print("ERROR! On no, alguna cosa ha fallat!")
            simulating = False
            break


    elif v_curr > 0:
        
        next_ev = check_next_event(z_curr, v_curr, theta_curr, w_curr)
        
        if next_ev['type'] == 'ESCAPE':
            
            t_fall = falling_impact_study_time(z_curr, v_curr, direction=0)
            if t_fall:
                currSimInter += 1
                simulateFallUntil(z_curr, v_curr, theta_curr, w_curr, t_total, t_fall, frames_data, currSimInter, False)
                z_curr, v_curr, theta_curr = update_state(z_curr, v_curr, theta_curr, w_curr, t_fall)
                print(f"Segons el update state despres de el interval {currSimInter}, z_cm = {z_curr}, theta = {theta_curr}")
                t_total += t_fall
            else:
                print("Error, the coin doesn't reach Z* never (ERROR)")
                outcome = "ERROR"
                simulating = False
                break

    
    impacte = simulate_until_impact(z_curr, v_curr, theta_curr, w)
    if impacte is None:
        z_curr, v_curr, theta_curr = update_state(z_curr, v_curr, theta_curr, w_curr, 0.01)
        print("No impact!")
        continue
    
    currSimInter += 1
    simulateFallUntil(z_curr, v_curr, theta_curr, w_curr, t_total, impacte[2], frames_data, currSimInter, True)
    
    j1, j2 = impacte[0], impacte[1]
    t_recorregut = impacte[2]
    z_curr = impacte[3]
    v_curr = impacte[4]
    theta_curr = impacte[5]

    t_total += t_recorregut
    bounce_count += 1
    print(f"Segons el update state despres de el interval {currSimInter}, z_cm = {z_curr}, theta = {theta_curr}")
    print("Colision al segundo: ", t_total)

    v_curr, w_curr = resolve_bounce(v_curr, w_curr, theta_curr, j1, j2)
    E_total = energy(z_curr, v_curr, w_curr)
    
    if E_total < E_C:
        simulating = False
        z_final, v_final, theta_final, w_final = slide_until_cusp(z_curr, v_curr, theta_curr, w_curr, j1, j2)

        # Actualitzem l'estat final per al plot
        z_curr = z_final
        v_curr = 0.0 # Ja està morta
        w_curr = 0.0
        theta_curr = theta_final

        break

    # At this point the coin has to bounce but we have so many opcions:
    #   A. The coin reaches Z* going up before any corner collides ->
    #      -> we move to the moment that reaches z* but going down
    #   B. A diferent corner collides before the coin reaches Z*
    #   C. The same corner collides (because it's slideing)

    # A and B are solved if we continue the while, so we just have to solve the C

    # ---------------------------------------------------
    # D) WHAT HAPPENS NEXT?
    # ---------------------------------------------------

    # If the coin still going down, the coin is slideing
    if v_curr <= 0:
      print("After collision the velocity is negative, so the coin slides")
      sliding = True

    # If the coin is going up we need to see if the next event is anothe collision with the same corner, fast and weak
    else:
        print("After collision the velocity is positive")
        next_ev = check_next_event(z_curr, v_curr, theta_curr, w_curr)

        sliding = False

        if next_ev['type'] == 'ERROR':
            print("It happened something bad")
            outcome = "ERROR"
            break

        if next_ev['type'] == 'IMPACT':
            # The coin is going to collide before escape
            print("The coin will collide before escape")
            dt2 = next_ev['dt']
            j1_next, j2_next = next_ev['details'][0], next_ev['details'][1]

            # SLIDING CONDITIONS
            pivoting = (j1_next == j1 and j2_next == j2)
            fast = (dt2 < 1e-8)
            weak = (abs(v_curr) < 0.5)

            if pivoting and fast and weak:
              # If this happens the coin will slide
              print("The coin is sliding")
              sliding = True

            else:
                # The coin won't slide, but we now that is going to collide before escape
                print("The coin is not sliding")
                # We move a little bit to the next collision (with this we ensure the coin is not touching the floor)
              
                simulateFallUntil(z_curr, v_curr, theta_curr, w_curr, t_total, dt2/2, frames_data, False)
                z_curr, v_curr, theta_curr = update_state(z_curr, v_curr, theta_curr, w_curr, dt2/2)
                t_total += dt2/2

                continue

        elif next_ev['type'] == 'ESCAPE':
            # The coin is going to escape before colliding
            print("The coin will escape to a height > Z*")
            dt2 = next_ev['dt']

            currSimInter += 1
            simulateFallUntil(z_curr, v_curr, theta_curr, w_curr, t_total, dt2+1e-5, frames_data, currSimInter, False)
            z_curr, v_curr, theta_curr = update_state(z_curr, v_curr, theta_curr, w_curr, dt2+1e-5)
            print(f"Segons el update state despres de el interval {currSimInter}, z_cm = {z_curr}, theta = {theta_curr}")
            t_total += dt2/2 + 1e-5
            continue

    if sliding:
        print(f"   [SLIDING START] Theta: {theta_curr:.2f}")

        # 1. CAPTURAR EL RESULTAT (Correcte: Ara recollim els 4 valors que retorna)
        # Nota: v_new i w_new ja inclouen el 'kick' de resolve_bounce del final de la funció.
        z_new, v_new, theta_new, w_new = slide_until_cusp(z_curr, v_curr, theta_curr, w_curr, j1, j2)

        # 2. CALCULAR EL TEMPS DE LLISCAMENT (Física: t = distància / velocitat)
        # La funció slide mou la moneda, però no ens diu quant triga. Ho estimem:
        # Usem la velocitat angular mitjana entre l'inici i el final del lliscament.
        # (Nota: w_curr és l'inicial, w_new és la final post-xoc.
        #  Per ser més precisos amb el temps de lliscament, idealment voldríem la w just abans del xoc,
        #  però la mitjana és una aproximació suficientment bona per a simulació estocàstica).

        w_avg = (w_curr + w_new) / 2.0

        # Evitem dividir per zero
        if abs(w_avg) > 1e-9:
            dt_slide = abs(theta_new - theta_curr) / abs(w_avg)
        else:
            dt_slide = 0.0

        print("Time spent sliding: ", dt_slide)
        currSimInter += 1
        simulateFallUntil(z_curr, v_curr, theta_curr, w_curr, t_total, dt_slide, frames_data, currSimInter, False)

        # 3. ACTUALITZAR L'ESTAT PRINCIPAL
        # Això és el que faltava: traspassar els valors nous a les variables del bucle
        z_curr = z_new
        v_curr = v_new
        theta_curr = theta_new
        w_curr = w_new

        # 4. ACTUALITZAR COMPTADORS
        t_total += dt_slide
        bounce_count += 1 # El 'kick' final compta com un rebot tècnic
        #print(f"Now, at time {t_total:.4f}s the coin has finished slideing")
        #print(f"The time between we started studying the collision and it happened was {dt_slide:.4f}")
        #print(f"BOUNCE {bounce_count}: v_impact={v_curr:.6f} | w_impact={w_curr:.6f}")
        #print(f"   [SLIDING END] Theta: {theta_curr:.2f}")
    
    print("En acabar bucle aquest es el temps total: ", t_total)




abs_time = 0
fig = generateFigure(frames_data, abs_time)
printSimToBrowser(fig)




