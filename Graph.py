import numpy as np
import plotly.graph_objects as go
import plotly.io as pio
import json

from simulationInitialConditions import *
from functions import *

dt = 0.05
T_max = 6
# vamos a actualizar la z0, la v0 y la theta0 cada vez que se produzca un impacto




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


def simulateFallUntil(z_0, v_0, theta_0, w, t_start, t_end, frames_data, hasCollisionAtEnd):
    time_points = np.arange(0, t_end, dt)

    if (hasCollisionAtEnd):
        if time_points[-1] != t_end:
            time_points = np.append(time_points, t_end) # asi tendremos el intervalo de tiempo en que choca entre nuestros time_points
    
    for t in time_points:
        z_cm = z_0 + v_0 * t - 0.5 * g * t**2
        theta = theta_0 + w * t
    
        pos_vertices = get_coin_vertices(np.array([X_CM, z_cm]), theta)

        frame = {
            't': t + t_start,
            'pos_vertices': pos_vertices,
            'z_cm': z_cm,
            'theta': theta,
            'X_CM': x_0
        }

        frames_data.append(frame)



X_CM = 0 # this is going to be constant
frames_data = []

direction = 0 if z_0 > Z_STAR else 1
t_fall = falling_impact_study_time(z_0, v_0, direction)
t_total = 0
if t_fall:
    simulateFallUntil(z_0, v_0, theta_0, w, 0, t_fall, frames_data, False)
    z_0, v_0, theta_0 = update_state(z_0, v_0, theta_0, w, t_fall)
    t_total += t_fall
else:
    print("On no, alguna cosa ha fallat!")

impacte = simulate_until_impact(z_0, v_0, theta_0, w)
if impacte is None:
    print("Hem parat de rebotar")
else:
    simulateFallUntil(z_0, v_0, theta_0, w, t_total, impacte[2], frames_data, True)


abs_time = 0


initial_frame = frames_data[0]
initial_pos_vertices = initial_frame['pos_vertices']

print(frames_data[-1])


fig = go.Figure(
    data=[
        # La forma de la moneda (Scatter con modo lines)
        go.Scatter(
            x=initial_pos_vertices[:, 0], 
            y=initial_pos_vertices[:, 1], 
            mode='lines', 
            line=dict(color='blue', width=4), 
            fill='toself', 
            fillcolor='lightblue', 
            name='Moneda'
        ),
        *[go.Scatter(
            # Usamos solo los 4 primeros vértices para los puntos
            x=[initial_pos_vertices[i, 0]], 
            y=[initial_pos_vertices[i, 1]], 
            mode='markers', 
            marker=dict(color='darkgreen', size=7, symbol='circle'), 
            name='Vértices',
            hovertemplate=f'{NOMBRES_VERTICES[i]}<br>(%{{x:.4f}}, %{{y:.4f}})<extra></extra>'
        ) for i in range(4)],
        # El centro de masa
        go.Scatter(
            x=[initial_frame['X_CM']], 
            y=[initial_frame['z_cm']], 
            mode='markers', 
            marker=dict(color='red', size=8), 
            name='CM'
        )
    ],
    layout=go.Layout(
        xaxis=dict(range=[-z_0, z_0], autorange=False, title="Posición X (m)"), # los rangos son para que no se deforme la moneda
        yaxis=dict(range=[0, 2 * z_0], autorange=False, title="Posición Z (m)", scaleanchor="x", scaleratio=1),
        title="Simulación de Caída de Moneda con Rotación",
        updatemenus=[{
            "buttons": [
                {
                    "args": [None, {"frame": {"duration": dt*1000, "redraw": True}, "fromcurrent": True}],
                    "label": "Play",
                    "method": "animate"
                }
            ],
            "direction": "left",
            "pad": {"r": 10, "t": 87},
            "showactive": False,
            "type": "buttons",
            "x": 0.1,
            "xanchor": "right",
            "y": 0,
            "yanchor": "top"
        }],
        # Slider para la animación
        sliders=[{
            "steps": [
                {
                    "args": [[f"frame_{i}"], {"frame": {"duration": dt*1000, "redraw": True}, "mode": "immediate"}],
                    "label": "IMPACTO" if np.isclose(frame['t'], abs_time) else f"{frame['t']:.2f} s",
                    "method": "animate"
                } for i, frame in enumerate(frames_data)
            ],
            "transition": {"duration": 0},
            "x": 0.1,
            "y": 0,
            "len": 0.9,
            "active": 0
        }]
    )
)

fig.frames = [
    go.Frame(
        data=[
            # Data de la moneda (líneas y relleno)
            go.Scatter(
                x=frame['pos_vertices'][:, 0], 
                y=frame['pos_vertices'][:, 1], 
                mode='lines', 
                line=dict(color='blue', width=4), 
                fill='toself', 
                fillcolor='lightblue'
            ),
            *[go.Scatter(
                x=[frame['pos_vertices'][i, 0]], 
                y=[frame['pos_vertices'][i, 1]], 
                mode='markers', 
                marker=dict(color='darkgreen', size=7, symbol='circle'), 
                name='Vértices',
                hovertemplate=f'{NOMBRES_VERTICES[i]}<br>(%{{x:.4f}}, %{{y:.4f}})<extra></extra>'
            ) for i in range(4)],
            # Data del CM (punto rojo)
            go.Scatter(
                x=[frame['X_CM']], 
                y=[frame['z_cm']], 
                mode='markers', 
                marker=dict(color='red', size=8)
            )
        ],
        name=f"frame_{i}"
    ) for i, frame in enumerate(frames_data)
]


pio.write_json(fig, 'coin_toss_full_animation.json')
print("Animación completa generada: coin_toss_full_animation.json")

json_file_path = 'coin_toss_full_animation.json' 


with open(json_file_path, 'r') as f:
    fig_json = json.load(f)

fig = pio.from_json(json.dumps(fig_json))
fig.show()