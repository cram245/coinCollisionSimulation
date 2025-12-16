import plotly.graph_objects as go
import plotly.io as pio
import json

from simulationInitialConditions import *

def get_label_text(frame):
        return (f"<b>Temps:</b> {frame['t']:.3f} s<br>"
                f"<b>Altura centre masses:</b> {frame['z_cm']:.4f} m<br>"
                f"<b>Theta:</b> {frame['theta']:.2f} rad")


def generateFigure(frames_data):
    
    initial_frame = frames_data[0]
    initial_pos_vertices = initial_frame['pos_vertices']
    
    closed_vertices_init = np.vstack([initial_pos_vertices, initial_pos_vertices[0]])
    fig = go.Figure(
        data=[
            # La forma de la moneda (Scatter con modo lines)
            go.Scatter(
                x=closed_vertices_init[:, 0], 
                y=closed_vertices_init[:, 1], 
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
            xaxis=dict(range=[-z_0, z_0], autorange=False, title="X (m)"), # los rangos son para que no se deforme la moneda
            yaxis=dict(range=[0, 2 * z_0], autorange=False, title="Z (m)", scaleanchor="x", scaleratio=1),
            title="Simulación de Caída de Moneda",
            annotations=[{
                "text": get_label_text(initial_frame),
                "align": "left",
                "showarrow": False,
                "xref": "paper", "yref": "paper", # Usamos coordenadas de papel (0 a 1)
                "x": 0.95, "y": 0.95,             # Esquina superior derecha
                "bgcolor": "rgba(255, 255, 255, 0.8)",
                "bordercolor": "black",
                "borderwidth": 1,
                "font": {"family": "Courier New, monospace", "size": 14}
            }],
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
                        #"label": "IMPACTO" if np.isclose(frame['t'], abs_time) else f"{frame['t']:.2f} s",
                        "label": f"simInter {frame['simInter']} {frame['t']:.2f}",
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

    fig.add_hline(y=0, line_width=2, line_color="black", line_dash="solid") # linea del terra
    fig.add_hline(y=Z_STAR, line_width=2, line_color="red", line_dash="dot", 
                  annotation_text="Z*", annotation_position="top right") # linea de la Z_STAR

    frames = []
    for i, frame in enumerate(frames_data):
        closed_vertices_frame = np.vstack([frame['pos_vertices'], frame['pos_vertices'][0]])
        
        frames.append(go.Frame(
            data=[
                go.Scatter(
                    x=closed_vertices_frame[:, 0], 
                    y=closed_vertices_frame[:, 1], 
                    mode='lines'
                ),
                *[go.Scatter(
                    x=[frame['pos_vertices'][j, 0]], 
                    y=[frame['pos_vertices'][j, 1]]
                ) for j in range(4)],
                go.Scatter(
                    x=[frame['X_CM']], 
                    y=[frame['z_cm']]
                )
            ],
            layout=go.Layout(
                annotations=[{
                    "text": get_label_text(frame)
                }]
            ),
            name=f"frame_{i}"
        ))

    fig.frames = frames
    return fig

def printSimToBrowser(fig):
    pio.renderers.default = "browser"

    pio.write_json(fig, 'coin_toss_full_animation.json')
    print("Animación completa generada: coin_toss_full_animation.json")

    json_file_path = 'coin_toss_full_animation.json' 


    with open(json_file_path, 'r') as f:
        fig_json = json.load(f)

    fig = pio.from_json(json.dumps(fig_json))
    fig.show()