import numpy as np
import plotly.graph_objects as go
import plotly.io as pio


pio.renderers.default = "browser"

tini, tend, dt = 0.0, 100.0, 0.1
numTotalSteps = int((tend - tini) / dt) + 1
damping = 0.9
g = np.array([0.0, -9.8])
xini = np.array([[5.0, 5.0], [4.0, 5.0], [4.0, 3.0], [5.0, 3.0]]) #initial position inside the box (ordered for drawing a rectangle)
vini = np.array([[1.0, 2.0], [1.0, 2.0], [1.0, 2.0], [1.0, 2.0]]) # Apply initial velocity to all vertices


# Simulation for a rectangle's movement

x = xini.copy()  # Current positions of vertices
v = vini.copy()  # Current velocities of vertices

positions_2d = [] # To store the 2D trajectory of all vertices over time

# Define the boundaries for collision detection
x_min, x_max = 0.0, 10.0
y_min, y_max = 0.0, 10.0

# Time-stepping loop
for t_step in range(numTotalSteps):
    positions_2d.append(x.copy())

    # Update velocities (simple Euler integration with damping)
    # Apply gravity and then dampen the velocity
    v = v + g * dt
    v = v * (1 - damping * dt) # Simple linear damping based on velocity

    # Update positions
    x_new = x + v * dt

    # Collision detection and response for each vertex
    for i in range(x.shape[0]): # Iterate through each vertex
        # X-axis collision
        if x_new[i, 0] < x_min:
            x_new[i, 0] = x_min  # Snap to boundary
            v[i, 0] = -v[i, 0] * damping # Reverse and dampen velocity
        elif x_new[i, 0] > x_max:
            x_new[i, 0] = x_max  # Snap to boundary
            v[i, 0] = -v[i, 0] * damping # Reverse and dampen velocity

        # Y-axis collision
        if x_new[i, 1] < y_min:
            x_new[i, 1] = y_min  # Snap to boundary
            v[i, 1] = -v[i, 1] * damping # Reverse and dampen velocity
        elif x_new[i, 1] > y_max:
            x_new[i, 1] = y_max  # Snap to boundary
            v[i, 1] = -v[i, 1] * damping # Reverse and dampen velocity

    x = x_new

Traj_2d = np.array(positions_2d)

# Add a third dimension for Z, assuming a 2D rectangle in XY plane with Z=0
# Traj will be (numTotalSteps, num_vertices, 3) for 3D plotting
Traj = np.zeros((Traj_2d.shape[0], Traj_2d.shape[1], 3))
Traj[:, :, :2] = Traj_2d # Copy X and Y coordinates
# Z coordinates are already 0 by default

# Calculate the center of mass trajectory
# The center of mass is the average of the vertex positions
center_of_mass_trajectory = np.mean(Traj[:, :, :2], axis=1)

print(f"Simulation complete. Trajectory shape: {Traj.shape}")
print(f"Center of mass trajectory shape: {center_of_mass_trajectory.shape}")


# Create figure
fig = go.Figure(
    data=[
        go.Scatter(
            x=Traj[0, :, 0],
            y=Traj[0, :, 1],
            mode="lines",
            line=dict(width=2, color="blue"),
            fill="toself", # Fills the area inside the rectangle
            name="Rectangle"
        ),
        go.Scatter(
            x=[center_of_mass_trajectory[0, 0]],
            y=[center_of_mass_trajectory[0, 1]],
            mode="markers",
            marker=dict(size=8, color="red"),
            name="Center of Mass"
        ),
        go.Scatter(
            x=[],
            y=[],
            mode="lines",
            line=dict(width=2, color="red", dash="dash"),
            name="Center of Mass Trajectory"
        )
    ],
    layout=go.Layout(
        xaxis=dict(range=[x_min - 1, x_max + 1], autorange=False, zeroline=False),
        yaxis=dict(range=[y_min - 1, y_max + 1], autorange=False, zeroline=False),
        title_text="Falling Rectangle Simulation",
        hovermode="closest",
        updatemenus=[{
            "buttons": [
                {
                    "args": [None, {"frame": {"duration": 50, "redraw": True}, "fromcurrent": True, "transition": {"duration": 0, "easing": "quadratic-in-out"}}],
                    "label": "Play",
                    "method": "animate"
                },
                {
                    "args": [[None], {"frame": {"duration": 0, "redraw": True}, "mode": "immediate", "transition": {"duration": 0}}],
                    "label": "Pause",
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
        }]
    )
)

# Define frames for the animation
frames = []
for k in range(numTotalSteps):
    frame_data = [
        go.Scatter(
            x=Traj[k, :, 0],
            y=Traj[k, :, 1],
            mode="lines",
            line=dict(width=2, color="blue"),
            fill="toself"
        ),
        go.Scatter(
            x=[center_of_mass_trajectory[k, 0]],
            y=[center_of_mass_trajectory[k, 1]],
            mode="markers",
            marker=dict(size=8, color="red")
        ),
        go.Scatter(
            x=center_of_mass_trajectory[:k+1, 0],
            y=center_of_mass_trajectory[:k+1, 1],
            mode="lines",
            line=dict(width=2, color="red", dash="dash")
        )
    ]
    frames.append(go.Frame(data=frame_data, name=str(k)))

fig.frames = frames

fig.show()
