"""
3D plotting examples for Modelspace

Shows how to:
 - load `results/help.csv`
 - create a 3D scatter of spacecraft ECI position (sc_eci_pos_0/1/2) with Matplotlib
 - create an interactive 3D scatter with Plotly (if installed)

Run: python3 python/examples/3d_plot_example.py
"""

import os
import pandas as pd
import numpy as np

# Matplotlib example
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# Optional: Plotly example
try:
    import plotly.express as px
    PLOTLY_AVAILABLE = True
except Exception:
    PLOTLY_AVAILABLE = False

HERE = os.path.dirname(__file__)
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
HELP_CSV = os.path.join(ROOT, 'results', 'help.csv')

if not os.path.exists(HELP_CSV):
    raise FileNotFoundError(f"Expected results file not found: {HELP_CSV}")

# Load the file
df = pd.read_csv(HELP_CSV)

# Columns from the file: sc_eci_pos_0, sc_eci_pos_1, sc_eci_pos_2
x = df['sc_eci_pos_0'].to_numpy()
y = df['sc_eci_pos_1'].to_numpy()
z = df['sc_eci_pos_2'].to_numpy()

# 1) Matplotlib 3D scatter and line (trajectory)
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z, '-o', markersize=4, label='SC trajectory')
ax.scatter(x[0], y[0], z[0], color='green', s=50, label='start')
ax.scatter(x[-1], y[-1], z[-1], color='red', s=50, label='end')
ax.set_xlabel('ECI X (m)')
ax.set_ylabel('ECI Y (m)')
ax.set_zlabel('ECI Z (m)')
ax.set_title('Spacecraft ECI Trajectory (Matplotlib 3D)')
ax.legend()
plt.tight_layout()
plt.show()

# 2) Plotly interactive scatter (optional)
if PLOTLY_AVAILABLE:
    fig2 = px.scatter_3d(df, x='sc_eci_pos_0', y='sc_eci_pos_1', z='sc_eci_pos_2', color=df.index,
                         title='Spacecraft ECI Trajectory (Plotly)', labels={'color':'sample index'})
    # connect points in order by adding a line trace
    fig2.add_scatter3d(x=x, y=y, z=z, mode='lines', line=dict(color='black', width=2), showlegend=False)
    fig2.show()
else:
    print('\nPlotly not available. To enable interactive plots, install with:')
    print('    pip install plotly')

# 3) Minimal example: create a simple 3D surface (if user needs surface plotting)
# We'll create a small parametric surface as an example
u = np.linspace(0, 2 * np.pi, 40)
v = np.linspace(0, np.pi, 20)
u, v
U, V = np.meshgrid(u, v)
R = 1.0
X = R * np.cos(U) * np.sin(V)
Y = R * np.sin(U) * np.sin(V)
Z = R * np.cos(V)

fig3 = plt.figure(figsize=(7, 5))
ax3 = fig3.add_subplot(111, projection='3d')
ax3.plot_surface(X, Y, Z, cmap='viridis', edgecolor='none', alpha=0.9)
ax3.set_title('Example 3D Surface (sphere)')
plt.tight_layout()
plt.show()
