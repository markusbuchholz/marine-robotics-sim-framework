#!/usr/bin/env python3
"""
Interactive 3D Cable (Catenary) Model with Sliders

- The ASV (boat) position is controlled by two sliders for X and Y (with Z fixed at 0).
- The AUV (ROV) position is controlled by three sliders (X, Y, and Z).
- A slider also controls the total cable (tether) length.
- The cable is computed in the vertical plane joining the two points using a catenary model
  that solves for parameters to satisfy the endpoint conditions and the overall cable length.

Adjust the slider ranges as needed.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D  # needed for 3D plotting
from scipy.optimize import fsolve

# ------------------------------------------------------------
# Solve for catenary parameters a and c given:
#   D  = horizontal distance between endpoints,
#   dz = vertical difference (AUV_z - ASV_z), typically negative,
#   L  = total cable length.
#
# The model uses:
#   z(x) = a * cosh((x-c)/a) - a*cosh(c/a)
#
# and we require:
#   (1)  a*(cosh((D-c)/a) - cosh(c/a)) = dz, and
#   (2)  a*(sinh((D-c)/a) + sinh(c/a)) = L.
# ------------------------------------------------------------
def solve_catenary_params(D, dz, L):
    def equations(vars):
        a, c = vars
        F1 = a * (np.cosh((D - c) / a) - np.cosh(c / a)) - dz
        F2 = a * (np.sinh((D - c) / a) + np.sinh(c / a)) - L
        return [F1, F2]
    # Try a range of initial guesses (we require c > D/2)
    guesses = []
    for a_guess in np.linspace(0.1, L/2, 15):
        for c_guess in np.linspace(D/2 + 0.01, D*2, 15):
            guesses.append((a_guess, c_guess))
    for guess in guesses:
        sol, infodict, ier, msg = fsolve(equations, guess, full_output=True)
        if ier == 1:
            a_sol, c_sol = sol
            if a_sol > 0 and c_sol > D/2:
                return sol
    raise RuntimeError("Could not solve for catenary parameters with any initial guess.")

# ------------------------------------------------------------
# Compute the 3D cable coordinates.
#
# boat_attach: [ASV_x, ASV_y, ASV_z]  (ASV_z is 0)
# auv:         [AUV_x, AUV_y, AUV_z]
# L: total cable length.
# ------------------------------------------------------------
def compute_catenary_curve(boat_attach, auv, L, num_points=1000):
    boat_attach = np.array(boat_attach, dtype=float)
    auv = np.array(auv, dtype=float)
    
    # Extract horizontal components and vertical positions:
    B_h = boat_attach[:2]   # ASV (boat) XY position
    A_h = auv[:2]           # AUV (ROV) XY position
    z_B = boat_attach[2]
    z_A = auv[2]
    
    # Horizontal distance D and unit vector U from boat to AUV:
    delta = A_h - B_h
    D = np.linalg.norm(delta)
    if D == 0:
        # If the endpoints are vertically aligned, return a straight vertical line.
        x_vals = np.full(num_points, B_h[0])
        y_vals = np.full(num_points, B_h[1])
        z_vals = np.linspace(z_B, z_A, num_points)
        return np.column_stack((x_vals, y_vals, z_vals))
    U = delta / D
    
    # Vertical difference:
    dz = z_A - z_B  # (typically negative if AUV is below the boat)
    
    # Solve for catenary parameters a and c:
    a, c = solve_catenary_params(D, dz, L)
    
    # Local horizontal coordinate along the cable from 0 to D:
    x_local = np.linspace(0, D, num_points)
    # Compute the vertical profile:
    z_local = a * np.cosh((x_local - c) / a) - a * np.cosh(c / a)
    
    # Build 3D coordinates:
    horiz_coords = boat_attach[:2] + np.outer(x_local, U)
    z_coords = z_B + z_local
    coords = np.column_stack((horiz_coords, z_coords))
    return coords

# ------------------------------------------------------------
# Set up the interactive figure with sliders.
# ------------------------------------------------------------
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.1, bottom=0.35)  # leave space for the sliders

# Initial values:
ASV_x_init = 0.0
ASV_y_init = 0.0
ASV_z = 0.0  # fixed
AUV_x_init = 0.0
AUV_y_init = 0.0
AUV_z_init = -5.0
L_init = 15.0

# Compute the initial cable coordinates:
boat_attach = [ASV_x_init, ASV_y_init, ASV_z]
auv = [AUV_x_init, AUV_y_init, AUV_z_init]
try:
    cable_coords = compute_catenary_curve(boat_attach, auv, L_init, num_points=1000)
except Exception as e:
    print("Error computing cable:", e)
    cable_coords = np.zeros((1000, 3))

# Plot the cable and endpoint markers:
cable_line, = ax.plot(cable_coords[:,0], cable_coords[:,1], cable_coords[:,2],
                      'b-', lw=2, label='Cable')
asv_marker = ax.scatter(boat_attach[0], boat_attach[1], boat_attach[2],
                        c='blue', marker='x', s=100, label='ASV')
auv_marker = ax.scatter(auv[0], auv[1], auv[2],
                        c='red', marker='o', s=100, label='AUV')
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.legend()

# Create slider axes (positioned below the 3D plot):
axcolor = 'lightgoldenrodyellow'
ax_asv_x = plt.axes([0.1, 0.25, 0.8, 0.03], facecolor=axcolor)
ax_asv_y = plt.axes([0.1, 0.20, 0.8, 0.03], facecolor=axcolor)
ax_auv_x = plt.axes([0.1, 0.15, 0.8, 0.03], facecolor=axcolor)
ax_auv_y = plt.axes([0.1, 0.10, 0.8, 0.03], facecolor=axcolor)
ax_auv_z = plt.axes([0.1, 0.05, 0.8, 0.03], facecolor=axcolor)
ax_L     = plt.axes([0.1, 0.00, 0.8, 0.03], facecolor=axcolor)

slider_asv_x = Slider(ax_asv_x, 'ASV X', -10.0, 10.0, valinit=ASV_x_init)
slider_asv_y = Slider(ax_asv_y, 'ASV Y', -10.0, 10.0, valinit=ASV_y_init)
slider_auv_x = Slider(ax_auv_x, 'AUV X', -10.0, 10.0, valinit=AUV_x_init)
slider_auv_y = Slider(ax_auv_y, 'AUV Y', -10.0, 10.0, valinit=AUV_y_init)
slider_auv_z = Slider(ax_auv_z, 'AUV Z', -20.0, -1.0, valinit=AUV_z_init)
slider_L     = Slider(ax_L,     'Cable L', 5.0, 30.0, valinit=L_init)

def update(val):
    # Get current slider values:
    asv_x = slider_asv_x.val
    asv_y = slider_asv_y.val
    auv_x = slider_auv_x.val
    auv_y = slider_auv_y.val
    auv_z = slider_auv_z.val
    L = slider_L.val

    # Define new positions:
    boat_attach = [asv_x, asv_y, ASV_z]
    auv = [auv_x, auv_y, auv_z]
    
    try:
        # Compute the updated cable curve:
        new_coords = compute_catenary_curve(boat_attach, auv, L, num_points=1000)
        # Update the cable line:
        cable_line.set_xdata(new_coords[:,0])
        cable_line.set_ydata(new_coords[:,1])
        cable_line.set_3d_properties(new_coords[:,2])
    except Exception as e:
        print("Error updating cable:", e)
        return

    # Update the markers by clearing and replotting (for simplicity):
    ax.collections.clear()  # remove previous scatter plots
    ax.scatter(boat_attach[0], boat_attach[1], boat_attach[2],
               c='blue', marker='x', s=100, label='ASV')
    ax.scatter(auv[0], auv[1], auv[2],
               c='red', marker='o', s=100, label='AUV')
    
    fig.canvas.draw_idle()

# Attach the update function to each slider:
slider_asv_x.on_changed(update)
slider_asv_y.on_changed(update)
slider_auv_x.on_changed(update)
slider_auv_y.on_changed(update)
slider_auv_z.on_changed(update)
slider_L.on_changed(update)

plt.show()
