#!/usr/bin/env python3
"""
Catenary cable model in 3D with slack, solving for the unique catenary that:
  - Connects a boat attachment point to an ROV point,
  - Has a specified total cable (tether) length L.
  
The local (2D, vertical plane) cable equation is:
    z(x) = a*cosh((x-c)/a) - a*cosh(c/a)
with x running from 0 (boat attachment) to D (horizontal distance between endpoints).

We solve for parameters a and c such that:
    (1) a*(cosh((D-c)/a) - cosh(c/a)) = Δz   and
    (2) a*(sinh((D-c)/a) + sinh(c/a)) = L,
with Δz = (z_R - z_B) (typically negative if the ROV is below the boat).
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve
from mpl_toolkits.mplot3d import Axes3D

def solve_catenary_params(D, dz, L):
    """
    Given:
      D  -- horizontal distance between endpoints,
      dz -- vertical difference (z_R - z_B), typically negative,
      L  -- total cable length (must be > minimal taut length).
      
    Solve for catenary parameters a and c in:
       F1(a,c) = a*(cosh((D-c)/a) - cosh(c/a)) - dz = 0,
       F2(a,c) = a*(sinh((D-c)/a) + sinh(c/a)) - L  = 0.
       
    Because convergence may depend on the initial guess, we try a range of guesses.
    (We require c > D/2 so that F1 can be negative when dz < 0.)
    """
    def equations(vars):
        a, c = vars
        F1 = a * (np.cosh((D - c) / a) - np.cosh(c / a)) - dz
        F2 = a * (np.sinh((D - c) / a) + np.sinh(c / a)) - L
        return [F1, F2]
    
    guesses = []
    # Try a range for a_guess (from a small value up to, say, L/2) and for c_guess (c > D/2)
    for a_guess in np.linspace(0.1, L/2, 15):
        for c_guess in np.linspace(D/2 + 0.01, D*2, 15):
            guesses.append((a_guess, c_guess))
    
    for guess in guesses:
        sol, infodict, ier, msg = fsolve(equations, guess, full_output=True)
        if ier == 1:
            # Check that the solution makes sense:
            a_sol, c_sol = sol
            # We require a > 0 and c > D/2.
            if a_sol > 0 and c_sol > D/2:
                return sol
    raise RuntimeError("Could not solve for catenary parameters with any initial guess.")

def compute_catenary_curve(boat_attach, rov, L, num_points=1000):
    """
    Compute the 3D coordinates for the cable curve that:
      - Attaches at boat_attach (a 3D point),
      - Ends at rov (a 3D point),
      - Has total length L.
      
    The cable is assumed to lie in the vertical plane containing both endpoints.
    """
    boat_attach = np.array(boat_attach, dtype=float)
    rov = np.array(rov, dtype=float)
    
    # Horizontal (plan–view) positions and vertical positions:
    B_h = boat_attach[:2]   # boat attachment (x,y)
    R_h = rov[:2]           # ROV horizontal position
    z_B = boat_attach[2]
    z_R = rov[2]
    
    # Horizontal distance D and unit vector U (from boat_attach toward rov)
    delta = R_h - B_h
    D = np.linalg.norm(delta)
    if D == 0:
        # If endpoints are vertically aligned, simply use a vertical line.
        x_vals = np.full(num_points, B_h[0])
        y_vals = np.full(num_points, B_h[1])
        z_vals = np.linspace(z_B, z_R, num_points)
        return np.column_stack((x_vals, y_vals, z_vals))
    U = delta / D
    
    # Vertical difference (Δz)
    dz = z_R - z_B  # typically negative if the ROV is below the boat.
    
    # Solve for catenary parameters a and c.
    a, c = solve_catenary_params(D, dz, L)
    
    # Create a set of horizontal positions along the local x–axis (from 0 to D).
    x_local = np.linspace(0, D, num_points)
    # Compute the vertical profile in the local coordinate:
    z_local = a * np.cosh((x_local - c) / a) - a * np.cosh(c / a)
    
    # Assemble the 3D coordinates: horizontal positions follow boat_attach + (x_local * U)
    horiz_coords = boat_attach[:2] + np.outer(x_local, U)
    z_coords = z_B + z_local
    coords = np.column_stack((horiz_coords, z_coords))
    return coords

# ================================
# Example usage:
# ================================
if __name__ == "__main__":
    # Example scenario:
    # The cable attaches to the boat at a point (e.g., 0.5 m forward of center, at the waterline),
    # and the ROV is directly under the boat (in plan view) but at a depth of 5 m.
    boat_attach = [0.5, 0.0, 0.0]   # (x, y, z) in meters; assume waterline is z = 0.
    rov = [3.0, 3.0, -5.0]          # ROV position: directly under the boat’s center.
    
    # Total cable length (must be greater than the straight-line distance).
    L = 15.0  # meters
    
    try:
        cable_coords = compute_catenary_curve(boat_attach, rov, L, num_points=1000)
    except RuntimeError as e:
        print("Error:", e)
        exit(1)
    
    # Print endpoint information:
    print("Boat attachment point:", boat_attach)
    print("ROV position (as specified):", rov)
    print("Computed cable end point:", cable_coords[-1])
    
    # Plotting the cable in 3D:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(cable_coords[:,0], cable_coords[:,1], cable_coords[:,2],
            'b-', lw=2, label="Cable")
    ax.scatter(boat_attach[0], boat_attach[1], boat_attach[2],
               c='blue', marker='x', s=100, label="Boat Attach")
    ax.scatter(rov[0], rov[1], rov[2],
               c='red', marker='o', s=100, label="ROV")
    
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()
    plt.title("Catenary Cable Curve (with Slack)")
    plt.show()
